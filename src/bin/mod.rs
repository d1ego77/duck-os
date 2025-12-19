use anyhow::{Result, bail};
use embedded_io_async::Read;
use esp_bootloader_esp_idf::partitions::{
    get_running_partition,
    iter_partitions,
    Partition,
    PartitionType,
    AppPartitionSubtype,
};
use esp_hal::flash::Flash;

const BUFFER_SIZE: usize = 1024;

pub struct OtaUpdater<'a> {
    flash: &'a mut Flash,
    target: Partition,
}

impl<'a> OtaUpdater<'a> {
    pub fn new(flash: &'a mut Flash) -> Result<Self> {
        let running = get_running_partition()?;

        let mut target: Option<Partition> = None;

        for p in iter_partitions() {
            if p.partition_type == PartitionType::App {
                match (running.subtype, p.subtype) {
                    (AppPartitionSubtype::Ota0, AppPartitionSubtype::Ota1)
                    | (AppPartitionSubtype::Ota1, AppPartitionSubtype::Ota0) => {
                        target = Some(p);
                        break;
                    }
                    _ => {}
                }
            }
        }

        let target = target.ok_or_else(|| anyhow::anyhow!("OTA target not found"))?;

        Ok(Self { flash, target })
    }

    pub async fn write_firmware<R>(&mut self, mut stream: R) -> Result<()>
    where
        R: Read,
    {
        let mut offset = 0u32;
        let mut buffer = [0u8; BUFFER_SIZE];

        self.flash.erase(self.target.offset, self.target.size)?;

        loop {
            let n = stream.read(&mut buffer).await?;
            if n == 0 {
                break;
            }

            self.flash.write(
                self.target.offset + offset,
                &buffer[..n],
            )?;

            offset += n as u32;

            if offset > self.target.size {
                bail!("Firmware demasiado grande");
            }
        }

        esp_bootloader_esp_idf::ota::Ota::set_current_app_partition(&self.target)?;
        Ok(())
    }
}
