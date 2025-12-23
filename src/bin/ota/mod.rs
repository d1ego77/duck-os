use alloc::string::ToString;
use defmt::info;
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use esp_bootloader_esp_idf::{ota_updater::OtaUpdater, partitions::FlashRegion};

use crate::helpers::{DuckError, DuckResult};
use embedded_storage::Storage;

struct Ota<'a, F>
where
    F: embedded_storage::Storage,
{
    updater: OtaUpdater<'a, F>,
}

impl<'a, F> Ota<'a, F>
where
    F: embedded_storage::Storage,
{
    fn new(
        storage: &'a mut F,
        buffer: &'a mut [u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN],
    ) -> DuckResult<Self> {
        let updater = match OtaUpdater::new(storage, buffer) {
            Ok(ota_updater) => ota_updater,
            Err(e) => {
                info!("Fail to get updater: {}", e.to_string());
                return Err(DuckError::NetworkError);
            }
        };
        Ok(Self { updater })
    }
    async fn load_firmware_in_next_partition<'p, R>(
        stream: &mut R,
        mut partition: FlashRegion<'p, F>,
    ) where
        R: Read,
    {
        let mut buf = [0; 4096];
        let mut offset = 0u32;
        loop {
            let n = match stream.read(&mut buf).await {
                Ok(0) => {
                    info!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    info!("read error: {:?}", e);
                    break;
                }
            };

            let data = &buf[..n];

            // ✅ Saltar headers HTTP
            let payload = data;

            info!("Received {} bytes", payload.len());

            info!("Writing offset {} len {}", offset, payload.len());

            match partition.write(offset, payload) {
                Ok(_) => info!("Chunk written"),
                Err(e) => {
                    info!("❌ Error writing flash at offset {}: {:?}", offset, e);
                    break;
                }
            }

            offset += payload.len() as u32;

            // ✅ Dejar respirar la pila TCP (OBLIGATORIO)
            embassy_time::Timer::after_millis(1).await;
        }
        info!("Descarga finalizada: {} bytes", offset);
    }
    async fn write_firmware<R>(mut self, firmware_stream: &mut R)
    where
        R: Read,
    {
        if let Ok(state) = self.updater.current_ota_state() {
            if state == esp_bootloader_esp_idf::ota::OtaImageState::New
                || state == esp_bootloader_esp_idf::ota::OtaImageState::PendingVerify
            {
                info!("Changed state to valid");
                self.updater
                    .set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::Valid)
                    .unwrap();
            }
        }

        let (next_app_partition, _) = self.updater.next_partition().unwrap();

        Ota::load_firmware_in_next_partition(firmware_stream, next_app_partition).await;

        info!("Activating partition...");
        match self.updater.activate_next_partition() {
            Ok(_) => {
                info!("Next Partition activated");
            }
            Err(_) => {
                info!("ERROR activating partition");
            }
        }

        self.updater
            .set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::New)
            .unwrap();

        set_rgb_led_color(DuckColor::Blue).await;
        info!("Reiniciando...");
        Timer::after(Duration::from_secs(5)).await;

        esp_hal::rom::software_reset();
    }
}
