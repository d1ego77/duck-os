use core::net::Ipv4Addr;

use alloc::string::{String, ToString};
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use esp_bootloader_esp_idf::{ota_updater::OtaUpdater, partitions::FlashRegion};
use log::info;

use crate::{
    helpers::{DuckError, DuckResult, RgbColor, RgbLedCommand},
    rgb_led::set_rgb_led_color,
};
use embedded_storage::Storage;

use heapless::format;

pub struct Ota<'a, F>
where
    F: embedded_storage::Storage,
{
    updater: OtaUpdater<'a, F>,
}

impl<'a, F> Ota<'a, F>
where
    F: embedded_storage::Storage,
{
    pub fn new(
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
    pub async fn load_firmware_in_next_partition<'p, R>(
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

            // Saltar headers HTTP
            let payload = data;

            info!("Received {} bytes", payload.len());

            info!("Writing offset {} len {}", offset, payload.len());

            match partition.write(offset, payload) {
                Ok(_) => info!("Chunk written"),
                Err(e) => {
                    info!("X Error writing flash at offset {}: {:?}", offset, e);
                    break;
                }
            }

            offset += payload.len() as u32;

            // Dejar respirar la pila TCP (OBLIGATORIO)
            embassy_time::Timer::after_millis(1).await;
        }
        info!("Descarga finalizada: {} bytes", offset);
    }
    pub async fn write_firmware<R>(mut self, firmware_stream: &mut R)
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

        set_rgb_led_color(RgbLedCommand::SetColor(RgbColor::Blue)).await;
        info!("Restarting...");
        Timer::after(Duration::from_secs(5)).await;

        esp_hal::rom::software_reset();
    }
}
pub struct OtaHttpUpdater<'a, F>
where
    F: embedded_storage::Storage,
{
    ota: Ota<'a, F>,
    socket: TcpSocket<'a>,
    host: String,
    path: String,
}
impl<'a, F> OtaHttpUpdater<'a, F>
where
    F: embedded_storage::Storage,
{
    pub fn new(ota: Ota<'a, F>, socket: TcpSocket<'a>, host: &str, path: &str) -> Self {
        Self {
            ota,
            socket,
            host: host.into(),
            path: path.into(),
        }
    }
    pub async fn update_firmware(mut self) {
        self.ota.write_firmware(&mut self.socket).await;
    }
    fn get_remote_endpoint(&self) -> Option<(Ipv4Addr, u16)> {
        match self.host.strip_prefix("http://") {
            Some(s) => {
                match s.split_once(':') {
                    Some((ip, port)) => {
                        let mut octets = [0u8; 4];
                        let mut i = 0;
                        for part in ip.split('.') {
                            if i >= 4 {
                                return None;
                            }
                            octets[i] = part.parse().ok()?;
                            i += 1;
                        }
                        if i != 4 {
                            return None;
                        }
                        // parsear puerto
                        let port = match port.parse().ok() {
                            Some(port) => port,
                            None => 0,
                        };
                        let [a, b, c, d] = octets;
                        Some((core::net::Ipv4Addr::new(a, b, c, d), port))
                    }
                    None => {
                        return None;
                    }
                }
            }
            None => {
                return None;
            }
        }
    }
    pub async fn check_firmware_server_connection(mut self) -> DuckResult<Self> {
        let request_str = format!(
            "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
            self.path, self.host
        );
        self.socket.set_keep_alive(Some(Duration::from_secs(15)));

        info!("connecting...");
        if let Some(remote_endpoint) = self.get_remote_endpoint() {
            if let Err(e) = self.socket.connect(remote_endpoint).await {
                info!("connect error: {:?}", e);
                return Err(DuckError::NetworkError);
            }
        } else {
            info!("fail to get remote endpoint information error");
            return Err(DuckError::NetworkError);
        }

        let request: heapless::String<1024> = if let Ok(request_str) = request_str {
            request_str
        } else {
            info!("Fail heapless string conversion");
            return Err(DuckError::NetworkError);
        };

        match self.socket.write_all(request.as_bytes()).await {
            Ok(_) => {
                let mut buf = [0u8; 1];
                let mut last = [0u8; 4];

                loop {
                    match self.socket.read_exact(&mut buf).await {
                        Ok(_) => {
                            // Saltar headers
                            last.rotate_left(1);
                            last[3] = buf[0];
                            if &last == b"\r\n\r\n" {
                                break;
                            }
                        }
                        Err(_) => {
                            return Err(DuckError::NetworkError);
                        }
                    };
                }
            }
            Err(_) => return Err(DuckError::NetworkError),
        };

        Ok(self)
    }
}
