use core::net::Ipv4Addr;

use alloc::string::{String, ToString};
use defmt::info;
use embassy_net::{Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use esp_storage::FlashStorage;
use heapless::format;

use crate::{
    CURRENT_VERSION,
    helpers::{
        DuckError, DuckResult, RgbColor, extract_version, is_newer, set_rgb_led_color,
        set_rgb_led_online,
    },
    ota::{Ota, OtaHttpUpdater},
};

pub struct DuckFirmware<'a> {
    boot_button: esp_hal::gpio::Input<'a>,
    flash: FlashStorage<'a>,
    stack: Stack<'a>,
    host: String,
    path: String,
    version_path: String,
}

impl<'a> DuckFirmware<'a> {
    pub fn new(
        boot_button: esp_hal::gpio::Input<'a>,
        flash: FlashStorage<'a>,
        stack: Stack<'a>,
        host: &str,
        path: &str,
        version_path: &str,
    ) -> Self {
        Self {
            boot_button,
            flash,
            stack,
            host: host.into(),
            version_path: version_path.into(),
            path: path.into(),
        }
    }
    pub async fn update_firmware(&mut self) {
        loop {
            if !self.boot_button.is_low() {
                info!("cheking {}", self.boot_button.is_low());
                Timer::after(Duration::from_secs(5)).await;
                continue;
            } else {
                set_rgb_led_color(RgbColor::Red).await;
                Timer::after(Duration::from_secs(1)).await;
                match self.get_server_framework_version().await {
                    Ok(version) => {
                        info!(
                            "New version: {} - Current Version:{}",
                            version, CURRENT_VERSION
                        );
                        if is_newer(version.as_str(), CURRENT_VERSION) {
                            self.write_new_firmware().await;
                        } else {
                            set_rgb_led_online().await;
                        }
                    }
                    Err(_) => {
                        info!("Fail to get server version");
                    }
                }
            }

            Timer::after(Duration::from_secs(2)).await;
        }
    }

    #[allow(unused_assignments)]
    pub async fn write_new_firmware(&mut self) {
        let mut buffer = [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN];
        let mut rx_buffer = [0; 4096];
        let mut tx_buffer = [0; 4096];

        set_rgb_led_color(RgbColor::Pink).await;
        Timer::after(Duration::from_secs(2)).await;
        info!("Name: {}", self.host);
        info!("Path: {}", self.path);
        let socket = TcpSocket::new(self.stack, &mut rx_buffer, &mut tx_buffer);
        let ota = Ota::new(&mut self.flash, &mut buffer);
        match ota {
            Ok(ota) => {
                let ota_http_updater = OtaHttpUpdater::new(ota, socket, &self.host, &self.path);
                match ota_http_updater.check_firmware_server_connection().await {
                    Ok(firmware_server) => firmware_server.update_firmware().await,
                    Err(_) => {
                        info!("Fail to get the new firmware");
                    }
                };
            }
            Err(_) => {
                info!("Fail to build OTA");
            }
        }
        set_rgb_led_online().await;
        Timer::after(Duration::from_secs(2)).await;
    }
    async fn get_server_framework_version(&mut self) -> DuckResult<String> {
        let mut rx_buffer = [0; 4096];
        let mut tx_buffer = [0; 4096];

        let request_str = format!(
            "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
            self.version_path, self.host
        );
        let mut socket = TcpSocket::new(self.stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_keep_alive(Some(Duration::from_secs(15)));

        info!("connecting...");
        let remote = self.get_remote_endpoint().ok_or(DuckError::NetworkError)?;

        socket
            .connect(remote)
            .await
            .map_err(|_| DuckError::NetworkError)?;

        let request: heapless::String<1024> = request_str.map_err(|_| DuckError::NetworkError)?;

        socket
            .write_all(request.as_bytes())
            .await
            .map_err(|_| DuckError::NetworkError)?;

        let mut buf = [0u8; 1];
        let mut last = [0u8; 4];

        loop {
            socket
                .read_exact(&mut buf)
                .await
                .map_err(|_| DuckError::NetworkError)?;

            last.rotate_left(1);
            last[3] = buf[0];

            if &last == b"\r\n\r\n" {
                break;
            }
        }

        // Read json body
        let mut body = [0u8; 128];
        let mut len = 0;

        loop {
            match socket.read(&mut body[len..]).await {
                Ok(0) => break, // EOF
                Ok(n) => {
                    len += n;
                    if len >= body.len() {
                        break;
                    }
                }
                Err(_) => return Err(DuckError::NetworkError),
            }
        }

        let version_str =
            core::str::from_utf8(&body[..len]).map_err(|_| DuckError::NetworkError)?;

        let version = match extract_version(version_str) {
            Some(version) => version.to_string(),
            None => "0.0.0".to_string(),
        };
        Ok(version)
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
}
