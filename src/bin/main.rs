#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
use alloc::borrow::ToOwned;
use alloc::string::{String, ToString};
use bt_hci::controller::ExternalController;
use core::net::Ipv4Addr;
use embassy_executor::Spawner;
use embassy_net::Config;
use embassy_net::Runner;
use embassy_net::Stack;
use embassy_net::driver::Driver;
use embassy_net::tcp::TcpSocket;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use embedded_storage::Storage;
use esp_bootloader_esp_idf::ota_updater::OtaUpdater;
use esp_bootloader_esp_idf::partitions::FlashRegion;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::GPIO8;
use esp_hal::peripherals::RMT;
use esp_hal::rmt::PulseCode;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::SmartLedsAdapter;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::{
    ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
};
use esp_storage::FlashStorage;
use heapless::format;
use log::info;
use rgb::Grb;
use smart_leds::SmartLedsWrite;
use trouble_host::prelude::*;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const CURRENT_VERSION: &str = "1.0.5";
const FIRMWARE_FILE_NAME: &str = "duck-firmware.bin";
const VERSION_FILE_NAME: &str = "version.json";
const FIRMWARE_HOST: &str = "http://192.168.100.185:80";
const WIFI_NAME: &str = "Diego";
const WIFI_PASSWORD: &str = "Diego777";

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

static WIFI_READY: Signal<CriticalSectionRawMutex, WifiState> = Signal::new();
static CHANGE_LED_COLOR: Channel<CriticalSectionRawMutex, DuckColor, 4> = Channel::new();

#[allow(dead_code)]
enum WifiState {
    Connected,
    NoConnected,
}

struct NetworkConfiguration<D>
where
    D: Driver,
{
    wifi_interface: D,
    config: Config,
    seed: u64,
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");
    info!("Current Version: {}", get_current_version());

    spawner
        .spawn(rgb_control(peripherals.RMT, peripherals.GPIO8))
        .ok();

    let radio_init = &*mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );

    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = interfaces.sta;

    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let _stack = trouble_host::new(ble_controller, &mut resources);

    // Initialize wifi service|
    spawner
        .spawn(initialize_wifi_connection(
            wifi_controller,
            &WIFI_NAME,
            &WIFI_PASSWORD,
        ))
        .ok();

    let network_config = NetworkConfiguration {
        wifi_interface,
        config: dhcpv4_default_config(),
        seed: generate_seed(),
    };

    // Initialize  network connection|
    let (stack, runner) = build_network(network_config);
    spawner.spawn(net_task(runner)).ok();

    wait_for_network_connection(stack).await;

    wait_for_network_ip(stack).await;

    let flash = FlashStorage::new(peripherals.FLASH);
    let input_config = esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up);
    let boot_button = esp_hal::gpio::Input::new(peripherals.GPIO9, input_config);
    let duck_firmware = DuckFirmware::new(
        boot_button,
        flash,
        stack.clone(),
        FIRMWARE_HOST,
        FIRMWARE_FILE_NAME,
        VERSION_FILE_NAME,
    );
    spawner.spawn(firmware_update_task(duck_firmware)).ok();

    loop {
        info!("running...");
        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn firmware_update_task(mut duck_firmware: DuckFirmware<'static>) {
    duck_firmware.update_firmware().await;

}
struct DuckFirmware<'a> {
    boot_button: esp_hal::gpio::Input<'a>,
    flash: FlashStorage<'a>,
    stack: Stack<'a>,
    host: String,
    path: String,
    version_path: String,
}

impl<'a> DuckFirmware<'a> {
    fn new(
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
    async fn update_firmware(&mut self) {
        loop {
            if !self.boot_button.is_low(){
                info!("cheking {}", self.boot_button.is_low());
                Timer::after(Duration::from_secs(5)).await;
                continue;
            } else {
                match self.get_server_framework_version().await {
                    Ok(version) => {
                        info!("New version: {} - Current Version:{}", version, CURRENT_VERSION);
                        Timer::after(Duration::from_secs(20)).await;
                        if is_newer(version.as_str(), CURRENT_VERSION) {
                            self.write_new_firmware().await;
                        }
                    },
                    Err(_) => {
                        info!("Fail to get server version");
                    }
                }
            }
        }
    }
    #[allow(unused_assignments)]
    async fn write_new_firmware(&mut self) {
        let mut buffer = [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN];
        let mut rx_buffer = [0; 4096];
        let mut tx_buffer = [0; 4096];

        set_rgb_led_color(DuckColor::Pink).await;
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
        set_rgb_led_color(DuckColor::Green).await;
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

        /* -------------------------------------------------
         * 1️⃣ Saltar headers HTTP
         * ------------------------------------------------- */
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

        /* -------------------------------------------------
         * 2️⃣ Leer body (JSON)
         * ------------------------------------------------- */
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

        info!("response body: {}", version_str);

        Ok(version_str.to_string())
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

fn parse_version(v: &str) -> Option<(u8, u8, u8)> {
    let mut it = v.trim().split('.');
    Some((
        it.next()?.parse().ok()?,
        it.next()?.parse().ok()?,
        it.next()?.parse().ok()?,
    ))
}

fn is_newer(remote: &str, local: &str) -> bool {
    match (parse_version(remote), parse_version(local)) {
        (Some(r), Some(l)) => r > l,
        _ => false,
    }
}

fn build_network<'a, D>(config: NetworkConfiguration<D>) -> (Stack<'a>, Runner<'a, D>)
where
    D: Driver,
{
    embassy_net::new(
        config.wifi_interface,
        config.config,
        mk_static!(
            embassy_net::StackResources<3>,
            embassy_net::StackResources::<3>::new()
        ),
        config.seed,
    )
}
async fn wait_for_network_connection<'a>(stack: Stack<'a>) {
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
async fn wait_for_network_ip<'a>(stack: Stack<'a>) {
    set_rgb_led_color(DuckColor::Pink).await;
    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            set_rgb_led_color(DuckColor::Orange).await;
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
fn generate_seed() -> u64 {
    let rng = esp_hal::rng::Rng::new();
    (rng.random() as u64) << 32 | rng.random() as u64
}
fn dhcpv4_default_config() -> Config {
    embassy_net::Config::dhcpv4(Default::default())
}
fn get_current_version() -> &'static str {
    CURRENT_VERSION
}

struct OtaHttpUpdater<'a, F>
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
    fn new(ota: Ota<'a, F>, socket: TcpSocket<'a>, host: &str, path: &str) -> Self {
        Self {
            ota,
            socket,
            host: host.into(),
            path: path.into(),
        }
    }
    async fn update_firmware(mut self) {
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
    async fn check_firmware_server_connection(mut self) -> DuckResult<Self> {
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

#[derive(Debug)]
enum DuckError {
    NetworkError,
}
#[allow(dead_code)]
fn handle_error(error: &DuckError) {
    match error {
        DuckError::NetworkError => defmt::error!("Network error ocurred."),
    }
}

type DuckResult<T> = core::result::Result<T, DuckError>;

//#[allow(dead_code)]
// async fn check_firmware_server_connection<'a>(
//     socket: &'a mut TcpSocket<'a>,
//     host: &'a str,
//     path: &'a str,
// ) -> DuckResult<&'a mut TcpSocket<'a>> {
//     let request_str = format!(
//         "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
//         path, host
//     );

//     let request: heapless::String<1024> = if let Ok(request_str) = request_str {
//         request_str
//     } else {
//         return Err(DuckError::NetworkError);
//     };

//     match socket.write_all(request.as_bytes()).await {
//         Ok(_) => {
//             let mut buf = [0u8; 1];
//             let mut last = [0u8; 4];

//             loop {
//                 match socket.read_exact(&mut buf).await {
//                     Ok(_) => {
//                         // Saltar headers
//                         last.rotate_left(1);
//                         last[3] = buf[0];
//                         if &last == b"\r\n\r\n" {
//                             break;
//                         }
//                     }
//                     Err(_) => {
//                         return Err(DuckError::NetworkError);
//                     }
//                 };
//             }
//         }
//         Err(_) => return Err(DuckError::NetworkError),
//     };

//     Ok(socket)
// }

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    loop {
        match WIFI_READY.wait().await {
            wifi_state => match wifi_state {
                WifiState::Connected => {
                    break;
                }
                WifiState::NoConnected => {
                    Timer::after(Duration::from_secs(2)).await;
                    continue;
                }
            },
        }
    }
    runner.run().await
}

#[embassy_executor::task]
async fn initialize_wifi_connection(
    mut controller: WifiController<'static>,
    wifi_name: &'static str,
    password: &'static str,
) {
    set_rgb_led_color(DuckColor::Red).await;
    info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(wifi_name.into())
                    .with_password(password.into()),
            );
            controller.set_config(&client_config).unwrap();
            info!("Wifi: starting ....");
            controller.start_async().await.unwrap();
            info!("Wifi: started!");

            info!("Wifi: connections list:");
            let scan_config = ScanConfig::default().with_max(10);
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .unwrap();
            for ap in result {
                info!("{:?}", ap);
            }
        }
        info!("Wifi: connecting....");

        match controller.connect_async().await {
            Ok(_) => {
                set_rgb_led_color(DuckColor::Blue).await;
                info!("Wifi: connected!");
                WIFI_READY.signal(WifiState::Connected);
            }
            Err(e) => {
                info!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn rgb_control(rmt: RMT<'static>, gpio8: GPIO8<'static>) {
    let mut rmt_buffer = esp_hal_smartled::smart_led_buffer!(1);
    let mut led = RgbLedComponent::new(rmt, gpio8, &mut rmt_buffer);

    loop {
        let val = CHANGE_LED_COLOR.receive().await;
        led.set_color(val, 10);
    }
}

async fn set_rgb_led_color(color: DuckColor) {
    CHANGE_LED_COLOR.send(color).await
}

#[allow(dead_code)]
enum DuckColor {
    Red,
    Green,
    Blue,
    Yellow,
    Pink,
    Orange,
}
struct RgbLedComponent<'ch, Color = Grb<u8>> {
    rgb_led: SmartLedsAdapter<'ch, 25, Color>,
}
impl<'ch> RgbLedComponent<'ch, Grb<u8>> {
    fn new(
        peripheral: RMT<'ch>,
        gpio8: GPIO8<'ch>,
        rmt_buffer: &'ch mut [PulseCode; 25],
    ) -> RgbLedComponent<'ch> {
        let rmt: esp_hal::rmt::Rmt<'_, esp_hal::Blocking> = {
            let frequency: esp_hal::time::Rate = esp_hal::time::Rate::from_mhz(80);
            esp_hal::rmt::Rmt::new(peripheral, frequency)
        }
        .unwrap();
        Self {
            rgb_led: SmartLedsAdapter::new(rmt.channel0, gpio8, rmt_buffer),
        }
    }
    fn set_color(&mut self, color: DuckColor, level: u8) {
        let color = match color {
            DuckColor::Red => smart_leds::hsv::Hsv {
                hue: 0,
                sat: 255,
                val: 255,
            },
            DuckColor::Green => smart_leds::hsv::Hsv {
                hue: 87,
                sat: 255,
                val: 255,
            },
            DuckColor::Blue => smart_leds::hsv::Hsv {
                hue: 191,
                sat: 255,
                val: 255,
            },
            DuckColor::Yellow => smart_leds::hsv::Hsv {
                hue: 45,
                sat: 255,
                val: 255,
            },
            DuckColor::Pink => smart_leds::hsv::Hsv {
                hue: 213,
                sat: 255,
                val: 255,
            },
            DuckColor::Orange => smart_leds::hsv::Hsv {
                hue: 30,
                sat: 255,
                val: 255,
            },
        };
        let data: rgb::RGB8 = smart_leds::hsv::hsv2rgb(color);
        self.rgb_led
            .write(smart_leds::brightness(
                smart_leds::gamma([data].into_iter()),
                level,
            ))
            .expect("Error al setear el color")
    }
}
