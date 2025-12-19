#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
use core::ffi::c_float;
use core::net::Ipv4Addr;
use core::str::FromStr;

use alloc::borrow::ToOwned;
use alloc::string::{String, ToString};
use bt_hci::controller::ExternalController;
use embassy_embedded_hal::adapter::BlockingAsync;
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
use embedded_storage::nor_flash::NorFlash;
use embedded_storage_async::nor_flash::{NorFlash as AsyncNorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::ota_updater::{self, OtaUpdater};
use esp_bootloader_esp_idf::partitions::{AppPartitionSubType, FlashRegion, PartitionTable};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::FLASH;
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
use trouble_host::prelude::*;
use smart_leds::SmartLedsWrite;

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

const CURRENT_VERSION: &str = "1.0.0.0";
const FIRMWARE_FILE_NAME: &str = "duck-os-new.bin";
const FIRMWARE_HOST: &str = "http://192.168.100.185:80";

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;
static CH_LOAD_FIRMWARE: Channel<CriticalSectionRawMutex, FirmwareInfo, 1> = Channel::new();
static WIFI_READY: Signal<CriticalSectionRawMutex, WifiState> = Signal::new();
static CHANNEL_LED_COLOR: Channel<CriticalSectionRawMutex, u32, 4> = Channel::new();

struct FirmwareInfo {
    firmware_type: u16,
    host: String,
    path: String,
}
enum WifiState {
    connected,
    no_connected,
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
            "Diego",
            "Diego777",
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

    spawner
        .spawn(firmware_loader_service(flash, stack.clone()))
        .ok();

    loop {
        info!("running...");
        let firmware_info = FirmwareInfo {
            firmware_type: 1,
            host: FIRMWARE_HOST.into(),
            path: FIRMWARE_FILE_NAME.into(),
        };
        CH_LOAD_FIRMWARE.send(firmware_info).await;
        Timer::after(Duration::from_secs(1)).await;
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
    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
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
    async fn update_firmware(self) {
        self.ota.write_firmware(&self.socket).await;
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
    async fn http_get(mut self) -> DuckResult<Self> {
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
    async fn write_firmware<R>(mut self, stream: &R)
    where
        R: Read,
    {
        let np = self.updater.next_partition().unwrap();
        // loop{
        info!("test");

        // }
    }
}

#[derive(Debug)]
enum DuckError {
    NetworkError,
}
fn handle_error(error: &DuckError) {
    match error {
        DuckError::NetworkError => defmt::error!("Network error ocurred."),
    }
}

pub type DuckResult<T> = core::result::Result<T, DuckError>;
async fn http_get<'a>(
    socket: &'a mut TcpSocket<'a>,
    host: &'a str,
    path: &'a str,
) -> DuckResult<&'a mut TcpSocket<'a>> {
    let request_str = format!(
        "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
        path, host
    );

    let request: heapless::String<1024> = if let Ok(request_str) = request_str {
        request_str
    } else {
        return Err(DuckError::NetworkError);
    };

    match socket.write_all(request.as_bytes()).await {
        Ok(_) => {
            let mut buf = [0u8; 1];
            let mut last = [0u8; 4];

            loop {
                match socket.read_exact(&mut buf).await {
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

    Ok(socket)
}

#[embassy_executor::task]
async fn firmware_loader_service(mut flash: FlashStorage<'static>, stack: Stack<'static>) {
    let mut firmware_loader_running = false;
    let mut buffer = [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN];

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        // let mut ota = Ota::new(&mut flash_storage);
        let firmware_info = CH_LOAD_FIRMWARE.receive().await;
        if firmware_loader_running == true {
            continue;
        }
        firmware_loader_running = true;
        // ota.show_partitions_info();
        info!("Type: {}", firmware_info.firmware_type);
        info!("Name: {}", firmware_info.host);
        info!("Path: {}", firmware_info.path);
        let socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        info!("paso1");
        let ota = Ota::new(&mut flash, &mut buffer);
        info!("paso2");
        let ota_http_updater = OtaHttpUpdater::new(
            ota.expect("Error"),
            socket,
            &firmware_info.host,
            &firmware_info.path,
        );
        info!("paso3");
        match ota_http_updater.http_get().await {
            Ok(http_get) => http_get.update_firmware().await,
            Err(_) => {
                info!("Fail to get the new firmware");
            }
        };
        info!("paso4");

        firmware_loader_running = false;
        Timer::after(Duration::from_secs(10)).await;
    }
}
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    loop {
        match WIFI_READY.wait().await {
            wifi_state => match wifi_state {
                WifiState::connected => {
                    break;
                }
                WifiState::no_connected => {
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
    CHANNEL_LED_COLOR.send(1).await;
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
                CHANNEL_LED_COLOR.send(2).await;
                info!("Wifi: connected!");
                WIFI_READY.signal(WifiState::connected);
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
    //
    // RgbLed component
    //
    //--------------------------
    let mut rmt_buffer = esp_hal_smartled::smart_led_buffer!(1);
    let mut led = RGBLedComponent::new(rmt, gpio8, &mut rmt_buffer);
    let mut led_sensor_manager: SensorManager = SensorManager::new(&mut led, "Led");

    loop {
        let val = CHANNEL_LED_COLOR.receive().await;

        match val {
            1 => led_sensor_manager.set_value("hue=48&sat=255&val=255"),
            2 => led_sensor_manager.set_value("hue=170&sat=255&val=255"),
            3 => led_sensor_manager.set_value("hue=127&sat=255&val=255"),
            4 => led_sensor_manager.set_value("hue=191&sat=255&val=255"),
            _ => led_sensor_manager.set_value("hue=0&sat=255&val=255"),
        }
        info!("Recibido: {}", val);
    }
}
struct SensorManager<'a> {
    id: String,
    sensor: &'a mut dyn Sensor,
}
impl<'a> SensorManager<'a> {
    fn new(sensor: &'a mut dyn Sensor, id: &str) -> Self {
        Self {
            id: id.to_owned(),
            sensor,
        }
    }
    fn get_response(&mut self) -> heapless::String<1024> {
        let ligth_response: heapless::String<1024> = format!(
            "<html><head></head><body><p> {}: {} </p></body></html>",
            self.id,
            self.sensor.get_info()
        )
        .unwrap();
        http_response(200, "text/html; charset=utf-8", ligth_response.as_str())
    }
    fn set_value(&mut self, value: &str) {
        self.sensor.set_value(value.to_owned());
    }
}
///
/// RGB Component
///
struct RGBLedComponent<'ch, Color = Grb<u8>> {
    led: SmartLedsAdapter<'ch, 25, Color>,
}

impl<'ch> RGBLedComponent<'ch, Grb<u8>> {
    fn new(rmt: RMT<'ch>, gpio8: GPIO8<'ch>, rmt_buffer: &'ch mut [PulseCode; 25]) -> Self {
        let rmt: esp_hal::rmt::Rmt<'_, esp_hal::Blocking> = {
            let frequency: esp_hal::time::Rate = esp_hal::time::Rate::from_mhz(80);
            esp_hal::rmt::Rmt::new(rmt, frequency)
        }
        .expect("Failed to initialize RMT");

        let rmt_channel = rmt.channel0;

        let led = esp_hal_smartled::SmartLedsAdapter::new(rmt_channel, gpio8, rmt_buffer);
        Self { led }
    }
    fn set_color(&mut self, hue: u8, sat: u8, val: u8) {
        let color = smart_leds::hsv::Hsv { hue, sat, val };
        let data: rgb::RGB8;
        let level = 10;

        //color.hue = 247;
        // // Convert from the HSV color space (where we can easily transition from one
        // // color to the other) to the RGB color space that we can then send to the LED
        data = smart_leds::hsv::hsv2rgb(color);
        // // When sending to the LED, we do a gamma correction first (see smart_leds docs
        // // for details <https://docs.rs/smart-leds/latest/smart_leds/struct.Gamma.html>)
        // // and then limit the brightness level to 10 out of 255 so that the output
        // // is not too bright.

        self.led
            .write(smart_leds::brightness(
                smart_leds::gamma([data].into_iter()),
                level,
            ))
            .unwrap();
    }
    fn off(&mut self) {
        let color = smart_leds::hsv::Hsv {
            hue: 0,
            sat: 0,
            val: 0,
        };
        let data: rgb::RGB8;
        let level = 20;

        data = smart_leds::hsv::hsv2rgb(color);
        self.led
            .write(smart_leds::brightness(
                smart_leds::gamma([data].into_iter()),
                level,
            ))
            .unwrap();
    }
}
trait Sensor {
    fn get_info(&mut self) -> String;
    fn set_value(&mut self, values: String);
}
fn http_response(status: u16, content_type: &str, body: &str) -> heapless::String<1024> {
    format!(
        "HTTP/1.1 {} OK\r\n{}Content-Type: {}\r\nContent-Length: {}\r\n\r\n{}",
        status,
        cors_headers(),
        content_type,
        body.len(),
        body
    )
    .unwrap()
}
fn cors_headers() -> &'static str {
    // Puedes ajustar los valores si quieres restringir orígenes o métodos
    "Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type\r\n"
}
impl<'ch> Sensor for RGBLedComponent<'ch> {
    fn get_info(&mut self) -> String {
        todo!()
    }

    fn set_value(&mut self, values: String) {
        info!("valor: {}", values);

        let mut parts = values.split("&").into_iter();
        let hue = match parts.next() {
            Some(path) => path
                .split("=")
                .last()
                .unwrap_or("0")
                .parse::<u8>()
                .unwrap_or_default(),
            None => 0,
        };
        let sat = match parts.next() {
            Some(path) => path
                .split("=")
                .last()
                .unwrap_or("0")
                .parse::<u8>()
                .unwrap_or_default(),
            None => 0,
        };
        let val = match parts.next() {
            Some(path) => path
                .split("=")
                .last()
                .unwrap_or("0")
                .parse::<u8>()
                .unwrap_or_default(),
            None => 0,
        };
        info!("Hue{}, sat{}, val{}", hue, sat, val);
        self.set_color(hue, sat, val);
    }
}
