#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
use core::ffi::c_float;
use core::str::FromStr;

use alloc::string::{String, ToString};
use bt_hci::controller::ExternalController;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use embedded_storage::Storage;
use embedded_storage::nor_flash::NorFlash;
use embedded_storage_async::nor_flash::{NorFlash as AsyncNorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::ota_updater::{self, OtaUpdater};
use esp_bootloader_esp_idf::partitions::{AppPartitionSubType, FlashRegion, PartitionTable};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::FLASH;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::{
    ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState
};
use esp_storage::FlashStorage;
use embassy_net::Runner;
use heapless::format;
use log::info;
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

struct FirmwareInfo {
    firm_type: u16,
    name: String,
    path: String,
}
static CH_LOAD_FIRMWARE: Channel<CriticalSectionRawMutex, FirmwareInfo, 1> = Channel::new();

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

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

    //Wifi connections
    let config = embassy_net::Config::dhcpv4(Default::default());
    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;


    spawner
        .spawn(initialize_wifi_connection(
            wifi_controller,
            "Diego",
            "Diego777",
        ))
        .ok();
    Timer::after(Duration::from_secs(20)).await;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(
            embassy_net::StackResources<3>,
            embassy_net::StackResources::<3>::new()
        ),
        seed,
    );

    spawner.spawn(net_task(runner)).ok();

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Version: {}", 11);
    info!("Waiting to get IP address...");
    Timer::after(Duration::from_millis(50)).await;

    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    Timer::after(Duration::from_secs(2)).await;



    let flash = FlashStorage::new(peripherals.FLASH);

    spawner.spawn(firmware_loader(flash, stack.clone())).ok();

    loop {
        info!("Hello world!");
        let firmware_info = FirmwareInfo {
            firm_type: 1,
            name: "duck-os.bin".into(),
            path: "http://127.0.0.1".into(),
        };
        CH_LOAD_FIRMWARE.send(firmware_info).await;
        Timer::after(Duration::from_secs(1)).await;
    }
}

struct OtaHttpUpdater<'a, F>
where
    F: embedded_storage::Storage,
{
    ota: Ota<'a, F>,
    socket: TcpSocket<'a>,
    host: String,
    path: String
}
impl<'a, F> OtaHttpUpdater<'a, F>
where
    F: embedded_storage::Storage,
{
    fn new(ota: Ota<'a, F>, socket: TcpSocket<'a>, host: &str, path: &str) -> Self {
        Self { ota, socket, host: host.into(), path: path.into() }
    }
    async fn update_firmware(self){
        self.ota.write_firmware(&self.socket).await;
    }
    async fn http_get(
        mut self,
    ) -> DuckResult<Self> {
        let request_str = format!(
            "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
            self.path, self.host
        );
        self.socket.set_keep_alive(Some(Duration::from_secs(15)));
        let remote_endpoint = (core::net::Ipv4Addr::new(192, 168, 100, 56), 80);
        info!("connecting...");
        if let Err(e) = self.socket.connect(remote_endpoint).await {
            info!("connect error: {:?}", e);
        }
        let request: heapless::String<1024> = if let Ok(request_str) = request_str {
            request_str
        } else {
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
            Ok(ota_updater)=> ota_updater,
            Err(e) => {
                info!("Fail to get updater: {}", e.to_string());
                return Err(DuckError::NetworkError);
            }
        };
        Ok(Self {
            updater,
        })
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
async fn firmware_loader(mut flash: FlashStorage<'static>, stack: Stack<'static>) {
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
        info!("Type: {}", firmware_info.firm_type);
        info!("Name: {}", firmware_info.name);
        info!("Path: {}", firmware_info.path);
        let socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        info!("paso1");
        let ota = Ota::new(&mut flash, &mut buffer);
        info!("paso2");
        let ota_http_updater = OtaHttpUpdater::new(ota.expect("Error"), socket, "http://192.168.100.56:80", "firmware.bin");
        info!("paso3");
        ota_http_updater.http_get().await.expect("Error al conectar").update_firmware().await;
        info!("paso4");

        firmware_loader_running = false;
        Timer::after(Duration::from_secs(10)).await;
    }
}
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
#[embassy_executor::task]
async fn initialize_wifi_connection(
    mut controller: WifiController<'static>,
    wifi_name: &'static str,
    password: &'static str,
) {
    info!("start connection task");
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
            Ok(_) => info!("Wifi: connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}
