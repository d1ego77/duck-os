#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::ffi::c_float;

use alloc::string::String;
use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_storage::Storage;
use esp_bootloader_esp_idf::partitions::PartitionTable;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::FLASH;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::{ClientConfig, ModeConfig, ScanConfig, WifiController, WifiEvent, WifiStaState};
use esp_storage::FlashStorage;
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

    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let _stack = trouble_host::new(ble_controller, &mut resources);

    //Wifi connections
    let config = embassy_net::Config::dhcpv4(Default::default());
    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let wifi_interface = interfaces.sta;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(
            embassy_net::StackResources<3>,
            embassy_net::StackResources::<3>::new()
        ),
        seed,
    );

    spawner.spawn(initialize_wifi_connection(wifi_controller,"Diego", "Diego777")).ok();
    spawner.spawn(firmware_loader(peripherals.FLASH, stack.clone())).ok();


    loop {
        info!("Hello world!");
        let firmware_info = FirmwareInfo { firm_type: 1, name: "duck-os.bin".into(), path: "http://127.0.0.1".into()};
        CH_LOAD_FIRMWARE.send(firmware_info).await;
        Timer::after(Duration::from_secs(1)).await;
    }
}

struct Ota<'a>{
    flash_storage: &'a FlashStorage<'a>,
    buffer: [u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN],
}
impl <'a> Ota <'a> {
    fn new(storage:&'a FlashStorage<'a>) -> Self {
        Self {
            flash_storage: storage,
            buffer: [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN],
        }
    }
    fn show_partitions_info(&mut self) {
        if let Ok(pt) = esp_bootloader_esp_idf::partitions::read_partition_table(
            &mut self.flash_storage,
            &mut self.buffer,
        ) {
            for part in pt.iter() {
                let size = part.len();
                //info!("{:?}", part)
                info!(
                    "Partition: {}  | offset:{} | size: {}",
                    part.label_as_str(),
                    part.offset(),
                    size
                );
            }
            if let Ok(current_partition) = pt.booted_partition() {
                if let Some(current_partition) = current_partition {
                    info!("Partition booted: {:?}", current_partition.label_as_str());
                }
            }
            if let Ok(mut ota_updater) = esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(
                 storage,
                 &mut self.buffer,
             ) {
                 info!("Current partition stated: {:?}", ota_updater.current_ota_state())
             }
        }
    }
}

#[embassy_executor::task]
async fn firmware_loader(flash: FLASH<'static>, stack: Stack<'static>) {
    let mut firmware_loader_running = false;
    let mut flash_storage = FlashStorage::new(flash);
    loop {
         let mut ota = Ota::new(&mut flash_storage);
         let firmware_info = CH_LOAD_FIRMWARE.receive().await;
         if firmware_loader_running == true {
             continue;
         }
        firmware_loader_running = true;
        ota.show_partitions_info();
        info!("Type: {}", firmware_info.firm_type);
        info!("Name: {}", firmware_info.name);
        info!("Path: {}", firmware_info.path);
        Timer::after(Duration::from_secs(10)).await;
        firmware_loader_running = false;
    }
}

#[embassy_executor::task]
async fn initialize_wifi_connection(mut controller: WifiController<'static>, wifi_name: &'static str, password:&'static str) {
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
