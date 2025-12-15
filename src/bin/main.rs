#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::ffi::c_float;

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_storage::Storage;
use esp_bootloader_esp_idf::partitions::PartitionTable;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::FLASH;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use log::info;
use trouble_host::prelude::*;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

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

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let _stack = trouble_host::new(ble_controller, &mut resources);

    // TODO: Spawn some tasks
    let _ = spawner;

    let ota = Ota::new(&spawner, peripherals.FLASH);
    ota.show_partitions_info();

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

struct Ota<'a> {
    spawner: &'a Spawner,
    flash_storage: FlashStorage<'a>,
    buffer: [u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN],
}
impl<'a> Ota<'a> {
    fn new(spawner: &'a Spawner, flash: FLASH<'a>) -> Self {
        Self {
            spawner,
            flash_storage: FlashStorage::new(flash),
            buffer: [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN],
        }
    }
    fn show_partitions_info(mut self) {
        if let Ok(pt) = esp_bootloader_esp_idf::partitions::read_partition_table(
            &mut self.flash_storage,
            &mut self.buffer,
        ) {
            for part in pt.iter() {
                let size = part.len() ;
                info!("{:?}", part)
                //info!("Partition: {}  | offset:{} | size: {}", part.label_as_str(), part.offset(), size);
            }
            if let Ok(current_partition) = pt.booted_partition() {
                if let Some(current_partition) = current_partition {
                    info!(
                        "Partition booted: {:?}",
                        current_partition.label_as_str()
                    );
                }
            }
        }
    }

    fn say_duck(self) {
        self.spawner.spawn(print_duck()).ok();
    }
}
#[embassy_executor::task]
async fn print_duck() {
    loop {
        info!("Duck duck duck!");
        Timer::after(Duration::from_secs(1)).await;
    }
}
