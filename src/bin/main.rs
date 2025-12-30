#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
mod channel;
mod firmware;
mod helpers;
mod ota;
mod rgb_led;
mod wifi;

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::GPIO8;
use esp_hal::peripherals::RMT;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::WifiDevice;
use esp_storage::FlashStorage;
use log::info;
use trouble_host::prelude::*;

use crate::channel::CHANGE_LED_COLOR;
use crate::firmware::DuckFirmware;
use crate::rgb_led::RgbLedComponent;
use crate::rgb_led::breath;
use crate::rgb_led::set_rgb_led_offline;
use crate::wifi::NetworkConnection;
use crate::wifi::Wifi;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

const CURRENT_VERSION: &str = "1.0.44";
const FIRMWARE_FILE_NAME: &str = "duck-firmware.bin";
const VERSION_FILE_NAME: &str = "version.json";
const FIRMWARE_HOST: &str = "http://192.168.100.185:80";
const WIFI_NAME: &str = "Diego";
const WIFI_PASSWORD: &str = "Diego777";

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
    info!("Current Version: {}", get_current_version());

    let radio_init = &*wifi::mk_static!(
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

    spawner
        .spawn(rgb_control(peripherals.RMT, peripherals.GPIO8))
        .ok();

    // Build wifi, network and network stack
    let (wifi, network, network_stack) =
        wifi::DuckNet::new(wifi_interface, wifi_controller, &WIFI_NAME, &WIFI_PASSWORD);
    // Connect ESP32 to the current wifi
    spawner.spawn(wifi_connection_task(wifi)).ok();
    // Create a connection network
    spawner.spawn(network_connection_task(network)).ok();
    // Before to continue wait for the network link up
    network_stack.wait_for_network_link_up().await;
    // Before to continue wait for a network IP
    network_stack.wait_for_network_ip().await;

    let flash = FlashStorage::new(peripherals.FLASH);
    let input_config = esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up);
    let boot_button = esp_hal::gpio::Input::new(peripherals.GPIO9, input_config);
    let duck_firmware = firmware::DuckFirmware::new(
        boot_button,
        flash,
        network_stack.get_stack(),
        FIRMWARE_HOST,
        FIRMWARE_FILE_NAME,
        VERSION_FILE_NAME,
    );
    spawner.spawn(firmware_update_task(duck_firmware)).ok();
    spawner.spawn(breath_task()).ok();
    loop {
        info!("running...");
        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn breath_task() {
    breath().await;
}
#[embassy_executor::task]
async fn firmware_update_task(mut duck_firmware: DuckFirmware<'static>) {
    duck_firmware.update_firmware().await;
}

#[embassy_executor::task]
async fn network_connection_task(connection: NetworkConnection<'static, WifiDevice<'static>>) {
    connection.connect_task().await
}

#[embassy_executor::task]
async fn wifi_connection_task(mut wifi: Wifi) {
    set_rgb_led_offline().await;
    wifi.wifi_connect_task().await;
}

#[embassy_executor::task]
async fn rgb_control(rmt: RMT<'static>, gpio8: GPIO8<'static>) {
    let mut rmt_buffer = esp_hal_smartled::smart_led_buffer!(1);
    let mut led = RgbLedComponent::new(rmt, gpio8, &mut rmt_buffer);

    loop {
        let val = CHANGE_LED_COLOR.receive().await;
        let color = match val {
            helpers::RgbLedCommand::SetColor(rgb_color) => rgb_color.hsv(),
            helpers::RgbLedCommand::SetCustom(hsv) => hsv,
        };
        led.set_color(color, 10);
    }
}

fn get_current_version() -> &'static str {
    CURRENT_VERSION
}
