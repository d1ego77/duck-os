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
use esp_hal::Blocking;
use esp_hal::analog::adc::Adc;
use esp_hal::analog::adc::AdcPin;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::ADC1;
use esp_hal::peripherals::GPIO6;
use esp_hal::peripherals::GPIO8;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::WifiDevice;
use esp_storage::FlashStorage;
use log::info;
use trouble_host::prelude::*;

use crate::channel::CHANGE_LED_COLOR;
use crate::firmware::DuckFirmware;
use crate::rgb_led::RgbLed;
use crate::rgb_led::breath;
use crate::rgb_led::set_rgb_led_light;
use crate::rgb_led::set_rgb_led_offline;
use crate::rgb_led::set_rgb_led_online;
use crate::wifi::NetworkConnection;
use crate::wifi::Wifi;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

const CURRENT_VERSION: &str = "1.0.60";
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

    let led = rgb_led::RgbLed::new(peripherals.RMT, peripherals.GPIO8);
    spawner.spawn(rgb_manager_task(led)).ok();

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
    let light_sensor: LightSensor = LightSensor::new(peripherals.GPIO6, peripherals.ADC1);
    spawner.spawn(light_sensor_manager_task(light_sensor)).ok();
    spawner.spawn(breath_task()).ok();
    loop {
        info!("running...");
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn light_sensor_manager_task(mut light_sensor: LightSensor<'static>) {
    loop {
        let intensity = light_sensor.current_light_intensity_value();
        let adc = adc_to_255_step5(intensity);
        info!("intensity{}", adc);
        set_rgb_led_light(adc).await;
        info!("Light Level: {}", intensity);
        Timer::after(Duration::from_secs(1)).await;
    }
}

fn adc_to_255_step10(adc: u16) -> u8 {
    const IN_MIN: u16 = 2146;
    const IN_MAX: u16 = 4095;

    let adc = adc.clamp(IN_MIN, IN_MAX);

    let mut value = ((IN_MAX - adc) as u32 * 255 / (IN_MAX - IN_MIN) as u32) as u8;

    value = ((value + 5) / 10) * 10;

    if value > 250 { 255 } else { value }
}
fn adc_to_255(adc: u16) -> u8 {
    const IN_MIN: u16 = 2146;
    const IN_MAX: u16 = 4095;

    let adc = adc.clamp(IN_MIN, IN_MAX);

    let scaled = (IN_MAX - adc) as u32 * 255 / (IN_MAX - IN_MIN) as u32;

    scaled as u8
}

fn adc_to_255_step5(adc: u16) -> u8 {
    const IN_MIN: u16 = 2146; // luz
    const IN_MAX: u16 = 4095; // oscuridad
    const STEP: u8 = 5;

    // Clamp
    let adc = adc.clamp(IN_MIN, IN_MAX);

    // Mapeo invertido correcto
    let mut value = ((adc - IN_MIN) as u32 * 255 / (IN_MAX - IN_MIN) as u32) as u8;

    // Cuantización a pasos de 5 (redondeo)
    value = ((value + STEP / 2) / STEP) * STEP;

    // Preservar el máximo real
    if value > 255 - STEP { 255 } else { value }
}

#[embassy_executor::task]
async fn breath_task() {
    breath().await;
    set_rgb_led_online().await;
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
async fn rgb_manager_task(led: RgbLed<'static, GPIO8<'static>>) {
    let mut buffer = esp_hal_smartled::smart_led_buffer!(1);
    let mut adapter = led.with_buffer(&mut buffer);
    loop {
        let val = CHANGE_LED_COLOR.receive().await;
        let (color, level) = match val {
            helpers::RgbLedCommand::SetColor(rgb_color) => (rgb_color.hsv(), 10),
            helpers::RgbLedCommand::SetCustomColor(hsv) => hsv,
        };
        adapter.set_color(color, level);
    }
}

fn get_current_version() -> &'static str {
    CURRENT_VERSION
}

struct LightSensor<'a> {
    pin: AdcPin<GPIO6<'a>, ADC1<'a>>,
    adc1: Adc<'a, ADC1<'a>, Blocking>,
}
impl<'a> LightSensor<'a> {
    fn new(gpio6: GPIO6<'a>, adc1: ADC1<'a>) -> Self {
        let mut adc1_config = esp_hal::analog::adc::AdcConfig::new();
        let pin = adc1_config.enable_pin(gpio6, esp_hal::analog::adc::Attenuation::_11dB);
        LightSensor {
            adc1: esp_hal::analog::adc::Adc::new(adc1, adc1_config),
            pin: pin,
        }
    }
    fn current_light_intensity_value(&mut self) -> u16 {
        match nb::block!(self.adc1.read_oneshot(&mut self.pin)) {
            Ok(val) => val,
            Err(_) => 0,
        }
    }
}
