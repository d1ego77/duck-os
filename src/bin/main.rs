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

use alloc::borrow::ToOwned;
use alloc::string::String;
use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Timer};
use esp_hal::Blocking;
use esp_hal::analog::adc::Adc;
use esp_hal::analog::adc::AdcChannel;
use esp_hal::analog::adc::AdcPin;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::AnalogPin;
use esp_hal::gpio::interconnect::PeripheralInput;
use esp_hal::peripherals::ADC1;
use esp_hal::peripherals::GPIO2;
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
use crate::rgb_led::set_rgb_led_offline;
use crate::rgb_led::set_rgb_led_online;
use crate::wifi::NetworkConnection;
use crate::wifi::Wifi;
use heapless::{String as HString, format};

use bme280::i2c::BME280;
use esp_hal::i2c::master::{Config, I2c};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

const CURRENT_VERSION: &str = "1.0.78";
const FIRMWARE_FILE_NAME: &str = "duck-firmware.bin";
const VERSION_FILE_NAME: &str = "version.json";
const FIRMWARE_HOST: &str = "http://192.168.100.185:80";
const WIFI_NAME: &str = "Diego";
const WIFI_PASSWORD: &str = "Diego777";
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;
const WEB_SERVER_PORT: u16 = 8080;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // let mut bme = bme280::new_primary(i2c);

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
    let stack = network_stack.get_stack();
    // Connect ESP32 to the current wifi
    spawner.spawn(wifi_connection_task(wifi)).ok();
    // Create a connection network
    spawner.spawn(network_connection_task(network)).ok();
    // Before to continue wait for the network link up
    network_stack.wait_for_network_link_up().await;
    // Before to continue wait for a network IP
    network_stack.wait_for_network_ip().await;
    let sensor = Sensor::new(peripherals.GPIO6, peripherals.GPIO2, peripherals.ADC1);
    spawner.spawn(web_server(stack, sensor)).ok();

    let flash = FlashStorage::new(peripherals.FLASH);
    let input_config = esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up);
    let boot_button = esp_hal::gpio::Input::new(peripherals.GPIO9, input_config);
    let duck_firmware = firmware::DuckFirmware::new(
        boot_button,
        flash,
        stack,
        FIRMWARE_HOST,
        FIRMWARE_FILE_NAME,
        VERSION_FILE_NAME,
    );
    spawner.spawn(firmware_update_task(duck_firmware)).ok();
    // let sensor = Sensor::new(peripherals.GPIO6, peripherals.GPIO2, peripherals.ADC1);
    // spawner.spawn(sensor_manager_task(sensor)).ok();
    spawner.spawn(breath_task()).ok();
    // loop {
    //     info!("Running...");
    //     Timer::after(Duration::from_secs(10)).await;
    // }
    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO5)
        .with_scl(peripherals.GPIO4);

    let mut bme280 = BME280::new_primary(i2c);

    let mut delay = esp_hal::delay::Delay::new();

    info!("Murio!");
    bme280.init(&mut delay).unwrap();

    loop {
        info!("Entro!");
        let data = bme280.measure(&mut delay).unwrap();
        let t = data.temperature;
        let h = data.humidity;
        let p = data.pressure;
        info!("t: {}, h: {}, p: {}", t, h, p);
        delay.delay_millis(2000u32);
    }
}

//
// Check sensor values periodically.
//
#[embassy_executor::task]
async fn sensor_manager_task(mut sensor_manager: Sensor<'static, GPIO6<'static>, GPIO2<'static>>) {
    loop {
        let light = sensor_manager.current_light();
        let moisture = sensor_manager.current_moisture();
        // let adc = light_to_intensity(light);

        info!("Light: {}%", light);
        info!("Moisture: {}% ", moisture);

        // set_rgb_led_light(adc).await;

        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn web_server(
    stack: embassy_net::Stack<'static>,
    mut sensor_manager: Sensor<'static, GPIO6<'static>, GPIO2<'static>>,
) {
    info!("Entro a webserver");
    loop {
        Timer::after(Duration::from_millis(20)).await;
        let mut rx_buffer = [0; 4096];
        let mut tx_buffer = [0; 4096];

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        info!("Esperando conexiones ");
        socket.accept(WEB_SERVER_PORT).await.unwrap();
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        info!("Conexion recibida ");
        let mut buf = [0; 1024];
        loop {
            let n = match socket.read(&mut buf).await {
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

            // let data = &buf[..n];
            let data = core::str::from_utf8(&buf[..n]).unwrap();

            // Saltar headers HTTP
            let payload = data;

            info!("Received {} bytes", payload);

            let light = sensor_manager.current_light();
            let moisture = sensor_manager.current_moisture();

            let sensor_info: HString<1024> = format!(
                "<html><head></head><body><h1>Light: {}% - Moisture: {}%</h1></body></html>",
                light, moisture
            )
            .unwrap();

            let response = http_response(200, "text/html; charset=utf-8", &sensor_info);
            socket.write(response.as_bytes()).await.ok();
            Timer::after(Duration::from_millis(30)).await;
            socket.close();
            break;
        }
    }
}

fn get_request(buf: &[u8; 1024], request_size: usize) -> String {
    let request = core::str::from_utf8(&buf[..request_size]).unwrap();
    request.to_owned()
}

fn http_response(status: u16, content_type: &str, body: &str) -> HString<1024> {
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
///
/// Transform a light value to a rgb intensity.
///
fn light_to_intensity(adc: u16) -> u8 {
    let value = if adc <= 2200 {
        40
    } else if adc > 2300 && adc < 3000 {
        10
    } else {
        0
    };
    value
}

//
// Embassy RGB led animation
//
#[embassy_executor::task]
async fn breath_task() {
    breath().await;
    set_rgb_led_online().await;
}

///
/// Update firmware when a new version is available.
///
#[embassy_executor::task]
async fn firmware_update_task(mut duck_firmware: DuckFirmware<'static>) {
    duck_firmware.update_firmware().await;
}

///
/// Conection network initialization
///
#[embassy_executor::task]
async fn network_connection_task(connection: NetworkConnection<'static, WifiDevice<'static>>) {
    connection.connect_task().await
}

///
/// Wifi initialization
///
#[embassy_executor::task]
async fn wifi_connection_task(mut wifi: Wifi) {
    set_rgb_led_offline().await;
    wifi.wifi_connect_task().await;
}

///
/// Rgb change color administrator
///
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

//
//Get current version of firmware
//
fn get_current_version() -> &'static str {
    CURRENT_VERSION
}

//
// Wrap sensors
//    Light sensor
//    Moisture sensor
struct Sensor<'a, I, M>
where
    I: PeripheralInput<'a>,
    M: PeripheralInput<'a>,
{
    light_sensor_pin: AdcPin<I, ADC1<'a>>,
    moisture_sensor_pin: AdcPin<M, ADC1<'a>>,
    adc1: Adc<'a, ADC1<'a>, Blocking>,
}

impl<'a, I, M> Sensor<'a, I, M>
where
    I: PeripheralInput<'a> + AdcChannel + AnalogPin,
    M: PeripheralInput<'a> + AdcChannel + AnalogPin,
{
    // Sensor manager
    fn new(gpio_light: I, gpio_moisture: M, adc1: ADC1<'a>) -> Self {
        let mut adc1_config = esp_hal::analog::adc::AdcConfig::new();
        let gpio_light =
            adc1_config.enable_pin(gpio_light, esp_hal::analog::adc::Attenuation::_11dB);
        let gpio_moisture =
            adc1_config.enable_pin(gpio_moisture, esp_hal::analog::adc::Attenuation::_11dB);
        Sensor {
            adc1: esp_hal::analog::adc::Adc::new(adc1, adc1_config),
            light_sensor_pin: gpio_light,
            moisture_sensor_pin: gpio_moisture,
        }
    }

    // Get current light value from sensor
    fn current_light(&mut self) -> u16 {
        match nb::block!(self.adc1.read_oneshot(&mut self.light_sensor_pin)) {
            Ok(val) => light_to_percent(val),
            Err(_) => 0,
        }
    }

    // Get current moisture value from sensor
    fn current_moisture(&mut self) -> u8 {
        match nb::block!(self.adc1.read_oneshot(&mut self.moisture_sensor_pin)) {
            Ok(val) => moistorure_to_percent(val),
            Err(_) => 0,
        }
    }
}

fn light_to_percent(value: u16) -> u16 {
    const MIN_LIGHT: u16 = 2100;
    const MAX_LIGHT: u16 = 3000;
    let percent = ((value - MIN_LIGHT) * 100) / MAX_LIGHT;
    percent
}

fn moistorure_to_percent(value: u16) -> u8 {
    let wet_start = 2700i32;
    let wet_end = 2800i32;
    let dry = 4095i32;
    let v = value as i32;

    if v < wet_start {
        0
    } else if v <= wet_end {
        100
    } else {
        (((dry - v) * 100) / (dry - wet_end)).clamp(0, 100) as u8
    }
}
