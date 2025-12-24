#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
mod firmware;
mod helpers;
mod ota;
mod rgb_led;
use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_net::Config;
use embassy_net::Runner;
use embassy_net::Stack;
use embassy_net::driver::Driver;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::GPIO8;
use esp_hal::peripherals::RMT;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::{
    ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
};
use esp_storage::FlashStorage;
use log::info;
use trouble_host::prelude::*;

use crate::firmware::DuckFirmware;
use crate::helpers::CHANGE_LED_COLOR;
use crate::helpers::RgbColor;
use crate::helpers::set_rgb_led_color;
use crate::helpers::{WIFI_READY, WifiState};
use crate::rgb_led::RgbLedComponent;

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

const CURRENT_VERSION: &str = "1.0.30";
const FIRMWARE_FILE_NAME: &str = "duck-firmware.bin";
const VERSION_FILE_NAME: &str = "version.json";
const FIRMWARE_HOST: &str = "http://192.168.100.185:80";
const WIFI_NAME: &str = "Diego";
const WIFI_PASSWORD: &str = "Diego777";

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

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

    spawner
        .spawn(rgb_control(peripherals.RMT, peripherals.GPIO8))
        .ok();

    // Initialize wifi service
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
        seed: helpers::generate_seed(),
    };

    // Initialize  network connection
    let (stack, runner) = build_network(network_config);
    spawner.spawn(net_task(runner)).ok();

    wait_for_network_connection(stack).await;

    wait_for_network_ip(stack).await;

    let flash = FlashStorage::new(peripherals.FLASH);
    let input_config = esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up);
    let boot_button = esp_hal::gpio::Input::new(peripherals.GPIO9, input_config);
    let duck_firmware = firmware::DuckFirmware::new(
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
    set_rgb_led_color(RgbColor::Red).await;
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
                set_rgb_led_color(RgbColor::Blue).await;
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
    set_rgb_led_color(RgbColor::Pink).await;
    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            set_rgb_led_color(RgbColor::White).await;
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

fn dhcpv4_default_config() -> Config {
    embassy_net::Config::dhcpv4(Default::default())
}
fn get_current_version() -> &'static str {
    CURRENT_VERSION
}
