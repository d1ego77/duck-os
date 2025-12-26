use embassy_executor::Spawner;
use embassy_net::driver::Driver;
use embassy_net::{Config, Runner, Stack};
use log::info;

use esp_radio::wifi::{
    ClientConfig, ModeConfig, ScanConfig, WifiController, WifiEvent, WifiStaState,
};

use crate::channel::WIFI_READY;
use crate::helpers::{RgbColor, WifiState, generate_seed};
use crate::rgb_led::{set_rgb_led_color, set_rgb_led_online};
use embassy_time::{Duration, Timer};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
pub(crate) use mk_static;

struct NetworkConfiguration<D>
where
    D: Driver,
{
    wifi_interface: D,
    config: Config,
    seed: u64,
}
pub async fn wifi_connect(
    mut controller: WifiController<'static>,
    wifi_name: &'static str,
    password: &'static str,
) {
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
pub fn build_network<'a, D>(wifi_interface: D) -> (Stack<'a>, Runner<'a, D>)
where
    D: Driver,
{
    let config = NetworkConfiguration {
        wifi_interface,
        config: dhcpv4_default_config(),
        seed: generate_seed(),
    };
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
pub async fn wait_for_network_connection<'a>(stack: Stack<'a>) {
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
pub async fn wait_for_network_ip<'a>(stack: Stack<'a>) {
    set_rgb_led_color(RgbColor::Pink).await;
    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            set_rgb_led_online().await;
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

fn dhcpv4_default_config() -> Config {
    embassy_net::Config::dhcpv4(Default::default())
}
