use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};

pub static WIFI_READY: Signal<CriticalSectionRawMutex, WifiState> = Signal::new();
pub static CHANGE_LED_COLOR: Channel<CriticalSectionRawMutex, DuckColor, 4> = Channel::new();

pub async fn set_rgb_led_color(color: DuckColor) {
    CHANGE_LED_COLOR.send(color).await
}

#[allow(dead_code)]
pub enum DuckColor {
    Red,
    Green,
    Blue,
    Yellow,
    Pink,
    Orange,
    None,
}

#[derive(Debug)]
pub enum DuckError {
    NetworkError,
}

pub type DuckResult<T> = core::result::Result<T, DuckError>;

#[allow(dead_code)]
pub enum WifiState {
    Connected,
    NoConnected,
}
