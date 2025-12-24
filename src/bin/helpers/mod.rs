use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};

// Signal to notify wifi initialization is ready
pub static WIFI_READY: Signal<CriticalSectionRawMutex, WifiState> = Signal::new();

// Signal to nitify change color of rgb led
pub static CHANGE_LED_COLOR: Channel<CriticalSectionRawMutex, RgbColor, 4> = Channel::new();

// Send the signal to change the led color
pub async fn set_rgb_led_color(color: RgbColor) {
    CHANGE_LED_COLOR.send(color).await
}

// Preconfigured rgb led colors
#[allow(dead_code)]
pub enum RgbColor {
    Red,
    Green,
    Blue,
    Yellow,
    Pink,
    Orange,
    Gray,
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
