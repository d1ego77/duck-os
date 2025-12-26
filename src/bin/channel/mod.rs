use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};

use crate::helpers::{RgbColor, WifiState};

pub static WIFI_READY: Signal<CriticalSectionRawMutex, WifiState> = Signal::new();
pub static CHANGE_LED_COLOR: Channel<CriticalSectionRawMutex, RgbColor, 4> = Channel::new();
