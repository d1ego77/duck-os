pub static WIFI_READY: Signal<CriticalSectionRawMutex, WifiState> = Signal::new();
pub static CHANGE_LED_COLOR: Channel<CriticalSectionRawMutex, DuckColor, 4> = Channel::new();
