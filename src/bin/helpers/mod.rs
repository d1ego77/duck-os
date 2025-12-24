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
    White,
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

pub fn generate_seed() -> u64 {
    let rng = esp_hal::rng::Rng::new();
    (rng.random() as u64) << 32 | rng.random() as u64
}
pub fn extract_version(json: &str) -> Option<heapless::String<16>> {
    let key = "\"version\"";
    let start = json.find(key)? + key.len();
    let start = json[start..].find('"')? + start + 1;
    let end = json[start..].find('"')? + start;

    let mut version = heapless::String::<16>::new();
    version.push_str(&json[start..end]).ok()?;

    Some(version)
}

pub fn parse_version(v: &str) -> Option<(u8, u8, u8)> {
    let mut it = v.trim().split('.');
    Some((
        it.next()?.parse().ok()?,
        it.next()?.parse().ok()?,
        it.next()?.parse().ok()?,
    ))
}

pub fn is_newer(remote: &str, local: &str) -> bool {
    match (parse_version(remote), parse_version(local)) {
        (Some(r), Some(l)) => r > l,
        _ => false,
    }
}

#[allow(dead_code)]
fn handle_error(error: &DuckError) {
    match error {
        DuckError::NetworkError => defmt::error!("Network error ocurred."),
    }
}
