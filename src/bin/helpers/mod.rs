// Preconfigured rgb led colors
#[allow(dead_code)]
pub enum RgbColor {
    Red,
    Burgundy,
    Green,
    Blue,
    Yellow,
    Pink,
    Orange,
    Gray,
    White,
    None,
}

impl RgbColor {
    pub fn hsv(&self) -> smart_leds::hsv::Hsv {
        match self {
            RgbColor::Red => smart_leds::hsv::Hsv {
                hue: 0,
                sat: 255,
                val: 255,
            },
            RgbColor::Burgundy => smart_leds::hsv::Hsv {
                hue: 87,
                sat: 200,
                val: 175,
            },
            RgbColor::Green => smart_leds::hsv::Hsv {
                hue: 87,
                sat: 255,
                val: 255,
            },
            RgbColor::Blue => smart_leds::hsv::Hsv {
                hue: 191,
                sat: 255,
                val: 255,
            },
            RgbColor::Yellow => smart_leds::hsv::Hsv {
                hue: 45,
                sat: 255,
                val: 255,
            },
            RgbColor::Pink => smart_leds::hsv::Hsv {
                hue: 213,
                sat: 255,
                val: 255,
            },
            RgbColor::Gray => smart_leds::hsv::Hsv {
                hue: 0,
                sat: 0,
                val: 128,
            },
            RgbColor::Orange => smart_leds::hsv::Hsv {
                hue: 30,
                sat: 255,
                val: 255,
            },
            RgbColor::White => smart_leds::hsv::Hsv {
                hue: 255,
                sat: 0,
                val: 255,
            },
            RgbColor::None => smart_leds::hsv::Hsv {
                hue: 0,
                sat: 0,
                val: 0,
            },
        }
    }
}

pub enum RgbLedCommand {
    SetColor(RgbColor),
    SetCustom(smart_leds::hsv::Hsv),
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
