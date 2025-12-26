use esp_hal::peripherals::{GPIO8, RMT};
use esp_hal::rmt::PulseCode;
use esp_hal_smartled::SmartLedsAdapter;
use smart_leds::SmartLedsWrite;

use rgb::Grb;

use crate::channel::CHANGE_LED_COLOR;
use crate::helpers::RgbColor;

// Send the signal to change the led color
pub async fn set_rgb_led_color(color: RgbColor) {
    CHANGE_LED_COLOR.send(color).await
}

pub async fn set_rgb_led_online() {
    CHANGE_LED_COLOR.send(RgbColor::White).await;
}

pub struct RgbLedComponent<'ch, Color = Grb<u8>> {
    rgb_led: SmartLedsAdapter<'ch, 25, Color>,
}
impl<'ch> RgbLedComponent<'ch, Grb<u8>> {
    pub fn new(
        peripheral: RMT<'ch>,
        gpio8: GPIO8<'ch>,
        rmt_buffer: &'ch mut [PulseCode; 25],
    ) -> RgbLedComponent<'ch> {
        let rmt: esp_hal::rmt::Rmt<'_, esp_hal::Blocking> = {
            let frequency: esp_hal::time::Rate = esp_hal::time::Rate::from_mhz(80);
            esp_hal::rmt::Rmt::new(peripheral, frequency)
        }
        .unwrap();
        Self {
            rgb_led: SmartLedsAdapter::new(rmt.channel0, gpio8, rmt_buffer),
        }
    }
    pub fn set_color(&mut self, color: RgbColor, level: u8) {
        let color = match color {
            RgbColor::Red => smart_leds::hsv::Hsv {
                hue: 0,
                sat: 255,
                val: 255,
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
        };
        let data: rgb::RGB8 = smart_leds::hsv::hsv2rgb(color);
        self.rgb_led
            .write(smart_leds::brightness(
                smart_leds::gamma([data].into_iter()),
                level,
            ))
            .expect("Error al setear el color")
    }
}
