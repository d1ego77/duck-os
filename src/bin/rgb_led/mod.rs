use embassy_time::{Duration, Timer};
use esp_hal::peripherals::{GPIO8, RMT};
use esp_hal::rmt::PulseCode;
use esp_hal_smartled::SmartLedsAdapter;
use smart_leds::SmartLedsWrite;

use rgb::Grb;

use crate::channel::CHANGE_LED_COLOR;
use crate::helpers::{RgbColor, RgbLedCommand};

// Send the signal to change the led color
pub async fn set_rgb_led_color(color: RgbLedCommand) {
    CHANGE_LED_COLOR.send(color).await;
}

pub async fn set_rgb_led_wifi_connected() {
    CHANGE_LED_COLOR
        .send(RgbLedCommand::SetColor(RgbColor::Blue))
        .await;
}

pub async fn set_rgb_led_offline() {
    CHANGE_LED_COLOR
        .send(RgbLedCommand::SetColor(RgbColor::Red))
        .await;
}

pub async fn set_rgb_led_online() {
    CHANGE_LED_COLOR
        .send(RgbLedCommand::SetColor(RgbColor::Burgundy))
        .await;
}

pub async fn breath() {
    loop {
        CHANGE_LED_COLOR
            .send(RgbLedCommand::SetColor(RgbColor::Blue))
            .await;
        Timer::after(Duration::from_secs(1)).await;
        CHANGE_LED_COLOR
            .send(RgbLedCommand::SetColor(RgbColor::Red))
            .await;
        Timer::after(Duration::from_secs(1)).await;
        CHANGE_LED_COLOR
            .send(RgbLedCommand::SetColor(RgbColor::Green))
            .await;
        Timer::after(Duration::from_secs(1)).await;
        CHANGE_LED_COLOR
            .send(RgbLedCommand::SetColor(RgbColor::Yellow))
            .await;
        Timer::after(Duration::from_secs(1)).await;
        CHANGE_LED_COLOR
            .send(RgbLedCommand::SetColor(RgbColor::Pink))
            .await;
        Timer::after(Duration::from_secs(1)).await;
        CHANGE_LED_COLOR
            .send(RgbLedCommand::SetColor(RgbColor::White))
            .await;
        Timer::after(Duration::from_secs(1)).await;
    }
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
    pub fn set_color(&mut self, color: smart_leds::hsv::Hsv, level: u8) {
        let data: rgb::RGB8 = smart_leds::hsv::hsv2rgb(color);
        self.rgb_led
            .write(smart_leds::brightness(
                smart_leds::gamma([data].into_iter()),
                level,
            ))
            .expect("Error al setear el color")
    }
}
