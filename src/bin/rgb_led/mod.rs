use embassy_time::{Duration, Timer};
use esp_hal::Blocking;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::peripherals::{GPIO8, RMT};
use esp_hal::rmt::{PulseCode, Rmt};
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
        .send(RgbLedCommand::SetColor(RgbColor::White))
        .await;
}
pub async fn set_rgb_led_light(level: u8) {
    CHANGE_LED_COLOR
        .send(RgbLedCommand::SetCustomColor((
            smart_leds::hsv::Hsv {
                hue: 255,
                sat: 0,
                val: 255,
            },
            level,
        )))
        .await;
}
pub async fn breath() {
    for _ in 0..2 {
        for i in 0..255 {
            CHANGE_LED_COLOR
                .send(RgbLedCommand::SetCustomColor((
                    smart_leds::hsv::Hsv {
                        hue: i,
                        sat: 255,
                        val: 255,
                    },
                    20,
                )))
                .await;
            Timer::after(Duration::from_millis(30)).await
        }
    }
}
pub async fn breath2() {
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

pub struct RgbLed<'ch, O>
where
    O: PeripheralOutput<'ch>,
{
    rmt: Rmt<'ch, Blocking>,
    gpio: O,
}
impl<'ch, O> RgbLed<'ch, O>
where
    O: PeripheralOutput<'ch>,
{
    pub fn new(peripheral: RMT<'ch>, gpio: O) -> Self {
        let rmt: esp_hal::rmt::Rmt<'_, esp_hal::Blocking> = {
            let frequency: esp_hal::time::Rate = esp_hal::time::Rate::from_mhz(80);
            esp_hal::rmt::Rmt::new(peripheral, frequency)
        }
        .unwrap();

        Self { gpio, rmt }
    }
    pub fn with_buffer(self, buffer: &'ch mut [PulseCode; 25]) -> RgbLedAdapter<'ch> {
        let rgb_led = SmartLedsAdapter::new(self.rmt.channel0, self.gpio, buffer);
        RgbLedAdapter::new(rgb_led)
    }
}

pub struct RgbLedAdapter<'ch> {
    rgb_led: SmartLedsAdapter<'ch, 25>,
}
impl<'ch> RgbLedAdapter<'ch> {
    pub fn new(rgb_led: SmartLedsAdapter<'ch, 25>) -> Self {
        Self { rgb_led }
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
