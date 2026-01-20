use alloc::{borrow::ToOwned, string::String};
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::{GPIO2, GPIO6};
use heapless::{String as HString, format};
use log::info;

use crate::{
    I2cSensor, Sensor,
    helpers::{WEB_SERVER_PORT, get_current_version},
};

pub fn cors_headers() -> &'static str {
    // Puedes ajustar los valores si quieres restringir orígenes o métodos
    "Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type\r\n"
}

pub fn http_response(status: u16, content_type: &str, body: &str) -> HString<1024> {
    format!(
        "HTTP/1.1 {} OK\r\n{}Content-Type: {}\r\nContent-Length: {}\r\n\r\n{}",
        status,
        cors_headers(),
        content_type,
        body.len(),
        body
    )
    .unwrap()
}

fn get_request(buf: &[u8; 1024], request_size: usize) -> String {
    let request = core::str::from_utf8(&buf[..request_size]).unwrap();
    request.to_owned()
}

///
/// Web server
///
#[embassy_executor::task]
pub async fn web_server_task(
    stack: embassy_net::Stack<'static>,
    mut sensor_manager: Sensor<'static, GPIO6<'static>, GPIO2<'static>>,
    mut i2c_sensor_manager: I2cSensor<'static>,
) {
    loop {
        Timer::after(Duration::from_millis(20)).await;
        let mut rx_buffer = [0; 4096];
        let mut tx_buffer = [0; 4096];

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        info!("Esperando conexiones ");
        socket.accept(WEB_SERVER_PORT).await.unwrap();
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        info!("Conexion recibida ");
        let mut buf = [0; 1024];
        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    info!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    info!("read error: {:?}", e);
                    break;
                }
            };

            let payload = core::str::from_utf8(&buf[..n]).unwrap();

            let (temperature, humidity, pressure, altitude) = i2c_sensor_manager.current_values();
            let light = sensor_manager.current_light();
            let moisture = sensor_manager.current_moisture();

            let sensor_json: HString<512> = format!(
                "{{\
                \"light\": {},\
                \"moisture\": {},\
                \"temperature\": {:.2},\
                \"humidity\": {:.2},\
                \"pressure\": {:.2},\
                \"altitude\": {:.2},\
                \"version\": \"{}\"\
                }}",
                light,
                moisture,
                temperature,
                humidity,
                pressure,
                altitude,
                get_current_version()
            )
            .unwrap();

            let response = http_response(200, "application/json; charset=utf-8", &sensor_json);
            socket.write(response.as_bytes()).await.ok();
            Timer::after(Duration::from_millis(30)).await;
            socket.close();
            break;
        }
    }
}
