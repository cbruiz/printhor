use embassy_time::Duration;
use crate::board::{comm, TRINAMIC_UART_BAUD_RATE};
use crate::device::AxisChannel;

#[allow(unused)]
pub struct MockedTrinamicDriver {
    pub(crate) uart_trinamic: comm::SingleWireSoftwareUart
}

impl MockedTrinamicDriver {
    pub(crate) fn new(x_rxtx_pin: crate::board::MockedIOPin, y_rxtx_pin: crate::board::MockedIOPin, z_rxtx_pin: crate::board::MockedIOPin, e_rxtx_pin: crate::board::MockedIOPin) -> Self {
        let uart_trinamic = comm::SingleWireSoftwareUart::new(
            TRINAMIC_UART_BAUD_RATE,
            x_rxtx_pin, y_rxtx_pin, z_rxtx_pin, e_rxtx_pin
        );

        Self {
            uart_trinamic
        }
    }
}

#[embassy_executor::task(pool_size=1)]
pub async fn trinamic_driver_simulator(mut driver: MockedTrinamicDriver) {

    log::info!("Trinamic Simulator stared");

    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(100));
    let mut buff: [u8; 32] = [0; 32];
    loop {
        driver.uart_trinamic.set_axis_channel(Some(AxisChannel::TMCUartX));

        let mut reader = tmc2209::Reader::default();

        loop {
            match driver.uart_trinamic.read_until_idle(&mut buff).await {
                Ok(num_bytes_read) => {
                    buff[1] = 0xff;
                    log::info!("Simulated Uart read {} bytes", num_bytes_read);

                    if num_bytes_read > 0 {

                        match reader.read_response(&buff[0..num_bytes_read]) {
                            (_n, Some(response)) => {
                                let ma = response.master_addr();
                                let mm = response.reg_addr();
                                log::info!("MA: {}, mm: {:?}", ma, mm)
                                /*
                                return match response.register::<T>() {
                                    Ok(r) => {
                                        //let x = alloc::format!("{:?}", r);
                                        //crate::info!("response[0]. l={} : {}", _n, x.as_str());
                                        //Ok(r)
                                    }
                                    _ => {
                                        //Err(TrinamicError::ReadError)
                                    }
                                }

                                 */
                            }
                            (n, None) => {
                                let x = format!("{:?}", reader.awaiting());
                                log::warn!("Uncompleted. (readed {}: {:?}) awaiting {}", n, &buff[0..num_bytes_read], x.as_str());
                            }
                        }
                    }
                }
                Err(_) => {
                    // Respawn
                    ticker.next().await;
                }
            }
        }
    }

}