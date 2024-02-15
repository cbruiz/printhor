//! TODO: This feature is still very experimental/preliminar
use embassy_time::Instant;
use hwa::soft_uart::SerialError;
use crate::hwa;

pub enum TrinamicError {
    Timeout,
    WriteError,
    ReadError,
}

pub struct TrinamicController
{
    uart: hwa::device::TrinamicUart,
}
impl TrinamicController
{
    pub fn new(uart: hwa::device::TrinamicUart) -> Self {
        Self{uart}
    }

    pub async fn init(&mut self) -> Result<(),TrinamicError> {
        hwa::info!("Trinamic_uart CMD");

        cfg_if::cfg_if! {
            if #[cfg(feature = "trinamic-uart-multi-channel")] {
                self.uart.set_axis_channel(Some(hwa::device::AxisChannel::TMCUartX));
            }
        }

        let _status = self.read_register::<tmc2209::reg::DRV_STATUS>(0).await?;

        let mut gconf = tmc2209::reg::GCONF::default();
        gconf.set_en_spread_cycle(true);
        gconf.set_shaft(false);
        gconf.set_mstep_reg_select(true);
        let _ = self.write_register(0, gconf).await?;
        //let _ = self.write_register(1, gconf).await?;
        //let _ = self.write_register(2, gconf).await?;
        //let _ = self.write_register(3, gconf).await?;

        let mut chopconf = tmc2209::reg::CHOPCONF::default();
        chopconf.set_intpol(true);
        chopconf.set_mres(16);
        let _ = self.write_register(0, chopconf).await?;
        //let _ = self.write_register(1, chopconf).await?;
        //let _ = self.write_register(2, chopconf).await?;
        //let _ = self.write_register(3, chopconf).await?;
        Ok(())
    }

    async fn write_register<T: tmc2209::WritableRegister>(&mut self, slave_addr: u8, reg: T) -> Result<(), TrinamicError>
    {
        self.raw_write(tmc2209::WriteRequest::new(slave_addr, reg).bytes()).await
    }

    async fn read_register<T: tmc2209::ReadableRegister + core::fmt::Debug>(&mut self, slave_addr: u8) -> Result<T, TrinamicError>
    {
        hwa::info!("Sending request...");
        self.raw_write(tmc2209::read_request::<T>(slave_addr).bytes()).await?;

        hwa::info!("Now reading response...");
        let mut buff:[u8; 8] = [0; 8];
        let mut reader = tmc2209::Reader::default();
        let reception_timeout = Instant::now() + embassy_time::Duration::from_secs(5);
        loop {
            if Instant::now() > reception_timeout {
                hwa::info!("TO Req");
                return Err(TrinamicError::Timeout);
            }
            match self.uart.read_until_idle(&mut buff).await {
                Ok(num_bytes_read) => {
                    if num_bytes_read > 0 {
                        hwa::info!("Uart read {} bytes", num_bytes_read);
                        match reader.read_response(&buff[0..num_bytes_read]) {
                            (_n, Some(response)) => {
                                return match response.register::<T>() {
                                    Ok(r) => {
                                        let x = alloc::format!("{:?}", r);
                                        hwa::debug!("response[0]. l={} : {}", _n, x.as_str());
                                        Ok(r)
                                    }
                                    _ => {
                                        Err(TrinamicError::ReadError)
                                    }
                                }
                            }
                            (n, None) => {
                                let x = alloc::format!("{:?}", reader.awaiting());
                                hwa::warn!("Uncompleted. (readed {}: {:?}) awaiting {}", n, &buff[0..num_bytes_read], x.as_str());
                            }
                        }
                    }
                }
                Err(SerialError::Timeout) => {
                    // Lost one frame.
                    // Continue until [reception_timeout]
                }
                Err(err) => {
                    hwa::error!("Read error reading trinamic: {:?}", err);
                    return Err(TrinamicError::ReadError)
                }
            }
        }
    }

    pub async fn raw_write(&mut self, bytes: &[u8]) -> Result<(), TrinamicError> {
        hwa::info!("Trinamic_uart sent {:?}", bytes);
        self.uart.write(bytes).await.map_err(|_| TrinamicError::WriteError)?;
        let _ = self.uart.blocking_flush().map_err(|_| TrinamicError::WriteError)?;
        Ok(())
    }

}
