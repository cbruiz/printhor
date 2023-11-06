//! TODO: This feature is still very experimental/preliminar
use crate::hwa;
use crate::hwa::device::UartTrinamic;

pub enum TrinamicError {
    Timeout,
    WriteError,
    ReadError,
}

pub struct TrinamicController
{
    uart: UartTrinamic,
}
impl TrinamicController
{
    pub fn new(uart: UartTrinamic) -> Self {
        Self{uart}
    }

    pub async fn init(&mut self) -> Result<(),TrinamicError> {
        hwa::info!("Trinamic_uart CMD");

        let _status = self.read_register::<tmc2209::reg::DRV_STATUS>(0).await?;

        let mut gconf = tmc2209::reg::GCONF::default();
        gconf.set_en_spread_cycle(true);
        gconf.set_shaft(false);
        gconf.set_mstep_reg_select(true);
        let _ = self.write_register(0, gconf).await?;
        let _ = self.write_register(1, gconf).await?;
        let _ = self.write_register(2, gconf).await?;
        let _ = self.write_register(3, gconf).await?;

        let mut chopconf = tmc2209::reg::CHOPCONF::default();
        chopconf.set_intpol(true);
        chopconf.set_mres(16);
        let _ = self.write_register(0, chopconf).await?;
        let _ = self.write_register(1, chopconf).await?;
        let _ = self.write_register(2, chopconf).await?;
        let _ = self.write_register(3, chopconf).await?;
        Ok(())
    }

    async fn write_register<T: tmc2209::WritableRegister>(&mut self, slave_addr: u8, reg: T) -> Result<(), TrinamicError>
    {
        self.raw_write(tmc2209::WriteRequest::new(slave_addr, reg).bytes()).await
    }

    async fn read_register<T: tmc2209::ReadableRegister>(&mut self, slave_addr: u8) -> Result<T, TrinamicError>
    {
        self.raw_write(tmc2209::read_request::<T>(slave_addr).bytes()).await?;
        let mut buff:[u8; 8] = [0; 8];
        let mut reader = tmc2209::Reader::default();
        loop {
            match embassy_time::with_timeout(
                embassy_time::Duration::from_secs(1),
                self.uart.read_until_idle(&mut buff)).await {
                Ok(Ok(num_bytes_read)) => {
                    hwa::trace!("read {} bytes", num_bytes_read);
                    if num_bytes_read > 0 {
                        match reader.read_response(&buff[0..num_bytes_read]) {
                            (_n, Some(response)) => {
                                return match response.register::<T>() {
                                    Ok(r) => {
                                        //let x = alloc::format!("{:?}", r);
                                        //crate::info!("response[0]. l={} : {}", _n, x.as_str());
                                        Ok(r)
                                    }
                                    _ => {
                                        Err(TrinamicError::ReadError)
                                    }
                                }
                            }
                            (n, None) => {
                                let x = alloc::format!("{:?}", reader.awaiting());
                                hwa::trace!("Uncompleted. (readed {}: {}) awaiting {}", n, &buff[0..num_bytes_read], x.as_str());
                            }
                        }
                    }
                }
                Ok(Err(err)) => {
                    let err = alloc::format!("{:?}", err);
                    hwa::error!("read error reading trinamic: {:?}", err.as_str());
                    return Err(TrinamicError::ReadError)
                }
                Err(_) => {
                    hwa::error!("timeout waiting for trinamic response");
                    return Err(TrinamicError::Timeout)
                }
            }
        }
    }

    async fn raw_write(&mut self, bytes: &[u8]) -> Result<(), TrinamicError> {
        hwa::debug!("Trinamic_uart sent {}", bytes);
        self.uart.write(bytes).await.map_err(|_| TrinamicError::WriteError)?;
        Ok(self.uart.blocking_flush().map_err(|_| TrinamicError::WriteError)?)
    }
}
