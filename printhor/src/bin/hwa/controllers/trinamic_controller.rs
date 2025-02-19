//! TODO: This feature is still very experimental/preliminar
use crate::hwa;
use embassy_time::Instant;
use hwa::uart::SerialError;
#[allow(unused)]
use hwa::Contract;
use hwa::CoordSel;

/// Represents errors that can occur in the Trinamic UART controller.
#[derive(Debug, Clone, Copy)]
pub enum TrinamicError {
    /// Indicates there is something bad in the commanded settings
    BadSettings,
    /// Indicates a timeout error, which occurs when an operation exceeds the allotted time.
    Timeout,
    /// Indicates a write error, which occurs when there is an issue writing data to the UART.
    WriteError,
    /// Indicates a read error, which occurs when there is a problem reading data from the UART.
    ReadError,
}

/// Represents a controller for Trinamic UART-based devices.
///
/// The `TrinamicController` is responsible for handling communication
/// and control of Trinamic UART-based stepper motor drivers. It provides
/// methods to initialize the steppers, write to and read from the registers,
/// and manage different motion configurations of the steppers.
pub struct TrinamicController {
    /// UART interface for communicating with the Trinamic device.
    ///
    /// This interface facilitates the transfer of data between the controller
    /// and the Trinamic device, allowing for sending commands and receiving responses.
    uart: hwa::types::TrinamicUart,
    /// Motion configuration reference used by the controller.
    ///
    /// The motion configuration holds various settings related to the movement
    /// of the stepper motors, such as microstep configurations and possibly other
    /// parameters needed to correctly drive the motors.
    motion_config: hwa::controllers::MotionConfig,
}
impl TrinamicController {
    pub const fn new(
        uart: hwa::types::TrinamicUart,
        motion_config: hwa::controllers::MotionConfig,
    ) -> Self {
        Self {
            uart,
            motion_config,
        }
    }

    pub async fn init(&mut self) -> Result<(), TrinamicError> {
        hwa::debug!("[TrinamicController] Init");

        let mut init_error = false;

        // TODO: 1- Right now, [hwa::CoordSel] is still not feature based, but will be
        // TODO: 2- Right now, stepper directions are hard-coded, but will be [cfg_env] conf based

        // 1. Get the micro_steps setting from shared config
        let micro_steps_setting = self.motion_config.get_micro_steps();

        #[cfg(feature = "with-x-axis")]
        {
            use hwa::traits::TrinamicUartTrait;
            let address = self
                .uart
                .get_tmc_address(CoordSel::X)
                .map_err(|_| TrinamicError::BadSettings)?;

            init_error |= Self::init_channel(
                &mut self.uart,
                CoordSel::X,
                address,
                micro_steps_setting.x.unwrap(),
            )
            .await
            .is_err();
        }

        #[cfg(feature = "with-y-axis")]
        {
            use hwa::traits::TrinamicUartTrait;
            let address = self
                .uart
                .get_tmc_address(CoordSel::Y)
                .map_err(|_| TrinamicError::BadSettings)?;

            init_error |= Self::init_channel(
                &mut self.uart,
                CoordSel::Y,
                address,
                micro_steps_setting.y.unwrap(),
            )
            .await
            .is_err();
        }

        #[cfg(feature = "with-z-axis")]
        {
            use hwa::traits::TrinamicUartTrait;
            let address = self
                .uart
                .get_tmc_address(CoordSel::Z)
                .map_err(|_| TrinamicError::BadSettings)?;

            init_error |= Self::init_channel(
                &mut self.uart,
                CoordSel::Z,
                address,
                micro_steps_setting.z.unwrap(),
            )
            .await
            .is_err();
        }

        #[cfg(feature = "with-e-axis")]
        {
            use hwa::traits::TrinamicUartTrait;
            let address = self
                .uart
                .get_tmc_address(CoordSel::E)
                .map_err(|_| TrinamicError::BadSettings)?;

            init_error |= Self::init_channel(
                &mut self.uart,
                CoordSel::E,
                address,
                micro_steps_setting.e.unwrap(),
            )
            .await
            .is_err();
        }

        if init_error {
            hwa::error!("[TrinamicController] returning bad settings error");
            Err(TrinamicError::BadSettings)
        } else {
            #[cfg(feature = "trace-commands")]
            hwa::info!("[TrinamicController] returning OK");
            Ok(())
        }
    }

    async fn init_channel(
        uart: &mut hwa::types::TrinamicUart,
        channel: hwa::CoordSel,
        address: u8,
        configured_micro_steps: u16,
    ) -> Result<(), ()> {
        use hwa::traits::TrinamicUartTrait;
        uart.select_stepper_of_axis(channel)?;
        if let Ok(micro_steps_tmc_power_of_two) = to_tmc_micro_steps(configured_micro_steps) {
            match Self::init_stepper(uart, address, micro_steps_tmc_power_of_two).await {
                Ok(_) => {
                    //#[cfg(feature = "trace-commands")]
                    hwa::info!(
                        "[trace-commands] [TrinamicController] {:?} initialized",
                        channel
                    );
                    let _ = uart.select_stepper_of_axis(CoordSel::empty());
                    Ok(())
                }
                Err(_err) => {
                    hwa::error!(
                        "[TrinamicController] Unable to initialize stepper: {:?}",
                        channel,
                    );
                    let _ = uart.select_stepper_of_axis(CoordSel::empty());
                    Err(())
                }
            }
        } else {
            hwa::error!(
                "[TrinamicController]: Error initializing {:?} invalid micro-stepping setting ({})",
                channel,
                configured_micro_steps
            );
            let _ = uart.select_stepper_of_axis(CoordSel::empty());
            Err(())
        }
    }

    async fn init_stepper(
        uart: &mut hwa::types::TrinamicUart,
        addr: u8,
        micro_steps_pow_of_2: u32,
    ) -> Result<(), TrinamicError> {
        //#[cfg(feature = "trace-commands")]
        hwa::info!("[TrinamicController] applying gconf on {}", addr);

        let mut g_conf = tmc2209::reg::GCONF::default();
        // Invert direction
        g_conf.set_shaft(false);
        // Enable UART Comm
        g_conf.set_pdn_disable(true);
        // Enable Spread Cycle: Higher Torque, more noise
        g_conf.set_en_spread_cycle(true);
        // Enable multistepping
        g_conf.set_mstep_reg_select(true);
        // Enable multi-stepping filtering
        g_conf.set_multistep_filt(false);
        if Self::write_register(uart, addr, g_conf).await.is_ok() {
            //#[cfg(feature = "trace-commands")]
            hwa::info!("[TrinamicController] applying chopconf on addr {}", addr);
            let mut chop_conf = tmc2209::reg::CHOPCONF::default();
            chop_conf.set_intpol(false);
            #[cfg(not(feature = "pulsed"))]
            chop_conf.set_dedge(true);
            chop_conf.set_mres(micro_steps_pow_of_2);
            if Self::write_register(uart, addr, chop_conf).await.is_ok() {
                hwa::info!("[TrinamicController] Write OK to addr {}", addr);
            }
        } else {
            hwa::error!("[TrinamicController] Error initializing stepper {}", addr);
        }
        // TODO this is still a mess
        // Still not receiving the proper clean result, however the data is properly sent

        if let Ok(chop_conf) = Self::read_register::<tmc2209::reg::CHOPCONF>(uart, addr).await {
            if chop_conf.ntpol() == false && chop_conf.mres() == micro_steps_pow_of_2 {
                hwa::warn!(
                    "[TrinamicController] Trinamic check for stepper {} is OK",
                    addr
                );
            } else {
                hwa::warn!(
                    "[TrinamicController] Trinamic check for stepper {} is BAD",
                    addr
                )
            }
        } else {
            hwa::warn!(
                "[TrinamicController] Unable to retrieve status from {}",
                addr
            )
        }

        // TODO: Assuming OK as of now...
        Ok(())
    }

    async fn write_register<T: tmc2209::WritableRegister>(
        uart: &mut hwa::types::TrinamicUart,
        slave_addr: u8,
        reg: T,
    ) -> Result<(), TrinamicError> {
        Self::raw_write(uart, tmc2209::WriteRequest::new(slave_addr, reg).bytes()).await
    }

    #[allow(unused)]
    async fn read_register<T: tmc2209::ReadableRegister + core::fmt::Debug>(
        uart: &mut hwa::types::TrinamicUart,
        slave_addr: u8,
    ) -> Result<T, TrinamicError> {
        hwa::info!("[TrinamicController] Send read request");
        Self::raw_write(uart, tmc2209::read_request::<T>(slave_addr).bytes()).await?;

        hwa::info!("[TrinamicController] Now reading response...");
        let mut buff = [0u8; 32];
        let mut reader = tmc2209::Reader::default();
        let reception_deadline = Instant::now() + embassy_time::Duration::from_secs(5);
        loop {
            if Instant::now() > reception_deadline {
                hwa::warn!("Reception deadline expired!");
                return Err(TrinamicError::Timeout);
            }
            use hwa::traits::TrinamicUartTrait;
            match uart.read_until_idle(&mut buff).await {
                Ok(num_bytes_read) => {
                    if num_bytes_read > 0 {
                        hwa::info!("[TrinamicController] Uart read {} bytes", num_bytes_read);
                        match reader.read_response(&buff[0..num_bytes_read]) {
                            (_n, Some(response)) => {
                                return match response.register::<T>() {
                                    Ok(r) => {
                                        let x = alloc::format!("{:?}", r);
                                        hwa::debug!("response[0]. l={} : {}", _n, x.as_str());
                                        Ok(r)
                                    }
                                    _ => Err(TrinamicError::ReadError),
                                }
                            }
                            (n, None) => {
                                let x = alloc::format!("{:?}", reader.awaiting());
                                hwa::warn!(
                                    "[TrinamicController] Uncompleted. (readed {}: {:?}) awaiting {}",
                                    n,
                                    &buff[0..num_bytes_read],
                                    x.as_str()
                                );
                            }
                        }
                    }
                }
                Err(SerialError::Timeout) => {
                    //hwa::trace!("Frame lost");
                    // Lost one frame.
                    // Continue until [reception_timeout]
                }
                Err(err) => {
                    hwa::error!(
                        "[TrinamicController] Read error reading trinamic: {:?}",
                        err
                    );
                    return Err(TrinamicError::ReadError);
                }
            }
        }
    }

    pub async fn raw_write(
        uart: &mut hwa::types::TrinamicUart,
        bytes: &[u8],
    ) -> Result<(), TrinamicError> {
        hwa::info!("[TrinamicController] sending {:?}", bytes);
        use hwa::traits::TrinamicUartTrait;
        uart.write(bytes)
            .await
            .map_err(|_| TrinamicError::WriteError)?;
        let _ = uart.flush().await.map_err(|_| TrinamicError::WriteError)?;
        Ok(())
    }
}

/// Convert the number of microsteps (1, 2, 4,...) to the proper TMC setting
fn to_tmc_micro_steps(num_microsteps: u16) -> Result<u32, ()> {
    match num_microsteps {
        1 => Ok(8),
        2 => Ok(7),
        4 => Ok(6),
        8 => Ok(5),
        16 => Ok(4),
        32 => Ok(3),
        64 => Ok(2),
        128 => Ok(1),
        256 => Ok(0), // Weird, but that's what TMC datasheet says
        _ => Err(()),
    }
}
