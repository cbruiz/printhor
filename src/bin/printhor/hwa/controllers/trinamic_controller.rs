//! TODO: This feature is still very experimental/preliminar
use crate::hwa;
use hwa::device::{AxisChannel, TrinamicUart};
use hwa::soft_uart::SerialError;
use embassy_time::Instant;

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
    uart: TrinamicUart,
    /// Motion configuration reference used by the controller.
    ///
    /// The motion configuration holds various settings related to the movement
    /// of the stepper motors, such as microstep configurations and possibly other
    /// parameters needed to correctly drive the motors.
    motion_config: hwa::StaticController<hwa::MotionConfigMutexType, hwa::controllers::MotionConfig>,
}
impl TrinamicController {
    pub const fn new(
        uart: TrinamicUart,
        motion_config: hwa::StaticController<hwa::MotionConfigMutexType, hwa::controllers::MotionConfig>,
    ) -> Self {
        Self {
            uart,
            motion_config,
        }
    }

    pub async fn init(&mut self) -> Result<(), TrinamicError> {
        hwa::debug!("[TrinamicController] Init");

        #[cfg(feature = "native")]
        hwa::resume_trinamic(AxisChannel::TMCUartX);

        let mut init_error = false;

        // TODO: 1- Right now, [hwa::device::AxisChannel] is still not feature based, but will be
        // TODO: 2- Right now, stepper directions are hard-coded, but will be [cfg_env] conf based

        // 1. Get the micro_steps setting from shared config
        let micro_steps_setting = self.motion_config.lock()
            .await.micro_steps_per_axis.clone();

        #[cfg(feature = "with-x-axis")]
        {
            #[cfg(feature = "native")]
            hwa::resume_trinamic(AxisChannel::TMCUartX);

            init_error |= Self::init_channel(
                &mut self.uart,
                AxisChannel::TMCUartX,
                0,
                micro_steps_setting[0]
            ).await.is_err();
        }

        #[cfg(feature = "with-y-axis")]
        {
            #[cfg(feature = "native")]
            hwa::resume_trinamic(AxisChannel::TMCUartY);

            init_error |= Self::init_channel(
                &mut self.uart,
                AxisChannel::TMCUartY,
                1,
                micro_steps_setting[1]
            ).await.is_err();
        }

        #[cfg(feature = "with-z-axis")]
        {
            #[cfg(feature = "native")]
            hwa::resume_trinamic(AxisChannel::TMCUartZ);

            init_error |= Self::init_channel(
                &mut self.uart,
                AxisChannel::TMCUartZ,
                2,
                micro_steps_setting[2]
            ).await.is_err();
        }

        #[cfg(feature = "with-e-axis")]
        {
            #[cfg(feature = "native")]
            hwa::resume_trinamic(AxisChannel::TMCUartE);

            init_error |= Self::init_channel(
                &mut self.uart,
                AxisChannel::TMCUartE,
                3,
                micro_steps_setting[3]
            ).await.is_err();
        }

        #[cfg(feature = "native")]
        hwa::pause_trinamic();

        if init_error {
            Err(TrinamicError::BadSettings)
        }
        else {
            Ok(())
        }

    }

    async fn init_channel(
        uart: &mut TrinamicUart,
        channel: AxisChannel,
        address: u8,
        configured_micro_steps: u16) -> Result<(), ()>
    {

        cfg_if::cfg_if! {
            if #[cfg(feature = "trinamic-uart-multi-channel")] {
                uart.set_axis_channel(Some(channel));
            }
        }
        if let Ok(micro_steps_tmc_power_of_two) = to_tmc_micro_steps(configured_micro_steps) {
            match Self::init_stepper(
                uart, address, micro_steps_tmc_power_of_two
            ).await
            {
                Ok(_) => {
                    #[cfg(feature = "trace-commands")]
                    hwa::info!("[trace-commands] [TrinamicController] {} initialized", channel);
                    Ok(())},
                Err(_err) => {
                    hwa::error!("[TrinamicController] Unable to initialize stepper: {:?}: {:?}", channel, _err);
                    Err(())
                }
            }
        }
        else {
            hwa::error!("[TrinamicController]: Error initializing {:?} invalid micro-stepping setting ({})",
                channel,
                configured_micro_steps
            );
            Err(())
        }

    }

    async fn init_stepper(
        uart: &mut TrinamicUart,
        addr: u8,
        micro_steps_pow_of_2: u32,
    ) -> Result<(), TrinamicError> {
        #[cfg(feature = "trace-commands")]
        hwa::info!("[TrinamicController] applying gconf on {}", addr);

        let mut gconf = tmc2209::reg::GCONF::default();
        // Invert direction
        gconf.set_shaft(false);
        // Enable UART Comm
        gconf.set_pdn_disable(true);
        // Enable Spread Cycle: Higher Torque, more noise
        gconf.set_en_spread_cycle(true);
        // Enable multistepping
        gconf.set_mstep_reg_select(true);
        // Enable multistepping filtering
        gconf.set_multistep_filt(false);
        if Self::write_register(uart, addr, gconf).await.is_ok() {
            #[cfg(feature = "trace-commands")]
            hwa::debug!("[TrinamicController] applying chopconf on addr {}", addr);
            let mut chopconf = tmc2209::reg::CHOPCONF::default();
            chopconf.set_intpol(false);
            #[cfg(not(feature = "pulsed"))]
            chopconf.set_dedge(true);
            chopconf.set_mres(micro_steps_pow_of_2);
            if Self::write_register(uart, addr, chopconf).await.is_ok() {
                hwa::debug!("[TrinamicController] Write OK to addr {}", addr);
            }
        } else {
            hwa::error!("[TrinamicController] Error initializing stepper {}", addr);
        }

        Ok(())
    }

    #[inline]
    async fn write_register<T: tmc2209::WritableRegister>(
        uart: &mut TrinamicUart,
        slave_addr: u8,
        reg: T,
    ) -> Result<(), TrinamicError> {
        Self::raw_write(
            uart, tmc2209::WriteRequest::new(slave_addr, reg)
            .bytes()
        ).await
    }

    #[inline]
    #[allow(unused)]
    async fn read_register<T: tmc2209::ReadableRegister + core::fmt::Debug>(
        uart: &mut TrinamicUart,
        slave_addr: u8,
    ) -> Result<T, TrinamicError> {

        hwa::debug!("[TrinamicController] Send read request");
        Self::raw_write(
            uart,
            tmc2209::read_request::<T>(slave_addr).bytes(),
        ).await?;

        hwa::debug!("[TrinamicController] Now reading response...");
        let mut buff = [0u8; 32];
        let mut reader = tmc2209::Reader::default();
        let reception_timeout = Instant::now() + embassy_time::Duration::from_secs(5);
        loop {
            if Instant::now() > reception_timeout {
                hwa::debug!("TO Req");
                return Err(TrinamicError::Timeout);
            }
            match uart.read_until_idle(&mut buff).await {
                Ok(num_bytes_read) => {
                    if num_bytes_read > 0 {
                        hwa::debug!("[TrinamicController] Uart read {} bytes", num_bytes_read);
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
                    // Lost one frame.
                    // Continue until [reception_timeout]
                }
                Err(err) => {
                    hwa::error!("[TrinamicController] Read error reading trinamic: {:?}", err);
                    return Err(TrinamicError::ReadError);
                }
            }
        }
    }

    pub async fn raw_write(
        uart: &mut TrinamicUart,
        bytes: &[u8],
    ) -> Result<(), TrinamicError> {
        hwa::debug!("[TrinamicController] sending {:?}", bytes);
        uart
            .write(bytes)
            .await
            .map_err(|_| TrinamicError::WriteError)?;
        let _ = uart
            .blocking_flush()
            .map_err(|_| TrinamicError::WriteError)?;
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
