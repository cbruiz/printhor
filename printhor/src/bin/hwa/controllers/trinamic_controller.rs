//! TODO: This feature is still very experimental/preliminar
use crate::hwa;
use embassy_time::Instant;
#[allow(unused)]
use hwa::Contract;
use hwa::CoordSel;
use hwa::HwiContract;
use hwa::uart::SerialError;

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
    /// The drivers of these axes could not be configured (no answer, or the configuration read
    /// back does not match) after the retries
    Unconfigured(hwa::CoordSel),
}

/// The health of the drivers, as reported by [TrinamicController::check]. Each field holds the
/// axes in that condition.
#[derive(Debug, Clone, Copy)]
pub struct TrinamicHealth {
    /// Reset since configured (e.g. a supply dip), or charge pump undervoltage: the
    /// configuration is lost (they have to be configured again)
    pub reset: hwa::CoordSel,
    /// Shut down by a fault (over-temperature, short circuit)
    pub fault: hwa::CoordSel,
    /// No (valid) answer
    pub unanswered: hwa::CoordSel,
}

impl TrinamicHealth {
    /// Whether every driver is configured and healthy
    pub fn is_ok(&self) -> bool {
        self.reset.is_empty() && self.fault.is_empty() && self.unanswered.is_empty()
    }
}

/// The axes having a Trinamic driver
const TMC_AXES: &[CoordSel] = &[
    #[cfg(feature = "with-x-axis")]
    CoordSel::X,
    #[cfg(feature = "with-y-axis")]
    CoordSel::Y,
    #[cfg(feature = "with-z-axis")]
    CoordSel::Z,
    #[cfg(feature = "with-e-axis")]
    CoordSel::E,
];

/// GSTAT flags (reset, drv_err, uv_cp): written as 1 to clear them
const GSTAT_CLEAR_ALL: u32 = 0b111;

/// Attempts to configure each driver: a single UART failure (noise, a driver still powering up)
/// is recovered by writing the configuration again
const INIT_ATTEMPTS: u8 = 3;

/// Whether the configuration of each driver is read back to verify it (a write is not
/// acknowledged). Overridable at build time, for a board whose single-wire UART can not read
#[const_env::env_item("TMC_INIT_VERIFY")]
const TMC_INIT_VERIFY: bool = true;

/// Delay per step of the current reduction to the standstill current (`IHOLDDELAY`, in 2^18
/// clocks, ~22 ms each), as Marlin. Only for the drivers with a run current
/// ([Contract::TRINAMIC_RUN_CURRENT_MA])
const TMC_IHOLD_DELAY: u8 = 10;

/// Delay from the standstill detection to the current reduction (`TPOWERDOWN`, in 2^18 clocks:
/// ~2.8 s), as Marlin. Only for the drivers with a run current
const TMC_TPOWERDOWN: u8 = 128;

/// The current of a driver set through the UART: run and standstill current scales (`CS`, 0 to
/// 31) for the sense voltage range `vsense`
#[derive(Debug, Clone, Copy, PartialEq)]
struct CurrentScale {
    vsense: bool,
    irun: u8,
    ihold: u8,
}

impl CurrentScale {
    /// The current scale of a RMS run current (mA), as Marlin (TMCStepper `rms_current`): the
    /// high sensitivity range (`vsense`) when the scale would be below 16 in the low one, the
    /// standstill current the run scale times `hold_multiplier` (truncated)
    fn from_rms_ma(rms_ma: u16, rsense_ohms: f32, hold_multiplier: f32) -> Self {
        let (vsense, cs) = tmc2209::rms_current_to_vsense_cs(rsense_ohms, rms_ma as f32);
        let irun = cs.min(31);
        let ihold = ((irun as f32 * hold_multiplier) as u8).min(31);
        Self { vsense, irun, ihold }
    }

    /// The current scale of the driver of `axis`, when it has a run current
    fn of_axis(axis: CoordSel) -> Option<Self> {
        Contract::TRINAMIC_RUN_CURRENT_MA
            .get_coord(axis)
            .filter(|ma| *ma > 0)
            .map(|ma| Self::from_rms_ma(ma, Contract::TRINAMIC_RSENSE_OHMS, Contract::TRINAMIC_HOLD_MULTIPLIER))
    }
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

        let mut failed = hwa::CoordSel::empty();

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

            if Self::init_channel(
                &mut self.uart,
                CoordSel::X,
                address,
                micro_steps_setting.x.ok_or(TrinamicError::BadSettings)?,
            )
            .await
            .is_err()
            {
                failed.set(CoordSel::X, true);
            }
        }

        #[cfg(feature = "with-y-axis")]
        {
            use hwa::traits::TrinamicUartTrait;
            let address = self
                .uart
                .get_tmc_address(CoordSel::Y)
                .map_err(|_| TrinamicError::BadSettings)?;

            if Self::init_channel(
                &mut self.uart,
                CoordSel::Y,
                address,
                micro_steps_setting.y.ok_or(TrinamicError::BadSettings)?,
            )
            .await
            .is_err()
            {
                failed.set(CoordSel::Y, true);
            }
        }

        #[cfg(feature = "with-z-axis")]
        {
            use hwa::traits::TrinamicUartTrait;
            let address = self
                .uart
                .get_tmc_address(CoordSel::Z)
                .map_err(|_| TrinamicError::BadSettings)?;

            if Self::init_channel(
                &mut self.uart,
                CoordSel::Z,
                address,
                micro_steps_setting.z.ok_or(TrinamicError::BadSettings)?,
            )
            .await
            .is_err()
            {
                failed.set(CoordSel::Z, true);
            }
        }

        #[cfg(feature = "with-e-axis")]
        {
            use hwa::traits::TrinamicUartTrait;
            let address = self
                .uart
                .get_tmc_address(CoordSel::E)
                .map_err(|_| TrinamicError::BadSettings)?;

            if Self::init_channel(
                &mut self.uart,
                CoordSel::E,
                address,
                micro_steps_setting.e.ok_or(TrinamicError::BadSettings)?,
            )
            .await
            .is_err()
            {
                failed.set(CoordSel::E, true);
            }
        }

        if !failed.is_empty() {
            hwa::debug!("[TrinamicController] Drivers not configured: {:?}", failed);
            Err(TrinamicError::Unconfigured(failed))
        } else {
            #[cfg(feature = "trace-commands")]
            hwa::debug!("[TrinamicController] returning OK");
            Ok(())
        }
    }

    /// Whether the drivers can be read ([TrinamicController::check] is possible). Not when the
    /// read back is disabled (`TMC_INIT_VERIFY`: a single-wire UART that can not read).
    pub const fn can_read() -> bool {
        TMC_INIT_VERIFY
    }

    /// The axes having a driver
    pub const fn axes() -> &'static [CoordSel] {
        TMC_AXES
    }

    /// Checks the health of the drivers of `axes` (as Klipper does periodically): the global
    /// status (GSTAT) tells whether the driver has been reset since it was configured (the
    /// configuration is lost) or shut down by a fault. Only meaningful when
    /// [TrinamicController::can_read].
    ///
    /// The cost is a register read per driver (two on a fault): on a bit-banged UART at a low
    /// baud rate (e.g. the MKS Robin Nano, 1200 baud) about 100 ms each, so the caller should
    /// check one driver at a time. Each read is bounded (see `read_register`), but the caller
    /// should bound the whole check as well: a UART read may never complete.
    pub async fn check(&mut self, axes: CoordSel) -> TrinamicHealth {
        use hwa::traits::TrinamicUartTrait;
        let mut health = TrinamicHealth {
            reset: CoordSel::empty(),
            fault: CoordSel::empty(),
            unanswered: CoordSel::empty(),
        };
        for axis in TMC_AXES.iter().copied().filter(|axis| axes.contains(*axis)) {
            let address = match self.uart.get_tmc_address(axis) {
                Ok(address) => address,
                Err(_) => {
                    health.unanswered.set(axis, true);
                    continue;
                }
            };
            if self.uart.select_stepper_of_axis(axis).is_err() {
                health.unanswered.set(axis, true);
                continue;
            }
            match Self::read_register::<tmc2209::reg::GSTAT>(&mut self.uart, address).await {
                Ok(gstat) => {
                    if gstat.reset() || gstat.uv_cp() {
                        hwa::warn!(
                            "[TrinamicController] {:?} driver reset (reset: {}, uv_cp: {})",
                            axis,
                            gstat.reset(),
                            gstat.uv_cp()
                        );
                        health.reset.set(axis, true);
                    }
                    if gstat.drv_err() {
                        health.fault.set(axis, true);
                        match Self::read_register::<tmc2209::reg::DRV_STATUS>(&mut self.uart, address).await {
                            Ok(_drv) => hwa::error!(
                                "[TrinamicController] {:?} driver fault: ot: {} s2ga: {} s2gb: {} s2vsa: {} s2vsb: {}",
                                axis,
                                _drv.ot(),
                                _drv.s2ga(),
                                _drv.s2gb(),
                                _drv.s2vsa(),
                                _drv.s2vsb()
                            ),
                            Err(_) => hwa::error!("[TrinamicController] {:?} driver fault", axis),
                        }
                    }
                }
                Err(_) => {
                    hwa::warn!("[TrinamicController] {:?} driver not answering", axis);
                    health.unanswered.set(axis, true);
                }
            }
            let _ = self.uart.select_stepper_of_axis(CoordSel::empty());
        }
        health
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
            let current = CurrentScale::of_axis(channel);
            if let Some(_c) = current {
                hwa::debug!(
                    "[TrinamicController] {:?}: run current {} mA RMS (IRUN {}, IHOLD {}, vsense {})",
                    channel,
                    Contract::TRINAMIC_RUN_CURRENT_MA.get_coord(channel).unwrap_or(0),
                    _c.irun,
                    _c.ihold,
                    _c.vsense
                );
            }
            let mut result = Err(TrinamicError::Timeout);
            for attempt in 1..=INIT_ATTEMPTS {
                result = Self::init_stepper(uart, address, micro_steps_tmc_power_of_two, current).await;
                if result.is_ok() {
                    break;
                }
                hwa::debug!(
                    "[TrinamicController] {:?} not configured (attempt {} of {})",
                    channel,
                    attempt,
                    INIT_ATTEMPTS
                );
                embassy_time::Timer::after_millis(50).await;
            }
            match result {
                Ok(_) => {
                    #[cfg(feature = "trace-commands")]
                    hwa::info!(
                        "[trace-commands] [TrinamicController] {:?} initialized",
                        channel
                    );
                    let _ = uart.select_stepper_of_axis(CoordSel::empty());
                    Ok(())
                }
                Err(_err) => {
                    hwa::debug!(
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
        current: Option<CurrentScale>,
    ) -> Result<(), TrinamicError> {
        // The current registers are written before GCONF: GCONF switches to the internal current
        // reference, where the power-on IRUN (31, full scale) would drive ~1.8 A RMS (0.11 Ω)
        // until IHOLD_IRUN was written
        #[cfg(feature = "trace-commands")]
        hwa::info!("[TrinamicController] applying chopconf on addr {}", addr);
        let mut chop_conf = tmc2209::reg::CHOPCONF::default();
        chop_conf.set_intpol(false);
        #[cfg(not(feature = "pulsed"))]
        chop_conf.set_dedge(true);
        chop_conf.set_mres(micro_steps_pow_of_2.into());
        if let Some(c) = current {
            chop_conf.set_vsense(c.vsense);
        }
        Self::write_register(uart, addr, chop_conf).await?;

        if let Some(c) = current {
            #[cfg(feature = "trace-commands")]
            hwa::info!("[TrinamicController] applying ihold_irun on addr {}", addr);
            let mut ihold_irun = tmc2209::reg::IHOLD_IRUN::default();
            ihold_irun.set_irun(c.irun);
            ihold_irun.set_ihold(c.ihold);
            ihold_irun.set_ihold_delay(TMC_IHOLD_DELAY);
            Self::write_register(uart, addr, ihold_irun).await?;
            let mut power_down = tmc2209::reg::TPOWERDOWN::default();
            power_down.set(TMC_TPOWERDOWN);
            Self::write_register(uart, addr, power_down).await?;
        }

        #[cfg(feature = "trace-commands")]
        hwa::info!("[TrinamicController] applying gconf on {}", addr);
        let mut g_conf = tmc2209::reg::GCONF::default();
        // Current reference: internal when the run current is set through the UART, otherwise
        // the voltage of the VREF pin (analog scaling, the power-on setting)
        g_conf.set_i_scale_analog(current.is_none());
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
        Self::write_register(uart, addr, g_conf).await?;
        // Clear the latched status flags (reset, drv_err): from now on, a reset reported by GSTAT
        // means the configuration is lost (see [TrinamicController::check])
        Self::write_register(uart, addr, tmc2209::reg::GSTAT::from(GSTAT_CLEAR_ALL)).await?;

        // The configuration is only trusted once read back: a write is not acknowledged (a driver
        // without VM, or a noisy line, silently drops it)
        if !TMC_INIT_VERIFY {
            return Ok(());
        }
        let chop_conf = Self::read_register::<tmc2209::reg::CHOPCONF>(uart, addr).await?;
        #[allow(unused_mut)]
        let mut matches = chop_conf.intpol() == false
            && u32::from(chop_conf.mres()) == micro_steps_pow_of_2
            && current.is_none_or(|c| chop_conf.vsense() == c.vsense);
        #[cfg(not(feature = "pulsed"))]
        {
            matches &= chop_conf.dedge();
        }
        if matches {
            #[cfg(feature = "trace-commands")]
            hwa::info!("[TrinamicController] Trinamic check for stepper {} is OK", addr);
            Ok(())
        } else {
            hwa::warn!("[TrinamicController] Trinamic check for stepper {} is BAD", addr);
            Err(TrinamicError::BadSettings)
        }
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
        #[cfg(feature = "trace-commands")]
        hwa::debug!("[TrinamicController] Send read request");
        Self::raw_write(uart, tmc2209::read_request::<T>(slave_addr).bytes()).await?;

        #[cfg(feature = "trace-commands")]
        hwa::debug!("[TrinamicController] Now reading response...");
        let mut buff = [0u8; 32];
        let mut reader = tmc2209::Reader::default();
        let reception_deadline = Instant::now() + embassy_time::Duration::from_secs(1);
        loop {
            if Instant::now() > reception_deadline {
                hwa::debug!("[TrinamicController] Reception deadline expired!");
                return Err(TrinamicError::Timeout);
            }
            use hwa::traits::TrinamicUartTrait;
            match uart.read_until_idle(&mut buff).await {
                Ok(num_bytes_read) => {
                    if num_bytes_read > 0 {
                        #[cfg(feature = "trace-commands")]
                        hwa::debug!("[TrinamicController] UART read {} bytes", num_bytes_read);
                        match reader.read_response(&buff[0..num_bytes_read]) {
                            (_n, Some(response)) => {
                                return match response.register::<T>() {
                                    Ok(r) => {
                                        // Heap-free: a register Debug can be long
                                        #[cfg(feature = "native")]
                                        hwa::debug!("response[0]. l={} : {:?}", _n, r);
                                        Ok(r)
                                    }
                                    _ => Err(TrinamicError::ReadError),
                                };
                            }
                            (n, None) => {
                                use core::fmt::Write;
                                let mut x = crate::processing::LineBuf::<48>::new();
                                let _ = write!(x, "{:?}", reader.awaiting());
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
        #[cfg(feature = "trace-commands")]
        hwa::debug!("[TrinamicController] sending {:?}", bytes);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn current_scale_as_marlin() {
        // Marlin (TMCStepper `rms_current`) with R_sense 0.11 Ω and HOLD_MULTIPLIER 0.5: the low
        // currents in the high sensitivity range (CS < 16 in the low one)
        let scale = |ma| CurrentScale::from_rms_ma(ma, 0.11, 0.5);
        assert_eq!(scale(580), CurrentScale { vsense: true, irun: 17, ihold: 8 });
        assert_eq!(scale(800), CurrentScale { vsense: true, irun: 25, ihold: 12 });
        assert_eq!(scale(1200), CurrentScale { vsense: false, irun: 20, ihold: 10 });
        // Capped to the full scale
        assert_eq!(scale(3000).irun, 31);
    }
}
