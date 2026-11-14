//! This module defines the G-code processing logic for a 3D printer control system.
//!
//! The purpose of this module is to handle various aspects of G-code interpretation
//! and execution, including communication with hardware controllers and asynchronous
//! message handling. The module includes structures and functions that facilitate
//! the execution of G-codes on the hardware components of the 3D printer.
//!
//! # Structures
//!
//! - `GCodeProcessorParams`: Contains configuration parameters and references to
//!   various hardware controllers needed for G-code processing.
//! - `GCodeProcessor`: The main G-code processor structure that includes methods for
//!   writing responses to communication channels and handling G-code execution.
//!
//! # Features
//!
//! The module supports conditional compilation based on feature flags. The following
//! feature flags are used:
//!
//! - `with-motion`: Enables motion planning features.
//! - `with-serial-usb`: Enables serial USB communication.
//! - `with-serial-port-1`, `with-serial-port-2`: Enable serial port communication for two ports.
//! - `with-ps-on`: Enables PSU control.
//! - `with-probe`: Enables probe hardware support.
//! - `with-hot-end`, `with-hot-bed`: Enable hotend and hotbed control respectively.
//! - `with-fan-layer`, `with-fan-extra-1`: Enable additional fan control.
//! - `with-laser`: Enables laser PWM control.
//!
//! # Usage
//!
//! To use the GCodeProcessor, create an instance of `GCodeProcessorParams` with the
//! needed references and initialize `GCodeProcessor` using the `new` method. Utilize
//! the provided async methods to write to communication channels and handle G-code commands.
//!
//! # Remark
//! Do not **format** on f32 directly and use [math::Real] instead to avoid large code generation.
//!
//! Reasoning: core::fmt::float::impl_general_format macro expansion will take around:
//! - 7.6KiB by core::fmt::float::float_to_decimal_common_shortest
//! - 6.3KiB by core::fmt::float::float_to_decimal_common_exact
//!
//! Around 14KB in total
//!
//! Actually, 5.1KiB only using Debug format trait and/or defmt fmt trait with lexical_core/ryu.

use crate::hwa;

use crate::processing::{
    CodeExecutionFailure, CodeExecutionResult, CodeExecutionSuccess, GCodeCmd, GCodeValue,
};
use hwa::Contract;
#[cfg(feature = "with-probe")]
use hwa::controllers::ProbeTrait;
use hwa::math;
#[allow(unused)]
use hwa::{AsyncMutexStrategy, HwiContract, SyncMutexStrategy};
#[allow(unused)]
use hwa::{CommChannel, DeferAction, DeferEvent, EventFlags, EventStatus};
use strum::VariantNames;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-serial-port-1", feature="with-serial-port-2"))]
    {
        #[allow(unused)]
        use embedded_io_async::Write;
    }
}

/// Min hot-end temperature to allow extrusion, as `f32` bits (M302 S)
#[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
static COLD_EXTRUSION_MIN_TEMP: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(<Contract as HwiContract>::EXTRUDE_MIN_TEMP.to_bits());

/// Power [0-100] of the extra fan requested with `M106 P1` (the hot-end auto fan may override it)
#[cfg(feature = "with-fan-extra-1")]
pub static FAN_EXTRA1_MANUAL_POWER: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

/// Whether cold extrusion is allowed (M302 P1)
#[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
static COLD_EXTRUSION_ALLOWED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// The cold extrusion settings: (min temp, allowed)
#[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
#[allow(unused)]
pub(crate) fn cold_extrusion_settings() -> (f32, bool) {
    (
        f32::from_bits(COLD_EXTRUSION_MIN_TEMP.load(core::sync::atomic::Ordering::Relaxed)),
        COLD_EXTRUSION_ALLOWED.load(core::sync::atomic::Ordering::Relaxed),
    )
}

/// Updates the cold extrusion settings (the given ones)
#[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
#[allow(unused)]
pub(crate) fn set_cold_extrusion_settings(min_temp: Option<f32>, allowed: Option<bool>) {
    if let Some(t) = min_temp {
        COLD_EXTRUSION_MIN_TEMP.store(t.max(0.0).to_bits(), core::sync::atomic::Ordering::Relaxed);
    }
    if let Some(a) = allowed {
        COLD_EXTRUSION_ALLOWED.store(a, core::sync::atomic::Ordering::Relaxed);
    }
}

/// Formats the set coordinates of a vector as GCode words (`X80.00 Y80.00 ...`)
#[cfg(feature = "with-motion")]
fn axis_words(v: &hwa::math::TVector<math::Real>) -> alloc::string::String {
    let mut out = alloc::string::String::new();
    v.foreach_values(|coord, value| {
        if !out.is_empty() {
            out.push(' ');
        }
        out.push_str(alloc::format!("{}{:?}", coord.name(), value.rdp(2)).as_str());
    });
    out
}

#[derive(Clone)]
pub struct GCodeProcessor {
    pub event_bus: hwa::types::EventBus,

    #[cfg(feature = "with-motion")]
    pub motion_planner: hwa::controllers::MotionPlanner,

    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: hwa::types::SerialUsbTxController,

    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: hwa::types::SerialPort1TxController,

    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: hwa::types::SerialPort2TxController,

    #[cfg(feature = "with-ps-on")]
    pub ps_on: hwa::types::PSOnController,

    #[cfg(feature = "with-probe")]
    pub probe: hwa::types::ProbeController,

    #[cfg(feature = "with-hot-end")]
    pub hot_end: hwa::types::HotEndController,

    #[cfg(feature = "with-hot-bed")]
    pub hot_bed: hwa::types::HotBedController,

    #[cfg(feature = "with-fan-layer")]
    pub fan_layer: hwa::types::FanLayerController,

    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra1: hwa::types::FanExtra1Controller,

    #[cfg(feature = "with-laser")]
    pub _laser: hwa::types::LaserController,

    #[cfg(feature = "with-eeprom")]
    pub eeprom: hwa::types::EepromController,
}

impl GCodeProcessor {
    pub const fn new(
        event_bus: hwa::types::EventBus,
        #[cfg(feature = "with-serial-usb")] serial_usb_tx: hwa::types::SerialUsbTxController,
        #[cfg(feature = "with-serial-port-1")] serial_port1_tx: hwa::types::SerialPort1TxController,
        #[cfg(feature = "with-serial-port-2")] serial_port2_tx: hwa::types::SerialPort2TxController,
        #[cfg(feature = "with-motion")] motion_planner: hwa::controllers::MotionPlanner,
        #[cfg(feature = "with-ps-on")] ps_on: hwa::types::PSOnController,

        #[cfg(feature = "with-probe")] probe: hwa::types::ProbeController,

        #[cfg(feature = "with-hot-end")] hot_end: hwa::types::HotEndController,
        #[cfg(feature = "with-hot-bed")] hot_bed: hwa::types::HotBedController,

        #[cfg(feature = "with-fan-layer")] fan_layer: hwa::types::FanLayerController,

        #[cfg(feature = "with-fan-extra-1")] fan_extra1: hwa::types::FanExtra1Controller,

        #[cfg(feature = "with-laser")] _laser: hwa::types::LaserController,

        #[cfg(feature = "with-eeprom")] eeprom: hwa::types::EepromController,
    ) -> Self {
        Self {
            event_bus,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_tx,
            #[cfg(feature = "with-ps-on")]
            ps_on,
            #[cfg(feature = "with-motion")]
            motion_planner,
            #[cfg(feature = "with-probe")]
            probe,
            #[cfg(feature = "with-hot-end")]
            hot_end,
            #[cfg(feature = "with-hot-bed")]
            hot_bed,
            #[cfg(feature = "with-fan-layer")]
            fan_layer,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1,
            #[cfg(feature = "with-laser")]
            _laser,
            #[cfg(feature = "with-eeprom")]
            eeprom,
        }
    }

    /// Writes one M503 section right away, so that the report never needs a big heap buffer
    async fn write_section(
        &self,
        channel: CommChannel,
        detailed: bool,
        title: &str,
        line: alloc::string::String,
    ) {
        if detailed {
            self.write(channel, "echo:; ").await;
            self.write(channel, title).await;
            self.write(channel, "\n").await;
        }
        self.write(channel, "echo:  ").await;
        self.write(channel, line.as_str()).await;
        self.write(channel, "\n").await;
    }

    pub async fn write_ok(&self, channel: CommChannel) {
        self.write(channel, "ok\n").await;
    }

    pub async fn write(&self, channel: CommChannel, _msg: &str) {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            CommChannel::SerialUsb => {
                // A USB packet can not exceed the endpoint max packet size (the driver rejects it
                // with BufferOverflow), so the message is split. A trailing full-size packet is
                // followed by a zero-length one so that the host delivers the data right away.
                let mut tx = self.serial_usb_tx.lock().await;
                // Full-speed USB bulk endpoints use 64-byte packets
                const MAX: usize = 64;
                let bytes = _msg.as_bytes();
                for chunk in bytes.chunks(MAX) {
                    let _ = tx.write_packet(chunk).await;
                }
                if !bytes.is_empty() && bytes.len() % MAX == 0 {
                    let _ = tx.write_packet(&[]).await;
                }
            }
            #[cfg(feature = "with-serial-port-1")]
            CommChannel::SerialPort1 => {
                #[allow(unused)]
                use hwa::AsyncWrapperWriter;
                let mut mg = self.serial_port1_tx.lock().await;
                let _ = mg.wrapped_write(_msg.as_bytes()).await;
                mg.wrapped_flush().await;
            }
            #[cfg(feature = "with-serial-port-2")]
            CommChannel::SerialPort2 => {
                #[allow(unused)]
                use hwa::AsyncWrapperWriter;
                let mut mg = self.serial_port2_tx.lock().await;
                let _ = mg.wrapped_write(_msg.as_bytes()).await;
                mg.wrapped_flush().await;
            }
            CommChannel::Sink => {}
            CommChannel::Internal => {
                cfg_if::cfg_if! {
                    if #[cfg(feature="native")] {
                        use std::io::Write;
                        // In native backend, we will use stdout as convention for internal output
                        std::print!("<< {}", _msg);
                        if std::io::stdout().flush().is_err() {
                            hwa::error!("[Internal] Could not flush (unexpectedly)");
                        }
                    }
                    else {
                        // A message shall be always end by \n, so we need to trim it for pretty printing
                        hwa::info!("[Internal] {}", _msg.trim_end());
                    }
                }
            }
        }
    }

    #[allow(unused)]
    pub async fn flush(&self, channel: CommChannel) {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            CommChannel::SerialUsb => {
                //let _ = self.serial_usb_tx.lock().await.write_packet(b"").await;
            }
            #[cfg(feature = "with-serial-port-1")]
            CommChannel::SerialPort1 => {
                use hwa::AsyncWrapperWriter;
                let mut mg = self.serial_port1_tx.lock().await;
                mg.wrapped_flush().await;
            }
            #[cfg(feature = "with-serial-port-2")]
            CommChannel::SerialPort2 => {
                use hwa::AsyncWrapperWriter;
                let mut mg = self.serial_port2_tx.lock().await;
                mg.wrapped_flush().await;
            }
            CommChannel::Internal => {}
            CommChannel::Sink => {}
        }
    }

    /// Writes the message to every host channel.
    ///
    /// Each write is bounded in time: it is used by safety tasks (temperature control), which must
    /// never block because of a host not reading (e.g. USB connected without a host).
    pub async fn write_all(&self, _msg: &str) {
        let timeout = embassy_time::Duration::from_millis(250);
        #[cfg(feature = "with-serial-usb")]
        let _ = embassy_time::with_timeout(timeout, self.write(CommChannel::SerialUsb, _msg)).await;
        #[cfg(feature = "with-serial-port-1")]
        let _ =
            embassy_time::with_timeout(timeout, self.write(CommChannel::SerialPort1, _msg)).await;
        #[cfg(feature = "with-serial-port-2")]
        let _ =
            embassy_time::with_timeout(timeout, self.write(CommChannel::SerialPort2, _msg)).await;
        let _ = embassy_time::with_timeout(timeout, self.write(CommChannel::Internal, _msg)).await;
    }

    /// Fails with [CodeExecutionFailure::PowerRequired] when the ATX power is off
    #[allow(unused)]
    async fn require_power(&self) -> Result<(), CodeExecutionFailure> {
        #[cfg(feature = "with-ps-on")]
        if !self
            .event_bus
            .get_status()
            .await
            .contains(EventFlags::ATX_ON)
        {
            return Err(CodeExecutionFailure::PowerRequired);
        }
        Ok(())
    }

    /// Waits until every queued move has been executed
    #[cfg(feature = "with-motion")]
    pub async fn wait_for_motion_idle(&self) {
        loop {
            let status = self.event_bus.get_status().await;
            if status.contains(EventFlags::MOV_QUEUE_EMPTY) && !status.contains(EventFlags::MOVING)
            {
                break;
            }
            embassy_time::Timer::after_millis(10).await;
        }
    }

    /// Discards every queued move and interrupts the ongoing one (M410, M112).
    ///
    /// The planned position is synchronized with the real one afterward.
    #[cfg(feature = "with-motion")]
    pub async fn quick_stop(&self) {
        crate::tasks::task_stepper::abort_motion(true);
        let _ = embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            self.wait_for_motion_idle(),
        )
        .await;
        let current = self.motion_planner.motion_status().get_current_position();
        self.motion_planner
            .motion_status()
            .update_last_planned_position(0, &current);
        crate::tasks::task_stepper::abort_motion(false);
    }

    /// Turns every heater off (target = 0)
    pub async fn shutdown_heaters(&self) {
        #[cfg(feature = "with-hot-end")]
        self.hot_end.lock().await.shutdown();
        #[cfg(feature = "with-hot-bed")]
        self.hot_bed.lock().await.shutdown();
    }

    /// Halts the machine: raises SYS_ALARM, shuts the heaters down, stops the motion and disables
    /// the steppers. The position is lost, so homing is required after `M999`.
    pub async fn emergency_stop(&self, reason: &str) {
        hwa::warn!("[GCodeProcessor] Emergency stop: {}", reason);
        self.event_bus
            .publish_event(EventStatus::containing(EventFlags::SYS_ALARM))
            .await;
        self.shutdown_heaters().await;
        #[cfg(feature = "with-fan-layer")]
        self.fan_layer.lock().await.set_power(0);
        #[cfg(feature = "with-motion")]
        {
            self.quick_stop().await;
            self.motion_planner
                .motion_driver()
                .lock()
                .await
                .step_actuator()
                .disable_steppers(hwa::CoordSel::all_axis());
            // The next move must enable its drivers again
            hwa::controllers::motion_control::STEP_DRIVER.reset();
            self.motion_planner.motion_status().invalidate_position();
        }
        self.write_all(alloc::format!("Error:{}\nError:Printer halted. kill() called!\n", reason).as_str())
            .await;
    }

    /// Whether the command sets or waits for a temperature
    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    fn is_temperature_command(value: &GCodeValue) -> bool {
        match value {
            #[cfg(feature = "with-hot-end")]
            GCodeValue::M104(_) | GCodeValue::M109(_) => true,
            #[cfg(feature = "with-hot-bed")]
            GCodeValue::M140(_) | GCodeValue::M190(_) => true,
            GCodeValue::M303(_) => true,
            _ => false,
        }
    }

    /// Whether the command can be executed while the machine is halted (SYS_ALARM)
    fn allowed_while_halted(value: &GCodeValue) -> bool {
        match value {
            GCodeValue::Nop
            | GCodeValue::G
            | GCodeValue::M
            | GCodeValue::M100
            | GCodeValue::M105
            | GCodeValue::M110(_)
            | GCodeValue::M112
            | GCodeValue::M115
            | GCodeValue::M117(_)
            | GCodeValue::M118(_)
            | GCodeValue::M503(_)
            | GCodeValue::M79
            | GCodeValue::M999 => true,
            #[cfg(feature = "with-motion")]
            GCodeValue::M114 | GCodeValue::M119 => true,
            #[cfg(feature = "with-ps-on")]
            GCodeValue::M81 => true,
            _ => false,
        }
    }

    /// Prevents extrusion when the hot-end is too cold (see M302).
    ///
    /// Returns the command to execute instead: the same one with the extruder words removed when
    /// the extrusion is prevented. The logical extruder position is updated as if the extrusion
    /// took place, so that later absolute E moves do not try to recover the skipped amount.
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    async fn prevent_cold_extrusion(&self, channel: CommChannel, gc: &GCodeCmd) -> Option<GCodeCmd> {
        let min_temp = self.cold_extrusion_min_temp();
        let e = match &gc.value {
            GCodeValue::G1(t) => t.e,
            #[cfg(feature = "with-arcs")]
            GCodeValue::G2(arc) | GCodeValue::G3(arc) => arc.coords.e,
            #[cfg(feature = "with-beziers")]
            GCodeValue::G5(bz) => bz.coords.e,
            GCodeValue::G11 => Some(math::ONE),
            _ => None,
        };
        let e = e?;
        let min_temp = min_temp?;
        if self.event_bus.has_flags(EventFlags::DRY_RUN).await {
            return None;
        }
        let current = self.hot_end.lock().await.get_current_temp();
        if current >= min_temp {
            return None;
        }
        let status = self.motion_planner.motion_status();
        if !matches!(gc.value, GCodeValue::G11) {
            let target = match status.is_e_relative() {
                true => status.get_logical_e() + e,
                false => e,
            };
            status.set_logical_e(target);
        }
        let _ = self
            .write(channel, "echo: cold extrusion prevented\n")
            .await;
        let mut cmd = GCodeCmd::new(gc.order_num, gc.line_tag, gc.value.clone());
        match &mut cmd.value {
            GCodeValue::G1(t) => t.e = None,
            #[cfg(feature = "with-arcs")]
            GCodeValue::G2(arc) | GCodeValue::G3(arc) => arc.coords.e = None,
            #[cfg(feature = "with-beziers")]
            GCodeValue::G5(bz) => bz.coords.e = None,
            _ => cmd.value = GCodeValue::Nop,
        }
        Some(cmd)
    }

    /// The min hot-end temperature to extrude, or `None` when cold extrusion is allowed
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    fn cold_extrusion_min_temp(&self) -> Option<f32> {
        match COLD_EXTRUSION_ALLOWED.load(core::sync::atomic::Ordering::Relaxed) {
            true => None,
            false => Some(f32::from_bits(
                COLD_EXTRUSION_MIN_TEMP.load(core::sync::atomic::Ordering::Relaxed),
            )),
        }
    }

    /// Writes an error report when a heater rejects a target temperature
    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    async fn report_heater_error(
        &self,
        channel: CommChannel,
        err: hwa::controllers::HeaterError,
        max: f32,
    ) -> CodeExecutionFailure {
        let msg = match err {
            hwa::controllers::HeaterError::Faulted(f) => alloc::format!(
                "echo: Heater in fault state ({}). Send M999 to clear\n",
                f.description()
            ),
            hwa::controllers::HeaterError::TargetTooHigh => alloc::format!(
                "echo: Target temperature too high (max: {:?})\n",
                math::Real::from_f32(max)
            ),
        };
        self.write(channel, msg.as_str()).await;
        CodeExecutionFailure::ERR
    }

    /// Executes the given GCode command.
    ///
    /// # Arguments
    ///
    /// * `channel` - The communication channel to use for the execution.
    /// * `gc` - The GCode command to execute.
    /// * `_blocking` - A boolean indicating whether the execution should be blocking or not.
    ///
    /// # Returns
    ///
    /// * `CodeExecutionResult` - The result of the GCode command execution.
    ///
    /// # Errors
    ///
    /// This method may return `CodeExecutionFailure::PowerRequired` if the ATX power is not on
    /// and the command requires power. It may also return `CodeExecutionFailure::BUSY` if the
    /// system is currently busy, for instance during homing or probing.
    ///
    /// # Features
    ///
    /// This method supports conditional compilation flags such as `grbl-compat`, `with-motion`,
    /// and `with-probe`. Based on these features, the behavior and supported GCode commands
    /// will vary.
    ///
    /// # Async
    ///
    /// This is an asynchronous method and should be awaited.
    pub async fn execute(
        &mut self,
        channel: CommChannel,
        gc: &GCodeCmd,
        _blocking: bool,
    ) -> CodeExecutionResult {
        if !Self::allowed_while_halted(&gc.value)
            && self.event_bus.has_flags(EventFlags::SYS_ALARM).await
        {
            let _ = self
                .write(channel, "echo: Printer halted. Send M999 to resume\n")
                .await;
            return Err(CodeExecutionFailure::ERR);
        }
        // Dry run (M37): temperatures are ignored (as in Marlin)
        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        if Self::is_temperature_command(&gc.value)
            && self.event_bus.has_flags(EventFlags::DRY_RUN).await
        {
            hwa::debug!("DRY_RUN: Ignoring temperature command");
            return Ok(CodeExecutionSuccess::OK);
        }
        #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
        let replacement = self.prevent_cold_extrusion(channel, gc).await;
        #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
        let gc = replacement.as_ref().unwrap_or(gc);
        let result = match &gc.value {
            #[cfg(feature = "grbl-compat")]
            GCodeValue::GRBLCmd => {
                let str = format!(
                    "[VER:1.1 {} v{}:]\n[MSG: Machine: {} {}]\n",
                    Contract::FIRMWARE_NAME,
                    Contract::FIRMWARE_VERSION,
                    Contract::MACHINE_BOARD,
                    Contract::MACHINE_TYPE,
                );
                self.write(channel, str.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::G => {
                for x in GCodeValue::VARIANTS.iter().filter(|x| x.starts_with("G")) {
                    let _ = self
                        .write(channel, alloc::format!("echo: {}\n", x).as_str())
                        .await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G0(_) | GCodeValue::G1(_) => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                Ok(self
                    .motion_planner
                    .plan(channel, &gc, _blocking, &self.event_bus)
                    .await?)
            }
            #[cfg(feature = "with-arcs")]
            GCodeValue::G2(_) | GCodeValue::G3(_) => {
                self.require_power().await?;
                Ok(self
                    .motion_planner
                    .plan(channel, &gc, _blocking, &self.event_bus)
                    .await?)
            }
            #[cfg(feature = "with-beziers")]
            GCodeValue::G5(_) => {
                self.require_power().await?;
                Ok(self
                    .motion_planner
                    .plan(channel, &gc, _blocking, &self.event_bus)
                    .await?)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G4(_) => Ok(self
                .motion_planner
                .plan(channel, &gc, _blocking, &self.event_bus)
                .await?),

            #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
            GCodeValue::G10 | GCodeValue::G11 => {
                self.require_power().await?;
                Ok(self
                    .motion_planner
                    .plan(channel, &gc, _blocking, &self.event_bus)
                    .await?)
            }
            #[cfg(all(feature = "with-motion", not(feature = "with-e-axis")))]
            GCodeValue::G10 | GCodeValue::G11 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G17 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G21 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G28(_) => match self.event_bus.has_flags(EventFlags::HOMING).await {
                true => Err(CodeExecutionFailure::BUSY),
                false => {
                    #[cfg(feature = "with-ps-on")]
                    if !self
                        .event_bus
                        .get_status()
                        .await
                        .contains(EventFlags::ATX_ON)
                    {
                        return Err(CodeExecutionFailure::PowerRequired);
                    }
                    hwa::debug!("Planing homing");
                    let result = self
                        .motion_planner
                        .plan(channel, &gc, _blocking, &self.event_bus)
                        .await;
                    hwa::debug!("Homing planned");
                    result
                }
            },
            #[cfg(feature = "with-motion")]
            GCodeValue::G29 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G29_1 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G29_2 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-probe")]
            GCodeValue::G31 => {
                if self.event_bus.has_flags(EventFlags::HOMING).await {
                    Err(CodeExecutionFailure::BUSY)
                } else {
                    #[cfg(feature = "with-ps-on")]
                    if !self
                        .event_bus
                        .get_status()
                        .await
                        .contains(EventFlags::ATX_ON)
                    {
                        return Err(CodeExecutionFailure::PowerRequired);
                    }
                    self.probe.lock().await.probe_pin_up(300_000).await;
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            #[cfg(feature = "with-probe")]
            GCodeValue::G32 => {
                if self.event_bus.has_flags(EventFlags::HOMING).await {
                    Err(CodeExecutionFailure::BUSY)
                } else {
                    #[cfg(feature = "with-ps-on")]
                    if !self
                        .event_bus
                        .get_status()
                        .await
                        .contains(EventFlags::ATX_ON)
                    {
                        return Err(CodeExecutionFailure::PowerRequired);
                    }
                    let mut md = self.probe.lock().await;
                    md.probe_pin_down(300_000).await;
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            #[cfg(feature = "with-probe")]
            GCodeValue::M280(_sp) => {
                if _sp.p.and_then(|v| v.to_i32()).unwrap_or(0) != 0 {
                    self.write(channel, "echo: M280: unknown servo index\n").await;
                    return Err(CodeExecutionFailure::ERR);
                }
                match _sp.s.and_then(|v| v.to_i32()) {
                    Some(angle) if (0..=180).contains(&angle) => {
                        self.probe.lock().await.probe_set_angle(angle as u16, 0).await;
                        Ok(CodeExecutionSuccess::OK)
                    }
                    None => {
                        // No angle: report the PWM state (diagnostic)
                        let (duty, max) = self.probe.lock().await.probe_pwm_state();
                        // Pulse width assuming the nominal 50Hz (20ms) servo period
                        let msg = alloc::format!(
                            "echo: M280 P0 duty: {} max: {} pulse: {}us\n",
                            duty,
                            max,
                            duty as u64 * 20_000 / max.max(1) as u64
                        );
                        self.write(channel, msg.as_str()).await;
                        Ok(CodeExecutionSuccess::OK)
                    }
                    _ => {
                        self.write(channel, "echo: M280: S<angle> in [0, 180] required\n").await;
                        Err(CodeExecutionFailure::ERR)
                    }
                }
            }
            #[cfg(feature = "with-probe")]
            GCodeValue::M401 => {
                self.probe
                    .lock()
                    .await
                    .probe_pin_down(hwa::drivers::motion_driver::PROBE_SETTLE_US)
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-probe")]
            GCodeValue::M402 => {
                self.probe
                    .lock()
                    .await
                    .probe_pin_up(hwa::drivers::motion_driver::PROBE_SETTLE_US)
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::G80 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G90 => {
                self.motion_planner
                    .motion_status()
                    .set_absolute_positioning(true);
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G91 => {
                self.motion_planner
                    .motion_status()
                    .set_absolute_positioning(false);
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G92(_pos) => {
                self.motion_planner
                    .plan(channel, &gc, _blocking, &self.event_bus)
                    .await?;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G94 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M => {
                for x in GCodeValue::VARIANTS.iter().filter(|x| x.starts_with("M")) {
                    let _ = self
                        .write(channel, alloc::format!("echo: {}\n", x).as_str())
                        .await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M3 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M4 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M5 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M37(t) => {
                let dry_run_set = !t.s.unwrap_or(math::ZERO).is_zero();
                let _ = self
                    .event_bus
                    .publish_event(if dry_run_set {
                        EventStatus::containing(EventFlags::DRY_RUN)
                    } else {
                        EventStatus::not_containing(EventFlags::DRY_RUN)
                    })
                    .await;
                if dry_run_set {
                    let _ = self.write(channel, "echo: DRY_RUN mode enabled\n").await;
                } else {
                    let _ = self.write(channel, "echo: DRY_RUN mode disabled\n").await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M73 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M79 => {
                let _ = self.write(channel, "echo: Software reset\n").await;
                self.flush(channel).await;
                hwa::Contract::sys_reset();
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-ps-on")]
            GCodeValue::M80 => {
                hwa::debug!("Received PowerOn");
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-ps-on")] {
                        self.ps_on.apply_mut(|pin| pin.set_high());
                        embassy_time::Timer::after_millis(500).await;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-trinamic")] {
                        let _ = self.motion_planner.motion_driver().lock().await.trinamic_controller.init().await.is_ok();
                    }
                }
                #[cfg(feature = "with-ps-on")]
                self.event_bus
                    .publish_event(EventStatus::containing(EventFlags::ATX_ON))
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-ps-on")]
            GCodeValue::M81 => {
                hwa::info!("Received PowerOff");
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-ps-on")] {
                        self.ps_on.apply_mut(
                            |pin| pin.set_low()
                        );
                    }
                }
                #[cfg(feature = "with-ps-on")]
                self.event_bus
                    .publish_event(EventStatus::not_containing(EventFlags::ATX_ON))
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M82 => {
                self.motion_planner
                    .motion_status()
                    .set_relative_extrusion(false);
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M83 => {
                self.motion_planner
                    .motion_status()
                    .set_relative_extrusion(true);
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M84 => {
                // Wait for the queued moves before disabling the steppers (bounded, so that a
                // stalled motion, e.g. ATX power off, can not block the issuer forever)
                let _ = embassy_time::with_timeout(
                    embassy_time::Duration::from_secs(120),
                    self.wait_for_motion_idle(),
                )
                .await;
                self.motion_planner
                    .motion_driver()
                    .lock()
                    .await
                    .step_actuator()
                    .disable_steppers(hwa::CoordSel::all_axis());
                // The next move must enable its drivers again
                hwa::controllers::motion_control::STEP_DRIVER.reset();
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M92(steps) => {
                let v: hwa::math::TVector<math::Real> = steps.into();
                self.motion_planner
                    .motion_config()
                    .update_steps_per_space_unit(&v);
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M100 => {
                let heap_current = hwa::Contract::heap_current_size();
                let stack_current = hwa::Contract::stack_reservation_current_size();
                if Contract::MAX_HEAP_SIZE_BYTES > 0 {
                    let z1 = alloc::format!(
                        "echo: {}/{} bytes ({:?}%) of heap usage\n",
                        heap_current,
                        Contract::MAX_HEAP_SIZE_BYTES,
                        math::Real::from_f32(
                            100.0f32 * heap_current as f32 / Contract::MAX_HEAP_SIZE_BYTES as f32
                        ),
                    );
                    self.write(channel, z1.as_str()).await;
                } else {
                    let z1 = alloc::format!("echo: {} bytes of heap usage\n", heap_current);
                    self.write(channel, z1.as_str()).await;
                }

                let z2 = alloc::format!(
                    "echo: {}/{} bytes ({:?}%) of expected stack reservation\n",
                    stack_current,
                    Contract::MAX_EXPECTED_STATIC_ALLOC_BYTES,
                    math::Real::from_f32(
                        100.0f32 * stack_current as f32
                            / Contract::MAX_EXPECTED_STATIC_ALLOC_BYTES as f32
                    ),
                );
                self.write(channel, z2.as_str()).await;

                let status = self.event_bus.get_status().await;
                let z3 = alloc::format!("echo: Status: {:?}\n", status);
                self.write(channel, z3.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            // Set hotend temperature
            // An immediate command
            #[cfg(feature = "with-hot-end")]
            GCodeValue::M104(s) => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let val = s.s.map(|v| v.to_f64() as f32).unwrap_or(0.0);
                let mut h = self.hot_end.lock().await;
                let max = h.max_target_temp();
                match h
                    .set_target_temp(
                        CommChannel::Internal,
                        DeferAction::HotEndTemperature,
                        val,
                        gc.order_num,
                        false,
                    )
                    .await
                {
                    Ok(_) => Ok(CodeExecutionSuccess::OK),
                    Err(e) => {
                        drop(h);
                        Err(self.report_heater_error(channel, e, max).await)
                    }
                }
            }
            GCodeValue::M105 => {
                let hotend_temp_report = {
                    #[cfg(feature = "with-hot-end")]
                    {
                        let mut h = self.hot_end.lock().await;
                        alloc::format!(
                            " T:{:?} /{:?} T@:{:?}{}",
                            math::Real::from_f32(h.get_current_temp()),
                            math::Real::from_f32(h.get_target_temp()),
                            math::Real::from_f32(h.get_current_power()),
                            if cfg!(feature = "marlin-compat") {
                                alloc::string::String::new()
                            } else {
                                alloc::format!(" TZ:{:?}", math::Real::from_f32(h.get_current_resistance()))
                            },
                        )
                    }
                    #[cfg(not(feature = "with-hot-end"))]
                    alloc::string::String::from(" T:0 /0 T@:0")
                };
                let hotbed_temp_report = {
                    #[cfg(feature = "with-hot-bed")]
                    {
                        let mut h = self.hot_bed.lock().await;
                        alloc::format!(
                            " B:{:?} /{:?} B@:{:?}{}",
                            math::Real::from_f32(h.get_current_temp()),
                            math::Real::from_f32(h.get_target_temp()),
                            math::Real::from_f32(h.get_current_power()),
                            if cfg!(feature = "marlin-compat") {
                                alloc::string::String::new()
                            } else {
                                alloc::format!(" BZ:{:?}", math::Real::from_f32(h.get_current_resistance()))
                            },
                        )
                    }
                    #[cfg(not(feature = "with-hot-bed"))]
                    alloc::string::String::from(" B:0 /0 B@:0")
                };

                let report = alloc::format!("ok{}{}\n", hotend_temp_report, hotbed_temp_report);
                let _ = self.write(channel, report.as_str()).await;
                Ok(CodeExecutionSuccess::CONSUMED)
            }
            #[cfg(feature = "with-fan-layer")]
            GCodeValue::M106(_fan) => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                // S in [0, 255] (default: full power) mapped to the PWM scale [0, 100]
                let s = _fan
                    .s
                    .and_then(|v| v.to_i32())
                    .unwrap_or(255)
                    .clamp(0, 255);
                let percent = ((s * 100 + 127) / 255) as u8;
                match _fan.p.and_then(|v| v.to_i32()).unwrap_or(0) {
                    0 => self.fan_layer.lock().await.set_power(percent),
                    #[cfg(feature = "with-fan-extra-1")]
                    1 => {
                        FAN_EXTRA1_MANUAL_POWER.store(percent, core::sync::atomic::Ordering::Relaxed);
                        self.fan_extra1.lock().await.set_power(percent);
                    }
                    _ => {
                        self.write(channel, "echo: M106: unknown fan index\n").await;
                        return Err(CodeExecutionFailure::ERR);
                    }
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-fan-layer")]
            GCodeValue::M107 => {
                self.fan_layer.lock().await.set_power(0);
                Ok(CodeExecutionSuccess::OK)
            }
            // Wait for hot-end temperature
            // Mostly deferred code
            #[cfg(feature = "with-hot-end")]
            GCodeValue::M109(s) => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let mut he = self.hot_end.lock().await;
                let max = he.max_target_temp();
                let result = match (s.s, s.r) {
                    // M109 R: wait for heating and cooling
                    (_, Some(r)) => he
                        .set_target_temp(channel, DeferAction::HotEndTemperature, r.to_f64() as f32, gc.order_num, true)
                        .await,
                    // M109 S: wait only when heating
                    (Some(v), None) => he
                        .set_target_temp(channel, DeferAction::HotEndTemperature, v.to_f64() as f32, gc.order_num, false)
                        .await,
                    // M109: wait for the current target
                    (None, None) => Ok(he
                        .ping_subscribe(channel, DeferAction::HotEndTemperature, gc.order_num)
                        .await),
                };
                drop(he);
                match result {
                    Ok(true) => Ok(CodeExecutionSuccess::DEFERRED(EventStatus::containing(
                        EventFlags::HOT_END_TEMP_OK,
                    ))),
                    Ok(false) => Ok(CodeExecutionSuccess::OK),
                    Err(e) => Err(self.report_heater_error(channel, e, max).await),
                }
            }
            GCodeValue::M110(_n) => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::M114 => {
                let _pos: hwa::controllers::Position = self
                    .motion_planner
                    .motion_status()
                    .get_last_planned_position();
                let _rpos: hwa::controllers::Position =
                    self.motion_planner.motion_status().get_current_position();
                hwa::debug!(
                    "M114: p_pos: [ {:?}], [{:?}] ] r_pos: [ {:?}], [{:?}] ]",
                    _pos.world_pos,
                    _pos.space_pos,
                    _rpos.world_pos,
                    _rpos.space_pos
                );
                let _step_pos: hwa::math::TVector<i32> = (_rpos.world_pos
                    * self
                        .motion_planner
                        .motion_config()
                        .get_steps_per_space_unit_as_vector())
                .map_values(|_c, _v| Some(_v.to_i32().unwrap_or(0)));
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-x-axis")] {
                        let r_x = _pos.world_pos.x.unwrap_or(math::ZERO);
                        let c_x = _step_pos.x.unwrap_or(0);
                    }
                    else {
                        let r_x = math::ZERO;
                        let c_x = math::ZERO;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-y-axis")] {
                        let r_y = _pos.world_pos.y.unwrap_or(math::ZERO);
                        let c_y = _step_pos.y.unwrap_or(0);
                    }
                    else {
                        let r_y = math::ZERO;
                        let c_y = math::ZERO;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-z-axis")] {
                        let r_z = _pos.world_pos.z.unwrap_or(math::ZERO);
                        let c_z = _step_pos.z.unwrap_or(0);
                    }
                    else {
                        let r_z = math::ZERO;
                        let c_z = math::ZERO;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-e-axis")] {
                        let r_e = self.motion_planner.motion_status().get_logical_e();
                        let c_e = _step_pos.e.unwrap_or(0);
                    }
                    else {
                        let r_e = math::ZERO;
                        let c_e = math::ZERO;
                    }
                }
                let z = alloc::format!(
                    "X:{:?} Y:{:?} Z:{:?} E:{:?} Count: X:{:?} Y:{:?} Z:{:?} E:{:?}\n",
                    r_x,
                    r_y,
                    r_z,
                    r_e,
                    c_x,
                    c_y,
                    c_z,
                    c_e,
                );
                let _ = self.write(channel, z.as_str()).await;
                let z2 = alloc::format!("echo: Space {:#?}\n", _rpos.space_pos);
                let _ = self.write(channel, z2.as_str()).await;
                let _ = self.write(channel, "ok\n").await;
                Ok(CodeExecutionSuccess::CONSUMED)
            }
            #[cfg(feature = "marlin-compat")]
            GCodeValue::M115 => {
                // Marlin-style identification: displays such as the BTT TFT only enable themselves
                // when FIRMWARE_NAME (no `echo:` prefix, no space after the colon) names Marlin.
                let ident = alloc::format!(
                    "FIRMWARE_NAME:Marlin 2.1.2 ({} {}) SOURCE_CODE_URL:{} PROTOCOL_VERSION:1.0 MACHINE_TYPE:{} EXTRUDER_COUNT:{} UUID:{}\n",
                    hwa::Contract::FIRMWARE_NAME,
                    hwa::Contract::FIRMWARE_VERSION,
                    hwa::Contract::FIRMWARE_URL,
                    hwa::Contract::MACHINE_TYPE,
                    hwa::Contract::EXTRUDER_COUNT,
                    hwa::Contract::MACHINE_UUID,
                );
                let _ = self.write(channel, ident.as_str()).await;
                let sd = if cfg!(feature = "with-sd-card") { 1 } else { 0 };
                let caps = alloc::format!(
                    "Cap:SERIAL_XON_XOFF:0\nCap:BINARY_FILE_TRANSFER:0\nCap:EEPROM:0\nCap:VOLUMETRIC:0\nCap:AUTOREPORT_POS:0\nCap:AUTOREPORT_TEMP:0\nCap:PROGRESS:0\nCap:PRINT_JOB:{}\nCap:AUTOLEVEL:0\nCap:RUNOUT:0\nCap:Z_PROBE:0\nCap:LEVELING_DATA:0\nCap:BUILD_PERCENT:0\nCap:SOFTWARE_POWER:{}\nCap:TOGGLE_LIGHTS:0\nCap:CASE_LIGHT_BRIGHTNESS:0\nCap:EMERGENCY_PARSER:0\nCap:HOST_ACTION_COMMANDS:0\nCap:PROMPT_SUPPORT:0\nCap:SDCARD:{}\nCap:MULTI_VOLUME:0\nCap:REPEAT:0\nCap:SD_WRITE:0\nCap:AUTOREPORT_SD_STATUS:0\nCap:LONG_FILENAME:0\nCap:THERMAL_PROTECTION:0\nCap:MOTION_MODES:0\nCap:ARCS:0\nCap:BABYSTEPPING:0\nCap:CHAMBER_TEMPERATURE:0\nCap:COOLER_TEMPERATURE:0\nCap:MEATPACK:0\n",
                    if cfg!(feature = "with-print-job") { 1 } else { 0 },
                    if cfg!(feature = "with-ps-on") { 1 } else { 0 },
                    sd,
                );
                let _ = self.write(channel, caps.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(not(feature = "marlin-compat"))]
            GCodeValue::M115 => {
                let _ = self.write(channel, "echo: FIRMWARE_NAME: ").await;
                let _ = self.write(channel, hwa::Contract::FIRMWARE_NAME).await;
                let _ = self.write(channel, " FIRMWARE_VERSION: ").await;
                let _ = self.write(channel, hwa::Contract::FIRMWARE_VERSION).await;
                let _ = self.write(channel, " FIRMWARE_URL: ").await;
                let _ = self.write(channel, hwa::Contract::FIRMWARE_URL).await;
                let _ = self.write(channel, " MACHINE_TYPE: ").await;
                let _ = self.write(channel, hwa::Contract::MACHINE_TYPE).await;
                let _ = self.write(channel, " MACHINE_BOARD: ").await;
                let _ = self.write(channel, hwa::Contract::MACHINE_BOARD).await;
                let _ = self.write(channel, " MACHINE_PROCESSOR: ").await;
                let _ = self.write(channel, hwa::Contract::MACHINE_PROCESSOR).await;
                let _ = self.write(channel, " MACHINE_UUID: ").await;
                let _ = self.write(channel, hwa::Contract::MACHINE_UUID).await;
                let _ = self.write(channel, " EXTRUDER_COUNT: ").await;
                let _ = self.write(channel, hwa::Contract::EXTRUDER_COUNT).await;
                let _ = self.write(channel, "\n").await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M117(_msg) => {
                if let Some(msg) = _msg {
                    let _ = self.write(Contract::DISPLAY_CHANNEL, "echo: ").await;
                    let _ = self.write(Contract::DISPLAY_CHANNEL, msg.as_str()).await;
                    let _ = self.write(Contract::DISPLAY_CHANNEL, "\n").await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M118(_msg) => {
                if let Some(msg) = _msg {
                    let _ = self.write(Contract::HOST_CHANNEL, "echo: ").await;
                    let _ = self.write(Contract::HOST_CHANNEL, msg.as_str()).await;
                    let _ = self.write(Contract::HOST_CHANNEL, "\n").await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M119 => {
                let mg = self.motion_planner.motion_driver().lock().await;
                let x_endstop = if mg.step_actuator.end_stop_triggered(hwa::CoordSel::X) {
                    1
                } else {
                    0
                };
                let y_endstop = if mg.step_actuator.end_stop_triggered(hwa::CoordSel::Y) {
                    1
                } else {
                    0
                };
                let z_endstop = if mg.step_actuator.end_stop_triggered(hwa::CoordSel::Z) {
                    1
                } else {
                    0
                };
                drop(mg);
                let z = alloc::format!("M119 X {} Y {} Z {}\n", x_endstop, y_endstop, z_endstop);
                let _ = self.write(channel, z.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            // Set hot-bed temperature
            // An immediate command
            #[cfg(feature = "with-hot-bed")]
            GCodeValue::M140(s) => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let val = s.s.map(|v| v.to_f64() as f32).unwrap_or(0.0);
                let mut h = self.hot_bed.lock().await;
                let max = h.max_target_temp();
                match h
                    .set_target_temp(
                        CommChannel::Internal,
                        DeferAction::HotBedTemperature,
                        val,
                        gc.order_num,
                        false,
                    )
                    .await
                {
                    Ok(_) => Ok(CodeExecutionSuccess::OK),
                    Err(e) => {
                        drop(h);
                        Err(self.report_heater_error(channel, e, max).await)
                    }
                }
            }
            // Wait for hot-bed temperature
            // A normally deferred command
            #[cfg(feature = "with-hot-bed")]
            GCodeValue::M190(s) => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let mut he = self.hot_bed.lock().await;
                let max = he.max_target_temp();
                let result = match (s.s, s.r) {
                    (_, Some(r)) => he
                        .set_target_temp(channel, DeferAction::HotBedTemperature, r.to_f64() as f32, gc.order_num, true)
                        .await,
                    (Some(v), None) => he
                        .set_target_temp(channel, DeferAction::HotBedTemperature, v.to_f64() as f32, gc.order_num, false)
                        .await,
                    (None, None) => Ok(he
                        .ping_subscribe(channel, DeferAction::HotBedTemperature, gc.order_num)
                        .await),
                };
                drop(he);
                match result {
                    Ok(true) => Ok(CodeExecutionSuccess::DEFERRED(EventStatus::containing(
                        EventFlags::HOT_BED_TEMP_OK,
                    ))),
                    Ok(false) => Ok(CodeExecutionSuccess::OK),
                    Err(e) => Err(self.report_heater_error(channel, e, max).await),
                }
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M201(accel) => {
                let v: hwa::math::TVector<math::Real> = accel.into();
                self.motion_planner
                    .motion_config()
                    .update_max_accel(&v.map_values(|_, x| x.is_defined_positive().then_some(x)));
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M203(speed) => {
                let v: hwa::math::TVector<math::Real> = speed.into();
                self.motion_planner
                    .motion_config()
                    .update_max_speed(&v.map_values(|_, x| x.is_defined_positive().then_some(x)));
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M204(accel) => {
                let positive = |v: Option<math::Real>| v.filter(|x| x.is_defined_positive());
                let cfg = self.motion_planner.motion_config();
                if let Some(s) = positive(accel.s) {
                    cfg.set_print_accel(Some(s));
                    cfg.set_travel_accel(Some(s));
                }
                if let Some(p) = positive(accel.p) {
                    cfg.set_print_accel(Some(p));
                }
                if let Some(t) = positive(accel.t) {
                    cfg.set_travel_accel(Some(t));
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M205 => {
                hwa::debug!("M205 ignored: jerk semantics differ from the S-curve profile");
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M206(offset) => {
                // Takes effect on the next homing (as the stored offset in Marlin)
                let v: hwa::math::TVector<math::Real> = offset.into();
                self.motion_planner.motion_config().update_home_offset(&v);
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
            GCodeValue::M207(p) => {
                let cfg = self.motion_planner.motion_config();
                let mut r = cfg.get_retraction();
                if let Some(s) = p.s {
                    r.length = s.abs();
                }
                if let Some(f) = p.f.filter(|f| f.is_defined_positive()) {
                    r.feed_rate = f / math::Real::from_lit(60, 0);
                }
                if let Some(z) = p.z {
                    r.z_hop = z.abs();
                }
                cfg.set_retraction(r);
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
            GCodeValue::M208(p) => {
                let cfg = self.motion_planner.motion_config();
                let mut r = cfg.get_retraction();
                if let Some(s) = p.s {
                    r.recover_extra = s;
                }
                if let Some(f) = p.f.filter(|f| f.is_defined_positive()) {
                    r.recover_feed_rate = f / math::Real::from_lit(60, 0);
                }
                cfg.set_retraction(r);
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M220(s) => {
                #[cfg(feature = "with-motion")]
                {
                    let cfg = self.motion_planner.motion_config();
                    match s.s.and_then(|v| v.to_i32()) {
                        Some(v) => cfg.set_speed_rate(v.clamp(10, 255) as u8),
                        None => {
                            let msg = alloc::format!("echo: FR:{}%\n", cfg.get_speed_rate());
                            self.write(channel, msg.as_str()).await;
                        }
                    }
                }
                #[cfg(not(feature = "with-motion"))]
                let _ = s;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M221(s) => {
                #[cfg(feature = "with-motion")]
                {
                    let cfg = self.motion_planner.motion_config();
                    match s.s.and_then(|v| v.to_i32()) {
                        Some(v) => cfg.set_flow_rate(v.clamp(10, 255) as u8),
                        None => {
                            let msg = alloc::format!("echo: E0 Flow: {}%\n", cfg.get_flow_rate());
                            self.write(channel, msg.as_str()).await;
                        }
                    }
                }
                #[cfg(not(feature = "with-motion"))]
                let _ = s;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-hot-end")]
            GCodeValue::M301(pid) => {
                let f = |v: Option<math::Real>| v.map(|x| x.to_f64() as f32);
                self.hot_end.lock().await.set_pid(f(pid.p), f(pid.i), f(pid.d));
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-hot-bed")]
            GCodeValue::M304(pid) => {
                let f = |v: Option<math::Real>| v.map(|x| x.to_f64() as f32);
                self.hot_bed.lock().await.set_pid(f(pid.p), f(pid.i), f(pid.d));
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
            GCodeValue::M302(p) => {
                match (p.s, p.p) {
                    (None, None) => {
                        let msg = match self.cold_extrusion_min_temp() {
                            Some(t) => alloc::format!(
                                "echo: Cold extrudes are disabled (min temp {:?}C)\n",
                                math::Real::from_f32(t)
                            ),
                            None => alloc::string::String::from("echo: Cold extrudes are enabled\n"),
                        };
                        self.write(channel, msg.as_str()).await;
                    }
                    (s, p) => {
                        if let Some(s) = s {
                            COLD_EXTRUSION_MIN_TEMP.store(
                                (s.to_f64() as f32).max(0.0).to_bits(),
                                core::sync::atomic::Ordering::Relaxed,
                            );
                        }
                        if let Some(p) = p {
                            COLD_EXTRUSION_ALLOWED
                                .store(!p.is_zero(), core::sync::atomic::Ordering::Relaxed);
                        }
                    }
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
            GCodeValue::M303(p) => {
                self.require_power().await?;
                let heater = p.e.and_then(|v| v.to_i32()).unwrap_or(0);
                let cycles = p.c.and_then(|v| v.to_i32()).unwrap_or(5).clamp(3, 20) as u8;
                let result = match heater {
                    #[cfg(feature = "with-hot-bed")]
                    -1 => {
                        let target = p.s.map(|v| v.to_f64() as f32).unwrap_or(70.0);
                        let mut h = self.hot_bed.lock().await;
                        let max = h.max_target_temp();
                        h.start_autotune(target, cycles).map_err(|e| (e, max))
                    }
                    #[cfg(feature = "with-hot-end")]
                    0 => {
                        let target = p.s.map(|v| v.to_f64() as f32).unwrap_or(150.0);
                        let mut h = self.hot_end.lock().await;
                        let max = h.max_target_temp();
                        h.start_autotune(target, cycles).map_err(|e| (e, max))
                    }
                    _ => {
                        self.write(channel, "echo: PID Autotune failed! Bad heater id\n").await;
                        return Err(CodeExecutionFailure::ERR);
                    }
                };
                match result {
                    Ok(_) => {
                        self.write(channel, "echo: PID Autotune start\n").await;
                        Ok(CodeExecutionSuccess::OK)
                    }
                    Err((e, max)) => Err(self.report_heater_error(channel, e, max).await),
                }
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M400 => Ok(self
                .motion_planner
                .plan(channel, &gc, _blocking, &self.event_bus)
                .await?),
            #[cfg(feature = "with-motion")]
            GCodeValue::M410 => {
                self.quick_stop().await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M112 => {
                self.emergency_stop("Emergency stop (M112)").await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M999 => {
                #[cfg(feature = "with-hot-end")]
                self.hot_end.lock().await.clear_fault();
                #[cfg(feature = "with-hot-bed")]
                self.hot_bed.lock().await.clear_fault();
                self.event_bus
                    .publish_event(EventStatus::not_containing(EventFlags::SYS_ALARM))
                    .await;
                #[cfg(feature = "with-motion")]
                if !self
                    .motion_planner
                    .motion_status()
                    .get_last_planned_position()
                    .is_set
                {
                    self.write(channel, "echo: Resumed. Homing (G28) is required\n").await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-eeprom")]
            GCodeValue::M500 => match crate::processing::settings_store::store(self).await {
                Ok((size, crc)) => {
                    let msg = alloc::format!("echo:Settings Stored ({} bytes; crc {})\n", size, crc);
                    self.write(channel, msg.as_str()).await;
                    Ok(CodeExecutionSuccess::OK)
                }
                Err(e) => {
                    let msg = alloc::format!("echo:EEPROM error: {}\n", e.description());
                    self.write(channel, msg.as_str()).await;
                    Err(CodeExecutionFailure::ERR)
                }
            },
            #[cfg(feature = "with-eeprom")]
            GCodeValue::M501 => match crate::processing::settings_store::retrieve(self).await {
                Ok((size, crc)) => {
                    let msg = alloc::format!(
                        "echo:Stored settings retrieved ({} bytes; crc {})\n",
                        size,
                        crc
                    );
                    self.write(channel, msg.as_str()).await;
                    Ok(CodeExecutionSuccess::OK)
                }
                Err(e) => {
                    let msg = alloc::format!("echo:EEPROM error: {}\n", e.description());
                    self.write(channel, msg.as_str()).await;
                    Err(CodeExecutionFailure::ERR)
                }
            },
            GCodeValue::M502 => {
                // Restore the default settings
                #[cfg(feature = "with-motion")]
                self.motion_planner.motion_config().apply_defaults();
                #[cfg(feature = "with-hot-end")]
                self.hot_end.lock().await.reset_pid();
                #[cfg(feature = "with-hot-bed")]
                self.hot_bed.lock().await.reset_pid();
                #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
                {
                    COLD_EXTRUSION_MIN_TEMP.store(
                        <Contract as HwiContract>::EXTRUDE_MIN_TEMP.to_bits(),
                        core::sync::atomic::Ordering::Relaxed,
                    );
                    COLD_EXTRUSION_ALLOWED.store(false, core::sync::atomic::Ordering::Relaxed);
                }
                #[cfg(feature = "with-trinamic")]
                {
                    let success = self
                        .motion_planner
                        .motion_driver()
                        .lock()
                        .await
                        .trinamic_controller
                        .init()
                        .await
                        .is_ok();
                    if !success {
                        self.write(channel, "echo: M502 Trinamic init failed\n").await;
                    }
                }
                self.write(channel, "echo: Hardcoded Default Settings Loaded\n").await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M503(_param) => {
                let detailed = !_param.s.unwrap_or(hwa::math::ONE).is_zero();
                self.write_section(channel, detailed, "Units in mm", alloc::string::String::from("G21")).await;
                #[cfg(feature = "with-motion")]
                {
                    let cfg = self.motion_planner.motion_config();
                    let status = self.motion_planner.motion_status();
                    self.write_section(channel, detailed, 
                        "Steps per unit:",
                        alloc::format!("M92 {}", axis_words(&cfg.get_steps_per_space_unit_as_vector())),
                    ).await;
                    self.write_section(channel, detailed, 
                        "Maximum feedrates (units/s):",
                        alloc::format!("M203 {}", axis_words(&cfg.get_max_speed())),
                    ).await;
                    self.write_section(channel, detailed, 
                        "Maximum Acceleration (units/s2):",
                        alloc::format!("M201 {}", axis_words(&cfg.get_max_accel())),
                    ).await;
                    self.write_section(channel, detailed, 
                        "Acceleration (units/s2): P<print_accel> T<travel_accel> (0 = per axis limits)",
                        alloc::format!(
                            "M204 P{:?} T{:?}",
                            cfg.get_print_accel().unwrap_or(math::ZERO).rdp(2),
                            cfg.get_travel_accel().unwrap_or(math::ZERO).rdp(2)
                        ),
                    ).await;
                    self.write_section(channel, detailed, 
                        "Jerk (units/s3, S-curve profile):",
                        alloc::format!("; {}", axis_words(&cfg.get_max_jerk())),
                    ).await;
                    self.write_section(channel, detailed, 
                        "Home offset:",
                        alloc::format!("M206 {}", axis_words(&cfg.get_home_offset())),
                    ).await;
                    self.write_section(channel, detailed, 
                        "Feedrate / Flow percentage:",
                        alloc::format!("M220 S{} ; M221 S{}", cfg.get_speed_rate(), cfg.get_flow_rate()),
                    ).await;
                    self.write_section(channel, detailed, 
                        "Extruder mode:",
                        alloc::string::String::from(match status.is_relative_extrusion() {
                            true => "M83",
                            false => "M82",
                        }),
                    ).await;
                    #[cfg(feature = "with-e-axis")]
                    {
                        let r = cfg.get_retraction();
                        self.write_section(channel, detailed, 
                            "Retract: S<length> F<units/m> Z<lift>",
                            alloc::format!(
                                "M207 S{:?} F{:?} Z{:?}",
                                r.length.rdp(2),
                                (r.feed_rate * math::Real::from_lit(60, 0)).rdp(2),
                                r.z_hop.rdp(2)
                            ),
                        ).await;
                        self.write_section(channel, detailed, 
                            "Recover: S<length> F<units/m>",
                            alloc::format!(
                                "M208 S{:?} F{:?}",
                                r.recover_extra.rdp(2),
                                (r.recover_feed_rate * math::Real::from_lit(60, 0)).rdp(2)
                            ),
                        ).await;
                    }
                }
                #[cfg(feature = "with-hot-end")]
                {
                    let (kp, ki, kd) = self.hot_end.lock().await.get_pid();
                    self.write_section(channel, detailed, 
                        "Hotend PID settings:",
                        alloc::format!(
                            "M301 P{:?} I{:?} D{:?}",
                            math::Real::from_f32(kp).rdp(2),
                            math::Real::from_f32(ki).rdp(3),
                            math::Real::from_f32(kd).rdp(2)
                        ),
                    ).await;
                }
                #[cfg(feature = "with-hot-bed")]
                {
                    let (kp, ki, kd) = self.hot_bed.lock().await.get_pid();
                    self.write_section(channel, detailed, 
                        "Bed PID settings:",
                        alloc::format!(
                            "M304 P{:?} I{:?} D{:?}",
                            math::Real::from_f32(kp).rdp(2),
                            math::Real::from_f32(ki).rdp(3),
                            math::Real::from_f32(kd).rdp(2)
                        ),
                    ).await;
                }
                #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
                self.write_section(channel, detailed, 
                    "Cold extrusion: S<min temp> P<1 = allowed>",
                    alloc::format!(
                        "M302 S{:?} P{}",
                        math::Real::from_f32(f32::from_bits(
                            COLD_EXTRUSION_MIN_TEMP.load(core::sync::atomic::Ordering::Relaxed)
                        )),
                        COLD_EXTRUSION_ALLOWED.load(core::sync::atomic::Ordering::Relaxed) as u8
                    ),
                ).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M862_1 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::M862_3 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::M900 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::M907 => Ok(CodeExecutionSuccess::OK),
            _ => Err(CodeExecutionFailure::NotYetImplemented),
        };
        result
    }
}

impl Drop for GCodeProcessor {
    fn drop(&mut self) {}
}
