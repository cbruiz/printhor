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
//! - 7.6KiB by [core::fmt::float::float_to_decimal_common_shortest]
//! - 6.3KiB by [core::fmt::float::float_to_decimal_common_exact]
//!
//! Around 14KB in total
//!
//! Actually, 5.1KiB only using [math::Real::format] and/or [math::Real::fmt] with lexical_core/ryu.

use crate::control;
use crate::hwa;
use control::{CodeExecutionFailure, CodeExecutionResult, CodeExecutionSuccess};
use control::{GCodeCmd, GCodeValue};
use hwa::math;
#[allow(unused)]
use hwa::AsyncMutexStrategy;
use hwa::Contract;
use hwa::HwiContract;
#[allow(unused)]
use hwa::SyncMutexStrategy;

#[cfg(feature = "with-probe")]
use hwa::controllers::ProbeTrait;
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
        }
    }

    pub async fn write_ok(&self, channel: CommChannel) {
        self.write(channel, "ok\n").await;
    }

    pub async fn write(&self, channel: CommChannel, _msg: &str) {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            CommChannel::SerialUsb => {
                let _ = self
                    .serial_usb_tx
                    .lock()
                    .await
                    .write_packet(_msg.as_bytes())
                    .await;
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
            #[cfg(feature = "with-motion")]
            GCodeValue::G4(_) => Ok(self
                .motion_planner
                .plan(channel, &gc, _blocking, &self.event_bus)
                .await?),

            #[cfg(feature = "with-motion")]
            GCodeValue::G10 => Ok(CodeExecutionSuccess::OK),
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
                let updated_coords = _pos.as_vector();
                hwa::warn!("TODO: G92 {:?}", updated_coords);
                // TODO: Push "position" move into move queue
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
            GCodeValue::M83 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::M84 => Ok(CodeExecutionSuccess::OK),
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
                let val = s.s.and_then(|v| v.to_i32()).unwrap_or(0);
                let mut h = self.hot_end.lock().await;
                h.set_target_temp(
                    CommChannel::Internal,
                    DeferAction::HotEndTemperature,
                    val as f32,
                    gc.order_num,
                )
                .await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M105 => {
                let hotend_temp_report = {
                    #[cfg(feature = "with-hot-end")]
                    {
                        let mut h = self.hot_end.lock().await;
                        alloc::format!(
                            " T:{:?} /{:?} T@:{:?} TZ:{:?}",
                            math::Real::from_f32(h.get_current_temp()),
                            math::Real::from_f32(h.get_target_temp()),
                            math::Real::from_f32(h.get_current_power()),
                            math::Real::from_f32(h.get_current_resistance()),
                        )
                    }
                    #[cfg(not(feature = "with-hot-end"))]
                    alloc::string::String::new()
                };
                let hotbed_temp_report = {
                    #[cfg(feature = "with-hot-bed")]
                    {
                        let mut h = self.hot_bed.lock().await;
                        alloc::format!(
                            " B:{:?} /{:?} B@:{:?} BZ:{:?}",
                            math::Real::from_f32(h.get_current_temp()),
                            math::Real::from_f32(h.get_target_temp()),
                            math::Real::from_f32(h.get_current_power()),
                            math::Real::from_f32(h.get_current_resistance()),
                        )
                    }
                    #[cfg(not(feature = "with-hot-bed"))]
                    alloc::string::String::new()
                };

                let report = alloc::format!("ok{}{}\n", hotend_temp_report, hotbed_temp_report);
                let _ = self.write(channel, report.as_str()).await;
                Ok(CodeExecutionSuccess::CONSUMED)
            }
            #[cfg(feature = "with-fan-layer")]
            GCodeValue::M106 => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                //crate::info!("M106 BEGIN");
                self.fan_layer.lock().await.set_power(255);
                //crate::info!("M106 END");
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
                let deferred = {
                    let mut he = self.hot_end.lock().await;
                    let value = s.s.and_then(|v| v.to_i32()).unwrap_or(0);
                    let was_deferred = he
                        .set_target_temp(
                            channel,
                            DeferAction::HotEndTemperature,
                            value as f32,
                            gc.order_num,
                        )
                        .await;
                    value > 0 && was_deferred
                };
                if deferred {
                    Ok(CodeExecutionSuccess::DEFERRED(EventStatus::containing(
                        EventFlags::HOT_END_TEMP_OK,
                    )))
                } else {
                    Ok(CodeExecutionSuccess::OK)
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
                let z = alloc::format!("{:?} Count {:?}\n", _pos.world_pos, _rpos.world_pos.map(
                    |_c, _v| {
                        Some(0u32)
                    }
                    
                ));
                let _ = self.write(channel, z.as_str()).await;
                let _ = self.write(channel, "ok").await;
                let z2 = alloc::format!("echo: Space {:#?}\n", _rpos.space_pos);
                let _ = self.write(channel, z2.as_str()).await;
                Ok(CodeExecutionSuccess::CONSUMED)
            }
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
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M117 => {
                let _ = self.write(channel, "echo: (display msg)\n").await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M119 => {
                todo!("")
                /*
                let mut d = self.motion_planner.motion_driver.lock().await;
                let z = alloc::format!(
                    "M119 X {} Y {} Z {}\n",
                    if d.endstop_triggered(StepperChannel::X) {
                        "1"
                    } else {
                        "0"
                    },
                    if d.endstop_triggered(StepperChannel::Y) {
                        "1"
                    } else {
                        "0"
                    },
                    if d.endstop_triggered(StepperChannel::Z) {
                        "1"
                    } else {
                        "0"
                    },
                );
                drop(d);
                let _ = self.write(channel, z.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
                 */
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
                let val = s.s.and_then(|v| v.to_i32()).unwrap_or(0);
                let mut h = self.hot_bed.lock().await;
                h.set_target_temp(
                    CommChannel::Internal,
                    DeferAction::HotBedTemperature,
                    val as f32,
                    gc.order_num,
                )
                .await;
                Ok(CodeExecutionSuccess::OK)
            }
            // Wait for hot-bed temperature
            // A normally deferred command
            #[cfg(feature = "with-hot-bed")]
            GCodeValue::M190 => {
                #[cfg(feature = "with-ps-on")]
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let deferred = {
                    let mut he = self.hot_bed.lock().await;
                    he.ping_subscribe(channel, DeferAction::HotBedTemperature, gc.order_num)
                        .await
                };
                if deferred {
                    Ok(CodeExecutionSuccess::DEFERRED(EventStatus::containing(
                        EventFlags::HOT_BED_TEMP_OK,
                    )))
                } else {
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M201 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::M203 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M204 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M205 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::M206 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M220(_) => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M221(_) => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-trinamic")]
            GCodeValue::M502 => {
                /*
                if !self.event_bus.get_status().await.contains(EventFlags::ATX_ON) {
                    return Err(CodeExecutionFailure::PowerRequired)
                }
                 */
                let success = {
                    self.motion_planner
                        .motion_driver()
                        .lock()
                        .await
                        .trinamic_controller
                        .init()
                        .await
                        .is_ok()
                };
                if success {
                    let _ = self.write(channel, "ok; M502\n").await;
                } else {
                    let _ = self.write(channel, "error; M502 (internal error)\n").await;
                }
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
