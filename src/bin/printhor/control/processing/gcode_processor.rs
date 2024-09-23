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
//! Actually, 5.1KiB only using [math::Real::format] and/or [math::Real::fmt] with lexical_core.
use crate::control::{GCodeCmd, GCodeValue};
use crate::control::{CodeExecutionFailure, CodeExecutionResult, CodeExecutionSuccess};
use crate::hwa;
use crate::machine::MACHINE_INFO;
use crate::math;

#[allow(unused)]
use printhor_hwa_common::StepperChannel;

#[cfg(feature = "with-probe")]
use hwa::controllers::ProbeTrait;
#[allow(unused)]
use hwa::{CommChannel, DeferAction, DeferEvent, EventBusRef, EventFlags, EventStatus};
use strum::VariantNames;
#[cfg(feature = "with-motion")]
use crate::tgeo::TVector;
cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-serial-port-1", feature="with-serial-port-2"))]
    {
        #[allow(unused)]
        use embedded_io_async::Write;
        #[allow(unused)]
        use printhor_hwa_common::AsyncWrapper;
    }
}

pub struct GCodeProcessorParams {
    pub event_bus: EventBusRef,

    #[cfg(feature = "with-motion")]
    pub motion_planner: hwa::controllers::MotionPlannerRef,
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: hwa::device::USBSerialTxControllerRef,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: hwa::device::UartPort1TxControllerRef,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: hwa::device::UartPort2TxControllerRef,
    #[cfg(feature = "with-ps-on")]
    pub ps_on: hwa::controllers::PsOnRef,
    #[cfg(feature = "with-probe")]
    pub probe: hwa::controllers::ServoControllerRef,
    #[cfg(feature = "with-hot-end")]
    pub hotend: hwa::controllers::HotendControllerRef,
    #[cfg(feature = "with-hot-bed")]
    pub hotbed: hwa::controllers::HotbedControllerRef,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer: hwa::controllers::FanLayerPwmControllerRef,
    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra_1: hwa::controllers::FanExtra1PwmControllerRef,
    #[cfg(feature = "with-laser")]
    pub laser: hwa::controllers::LaserPwmControllerRef,
}

#[derive(Clone)]
pub struct GCodeProcessor {
    pub event_bus: EventBusRef,

    #[cfg(feature = "with-motion")]
    pub motion_planner: hwa::controllers::MotionPlannerRef,
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: hwa::device::USBSerialTxControllerRef,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: hwa::device::UartPort1TxControllerRef,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: hwa::device::UartPort2TxControllerRef,
    #[cfg(feature = "with-ps-on")]
    pub ps_on: hwa::controllers::PsOnRef,
    #[cfg(feature = "with-probe")]
    #[allow(unused)]
    pub probe: hwa::controllers::ServoControllerRef,
    #[cfg(feature = "with-hot-end")]
    pub hotend: hwa::controllers::HotendControllerRef,
    #[cfg(feature = "with-hot-bed")]
    pub hotbed: hwa::controllers::HotbedControllerRef,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer: hwa::controllers::FanLayerPwmControllerRef,
    #[cfg(feature = "with-fan-extra-1")]
    #[allow(unused)]
    pub fan_extra_1: hwa::controllers::FanExtra1PwmControllerRef,
    #[cfg(feature = "with-laser")]
    pub _laser: hwa::controllers::LaserPwmControllerRef,
}

impl GCodeProcessor {
    pub fn new(params: GCodeProcessorParams) -> Self {

        Self {
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx: params.serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx: params.serial_port1_tx,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_tx: params.serial_port2_tx,
            #[cfg(feature = "with-ps-on")]
            ps_on: params.ps_on,
            #[cfg(feature = "with-motion")]
            motion_planner: params.motion_planner,
            #[cfg(feature = "with-probe")]
            probe: params.probe,
            #[cfg(feature = "with-hot-end")]
            hotend: params.hotend,
            #[cfg(feature = "with-hot-bed")]
            hotbed: params.hotbed,
            #[cfg(feature = "with-fan-layer")]
            fan_layer: params.fan_layer,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra_1: params.fan_extra_1,
            #[cfg(feature = "with-laser")]
            _laser: params.laser,

            event_bus: params.event_bus,
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
            CommChannel::Internal => {
                hwa::info!("[Internal] {}", _msg)
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
                use printhor_hwa_common::AsyncWrapper;
                let mut mg = self.serial_port1_tx.lock().await;
                mg.wrapped_flush().await;
            }
            #[cfg(feature = "with-serial-port-2")]
            CommChannel::SerialPort2 => {
                use printhor_hwa_common::AsyncWrapper;
                let mut mg = self.serial_port2_tx.lock().await;
                mg.wrapped_flush().await;
            }
            CommChannel::Internal => {}
        }
    }


    /// Executes the given GCode command.
    ///
    /// # Arguments
    ///
    /// * `channel` - The communication channel to use for the execution.
    /// * `gc` - The GCode command to execute.
    /// * `blocking` - A boolean indicating whether the execution should be blocking or not.
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
    #[allow(unused)]
    pub(crate) async fn execute(
        &mut self,
        channel: CommChannel,
        gc: &GCodeCmd,
        blocking: bool,
    ) -> CodeExecutionResult {
        let result = match &gc.value {
            #[cfg(feature = "grbl-compat")]
            GCodeValue::GRBLCmd => {
                let str = format!(
                    "[VER:1.1 {} v{}:]\n[MSG: Machine: {} {}]\n",
                    MACHINE_INFO.firmware_name,
                    MACHINE_INFO.firmware_version,
                    MACHINE_INFO.machine_board,
                    MACHINE_INFO.machine_type,
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
                    .plan(channel, &gc, blocking, &self.event_bus)
                    .await?)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G4 => {
                if !blocking {
                    self.motion_planner
                        .defer_channel
                        .send(DeferEvent::AwaitRequested(DeferAction::Dwell, channel))
                        .await;
                }
                Ok(self
                    .motion_planner
                    .plan(channel, &gc, blocking, &self.event_bus)
                    .await?)
            }
            GCodeValue::G10 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::G17 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::G21 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G28(_) => match self.event_bus.has_flags(EventFlags::HOMING).await {
                true => Err(CodeExecutionFailure::BUSY),
                false => {
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
                        .plan(channel, &gc, blocking, &self.event_bus)
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
                    if !self
                        .event_bus
                        .get_status()
                        .await
                        .contains(EventFlags::ATX_ON)
                    {
                        return Err(CodeExecutionFailure::PowerRequired);
                    }
                    let md = self.motion_planner.motion_driver.lock().await;
                    md.probe_controller.lock().await.probe_pin_up(300_000).await;
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            #[cfg(feature = "with-probe")]
            GCodeValue::G32 => {
                if self.event_bus.has_flags(EventFlags::HOMING).await {
                    Err(CodeExecutionFailure::BUSY)
                } else {
                    if !self
                        .event_bus
                        .get_status()
                        .await
                        .contains(EventFlags::ATX_ON)
                    {
                        return Err(CodeExecutionFailure::PowerRequired);
                    }
                    let md = self.motion_planner.motion_driver.lock().await;
                    md.probe_controller
                        .lock()
                        .await
                        .probe_pin_down(300_000)
                        .await;
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            GCodeValue::G80 => Ok(CodeExecutionSuccess::OK),
            #[cfg(feature = "with-motion")]
            GCodeValue::G90 => {
                self.motion_planner.set_absolute_positioning(true).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G91 => {
                self.motion_planner.set_absolute_positioning(false).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G92(_pos) => {
                self.motion_planner
                    .set_last_planned_pos(&TVector {
                        x: _pos.x,
                        y: _pos.y,
                        z: _pos.z,
                        e: _pos.e,
                    })
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
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
            GCodeValue::M73 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M79 => {
                let _ = self.write(channel, "echo: Software reset\n").await;
                self.flush(channel).await;
                hwa::sys_reset();
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M80 => {
                hwa::info!("Received PowerOn");
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-ps-on")] {
                        self.ps_on.lock().await.set_high();
                        embassy_time::Timer::after_millis(500).await;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-trinamic")] {
                        let _ = self.motion_planner.motion_driver.lock().await.trinamic_controller.init().await.is_ok();
                    }
                }
                self.event_bus
                    .publish_event(EventStatus::containing(EventFlags::ATX_ON))
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M81 => {
                hwa::info!("Received PowerOff");
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-ps-on")] {
                        self.ps_on.lock().await.set_low();
                    }
                }
                self.event_bus
                    .publish_event(EventStatus::not_containing(EventFlags::ATX_ON))
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M83 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M84 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M100 => {
                let heap_current = hwa::mem::heap_current_size();
                let heap_max = hwa::mem::heap_max_size();
                let stack_current = hwa::mem::stack_reservation_current_size();
                let stack_max = hwa::mem::stack_reservation_max_size();

                let z1 = alloc::format!(
                    "echo: {}/{} {}% of heap usage\n",
                    heap_current,
                    heap_max,
                    math::Real::from_f32(100.0f32 * heap_current as f32 / heap_max as f32),
                );
                self.write(channel, z1.as_str()).await;

                let z2 = alloc::format!(
                    "echo: {}/{} {}% of stack reservation\n",
                    stack_current,
                    stack_max,
                    math::Real::from_f32(100.0f32 * stack_current as f32 / stack_max as f32),
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
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let val = s.s.and_then(|v| v.to_i32()).unwrap_or(0);
                let mut h = self.hotend.lock().await;
                h.set_target_temp(
                    CommChannel::Internal,
                    DeferAction::HotEndTemperature,
                    val as f32,
                )
                .await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M105 => {
                let hotend_temp_report = {
                    #[cfg(feature = "with-hot-end")]
                    {
                        let mut h = self.hotend.lock().await;
                        alloc::format!(
                            " T:{} /{} T@:{} TZ:{}",
                            math::Real::from_f32(h.get_current_temp()),
                            math::Real::from_f32(h.get_target_temp()),
                            math::Real::from_f32(h.get_current_power().await),
                            math::Real::from_f32(h.get_current_resistance()),
                        )
                    }
                    #[cfg(not(feature = "with-hot-end"))]
                    alloc::string::String::new()
                };
                let hotbed_temp_report = {
                    #[cfg(feature = "with-hot-bed")]
                    {
                        let mut h = self.hotbed.lock().await;
                        alloc::format!(
                            " B:{} /{} B@:{} BZ:{}",
                            math::Real::from_f32(h.get_current_temp()),
                            math::Real::from_f32(h.get_target_temp()),
                            math::Real::from_f32(h.get_current_power().await),
                            math::Real::from_f32(h.get_current_resistance()),
                        )
                    }
                    #[cfg(not(feature = "with-hot-bed"))]
                    alloc::string::String::new()
                };

                let report = alloc::format!(
                    "ok{}{}\n", hotend_temp_report, hotbed_temp_report
                );
                let _ = self.write(channel, report.as_str()).await;
                Ok(CodeExecutionSuccess::CONSUMED)
            }
            #[cfg(feature = "with-fan-layer")]
            GCodeValue::M106 => {
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                //crate::info!("M106 BEGIN");
                self.fan_layer.lock().await.set_power(255).await;
                //crate::info!("M106 END");
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-fan-layer")]
            GCodeValue::M107 => {
                self.fan_layer.lock().await.set_power(0).await;
                Ok(CodeExecutionSuccess::OK)
            }
            // Wait for hot-end temperature
            // Mostly deferred code
            #[cfg(feature = "with-hot-end")]
            GCodeValue::M109(s) => {
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let deferred = {
                    let mut he = self.hotend.lock().await;
                    let value = s.s.and_then(|v| v.to_i32()).unwrap_or(0);
                    let was_deferred = he
                        .set_target_temp(channel, DeferAction::HotEndTemperature, value as f32)
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
                let _pos = self
                    .motion_planner
                    .get_last_planned_pos()
                    .await
                    .unwrap_or(TVector::zero());
                let _spos = self
                    .motion_planner
                    .get_last_planned_real_pos()
                    .await
                    .unwrap_or(TVector::zero());
                let z = alloc::format!(
                    "X:{} Y:{} Z:{} E:{} Count X:{} Y:{} Z:{}\n",
                    _pos.x.unwrap_or(math::ZERO),
                    _pos.y.unwrap_or(math::ZERO),
                    _pos.z.unwrap_or(math::ZERO),
                    _pos.e.unwrap_or(math::ZERO),
                    _spos.x.unwrap_or(math::ZERO),
                    _spos.y.unwrap_or(math::ZERO),
                    _spos.z.unwrap_or(math::ZERO),
                );
                let _ = self.write(channel, z.as_str()).await;

                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M115 => {
                let _ = self.write(channel, "echo: FIRMWARE_NAME: ").await;
                let _ = self.write(channel, MACHINE_INFO.firmware_name).await;
                let _ = self.write(channel, " FIRMWARE_VERSION: ").await;
                let _ = self.write(channel, MACHINE_INFO.firmware_version).await;
                let _ = self.write(channel, " FIRMWARE_URL: ").await;
                let _ = self.write(channel, MACHINE_INFO.firmware_url).await;
                let _ = self.write(channel, " MACHINE_TYPE: ").await;
                let _ = self.write(channel, MACHINE_INFO.machine_type).await;
                let _ = self.write(channel, " MACHINE_BOARD: ").await;
                let _ = self.write(channel, MACHINE_INFO.machine_board).await;
                let _ = self.write(channel, " MACHINE_PROCESSOR: ").await;
                let _ = self.write(channel, MACHINE_INFO.machine_processor).await;
                let _ = self.write(channel, " MACHINE_UUID: ").await;
                let _ = self.write(channel, MACHINE_INFO.machine_uuid).await;
                let _ = self.write(channel, " EXTRUDER_COUNT: ").await;
                let _ = self
                    .write(
                        channel,
                        alloc::format!("{}\n", MACHINE_INFO.extruder_count).as_str(),
                    )
                    .await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M117 => {
                let _ = self.write(channel, "echo: (display msg)\n").await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M119 => {
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
            }
            // Set hot-bed temperature
            // An immediate command
            #[cfg(feature = "with-hot-bed")]
            GCodeValue::M140(s) => {
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let val = s.s.and_then(|v| v.to_i32()).unwrap_or(0);
                let mut h = self.hotbed.lock().await;
                h.set_target_temp(
                    CommChannel::Internal,
                    DeferAction::HotbedTemperature,
                    val as f32,
                )
                .await;
                Ok(CodeExecutionSuccess::OK)
            }
            // Wait for hot-bed temperature
            // A normally deferred command
            #[cfg(feature = "with-hot-bed")]
            GCodeValue::M190 => {
                if !self
                    .event_bus
                    .get_status()
                    .await
                    .contains(EventFlags::ATX_ON)
                {
                    return Err(CodeExecutionFailure::PowerRequired);
                }
                let deferred = {
                    let mut he = self.hotbed.lock().await;
                    he.ping_subscribe(channel, DeferAction::HotbedTemperature)
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
            GCodeValue::M201 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M203 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M204 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M205 => Ok(CodeExecutionSuccess::OK),
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
                        .motion_driver
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
            GCodeValue::M862_1 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M862_3 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M900 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M907 => Ok(CodeExecutionSuccess::OK),
            _ => Err(CodeExecutionFailure::NotYetImplemented),
        };
        result
    }
}

impl Drop for GCodeProcessor {
    fn drop(&mut self) {}
}
