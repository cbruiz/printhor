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

/// The set coordinates of a vector as GCode words (`X80.00 Y80.00 ...`), written straight into
/// the destination (no heap)
#[cfg(feature = "with-motion")]
struct AxisWords<'a>(&'a hwa::math::TVector<math::Real>);

#[cfg(feature = "with-motion")]
impl core::fmt::Display for AxisWords<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut result = Ok(());
        let mut first = true;
        self.0.foreach_values(|coord, value| {
            if result.is_ok() {
                let sep = if first { "" } else { " " };
                first = false;
                result = write!(f, "{}{}{:?}", sep, coord.name(), value.rdp(2));
            }
        });
        result
    }
}

/// `M503` setting: the G-code + per axis " X" + a number (16 chars is plenty for any setting)
const M503_LINE_CAPACITY: usize = {
    let per_axis = 18 * hwa::CoordSel::num_axis();
    if per_axis > 48 { 16 + per_axis } else { 64 }
};

/// M220 `B`/`R`: the backed up feed rate percentage (100 until the first `B`, as Marlin)
#[cfg(feature = "with-motion")]
static M220_BACKUP: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(100);

/// M220/M221 percentages are kept in a `u8`: clamped to [10, 255]
#[cfg(feature = "with-motion")]
fn clamp_rate(v: i32) -> u8 {
    v.clamp(10, 255) as u8
}

/// Longest `M503` section title (77 chars, the M204 one). Longer ones are truncated
const M503_TITLE_CAPACITY: usize = 80;

/// One `M503` section: "echo:; " + title + "\n" + "echo:  " + setting + "\n" (also fits a
/// comment section: "echo:; " + title + " " + value + "\n")
const M503_SECTION_CAPACITY: usize = 7 + M503_TITLE_CAPACITY + 1 + 7 + M503_LINE_CAPACITY + 1;

/// Formats one `M503` section (the setting line, preceded by its title line when `detailed`)
/// into a stack buffer, to be written with a single write: no heap, and a single buffer alive
/// across the write `.await`.
///
/// Written in pieces, a display (BTT TFT) can get the bare `echo:  ` prefix as a line of its own
/// (it frames the reception by line idle, and other tasks may write between the pieces) and pops
/// it up as an empty message.
fn m503_section(
    detailed: bool,
    title: &str,
    setting: core::fmt::Arguments<'_>,
) -> LineBuf<M503_SECTION_CAPACITY> {
    let mut out = LineBuf::new();
    if detailed {
        out.push_str("echo:; ");
        out.push_str(title);
        out.push_newline();
    }
    out.push_str("echo:  ");
    let _ = core::fmt::Write::write_fmt(&mut out, setting);
    out.push_newline();
    out
}

/// Formats a comment-only `M503` line (a setting without a G-code to restore it), for the
/// detailed report only: displays such as the BTT TFT pop up any `echo:` line that is neither a
/// `echo:;` comment nor a `echo:  G`/`echo:  M` setting. See [m503_section]
fn m503_comment(title: &str, value: core::fmt::Arguments<'_>) -> LineBuf<M503_SECTION_CAPACITY> {
    let mut out = LineBuf::new();
    out.push_str("echo:; ");
    out.push_str(title);
    out.push_str(" ");
    let _ = core::fmt::Write::write_fmt(&mut out, value);
    out.push_newline();
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

    /// `M800`: the motion execution metrics (see [hwa::controllers::motion_control::motion_metrics])
    #[cfg(feature = "with-motion")]
    async fn report_motion_metrics(&self, channel: CommChannel) {
        #[cfg(not(feature = "motion-metrics"))]
        self.write(channel, "echo:M800: build with the motion-metrics feature\n").await;
        #[cfg(feature = "motion-metrics")]
        {
            use hwa::controllers::motion_control::motion_metrics as mm;
            let m = mm::snapshot();
            let avg = |sum: u64, n: u32| if n > 0 { sum / n as u64 } else { 0 };
            // Per mille, written as a percentage with one decimal
            let permille = |part: u64, total: u64| if total > 0 { part * 1000 / total } else { 0 };
            let window_us = mm::now_us().saturating_sub(m.since_us);
            let stall_ms = m.stall_ticks as u64
                * crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US as u64
                / 1000;
            let isr_load = permille(m.isr_busy_us, window_us);
            let efficiency = permille(m.planned_us, m.wall_us);
            let avg_speed = if m.wall_us > 0 { m.distance_um * 1000 / m.wall_us } else { 0 };
            // One line at a time: only one buffer is alive across each write
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 window={}ms\n",
                    window_us / 1000
                ))).await;
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 isr ticks={} load={}.{}% max={}us stalls={}ms episodes={}\n",
                    m.isr_ticks, isr_load / 10, isr_load % 10, m.isr_max_us, stall_ms, m.stall_episodes
                ))).await;
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 segments n={} dist={}mm planned={}ms wall={}ms eff={}.{}%\n",
                    m.segments, m.distance_um / 1000, m.planned_us / 1000, m.wall_us / 1000,
                    efficiency / 10, efficiency % 10
                ))).await;
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 speed avg={}mm/s full_stops={} starved_gaps={} ({}ms)\n",
                    avg_speed, m.full_stops, m.starved_gaps, m.starved_us / 1000
                ))).await;
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 stepper profile avg={}us max={}us interp avg={}us max={}us push_wait={}ms\n",
                    avg(m.profile_us, m.segments), m.profile_max_us,
                    avg(m.interp_us, m.segments), m.interp_max_us, m.push_wait_us / 1000
                ))).await;
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 planner n={} lock avg={}us max={}us chain avg={}us max={}us\n",
                    m.plans, avg(m.plan_us, m.plans), m.plan_max_us,
                    avg(m.chain_us, m.plans), m.chain_max_us
                ))).await;
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 queue full_waits={} no_lookahead={} min_lookahead={}\n",
                    m.queue_full_waits, m.no_lookahead,
                    if m.min_lookahead == u8::MAX { 0 } else { m.min_lookahead }
                ))).await;
            self.write_line(channel, LineBuf::<128>::from_fmt(format_args!(
                    "echo:M800 slowdown n={} move_period avg={}us max={}us\n",
                    m.slowdowns, avg(m.move_period_us, m.move_periods), m.move_period_max_us
                ))).await;
        }
    }

    /// Writes a formatted line (the buffer only lives during the write)
    #[allow(unused)]
    async fn write_line<const N: usize>(&self, channel: CommChannel, line: LineBuf<N>) {
        self.write(channel, line.as_str()).await;
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

    /// Writes the reason of a command failure: to `channel`, or, when the command was run with no
    /// output (a print job runs with [CommChannel::Sink]), to every host channel (and the log), so
    /// that the reason of a job aborted by the failure is not lost.
    pub async fn write_failure_reason(&self, channel: CommChannel, msg: &str) {
        match channel {
            CommChannel::Sink => self.write_all(msg).await,
            _ => self.write(channel, msg).await,
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

    /// [EventFlags::TMC_OK] when something manages it (`with-trinamic`): only then it is cleared
    /// (without, it must stay set: nothing would set it again)
    #[allow(unused)]
    const TMC_OK_IF_MANAGED: EventFlags = {
        #[cfg(feature = "with-trinamic")]
        {
            EventFlags::TMC_OK
        }
        #[cfg(not(feature = "with-trinamic"))]
        {
            EventFlags::empty()
        }
    };

    /// Requires the machine to be ready for motion (moves, homing, probing): the power on (fails at
    /// once with [CodeExecutionFailure::PowerRequired] otherwise) and the drivers configured
    /// ([EventFlags::TMC_OK], always set without `with-trinamic`). The drivers are configured by
    /// their own task (e.g. right after `M80`), so they are waited for, with a timeout: on
    /// timeout (or in alarm mode) the command fails with [CodeExecutionFailure::PowerRequired].
    #[cfg(feature = "with-motion")]
    async fn require_motion_ready(&self, _channel: CommChannel) -> Result<(), CodeExecutionFailure> {
        self.require_power().await?;
        if crate::helpers::wait_for_flags_timeout(
            &self.event_bus,
            crate::helpers::MOTION_READY_FLAGS,
            embassy_time::Duration::from_millis(crate::helpers::MOTION_READY_TIMEOUT_MS),
        )
        .await
        .is_err()
        {
            let status = self.event_bus.get_status().await;
            if !status.contains(EventFlags::TMC_OK) {
                hwa::warn!("[GCodeProcessor] Motion rejected: TMC drivers not ready");
                self.write(_channel, "echo: TMC drivers not ready\n").await;
            }
            return Err(CodeExecutionFailure::PowerRequired);
        }
        Ok(())
    }

    /// Waits until every queued move has been executed
    #[cfg(feature = "with-motion")]
    /// Shifts the logical position (current and last planned) by `delta` (only the axes set):
    /// the machine does not move, its coordinates change. After the queued motion, which was
    /// planned in the former coordinates. Nothing to do when the position is unknown.
    #[cfg(feature = "with-motion")]
    async fn shift_logical_position(&self, order_num: u32, delta: &hwa::math::TVector<math::Real>) {
        let mut any = false;
        delta.foreach_values(|_, d| any |= !d.is_zero());
        let status = self.motion_planner.motion_status();
        if !any || !status.get_last_planned_position().is_set {
            return;
        }
        self.wait_for_motion_idle().await;
        let shift = |p: hwa::controllers::Position| {
            let mut world = p.world_pos;
            delta.foreach_values(|c, d| {
                if let Some(v) = world.get_coord(c) {
                    world.set_coord(c, Some(v + *d));
                }
            });
            hwa::controllers::Position::new_with_world_projection(&world)
        };
        status.update_last_planned_position(order_num, &shift(status.get_last_planned_position()));
        status.update_current_position(order_num, &shift(status.get_current_position()));
    }

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
    /// `M27` report (Marlin format): `SD printing byte <pos>/<size>`, or `Not SD printing`
    #[cfg(feature = "with-print-job")]
    pub async fn write_sd_status(&self, channel: CommChannel) {
        match hwa::controllers::JOB_PROGRESS.sd_bytes() {
            Some((pos, size)) => {
                let mut line = LineBuf::<SD_STATUS_CAPACITY>::new();
                line.push_str("SD printing byte ");
                line.push_u32(pos);
                line.push_str("/");
                line.push_u32(size);
                line.push_str("\n");
                let _ = self.write(channel, line.as_str()).await;
            }
            None => {
                let _ = self.write(channel, "Not SD printing\n").await;
            }
        }
    }

    /// Writes `prefix`, `value` and `suffix` (no heap)
    #[cfg(feature = "with-print-job")]
    pub async fn write_u32(&self, channel: CommChannel, prefix: &str, value: u32, suffix: &str) {
        let mut num = LineBuf::<{ I32_LEN }>::new();
        num.push_u32(value);
        let _ = self.write(channel, prefix).await;
        let _ = self.write(channel, num.as_str()).await;
        let _ = self.write(channel, suffix).await;
    }

    /// `M73` report: `echo: Progress: <p>%; Remaining: <r> min` (the unknown parts omitted)
    #[cfg(feature = "with-print-job")]
    pub async fn write_progress(&self, channel: CommChannel) {
        let progress = &hwa::controllers::JOB_PROGRESS;
        let mut line = LineBuf::<PROGRESS_CAPACITY>::new();
        line.push_str("echo: Progress:");
        match progress.percent() {
            Some(p) => {
                line.push_str(" ");
                line.push_u32(u32::from(p));
                line.push_str("%");
            }
            None => line.push_str(" -"),
        }
        if let Some(r) = progress.remaining_min() {
            line.push_str("; Remaining: ");
            line.push_u32(r);
            line.push_str(" min");
        }
        line.push_str("\n");
        let _ = self.write(channel, line.as_str()).await;
    }

    /// `G30`: single probe at the current position, or at `x`, `y` (absolute, where the probe
    /// goes). See [hwa::drivers::MotionDriver::probe_action].
    ///
    /// Only scalars are kept across the awaits (positions and offsets are read again when needed):
    /// this future is part of the command execution of every task that runs commands.
    #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
    async fn probe_single(
        &self,
        channel: CommChannel,
        order_num: u32,
        line_tag: Option<u32>,
        x: Option<math::Real>,
        y: Option<math::Real>,
    ) -> CodeExecutionResult {
        self.require_motion_ready(channel).await?;
        // After the queued motion (e.g. a G28 just sent): the position is known then
        self.wait_for_motion_idle().await;
        if self.event_bus.has_flags(EventFlags::HOMING).await {
            return Err(CodeExecutionFailure::BUSY);
        }
        if !self.motion_planner.motion_status().get_last_planned_position().is_set {
            return Err(CodeExecutionFailure::HomingRequired);
        }
        if x.is_some() || y.is_some() {
            // The nozzle goes to the point minus the probe offset (in the current G90/G91 mode)
            let status = self.motion_planner.motion_status();
            let offset = self.motion_planner.motion_config().get_probe_offset();
            let planned = status.get_last_planned_position().world_pos;
            let absolute = status.is_absolute_positioning();
            // As Marlin: a point the nozzle can not reach (soft end-stops) is not probed, instead
            // of probing elsewhere (the move would be clipped)
            let check_reach = status.soft_endstops() && status.is_homed();
            let (min, max) = self.motion_planner.motion_config().get_logical_travel();
            let mut target: hwa::math::TVector<math::Real> = hwa::math::TVector::new();
            for (coord, value) in [(Self::PROBE_X, x), (Self::PROBE_Y, y)] {
                if let (Some(coord), Some(v)) = (coord, value) {
                    let nozzle = v - offset.get_coord(coord).unwrap_or(math::ZERO);
                    let below = min.get_coord(coord).is_some_and(|low| nozzle < low);
                    let above = max.get_coord(coord).is_some_and(|high| nozzle > high);
                    if check_reach && (below || above) {
                        self.write(channel, "echo:?Position out of reach\n").await;
                        return Err(CodeExecutionFailure::ERR);
                    }
                    let current = planned.get_coord(coord).unwrap_or(math::ZERO);
                    target.set_coord(coord, Some(if absolute { nozzle } else { nozzle - current }));
                }
            }
            self.motion_planner
                .plan_rapid_move(channel, target, &self.event_bus, order_num, line_tag)
                .await?;
            self.wait_for_motion_idle().await;
        }
        // Down to the nozzle height where the probe triggered on the bed during the homing
        // (`-probe_offset.z` above the bed plane), plus some overtravel (the bed may be lower
        // elsewhere)
        let start_z = self.probe_nozzle_z();
        let trigger_plane = self.motion_planner.motion_config().get_logical_trigger_z();
        let max_descent = (start_z - trigger_plane).max(math::ZERO) + PROBE_OVERTRAVEL_WU;
        let result = self
            .motion_planner
            .motion_driver()
            .lock()
            .await
            .probe_action(&self.motion_planner.motion_config(), max_descent)
            .await;
        match result {
            Ok(descent) => {
                // Marlin semantics: the nozzle height at the trigger plus the probe Z offset, at
                // the probe XY. Same coordinates as M114 (the position is unchanged)
                let pos = self.motion_planner.motion_status().get_current_position().world_pos;
                let offset = self.motion_planner.motion_config().get_probe_offset();
                let at = |coord: Option<hwa::CoordSel>| match coord {
                    Some(c) => {
                        (pos.get_coord(c).unwrap_or(math::ZERO)
                            + offset.get_coord(c).unwrap_or(math::ZERO))
                        .to_f64() as f32
                    }
                    None => 0.0,
                };
                let mut line = LineBuf::<G30_REPORT_CAPACITY>::new();
                line.push_str("Bed X: ");
                line.push_fixed2(at(Self::PROBE_X));
                line.push_str(" Y: ");
                line.push_fixed2(at(Self::PROBE_Y));
                line.push_str(" Z: ");
                line.push_fixed2(
                    (start_z - descent + offset.get_coord(hwa::CoordSel::Z).unwrap_or(math::ZERO))
                        .to_f64() as f32,
                );
                line.push_str("\n");
                let _ = self.write(channel, line.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            Err(hwa::drivers::motion_driver::ProbeError::Aborted) => {
                let _ = self.write(channel, "echo: G30: probing aborted\n").await;
                Err(CodeExecutionFailure::ERR)
            }
            Err(hwa::drivers::motion_driver::ProbeError::NotTriggered) => {
                // A safeguard tripped (probe failure, or no bed below): alarm mode, recoverable
                // with M999 (and homing)
                let _ = self.write(channel, "Error:Probing failed\n").await;
                self.emergency_stop("Probing failed").await;
                Err(CodeExecutionFailure::ERR)
            }
        }
    }

    /// The X axis of the probe report, when enabled
    #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
    const PROBE_X: Option<hwa::CoordSel> = {
        #[cfg(feature = "with-x-axis")]
        {
            Some(hwa::CoordSel::X)
        }
        #[cfg(not(feature = "with-x-axis"))]
        {
            None
        }
    };

    /// The Y axis of the probe report, when enabled
    #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
    const PROBE_Y: Option<hwa::CoordSel> = {
        #[cfg(feature = "with-y-axis")]
        {
            Some(hwa::CoordSel::Y)
        }
        #[cfg(not(feature = "with-y-axis"))]
        {
            None
        }
    };

    /// The current nozzle Z (world units)
    #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
    fn probe_nozzle_z(&self) -> math::Real {
        self.motion_planner
            .motion_status()
            .get_current_position()
            .world_pos
            .z
            .unwrap_or(math::ZERO)
    }

    pub async fn quick_stop(&self) {
        crate::tasks::task_stepper::abort_motion(true);
        let _ = embassy_time::with_timeout(embassy_time::Duration::from_secs(5), async {
            self.wait_for_motion_idle().await;
            // Also the plans already handed to the step executor, while the abort is still set:
            // the tool actions among them are discarded instead of applied after the stop
            hwa::controllers::motion_control::STEP_DRIVER.flush().await;
        })
        .await;
        // The queued tool actions were discarded with the motion. A laser (or spindle) left on
        // over a still head burns a spot: it is turned off (and the coolant)
        self.cutter_off_now().await;
        let current = self.motion_planner.motion_status().get_current_position();
        self.motion_planner
            .motion_status()
            .update_last_planned_position(0, &current);
        crate::tasks::task_stepper::abort_motion(false);
    }

    /// A tool power word (`S` of M106, M3, M4) in [0, 255], full power by default, as a
    /// percentage [0, 100]
    fn power_percent(s: Option<math::Real>) -> u8 {
        let s = s.and_then(|v| v.to_i32()).unwrap_or(255).clamp(0, 255);
        ((s * 100 + 127) / 255) as u8
    }

    /// Tool actions (fans, laser/spindle, coolant). With motion, they are queued with it: they
    /// take effect right after the motion queued before them and right before the motion queued
    /// after them, without breaking the motion chaining nor delaying the motion (see
    /// `MotionPlanner::schedule_tool_action`). Without motion, they are applied right away.
    async fn queue_tool_actions(
        &self,
        _channel: CommChannel,
        _order_num: u32,
        actions: hwa::controllers::ToolActions,
    ) -> CodeExecutionResult {
        #[cfg(feature = "with-motion")]
        {
            self.motion_planner
                .schedule_tool_action(_channel, actions, &self.event_bus, _order_num)
                .await
        }
        #[cfg(not(feature = "with-motion"))]
        {
            #[cfg(feature = "with-fan-layer")]
            if let Some(power) = actions.fan_layer {
                self.fan_layer.lock().await.set_power(power);
            }
            #[cfg(feature = "with-fan-extra-1")]
            if let Some(power) = actions.fan_extra1 {
                FAN_EXTRA1_MANUAL_POWER.store(power, core::sync::atomic::Ordering::Relaxed);
                self.fan_extra1.lock().await.set_power(power);
            }
            #[cfg(feature = "with-laser")]
            if let Some(cutter) = actions.cutter {
                // Without motion, the dynamic power (M4) is always off
                let power = match cutter {
                    hwa::controllers::CutterState::CounterClockwise(_) => 0,
                    _ => cutter.power(),
                };
                self._laser.lock().await.set_power(power);
            }
            let _ = actions;
            Ok(CodeExecutionSuccess::OK)
        }
    }

    /// Turns the laser/spindle and the coolant off right away, out of the motion queue
    /// (emergency stop, M410)
    async fn cutter_off_now(&self) {
        #[cfg(feature = "with-motion")]
        hwa::controllers::motion_control::STEP_DRIVER
            .apply_tools_now(&hwa::controllers::ToolActions::CUTTER_AND_COOLANT_OFF);
        #[cfg(all(not(feature = "with-motion"), feature = "with-laser"))]
        self._laser.lock().await.set_power(0);
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
        // At once (and again by the quick stop, once the queued motion is discarded)
        self.cutter_off_now().await;
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
        // In pieces: no heap in the emergency path
        self.write_all("Error:").await;
        self.write_all(reason).await;
        self.write_all("\nError:Printer halted. kill() called!\n").await;
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
            GCodeValue::M114 | GCodeValue::M119 | GCodeValue::M800(_) | GCodeValue::M801 => true,
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
        use core::fmt::Write;
        let mut msg = LineBuf::<128>::new();
        let _ = match err {
            hwa::controllers::HeaterError::Faulted(f) => write!(
                msg,
                "echo: Heater in fault state ({}). Send M999 to clear\n",
                f.description()
            ),
            hwa::controllers::HeaterError::TargetTooHigh => write!(
                msg,
                "echo: Target temperature too high (max: {:?})\n",
                math::Real::from_f32(max)
            ),
        };
        self.write_failure_reason(channel, msg.as_str()).await;
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
            self.write_failure_reason(channel, "echo: Printer halted. Send M999 to resume\n")
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
                for piece in [
                    "[VER:1.1 ",
                    Contract::FIRMWARE_NAME,
                    " v",
                    Contract::FIRMWARE_VERSION,
                    ":]\n[MSG: Machine: ",
                    Contract::MACHINE_BOARD,
                    " ",
                    Contract::MACHINE_TYPE,
                    "]\n",
                ] {
                    self.write(channel, piece).await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::G => {
                for x in GCodeValue::VARIANTS.iter().filter(|x| x.starts_with("G")) {
                    self.write(channel, "echo: ").await;
                    self.write(channel, x).await;
                    self.write(channel, "\n").await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::G0(_) | GCodeValue::G1(_) => {
                self.require_motion_ready(channel).await?;
                Ok(self
                    .motion_planner
                    .plan(channel, &gc, _blocking, &self.event_bus)
                    .await?)
            }
            #[cfg(feature = "with-arcs")]
            GCodeValue::G2(_) | GCodeValue::G3(_) => {
                self.require_motion_ready(channel).await?;
                Ok(self
                    .motion_planner
                    .plan(channel, &gc, _blocking, &self.event_bus)
                    .await?)
            }
            #[cfg(feature = "with-beziers")]
            GCodeValue::G5(_) => {
                self.require_motion_ready(channel).await?;
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
                self.require_motion_ready(channel).await?;
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
                    self.require_motion_ready(channel).await?;
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
            #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
            GCodeValue::G30(_xy) => {
                #[cfg(feature = "with-x-axis")]
                let x = _xy.x;
                #[cfg(not(feature = "with-x-axis"))]
                let x = None;
                #[cfg(feature = "with-y-axis")]
                let y = _xy.y;
                #[cfg(not(feature = "with-y-axis"))]
                let y = None;
                self.probe_single(channel, gc.order_num, gc.line_tag, x, y).await
            }
            #[cfg(feature = "with-probe")]
            GCodeValue::G31 => {
                if self.event_bus.has_flags(EventFlags::HOMING).await {
                    Err(CodeExecutionFailure::BUSY)
                } else {
                    self.require_motion_ready(channel).await?;
                    self.probe
                        .lock()
                        .await
                        .probe_pin_up(hwa::drivers::motion_driver::PROBE_SETTLE_US)
                        .await;
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            #[cfg(feature = "with-probe")]
            GCodeValue::G32 => {
                if self.event_bus.has_flags(EventFlags::HOMING).await {
                    Err(CodeExecutionFailure::BUSY)
                } else {
                    self.require_motion_ready(channel).await?;
                    let mut md = self.probe.lock().await;
                    md.probe_deploy_latched(
                        hwa::drivers::motion_driver::PROBE_SETTLE_US,
                        hwa::drivers::motion_driver::PROBE_SW_MODE_SETTLE_US,
                    ).await;
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
                        let msg = LineBuf::<128>::from_fmt(format_args!(
                            "echo: M280 P0 duty: {} max: {} pulse: {}us\n",
                            duty,
                            max,
                            duty as u64 * 20_000 / max.max(1) as u64
                        ));
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
                    .probe_deploy_latched(
                        hwa::drivers::motion_driver::PROBE_SETTLE_US,
                        hwa::drivers::motion_driver::PROBE_SW_MODE_SETTLE_US,
                    )
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
            // The planner's answer is returned as is: when the position change fills the queue,
            // it is DEFERRED (answered by the defer task once a slot is freed). Answering OK here
            // as well gave the host two `ok` for one command
            GCodeValue::G92(_pos) => Ok(self
                .motion_planner
                .plan(channel, &gc, _blocking, &self.event_bus)
                .await?),
            #[cfg(feature = "with-motion")]
            GCodeValue::G94 => Ok(CodeExecutionSuccess::OK),
            GCodeValue::M => {
                for x in GCodeValue::VARIANTS.iter().filter(|x| x.starts_with("M")) {
                    self.write(channel, "echo: ").await;
                    self.write(channel, x).await;
                    self.write(channel, "\n").await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            // Laser/spindle and coolant: queued with the motion
            GCodeValue::M3(p) => {
                let power = Self::power_percent(p.s);
                self.queue_tool_actions(channel, gc.order_num, hwa::controllers::ToolActions {
                    cutter: Some(hwa::controllers::CutterState::Clockwise(power)),
                    ..hwa::controllers::ToolActions::NONE
                })
                .await
            }
            GCodeValue::M4(p) => {
                let power = Self::power_percent(p.s);
                self.queue_tool_actions(channel, gc.order_num, hwa::controllers::ToolActions {
                    cutter: Some(hwa::controllers::CutterState::CounterClockwise(power)),
                    ..hwa::controllers::ToolActions::NONE
                })
                .await
            }
            GCodeValue::M5 => {
                self.queue_tool_actions(channel, gc.order_num, hwa::controllers::ToolActions {
                    cutter: Some(hwa::controllers::CutterState::Off),
                    ..hwa::controllers::ToolActions::NONE
                })
                .await
            }
            GCodeValue::M7 => {
                self.queue_tool_actions(channel, gc.order_num, hwa::controllers::ToolActions {
                    coolant_mist: Some(true),
                    ..hwa::controllers::ToolActions::NONE
                })
                .await
            }
            GCodeValue::M8 => {
                self.queue_tool_actions(channel, gc.order_num, hwa::controllers::ToolActions {
                    coolant_flood: Some(true),
                    ..hwa::controllers::ToolActions::NONE
                })
                .await
            }
            GCodeValue::M9 => {
                self.queue_tool_actions(channel, gc.order_num, hwa::controllers::ToolActions {
                    coolant_mist: Some(false),
                    coolant_flood: Some(false),
                    ..hwa::controllers::ToolActions::NONE
                })
                .await
            }
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
            GCodeValue::M73(_p) => {
                #[cfg(feature = "with-print-job")]
                {
                    let percent = _p.p.map(|v| v.int().clamp(0, 100) as u8);
                    let remaining = _p.r.map(|v| v.int().clamp(0, i64::from(u32::MAX - 1)) as u32);
                    if percent.is_none() && remaining.is_none() {
                        self.write_progress(channel).await;
                    } else {
                        hwa::controllers::JOB_PROGRESS.set_slicer_progress(percent, remaining);
                    }
                }
                Ok(CodeExecutionSuccess::OK)
            }
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
                        // The drivers reset while their motor supply (VM) ramps up: they are
                        // configured once it is stable (Marlin's PSU_POWERUP_DELAY)
                        embassy_time::Timer::after_millis(PSU_POWERUP_DELAY_MS).await;
                    }
                }
                // The drivers reset while their supply ramps up: the power is reported on with the
                // drivers to be (re)configured, which their task does (M80 again only retries the
                // configuration). The answer waits for it (bounded) to report the result: when a
                // driver can not be configured, M80 fails and the motion is rejected (TMC_OK is
                // missing) while the task keeps retrying
                #[cfg(feature = "with-trinamic")]
                crate::tasks::task_trinamic::request_configuration();
                #[cfg(feature = "with-trinamic")]
                self.event_bus
                    .publish_event(
                        EventStatus::containing(EventFlags::ATX_ON)
                            .and_not_containing(EventFlags::TMC_OK),
                    )
                    .await;
                #[cfg(not(feature = "with-trinamic"))]
                self.event_bus
                    .publish_event(EventStatus::containing(EventFlags::ATX_ON))
                    .await;
                // In alarm mode they are configured after M999 (the motion is rejected meanwhile)
                #[cfg(feature = "with-trinamic")]
                if !self.event_bus.has_flags(EventFlags::SYS_ALARM).await
                    && crate::helpers::wait_for_flags_timeout(
                        &self.event_bus,
                        EventFlags::TMC_OK,
                        embassy_time::Duration::from_millis(crate::helpers::MOTION_READY_TIMEOUT_MS),
                    )
                    .await
                    .is_err()
                {
                    self.write(channel, "Error:TMC drivers not configured\n").await;
                    return Err(CodeExecutionFailure::ERR);
                }
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
                // The drivers lose their configuration without power
                self.event_bus
                    .publish_event(EventStatus::not_containing(
                        EventFlags::ATX_ON | Self::TMC_OK_IF_MANAGED,
                    ))
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
                use core::fmt::Write;
                // Heap-free: the report must work also with the heap exhausted
                let mut z1 = LineBuf::<96>::new();
                if Contract::MAX_HEAP_SIZE_BYTES > 0 {
                    let _ = write!(
                        z1,
                        "echo: {}/{} bytes ({:?}%) of heap usage\n",
                        heap_current,
                        Contract::MAX_HEAP_SIZE_BYTES,
                        math::Real::from_f32(
                            100.0f32 * heap_current as f32 / Contract::MAX_HEAP_SIZE_BYTES as f32
                        ),
                    );
                } else {
                    let _ = write!(z1, "echo: {} bytes of heap usage\n", heap_current);
                }
                self.write(channel, z1.as_str()).await;

                let mut z2 = LineBuf::<96>::new();
                let _ = write!(
                    z2,
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
                let mut z3 = LineBuf::<192>::new();
                let _ = write!(z3, "echo: Status: {:?}", status);
                self.write(channel, z3.as_str()).await;
                self.write(channel, "\n").await;
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
                // Hosts poll M105 every few seconds, also while moving or homing. The report is
                // built in a stack buffer with integer arithmetic (no heap, no float formatting
                // machinery), which matters on FPU-less cores (e.g. the Cortex-M0+ of the SKR)
                let mut report = LineBuf::<M105_CAPACITY>::new();
                report.push_str("ok");
                #[cfg(feature = "with-hot-end")]
                {
                    let mut h = self.hot_end.lock().await;
                    let (temp, target, power) =
                        (h.get_current_temp(), h.get_target_temp(), h.get_current_power());
                    let _resistance = h.get_current_resistance();
                    drop(h);
                    report.push_str(" T:");
                    report.push_fixed2(temp);
                    report.push_str(" /");
                    report.push_fixed2(target);
                    report.push_str(" T@:");
                    report.push_fixed2(power);
                    #[cfg(not(feature = "marlin-compat"))]
                    {
                        report.push_str(" TZ:");
                        report.push_fixed2(_resistance);
                    }
                }
                #[cfg(not(feature = "with-hot-end"))]
                report.push_str(" T:0 /0 T@:0");
                #[cfg(feature = "with-hot-bed")]
                {
                    let mut h = self.hot_bed.lock().await;
                    let (temp, target, power) =
                        (h.get_current_temp(), h.get_target_temp(), h.get_current_power());
                    let _resistance = h.get_current_resistance();
                    drop(h);
                    report.push_str(" B:");
                    report.push_fixed2(temp);
                    report.push_str(" /");
                    report.push_fixed2(target);
                    report.push_str(" B@:");
                    report.push_fixed2(power);
                    #[cfg(not(feature = "marlin-compat"))]
                    {
                        report.push_str(" BZ:");
                        report.push_fixed2(_resistance);
                    }
                }
                #[cfg(not(feature = "with-hot-bed"))]
                report.push_str(" B:0 /0 B@:0");
                report.push_str("\n");
                let _ = self.write(channel, report.as_str()).await;
                Ok(CodeExecutionSuccess::CONSUMED)
            }
            // Available with any fan. The layer fan (P0) of a machine without it is ignored (sliced
            // files always set it: the job must go on), an unknown index is rejected. Queued with
            // the motion (see `queue_tool_actions`)
            #[cfg(any(feature = "with-fan-layer", feature = "with-fan-extra-1"))]
            GCodeValue::M106(_fan) => {
                // No power required (as Marlin): only the duty is set, the fan spins once the ATX
                // power is on
                // S in [0, 255] (default: full power) mapped to the PWM scale [0, 100]
                let _percent = Self::power_percent(_fan.s);
                #[allow(unused_mut)]
                let mut actions = hwa::controllers::ToolActions::NONE;
                match _fan.p.and_then(|v| v.to_i32()).unwrap_or(0) {
                    #[cfg(feature = "with-fan-layer")]
                    0 => actions.fan_layer = Some(_percent),
                    #[cfg(not(feature = "with-fan-layer"))]
                    0 => hwa::debug!("M106: no layer fan (ignored)"),
                    #[cfg(feature = "with-fan-extra-1")]
                    1 => actions.fan_extra1 = Some(_percent),
                    _ => {
                        self.write(channel, "echo: M106: unknown fan index\n").await;
                        return Err(CodeExecutionFailure::ERR);
                    }
                }
                self.queue_tool_actions(channel, gc.order_num, actions).await
            }
            #[cfg(any(feature = "with-fan-layer", feature = "with-fan-extra-1"))]
            GCodeValue::M107 => {
                // No layer fan: nothing to turn off (sliced files always send it)
                #[cfg(not(feature = "with-fan-layer"))]
                hwa::debug!("M107: no layer fan (ignored)");
                self.queue_tool_actions(channel, gc.order_num, hwa::controllers::ToolActions {
                    #[cfg(feature = "with-fan-layer")]
                    fan_layer: Some(0),
                    ..hwa::controllers::ToolActions::NONE
                })
                .await
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
                let step_pos: hwa::math::TVector<i32> = (_rpos.world_pos
                    * self
                        .motion_planner
                        .motion_config()
                        .get_steps_per_space_unit_as_vector())
                .map_values(|_c, _v| Some(_v.to_i32().unwrap_or(0)));
                // Only the enabled axes, in Marlin order: the geometric ones (X, Y, Z, A, B, C,
                // I...W) and then E. Built in a stack buffer sized for the enabled axes (no heap)
                #[cfg(feature = "with-e-axis")]
                let logical_e = self.motion_planner.motion_status().get_logical_e();
                let axes = || {
                    let geometric = hwa::CoordSel::motion_relevant_axis().iter();
                    #[cfg(feature = "with-e-axis")]
                    return geometric.chain(core::iter::once(hwa::CoordSel::E));
                    #[cfg(not(feature = "with-e-axis"))]
                    geometric
                };
                let mut line = LineBuf::<M114_CAPACITY>::new();
                for (i, coord) in axes().enumerate() {
                    #[cfg(feature = "with-e-axis")]
                    let pos = if coord == hwa::CoordSel::E {
                        logical_e
                    } else {
                        _pos.world_pos.get_coord(coord).unwrap_or(math::ZERO)
                    };
                    #[cfg(not(feature = "with-e-axis"))]
                    let pos = _pos.world_pos.get_coord(coord).unwrap_or(math::ZERO);
                    if i > 0 {
                        line.push_str(" ");
                    }
                    line.push_str(coord.name());
                    line.push_str(":");
                    line.push_fixed2(pos.to_f64() as f32);
                }
                line.push_str(" Count:");
                for coord in axes() {
                    line.push_str(" ");
                    line.push_str(coord.name());
                    line.push_str(":");
                    line.push_i32(step_pos.get_coord(coord).unwrap_or(0));
                }
                line.push_str("\n");
                let _ = self.write(channel, line.as_str()).await;
                // Motor (space) coordinates: debug aid only (heap allocated, multi-line)
                #[cfg(feature = "debug-position")]
                {
                    let z2 = alloc::format!("echo: Space: {:#?}\n", _rpos.space_pos);
                    let _ = self.write(channel, z2.as_str()).await;
                }
                let _ = self.write(channel, "ok\n").await;
                Ok(CodeExecutionSuccess::CONSUMED)
            }
            #[cfg(feature = "marlin-compat")]
            GCodeValue::M115 => {
                // Marlin protocol format (no `echo:` prefix, no space after the colons), but the
                // firmware identifies itself: its own name and the version it was compiled from
                // Written in pieces: a formatted line (~300 bytes) does not fit in the heap of the
                // smaller boards (512 bytes on the SKR Mini E3), and the TFTs send M115 at boot
                for piece in [
                    "FIRMWARE_NAME:",
                    hwa::Contract::FIRMWARE_NAME,
                    " ",
                    FIRMWARE_VERSION,
                    " SOURCE_CODE_URL:",
                    hwa::Contract::FIRMWARE_URL,
                    " PROTOCOL_VERSION:1.0 MACHINE_TYPE:",
                    hwa::Contract::MACHINE_TYPE,
                    " EXTRUDER_COUNT:",
                    hwa::Contract::EXTRUDER_COUNT,
                    " UUID:",
                    hwa::Contract::MACHINE_UUID,
                    "\n",
                ] {
                    let _ = self.write(channel, piece).await;
                }
                // One line per capability, from the enabled features (no heap)
                for (name, enabled) in M115_CAPABILITIES {
                    let mut line = LineBuf::<M115_CAP_LINE_CAPACITY>::new();
                    line.push_str("Cap:");
                    line.push_str(name);
                    line.push_str(if *enabled { ":1\n" } else { ":0\n" });
                    let _ = self.write(channel, line.as_str()).await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(not(feature = "marlin-compat"))]
            GCodeValue::M115 => {
                let _ = self.write(channel, "echo: FIRMWARE_NAME: ").await;
                let _ = self.write(channel, hwa::Contract::FIRMWARE_NAME).await;
                let _ = self.write(channel, " FIRMWARE_VERSION: ").await;
                let _ = self.write(channel, FIRMWARE_VERSION).await;
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
                // An empty M117 only clears the status: nothing to show (an empty `echo:` line is
                // popped up as a blank message by the displays)
                if let Some(msg) = _msg.as_ref().map(|m| m.trim()).filter(|m| !m.is_empty()) {
                    // Marlin host status notification: the BTT TFT shows it in its status bar
                    #[cfg(feature = "marlin-compat")]
                    let prefix = "//action:notification ";
                    #[cfg(not(feature = "marlin-compat"))]
                    let prefix = "echo: ";
                    // Written in pieces: no heap
                    let _ = self.write(Contract::DISPLAY_CHANNEL, prefix).await;
                    let _ = self.write(Contract::DISPLAY_CHANNEL, msg).await;
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
                // A disabled axis has no end-stop: reported as not triggered
                let mg = self.motion_planner.motion_driver().lock().await;
                #[allow(unused_variables)]
                let triggered = |c: hwa::CoordSel| mg.step_actuator.end_stop_triggered(c);
                #[cfg(feature = "with-x-axis")]
                let x_endstop = triggered(hwa::CoordSel::X);
                #[cfg(not(feature = "with-x-axis"))]
                let x_endstop = false;
                #[cfg(feature = "with-y-axis")]
                let y_endstop = triggered(hwa::CoordSel::Y);
                #[cfg(not(feature = "with-y-axis"))]
                let y_endstop = false;
                #[cfg(feature = "with-z-axis")]
                let z_endstop = triggered(hwa::CoordSel::Z);
                #[cfg(not(feature = "with-z-axis"))]
                let z_endstop = false;
                drop(mg);
                // Marlin format: hosts and displays (BTT TFT) parse it
                #[cfg(feature = "marlin-compat")]
                {
                    let state = |t: bool| if t { "TRIGGERED\n" } else { "open\n" };
                    let mut r = LineBuf::<M119_CAPACITY>::new();
                    r.push_str("Reporting endstop status\n");
                    #[cfg(feature = "with-x-axis")]
                    { r.push_str("x_min: "); r.push_str(state(x_endstop)); }
                    #[cfg(feature = "with-y-axis")]
                    { r.push_str("y_min: "); r.push_str(state(y_endstop)); }
                    #[cfg(feature = "with-z-axis")]
                    { r.push_str("z_min: "); r.push_str(state(z_endstop)); }
                    let _ = self.write(channel, r.as_str()).await;
                    let _ = (x_endstop, y_endstop, z_endstop);
                }
                // Single write (no interleaving with other output) from a 17 bytes stack buffer
                #[cfg(not(feature = "marlin-compat"))]
                let mut line = *b"M119 X 0 Y 0 Z 0\n";
                #[cfg(not(feature = "marlin-compat"))]
                {
                    let bit = |t: bool| if t { b'1' } else { b'0' };
                    (line[7], line[11], line[15]) = (bit(x_endstop), bit(y_endstop), bit(z_endstop));
                    let _ = self
                        .write(channel, core::str::from_utf8(&line).unwrap_or(""))
                        .await;
                }
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
            GCodeValue::M205(_param) => {
                // X (or Y): the largest velocity change at a corner, as Marlin's classic jerk
                #[cfg(feature = "with-motion")]
                {
                    let v: hwa::math::TVector<math::Real> = _param.into();
                    if let Some(dv) = v
                        .get_coord(hwa::CoordSel::X)
                        .or(v.get_coord(hwa::CoordSel::Y))
                        .filter(|dv| !(*dv < math::ZERO))
                    {
                        self.motion_planner.motion_config().set_corner_speed_change(dv);
                    }
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M206(offset) => {
                // As Marlin: the logical coordinates shift right away (the machine does not move),
                // and the next homings keep the offset
                let v: hwa::math::TVector<math::Real> = offset.into();
                let cfg = self.motion_planner.motion_config();
                let before = cfg.get_home_offset();
                cfg.update_home_offset(&v);
                let delta = cfg.get_home_offset() - before;
                self.shift_logical_position(gc.order_num, &delta).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M851(offset) => {
                let v: hwa::math::TVector<math::Real> = offset.into();
                let cfg = self.motion_planner.motion_config();
                let mut given = false;
                v.foreach_values(|_, _| given = true);
                if !given {
                    // Marlin format: "echo:Probe Offset X0.00 Y0.00 Z-1.50"
                    let o = cfg.get_probe_offset();
                    let mut line = LineBuf::<96>::from("echo:Probe Offset");
                    o.selecting(hwa::CoordSel::motion_relevant_axis()).foreach_values(|c, v| {
                        line.push_str(" ");
                        line.push_str(c.name());
                        line.push_fixed2(v.to_f64() as f32);
                    });
                    line.push_str("\n");
                    self.write(channel, line.as_str()).await;
                } else {
                    // The positioning uses it from now on (G30, the next homing)
                    cfg.update_probe_offset(&v);
                }
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
            GCodeValue::M211(_s) => {
                // M211 S<0|1>: soft end-stops off/on (on by default: moves are clipped to the
                // travel of the axes, shifted by the M206 offsets). Reported in Marlin format (and
                // logical coordinates), so hosts/TFTs can parse it
                #[cfg(feature = "with-motion")]
                {
                    let status = self.motion_planner.motion_status();
                    if let Some(s) = _s.s.and_then(|v| v.to_i32()) {
                        status.set_soft_endstops(s != 0);
                    }
                    let (min, max) = self.motion_planner.motion_config().get_logical_travel();
                    // Written with push_fixed2: `{:.2}` would link core's float formatting (~10 KB)
                    let mut msg = LineBuf::<128>::new();
                    msg.push_str("echo:Soft endstops: ");
                    msg.push_str(if status.soft_endstops() { "On" } else { "Off" });
                    for (label, v) in [("  Min:", &min), ("  Max:", &max)] {
                        msg.push_str(label);
                        for (name, c) in [(" X", v.x), (" Y", v.y), (" Z", v.z)] {
                            msg.push_str(name);
                            msg.push_fixed2(c.unwrap_or(math::ZERO).to_f64() as f32);
                        }
                    }
                    msg.push_str("\n");
                    self.write(channel, msg.as_str()).await;
                }
                #[cfg(not(feature = "with-motion"))]
                self.write(channel, "echo:Soft endstops: Off\n").await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M220(_args) => {
                // Marlin semantics: no word reports `FR:<percentage>%`; otherwise, in this order,
                // `R` restores the backed up percentage, `B` backs up the one there was before,
                // and `S` sets a new one
                #[cfg(feature = "with-motion")]
                {
                    use core::sync::atomic::Ordering::Relaxed;
                    let cfg = self.motion_planner.motion_config();
                    if _args.s.is_none() && _args.b.is_none() && _args.r.is_none() {
                        let mut msg = LineBuf::<{ 3 + 3 + 2 }>::new();
                        msg.push_str("FR:");
                        msg.push_u32(u32::from(cfg.get_speed_rate()));
                        msg.push_str("%\n");
                        self.write(channel, msg.as_str()).await;
                    } else {
                        let current = cfg.get_speed_rate();
                        if _args.r.is_some() {
                            cfg.set_speed_rate(M220_BACKUP.load(Relaxed));
                        }
                        if _args.b.is_some() {
                            M220_BACKUP.store(current, Relaxed);
                        }
                        if let Some(v) = _args.s.and_then(|v| v.to_i32()) {
                            cfg.set_speed_rate(clamp_rate(v));
                        }
                    }
                }
                #[cfg(not(feature = "with-motion"))]
                let _ = _args;
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M221(_args) => {
                #[cfg(feature = "with-motion")]
                {
                    let cfg = self.motion_planner.motion_config();
                    // A single extruder (E0): any other one is reported as invalid (as Marlin)
                    match _args.t.and_then(|t| t.to_i32()).unwrap_or(0) {
                        0 => match _args.s.and_then(|v| v.to_i32()) {
                            Some(v) => cfg.set_flow_rate(clamp_rate(v)),
                            None => {
                                let mut msg = LineBuf::<{ 14 + 3 + 2 }>::new();
                                msg.push_str("echo:E0 Flow: ");
                                msg.push_u32(u32::from(cfg.get_flow_rate()));
                                msg.push_str("%\n");
                                self.write(channel, msg.as_str()).await;
                            }
                        },
                        t => {
                            let mut msg = LineBuf::<{ 27 + I32_LEN + 1 }>::new();
                            msg.push_str("echo:M221 Invalid extruder ");
                            msg.push_i32(t);
                            msg.push_str("\n");
                            self.write(channel, msg.as_str()).await;
                        }
                    }
                }
                #[cfg(not(feature = "with-motion"))]
                let _ = _args;
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
                            Some(t) => LineBuf::<128>::from_fmt(format_args!(
                                "echo: Cold extrudes are disabled (min temp {:?}C)\n",
                                math::Real::from_f32(t)
                            )),
                            None => LineBuf::<128>::from_fmt(format_args!("echo: Cold extrudes are enabled\n")),
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
                    let msg = LineBuf::<128>::from_fmt(format_args!("echo:Settings Stored ({} bytes; crc {})\n", size, crc));
                    self.write(channel, msg.as_str()).await;
                    Ok(CodeExecutionSuccess::OK)
                }
                Err(e) => {
                    let msg = LineBuf::<128>::from_fmt(format_args!("echo:EEPROM error: {}\n", e.description()));
                    self.write(channel, msg.as_str()).await;
                    Err(CodeExecutionFailure::ERR)
                }
            },
            #[cfg(feature = "with-eeprom")]
            GCodeValue::M501 => match crate::processing::settings_store::retrieve(self).await {
                Ok((size, crc)) => {
                    let msg = LineBuf::<128>::from_fmt(format_args!(
                        "echo:Stored settings retrieved ({} bytes; crc {})\n",
                        size,
                        crc
                    ));
                    self.write(channel, msg.as_str()).await;
                    Ok(CodeExecutionSuccess::OK)
                }
                Err(e) => {
                    let msg = LineBuf::<128>::from_fmt(format_args!("echo:EEPROM error: {}\n", e.description()));
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
                // The drivers are configured again (by their task) with the defaults. Waited for
                // (bounded) when they can be configured (powered, not in alarm mode)
                #[cfg(feature = "with-trinamic")]
                {
                    crate::tasks::task_trinamic::request_configuration();
                    self.event_bus
                        .publish_event(EventStatus::not_containing(EventFlags::TMC_OK))
                        .await;
                    if self.require_power().await.is_ok()
                        && !self.event_bus.has_flags(EventFlags::SYS_ALARM).await
                        && crate::helpers::wait_for_flags_timeout(
                            &self.event_bus,
                            EventFlags::TMC_OK,
                            embassy_time::Duration::from_millis(crate::helpers::MOTION_READY_TIMEOUT_MS),
                        )
                        .await
                        .is_err()
                    {
                        self.write(channel, "echo: M502 Trinamic init failed\n").await;
                    }
                }
                self.write(channel, "echo: Hardcoded Default Settings Loaded\n").await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M801 => {
                #[cfg(not(feature = "mcu-bench"))]
                self.write(channel, "echo:M801: build with the mcu-bench feature\n").await;
                #[cfg(feature = "mcu-bench")]
                for (name, value, unit) in crate::motion::mcu_bench::run().await.iter() {
                    if !name.is_empty() {
                        self.write_line(
                            channel,
                            LineBuf::<64>::from_fmt(format_args!("echo:M801 {}: {} {}\n", name, value, unit)),
                        )
                        .await;
                    }
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCodeValue::M800(_param) => {
                // J: S-curve jerk of the motion axes (the queued moves keep theirs)
                if let Some(j) = _param.j.filter(|j| j.is_defined_positive()) {
                    let cfg = self.motion_planner.motion_config();
                    let mut jerk = cfg.get_max_jerk();
                    jerk.assign_if_set(
                        hwa::CoordSel::motion_relevant_axis(),
                        &jerk.map_values(|_, _| Some(j)),
                    );
                    cfg.set_max_jerk(jerk);
                }
                if _param.s.is_some_and(|s| s.is_zero()) {
                    #[cfg(feature = "motion-metrics")]
                    hwa::controllers::motion_control::motion_metrics::reset();
                } else if _param.j.is_none() {
                    self.report_motion_metrics(channel).await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCodeValue::M503(_param) => {
                let detailed = !_param.s.unwrap_or(hwa::math::ONE).is_zero();
                self.write(channel, m503_section(detailed, "Units in mm", format_args!("G21")).as_str()).await;
                #[cfg(feature = "with-motion")]
                {
                    let cfg = self.motion_planner.motion_config();
                    let status = self.motion_planner.motion_status();
                    self.write(channel, m503_section(detailed,
                        "Steps per unit:",
                        format_args!("M92 {}", AxisWords(&cfg.get_steps_per_space_unit_as_vector())),
                    ).as_str()).await;
                    self.write(channel, m503_section(detailed,
                        "Maximum feedrates (units/s):",
                        format_args!("M203 {}", AxisWords(&cfg.get_max_speed())),
                    ).as_str()).await;
                    self.write(channel, m503_section(detailed,
                        "Maximum Acceleration (units/s2):",
                        format_args!("M201 {}", AxisWords(&cfg.get_max_accel())),
                    ).as_str()).await;
                    self.write(channel, m503_section(detailed,
                        "Acceleration (units/s2): P<print_accel> T<travel_accel> (0 = per axis limits)",
                        format_args!(
                            "M204 P{:?} T{:?}",
                            cfg.get_print_accel().unwrap_or(math::ZERO).rdp(2),
                            cfg.get_travel_accel().unwrap_or(math::ZERO).rdp(2)
                        ),
                    ).as_str()).await;
                    if detailed {
                        self.write(channel, m503_comment(
                            "Jerk (units/s3, S-curve profile):",
                            format_args!("{}", AxisWords(&cfg.get_max_jerk())),
                        ).as_str()).await;
                    }
                    #[cfg(feature = "with-probe")]
                    self.write(channel, m503_section(detailed,
                        "Z-Probe Offset (mm):",
                        format_args!(
                            "M851 {}",
                            AxisWords(&cfg.get_probe_offset().selecting(hwa::CoordSel::motion_relevant_axis()))
                        ),
                    ).as_str()).await;
                    self.write(channel, m503_section(detailed,
                        "Home offset:",
                        format_args!("M206 {}", AxisWords(&cfg.get_home_offset())),
                    ).as_str()).await;
                    self.write(channel, m503_section(detailed,
                        "Feedrate / Flow percentage:",
                        format_args!("M220 S{} ; M221 S{}", cfg.get_speed_rate(), cfg.get_flow_rate()),
                    ).as_str()).await;
                    self.write(channel, m503_section(detailed,
                        "Extruder mode:",
                        format_args!("{}", match status.is_relative_extrusion() {
                            true => "M83",
                            false => "M82",
                        }),
                    ).as_str()).await;
                    #[cfg(feature = "with-e-axis")]
                    {
                        let r = cfg.get_retraction();
                        self.write(channel, m503_section(detailed,
                            "Retract: S<length> F<units/m> Z<lift>",
                            format_args!(
                                "M207 S{:?} F{:?} Z{:?}",
                                r.length.rdp(2),
                                (r.feed_rate * math::Real::from_lit(60, 0)).rdp(2),
                                r.z_hop.rdp(2)
                            ),
                        ).as_str()).await;
                        self.write(channel, m503_section(detailed,
                            "Recover: S<length> F<units/m>",
                            format_args!(
                                "M208 S{:?} F{:?}",
                                r.recover_extra.rdp(2),
                                (r.recover_feed_rate * math::Real::from_lit(60, 0)).rdp(2)
                            ),
                        ).as_str()).await;
                    }
                }
                #[cfg(feature = "with-hot-end")]
                {
                    let (kp, ki, kd) = self.hot_end.lock().await.get_pid();
                    self.write(channel, m503_section(detailed,
                        "Hotend PID settings:",
                        format_args!(
                            "M301 P{:?} I{:?} D{:?}",
                            math::Real::from_f32(kp).rdp(2),
                            math::Real::from_f32(ki).rdp(3),
                            math::Real::from_f32(kd).rdp(2)
                        ),
                    ).as_str()).await;
                }
                #[cfg(feature = "with-hot-bed")]
                {
                    let (kp, ki, kd) = self.hot_bed.lock().await.get_pid();
                    self.write(channel, m503_section(detailed,
                        "Bed PID settings:",
                        format_args!(
                            "M304 P{:?} I{:?} D{:?}",
                            math::Real::from_f32(kp).rdp(2),
                            math::Real::from_f32(ki).rdp(3),
                            math::Real::from_f32(kd).rdp(2)
                        ),
                    ).as_str()).await;
                }
                #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
                self.write(channel, m503_section(detailed,
                    "Cold extrusion: S<min temp> P<1 = allowed>",
                    format_args!(
                        "M302 S{:?} P{}",
                        math::Real::from_f32(f32::from_bits(
                            COLD_EXTRUSION_MIN_TEMP.load(core::sync::atomic::Ordering::Relaxed)
                        )),
                        COLD_EXTRUSION_ALLOWED.load(core::sync::atomic::Ordering::Relaxed) as u8
                    ),
                ).as_str()).await;
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

/// Fixed-capacity, stack-allocated line buffer for status reports (`M105`, `M114`): no heap.
///
/// It is alive across the write `.await`, so it is part of the command future state: each report
/// sizes it to the exact worst case of its enabled fields (see [M105_CAPACITY], [M114_CAPACITY]).
/// Pushes beyond the capacity are truncated, never panic.
#[derive(Clone)]
pub struct LineBuf<const N: usize> {
    buf: [u8; N],
    len: u16,
}

/// Values are clamped to +/-999999.99 (in hundredths), so a fixed point number takes at most
/// `FIXED2_LEN` chars. The clamp is integer: 99999999 is not representable as `f32`
const MAX_HUNDREDTHS: i32 = 99_999_999;
/// "-999999.99"
const FIXED2_LEN: usize = 10;
/// "-2147483648"
const I32_LEN: usize = 11;

/// `M105`: "ok" + hot-end + hot-bed + "\n". Per heater: " T:" + " /" + " T@:"
/// (+ " TZ:" without marlin-compat) and their numbers
const M105_CAPACITY: usize = {
    let heater = if cfg!(feature = "marlin-compat") {
        3 + 2 + 4 + 3 * FIXED2_LEN
    } else {
        3 + 2 + 4 + 4 + 4 * FIXED2_LEN
    };
    2 + 2 * heater + 1
};

/// `M27`: "SD printing byte " + position + "/" + size + "\n" (u32 numbers: 10 digits)
#[cfg(feature = "with-print-job")]
const SD_STATUS_CAPACITY: usize = 17 + 10 + 1 + 10 + 1;

/// `M73`: "echo: Progress: " + "100%" + "; Remaining: " + minutes + " min" + "\n"
#[cfg(feature = "with-print-job")]
const PROGRESS_CAPACITY: usize = 16 + 4 + 13 + 10 + 4 + 1;

/// `M115` capabilities (Marlin names), each one enabled only when the firmware really supports it
/// with the enabled features
#[cfg(feature = "marlin-compat")]
const M115_CAPABILITIES: &[(&str, bool)] = &[
    // Host protocol
    ("SERIAL_XON_XOFF", false),
    ("BINARY_FILE_TRANSFER", false),
    // M112/M410 are processed in order: no out-of-band emergency parser
    ("EMERGENCY_PARSER", false),
    ("HOST_ACTION_COMMANDS", false),
    ("PROMPT_SUPPORT", false),
    // M500/M501
    ("EEPROM", cfg!(feature = "with-eeprom")),
    ("VOLUMETRIC", false),
    // No M154/M155
    ("AUTOREPORT_POS", false),
    ("AUTOREPORT_TEMP", false),
    // M27 (SD byte progress) and M73 (slicer progress)
    ("PROGRESS", cfg!(feature = "with-print-job")),
    ("BUILD_PERCENT", cfg!(feature = "with-print-job")),
    ("PRINT_JOB", cfg!(feature = "with-print-job")),
    // G29 is accepted without effect
    ("AUTOLEVEL", false),
    ("LEVELING_DATA", false),
    ("RUNOUT", false),
    // G30 single probe (also M280/M401/M402 and G31/G32)
    ("Z_PROBE", cfg!(all(feature = "with-probe", feature = "with-z-axis"))),
    // M80/M81
    ("SOFTWARE_POWER", cfg!(feature = "with-ps-on")),
    ("TOGGLE_LIGHTS", false),
    ("CASE_LIGHT_BRIGHTNESS", false),
    // M20/M21/M23...
    ("SDCARD", cfg!(feature = "with-sd-card")),
    ("MULTI_VOLUME", false),
    ("REPEAT", false),
    // No M28
    ("SD_WRITE", false),
    // M27 S
    ("AUTOREPORT_SD_STATUS", cfg!(feature = "with-print-job")),
    // No M33
    ("LONG_FILENAME", false),
    // Heater safeguards (thermal runaway, heating failed, MINTEMP/MAXTEMP)
    (
        "THERMAL_PROTECTION",
        cfg!(any(feature = "with-hot-end", feature = "with-hot-bed")),
    ),
    // G80 is accepted only
    ("MOTION_MODES", false),
    // G2/G3
    ("ARCS", cfg!(feature = "with-arcs")),
    ("BABYSTEPPING", false),
    ("CHAMBER_TEMPERATURE", false),
    ("COOLER_TEMPERATURE", false),
    ("MEATPACK", false),
];

/// "Cap:" + the longest name + ":1\n"
#[cfg(feature = "marlin-compat")]
const M115_CAP_LINE_CAPACITY: usize = 4 + 21 + 3;

/// Wait after turning the power supply on (M80) before configuring the drivers (Marlin's
/// `PSU_POWERUP_DELAY`). Overridable at build time
#[cfg(all(feature = "with-ps-on", not(test)))]
#[const_env::env_item("PSU_POWERUP_DELAY_MS")]
const PSU_POWERUP_DELAY_MS: u64 = 500;
/// Shorter in the tests: the simulated supply has no ramp up, and every scenario powers on
#[cfg(all(feature = "with-ps-on", test))]
const PSU_POWERUP_DELAY_MS: u64 = 50;

/// `M119` (marlin-compat): "Reporting endstop status\n" + per axis "x_min: TRIGGERED\n"
#[cfg(all(feature = "with-motion", feature = "marlin-compat"))]
const M119_CAPACITY: usize = 25 + 3 * 17;

/// `G30`: probing overtravel below the plane where the homing found the bed (world units)
#[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
const PROBE_OVERTRAVEL_WU: math::Real = hwa::make_real!(5.0);

/// `G30`: "Bed X: " + n + " Y: " + n + " Z: " + n + "\n"
#[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
const G30_REPORT_CAPACITY: usize = 7 + FIXED2_LEN + 4 + FIXED2_LEN + 4 + FIXED2_LEN + 1;

/// The version of the firmware, as compiled (the version of this crate)
const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");

/// `M114`: per enabled axis (E included) " X:" + position and " X:" + step count, plus
/// " Count:" and "\n"
#[cfg(feature = "with-motion")]
const M114_CAPACITY: usize = {
    let axes = hwa::CoordSel::all_axis().bits().count_ones() as usize;
    axes * (3 + FIXED2_LEN) + 7 + axes * (3 + I32_LEN) + 1
};

impl<const N: usize> LineBuf<N> {
    /// Formats into a new buffer: `LineBuf::<N>::from_fmt(format_args!(...))`, the heap-free
    /// `alloc::format!`
    pub(crate) fn from_fmt(args: core::fmt::Arguments<'_>) -> Self {
        let mut line = Self::new();
        let _ = core::fmt::Write::write_fmt(&mut line.cursor(), args);
        line
    }

    pub(crate) const fn new() -> Self {
        Self {
            buf: [0u8; N],
            len: 0,
        }
    }

    /// The (non generic) writer of the buffer. Every method delegates to it, so that the code is
    /// not instantiated once per capacity (a dozen of them)
    #[inline(always)]
    fn cursor(&mut self) -> LineCursor<'_> {
        LineCursor {
            buf: &mut self.buf,
            len: &mut self.len,
        }
    }

    #[inline(always)]
    pub(crate) fn push_str(&mut self, s: &str) {
        self.cursor().push_str(s)
    }

    /// Appends `v` with two decimals (e.g. `-12.30`). NaN (sensor error) is written as `NaN`.
    /// Out of range values saturate to +/-999999.99.
    #[allow(dead_code)]
    #[inline(always)]
    fn push_fixed2(&mut self, v: f32) {
        self.cursor().push_fixed2(v)
    }

    /// Appends `v` as a plain integer
    #[allow(dead_code)]
    #[inline(always)]
    fn push_u32(&mut self, v: u32) {
        self.cursor().push_digits(false, v, false)
    }

    /// Appends `v` as a plain integer
    #[allow(dead_code)]
    #[inline(always)]
    pub(crate) fn push_i32(&mut self, v: i32) {
        self.cursor().push_digits(v < 0, v.unsigned_abs(), false)
    }

    /// Ends the line with "\n". When full, its last chars are dropped to make room: a line never
    /// loses its end (it would be merged with the next one)
    fn push_newline(&mut self) {
        let s = self.as_str();
        let mut end = s.len().min(N.saturating_sub(1));
        while !s.is_char_boundary(end) {
            end -= 1;
        }
        self.len = end as u16;
        self.push_str("\n");
    }

    #[inline(always)]
    pub(crate) fn as_str(&self) -> &str {
        line_str(&self.buf, self.len)
    }
}

/// The text of a [LineBuf] (only whole UTF-8 chars are pushed, see [LineCursor::push_str])
fn line_str(buf: &[u8], len: u16) -> &str {
    core::str::from_utf8(buf.get(..len as usize).unwrap_or(&[])).unwrap_or("")
}

/// A [LineBuf] seen without its capacity in the type
struct LineCursor<'a> {
    buf: &'a mut [u8],
    len: &'a mut u16,
}

impl LineCursor<'_> {
    /// Truncated when it does not fit, never at the middle of a char
    fn push_str(&mut self, s: &str) {
        let room = self.buf.len().saturating_sub(*self.len as usize);
        let mut end = s.len().min(room);
        while !s.is_char_boundary(end) {
            end -= 1;
        }
        self.push_bytes(s.as_bytes().get(..end).unwrap_or(&[]));
    }

    fn push_bytes(&mut self, bytes: &[u8]) {
        let len = *self.len as usize;
        // zip: no length checks (panic paths)
        let mut n = 0;
        for (dst, src) in self.buf.iter_mut().skip(len).zip(bytes) {
            *dst = *src;
            n += 1;
        }
        *self.len = (len + n) as u16;
    }

    fn push_fixed2(&mut self, v: f32) {
        if v.is_nan() {
            self.push_str("NaN");
            return;
        }
        // Round half away from zero to hundredths
        let scaled = v * 100.0;
        let hundredths = ((if scaled < 0.0 { scaled - 0.5 } else { scaled + 0.5 }) as i32)
            .clamp(-MAX_HUNDREDTHS, MAX_HUNDREDTHS);
        self.push_digits(hundredths < 0, hundredths.unsigned_abs(), true);
    }

    /// Writes `magnitude` (in hundredths when `fixed2`), with the sign when `negative`
    fn push_digits(&mut self, negative: bool, mut magnitude: u32, fixed2: bool) {
        // Digits in reverse order (and the point when `fixed2`): at least one integer digit.
        // Without indexing (no panic paths)
        let mut tmp = [0u8; I32_LEN];
        let mut i = tmp.len();
        let mut put = |b: u8| {
            if let Some(slot) = i.checked_sub(1).and_then(|k| tmp.get_mut(k)) {
                *slot = b;
                i -= 1;
            }
        };
        let int_pos = if fixed2 { 2 } else { 0 };
        for pos in 0.. {
            if fixed2 && pos == 2 {
                put(b'.');
            }
            put(b'0' + (magnitude % 10) as u8);
            magnitude /= 10;
            if magnitude == 0 && pos >= int_pos {
                break;
            }
        }
        if negative {
            put(b'-');
        }
        self.push_bytes(tmp.get(i..).unwrap_or(&[]));
    }
}

impl core::fmt::Write for LineCursor<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.push_str(s);
        Ok(())
    }
}

impl<const N: usize> core::fmt::Debug for LineBuf<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.as_str(), f)
    }
}

impl<const N: usize> core::fmt::Display for LineBuf<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl<const N: usize> PartialEq for LineBuf<N> {
    fn eq(&self, other: &Self) -> bool {
        self.as_str() == other.as_str()
    }
}

/// Truncated when it does not fit
impl<const N: usize> From<&str> for LineBuf<N> {
    fn from(s: &str) -> Self {
        let mut line = Self::new();
        line.push_str(s);
        line
    }
}

/// `write!` into the stack buffer: the heap-free replacement of `alloc::format!` (the heap is
/// 256 bytes on the smallest boards). The text is truncated when it does not fit, never at the
/// middle of a char, and it never fails (a truncated report is better than an OOM panic).
impl<const N: usize> core::fmt::Write for LineBuf<N> {
    #[inline(always)]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.cursor().push_str(s);
        Ok(())
    }

    #[inline(always)]
    fn write_fmt(&mut self, args: core::fmt::Arguments<'_>) -> core::fmt::Result {
        self.cursor().write_fmt(args)
    }
}

#[cfg(test)]
mod line_buf_tests {
    use super::*;

    #[test]
    fn fmt_write_truncates_at_char_boundary() {
        use core::fmt::Write;
        let mut b = LineBuf::<8>::new();
        assert!(write!(b, "ab{}", 12345).is_ok());
        assert_eq!(b.as_str(), "ab12345");
        // Full: the rest is dropped, without failing
        assert!(write!(b, "xyz").is_ok());
        assert_eq!(b.as_str(), "ab12345x");
        // A multi-byte char that does not fit is not split
        let mut b = LineBuf::<4>::new();
        let _ = write!(b, "abcñ");
        assert_eq!(b.as_str(), "abc");
    }

    fn fixed2(v: f32) -> alloc::string::String {
        let mut r = LineBuf::<16>::new();
        r.push_fixed2(v);
        alloc::string::String::from(r.as_str())
    }

    fn int(v: i32) -> alloc::string::String {
        let mut r = LineBuf::<16>::new();
        r.push_i32(v);
        alloc::string::String::from(r.as_str())
    }

    #[test]
    fn fixed2_formatting() {
        assert_eq!(fixed2(0.0), "0.00");
        assert_eq!(fixed2(0.05), "0.05");
        assert_eq!(fixed2(23.456), "23.46");
        assert_eq!(fixed2(200.0), "200.00");
        assert_eq!(fixed2(-0.004), "0.00");
        assert_eq!(fixed2(-1.5), "-1.50");
        assert_eq!(fixed2(-0.25), "-0.25");
        assert_eq!(fixed2(100000.0), "100000.00");
        assert_eq!(fixed2(f32::NAN), "NaN");
        assert_eq!(fixed2(f32::INFINITY), "999999.99");
        assert_eq!(fixed2(f32::NEG_INFINITY), "-999999.99");
    }

    #[test]
    fn i32_formatting() {
        assert_eq!(int(0), "0");
        assert_eq!(int(7), "7");
        assert_eq!(int(-800), "-800");
        assert_eq!(int(i32::MAX), "2147483647");
        assert_eq!(int(i32::MIN), "-2147483648");
    }

    #[test]
    fn truncates_instead_of_panicking() {
        let mut r = LineBuf::<4>::new();
        r.push_str("abcdef");
        assert_eq!(r.as_str(), "abcd");
    }

    #[test]
    fn m105_worst_case_fits() {
        assert!(M105_CAPACITY <= u16::MAX as usize);
        let mut r = LineBuf::<M105_CAPACITY>::new();
        r.push_str("ok");
        let labels: &[&str] = if cfg!(feature = "marlin-compat") {
            &[" T:", " /", " T@:", " B:", " /", " B@:"]
        } else {
            &[" T:", " /", " T@:", " TZ:", " B:", " /", " B@:", " BZ:"]
        };
        for label in labels {
            r.push_str(label);
            r.push_fixed2(-1.0e9);
        }
        r.push_str("\n");
        assert!(r.as_str().ends_with('\n'), "report truncated: {}", r.as_str());
    }

    #[cfg(feature = "marlin-compat")]
    #[test]
    fn m115_capability_lines_fit_and_are_unique() {
        for (i, (name, _)) in M115_CAPABILITIES.iter().enumerate() {
            assert!(4 + name.len() + 3 <= M115_CAP_LINE_CAPACITY, "{} does not fit", name);
            assert!(
                M115_CAPABILITIES[..i].iter().all(|(other, _)| other != name),
                "{} is repeated",
                name
            );
        }
    }

    #[cfg(feature = "with-motion")]
    #[test]
    fn m114_worst_case_fits() {
        let mut r = LineBuf::<M114_CAPACITY>::new();
        for c in hwa::CoordSel::all_axis().iter() {
            r.push_str(" ");
            r.push_str(c.name());
            r.push_str(":");
            r.push_fixed2(-1.0e9);
        }
        r.push_str(" Count:");
        for c in hwa::CoordSel::all_axis().iter() {
            r.push_str(" ");
            r.push_str(c.name());
            r.push_str(":");
            r.push_i32(i32::MIN);
        }
        r.push_str("\n");
        assert!(r.as_str().ends_with('\n'), "report truncated: {}", r.as_str());
    }
}
