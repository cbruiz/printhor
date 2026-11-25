//! The Temperature controller task

use crate::{hwa, processing};
use embassy_time::{Duration, Ticker};
#[allow(unused)]
use hwa::math::Real;

/// Temperature control period (ms). Each period every heater is sampled, its safety conditions
/// checked and its PID updated.
const TEMPERATURE_CONTROL_PERIOD_MS: u64 = 500;

/// While autotuning, the temperatures are reported every this many control periods (2 s, as
/// Marlin does)
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
const AUTOTUNE_REPORT_PERIODS: u32 = 4;

/// Hot-end auto fan: above this temperature (ºC) the extra fan (FAN1, usually the hot-end heat-sink
/// fan) runs at full power, as Marlin's `E0_AUTO_FAN` does
#[cfg(all(feature = "with-hot-end", feature = "with-fan-extra-1"))]
const HOT_END_AUTO_FAN_TEMP: f32 = 50.0;

///
/// # Asynchronous Task Entry Point
///
/// The temperature control task. Every [TEMPERATURE_CONTROL_PERIOD_MS] milliseconds:
///
/// 1. Updates every heater (hot-end and hot-bed, if enabled): temperature sampling, safety checks
///    and PID control.
/// 2. When a heater reports a fault (thermal runaway, MAXTEMP, MINTEMP, heating failed...), halts
///    the machine (see [processing::GCodeProcessor::emergency_stop]) and reports the error to the
///    hosts with the Marlin format (`Error:...`), which host software recognizes.
/// 3. While `SYS_ALARM` is set (by any source, e.g. `M112`), keeps every heater shut down.
/// 4. Reports the PID autotune (`M303`) as Marlin does: each relay cycle, the temperatures every
///    [AUTOTUNE_REPORT_PERIODS] periods, and the outcome, to the channel that asked for it; its
///    response (`ok`) then completes.
#[embassy_executor::task(pool_size = 1)]
pub async fn task_temperature(processor: processing::GCodeProcessor) {
    hwa::info!("[task_temperature] Started");

    let event_bus = processor.event_bus.clone();
    #[cfg(all(feature = "with-hot-end", feature = "with-fan-extra-1"))]
    let mut auto_fan_on = false;
    let mut ticker = Ticker::every(Duration::from_millis(TEMPERATURE_CONTROL_PERIOD_MS));
    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    let mut periods: u32 = 0;

    loop {
        ticker.next().await;
        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        {
            periods = periods.wrapping_add(1);
        }
        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        #[allow(unused_mut)]
        let mut autotune_channel: Option<hwa::CommChannel> = None;

        #[cfg(test)]
        if crate::tasks::task_integration::INTEGRATION_STATUS.signaled() {
            hwa::info!("[task_temperature] Ending gracefully");
            return ();
        }

        #[allow(unused_mut)]
        let mut fault: Option<(&'static str, hwa::controllers::HeaterFault)> = None;

        #[cfg(feature = "with-hot-end")]
        {
            let mut h = processor.hot_end.lock().await;
            if let Some(f) = h.update(&event_bus).await {
                fault = Some(("0", f));
            }
            let events = h.take_autotune_events();
            let running = h.autotune_channel();
            autotune_channel = autotune_channel.or(running);
            drop(h);
            if let Some(end) = report_autotune(&processor, "", events, running).await {
                processor.hot_end.lock().await.complete_autotune(&end).await;
            }
        }

        #[cfg(all(feature = "with-hot-end", feature = "with-fan-extra-1"))]
        {
            let temp = processor.hot_end.lock().await.get_current_temp();
            // NaN (sensor error) also keeps the fan running
            let hot = !(temp < HOT_END_AUTO_FAN_TEMP);
            // While hot, full power is reapplied every period, so that `M106 P1 S0` can not
            // stop the heat-sink fan. Once cold, the manual setting is restored.
            if hot {
                processor.fan_extra1.lock().await.set_power(100);
            } else if auto_fan_on {
                let manual = crate::processing::FAN_EXTRA1_MANUAL_POWER
                    .load(core::sync::atomic::Ordering::Relaxed);
                processor.fan_extra1.lock().await.set_power(manual);
            }
            auto_fan_on = hot;
        }

        #[cfg(feature = "with-hot-bed")]
        {
            let mut h = processor.hot_bed.lock().await;
            if let Some(f) = h.update(&event_bus).await {
                fault = fault.or(Some(("-1", f)));
            }
            let events = h.take_autotune_events();
            let running = h.autotune_channel();
            autotune_channel = autotune_channel.or(running);
            drop(h);
            if let Some(end) = report_autotune(&processor, "bed", events, running).await {
                processor.hot_bed.lock().await.complete_autotune(&end).await;
            }
        }

        // While autotuning, the temperatures (as Marlin, the host gets no response meanwhile)
        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        if let Some(channel) = autotune_channel {
            if periods % AUTOTUNE_REPORT_PERIODS == 0 {
                let mut report = processing::LineBuf::<{ processing::M105_CAPACITY }>::new();
                processor.push_temperatures(&mut report).await;
                report.push_str("\n");
                processor.write(channel, report.as_str()).await;
            }
        }

        if let Some((heater_id, f)) = fault {
            // Heap-free: the emergency path must not depend on the heap
            use core::fmt::Write;
            let mut reason = processing::LineBuf::<96>::new();
            let _ = write!(reason, "{}, system stopped! Heater_ID: {}", f.description(), heater_id);
            processor.emergency_stop(reason.as_str()).await;
        } else if event_bus.has_flags(hwa::EventFlags::SYS_ALARM).await {
            processor.shutdown_heaters().await;
        }
    }
}

/// Reports the autotune events of a heater (see [hwa::controllers::HeaterController::take_autotune_events])
/// as Marlin does, to the channel that asked for it. `defaults` names the heater in the
/// `#define DEFAULT_<defaults>Kp` lines (Marlin: empty for the hot-end, `bed` for the bed).
/// `running` is the channel of the autotune of the heater, while it runs. Returns the end of
/// the autotune, if it ended: its response is to be completed then.
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
async fn report_autotune(
    processor: &processing::GCodeProcessor,
    defaults: &str,
    events: (
        Option<hwa::controllers::AutotuneCycle>,
        Option<hwa::controllers::AutotuneEnd>,
    ),
    running: Option<hwa::CommChannel>,
) -> Option<hwa::controllers::AutotuneEnd> {
    use hwa::controllers::AutotuneOutcome;
    type Line = processing::LineBuf<AUTOTUNE_LINE_CAPACITY>;
    let f = |v: Real| v.to_f64() as f32;
    let (cycle, end) = events;
    let channel = match (&cycle, &end) {
        (_, Some(end)) => end.request.channel,
        (Some(_), None) => running?,
        (None, None) => return None,
    };
    if let Some(cycle) = cycle {
        // ` bias: 92 d: 92 min: 196.56 max: 203.75[ Ku: 51.98 Tu: 22.37]`
        let mut line = Line::new();
        line.push_str(" bias: ");
        line.push_i32(cycle.bias.round().to_i32().unwrap_or(0));
        line.push_str(" d: ");
        line.push_i32(cycle.d.round().to_i32().unwrap_or(0));
        line.push_str(" min: ");
        line.push_fixed2(f(cycle.min));
        line.push_str(" max: ");
        line.push_fixed2(f(cycle.max));
        if let Some(e) = &cycle.estimate {
            line.push_str(" Ku: ");
            line.push_fixed2(f(e.ku));
            line.push_str(" Tu: ");
            line.push_fixed2(f(e.tu));
        }
        line.push_str("\n");
        processor.write(channel, line.as_str()).await;
        if let Some(e) = &cycle.estimate {
            processor.write(channel, cycle.rule.report()).await;
            processor.write(channel, "\n").await;
            processor.write(channel, gains_line(" Kp: ", " Ki: ", " Kd: ", e.kp, e.ki, e.kd).as_str()).await;
        }
    }
    let end = end?;
    match end.outcome {
        AutotuneOutcome::Done { kp, ki, kd, settled } => {
            if !settled {
                processor
                    .write(channel, "echo:PID Autotune: the oscillation did not settle, the result may be inaccurate\n")
                    .await;
            }
            processor
                .write(
                    channel,
                    "PID Autotune finished! Put the last Kp, Ki and Kd constants from below into Configuration.h\n",
                )
                .await;
            for (name, value) in [("Kp ", kp), ("Ki ", ki), ("Kd ", kd)] {
                let mut line = Line::new();
                line.push_str("#define DEFAULT_");
                line.push_str(defaults);
                line.push_str(name);
                line.push_fixed2(f(value));
                line.push_str("\n");
                processor.write(channel, line.as_str()).await;
            }
        }
        AutotuneOutcome::Overshoot => {
            processor.write(channel, "PID Autotune failed! Temperature too high\n").await
        }
        AutotuneOutcome::Timeout => processor.write(channel, "PID Autotune failed! timeout\n").await,
        AutotuneOutcome::NoOscillation => {
            processor.write(channel, "PID Autotune failed! No oscillation\n").await
        }
        AutotuneOutcome::Interrupted => {
            processor.write(channel, "PID Autotune failed! Interrupted\n").await
        }
    }
    Some(end)
}

/// ` Kp: 1.00 Ki: 2.00 Kd: 3.00`
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
fn gains_line(
    p: &str,
    i: &str,
    d: &str,
    kp: Real,
    ki: Real,
    kd: Real,
) -> processing::LineBuf<AUTOTUNE_LINE_CAPACITY> {
    let mut line = processing::LineBuf::<AUTOTUNE_LINE_CAPACITY>::new();
    for (name, value) in [(p, kp), (i, ki), (d, kd)] {
        line.push_str(name);
        line.push_fixed2(value.to_f64() as f32);
    }
    line.push_str("\n");
    line
}

/// The longest autotune report line: ` bias: 255 d: 255 min: <n> max: <n> Ku: <n> Tu: <n>`
/// (10 chars per number at most)
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
const AUTOTUNE_LINE_CAPACITY: usize = 7 + 11 + 4 + 11 + 6 + 10 + 6 + 10 + 5 + 10 + 5 + 10 + 1;
