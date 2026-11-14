//! The Temperature controller task

use crate::{hwa, processing};
use embassy_time::{Duration, Ticker};
#[allow(unused)]
use hwa::math::Real;

/// Temperature control period (ms). Each period every heater is sampled, its safety conditions
/// checked and its PID updated.
const TEMPERATURE_CONTROL_PERIOD_MS: u64 = 500;

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
/// 4. Reports the outcome of a finished PID autotune (`M303`).
#[embassy_executor::task(pool_size = 1)]
pub async fn task_temperature(processor: processing::GCodeProcessor) {
    hwa::info!("[task_temperature] Started");

    let event_bus = processor.event_bus.clone();
    #[cfg(all(feature = "with-hot-end", feature = "with-fan-extra-1"))]
    let mut auto_fan_on = false;
    let mut ticker = Ticker::every(Duration::from_millis(TEMPERATURE_CONTROL_PERIOD_MS));

    loop {
        ticker.next().await;

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
            let outcome = h.take_autotune_outcome();
            drop(h);
            if let Some(outcome) = outcome {
                report_autotune(&processor, "M301", outcome).await;
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
            let outcome = h.take_autotune_outcome();
            drop(h);
            if let Some(outcome) = outcome {
                report_autotune(&processor, "M304", outcome).await;
            }
        }

        if let Some((heater_id, f)) = fault {
            let reason = alloc::format!(
                "{}, system stopped! Heater_ID: {}",
                f.description(),
                heater_id
            );
            processor.emergency_stop(reason.as_str()).await;
        } else if event_bus.has_flags(hwa::EventFlags::SYS_ALARM).await {
            processor.shutdown_heaters().await;
        }
    }
}

async fn report_autotune(
    processor: &processing::GCodeProcessor,
    _mnemonic: &str,
    outcome: hwa::controllers::AutotuneOutcome,
) {
    let msg = match outcome {
        hwa::controllers::AutotuneOutcome::Done(kp, ki, kd) => alloc::format!(
            "echo: PID Autotune finished! Applied: {} P{:?} I{:?} D{:?}\n",
            _mnemonic,
            Real::from_f32(kp).rdp(2),
            Real::from_f32(ki).rdp(3),
            Real::from_f32(kd).rdp(2),
        ),
        hwa::controllers::AutotuneOutcome::Overshoot => {
            alloc::string::String::from("echo: PID Autotune failed! Temperature too high\n")
        }
        hwa::controllers::AutotuneOutcome::Timeout => {
            alloc::string::String::from("echo: PID Autotune failed! timeout\n")
        }
    };
    processor.write_all(msg.as_str()).await;
}
