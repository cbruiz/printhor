//! Boundary tests of the heater safety logic: thermistor conversion at the ADC extremes, MAXTEMP,
//! MINTEMP, sensor errors, the heating watch, thermal runaway, the target limits, fault latching,
//! PID constants and the deferred responses (M109/M190).
//!
//! The controller is built on a fake ADC (returns the sample set by the test) and a fake PWM, so
//! every case is deterministic and independent of the native simulator thermal model.

use super::*;
use crate::hwa;
use embassy_time::{Duration, Instant};
use hwa::HwiContract;

/// An ADC that returns the sample set by the test
pub struct FakeAdc {
    pub sample: u16,
}

impl hwa::traits::UnifiedAdc16 for FakeAdc {
    type VRefPin = ();
    type SamplePin = ();

    async fn read_adc(&mut self, _pin: &mut Self::SamplePin) -> u16 {
        self.sample
    }
}

/// A PWM that records the duty cycle
#[derive(Default)]
pub struct FakePwm {
    pub duty: u16,
}

impl hwa::traits::Pwm for FakePwm {
    type Channel = u8;
    type Time = u32;
    type Duty = u16;

    fn disable(&mut self, _channel: Self::Channel) {}
    fn enable(&mut self, _channel: Self::Channel) {}
    fn get_period(&self) -> Self::Time {
        0
    }
    fn get_duty(&self, _channel: Self::Channel) -> Self::Duty {
        self.duty
    }
    fn get_max_duty(&self) -> Self::Duty {
        255
    }
    fn set_duty(&mut self, _channel: Self::Channel, duty: Self::Duty) {
        self.duty = duty;
    }
    fn set_period<P>(&mut self, _period: P)
    where
        P: Into<Self::Time>,
    {
    }
}

type TestAdc = hwa::AsyncStandardStrategy<hwa::AsyncCsMutexType, FakeAdc>;
type TestPwm = hwa::SyncStandardStrategy<hwa::SyncCsMutexType, FakePwm>;
type TestHeater = HeaterController<TestAdc, TestPwm>;

/// The configuration of the native hot-end
fn config() -> HeaterConfig {
    HeaterConfig {
        max_temp: <hwa::Contract as HwiContract>::HOT_END_MAX_TEMP,
        min_temp: <hwa::Contract as HwiContract>::HOT_END_MIN_TEMP,
        protection_hysteresis: <hwa::Contract as HwiContract>::HOT_END_THERMAL_PROTECTION_HYSTERESIS,
        protection_period_s: <hwa::Contract as HwiContract>::HOT_END_THERMAL_PROTECTION_PERIOD_S,
        watch_increase: <hwa::Contract as HwiContract>::HOT_END_WATCH_TEMP_INCREASE,
        watch_period_s: <hwa::Contract as HwiContract>::HOT_END_WATCH_TEMP_PERIOD_S,
        temp_window: <hwa::Contract as HwiContract>::HOT_END_TEMP_WINDOW,
        pid: (
            <hwa::Contract as HwiContract>::HOT_END_PID_KP,
            <hwa::Contract as HwiContract>::HOT_END_PID_KI,
            <hwa::Contract as HwiContract>::HOT_END_PID_KD,
        ),
    }
}

const BETA: f32 = 3950.0;
const R_NOMINAL: f32 = 100_000.0;
const R_PULL_UP: f32 = 4685.0;

/// A heater on the fake ADC and PWM, its defer channel and the ADC (to change the sample).
fn heater(
    sample: u16,
) -> (
    TestHeater,
    hwa::types::DeferChannel,
    &'static hwa::AsyncMutex<hwa::AsyncCsMutexType, FakeAdc>,
) {
    let adc_mutex: &'static _ = Box::leak(Box::new(hwa::AsyncMutex::new(FakeAdc { sample })));
    let pwm_mutex: &'static _ = Box::leak(Box::new(hwa::SyncMutex::new(core::cell::RefCell::new(
        FakePwm::default(),
    ))));
    let adc = hwa::controllers::GenericAdcController::new(
        hwa::StaticAsyncController::new(TestAdc::new(adc_mutex)),
        (),
        0,
    );
    let pwm = hwa::controllers::GenericPwmController::new(
        hwa::StaticSyncController::new(TestPwm::new(pwm_mutex)),
        0u8,
    );
    let defer_channel: hwa::types::DeferChannel = hwa::GenericDeferChannel::new(Box::leak(
        Box::new(hwa::DeferChannelChannelType::new()),
    ));
    let heater = HeaterController::new(
        adc,
        pwm,
        BETA,
        R_NOMINAL,
        R_PULL_UP,
        defer_channel.clone(),
        DeferAction::HotEndTemperature,
        EventFlags::HOT_END_TEMP_OK,
        config(),
    );
    (heater, defer_channel, adc_mutex)
}

/// The ADC sample that reads the given thermistor resistance (with the controller's v_ratio = 1)
fn sample_for_resistance(r: f32) -> u16 {
    (4095.0 * r / (r + R_PULL_UP)).round() as u16
}

/// Turns the heater on towards `target` from `current` degrees, as M104 does
async fn heat(h: &mut TestHeater, current: f32, target: f32) {
    h.set_current_temp(current);
    assert_eq!(
        h.set_target_temp(CommChannel::Internal, DeferAction::HotEndTemperature, target, 1, false)
            .await,
        Ok(false)
    );
}

#[futures_test::test]
async fn thermistor_conversion_is_the_beta_equation() {
    let (h, _, _) = heater(0);
    // The nominal resistance is the reference temperature, 25 ºC
    let (t, r) = h.convert_to_celcius(sample_for_resistance(R_NOMINAL));
    assert!((t - 25.0).abs() < 0.5, "nominal resistance read {} ºC", t);
    assert!((r - R_NOMINAL).abs() / R_NOMINAL < 0.01, "resistance {}", r);
    // A NTC thermistor: the higher the sample (resistance), the colder
    let temps: Vec<f32> = [500u16, 1000, 2000, 3000, 3900]
        .iter()
        .map(|s| h.convert_to_celcius(*s).0)
        .collect();
    assert!(temps.windows(2).all(|w| w[0] > w[1]), "not monotonic: {:?}", temps);
}

/// The exact beta equation (1/T = 1/T0 + ln(R/R0)/B) for the resistance that the given sample
/// measures, so that only the conversion error is checked (not the ADC quantization).
fn exact_temperature(sample: u16) -> f32 {
    let r = sample as f32 * R_PULL_UP / (4095.0 - sample as f32);
    1.0 / ((r / R_NOMINAL).ln() / BETA + 1.0 / 298.15) - 273.15
}

#[test]
fn thermistor_conversion_is_accurate_in_the_printing_range() {
    let (h, _, _) = heater(0);
    for t in [150.0f32, 180.0, 200.0, 220.0, 240.0, 250.0, 260.0, 280.0, 300.0] {
        let r = R_NOMINAL * (BETA * (1.0 / (t + 273.15) - 1.0 / 298.15)).exp();
        let sample = sample_for_resistance(r);
        let (read, _) = h.convert_to_celcius(sample);
        let exact = exact_temperature(sample);
        assert!((read - exact).abs() <= 1.0, "at {} ºC read {} instead of {}", t, read, exact);
    }
}

#[futures_test::test]
async fn read_temp_updates_the_measured_temperature_and_resistance() {
    let (mut h, _, adc) = heater(sample_for_resistance(R_NOMINAL));
    let _ = h.read_temp().await;
    assert!((h.get_current_temp() - 25.0).abs() < 0.5);
    assert!((h.get_current_resistance() - R_NOMINAL).abs() / R_NOMINAL < 0.01);
    adc.lock().await.sample = sample_for_resistance(6_600.0);
    let _ = h.read_temp().await;
    assert!(
        (h.get_current_temp() - 100.0).abs() < 3.0,
        "read {}",
        h.get_current_temp()
    );
}

/// Whatever the conversion reads at the ADC extremes of the 12-bit range (shorted or open
/// thermistor), a heater that is heating must fault.
#[futures_test::test]
async fn sensor_failures_fault_while_heating() {
    for sample in [0u16, 1, 4094, 4095] {
        let (mut h, _, _) = heater(sample);
        heat(&mut h, 25.0, 200.0).await;
        let (t, _) = h.convert_to_celcius(sample);
        let fault = h.check_safety(t, Instant::now());
        assert!(
            matches!(
                fault,
                Some(HeaterFault::MaxTemp | HeaterFault::MinTemp | HeaterFault::SensorError)
            ),
            "sample {} read {} ºC and gave {:?}",
            sample,
            t,
            fault
        );
    }
}

/// A sample above the 12-bit range (the ADC trait is 16-bit) must not crash the temperature task:
/// it is a sensor error.
#[futures_test::test]
async fn out_of_range_samples_are_a_sensor_error() {
    for sample in [4096u16, 5000, u16::MAX] {
        let (mut h, _, _) = heater(sample);
        heat(&mut h, 25.0, 200.0).await;
        let (t, _) = h.convert_to_celcius(sample);
        assert_eq!(h.check_safety(t, Instant::now()), Some(HeaterFault::SensorError), "sample {}", sample);
    }
}

#[futures_test::test]
async fn maxtemp_applies_even_when_off() {
    let (mut h, _, _) = heater(0);
    let max = config().max_temp;
    assert!(!h.is_on());
    assert_eq!(h.check_safety(max, Instant::now()), Some(HeaterFault::MaxTemp));
    assert_eq!(h.check_safety(max + 100.0, Instant::now()), Some(HeaterFault::MaxTemp));
    assert_eq!(h.check_safety(max - 0.01, Instant::now()), None);
    heat(&mut h, 25.0, 200.0).await;
    assert_eq!(h.check_safety(max, Instant::now()), Some(HeaterFault::MaxTemp));
}

#[futures_test::test]
async fn mintemp_and_sensor_errors_only_while_heating() {
    let (mut h, _, _) = heater(0);
    let min = config().min_temp;
    // An unused heater with a disconnected sensor must not halt the machine
    assert_eq!(h.check_safety(f32::NAN, Instant::now()), None);
    assert_eq!(h.check_safety(min - 10.0, Instant::now()), None);
    heat(&mut h, 25.0, 200.0).await;
    assert_eq!(h.check_safety(f32::NAN, Instant::now()), Some(HeaterFault::SensorError));
    assert_eq!(h.check_safety(min - 0.01, Instant::now()), Some(HeaterFault::MinTemp));
    assert_eq!(h.check_safety(min, Instant::now()), None);
}

#[futures_test::test]
async fn heating_watch_detects_a_heater_that_does_not_heat() {
    let cfg = config();
    let watch_period = Duration::from_secs(cfg.watch_period_s as u64 + 1);
    let (mut h, _, _) = heater(0);
    heat(&mut h, 25.0, 200.0).await;
    // Before the watch period, a stalled temperature is not a fault yet
    assert_eq!(h.check_safety(25.0, Instant::now()), None);
    // After the watch period without the expected increase: heating failed
    assert_eq!(
        h.check_safety(25.0 + cfg.watch_increase - 0.1, Instant::now() + watch_period),
        Some(HeaterFault::HeatingFailed)
    );

    // With the expected increase, the watch is re-armed instead
    let (mut h, _, _) = heater(0);
    heat(&mut h, 25.0, 200.0).await;
    h.set_current_temp(25.0 + cfg.watch_increase);
    assert_eq!(
        h.check_safety(25.0 + cfg.watch_increase, Instant::now() + watch_period),
        None
    );
    let (deadline, watch_temp) = h.watch.expect("the watch is re-armed");
    assert!(deadline > Instant::now());
    assert_eq!(watch_temp, 25.0 + 2.0 * cfg.watch_increase);

    // Close to the target, no watch is armed (it would fire while approaching it slowly)
    let (mut h, _, _) = heater(0);
    heat(&mut h, 200.0 - cfg.protection_hysteresis, 200.0).await;
    assert!(h.watch.is_none());
}

#[futures_test::test]
async fn thermal_runaway_after_the_target_was_reached() {
    let cfg = config();
    let period = Duration::from_secs(cfg.protection_period_s as u64);
    let target = 200.0;
    let (mut h, _, _) = heater(0);
    heat(&mut h, 25.0, target).await;
    let t0 = Instant::now();
    // Reaching the target window switches the protection to the stable state
    assert_eq!(h.check_safety(target - cfg.protection_hysteresis, t0), None);
    assert!(matches!(h.protection, ProtectionState::Stable));
    assert!(h.watch.is_none());

    // A drop below the hysteresis starts the runaway period...
    let low = target - cfg.protection_hysteresis - 1.0;
    assert_eq!(h.check_safety(low, t0), None);
    assert_eq!(h.runaway_deadline, Some(t0 + period));
    // ... a recovery within the period cancels it...
    assert_eq!(h.check_safety(target, t0 + period / 2), None);
    assert_eq!(h.runaway_deadline, None);
    // ... and staying low for the whole period is a thermal runaway
    assert_eq!(h.check_safety(low, t0 + period), None);
    assert_eq!(h.check_safety(low, t0 + period + period / 2), None);
    assert_eq!(
        h.check_safety(low, t0 + period + period),
        Some(HeaterFault::ThermalRunaway)
    );
}

#[futures_test::test]
async fn autotune_suspends_the_runaway_detection_but_not_maxtemp() {
    let cfg = config();
    let target = 200.0;
    let (mut h, _, _) = heater(0);
    h.set_current_temp(25.0);
    assert_eq!(h.start_autotune(AutotuneRequest::new(target, 5)).await, Ok(()));
    // Not twice at once
    assert_eq!(h.start_autotune(AutotuneRequest::new(target, 5)).await, Err(HeaterError::Busy));
    assert!(h.is_autotuning());
    assert!(h.is_on());
    assert_eq!(h.get_target_temp(), target);
    assert_eq!(h.check_safety(target, Instant::now()), None);
    assert!(matches!(h.protection, ProtectionState::Stable));
    // The relay autotune oscillates below the target on purpose
    let far = Instant::now() + Duration::from_secs(10 * cfg.protection_period_s as u64);
    assert_eq!(h.check_safety(target - 50.0, Instant::now()), None);
    assert_eq!(h.check_safety(target - 50.0, far), None);
    assert_eq!(h.runaway_deadline, None);
    assert_eq!(h.check_safety(cfg.max_temp, far), Some(HeaterFault::MaxTemp));
    assert!(matches!(h.take_autotune_events(), (None, None)));
    // A new target cancels the autotune: its end is reported (and its response completed)
    heat(&mut h, 25.0, 180.0).await;
    assert!(!h.is_autotuning());
    assert!(matches!(
        h.take_autotune_events(),
        (None, Some(AutotuneEnd { outcome: AutotuneOutcome::Interrupted, .. }))
    ));
}

#[futures_test::test]
async fn target_limits() {
    let (mut h, _, _) = heater(0);
    let max_target = h.max_target_temp();
    assert_eq!(max_target, config().max_temp - TARGET_MAX_TEMP_MARGIN);
    assert_eq!(
        h.set_target_temp(CommChannel::Internal, DeferAction::HotEndTemperature, max_target + 0.1, 1, false)
            .await,
        Err(HeaterError::TargetTooHigh)
    );
    assert!(!h.is_on());
    assert_eq!(h.get_target_temp(), 0.0);
    assert_eq!(
        h.start_autotune(AutotuneRequest::new(max_target + 0.1, 5)).await,
        Err(HeaterError::TargetTooHigh)
    );
    assert!(!h.is_autotuning());
    // The limit itself is accepted
    heat(&mut h, 25.0, max_target).await;
    assert!(h.is_on());
    assert_eq!(h.get_target_temp(), max_target);
    // Zero or a negative target turns the heater off
    for off in [0.0f32, -10.0] {
        heat(&mut h, 25.0, 200.0).await;
        assert_eq!(
            h.set_target_temp(CommChannel::Internal, DeferAction::HotEndTemperature, off, 1, false)
                .await,
            Ok(false)
        );
        assert!(!h.is_on());
        assert_eq!(h.get_target_temp(), 0.0);
        assert_eq!(h.get_current_power(), 0.0);
    }
}

#[futures_test::test]
async fn a_fault_is_latched_until_cleared() {
    let (mut h, _, _) = heater(0);
    heat(&mut h, 25.0, 200.0).await;
    h.trigger_fault(HeaterFault::ThermalRunaway);
    assert_eq!(h.fault(), Some(HeaterFault::ThermalRunaway));
    assert!(!h.is_on());
    assert_eq!(h.get_target_temp(), 0.0);
    assert_eq!(h.get_current_power(), 0.0);
    // While faulted, the heater can not be turned on, nor autotuned
    assert_eq!(
        h.set_target_temp(CommChannel::Internal, DeferAction::HotEndTemperature, 200.0, 1, false)
            .await,
        Err(HeaterError::Faulted(HeaterFault::ThermalRunaway))
    );
    assert!(!h.is_on());
    assert_eq!(
        h.start_autotune(AutotuneRequest::new(200.0, 5)).await,
        Err(HeaterError::Faulted(HeaterFault::ThermalRunaway))
    );
    // Turning it off is always allowed
    assert_eq!(
        h.set_target_temp(CommChannel::Internal, DeferAction::HotEndTemperature, 0.0, 1, false)
            .await,
        Ok(false)
    );
    // M999 clears the fault; the heater stays off until a new target
    h.clear_fault();
    assert_eq!(h.fault(), None);
    assert!(!h.is_on());
    heat(&mut h, 25.0, 200.0).await;
    assert!(h.is_on());
}

#[futures_test::test]
async fn pid_constants_can_be_changed_and_restored() {
    let (mut h, _, _) = heater(0);
    let defaults = config().pid;
    assert_eq!(h.get_pid(), defaults);
    // M301 with only some parameters keeps the others
    h.set_pid(Some(10.0), None, Some(30.0));
    assert_eq!(h.get_pid(), (10.0, defaults.1, 30.0));
    h.set_pid(None, Some(0.5), None);
    assert_eq!(h.get_pid(), (10.0, 0.5, 30.0));
    // M502 restores the defaults
    h.reset_pid();
    assert_eq!(h.get_pid(), defaults);
}

#[futures_test::test]
async fn host_requests_are_deferred_until_the_target_is_reached() {
    let window = config().temp_window;
    let (mut h, defer_channel, _) = heater(0);
    let host = CommChannel::SerialPort1;

    // Far from the target: the response to the host is deferred (M109)
    h.set_current_temp(25.0);
    assert_eq!(
        h.set_target_temp(host, DeferAction::HotEndTemperature, 200.0, 7, false).await,
        Ok(true)
    );
    assert!(h.is_awaited());
    match defer_channel.try_receive() {
        Ok(hwa::DeferEvent::AwaitRequested(DeferAction::HotEndTemperature, channel, 7)) => {
            assert_eq!(channel, host)
        }
        _ => panic!("unexpected defer event"),
    }
    // Once reached, the deferred response is completed on the host channel
    h.flush_notification(DeferAction::HotEndTemperature, 7).await;
    assert!(!h.is_awaited());
    match defer_channel.try_receive() {
        Ok(hwa::DeferEvent::Completed(DeferAction::HotEndTemperature, channel, 7)) => {
            assert_eq!(channel, host)
        }
        _ => panic!("unexpected defer event"),
    }

    // Already within the window: nothing to wait for
    h.set_current_temp(200.0 - window);
    assert_eq!(
        h.set_target_temp(host, DeferAction::HotEndTemperature, 200.0, 8, false).await,
        Ok(false)
    );
    assert!(!h.is_awaited());
    assert!(defer_channel.try_receive().is_err());

    // Above the target: M109 S waits only when cooling is requested (M109 R)
    h.set_current_temp(230.0);
    assert_eq!(
        h.set_target_temp(host, DeferAction::HotEndTemperature, 210.0, 9, false).await,
        Ok(false)
    );
    assert_eq!(
        h.set_target_temp(host, DeferAction::HotEndTemperature, 200.0, 10, true).await,
        Ok(true)
    );
    assert!(matches!(
        defer_channel.try_receive(),
        Ok(hwa::DeferEvent::AwaitRequested(_, _, 10))
    ));

    // M109/M190 with no target only wait for an active heater
    let (mut h, defer_channel, _) = heater(0);
    assert!(!h.ping_subscribe(host, DeferAction::HotEndTemperature, 11).await);
    h.set_current_temp(25.0);
    heat(&mut h, 25.0, 200.0).await;
    assert!(h.ping_subscribe(host, DeferAction::HotEndTemperature, 12).await);
    assert!(matches!(
        defer_channel.try_receive(),
        Ok(hwa::DeferEvent::AwaitRequested(_, _, 12))
    ));
}

