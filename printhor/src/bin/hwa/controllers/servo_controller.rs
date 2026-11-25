//! TODO: This feature is still in incubation
use crate::hwa;
use embedded_hal_0::Pwm;

/// The `ProbeTrait` defines a set of asynchronous methods for controlling the probe.
/// Each method represents a specific probe action, with a customizable sleep duration.
///
/// # Methods
///
/// * `probe_pin_down(&mut self, sleep_us: u64)` - Lowers the probe pin.
/// * `probe_pin_up(&mut self, sleep_us: u64)` - Raises the probe pin.
/// * `probe_self_test(&mut self, sleep_us: u64)` - Initiates a self-test sequence.
/// * `probe_alarm_release(&mut self, sleep_us: u64)` - Releases any probe alarms.
/// * `probe_test_mode(&mut self, sleep_us: u64)` - Sets the probe into test mode.
pub trait ProbeTrait {
    /// Lowers the probe pin, causing it to make contact. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after lowering the pin.
    fn probe_pin_down(&mut self, sleep_us: u64) -> impl core::future::Future<Output = ()>;

    /// Deploys the probe and switches its output to latched mode (BLTouch V3 "SW mode").
    ///
    /// In its default mode a BLTouch signals the touch with a short pulse (a few ms), easily
    /// missed by a polled end-stop. In SW mode the output stays high after the touch until the
    /// probe is stowed, so polling it once per step is enough.
    ///
    /// # Parameters
    /// * `deploy_sleep_us` - Duration in microseconds to wait after deploying the pin.
    /// * `mode_sleep_us` - Duration in microseconds to wait after the SW mode command.
    fn probe_deploy_latched(
        &mut self,
        deploy_sleep_us: u64,
        mode_sleep_us: u64,
    ) -> impl core::future::Future<Output = ()>;

    /// Raises the probe pin, moving it away from contact. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after raising the pin.
    fn probe_pin_up(&mut self, sleep_us: u64) -> impl core::future::Future<Output = ()>;

    /// Initiates a self-test sequence for the probe. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after performing the self-test.
    fn probe_self_test(&mut self, sleep_us: u64) -> impl core::future::Future<Output = ()>;

    /// Releases any active probe alarms. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after releasing the alarm.
    fn probe_alarm_release(&mut self, sleep_us: u64) -> impl core::future::Future<Output = ()>;

    /// Sets the probe into test mode (60º: the BLTouch V3 SW mode, latched output).
    /// The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after entering test mode.
    fn probe_test_mode(&mut self, sleep_us: u64) -> impl core::future::Future<Output = ()>;

    /// Sets the raw servo angle (M280). The operation waits for a specified duration.
    fn probe_set_angle(&mut self, angle: u16, sleep_us: u64) -> impl core::future::Future<Output = ()>;

    /// Diagnostic: (duty, max duty) currently applied to the servo channel
    fn probe_pwm_state(&mut self) -> (u32, u32);
}

/// Time for a BLTouch to decode a command before the next one (several 20 ms servo periods)
const BLTOUCH_COMMAND_SETTLE_US: u64 = 200_000;

/// The `GenericServoController` structure is responsible for managing a servo motor
/// using an interrupt-controlled PWM (Pulse Width Modulation) channel.
/// It provides methods to control the angle of the servo and implements the `ProbeTrait` for additional probe-related actions.
///
/// # Fields
///
/// * `servo` - A reference to a static controller PWM servo (PwmServo).
/// * `channel` - The specific PWM channel associated with the servo motor.
pub struct GenericServoController<H>
where
    H: hwa::SyncMutexStrategy + 'static,
    H::Resource: Pwm + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty: Into<u32> + From<u16> + Copy,
{
    /// A reference to a static controller PWM servo.
    servo: hwa::StaticSyncController<H>,

    /// The specific PWM channel associated with the servo motor.
    channel: <H::Resource as Pwm>::Channel,
}

impl<H> GenericServoController<H>
where
    H: hwa::SyncMutexStrategy + 'static,
    H::Resource: Pwm + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty: Into<u32> + From<u16> + Copy,
{
    pub fn new(
        servo: hwa::StaticSyncController<H>,
        channel: <H::Resource as Pwm>::Channel,
    ) -> Self {
        Self { servo, channel }
    }

    pub async fn set_angle(&mut self, angle: u16, sleep_us: u64) {
        {
            let max_duty = self.servo.apply(|servo| servo.get_max_duty());
            // 100% duty period width (uS)
            const PERIOD: u32 = 20_000;
            // Standard (Arduino/Marlin) servo mapping: 0º -> 544 uS, 180º -> 2400 uS.
            // A BLTouch decodes the pulse width as a command, so it must match what it expects:
            // 10º deploy = 647 uS, 60º test = 1162 uS, 90º stow = 1473 uS, 120º self-test = 1782 uS,
            // 160º alarm release = 2194 uS.
            const MIN_PULSE_WIDTH: u32 = 544;
            const MAX_PULSE_WIDTH: u32 = 2400;

            let duty_us = MIN_PULSE_WIDTH
                + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * (angle.min(180) as u32) + 90) / 180;
            let duty_cnt = (duty_us * max_duty.into() / PERIOD) as u16;

            #[cfg(feature = "trace-commands")]
            hwa::info!(
                "Set probe angle: {:?} : duty: {:?} uS | {:?} max_duty: {:?}",
                angle,
                duty_us,
                duty_cnt,
                Into::<u32>::into(max_duty),
            );
            // Only the compare value changes (preloaded: applied at the next period). Disabling and
            // enabling the channel around it would emit a truncated pulse of random width, which a
            // BLTouch decodes as a random command (test mode, alarm...) and then gets stuck.
            // `enable` is idempotent: it only matters the first time.
            self.servo.apply_mut(|servo| {
                servo.set_duty(self.channel, duty_cnt.into());
                servo.enable(self.channel);
            });
        }
        embassy_time::Timer::after_micros(sleep_us).await;
    }
}

impl<H> Clone for GenericServoController<H>
where
    H: hwa::SyncMutexStrategy + 'static,
    H::Resource: Pwm + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty: Into<u32> + From<u16> + Copy,
{
    fn clone(&self) -> Self {
        Self::new(self.servo.clone(), self.channel.clone())
    }
}

impl<H> ProbeTrait for GenericServoController<H>
where
    H: hwa::SyncMutexStrategy + 'static,
    H::Resource: Pwm + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty: Into<u32> + From<u16> + Copy,
{
    /// Lowers the probe pin, causing it to make contact. The operation waits for a specified duration.
    ///
    /// This method is part of the `ProbeTrait` implementation for the `ServoController`
    /// and sets the probe to an angle of 10 degrees.
    ///
    /// # Parameters
    ///
    /// * `sleep_us` - Duration in microseconds to sleep after lowering the pin.
    ///
    /// # Example
    ///
    /// ```rust
    /// let mut controller = ServoController::new(servo, channel);
    /// controller.probe_pin_down(1000).await;
    /// ```
    async fn probe_pin_down(&mut self, sleep_us: u64) {
        self.set_angle(10, sleep_us).await;
    }

    async fn probe_deploy_latched(&mut self, deploy_sleep_us: u64, mode_sleep_us: u64) {
        // An alarm left by a previous operation would make the BLTouch ignore the deploy
        self.set_angle(160, BLTOUCH_COMMAND_SETTLE_US).await;
        self.set_angle(10, deploy_sleep_us).await;
        self.set_angle(60, mode_sleep_us).await;
    }

    /// Stows the pin. A BLTouch ignores the stow while in alarm (red led blinking, e.g. after a
    /// deploy with the pin blocked or a missed touch), so the alarm is released first (160º).
    async fn probe_pin_up(&mut self, sleep_us: u64) {
        self.set_angle(160, BLTOUCH_COMMAND_SETTLE_US).await;
        self.set_angle(90, sleep_us).await;
    }
    async fn probe_self_test(&mut self, sleep_us: u64) {
        self.set_angle(120, sleep_us).await;
    }
    async fn probe_alarm_release(&mut self, sleep_us: u64) {
        self.set_angle(160, sleep_us).await;
    }

    async fn probe_test_mode(&mut self, sleep_us: u64) {
        self.set_angle(60, sleep_us).await;
    }

    async fn probe_set_angle(&mut self, angle: u16, sleep_us: u64) {
        self.set_angle(angle, sleep_us).await;
    }

    fn probe_pwm_state(&mut self) -> (u32, u32) {
        let channel = self.channel;
        self.servo
            .apply(|servo| (servo.get_duty(channel).into(), servo.get_max_duty().into()))
    }
}
