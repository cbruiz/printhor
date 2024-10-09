//! TODO: This feature is still in incubation
use crate::hwa;
use embedded_hal_02::Pwm;
use hwa::{MutexStrategy, StaticController};

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
    fn probe_pin_down(&mut self, sleep_us: u64) -> impl std::future::Future<Output = ()>;

    /// Raises the probe pin, moving it away from contact. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after raising the pin.
    fn probe_pin_up(&mut self, sleep_us: u64) -> impl std::future::Future<Output = ()>;

    /// Initiates a self-test sequence for the probe. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after performing the self-test.
    fn probe_self_test(&mut self, sleep_us: u64) -> impl std::future::Future<Output = ()>;

    /// Releases any active probe alarms. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after releasing the alarm.
    fn probe_alarm_release(&mut self, sleep_us: u64) -> impl std::future::Future<Output = ()>;

    /// Sets the probe into test mode. The operation waits for a specified duration.
    ///
    /// # Parameters
    /// * `sleep_us` - Duration in microseconds to sleep after entering test mode.
    fn probe_test_mode(&mut self, sleep_us: u64) -> impl std::future::Future<Output = ()>;
}

/// The `ServoController` structure is responsible for managing a servo motor
/// using an interrupt-controlled PWM (Pulse Width Modulation) channel.
/// It provides methods to control the angle of the servo and implements the `ProbeTrait` for additional probe-related actions.
///
/// # Fields
///
/// * `servo` - A reference to a static controller PWM servo (PwmServo).
/// * `channel` - The specific PWM channel associated with the servo motor.
pub struct ServoController<H>
where
    H: MutexStrategy + Send + 'static,
    H::Resource: Pwm + Send + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty: Into<u32> + From<u16>,
{
    /// A reference to a static controller PWM servo.
    servo: StaticController<H>,

    /// The specific PWM channel associated with the servo motor.
    channel: <H::Resource as Pwm>::Channel,
}

impl<H> ServoController<H>
where
    H: MutexStrategy + Send + 'static,
    H::Resource: Pwm + Send + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty: Into<u32> + From<u16>,
{
    pub fn new(servo: StaticController<H>, channel: <H::Resource as Pwm>::Channel) -> Self {
        Self { servo, channel }
    }

    pub async fn set_angle(&mut self, angle: u16, sleep_us: u64) {
        {
            let mut mg = self.servo.lock().await;
            let max_duty = mg.get_max_duty();
            // 100% duty period width (uS)
            const PERIOD: u32 = 20_000;
            // minimum pulse width (uS)
            const MIN_PULSE_WIDTH: u32 = 600;
            // period width by degree (uS/deg)
            const US_BY_DEG: u32 = 10;

            let duty_us = MIN_PULSE_WIDTH + US_BY_DEG * (angle as u32);
            let duty_cnt = (duty_us * max_duty.into() / PERIOD) as u16;

            #[cfg(feature = "trace-commands")]
            hwa::info!(
                "Set probe angle: {} : duty: {} uS | {} max_duty: {:?}",
                angle,
                duty_us,
                duty_cnt,
                max_duty,
            );
            mg.disable(self.channel);
            mg.set_duty(self.channel, duty_cnt.into());
            mg.enable(self.channel);
        }
        embassy_time::Timer::after_micros(sleep_us).await;
    }
}

impl<H> ProbeTrait for ServoController<H>
where
    H: MutexStrategy + Send + 'static,
    H::Resource: Pwm + Send + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty: Into<u32> + From<u16>,
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

    async fn probe_pin_up(&mut self, sleep_us: u64) {
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
}
