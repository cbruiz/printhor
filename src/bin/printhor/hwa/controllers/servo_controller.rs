//! TODO: This feature is still in incubation
use crate::hwa;
#[allow(unused)]
use embedded_hal_02::Pwm;
use crate::InterruptControllerRef;
#[allow(unused)]
pub trait ProbeTrait {
    async fn probe_pin_down(&mut self, sleep_us: u64);
    async fn probe_pin_up(&mut self, sleep_us: u64);
    async fn probe_self_test(&mut self, sleep_us: u64);
    async fn probe_alarm_release(&mut self, sleep_us: u64);
    async fn probe_test_mode(&mut self, sleep_us: u64);
}

pub struct ServoController {
    servo: InterruptControllerRef<hwa::device::PwmServo>,
    channel: hwa::device::PwmChannel,
}

impl ServoController {

    pub fn new(servo: InterruptControllerRef<hwa::device::PwmServo>, channel: hwa::device::PwmChannel) -> Self {
        Self {servo, channel}
    }

    pub async fn set_angle(&mut self, angle: u16, sleep_us: u64) {

        let max_duty = self.servo.lock().await.get_max_duty() as u32;
        // 100% duty period width (uS)
        const PERIOD: u32 = 20_000;
        // minimum pulse width (uS)
        const MIN_PULSE_WIDTH: u32 = 600;
        // period width by degree (uS/deg)
        const US_BY_DEG: u32 = 10;

        let duty_us = MIN_PULSE_WIDTH + US_BY_DEG * (angle as u32);
        let duty_cnt = ((duty_us * max_duty) / PERIOD) as u16;

        hwa::debug!("Set probe angle: {} : duty: {} uS | {} max_duty: {}", angle, duty_us, duty_cnt, max_duty);
        self.servo.lock().await.disable(self.channel);
        self.servo.lock().await.set_duty(self.channel, duty_cnt.into());
        self.servo.lock().await.enable(self.channel);
        embassy_time::Timer::after_micros(sleep_us).await;
    }

}

impl ProbeTrait for ServoController {
    #[allow(unused)]
    #[inline(always)]
    async fn probe_pin_down(&mut self, sleep_us: u64) {
        self.set_angle(10, sleep_us).await;
    }
    #[allow(unused)]
    #[inline(always)]
    async fn probe_pin_up(&mut self, sleep_us: u64) {
        self.set_angle(90, sleep_us).await;
    }
    #[allow(unused)]
    #[inline(always)]
    async fn probe_self_test(&mut self, sleep_us: u64) {
        self.set_angle(120, sleep_us).await;
    }
    #[allow(unused)]
    #[inline(always)]
    async fn probe_alarm_release(&mut self, sleep_us: u64) {
        self.set_angle(160, sleep_us).await;
    }

    #[allow(unused)]
    #[inline(always)]
    async fn probe_test_mode(&mut self, sleep_us: u64) {
        self.set_angle(60, sleep_us).await;
    }
}

