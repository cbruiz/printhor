use super::mocked_pin::MockedIOPin;
use super::mocked_pin::PinStateRef;
use embedded_hal_02::Pwm;

pub type PwmChannel = u8;

pub struct MockedPwm {
    #[allow(unused)]
    p: MockedIOPin,
    duty: <Self as Pwm>::Duty,
}
impl MockedPwm {
    #[allow(unused)]
    pub(crate) const fn new(id: u8, pin_state: PinStateRef) -> Self {
        Self {
            p: MockedIOPin::new(id, pin_state),
            duty: 0,
        }
    }
}

impl Pwm for MockedPwm {
    fn set_period<P>(&mut self, _: P) where P: Into<u16> {  }
    fn set_duty(&mut self, _ch: <Self as Pwm>::Channel, duty: <Self as Pwm>::Duty) {
        let clamped_duty = duty.min(self.get_max_duty());
        log::debug!("set_duty: curr: {}, max: {}, next: {}",
            self.duty,
            self.get_max_duty(),
            clamped_duty,
        );
        self.duty = clamped_duty;
    }
    fn get_max_duty(&self) -> <Self as Pwm>::Duty { 16384u16  }
    fn get_duty(&self, _: <Self as Pwm>::Channel) -> <Self as Pwm>::Duty {
        self.duty
    }
    fn get_period(&self) -> <Self as Pwm>::Time { 0u16 }
    fn enable(&mut self, _: <Self as Pwm>::Channel) {
        self.duty = 0;
    }
    fn disable(&mut self, _: <Self as Pwm>::Channel) {
        self.duty = 0;
    }
    type Duty = u16;
    type Time = u16;
    type Channel = u8;
}