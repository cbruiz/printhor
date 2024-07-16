use std::collections::HashMap;
use super::mocked_pin::MockedIOPin;
use super::mocked_pin::PinStateRef;
use embedded_hal_02::Pwm;

pub type PwmChannel = u8;

pub struct MockedPwm {
    #[allow(unused)]
    p: MockedIOPin,
    // Allow state by channel to share pwm in the same way a MCU does
    duty_map: HashMap<PwmChannel, <Self as Pwm>::Duty>,

}
impl MockedPwm {
    #[allow(unused)]
    pub(crate) fn new(id: u8, pin_state: PinStateRef) -> Self {
        Self {
            p: MockedIOPin::new(id, pin_state),
            duty_map: HashMap::new(),
        }
    }
}

impl Pwm for MockedPwm {
    fn set_period<P>(&mut self, _: P) where P: Into<u16> {  }
    fn set_duty(&mut self, channel: <Self as Pwm>::Channel, duty: <Self as Pwm>::Duty) {
        let clamped_duty = duty.min(self.get_max_duty());
        let duty_of_channel = *self.duty_map.get(&channel).unwrap_or(&0);
        log::trace!("set_duty: curr: {}, max: {}, next: {}",
            duty_of_channel,
            self.get_max_duty(),
            clamped_duty,
        );
        self.duty_map.insert(channel, clamped_duty);
    }
    fn get_max_duty(&self) -> <Self as Pwm>::Duty { 16384u32  }
    fn get_duty(&self, channel: <Self as Pwm>::Channel) -> <Self as Pwm>::Duty {
        let duty_of_channel = *self.duty_map.get(&channel).unwrap_or(&0);
        log::trace!("get_duty(channel: {}) = {}", channel, duty_of_channel);
        duty_of_channel
    }
    fn get_period(&self) -> <Self as Pwm>::Time { 0u16 }
    fn enable(&mut self, _: <Self as Pwm>::Channel) {

    }
    fn disable(&mut self, channel: <Self as Pwm>::Channel) {
        self.duty_map.insert(channel, 0);
    }
    type Duty = u32;
    type Time = u16;
    type Channel = u8;
}