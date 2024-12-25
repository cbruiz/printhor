use printhor_hwa_common as hwa;
use std::collections::HashMap;
use super::mocked_pin::MockedIOPin;
use super::mocked_pin::PinStateRef;

pub struct MockedPwm {
    #[allow(unused)]
    p: MockedIOPin,
    // Allow state by channel to share pwm in the same way the MCU does
    duty_map: HashMap<u8, <Self as hwa::traits::Pwm>::Duty>,

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

impl hwa::traits::Pwm for MockedPwm {
    type Channel = u8;
    type Time = u16;
    type Duty = u32;
    fn disable(&mut self, channel: <Self as hwa::traits::Pwm>::Channel) {
        self.duty_map.insert(channel, 0);
    }
    fn enable(&mut self, _: <Self as hwa::traits::Pwm>::Channel) {

    }
    fn get_period(&self) -> <Self as hwa::traits::Pwm>::Time { 0u16 }
    fn get_duty(&self, channel: <Self as hwa::traits::Pwm>::Channel) -> <Self as hwa::traits::Pwm>::Duty {
        let duty_of_channel = *self.duty_map.get(&channel).unwrap_or(&0);
        hwa::trace!("get_duty(channel: {}) = {}", channel, duty_of_channel);
        duty_of_channel
    }
    fn get_max_duty(&self) -> <Self as hwa::traits::Pwm>::Duty { 16384u32  }
    fn set_duty(&mut self, channel: <Self as hwa::traits::Pwm>::Channel, duty: <Self as hwa::traits::Pwm>::Duty) {
        let clamped_duty = duty.min(self.get_max_duty());
        hwa::trace!("set_duty: curr: {}, max: {}, next: {}",
            *self.duty_map.get(&channel).unwrap_or(&0),
            self.get_max_duty(),
            clamped_duty,
        );
        self.duty_map.insert(channel, clamped_duty);
    }
    fn set_period<P>(&mut self, _: P) where P: Into<u16> {  }
}