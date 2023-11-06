use crate::hwi::native::PwmChannel;
use crate::hwi::native::device::{OutputPin};

pub(crate) struct MockedPwm {
    #[allow(unused)]
    p: OutputPin,
}
impl MockedPwm {
    pub(crate) const fn new() -> Self {
        Self {
            p: OutputPin::new(),
        }
    }

    pub(crate) fn get_max_duty(&self) -> u16 {
        16386u16
    }

    pub(crate) fn set_duty(&mut self, _c: PwmChannel, _d: u16) {

    }

    pub(crate) fn enable(&mut self, _c: PwmChannel) {

    }

    pub(crate) fn disable(&mut self, _c: PwmChannel) {

    }
}