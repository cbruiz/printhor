use crate::hwa;
use hwa::SyncMutexStrategy;
use printhor_hwa_common::StepperChannel;

pub struct MotionPins {
    pins: hwa::StaticSyncController<hwa::types::MotionPinsMutexStrategy>,
}

impl MotionPins {
    pub fn new(pins: hwa::StaticSyncController<hwa::types::MotionPinsMutexStrategy>) -> Self {
        Self { pins }
    }

    pub fn enable_steppers(&self, channels: StepperChannel) {
        use hwa::traits::MotionPinsTrait;
        self.pins.apply_mut(|pins| pins.set_enabled(channels, true))
    }

    pub fn set_forward_direction(&self, channels: StepperChannel, mask: StepperChannel) {
        use hwa::traits::MotionPinsTrait;
        self.pins.apply_mut(|pins| {
            pins.set_forward_direction(channels, mask);
        })
    }

    pub fn disable_steppers(&self, channels: StepperChannel) {
        use hwa::traits::MotionPinsTrait;
        self.pins.apply_mut(|pins| pins.disable(channels))
    }

    #[cfg(feature = "native")]
    pub fn set_end_stop_high(&self, channels: StepperChannel) {
        self.pins.apply_mut(|pins| {
            if channels.contains(StepperChannel::X) {
                pins.x_endstop_pin.set_high();
            }
            if channels.contains(StepperChannel::Y) {
                pins.y_endstop_pin.set_high();
            }
            if channels.contains(StepperChannel::Z) {
                pins.z_endstop_pin.set_high();
            }
        });
    }

    pub fn end_stop_triggered(&self, channels: StepperChannel) -> bool {
        use hwa::traits::MotionPinsTrait;
        self.pins.apply_mut(|pins| pins.endstop_triggered(channels))
    }

    pub fn step_toggle(&self, channels: StepperChannel) {
        use hwa::traits::MotionPinsTrait;
        self.pins.apply_mut(|pins| pins.step_toggle(channels));
    }
}

impl Clone for MotionPins {
    fn clone(&self) -> Self {
        Self::new(self.pins.clone())
    }
}
