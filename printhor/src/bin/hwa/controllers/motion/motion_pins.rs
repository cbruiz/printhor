use crate::hwa;
use hwa::SyncMutexStrategy;
use printhor_hwa_common::StepperChannel;

pub struct MotionPins {
    pins: hwa::StaticSyncController<hwa::types::MotionPinsMutexStrategy>
}

impl MotionPins {
    pub fn new(pins: hwa::StaticSyncController<hwa::types::MotionPinsMutexStrategy>) -> Self {
        Self { pins }
    }

    pub fn enable_steppers(&self, channels: StepperChannel) {
        self.pins.apply_mut(|pins| {
            if channels.contains(StepperChannel::X) {
                pins.enable_x_stepper()
            }
            if channels.contains(StepperChannel::Y) {
                pins.enable_y_stepper()
            }
            if channels.contains(StepperChannel::Z) {
                pins.enable_z_stepper()
            }
            if channels.contains(StepperChannel::E) {
                pins.enable_e_stepper()
            }
        })
    }

    pub fn set_forward_direction(&self, channels: StepperChannel) {
        self.pins.apply_mut(|pins| {
            pins.set_forward_direction(channels);
        })
    }

    pub fn disable_steppers(&self, channels: StepperChannel) {
        self.pins.apply_mut(|pins| {
            if channels.contains(StepperChannel::X) {
                pins.disable_x_stepper()
            }
            if channels.contains(StepperChannel::Y) {
                pins.disable_y_stepper()
            }
            if channels.contains(StepperChannel::Z) {
                pins.disable_z_stepper()
            }
            if channels.contains(StepperChannel::E) {
                pins.disable_e_stepper()
            }
        })
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
        self.pins.apply_mut(|pins| {
            let mut triggered = false;
            if channels.contains(StepperChannel::X) {
                triggered |= pins.endstop_triggered(StepperChannel::X)
            }
            if channels.contains(StepperChannel::Y) {
                triggered |= pins.endstop_triggered(StepperChannel::Y)
            }
            if channels.contains(StepperChannel::Z) {
                triggered |= pins.endstop_triggered(StepperChannel::Z)
            }
            if channels.contains(StepperChannel::E) {
                triggered |= pins.endstop_triggered(StepperChannel::E)
            }
            triggered
        })
    }

    pub fn step_toggle(&self, channels: StepperChannel) {
        self.pins.apply_mut(|pins| {

            if channels.contains(StepperChannel::X) {
                pins.x_step_pin.toggle();
            }
            if channels.contains(StepperChannel::Y) {
                pins.y_step_pin.toggle();
            }
            if channels.contains(StepperChannel::Z) {
                pins.z_step_pin.toggle();
            }
            if channels.contains(StepperChannel::E) {
                pins.e_step_pin.toggle();
            }
        });
    }

}

impl Clone for MotionPins {
    fn clone(&self) -> Self {
        Self::new(self.pins.clone())
    }
}

