use crate::hwa;
use hwa::SyncMutexStrategy;

pub struct MotionPins {
    pins: hwa::StaticSyncController<hwa::types::MotionPinsMutexStrategy>,
    #[cfg(feature = "with-motion-broadcast")]
    pub broadcast_channel: hwa::types::MotionBroadcastChannel,
}

impl MotionPins {
    pub fn new(
        pins: hwa::StaticSyncController<hwa::types::MotionPinsMutexStrategy>,
        #[cfg(feature = "with-motion-broadcast")]
        broadcast_channel: hwa::types::MotionBroadcastChannel,
    ) -> Self {
        Self {
            pins,
            #[cfg(feature = "with-motion-broadcast")]
            broadcast_channel,
        }
    }

    pub fn enable_steppers(&self, channels: hwa::CoordSel) {
        use hwa::traits::MotionPinsTrait;
        hwa::trace!("enable_steppers {:?}", channels);
        self.pins.apply_mut(|pins| pins.set_enabled(channels, true))
    }

    pub fn set_forward_direction(&self, channels: hwa::CoordSel, mask: hwa::CoordSel) {
        use hwa::traits::MotionPinsTrait;
        hwa::trace!("set_forward_direction {:?}", channels);
        self.pins.apply_mut(|pins| {
            pins.set_forward_direction(channels, mask);
        })
    }

    pub fn disable_steppers(&self, channels: hwa::CoordSel) {
        use hwa::traits::MotionPinsTrait;
        hwa::trace!("disable_steppers {:?}", channels);
        self.pins.apply_mut(|pins| pins.disable(channels))
    }

    #[cfg(feature = "native")]
    pub fn set_end_stop_high(&self, _channels: hwa::CoordSel) {
        self.pins.apply_mut(|_pins| {
            #[cfg(feature = "with-x-axis")]
            if _channels.contains(hwa::CoordSel::X) {
                _pins.x_endstop_pin.set_high();
            }
            #[cfg(feature = "with-y-axis")]
            if _channels.contains(hwa::CoordSel::Y) {
                _pins.y_endstop_pin.set_high();
            }
            #[cfg(feature = "with-z-axis")]
            if _channels.contains(hwa::CoordSel::Z) {
                _pins.z_endstop_pin.set_high();
            }
        });
    }

    pub fn end_stop_triggered(&self, channels: hwa::CoordSel) -> bool {
        use hwa::traits::MotionPinsTrait;
        self.pins.apply_mut(|pins| pins.endstop_triggered(channels))
    }

    pub fn step_toggle(&self, channels: hwa::CoordSel) {
        use hwa::traits::MotionPinsTrait;
        hwa::trace!("step_togle {:?}", channels);
        self.pins.apply_mut(|pins| pins.step_toggle(channels));
    }
}

impl Clone for MotionPins {
    fn clone(&self) -> Self {
        Self::new(
            self.pins.clone(),
            #[cfg(feature = "with-motion-broadcast")]
            self.broadcast_channel.clone(),
        )
    }
}
