//! Motion step actuator controller/adaptor
use crate::hwa;
use hwa::SyncMutexStrategy;

pub struct StepActuatorController {
    actuator: hwa::StaticSyncController<hwa::types::StepActuatorMutexStrategy>,
    #[cfg(feature = "with-motion-broadcast")]
    pub broadcast_channel: hwa::types::MotionBroadcastChannel,
}

impl StepActuatorController {
    pub const fn new(
        actuator: hwa::StaticSyncController<hwa::types::StepActuatorMutexStrategy>,
        #[cfg(feature = "with-motion-broadcast")]
        broadcast_channel: hwa::types::MotionBroadcastChannel,
    ) -> Self {
        Self {
            actuator,
            #[cfg(feature = "with-motion-broadcast")]
            broadcast_channel,
        }
    }

    pub fn enable_steppers(&self, channels: hwa::CoordSel) {
        use hwa::traits::StepActuatorTrait;
        hwa::debug!("enable_steppers {:?}", channels);
        self.actuator
            .apply_mut(|pins| pins.set_enabled(channels, true))
    }

    pub fn set_forward_direction(&self, channels: hwa::CoordSel, mask: hwa::CoordSel) {
        use hwa::traits::StepActuatorTrait;
        hwa::debug!("set_forward_direction {:?}", channels);
        #[cfg(all(test, feature = "native"))]
        step_trace::set_direction(channels, mask);
        self.actuator.apply_mut(|pins| {
            pins.set_forward_direction(channels, mask);
        })
    }

    pub fn disable_steppers(&self, channels: hwa::CoordSel) {
        use hwa::traits::StepActuatorTrait;
        hwa::debug!("disable_steppers {:?}", channels);
        self.actuator.apply_mut(|pins| pins.disable(channels))
    }

    /// The drivers whose (active low) enable pin is asserted in the simulator.
    #[cfg(all(test, feature = "native"))]
    pub fn enabled_steppers(&self) -> hwa::CoordSel {
        self.actuator.apply_mut(|_pins| {
            #[allow(unused_mut)]
            let mut enabled = hwa::CoordSel::empty();
            #[cfg(feature = "with-x-axis")]
            enabled.set(hwa::CoordSel::X, _pins.x_enable_pin.is_low());
            #[cfg(feature = "with-y-axis")]
            enabled.set(hwa::CoordSel::Y, _pins.y_enable_pin.is_low());
            #[cfg(feature = "with-z-axis")]
            enabled.set(hwa::CoordSel::Z, _pins.z_enable_pin.is_low());
            enabled
        })
    }

    pub fn end_stop_triggered(&self, channels: hwa::CoordSel) -> bool {
        use hwa::traits::StepActuatorTrait;
        self.actuator
            .apply_mut(|pins| pins.endstop_triggered(channels))
    }

    pub fn step_toggle(&self, channels: hwa::CoordSel) {
        use hwa::traits::StepActuatorTrait;
        hwa::trace!("step_toggle {:?}", channels);

        #[cfg(all(test, feature = "native"))]
        step_trace::step(channels);
        self.actuator.apply_mut(|pins| pins.step_toggle(channels));
    }
}

/// Test instrumentation of the native simulator: keeps the signed step count of every motor
/// (according to the last applied direction) and, while recording, the lattice position reached
/// after each step pulse (simultaneous pulses of several motors make a single position). The
/// extruder is counted apart (see [step_trace::extruder]): it is not part of the positions.
#[cfg(all(test, feature = "native"))]
pub mod step_trace {
    use crate::hwa;
    use std::sync::Mutex;

    /// Signed step count of X, Y and Z motors
    pub type Steps = [i64; 3];

    struct Trace {
        forward: [bool; 3],
        steps: Steps,
        recording: Option<std::vec::Vec<Steps>>,
        /// Extruder: direction, signed step count and steps done backwards
        #[cfg(feature = "with-e-axis")]
        e_forward: bool,
        #[cfg(feature = "with-e-axis")]
        e_steps: i64,
        #[cfg(feature = "with-e-axis")]
        e_backwards: i64,
    }

    static TRACE: Mutex<Trace> = Mutex::new(Trace {
        forward: [true; 3],
        steps: [0; 3],
        recording: None,
        #[cfg(feature = "with-e-axis")]
        e_forward: true,
        #[cfg(feature = "with-e-axis")]
        e_steps: 0,
        #[cfg(feature = "with-e-axis")]
        e_backwards: 0,
    });

    const AXES: [hwa::CoordSel; 3] = [hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z];

    pub(crate) fn set_direction(channels: hwa::CoordSel, mask: hwa::CoordSel) {
        let mut t = TRACE.lock().unwrap();
        for (i, axis) in AXES.iter().enumerate() {
            if mask.contains(*axis) {
                t.forward[i] = channels.contains(*axis);
            }
        }
        #[cfg(feature = "with-e-axis")]
        if mask.contains(hwa::CoordSel::E) {
            t.e_forward = channels.contains(hwa::CoordSel::E);
        }
    }

    pub(crate) fn step(channels: hwa::CoordSel) {
        let mut t = TRACE.lock().unwrap();
        let mut moved = false;
        for (i, axis) in AXES.iter().enumerate() {
            if channels.contains(*axis) {
                t.steps[i] += if t.forward[i] { 1 } else { -1 };
                moved = true;
            }
        }
        if moved {
            let steps = t.steps;
            if let Some(r) = t.recording.as_mut() {
                r.push(steps);
            }
        }
        #[cfg(feature = "with-e-axis")]
        if channels.contains(hwa::CoordSel::E) {
            match t.e_forward {
                true => t.e_steps += 1,
                false => {
                    t.e_steps -= 1;
                    t.e_backwards += 1;
                }
            }
        }
    }

    /// The signed step count of the extruder motor, and the steps it did backwards
    #[cfg(feature = "with-e-axis")]
    pub fn extruder() -> (i64, i64) {
        let t = TRACE.lock().unwrap();
        (t.e_steps, t.e_backwards)
    }

    /// The current step counts
    pub fn current() -> Steps {
        TRACE.lock().unwrap().steps
    }

    /// Starts recording. Returns the current step counts (the origin of the recording)
    pub fn start() -> Steps {
        let mut t = TRACE.lock().unwrap();
        t.recording = Some(std::vec::Vec::new());
        t.steps
    }

    /// Stops recording and returns every position reached since [start]
    pub fn stop() -> std::vec::Vec<Steps> {
        TRACE.lock().unwrap().recording.take().unwrap_or_default()
    }
}

impl Clone for StepActuatorController {
    fn clone(&self) -> Self {
        Self::new(
            self.actuator.clone(),
            #[cfg(feature = "with-motion-broadcast")]
            self.broadcast_channel.clone(),
        )
    }
}
/*
#[cfg(feature = "assert-motion")]
pub mod assertion {
    pub struct MotionStepActuatorStateController {

    }

    impl MotionStepActuatorStateController {

    }

    pub static ACTUATOR_STATE: MotionStepActuatorStateController = MotionStepActuatorStateController {

    };
}
*/
