//noinspection RsDetachedFile
// Math module
#[path = "../../src/bin/printhor/control/mod.rs"]
pub mod control;

#[path = "../../src/bin/printhor/tgeo.rs"]
pub mod tgeo;

#[path = "../../src/bin/printhor/math/mod.rs"]
pub mod math;

#[path = "../../src/bin/printhor/hwa/mod.rs"]
mod hwa_core;

#[path = "../../src/bin/printhor/helpers/mod.rs"]
pub mod helpers;

#[path = "../../src/bin/printhor/machine.rs"]
pub mod machine;


pub mod hwi {
    pub const MACHINE_BOARD: &str = "native";
    pub const MACHINE_PROCESSOR: &str = "native";
    pub const PROCESSOR_SYS_CK_MHZ: u32 = 100_000_000;

    pub const SEGMENT_QUEUE_SIZE: u8 = 10;

    pub const MAX_STATIC_MEMORY: usize = 16386;
    pub const HEAP_SIZE_BYTES: usize = 1024;

    /// Micro-segment sampling frequency in Hz
    pub const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 100;
    /// Micro-segment clock frequency in Hz
    pub const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;


    use super::hwa_core as hwa;
    pub type EventbusMutexType = hwa::SyncSendMutex;
    pub type EventBusChannelMutexType = hwa::NoopMutex;
    pub type DeferChannelMutexType = hwa::NoopMutex;
    pub type WatchdogMutexType = hwa::NoopMutex;
    pub type MotionDriverMutexType = hwa::SyncSendMutex;
    pub type MotionPlannerMutexType = hwa::NoopMutex;
    pub type MotionConfigMutexType = hwa::SyncSendMutex;
    pub type MotionStatusMutexType = hwa::NoopMutex;
    pub type MotionRingBufferMutexType = hwa::NoopMutex;
    pub type MotionSignalMutexType = hwa::NoopMutex;
    pub type SerialPort1MutexType = hwa::SyncSendMutex;

    pub mod device {
        use crate::prelude::tgeo::CoordSel;
        use printhor_hwa_common::StepperChannel;

        #[derive(Clone)]
        pub struct MotionPins {}

        impl MotionPins {
            pub fn new() -> Self {
                Self {}
            }
            pub fn enable(&mut self, channels: StepperChannel) {
                todo!()
            }

            pub fn disable(&mut self, channels: StepperChannel) {
                todo!()
            }

            pub fn set_forward_direction(&mut self, channels: StepperChannel) {
                todo!()
            }

            pub fn step_high(&mut self, channels: StepperChannel) {
                todo!()
            }

            pub fn step_low(&mut self, channels: StepperChannel) {
                todo!()
            }

            pub fn step_toggle(&mut self, _channels: StepperChannel) {}

            pub fn endstop_triggered(&mut self, channels: StepperChannel) -> bool {
                let mut triggered = false;
                #[cfg(feature = "with-x-axis")]
                if channels.contains(StepperChannel::X) {
                    //triggered |= self.pins.x_endstop_pin.is_high();
                }
                #[cfg(feature = "with-y-axis")]
                if channels.contains(StepperChannel::Y) {
                    //triggered |= self.pins.y_endstop_pin.is_high();
                }
                #[cfg(feature = "with-z-axis")]
                if channels.contains(StepperChannel::Z) {
                    //triggered |= self.pins.z_endstop_pin.is_high();
                }
                #[cfg(feature = "with-e-axis")]
                if channels.contains(StepperChannel::E) {
                    //triggered |= self.pins.e_endstop_pin.is_high();
                }
                triggered
            }
        }

        pub struct MotionDevice {
            pub motion_pins: MotionPins,
        }

        pub struct Watchdog {}
    }
}

pub mod hwa {
    pub use super::hwa_core::*;
    pub use crate::hwi::*;

    //#region "Mutex types for this board"

    //#endregion
}
