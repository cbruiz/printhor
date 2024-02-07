use bitflags::bitflags;
mod plan;
pub use plan::*;

bitflags! {
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub struct StepperChannel: u8 {
        const X    = 0b00000001;
        const Y    = 0b00000010;
        const Z    = 0b00000100;
        #[cfg(feature="has-extruder")]
        const E    = 0b00001000;
    }
}
