use bitflags::bitflags;
mod plan;
pub use plan::*;

bitflags! {
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub struct StepperChannel: u8 {
        const X    = 0b00000001;
        const Y    = 0b00000010;
        const Z    = 0b00000100;
        #[cfg(feature="with-hot-end")]
        const E    = 0b00001000;
        #[cfg(feature="with-hot-end")]
        const ALL  = Self::X.bits() | Self::Y.bits() | Self::Z.bits() | Self::E.bits();
        #[cfg(not(feature="with-hot-end"))]
        const ALL  = Self::X.bits() | Self::Y.bits() | Self::Z.bits();
        const UNSET  = 0b10000000;
    }
}
