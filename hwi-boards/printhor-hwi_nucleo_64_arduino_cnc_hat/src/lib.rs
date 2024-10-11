#![no_std]
extern crate alloc;

cfg_if::cfg_if! {
    if #[cfg(feature="nucleo64-f410rb")] {
        mod board_stm32f4;
        pub use board_stm32f4::*;
        pub use board_stm32f4::device;
    }

}
