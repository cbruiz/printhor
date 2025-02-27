#![no_std]
extern crate alloc;

cfg_if::cfg_if! {
    if #[cfg(feature="skr_mini_e3_v3")] {
        mod board_stm32g0;
        use board_stm32g0 as board;
    }
    else {
        compile_error!("You didn't specify any board");
    }

}
pub use board::Contract;