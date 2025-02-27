#![no_std]
#![allow(stable_features)]
extern crate alloc;

mod board_stm32f4;
pub use board::Contract;
use board_stm32f4 as board;
