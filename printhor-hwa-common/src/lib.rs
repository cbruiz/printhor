#![no_std]
#![cfg_attr(feature="nightly", feature(async_fn_in_trait))]
#[cfg(all(feature = "with-defmt", not(feature = "with-log")))]
pub use defmt::info;
#[cfg(all(feature = "with-log", not(feature = "with-defmt")))]
pub use log::info;

mod tracked_static_cell;
pub use tracked_static_cell::TrackedStaticCell;
mod event_bus;
pub use event_bus::*;

mod defer_channel;
pub use defer_channel::*;

mod shared_controller;
pub use shared_controller::*;

mod context;
pub use context::*;

pub use tracked_static_cell::COUNTER;

#[cfg(feature = "with-ui")]
mod display;

#[cfg(feature = "with-ui")]
pub use display::DisplayScreenUI;

pub mod soft_uart;
