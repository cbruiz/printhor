#![no_std]
#[feature(async_fn_in_trait)]
#[cfg(feature = "with-defmt")]
pub use defmt::info;
#[cfg(feature = "with-log")]
pub use log::info;

mod tracked_static_cell;
pub use tracked_static_cell::TrackedStaticCell;
mod event_bus;
pub use event_bus::*;

mod shared_controller;
pub use shared_controller::*;

mod context;
pub use context::*;

pub use tracked_static_cell::COUNTER;

#[cfg(feature = "with-ui")]
mod display;
#[cfg(feature = "with-ui")]
pub use display::DisplayScreenUI;