#![no_std]
extern crate alloc;

use alloc::boxed::Box;

pub use defmt;
pub use defmt::{debug, error, info, trace, warn};

mod board;

pub use board::device;
pub use board::Controllers;
pub use board::IODevices;
pub use board::MotionDevices;
pub use board::PwmDevices;
pub use board::SysDevices;

pub use board::heap_current_size;
pub use board::init;
pub use board::setup;
pub use board::stack_reservation_current_size;
pub use board::HEAP_SIZE_BYTES;
pub use board::MACHINE_BOARD;
pub use board::MACHINE_PROCESSOR;
pub use board::MACHINE_TYPE;
pub use board::MAX_STATIC_MEMORY;
#[cfg(feature = "with-sd-card")]
pub use board::SDCARD_PARTITION;
#[cfg(feature = "with-serial-usb")]
const USBSERIAL_BUFFER_SIZE: usize = 32;
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        const UART_PORT1_BUFFER_SIZE: usize = 32;
        const UART_PORT1_BAUD_RATE: u32 = 115200;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        const UART_PORT2_BUFFER_SIZE: usize = 32;
        const UART_PORT2_BAUD_RATE: u32 = 115200;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
        pub const SEGMENT_QUEUE_SIZE: u8 = 10;
        compile_error("Incomplete. Pending: Ticket, etc")
    }
}

pub use board::ADC_START_TIME_US;
pub use board::ADC_VREF_DEFAULT_MV;
use defmt::unwrap;
use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use printhor_hwa_common::TrackedStaticCell;

#[link_section = ".bss"]
static CORE1_STACK: TrackedStaticCell<Stack<4096>> = TrackedStaticCell::new();
#[link_section = ".bss"]
static EXECUTOR_HIGH: TrackedStaticCell<Executor> = TrackedStaticCell::new();

struct TokenHolder<S> {
    token: embassy_executor::SpawnToken<S>,
}

unsafe impl<S> Sync for TokenHolder<S> {}
unsafe impl<S> Send for TokenHolder<S> {}

#[inline]
pub fn launch_high_priotity<S: 'static + Send>(
    core: device::TaskStepperCore,
    token: embassy_executor::SpawnToken<S>,
) -> Result<(), ()> {
    // TODO: There must be a better way to tackle this
    let r = Box::new(TokenHolder { token });
    let stack = CORE1_STACK.init::<{ crate::MAX_STATIC_MEMORY }>("executor1::stack", Stack::new());

    spawn_core1(core, stack, || {
        let executor1 =
            EXECUTOR_HIGH.init::<{ crate::MAX_STATIC_MEMORY }>("executor1", Executor::new());
        executor1.run(|spawner| unwrap!(spawner.spawn(r.token)))
    });
    Ok(())
}

#[inline]
pub fn init_logger() {}

#[inline]
pub fn sys_reset() {
    cortex_m::peripheral::SCB::sys_reset();
}
