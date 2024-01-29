#![cfg_attr(feature="niglty", feature(trait_alias))]
#![cfg_attr(feature="niglty", feature(type_alias_impl_trait))]
#![allow(stable_features)]

mod board;

pub use log::{trace,debug,info,warn,error};

pub use board::device;
pub use board::consts;
pub use board::IODevices;
pub use board::Controllers;
pub use board::MotionDevices;
pub use board::PwmDevices;

pub use board::init;
pub use board::setup;
pub use board::heap_current_size;
pub use board::stack_reservation_current_size;
pub use board::MACHINE_BOARD;
pub use board::MACHINE_TYPE;
pub use board::MACHINE_PROCESSOR;
pub use board::HEAP_SIZE_BYTES;
pub use board::MAX_STATIC_MEMORY;
pub use board::VREF_SAMPLE;
#[cfg(feature = "with-sdcard")]
pub use board::SDCARD_PARTITION;
#[cfg(feature = "with-usbserial")]
const USBSERIAL_BUFFER_SIZE: usize = 32;
#[cfg(feature = "with-uart-port-1")]
const UART_PORT1_BUFFER_SIZE: usize = 32;

static EXECUTOR_HIGH: printhor_hwa_common::TrackedStaticCell<embassy_executor::Executor> = printhor_hwa_common::TrackedStaticCell::new();

struct TokenHolder<S> {
    token: embassy_executor::SpawnToken<S>
}

unsafe impl<S> Sync for TokenHolder<S> {}
unsafe impl<S> Send for TokenHolder<S> {}

#[inline]
pub fn launch_high_priotity<S: 'static>(token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
    use thread_priority::*;

    let r = Box::new(TokenHolder {token});
    let builder = std::thread::Builder::new()
        .name("isr-high-prio".to_owned());
    let _ = builder.spawn_with_priority(ThreadPriority::Max,|_| {
        let executor: &'static mut embassy_executor::Executor = EXECUTOR_HIGH.init("StepperExecutor", embassy_executor::Executor::new());
        executor.run(move |spawner| {
            spawner.spawn(
                r.token
            ).unwrap();
        });
    });
    Ok(())
}

#[inline]
pub fn init_logger() {
    env_logger::init();
}

#[inline]
pub fn sys_reset() {
    std::process::exit(0);
}



