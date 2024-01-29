#![no_std]
#![cfg_attr(feature="nightly", feature(type_alias_impl_trait))]
#![allow(stable_features)]
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::Priority;
use embassy_stm32::interrupt::InterruptExt;
use embassy_executor::InterruptExecutor;

pub use defmt::{trace,debug,info,warn, error};
pub use defmt;

mod board;

pub use board::device;
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
#[cfg(feature = "with-uart-port-1")]
const UART_PORT1_BAUD_RATE: u32 = 115200;

pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

cfg_if::cfg_if! {
    if #[cfg(feature="sk3_mini_e3_v2")] {
        #[interrupt]
        unsafe fn RTC() {
            EXECUTOR_HIGH.on_interrupt()
        }

        #[inline]
        pub fn launch_high_priotity<S: 'static + Send>(token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
            #[cfg(feature = "with-usbserial")]
            interrupt::USB_LP_CAN1_RX0.set_priority(Priority::P3);
            interrupt::RTC.set_priority(Priority::P2);

            let spawner = EXECUTOR_HIGH.start(interrupt::RTC);
            spawner.spawn(token).map_err(|_| ())
        }
    }
    else if #[cfg(feature="sk3_mini_e3_v3")] {
        #[interrupt]
        unsafe fn CEC() {
            EXECUTOR_HIGH.on_interrupt()
        }

        #[inline]
        pub fn launch_high_priotity<S: 'static + Send>(token: embassy_executor::SpawnToken<S>) -> Result<(),()> {

            #[cfg(feature = "with-usbserial")]
            interrupt::USB_UCPD1_2.set_priority(Priority::P3);
            interrupt::CEC.set_priority(Priority::P2);

            let spawner = EXECUTOR_HIGH.start(interrupt::CEC);
            spawner.spawn(token).map_err(|_| ())
        }
    }
}

#[inline]
pub fn init_logger() {
}

#[inline]
pub fn sys_reset() {
    cortex_m::peripheral::SCB::sys_reset();
}
