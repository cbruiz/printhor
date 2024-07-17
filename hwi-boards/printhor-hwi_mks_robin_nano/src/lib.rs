#![no_std]
#![allow(stable_features)]
#![cfg_attr(feature="nightly", feature(type_alias_impl_trait))]
pub use defmt::{trace,debug,info,warn, error};
pub use defmt;

mod board_stm32f4;

pub use board_stm32f4::device;
pub use board_stm32f4::SysDevices;
pub use board_stm32f4::IODevices;
pub use board_stm32f4::Controllers;
pub use board_stm32f4::MotionDevices;
pub use board_stm32f4::PwmDevices;

pub use board_stm32f4::init;
pub use board_stm32f4::setup;
pub use board_stm32f4::heap_current_size;
pub use board_stm32f4::stack_reservation_current_size;
pub use board_stm32f4::MACHINE_BOARD;
pub use board_stm32f4::MACHINE_TYPE;
pub use board_stm32f4::MACHINE_PROCESSOR;
pub use board_stm32f4::HEAP_SIZE_BYTES;
pub use board_stm32f4::MAX_STATIC_MEMORY;
pub use board_stm32f4::VREF_SAMPLE;
#[cfg(feature = "with-sdcard")]
pub use board_stm32f4::SDCARD_PARTITION;
#[cfg(feature = "with-serial-usb")]
const USBSERIAL_BUFFER_SIZE: usize = 32;
#[cfg(feature = "with-serial-port-1")]
const UART_PORT1_BUFFER_SIZE: usize = 32;
#[cfg(feature = "with-serial-port-1")]
const UART_PORT1_BAUD_RATE: u32 = 115200;
cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
        pub const SEGMENT_QUEUE_SIZE: u8 = 4;
    }
}
pub use board::ADC_START_TIME_US;
pub use board::ADC_VREF_DEFAULT_MV;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        cfg_if::cfg_if! {
            if #[cfg(feature = "executor-interrupt")] {
                use embassy_stm32::interrupt;
                use embassy_executor::InterruptExecutor;

                pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
                #[interrupt]
                unsafe fn RTC_ALARM() {
                    EXECUTOR_HIGH.on_interrupt()
                }
                #[inline]
                pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
                    let spawner = EXECUTOR_HIGH.start(interrupt::RTC_ALARM);
                    spawner.spawn(token).map_err(|_| ())
                }
            }
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
