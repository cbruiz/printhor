#![no_std]
#![allow(stable_features)]
#![cfg_attr(feature="nightly", feature(type_alias_impl_trait))]

pub use defmt::{trace,debug,info,warn, error};
pub use defmt;

mod board;

pub use board::device;
pub use board::SysDevices;
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
#[cfg(feature = "with-sdcard")]
pub use board::SDCARD_PARTITION;
#[cfg(feature = "with-serial-usb")]
const USBSERIAL_BUFFER_SIZE: usize = 32;
#[cfg(feature = "with-serial-port-1")]
const UART_PORT1_BUFFER_SIZE: usize = 32;
#[cfg(feature = "with-serial-port-1")]
const UART_PORT1_BAUD_RATE: u32 = 115200;
#[cfg(feature = "with-serial-port-2")]
const UART_PORT2_BUFFER_SIZE: usize = 1024;
#[cfg(feature = "with-serial-port-2")]
const UART_PORT2_BAUD_RATE: u32 = 115200;
cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
        pub const SEGMENT_QUEUE_SIZE: u8 = 20;
    }
}

pub use board::ADC_START_TIME_US;
pub use board::ADC_VREF_DEFAULT_MV;
cfg_if::cfg_if! {
    if #[cfg(feature="without-vref-int")] {
        pub use board::ADC_VREF_DEFAULT_SAMPLE;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="threaded")] {

        use embassy_stm32::interrupt;
        use embassy_stm32::interrupt::Priority;
        use embassy_stm32::interrupt::InterruptExt;
        use embassy_executor::{InterruptExecutor};

        pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

        cfg_if::cfg_if! {
            if #[cfg(feature="skr_mini_e3_v2")] {
                #[interrupt]
                unsafe fn RTC_ALARM() {
                    EXECUTOR_HIGH.on_interrupt()
                }

                #[inline]
                pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, spawner: embassy_executor::Spawner, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
                    interrupt::RTC_ALARM.set_priority(Priority::P2);

                    let spawner = EXECUTOR_HIGH.start(interrupt::RTC_ALARM);
                    spawner.spawn(token).map_err(|_| ())
                }
            }
            else if #[cfg(feature="skr_mini_e3_v3")] {
                #[interrupt]
                unsafe fn CEC() {
                    EXECUTOR_HIGH.on_interrupt()
                }

                #[inline]
                pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {

                    #[cfg(feature = "with-usbserial")]
                    interrupt::USB_UCPD1_2.set_priority(Priority::P3);
                    interrupt::CEC.set_priority(Priority::P2);

                    let spawner = EXECUTOR_HIGH.start(interrupt::CEC);
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
