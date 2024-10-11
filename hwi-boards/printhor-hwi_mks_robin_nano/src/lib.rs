#![no_std]
#![allow(stable_features)]
pub use defmt;
pub use defmt::{debug, error, info, trace, warn};

mod board_stm32f4;
pub mod board {
    pub use crate::board_stm32f4::comm;
    pub use crate::board_stm32f4::device;
    pub use crate::board_stm32f4::heap_current_size;
    pub use crate::board_stm32f4::init;
    pub use crate::board_stm32f4::io;
    pub use crate::board_stm32f4::setup;
    pub use crate::board_stm32f4::stack_reservation_current_size;
    pub use crate::board_stm32f4::Controllers;
    pub use crate::board_stm32f4::IODevices;
    pub use crate::board_stm32f4::MotionDevices;
    pub use crate::board_stm32f4::PwmDevices;
    pub use crate::board_stm32f4::SysDevices;
    pub use crate::board_stm32f4::HEAP_SIZE_BYTES;
    pub use crate::board_stm32f4::MACHINE_BOARD;
    pub use crate::board_stm32f4::MACHINE_PROCESSOR;
    pub use crate::board_stm32f4::MACHINE_TYPE;
    pub use crate::board_stm32f4::MAX_STATIC_MEMORY;
    pub use crate::board_stm32f4::PROCESSOR_SYS_CK_MHZ;
    pub use crate::board_stm32f4::STEPPER_PLANNER_CLOCK_FREQUENCY;
    pub use crate::board_stm32f4::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
    pub use embassy_executor::Spawner;
}
pub use crate::board_stm32f4::ADC_START_TIME_US;
pub use crate::board_stm32f4::ADC_VREF_DEFAULT_MV;
pub use board::*;
#[cfg(feature = "with-sd-card")]
pub use board_stm32f4::SDCARD_PARTITION;
#[cfg(feature = "with-serial-usb")]
const USBSERIAL_BUFFER_SIZE: usize = 512;
#[cfg(feature = "with-serial-port-1")]
const UART_PORT1_BUFFER_SIZE: usize = 512;
#[cfg(feature = "with-serial-port-1")]
const UART_PORT1_BAUD_RATE: u32 = 115200;
#[cfg(feature = "with-serial-port-2")]
compile_error!("Not implemented");

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
        pub const SEGMENT_QUEUE_SIZE: u8 = 4;
    }
}

#[inline]
pub fn init_logger() {}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub fn setup_timer() {
            unsafe {
                let p = cortex_m::Peripherals::steal();
                let mut syst = p.SYST;
                syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
                let reload: u32 = (board::PROCESSOR_SYS_CK_MHZ / STEPPER_PLANNER_CLOCK_FREQUENCY).max(1) - 1;
                hwa::info!("SYST reload set to {} ({} Hz)", reload, STEPPER_PLANNER_CLOCK_FREQUENCY);
                syst.set_reload(reload);
                syst.enable_counter();
                syst.enable_interrupt();
            }
        }

        extern "Rust" {fn do_tick();}

        #[no_mangle]
        pub extern "Rust" fn pause_ticker() {
            unsafe {
                let p = cortex_m::Peripherals::steal();
                let mut syst = p.SYST;
                syst.disable_counter();
                syst.disable_interrupt();
            }
            defmt::info!("Ticker Paused");
        }
        #[no_mangle]
        pub extern "Rust" fn resume_ticker() {

            unsafe {
                let p = cortex_m::Peripherals::steal();
                let mut syst = p.SYST;
                syst.enable_interrupt();
                syst.enable_counter();
            }
            defmt::info!("Ticker Resumed");
        }

        use cortex_m_rt::exception;
        #[exception]
        fn SysTick() {
            unsafe {
                do_tick();
            }
        }
    }
}

#[inline]
pub fn sys_reset() {
    cortex_m::peripheral::SCB::sys_reset();
}

// Execute closure f in an interrupt-free context.
// Required to safety lock a resource that can be also requested by an ISR.
// Such ISR, hence, won't miss its interrupt and won't be too much delayed
pub fn interrupt_free<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    cortex_m::interrupt::free(|_| f())
}
