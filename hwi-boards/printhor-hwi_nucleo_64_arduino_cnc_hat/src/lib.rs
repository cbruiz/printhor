#![no_std]
#![allow(stable_features)]
#[allow(unused)]
use printhor_hwa_common as hwa;
extern crate alloc;

cfg_if::cfg_if! {
    if #[cfg(feature="nucleo64-l476rg")] {
        mod board_stm32l4;
        pub mod board {
            pub use crate::board_stm32l4::SysDevices;
            pub use crate::board_stm32l4::IODevices;
            pub use crate::board_stm32l4::Controllers;
            pub use crate::board_stm32l4::MotionDevices;
            pub use crate::board_stm32l4::PwmDevices;
            pub use crate::board_stm32l4::init;
            pub use crate::board_stm32l4::setup;
            pub use crate::board_stm32l4::heap_current_size;
            pub use crate::board_stm32l4::stack_reservation_current_size;
            pub use crate::board_stm32l4::MACHINE_BOARD;
            pub use crate::board_stm32l4::MACHINE_TYPE;
            pub use crate::board_stm32l4::MACHINE_PROCESSOR;
            pub use crate::board_stm32l4::PROCESSOR_SYS_CK_MHZ;
            pub use crate::board_stm32l4::HEAP_SIZE_BYTES;
            pub use crate::board_stm32l4::MAX_STATIC_MEMORY;
            pub use crate::board_stm32l4::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
            pub use crate::board_stm32l4::STEPPER_PLANNER_CLOCK_FREQUENCY;
            pub use crate::board_stm32l4::io;
            pub use crate::board_stm32l4::device;
        }
        pub use board::*;
        pub use board_stm32l4::ADC_START_TIME_US;
        pub use board_stm32l4::ADC_VREF_DEFAULT_MV;
        #[cfg(feature = "with-sdcard")]
        pub use board_stm32l4::SDCARD_PARTITION;
        #[cfg(feature = "with-serial-usb")]
        const USBSERIAL_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BAUD_RATE: u32 = 115200;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
                pub const SEGMENT_QUEUE_SIZE: u8 = 4;
            }
        }
    }
    else if #[cfg(feature="nucleo64-f410rb")] {

        //#region "Mutex types for this board"

        pub type EventbusMutexType = hwa::NoopMutex;
        pub type EventBusChannelMutexType = hwa::NoopMutex;
        pub type DeferChannelMutexType = hwa::NoopMutex;
        pub type WatchdogMutexType = hwa::NoopMutex;
        pub type MotionDriverMutexType = hwa::SyncSendMutex;
        pub type MotionPlannerMutexType = hwa::NoopMutex;
        pub type MotionConfigMutexType = hwa::NoopMutex;
        pub type MotionStatusMutexType = hwa::NoopMutex;
        pub type MotionRingBufferMutexType = hwa::NoopMutex;
        pub type MotionSignalMutexType = hwa::NoopMutex;

        pub type SerialPort1MutexType = hwa::NoopMutex;

        //#endregion

        mod board_stm32f4;
        pub mod board {
            pub use crate::board_stm32f4::SysDevices;
            pub use crate::board_stm32f4::IODevices;
            pub use crate::board_stm32f4::Controllers;
            pub use crate::board_stm32f4::MotionDevices;
            pub use crate::board_stm32f4::PwmDevices;
            pub use crate::board_stm32f4::init;
            pub use crate::board_stm32f4::setup;
            pub use crate::board_stm32f4::heap_current_size;
            pub use crate::board_stm32f4::stack_reservation_current_size;
            pub use crate::board_stm32f4::MACHINE_BOARD;
            pub use crate::board_stm32f4::MACHINE_TYPE;
            pub use crate::board_stm32f4::MACHINE_PROCESSOR;
            pub use crate::board_stm32f4::PROCESSOR_SYS_CK_MHZ;
            pub use crate::board_stm32f4::HEAP_SIZE_BYTES;
            pub use crate::board_stm32f4::MAX_STATIC_MEMORY;
            pub use crate::board_stm32f4::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
            pub use crate::board_stm32f4::STEPPER_PLANNER_CLOCK_FREQUENCY;
            pub use crate::board_stm32f4::io;
            pub use crate::board_stm32f4::device;
        }
        pub use board::*;
        pub use crate::board_stm32f4::ADC_START_TIME_US;
        pub use crate::board_stm32f4::ADC_VREF_DEFAULT_MV;
        #[cfg(feature = "with-sdcard")]
        pub use board_stm32f4::SDCARD_PARTITION;
        #[cfg(feature = "with-serial-usb")]
        const USBSERIAL_BUFFER_SIZE: usize = 64;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BUFFER_SIZE: usize = 128;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BAUD_RATE: u32 = 115200;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
                pub const SEGMENT_QUEUE_SIZE: u8 = 4;
            }
        }

    }
    else {
        compile_error!("Board not set");
    }
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

pub fn setup_timer() {
    unsafe {
        let p = cortex_m::Peripherals::steal();
        let mut syst = p.SYST;
        syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        // Target: 0.000010 seg (10us)
        let reload: u32 = ((PROCESSOR_SYS_CK_MHZ / STEPPER_PLANNER_CLOCK_FREQUENCY) - 1).max(1);
        hwa::info!(
            "SYST reload set to {} ({} Hz)",
            reload,
            STEPPER_PLANNER_CLOCK_FREQUENCY
        );
        syst.set_reload(reload);
    }
}

#[cfg(feature = "with-motion")]
extern "Rust" {
    fn do_tick();
}

use cortex_m_rt::exception;
#[exception]
fn SysTick() {
    #[cfg(feature = "with-motion")]
    unsafe {
        do_tick();
    }
}

pub fn sys_reset() {
    cortex_m::peripheral::SCB::sys_reset();
}

pub fn sys_stop() {
    // Not needed
}

pub fn pause_ticker() {
    unsafe {
        let p = cortex_m::Peripherals::steal();
        let mut syst = p.SYST;
        syst.disable_counter();
        syst.disable_interrupt();
    }
    hwa::debug!("Ticker Paused");
}

pub fn resume_ticker() {
    hwa::debug!("Ticker Resumed");
    unsafe {
        let p = cortex_m::Peripherals::steal();
        let mut syst = p.SYST;
        syst.enable_interrupt();
        syst.enable_counter();
    }
    hwa::debug!("Ticker Resumed");
}

#[inline]
pub fn init_logger() {}
