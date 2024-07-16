#![no_std]

pub use defmt::{trace, debug, info, warn, error};
pub use defmt;

cfg_if::cfg_if! {
    if #[cfg(feature="nucleo64-l476rg")] {
        mod board_stm32l4;
        pub mod board {
            pub use board_stm32l4::device;
            pub use board_stm32l4::SysDevices;
            pub use board_stm32l4::IODevices;
            pub use board_stm32l4::Controllers;
            pub use board_stm32l4::MotionDevices;
            pub use board_stm32l4::PwmDevices;
            pub use board_stm32l4::init;
            pub use board_stm32l4::setup;
            pub use board_stm32l4::heap_current_size;
            pub use board_stm32l4::stack_reservation_current_size;
            pub use board_stm32l4::MACHINE_BOARD;
            pub use board_stm32l4::MACHINE_TYPE;
            pub use board_stm32l4::MACHINE_PROCESSOR;
            pub use board_stm32l4::PROCESSOR_SYS_CK_MHZ;
            pub use board_stm32l4::HEAP_SIZE_BYTES;
            pub use board_stm32l4::MAX_STATIC_MEMORY;
            pub use board_stm32l4::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
            pub use board_stm32l4::STEPPER_PLANNER_CLOCK_FREQUENCY;
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
    }
    else if #[cfg(feature="nucleo64-f410rb")] {
        mod board_stm32f4;
        pub mod board {
            pub use crate::board_stm32f4::device;
            pub use crate::board_stm32f4::io;
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
        }
        pub use board::*;
        pub use crate::board_stm32f4::ADC_START_TIME_US;
        pub use crate::board_stm32f4::ADC_VREF_DEFAULT_MV;
        #[cfg(feature = "with-sdcard")]
        pub use board_stm32f4::SDCARD_PARTITION;
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
    else {
        compile_error!("Board not set");
    }
}


cfg_if::cfg_if! {
    if #[cfg(feature="executor-interrupt")] {

        use embassy_stm32::interrupt;
        use embassy_executor::InterruptExecutor;

        pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

        #[interrupt]
        unsafe fn RNG() {
            EXECUTOR_HIGH.on_interrupt()
        }

        #[inline]
        pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {

            use embassy_stm32::interrupt::InterruptExt;
            #[cfg(feature = "with-serial-port-1")]
            interrupt::USART2.set_priority(embassy_stm32::interrupt::Priority::P3);
            interrupt::RNG.set_priority(embassy_stm32::interrupt::Priority::P2);
            let spawner = EXECUTOR_HIGH.start(interrupt::RNG);
            spawner.spawn(token).map_err(|_| ())
        }
    }
}

// Execute closure f in an interrupt-free context.
// Required to safety lock a resource that can be also requested by an ISR.
// Such ISR, hence, won't miss it's interrupt and won't be too much delayed
pub fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce() -> R,
{
    cortex_m::interrupt::free(|_| {
        f()
    })
}

pub fn setup_timer() {
    unsafe {
        let p = cortex_m::Peripherals::steal();
        let mut syst = p.SYST;
        syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        // Target: 0.000010 seg (10us)
        let reload: u32 = ((crate::board::PROCESSOR_SYS_CK_MHZ / crate::board::STEPPER_PLANNER_CLOCK_FREQUENCY) - 1).max(1);
        defmt::info!("SYST reload set to {}", reload);
        syst.set_reload(reload);
        syst.enable_counter();
        syst.enable_interrupt();
    }
}

extern "Rust" {fn do_tick();}

use cortex_m_rt::exception;

#[exception]
fn SysTick() {
    unsafe {
        do_tick();
    }
}

#[inline]
pub fn init_logger() {
}

#[inline]
pub fn sys_reset() {
    cortex_m::peripheral::SCB::sys_reset();
}
