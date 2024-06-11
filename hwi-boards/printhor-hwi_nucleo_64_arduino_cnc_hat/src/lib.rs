#![no_std]

pub use defmt::{trace, debug, info, warn, error};
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
pub use board::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
pub use board::STEPPER_PLANNER_CLOCK_FREQUENCY;
#[cfg(feature = "with-sdcard")]
pub use board::SDCARD_PARTITION;
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
pub use board::ADC_START_TIME_US;
pub use board::ADC_VREF_DEFAULT_MV;


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
        let reload: u32 = ((board::PROCESSOR_SYS_CK_MHZ / STEPPER_PLANNER_CLOCK_FREQUENCY) - 1).max(1);
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

