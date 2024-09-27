#![allow(stable_features)]
pub use printhor_hwa_common as hwa;
mod board;

pub use log::{trace, debug, info, warn, error};

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
pub use board::VREF_SAMPLE;
pub use board::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
pub use board::STEPPER_PLANNER_CLOCK_FREQUENCY;
#[cfg(feature = "with-sdcard")]
pub use board::SDCARD_PARTITION;
#[cfg(feature = "with-serial-port-1")]
const UART_PORT1_BUFFER_SIZE: usize = 32;
cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
        pub const SEGMENT_QUEUE_SIZE: u8 = 10;
    }
}
pub use board::ADC_START_TIME_US;
pub use board::ADC_VREF_DEFAULT_MV;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex};
use embassy_time::Duration;
cfg_if::cfg_if!{
    if #[cfg(feature="without-vref-int")] {
        pub use board::ADC_VREF_DEFAULT_SAMPLE;
    }
}

#[inline]
pub fn is_log_debug_enabled() -> bool {
    log::log_enabled!(log::Level::Debug)
}

#[inline]
pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, _spawner: Spawner, token: embassy_executor::SpawnToken<S>) -> Result<(),()>
{
    cfg_if::cfg_if! {
        if #[cfg(feature="executor-interrupt")] {
            static EXECUTOR_HIGH: printhor_hwa_common::TrackedStaticCell<embassy_executor::Executor> = printhor_hwa_common::TrackedStaticCell::new();

            let executor: &'static mut embassy_executor::Executor = EXECUTOR_HIGH.init("StepperExecutor", embassy_executor::Executor::new());
            executor.run(|s| {
                let x = s.make_send();
                thread_priority::ThreadPriority::Max.set_for_current().unwrap();
                x.spawn(task_stepper_isr()).unwrap();
            });
        }
        else {
            Ok(_spawner.spawn(token).map_err(|_| ())?)
        }
    }
}

#[inline]
pub fn init_logger() {
    use std::io::Write;
    env_logger::builder()
        .format(|buf, record| {
            writeln!(buf, "{}: {}", record.level(), record.args())
        })
        .init();
}



#[inline]
// Execute closure f in an interrupt-free context.
// In native this is not required, so does nothing
pub fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce() -> R,
{
    f()
}

extern "Rust" {fn do_tick();}

static TICKER_SIGNAL: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();

static TERMINATION: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();

#[embassy_executor::task]
pub async fn task_stepper_ticker()
{
    info!("[task_stepper_ticker] starting");
    let mut t = embassy_time::Ticker::every(Duration::from_micros((1_000_000 / board::STEPPER_PLANNER_CLOCK_FREQUENCY) as u64));
    loop {
        if embassy_time::with_timeout(Duration::from_secs(5), TICKER_SIGNAL.wait()).await.is_err() {
            if TERMINATION.signaled() {
                info!("[task_stepper_ticker] Ending gracefully");
                return ();
            }
            continue;
        }
        unsafe {
            do_tick();
        }
        t.next().await;
    }
}

pub fn sys_reset() {
    std::process::exit(0);
}

pub fn sys_stop() {
    info!("Sending terminate signal");
    TERMINATION.signal(true);
}

pub fn pause_ticker() {
    info!("Ticker Paused");
    TICKER_SIGNAL.reset();
}

pub fn resume_ticker() {
    debug!("Ticker Resumed");
    TICKER_SIGNAL.signal(true);
}
