#![allow(stable_features)]
#[allow(unused)]
use printhor_hwa_common as hwa;
mod board;

//#region "Mutex types for this board"

pub type EventbusMutexType = hwa::SyncSendMutex;
pub type EventBusChannelMutexType = hwa::NoopMutex;
pub type DeferChannelMutexType = hwa::NoopMutex;
pub type WatchdogMutexType = hwa::NoopMutex;
pub type MotionDriverMutexType = hwa::SyncSendMutex;
pub type MotionPlannerMutexType = hwa::NoopMutex;
pub type MotionConfigMutexType = hwa::SyncSendMutex;
pub type MotionStatusMutexType = hwa::NoopMutex;
pub type MotionRingBufferMutexType = hwa::NoopMutex;
pub type MotionSignalMutexType = hwa::NoopMutex;
pub type Pwm1MutexType = hwa::SyncSendMutex;
pub type Pwm2MutexType = hwa::SyncSendMutex;
pub type ProbeMutexType = hwa::SyncSendMutex;
pub type PwmHotEndMutexType = hwa::SyncSendMutex;
pub type AdcHotEndMutexType = hwa::NoopMutex;
pub type PwmHotBedMutexType = hwa::SyncSendMutex;
pub type AdcHotBedMutexType = hwa::NoopMutex;
pub type PwmFanLayerMutexType = hwa::SyncSendMutex;
pub type PwmFanExtra1MutexType = hwa::SyncSendMutex;
pub type PwmLaserMutexType = hwa::SyncSendMutex;
pub type PSOnMutexType = hwa::NoopMutex;

pub type ServoControllerMutexType = hwa::SyncSendMutex;
pub type FanLayerControllerMutexType = hwa::SyncSendMutex;
pub type FanExtra1ControllerMutexType = hwa::SyncSendMutex;
pub type LaserControllerMutexType = hwa::SyncSendMutex;
pub type HotEndControllerMutexType = hwa::SyncSendMutex;
pub type HotBedControllerMutexType = hwa::SyncSendMutex;


pub type SerialPort1MutexType = hwa::SyncSendMutex;
pub type SerialPort2MutexType = hwa::SyncSendMutex;
pub type SerialUsbMutexType = hwa::NoopMutex;

//#endregion

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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "with-trinamic")]
use crate::board::mocked_peripherals::TRINAMIC_SIMULATOR_PARK_SIGNAL;

cfg_if::cfg_if!{
    if #[cfg(feature="without-vref-int")] {
        pub use board::ADC_VREF_DEFAULT_SAMPLE;
    }
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

pub fn init_logger() {
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-log")] {
            use std::io::Write;
            env_logger::builder()
                .format(|buf, record| {
                    writeln!(buf, "{}: {}", record.level(), record.args())
                })
                .init();
        }
    }

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

#[cfg(feature = "with-motion")]
extern "Rust" {fn do_tick();}

static TICKER_SIGNAL: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();

static TERMINATION: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();

#[cfg(feature = "with-motion")]
#[embassy_executor::task]
pub async fn task_stepper_ticker()
{
    hwa::info!("[task_stepper_ticker] starting");
    let mut t = embassy_time::Ticker::every(embassy_time::Duration::from_micros((1_000_000 / board::STEPPER_PLANNER_CLOCK_FREQUENCY) as u64));
    loop {
        if embassy_time::with_timeout(embassy_time::Duration::from_secs(5), TICKER_SIGNAL.wait()).await.is_err() {
            if TERMINATION.signaled() {
                hwa::info!("[task_stepper_ticker] Ending gracefully");
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
    hwa::info!("Sending terminate signal");
    TERMINATION.signal(true);
}

pub fn pause_ticker() {
    hwa::info!("Ticker Paused");
    TICKER_SIGNAL.reset();
}

pub fn resume_ticker() {
    hwa::debug!("Ticker Resumed");
    TICKER_SIGNAL.signal(true);
}

#[cfg(feature = "with-trinamic")]
pub fn pause_trinamic() {
    TRINAMIC_SIMULATOR_PARK_SIGNAL.reset();
}

#[cfg(feature = "with-trinamic")]
pub fn resume_trinamic(channel: device::AxisChannel) {
    TRINAMIC_SIMULATOR_PARK_SIGNAL.signal(channel);
}
