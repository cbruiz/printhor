#![allow(stable_features)]
mod board;
pub use board::{device, Contract};

/*
//#region "Mutex types for this board"

type EventBusLockType = hwa::SyncSendMutex;
type EventBusPubSubMutexType = hwa::SyncSendMutex;
#[allow(unused)]
type DeferChannelMutexType = hwa::NoopRawMutexType;

type EventBusMutexStrategyType<L,D> = hwa::AsyncStandardStrategy<L,D>;

//pub type EventBusChannelMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::NoopRawMutexType, D>;

type WatchDogMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::NoopRawMutexType, D>;
pub type MotionDriverMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type MotionPlannerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::NoopRawMutexType, D>;
pub type MotionConfigMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type MotionStatusMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::NoopRawMutexType, D>;
pub type MotionRingBufferMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::NoopRawMutexType, D>;
pub type MotionSignalMutexType = hwa::NoopRawMutexType;

//pub type Pwm1MutexType = hwa::SyncSendMutex;
//pub type Pwm2MutexType = hwa::SyncSendMutex;
pub type Pwm1ControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type Pwm2ControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;

pub type Adc1ControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type AdcHotEndMutexStrategyType<D> = Adc1ControllerMutexStrategyType<D>;
pub type AdcHotBedMutexStrategyType<D> = Adc1ControllerMutexStrategyType<D>;

pub type Adc2ControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;

pub type ProbeServoControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;

pub type PwmProbeMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type PwmHotEndMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type PwmHotBedMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type PwmFanLayerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type PwmFanExtra1MutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type PwmLaserMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type PSOnMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::NoopRawMutexType, D>;

pub type PrinterControllerSignalMutexType = hwa::NoopRawMutexType;

pub type FanLayerControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type FanExtra1ControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type LaserControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type HotEndControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type HotBedControllerMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;

pub type Spi1MutexStrategyType<D> = hwa::Holdable<hwa::SyncSendMutex, D>;
pub type SDCardMutexStrategyType<D> = hwa::Holdable<hwa::SyncSendMutex, D>;
pub type SPIControllerMutexStrategyType<D> = hwa::Holdable<hwa::SyncSendMutex, D>;


pub type SerialPort1MutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type SerialPort2MutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;
pub type SerialUsbMutexStrategyType<D> = hwa::AsyncStandardStrategy<hwa::SyncSendMutex, D>;

//#endregion

pub use board::device;
pub use board::SysDevices;
pub use board::IODevices;
pub use board::Controllers;
pub use board::MotionDevices;
pub use board::PwmDevices;
pub use board::heap_current_size;
pub use board::stack_reservation_current_size;

pub use board::HEAP_SIZE_BYTES;
pub use board::VREF_SAMPLE;
#[cfg(feature = "with-sd-card")]
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
#[allow(unused)]
use crate::board::mocked_peripherals::MockedIOPin;
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


static TICKER_SIGNAL: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();

static TERMINATION: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();


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


pub struct Contract();
impl HwiContract for Contract {

    const MACHINE_TYPE: &'static str = "Simulator/debugger";
    const MACHINE_BOARD: &'static str = "PC";
    const MACHINE_PROCESSOR: &'static str = std::env::consts::ARCH;


    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;

    const HEAP_SIZE_BYTES: usize = 42;
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize = 42;

    cfg_if::cfg_if! {
        if #[cfg(feature="with-motion")] {
            #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
            const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 200;

            #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
            const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;

            #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
            const SEGMENT_QUEUE_SIZE: usize = 0;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {
            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_BETA: f32 = 3950.0;

            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_NOMINAL_RESISTANCE: f32 = 100000.0;

            #[const_env::from_env("HOT_END_THERM_PULL_UP_RESISTANCE")]
            const HOT_END_THERM_PULL_UP_RESISTANCE: f32 = 4685.0;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-bed")] {
            #[const_env::from_env("HOT_BED_THERM_BETA")]
            const HOT_BED_THERM_BETA: f32 = 3950.0;

            #[const_env::from_env("HOT_BED_THERM_NOMINAL_RESISTANCE")]
            const HOT_BED_THERM_NOMINAL_RESISTANCE: f32 = 100000.0;

            #[const_env::from_env("HOT_BED_THERM_PULL_UP_RESISTANCE")]
            const HOT_BED_THERM_PULL_UP_RESISTANCE: f32 = 4685.0;
        }
    }

    // To build eventbus
    type EventBusPubSubMutexType = crate::EventBusPubSubMutexType;

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            // To build defer channel
            type DeferChannelMutexType = crate::DeferChannelMutexType;
        }
    }

    type WatchDogMutexStrategy = crate::WatchDogMutexStrategyType<crate::device::WatchDog>;
    type EventBusMutexStrategy = crate::EventBusMutexStrategyType<crate::EventBusLockType, hwa::EventBusChannelController<EventBusPubSubMutexType>>;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-1")] {
            const SERIAL_PORT1_BAUD_RATE: u32 = 115200;
            const SERIAL_PORT1_RX_BUFFER_SIZE: usize = 64;
            type SerialPort1Tx = crate::SerialPort1MutexStrategyType<device::SerialPort1TxDevice>;
        }
    }


    #[cfg(feature = "with-serial-usb")]
    type SerialUsbRx = hwa::HwiResource<crate::device::SerialUsbInputStream>;

    #[cfg(feature = "with-serial-usb")]
    type SerialUsbTx = crate::SerialPort1MutexStrategyType<device::SerialUsbTxDevice>;

    #[cfg(feature = "with-serial-port-1")]
    type SerialPort1Rx = hwa::HwiResource<crate::device::SerialPort1InputStream>;

    #[cfg(feature = "with-serial-port-2")]
    type SerialPort2Tx = crate::SerialPort2MutexStrategyType<device::SerialPort2TxDevice>;

    #[cfg(feature = "with-serial-port-2")]
    type SerialPort2Rx = hwa::HwiResource<crate::device::SerialPort2InputStream>;
    #[cfg(feature = "with-ps-on")]
    type PSOnMutexStrategy = crate::PSOnMutexStrategyType<crate::device::PsOnPin>;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            type ProbePowerPwm = crate::PwmProbeMutexStrategyType<device::PwmProbe>;
            type ProbePowerPwmChannel = hwa::HwiResource<crate::device::PwmProbeChannel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-laser")] {
            type LaserPowerPwm = crate::PwmLaserMutexStrategyType<device::PwmLaser>;
            type LaserPowerPwmChannel = hwa::HwiResource<crate::device::PwmLaserChannel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-layer")] {
            type FanLayerPowerPwm = crate::PwmLaserMutexStrategyType<device::PwmFanLayer>;
            type FanLayerPowerPwmChannel = hwa::HwiResource<crate::device::PwmFanLayerChannel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-extra-1")] {
            type FanExtra1PowerPwm = crate::PwmFanLayerMutexStrategyType<device::PwmFanExtra1>;
            type FanExtra1PowerPwmChannel = hwa::HwiResource<crate::device::PwmFanExtra1Channel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-end")] {
            type HotEndAdc = crate::AdcHotEndMutexStrategyType<device::AdcHotEnd>;
            type HotEndAdcPin = hwa::HwiResource<crate::device::AdcHotEndPin>;
            type HotEndPowerPwm = crate::PwmHotEndMutexStrategyType<device::PwmHotEnd>;
            type HotEndPwmChannel = hwa::HwiResource<crate::device::PwmHotEndChannel>;
        }
    }



    async fn init(_spawner: embassy_executor::Spawner) ->  hwa::HwiContext<Self> {


    }
}

// As of not, the AdcTraits must be exported this way

cfg_if::cfg_if!{
    if #[cfg(feature = "with-hot-end")] {
        pub use device::AdcTrait as HotEndAdcTrait;
        pub use device::AdcPinTrait as HotEndAdcPinTrait;

    }
}
cfg_if::cfg_if!{
    if #[cfg(feature = "with-hot-bed")] {
        pub use device::AdcTrait as HotBedAdcTrait;
        pub use device::AdcPinTrait as HotBedAdcPinTrait;
    }
}

 */