///  Native board implementation. For debugging/simulation purposes
#[allow(unused)]
use printhor_hwa_common as hwa;

pub mod device;
cfg_if::cfg_if!{
    if #[cfg(feature = "with-trinamic")] {
        pub mod comm;
        cfg_if::cfg_if!{
            if #[cfg(not(feature = "with-motion"))] {
                compile_error!("with-trinamic requires with-motion");
            }
        }
    }
}


pub mod mocked_peripherals;


#[cfg(feature = "with-motion")]
use crate::task_stepper_ticker;

#[allow(unused)]
pub(crate) const PROCESSOR_SYS_CK_MHZ: u32 = 1_000_000_000;
pub const HEAP_SIZE_BYTES: usize = 1024;

pub const VREF_SAMPLE: u16 = 1210u16;
#[cfg(feature = "with-sd-card")]
pub const SDCARD_PARTITION: usize = 0;
// The bit-banging uart in native simulator is set to ultra low speed for obvious reasons
#[cfg(feature = "with-trinamic")]
pub const TRINAMIC_UART_BAUD_RATE: u32 = 100;


pub const ADC_START_TIME_US: u16 = 10;
pub const ADC_VREF_DEFAULT_MV: u16 = 1650;
#[allow(unused)]
pub const ADC_VREF_DEFAULT_SAMPLE: u16 = 2048;

cfg_if::cfg_if! {
    if #[cfg(feature="with-hot-end")] {
        #[const_env::from_env("HOT_END_THERM_BETA")]
        // The B value of the thermistor
        const HOT_END_THERM_BETA: f32 = 3950.0;

        #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
        // Nominal NTC thermistor value
        const HOT_END_THERM_NOMINAL_RESISTANCE: f32 = 100000.0;

        #[const_env::from_env("HOT_END_THERM_PULL_UP_RESISTANCE")]
        // Physically measured
        const HOT_END_THERM_PULL_UP_RESISTANCE: f32 = 4685.0;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-hot-bed")] {
        #[const_env::from_env("HOT_BED_THERM_BETA")]
        // The B value of the thermistor
        const HOT_BED_THERM_BETA: f32 = 3950.0;

        #[const_env::from_env("HOT_BED_THERM_NOMINAL_RESISTANCE")]
        // Nominal NTC thermistor value
        const HOT_BED_THERM_NOMINAL_RESISTANCE: f32 = 100000.0;

        #[const_env::from_env("HOT_BED_THERM_PULL_UP_RESISTANCE")]
        // Physically measured
        const HOT_BED_THERM_PULL_UP_RESISTANCE: f32 = 4685.0;
    }
}

/// Shared controllers
pub struct Controllers {
    pub sys_watchdog: hwa::StaticController<crate::WatchDogMutexStrategyType<device::WatchDog>>,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: hwa::StaticController<crate::SerialPort1MutexStrategyType<device::SerialPort1TxDevice>>,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: hwa::StaticController<crate::SerialPort2MutexStrategyType<device::SerialPort2TxDevice>>,
}

pub struct SysDevices {
    #[cfg(all(feature = "with-motion", feature="executor-interrupt"))]
    pub task_stepper_core: printhor_hwa_common::NoDevice,
    #[cfg(feature = "with-ps-on")]
    pub ps_on: hwa::StaticController<crate::PSOnMutexStrategyType<device::PsOnPin>>,
}

pub struct IODevices {
    /// Only single owner allowed
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: device::SerialPort1InputStream,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_rx_stream: device::SerialPort2InputStream,
    #[cfg(feature = "with-sd-card")]
    pub sd_card_device: device::SDCardBlockDevice,
}

pub struct PwmDevices {
    #[cfg(feature = "with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature = "with-hot-end")]
    pub hot_end: device::HotEndPeripherals,
    #[cfg(feature = "with-hot-bed")]
    pub hot_bed: device::HotBedPeripherals,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer: device::FanLayerPeripherals,
    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra_1: device::FanExtra1Peripherals,
    #[cfg(feature = "with-laser")]
    pub laser: device::LaserPeripherals,
}

pub struct MotionDevices {
    #[cfg(feature = "with-motion")]
    pub motion_devices: device::MotionDevice,
}

pub fn heap_current_size() -> u32 {
    0
}

#[inline]
pub fn stack_reservation_current_size() -> u32 {
    hwa::stack_allocation_get() as u32
}

