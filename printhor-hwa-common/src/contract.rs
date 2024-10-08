use crate as hwa;
#[allow(unused)]
use hwa::traits;
#[allow(unused)]
use hwa::RawHwiResource;

use core::future;

///! This module contains the interface contract of HWI boards

/// All boards must export a struct Called `Contract` at crate level implementing this trait
pub trait HwiContract: Sized {

    const MACHINE_TYPE: &'static str;
    const MACHINE_BOARD: &'static str;
    const MACHINE_PROCESSOR: &'static str;

    //#region "Hard-coded settings"

    /// Defines a timeout value for the watchdog timer in micro-seconds.
    /// This value is crucial for ensuring the system can recover from
    /// unexpected states by triggering a system reset or another defined
    /// recovery action if the system becomes unresponsive. The timeout value
    /// should be carefully chosen based on:
    ///
    /// 1. **System Responsiveness Needs**: A shorter timeout is useful for
    ///    highly responsive systems, ensuring quick recovery.
    /// 2. **Processing Time**: Consider the maximum time a valid operation
    ///    might need to complete. Timeout should be long enough to avoid
    ///    unnecessary resets during normal operation.
    /// 3. **Resource Constraints**: A longer timeout might be needed for
    ///    systems with limited processing power or more complex tasks.
    ///
    /// Modify this value as per the requirements of your specific application.
    const WATCHDOG_TIMEOUT_US: u32;

    /// The size of the heap in bytes.
    /// By convention, 0 means no heap
    const HEAP_SIZE_BYTES: usize;

    /// The maximum expected allocation by hwa and hwi statics
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize;

    fn heap_current_size() -> usize {
        0
    }
    fn heap_max_size() -> usize {
        Self::HEAP_SIZE_BYTES
    }
    fn stack_reservation_current_size() -> usize {
        printhor_hwa_utils::stack_allocation_get()
    }

    fn stack_reservation_max_size() -> usize {
        Self::MAX_EXPECTED_STATIC_ALLOC_BYTES
    }

    fn stack_reservation_usage_percentage() -> f32 {
        let alloc = printhor_hwa_utils::stack_allocation_get() as f32;
        let max = Self::stack_reservation_max_size() as f32;
        (100.0f32 * alloc) / max
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            /// The frequency at which a segment is sampled for the motion planner.
            /// For instance:
            ///
            /// Let's suppose segment `A` traveling from (0,0,0) to (1,0,0) at average velocity of 1mm/s
            ///
            /// It will take approximately 1 second
            ///
            /// With this parameter set to 100 (100Hz), the motion planner algorithm will compute finer
            /// micro-segments taking samples. In previous example, 100 micro segments will be planned
            /// for the whole trajectory of `A`
            #[cfg(feature="with-motion")]
            const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32;

            /// The frequency in Hz at which step planner will sample a micro-segment
            #[cfg(feature="with-motion")]
            const STEP_PLANNER_CLOCK_FREQUENCY: u32;

            /// The queue size of Motion Planner RingBuffer
            #[cfg(feature="with-motion")]
            const SEGMENT_QUEUE_SIZE: usize;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {
            /// The `Beta` value of the thermistor for **HotEnd** (see thermistor specs).
            ///
            /// Typically, 3950.0 for the classic `NTC **3950** 100K`
            const HOT_END_THERM_BETA: f32;

            /// The nominal thermistor value for **HotEnd**. (see thermistor specs).
            ///
            /// Represents the resistance value in Ohms at 25ºC
            ///
            /// Typically, 100000.0 for the classic `NTC 3950 **100K**`
            const HOT_END_THERM_NOMINAL_RESISTANCE: f32;

            /// The circuit impedance of the sensor electronics for **HotEnd**, depending on the board.
            /// Normally, thermistor is not connected directly to `AGND` and `ADC Pin`,
            /// so considering this variation is very important for accuracy.
            /// This value can be calculated based in the board schematics or physically measured.
            #[doc = include_str!("../../doc/thermal_sensors.md")]
            const HOT_END_THERM_PULL_UP_RESISTANCE: f32;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-bed")] {
            /// The `Beta` value of the thermistor for **HotBed** (see thermistor specs).
            ///
            /// Typically, 3950.0 for the classic `NTC **3950** 100K`
            const HOT_BED_THERM_BETA: f32;

            /// The nominal thermistor value for **HotBed**. (see thermistor specs).
            ///
            /// Represents the resistance value in Ohms at 25ºC
            ///
            /// Typically, 100000.0 for the classic `NTC 3950 **100K**`
            const HOT_BED_THERM_NOMINAL_RESISTANCE: f32;

            /// The circuit impedance of the sensor electronics for **HotBed**. Depending on the board.
            /// Normally, thermistor is not connected directly to `AGND` and `ADC Pin`,
            /// so considering this variation is very important for accuracy.
            /// This value can be calculated based in the board schematics or physically measured.
            #[doc = include_str!("../../doc/thermal_sensors.md")]
            const HOT_BED_THERM_PULL_UP_RESISTANCE: f32;
        }
    }

    //#endregion

    //#region "Custom MutexTypes"

    type EventBusPubSubMutexType: hwa::RawMutex;

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            type DeferChannelMutexType: hwa::RawMutex;
        }
    }

    //#endregion

    //#region "Locking strategies"

    type WatchDogMutexStrategy: hwa::MutexStrategy;

    type EventBusMutexStrategy: hwa::MutexStrategy;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-usb")] {
            type SerialUsbTx: hwa::MutexStrategy;
            type SerialUsbRx: hwa::RawHwiResource;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-1")] {

            const SERIAL_PORT1_BAUD_RATE: u32;
            const SERIAL_PORT1_RX_BUFFER_SIZE: usize;

            type SerialPort1Tx: hwa::MutexStrategy;
            type SerialPort1Rx: RawHwiResource + traits::ByteStream;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-2")] {
            type SerialPort2Tx: hwa::MutexStrategy;
            type SerialPort2Rx: hwa::RawHwiResource;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-ps-on")] {
            type PSOnMutexStrategy: hwa::MutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            type ProbePowerPwm: hwa::MutexStrategy;
            type ProbePowerPwmChannel: hwa::RawHwiResource;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-laser")] {
            type LaserPowerPwm: hwa::MutexStrategy;
            type LaserPowerPwmChannel: hwa::RawHwiResource;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-layer")] {
            type FanLayerPowerPwm: hwa::MutexStrategy;
            type FanLayerPowerPwmChannel: hwa::RawHwiResource;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-extra-1")] {
            type FanExtra1PowerPwm: hwa::MutexStrategy;
            type FanExtra1PowerPwmChannel: hwa::RawHwiResource;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-end")] {
            type HotEndAdc: hwa::MutexStrategy;
            type HotEndAdcPin: hwa::RawHwiResource;
            type HotEndPowerPwm: hwa::MutexStrategy;
            type HotEndPwmChannel: hwa::RawHwiResource;
        }
    }

    //#endregion

    /// Initialize the logger (optional)
    fn init_logger() {
    }

    /// Initialize the heap allocator (if needed)
    fn init_heap() {
    }

    // The HWI initialization
    fn init(spawner: embassy_executor::Spawner) -> impl future::Future<Output = HwiContext<Self>>;

    /// Resets the MCU/CPU
    fn sys_reset();

    /// Stop the MCU/CPU
    fn sys_stop();

    /// Execute closure f in an interrupt-free context.
    fn interrupt_free<F, R>(f: F) -> R where F: FnOnce() -> R;

    fn pause_ticker();

    fn resume_ticker();

}

pub struct HwiContext<C>
where C: HwiContract + Sized
{
    pub sys_watch_dog: crate::StaticController<C::WatchDogMutexStrategy>,

    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: crate::StaticController<C::SerialUsbTx>,
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_rx_stream: C::SerialUsbRx,

    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: crate::StaticController<C::SerialPort1Tx>,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: C::SerialPort1Rx,

    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: crate::StaticController<C::SerialPort2Tx>,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_rx_stream: C::SerialPort2Rx,

    #[cfg(feature = "with-ps-on")]
    pub ps_on: crate::StaticController<C::PSOnMutexStrategy>,

    #[cfg(feature = "with-probe")]
    pub probe_power_pwm: crate::StaticController<C::ProbePowerPwm>,

    #[cfg(feature = "with-probe")]
    pub probe_power_channel: C::ProbePowerPwmChannel,

    #[cfg(feature = "with-laser")]
    pub laser_power_pwm: crate::StaticController<C::LaserPowerPwm>,

    #[cfg(feature = "with-laser")]
    pub laser_power_channel: C::LaserPowerPwmChannel,

    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_power_pwm: crate::StaticController<C::FanLayerPowerPwm>,

    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_power_channel: C::FanLayerPowerPwmChannel,

    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra1_power_pwm: crate::StaticController<C::FanExtra1PowerPwm>,

    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra1_power_channel: C::FanExtra1PowerPwmChannel,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_adc: crate::StaticController<C::HotEndAdc>,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_adc_pin: C::HotEndAdcPin,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_power_pwm: crate::StaticController<C::HotEndPowerPwm>,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_power_channel: C::HotEndPwmChannel,

}


