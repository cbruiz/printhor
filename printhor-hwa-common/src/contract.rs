use crate as hwa;
#[allow(unused)]
use hwa::traits;

use core::future;

///! This module contains the interface contract of HWI boards

/// All boards must export a struct Called `Contract` at crate level implementing this trait
pub trait HwiContract: Sized {
    //#region "Global common constants"

    const FIRMWARE_NAME: &'static str = "PrinThor";
    const FIRMWARE_VERSION: &'static str = env!("CARGO_PKG_VERSION");
    const FIRMWARE_URL: &'static str = "https://github.com/cbruiz/printhor";
    const MACHINE_UUID: &'static str = "00000000-0000-0000-0000-000000000000";

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-e-axis")] {
            const EXTRUDER_COUNT: &'static str = "1";
        }
        else {
            const EXTRUDER_COUNT: &'static str = "0";
        }
    }

    //#endregion

    //#region "Board specific constants"

    /// The machine descriptive type
    const MACHINE_TYPE: &'static str;
    /// The machine bord name/model
    const MACHINE_BOARD: &'static str;
    /// The machine processor model
    const MACHINE_PROCESSOR: &'static str;
    /// The frequency at which MCU runs. Needed for computing motion ISR
    ///
    /// In native (std), embassy-executor is hard-coded to 1GHz
    const PROCESSOR_SYS_CK_MHZ: u32;

    /// The target [hwa::CommChannel] for M117 (display)
    const DISPLAY_CHANNEL: hwa::CommChannel = hwa::CommChannel::Internal;

    /// The target [hwa::CommChannel] for M118 (host)
    const HOST_CHANNEL: hwa::CommChannel = hwa::CommChannel::Internal;

    //#endregion

    //#region "WatchDog settings"

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

    //#endregion

    //#region "Memory management/tracking settings"

    /// The size of the heap in bytes.
    /// By convention, 0 means no heap
    const MAX_HEAP_SIZE_BYTES: usize;

    /// The maximum expected allocation by hwa and hwi statics
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize;

    fn heap_current_size() -> usize {
        0
    }

    fn stack_reservation_current_size() -> usize {
        printhor_hwa_utils::stack_allocation_get()
    }

    //#endregion

    //#region "Feature [with-motion] settings"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {

            /// The world unit magnitude. by default: mm
            #[const_env::from_env("WORLD_UNIT_MAGNITUDE")]
            const WORLD_UNIT_MAGNITUDE: &'static str = "mm";

            /// The (work)space unit magnitude. by default: mm
            #[const_env::from_env("SPACE_UNIT_MAGNITUDE")]
            const SPACE_UNIT_MAGNITUDE: &'static str = "mm";

            /// The physical unit magnitude. by default: mm
            #[const_env::from_env("PHYSICAL_UNIT_MAGNITUDE")]
            const PHYSICAL_UNIT_MAGNITUDE: &'static str = "mm";

            /// World size to apply bounds restriction in World Units ([WORLD_UNIT_MAGNITUDE])
            ///
            /// See [DEFAULT_WORLD_CENTER_WU] for constraints clarification
            const DEFAULT_WORLD_SIZE_WU: hwa::math::TVector<hwa::math::Real>;

            /// World center in World Units ([WORLD_UNIT_MAGNITUDE])
            ///
            /// Makes world coordinates bounded to: [- [DEFAULT_WORLD_SIZE] /2, [DEFAULT_WORLD_SIZE] /2]
            const DEFAULT_WORLD_CENTER_WU: hwa::math::TVector<hwa::math::Real>;

            /// Homing offset in World Units ([WORLD_UNIT_MAGNITUDE])
            ///
            /// Represent the world point reached when Homing performs
            const DEFAULT_WORLD_HOMING_POINT_WU: hwa::math::TVector<hwa::math::Real> = const {
                hwa::math::TVector::new_with_coord(hwa::CoordSel::motion_relevant_axis(), hwa::make_optional_real!(0.0))
            };

            /// Transforms a World position to (Work)space position. By default, identity transform applied
            ///
            /// Parameters:
            ///
            /// - `world_pos`: The world position.
            fn project_to_space(&self, _world_pos: &hwa::math::TVector<hwa::math::Real>) -> Result<hwa::math::TVector<hwa::math::Real>, ()> {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-motion-cartessian-kinematics")] {
                        use hwa::kinematics::WorldToSpaceTransformer;
                        const TRANSFORMER: hwa::kinematics::cartessian::DefaultTransformer = hwa::kinematics::cartessian::DefaultTransformer;
                        TRANSFORMER.project_to_space(_world_pos)
                    }
                    else if #[cfg(feature = "with-motion-core-xy-kinematics")] {
                        use hwa::kinematics::WorldToSpaceTransformer;
                        const TRANSFORMER: hwa::kinematics::core_xy::DefaultTransformer = hwa::kinematics::core_xy::DefaultTransformer;
                        TRANSFORMER.project_to_space(_world_pos)
                    }
                    else {
                        unreachable!("You must provide the transformer");
                    }
                }

            }

            /// Transforms a (Work)space position to World position. By default, identity transform applied
            ///
            /// Parameters:
            ///
            /// - `space_pos`: The world position.
            fn project_to_world(&self, _space_pos: &hwa::math::TVector<hwa::math::Real>) -> Result<hwa::math::TVector<hwa::math::Real>, ()> {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-motion-cartessian-kinematics")] {
                        use hwa::kinematics::WorldToSpaceTransformer;
                        const TRANSFORMER: hwa::kinematics::cartessian::DefaultTransformer = hwa::kinematics::cartessian::DefaultTransformer;
                        TRANSFORMER.project_to_world(_space_pos)
                    }
                    else if #[cfg(feature = "with-motion-core-xy-kinematics")] {
                        use hwa::kinematics::WorldToSpaceTransformer;
                        const TRANSFORMER: hwa::kinematics::core_xy::DefaultTransformer = hwa::kinematics::core_xy::DefaultTransformer;
                        TRANSFORMER.project_to_world(_space_pos)
                    }
                    else {
                        unreachable!("You must provide the transformer");
                    }
                }
            }

            /// Apply calculated min speed boundaries
            const CLAMP_MIN_SPEED: bool = true;
            /// Apply calculated max feed rate boundaries
            const CLAMP_MAX_FEED_RATE: bool = true;

            /// Default max speed in Physical Units / second
            const DEFAULT_MAX_SPEED_PS: hwa::math::TVector<hwa::math::Real>;
            /// Default max speed in physical units by second
            const DEFAULT_MAX_ACCEL_PS: hwa::math::TVector<hwa::math::Real>;
            /// Default max speed in physical units by second
            const DEFAULT_MAX_JERK_PS: hwa::math::TVector<hwa::math::Real>;
            /// Default max speed in physical units by second
            const DEFAULT_TRAVEL_SPEED_PS: hwa::math::Real;

            /// Default units per workspace unit
            const DEFAULT_UNITS_PER_WU: hwa::math::TVector<hwa::math::Real>;

            /// Default micro-steps per axis
            const DEFAULT_MICRO_STEPS_PER_AXIS: hwa::math::TVector<u16>;

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
            const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32;

            /// The frequency in Hz at which step planner will sample a micro-segment
            const STEP_PLANNER_CLOCK_FREQUENCY: u32;

            /// The queue size of Motion Planner RingBuffer
            /// Should be not very high, so capped to u8
            const SEGMENT_QUEUE_SIZE: u8;

            /// The queue size of Step Planner RingBuffer
            /// Should be not very high, so capped to u8
            const U_SEGMENT_QUEUE_SIZE: u8 = 2;
        }
    }

    //#endregion

    //#region "with-hot-end feature"

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {

            const HOT_END_ADC_V_REF_DEFAULT_MV: u16;

            const HOT_END_ADC_V_REF_DEFAULT_SAMPLE: u16;

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

    //#endregion

    //#region "with-hot-bed feature"

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-bed")] {

            const HOT_BED_ADC_V_REF_DEFAULT_MV: u16;

            const HOT_BED_ADC_V_REF_DEFAULT_SAMPLE: u16;

            /// The `Beta` value of the thermistor for **HotBed** (see thermistor specs).
            ///
            /// Typically, 3950.0 for the classic `NTC **3950** 100K`
            const HOT_BED_THERM_BETA: f32;

            /// The nominal thermistor value for **HotBed**. (see thermistor specs).
            ///
            /// Represents the resistance value in Ohms at 25ºC
            ///
            /// Typically, 100_000.0 for the classic `NTC 3950 **100K**`
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

    type EventBusPubSubMutexType: hwa::AsyncRawMutex;

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            type DeferChannelMutexType: hwa::AsyncRawMutex;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
            type MotionBroadcastChannelMutexType: hwa::AsyncRawMutex;

            type MotionSenderMutexStrategy: hwa::AsyncMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            type StepActuatorMutexType: hwa::SyncRawMutex;
            type MotionRingBufferMutexType: hwa::AsyncRawMutex;
            type MotionSignalMutexType: hwa::AsyncRawMutex;
            type MotionConfigMutexType: hwa::SyncRawMutex;
            type MotionStatusMutexType: hwa::SyncRawMutex;
            type MotionDriverMutexType: hwa::AsyncRawMutex;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-print-job")] {
            type PrinterControllerSignalMutexType: hwa::AsyncRawMutex;
        }
    }

    //#endregion

    //#region "Locking strategies"

    type WatchDogMutexStrategy: hwa::AsyncMutexStrategy;

    type EventBusMutexStrategy: hwa::AsyncMutexStrategy;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-usb")] {

            const SERIAL_USB_PACKET_SIZE: usize = 64;

            type SerialUsbTx: hwa::AsyncMutexStrategy;
            type SerialUsbRx: traits::GCodeByteStream;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-1")] {

            const SERIAL_PORT1_BAUD_RATE: u32;
            const SERIAL_PORT1_RX_BUFFER_SIZE: usize;

            type SerialPort1Tx: hwa::AsyncMutexStrategy;
            type SerialPort1Rx: traits::GCodeByteStream;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-2")] {
            const SERIAL_PORT2_BAUD_RATE: u32;
            const SERIAL_PORT2_RX_BUFFER_SIZE: usize;

            type SerialPort2Tx: hwa::AsyncMutexStrategy;
            type SerialPort2Rx: traits::GCodeByteStream;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-spi")] {
            const SPI_FREQUENCY: u32;
            type SpiController: hwa::AsyncMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-i2c")] {
            const I2C_FREQUENCY: u32;
            type I2cMotionMutexStrategy: hwa::AsyncMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            type StepActuatorMutexStrategy: hwa::SyncMutexStrategy;
            type StepActuator: traits::StepActuatorTrait;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-ps-on")] {
            type PSOnMutexStrategy: hwa::SyncMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            type ProbePwm: hwa::SyncMutexStrategy;
            type ProbePwmChannel: hwa::RawHwiResource + Copy;
        }
    }

    cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
            type HotEndAdc: hwa::AsyncMutexStrategy;
            type HotEndAdcPin: hwa::RawHwiResource;
            type HotEndPwm: hwa::SyncMutexStrategy;
            type HotEndPwmChannel: hwa::RawHwiResource + Copy;
        }
    }

    cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-bed")] {
            type HotBedAdc: hwa::AsyncMutexStrategy;
            type HotBedAdcPin: hwa::RawHwiResource;
            type HotBedPwm: hwa::SyncMutexStrategy;
            type HotBedPwmChannel: hwa::RawHwiResource + Copy;
        }
    }

    cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-layer")] {
            type FanLayerPwm: hwa::SyncMutexStrategy;
            type FanLayerPwmChannel: hwa::RawHwiResource + Copy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-sd-card")] {
            const SD_CARD_MAX_DIRS: usize;
            const SD_CARD_MAX_FILES: usize;
            type SDCardBlockDevice: hwa::traits::SDBlockDevice;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-laser")] {
            type LaserPwm: hwa::SyncMutexStrategy;
            type LaserPwmChannel: hwa::RawHwiResource + Copy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-extra-1")] {
            type FanExtra1Pwm: hwa::SyncMutexStrategy;
            type FanExtra1PwmChannel: hwa::RawHwiResource;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-trinamic")] {
            const TRINAMIC_UART_BAUD_RATE: u32;
            type TrinamicUartDevice: traits::TrinamicUartTrait;
        }
    }

    //#endregion

    /// Initialize the logger (optional)
    fn init_logger() {}

    /// Initialize the heap allocator (if needed)
    fn init_heap() {}

    // The HWI initialization
    fn init(spawner: embassy_executor::Spawner) -> impl future::Future<Output = HwiContext<Self>>;

    /// Resets the MCU/CPU
    fn sys_reset();

    /// Stop the MCU/CPU
    fn sys_stop();

    /// Execute closure f in an interrupt-free context.
    fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce() -> R;

    #[cfg(feature = "with-motion")]
    fn setup_ticker();

    #[cfg(feature = "with-motion")]
    fn pause_ticker();

    #[cfg(feature = "with-motion")]
    fn resume_ticker();

    #[cfg(feature = "with-motion-broadcast")]
    type HighPriorityCore;

    #[cfg(feature = "with-motion-broadcast")]
    fn launch_high_priotity<S: 'static + Sized + Send>(
        _core: Self::HighPriorityCore,
        token: embassy_executor::SpawnToken<S>,
    ) -> Result<(), ()>;
}

pub struct HwiContext<C>
where
    C: HwiContract + Sized,
{
    pub sys_watch_dog: hwa::StaticAsyncController<C::WatchDogMutexStrategy>,

    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: hwa::StaticAsyncController<C::SerialUsbTx>,
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_rx_stream: C::SerialUsbRx,

    #[cfg(feature = "with-serial-port-1")]
    pub serial_port_1_tx: hwa::StaticAsyncController<C::SerialPort1Tx>,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port_1_rx_stream: C::SerialPort1Rx,

    #[cfg(feature = "with-serial-port-2")]
    pub serial_port_2_tx: hwa::StaticAsyncController<C::SerialPort2Tx>,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port_2_rx_stream: C::SerialPort2Rx,

    #[cfg(feature = "with-spi")]
    pub spi: hwa::StaticAsyncController<C::SpiController>,

    #[cfg(feature = "with-i2c")]
    pub i2c: hwa::StaticAsyncController<C::I2cMotionMutexStrategy>,

    #[cfg(feature = "with-ps-on")]
    pub ps_on: hwa::StaticSyncController<C::PSOnMutexStrategy>,

    /// As of now, this mut be holdable
    #[cfg(feature = "with-motion")]
    pub motion_pins: C::StepActuator,

    #[cfg(feature = "with-trinamic")]
    pub trinamic_uart: C::TrinamicUartDevice,

    #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))]
    pub motion_sender: hwa::StaticAsyncController<C::MotionSenderMutexStrategy>,

    #[cfg(feature = "with-probe")]
    pub probe_pwm: hwa::StaticSyncController<C::ProbePwm>,

    #[cfg(feature = "with-probe")]
    pub probe_pwm_channel: C::ProbePwmChannel,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_adc: hwa::StaticAsyncController<C::HotEndAdc>,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_adc_pin: C::HotEndAdcPin,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_pwm: hwa::StaticSyncController<C::HotEndPwm>,

    #[cfg(feature = "with-hot-end")]
    pub hot_end_pwm_channel: C::HotEndPwmChannel,

    #[cfg(feature = "with-hot-bed")]
    pub hot_bed_adc: hwa::StaticAsyncController<C::HotBedAdc>,

    #[cfg(feature = "with-hot-bed")]
    pub hot_bed_adc_pin: C::HotBedAdcPin,

    #[cfg(feature = "with-hot-bed")]
    pub hot_bed_pwm: hwa::StaticSyncController<C::HotBedPwm>,

    #[cfg(feature = "with-hot-bed")]
    pub hot_bed_pwm_channel: C::HotBedPwmChannel,

    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_pwm: hwa::StaticSyncController<C::FanLayerPwm>,

    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_pwm_channel: C::FanLayerPwmChannel,

    #[cfg(feature = "with-sd-card")]
    pub sd_card_block_device: C::SDCardBlockDevice,

    #[cfg(feature = "with-laser")]
    pub laser_pwm: hwa::StaticSyncController<C::LaserPwm>,

    #[cfg(feature = "with-laser")]
    pub laser_pwm_channel: C::LaserPwmChannel,

    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra1_pwm: hwa::StaticSyncController<C::FanExtra1Pwm>,

    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra1_pwm_channel: C::FanExtra1PwmChannel,

    #[cfg(feature = "with-motion-broadcast")]
    pub high_priority_core: C::HighPriorityCore,
}
