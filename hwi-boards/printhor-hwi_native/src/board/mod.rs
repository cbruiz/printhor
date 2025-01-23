///  Native board implementation. For debugging/simulation purposes
use printhor_hwa_common as hwa;
use hwa::HwiContext;
use sysinfo;

pub mod device;

pub mod mocked_peripherals;
mod types;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-trinamic")] {
        pub mod comm;
    }
}

pub struct Contract;
impl hwa::HwiContract for Contract {

    //#region "Common constants"

    const MACHINE_TYPE: &'static str = "Simulator/debugger";
    const MACHINE_BOARD: &'static str = "PC";
    const MACHINE_PROCESSOR: &'static str = std::env::consts::ARCH;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-spi")] {
            #[const_env::from_env("SPI_FREQUENCY")]
            const SPI_FREQUENCY: u32 = 20_000_000;
            type SpiController = types::Spi1MutexStrategyType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-i2c")] {
            #[const_env::from_env("I2C_FREQUENCY")]
            const I2C_FREQUENCY: u32 = 1_000_000;
            type I2cMotionMutexStrategy = types::I2cMotionMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature="with-motion")] {
            #[const_env::from_env("PROCESSOR_SYS_CK_MHZ")]
            const PROCESSOR_SYS_CK_MHZ: u32 = 1_000_000_000;
        }
    }

    //#enregion

    //#region Hard-coded settings [...]

    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;

    #[const_env::from_env("MAX_HEAP_SIZE_BYTES")]
    const MAX_HEAP_SIZE_BYTES: usize = 0;

    #[const_env::from_env("MAX_EXPECTED_STATIC_ALLOC_BYTES")]
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize = 32768;

    //#endregion

    //#region Optional memory management/tracking [...]
    fn heap_current_size() -> usize {
        let mut system = sysinfo::System::new_all();
        system.refresh_all();
        match sysinfo::get_current_pid() {
            Ok(_pid) => {
                if let Some(process) = system.process(_pid) {
                    process.memory() as usize
                } else {
                    0
                }
            }
            Err(_) => {
                0
            }
        }
    }
    //#endregion

    //#region Constant settings for with-motion feature [...]

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {

            cfg_if::cfg_if! {
                if #[cfg(feature = "with-motion-broadcast")] {

                    #[const_env::from_env("WORLD_UNIT_MAGNITUDE")]
                    const WORLD_UNIT_MAGNITUDE: &'static str = "º";

                    #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
                    const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 25;

                    #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
                    const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 100;

                    #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
                    const SEGMENT_QUEUE_SIZE: u8 = 20;

                    const DEFAULT_MAX_SPEED: hwa::math::TVector<u32> = hwa::math::TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(720),
                        //
                        #[cfg(feature = "with-x-axis")] Some(720),
                        #[cfg(feature = "with-y-axis")] Some(720),
                        #[cfg(feature = "with-z-axis")] Some(720),
                        //
                        #[cfg(feature = "with-a-axis")] Some(720),
                        #[cfg(feature = "with-b-axis")] Some(720),
                        #[cfg(feature = "with-c-axis")] Some(720),
                        //
                        #[cfg(feature = "with-i-axis")] Some(720),
                        #[cfg(feature = "with-j-axis")] Some(720),
                        #[cfg(feature = "with-k-axis")] Some(720),
                        //
                        #[cfg(feature = "with-u-axis")] Some(720),
                        #[cfg(feature = "with-v-axis")] Some(720),
                        #[cfg(feature = "with-w-axis")] Some(720),
                    );

                    const DEFAULT_MAX_ACCEL: hwa::math::TVector<u32> = hwa::math::TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(720*4),
                        //
                        #[cfg(feature = "with-x-axis")] Some(720*4),
                        #[cfg(feature = "with-y-axis")] Some(720*4),
                        #[cfg(feature = "with-z-axis")] Some(720*4),
                        //
                        #[cfg(feature = "with-a-axis")] Some(720*4),
                        #[cfg(feature = "with-b-axis")] Some(720*4),
                        #[cfg(feature = "with-c-axis")] Some(720*4),
                        //
                        #[cfg(feature = "with-i-axis")] Some(720*4),
                        #[cfg(feature = "with-j-axis")] Some(720*4),
                        #[cfg(feature = "with-k-axis")] Some(720*4),
                        //
                        #[cfg(feature = "with-u-axis")] Some(720*4),
                        #[cfg(feature = "with-v-axis")] Some(720*4),
                        #[cfg(feature = "with-w-axis")] Some(720*4),
                    );

                    const DEFAULT_MAX_JERK: hwa::math::TVector<u32> = hwa::math::TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(720*8),
                        //
                        #[cfg(feature = "with-x-axis")] Some(720*8),
                        #[cfg(feature = "with-y-axis")] Some(720*8),
                        #[cfg(feature = "with-z-axis")] Some(720*8),
                        //
                        #[cfg(feature = "with-a-axis")] Some(720*8),
                        #[cfg(feature = "with-b-axis")] Some(720*8),
                        #[cfg(feature = "with-c-axis")] Some(720*8),
                        //
                        #[cfg(feature = "with-i-axis")] Some(720*8),
                        #[cfg(feature = "with-j-axis")] Some(720*8),
                        #[cfg(feature = "with-k-axis")] Some(720*8),
                        //
                        #[cfg(feature = "with-u-axis")] Some(720*8),
                        #[cfg(feature = "with-v-axis")] Some(720*8),
                        #[cfg(feature = "with-w-axis")] Some(720*8),
                    );

                    const DEFAULT_TRAVEL_SPEED: u16 = 720;

                    const DEFAULT_UNITS_PER_MM: hwa::math::TVector<f32> = hwa::math::TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(1.0),
                        //
                        #[cfg(feature = "with-x-axis")] Some(1.0),
                        #[cfg(feature = "with-y-axis")] Some(1.0),
                        #[cfg(feature = "with-z-axis")] Some(1.0),
                        //
                        #[cfg(feature = "with-a-axis")] Some(1.0),
                        #[cfg(feature = "with-b-axis")] Some(1.0),
                        #[cfg(feature = "with-c-axis")] Some(1.0),
                        //
                        #[cfg(feature = "with-i-axis")] Some(1.0),
                        #[cfg(feature = "with-j-axis")] Some(1.0),
                        #[cfg(feature = "with-k-axis")] Some(1.0),
                        //
                        #[cfg(feature = "with-u-axis")] Some(1.0),
                        #[cfg(feature = "with-v-axis")] Some(1.0),
                        #[cfg(feature = "with-w-axis")] Some(1.0),
                    );

                    const DEFAULT_MICRO_STEPS_PER_AXIS: hwa::math::TVector<u16> = hwa::math::TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(16),
                        //
                        #[cfg(feature = "with-x-axis")] Some(16),
                        #[cfg(feature = "with-y-axis")] Some(16),
                        #[cfg(feature = "with-z-axis")] Some(16),
                        //
                        #[cfg(feature = "with-a-axis")] Some(16),
                        #[cfg(feature = "with-b-axis")] Some(16),
                        #[cfg(feature = "with-c-axis")] Some(16),
                        //
                        #[cfg(feature = "with-i-axis")] Some(16),
                        #[cfg(feature = "with-j-axis")] Some(16),
                        #[cfg(feature = "with-k-axis")] Some(16),
                        //
                        #[cfg(feature = "with-u-axis")] Some(16),
                        #[cfg(feature = "with-v-axis")] Some(16),
                        #[cfg(feature = "with-w-axis")] Some(16),
                    );

                    const DEFAULT_MACHINE_BOUNDS: hwa::math::TVector<f32> = hwa::math::TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] None,
                        //
                        #[cfg(feature = "with-x-axis")] Some(90.0),
                        #[cfg(feature = "with-y-axis")] Some(90.0),
                        #[cfg(feature = "with-z-axis")] Some(90.0),
                        //
                        #[cfg(feature = "with-a-axis")] Some(90.0),
                        #[cfg(feature = "with-b-axis")] Some(90.0),
                        #[cfg(feature = "with-c-axis")] Some(90.0),
                        //
                        #[cfg(feature = "with-i-axis")] Some(90.0),
                        #[cfg(feature = "with-j-axis")] Some(90.0),
                        #[cfg(feature = "with-k-axis")] Some(90.0),
                        //
                        #[cfg(feature = "with-u-axis")] Some(90.0),
                        #[cfg(feature = "with-v-axis")] Some(90.0),
                        #[cfg(feature = "with-w-axis")] Some(90.0),
                    );

                }
                else {
                    #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
                    const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 100;

                    #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
                    const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;

                    #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
                    const SEGMENT_QUEUE_SIZE: u8 = 20;

                    const DEFAULT_MAX_SPEED: hwa::math::TVector<u32> = TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(600),
                        //
                        #[cfg(feature = "with-x-axis")] Some(600),
                        #[cfg(feature = "with-y-axis")] Some(600),
                        #[cfg(feature = "with-z-axis")] Some(100),
                        //
                        #[cfg(feature = "with-a-axis")] Some(600),
                        #[cfg(feature = "with-b-axis")] Some(600),
                        #[cfg(feature = "with-c-axis")] Some(100),
                        //
                        #[cfg(feature = "with-i-axis")] Some(600),
                        #[cfg(feature = "with-j-axis")] Some(600),
                        #[cfg(feature = "with-k-axis")] Some(100),
                        //
                        #[cfg(feature = "with-u-axis")] Some(600),
                        #[cfg(feature = "with-v-axis")] Some(600),
                        #[cfg(feature = "with-w-axis")] Some(100),
                    );

                    const DEFAULT_MAX_ACCEL: TVector<u32> = TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(9800),
                        //
                        #[cfg(feature = "with-x-axis")] Some(9800),
                        #[cfg(feature = "with-y-axis")] Some(9800),
                        #[cfg(feature = "with-z-axis")] Some(9800),
                        //
                        #[cfg(feature = "with-a-axis")] Some(9800),
                        #[cfg(feature = "with-b-axis")] Some(9800),
                        #[cfg(feature = "with-c-axis")] Some(9800),
                        //
                        #[cfg(feature = "with-i-axis")] Some(9800),
                        #[cfg(feature = "with-j-axis")] Some(9800),
                        #[cfg(feature = "with-k-axis")] Some(9800),
                        //
                        #[cfg(feature = "with-u-axis")] Some(9800),
                        #[cfg(feature = "with-v-axis")] Some(9800),
                        #[cfg(feature = "with-w-axis")] Some(9800),
                    );

                    const DEFAULT_MAX_ACCEL: TVector<u32> = TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(19600),
                        //
                        #[cfg(feature = "with-x-axis")] Some(19600),
                        #[cfg(feature = "with-y-axis")] Some(19600),
                        #[cfg(feature = "with-z-axis")] Some(6400),
                        //
                        #[cfg(feature = "with-a-axis")] None,
                        #[cfg(feature = "with-b-axis")] None,
                        #[cfg(feature = "with-c-axis")] None,
                        //
                        #[cfg(feature = "with-i-axis")] None,
                        #[cfg(feature = "with-j-axis")] None,
                        #[cfg(feature = "with-k-axis")] None,
                        //
                        #[cfg(feature = "with-u-axis")] None,
                        #[cfg(feature = "with-v-axis")] None,
                        #[cfg(feature = "with-w-axis")] None,
                    );

                    const DEFAULT_TRAVEL_SPEED: u16 = 600;

                    const DEFAULT_UNITS_PER_MM: TVector<f32> = TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(50.0),
                        //
                        #[cfg(feature = "with-x-axis")] Some(10.0),
                        #[cfg(feature = "with-y-axis")] Some(10.0),
                        #[cfg(feature = "with-z-axis")] Some(50.0),
                        //
                        #[cfg(feature = "with-a-axis")] None,
                        #[cfg(feature = "with-b-axis")] None,
                        #[cfg(feature = "with-c-axis")] None,
                        //
                        #[cfg(feature = "with-i-axis")] None,
                        #[cfg(feature = "with-j-axis")] None,
                        #[cfg(feature = "with-k-axis")] None,
                        //
                        #[cfg(feature = "with-u-axis")] None,
                        #[cfg(feature = "with-v-axis")] None,
                        #[cfg(feature = "with-w-axis")] None,
                    );

                    const DEFAULT_MICRO_STEPS_PER_AXIS: TVector<u16> = TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] Some(2),
                        //
                        #[cfg(feature = "with-x-axis")] Some(2),
                        #[cfg(feature = "with-y-axis")] Some(2),
                        #[cfg(feature = "with-z-axis")] Some(2),
                        //
                        #[cfg(feature = "with-a-axis")] None,
                        #[cfg(feature = "with-b-axis")] None,
                        #[cfg(feature = "with-c-axis")] None,
                        //
                        #[cfg(feature = "with-i-axis")] None,
                        #[cfg(feature = "with-j-axis")] None,
                        #[cfg(feature = "with-k-axis")] None,
                        //
                        #[cfg(feature = "with-u-axis")] None,
                        #[cfg(feature = "with-v-axis")] None,
                        #[cfg(feature = "with-w-axis")] None,
                    );

                    const DEFAULT_MACHINE_BOUNDS: TVector<f32> = TVector::from_coords(
                        #[cfg(feature = "with-e-axis")] None,
                        //
                        #[cfg(feature = "with-x-axis")] Some(200.0),
                        #[cfg(feature = "with-y-axis")] Some(200.0),
                        #[cfg(feature = "with-z-axis")] Some(200.0),
                        //
                        #[cfg(feature = "with-a-axis")] None,
                        #[cfg(feature = "with-b-axis")] None,
                        #[cfg(feature = "with-c-axis")] None,
                        //
                        #[cfg(feature = "with-i-axis")] None,
                        #[cfg(feature = "with-j-axis")] None,
                        #[cfg(feature = "with-k-axis")] None,
                        //
                        #[cfg(feature = "with-u-axis")] None,
                        #[cfg(feature = "with-v-axis")] None,
                        #[cfg(feature = "with-w-axis")] None,
                    );
                }
            }
        }
    }

    //#endregion

    //#region Constant settings for with-hot-end feature [...]

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {

            #[const_env::from_env("HOT_END_ADC_V_REF_DEFAULT_MV")]
            const HOT_END_ADC_V_REF_DEFAULT_MV: u16 = 4096;

            #[const_env::from_env("HOT_END_ADC_V_REF_DEFAULT_SAMPLE")]
            const HOT_END_ADC_V_REF_DEFAULT_SAMPLE: u16 = 4096;

            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_BETA: f32 = 3950.0;

            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_NOMINAL_RESISTANCE: f32 = 100_000.0;

            #[const_env::from_env("HOT_END_THERM_PULL_UP_RESISTANCE")]
            const HOT_END_THERM_PULL_UP_RESISTANCE: f32 = 1.0;
        }
    }

    //#endregion

    //#region Constant settings for with-hot-bed feature [...]

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-bed")] {

            #[const_env::from_env("HOT_BED_ADC_V_REF_DEFAULT_MV")]
            const HOT_BED_ADC_V_REF_DEFAULT_MV: u16 = 4096;

            #[const_env::from_env("HOT_BED_ADC_V_REF_DEFAULT_SAMPLE")]
            const HOT_BED_ADC_V_REF_DEFAULT_SAMPLE: u16 = 4096;

            #[const_env::from_env("HOT_BED_THERM_BETA")]
            const HOT_BED_THERM_BETA: f32 = 3950.0;

            #[const_env::from_env("HOT_BED_THERM_NOMINAL_RESISTANCE")]
            const HOT_BED_THERM_NOMINAL_RESISTANCE: f32 = 100_000.0;

            #[const_env::from_env("HOT_BED_THERM_PULL_UP_RESISTANCE")]
            const HOT_BED_THERM_PULL_UP_RESISTANCE: f32 = 1.0;
        }
    }

    //#endregion

    //#region Customization of Mutex types [...]

    type EventBusPubSubMutexType = types::EventBusPubSubMutexType;
    type WatchDogMutexStrategy = types::WatchDogMutexStrategy;

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            type DeferChannelMutexType = types::DeferChannelMutexType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            type MotionPinsMutexType = types::MotionPinsMutexType;
            type MotionSignalMutexType = types::MotionSignalMutexType;
            type MotionRingBufferMutexType = types::MotionRingBufferMutexType;
            type MotionConfigMutexType = types::MotionConfigMutexType;
            type MotionStatusMutexType = types::MotionStatusMutexType;
            type MotionDriverMutexType = types::MotionDriverMutexType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
            type MotionBroadcastChannelMutexType = types::MotionBroadcastChannelMutexType;

            type MotionSenderMutexStrategy = types::MotionSenderMutexStrategy;
        }
    }

    //#endregion

    //#region Locking strategies for shared devices/controllers/adaptors [...]

    type EventBusMutexStrategy = types::EventBusMutexStrategy;


    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-usb")] {
            #[const_env::from_env("SERIAL_USB_RX_BUFFER_SIZE")]
            const SERIAL_USB_RX_BUFFER_SIZE: usize = 128;
            type SerialUsbTx = types::SerialUsbTxMutexStrategy;
            type SerialUsbRx = device::SerialUsbRx;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-1")] {
            #[const_env::from_env("SERIAL_PORT1_BAUD_RATE")]
            const SERIAL_PORT1_BAUD_RATE: u32 = 115200;
            #[const_env::from_env("SERIAL_PORT1_RX_BUFFER_SIZE")]
            const SERIAL_PORT1_RX_BUFFER_SIZE: usize = 128;

            type SerialPort1Tx = types::SerialPort1TxMutexStrategy;
            type SerialPort1Rx = device::SerialPort1Rx;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-2")] {
            #[const_env::from_env("SERIAL_PORT2_BAUD_RATE")]
            const SERIAL_PORT2_BAUD_RATE: u32 = 115200;
            #[const_env::from_env("SERIAL_PORT2_RX_BUFFER_SIZE")]
            const SERIAL_PORT2_RX_BUFFER_SIZE: usize = 128;

            type SerialPort2Tx = types::SerialPort2TxMutexStrategy;
            type SerialPort2Rx = device::SerialPort2Rx;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            type MotionPinsMutexStrategy = types::MotionPinsMutexStrategy;
            type MotionPins = device::MotionPins;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-ps-on")] {
            type PSOnMutexStrategy = types::PSOnMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            type ProbePwm = types::ProbeMutexStrategy;
            type ProbePwmChannel = hwa::HwiResource<device::ProbePwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-end")] {
            type HotEndAdc = types::HotEndAdcMutexStrategy;
            type HotEndAdcPin = hwa::HwiResource<device::HotEndAdcPin>;
            type HotEndPwm = types::HotEndPwmMutexStrategy;
            type HotEndPwmChannel = hwa::HwiResource<device::HotEndPwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-bed")] {
            type HotBedAdc = types::HotBedAdcMutexStrategy;
            type HotBedAdcPin = hwa::HwiResource<device::HotBedAdcPin>;
            type HotBedPwm = types::HotBedPwmMutexStrategy;
            type HotBedPwmChannel = hwa::HwiResource<device::HotBedPwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-layer")] {
            type FanLayerPwm = types::FanLayerPwmMutexStrategy;
            type FanLayerPwmChannel = hwa::HwiResource<device::FanLayerPwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-laser")] {
            type LaserPwm = types::LaserPwmMutexStrategy;
            type LaserPwmChannel = hwa::HwiResource<device::LaserPwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-extra-1")] {
            type FanExtra1Pwm = types::FanExtra1PwmMutexStrategy;
            type FanExtra1PwmChannel = hwa::HwiResource<device::FanExtra1PwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-sd-card")] {
            const SD_CARD_MAX_FILES: usize = 4;
            const SD_CARD_MAX_DIRS: usize = 4;
            type SDCardBlockDevice = types::SDCardBlockDevice;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-print-job")] {
            type PrinterControllerSignalMutexType = types::PrinterControllerSignalMutexType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-trinamic")] {
            const TRINAMIC_UART_BAUD_RATE: u32 = 4096;
        }
    }

    //#endregion

    /// Initialize the logger (optional)
    fn init_logger() {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-log")] {
                use std::io::Write;
                let env = env_logger::Env::new().default_filter_or("info");
                env_logger::builder()
                    .parse_env(env)
                    .format(|buf, record| {
                        writeln!(buf, "{}: {}", record.level(), record.args())
                    })
                    .init();
            }
        }
    }

    /// Initialize the heap allocator (if needed)
    fn init_heap() {}

    // The HWI initialization
    async fn init(_spawner: embassy_executor::Spawner) -> HwiContext<Self> {

        #[cfg(feature = "with-motion")]
        launch_task_stepper_ticker();

        let _pin_state = hwa::make_static_ref!(
            "GlobalPinState",
            mocked_peripherals::PinsCell<mocked_peripherals::PinState>,
            mocked_peripherals::PinsCell::new(mocked_peripherals::PinState::new())
        );

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-usb"))] {
                let (usb_tx_device, usb_rx_device) = device::SerialUsbDevice::new("/tmp/printhor.sock".to_string()).await.split();
                let serial_usb_tx = hwa::make_static_async_controller!(
                    "UsbTx",
                    types::SerialUsbTxMutexStrategy,
                    usb_tx_device
                );
                let serial_usb_rx_stream = mocked_peripherals::MockedUartNamedPipeRxInputStream::new(usb_rx_device);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-1"))] {
                let (uart_port1_tx_device, uart_port1_rx_device) = device::SerialPort1Device::new(_spawner.make_send()).split();
                let serial_port1_tx = hwa::make_static_async_controller!(
                    "UartPort1Tx",
                    types::SerialPort1TxMutexStrategy,
                    uart_port1_tx_device
                );
                let serial_port1_rx_stream = device::SerialPort1Rx::new(uart_port1_rx_device);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-2"))] {
                let (uart_port2_tx_device, uart_port2_rx_device) = device::SerialPort2Device::new().split();
                let serial_port2_tx = hwa::make_static_async_controller!(
                    "UartPort2Tx",
                    types::SerialPort2TxMutexStrategy,
                    uart_port2_tx_device
                );
                let serial_port2_rx_stream = device::SerialPort2Rx::new(uart_port2_rx_device);
            }
        }

        #[cfg(all(feature = "with-trinamic"))]
        let trinamic_uart = {
            device::TrinamicUart::new(
                Self::TRINAMIC_UART_BAUD_RATE,
                mocked_peripherals::MockedIOPin::new(0, _pin_state),
                mocked_peripherals::MockedIOPin::new(1, _pin_state),
                mocked_peripherals::MockedIOPin::new(2, _pin_state),
                mocked_peripherals::MockedIOPin::new(3, _pin_state),
            )
        };

        #[cfg(all(feature = "with-trinamic"))]
        {
            _spawner.spawn(
                device::trinamic_driver_simulator(
                    device::MockedTrinamicDriver::new(
                        Self::TRINAMIC_UART_BAUD_RATE,
                        mocked_peripherals::MockedIOPin::new(0, _pin_state),
                        mocked_peripherals::MockedIOPin::new(1, _pin_state),
                        mocked_peripherals::MockedIOPin::new(2, _pin_state),
                        mocked_peripherals::MockedIOPin::new(3, _pin_state),
                    )
                )
            ).unwrap();
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-spi")] {
                let spi = hwa::make_static_async_controller!(
                    "SPI1",
                    types::Spi1MutexStrategyType,
                    device::Spi::new()

                );
                hwa::info!("SPI done");
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-i2c")] {
                let i2c = hwa::make_static_async_controller!(
                    "I2C1",
                    types::I2cMotionMutexStrategy,
                    device::I2c::new()

                );
                hwa::info!("I2C done");
            }
        }


        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                let motion_pins = device::MotionPins {
                    x_enable_pin: mocked_peripherals::MockedIOPin::new(4, _pin_state),
                    y_enable_pin: mocked_peripherals::MockedIOPin::new(5, _pin_state),
                    z_enable_pin: mocked_peripherals::MockedIOPin::new(6, _pin_state),
                    e_enable_pin: mocked_peripherals::MockedIOPin::new(7, _pin_state),
                    x_endstop_pin: mocked_peripherals::MockedIOPin::new(8, _pin_state),
                    y_endstop_pin: mocked_peripherals::MockedIOPin::new(9, _pin_state),
                    z_endstop_pin: mocked_peripherals::MockedIOPin::new(10, _pin_state),
                    e_endstop_pin: mocked_peripherals::MockedIOPin::new(11, _pin_state),
                    x_step_pin: mocked_peripherals::MockedIOPin::new(12, _pin_state),
                    y_step_pin: mocked_peripherals::MockedIOPin::new(13, _pin_state),
                    z_step_pin: mocked_peripherals::MockedIOPin::new(14, _pin_state),
                    e_step_pin: mocked_peripherals::MockedIOPin::new(15, _pin_state),
                    x_dir_pin: mocked_peripherals::MockedIOPin::new(16, _pin_state),
                    y_dir_pin: mocked_peripherals::MockedIOPin::new(17, _pin_state),
                    z_dir_pin: mocked_peripherals::MockedIOPin::new(18, _pin_state),
                    e_dir_pin: mocked_peripherals::MockedIOPin::new(19, _pin_state),
                };

                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-motion-broadcast")] {
                        let motion_sender = i2c.clone();
                    }
                }
                hwa::debug!("motion_driver done");
            }
        }

        #[cfg(any(
            feature = "with-probe",
            feature = "with-hot-end",
            feature = "with-hot-bed",
            feature = "with-fan-layer",
            feature = "with-fan-extra-1",
            feature = "with-laser"
        ))]
        let pwm1 = hwa::make_static_sync_controller!(
            "Pwm1Controller",
            types::Pwm1MutexStrategy,
            device::Pwm1::new(20, _pin_state)
        );

        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        let adc_any = hwa::make_static_async_controller!(
            "Adc1Controller",
            types::Adc1MutexStrategy,
            device::Adc1::new(0, 4096)
        );

        #[cfg(feature = "with-motion")]
        hwa::debug!("motion_planner done");

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                let probe_pwm = pwm1.clone();
                let probe_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-hot-end")] {
                let hot_end_adc = adc_any.clone();
                let hot_end_adc_pin = hwa::HwiResource::new(mocked_peripherals::MockedIOPin::new(23, _pin_state));
                let hot_end_pwm= pwm1.clone();
                let hot_end_pwm_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-hot-bed")] {
                let hot_bed_adc = adc_any.clone();
                let hot_bed_adc_pin = hwa::HwiResource::new(mocked_peripherals::MockedIOPin::new(24, _pin_state));
                let hot_bed_pwm= pwm1.clone();
                let hot_bed_pwm_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-fan-layer")] {
                let fan_layer_pwm = pwm1.clone();
                let fan_layer_pwm_channel = hwa::HwiResource::new(0u8);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-sd-card")] {
                let sd_card_block_device = {
                     device::SDCardBlockDevice::new("data/sdcard.img", false).unwrap()
                };
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-laser")] {
                let laser_pwm = pwm1.clone();
                let laser_pwm_channel = hwa::HwiResource::new(0u8);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-fan-extra-1")] {
                let fan_extra1_pwm = pwm1.clone();
                let fan_extra1_pwm_channel = hwa::HwiResource::new(0u8);
            }
        }

        hwa::HwiContext {
            sys_watch_dog: hwa::make_static_async_controller!(
                "WatchDogController",
                types::WatchDogMutexStrategy,
                device::WatchDog::new(_spawner.make_send(), Self::WATCHDOG_TIMEOUT_US)
            ),
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_tx,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_rx_stream,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_rx_stream,
            #[cfg(feature = "with-spi")]
            spi,
            #[cfg(feature = "with-i2c")]
            i2c,
            #[cfg(feature = "with-ps-on")]
            ps_on: hwa::make_static_sync_controller!(
                "PSOnController",
                types::PSOnMutexStrategy,
                device::PsOnPin::new(21, _pin_state)
            ),
            #[cfg(feature = "with-motion")]
            motion_pins,
            #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))]
            motion_sender,
            #[cfg(feature = "with-probe")]
            probe_pwm,
            #[cfg(feature = "with-probe")]
            probe_pwm_channel: probe_channel,
            #[cfg(feature = "with-laser")]
            laser_pwm,
            #[cfg(feature = "with-laser")]
            laser_pwm_channel,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_pwm,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_pwm_channel,
            #[cfg(feature = "with-hot-end")]
            hot_end_adc,
            #[cfg(feature = "with-hot-end")]
            hot_end_adc_pin,
            #[cfg(feature = "with-hot-end")]
            hot_end_pwm,
            #[cfg(feature = "with-hot-end")]
            hot_end_pwm_channel,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_adc,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_adc_pin,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_pwm,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_pwm_channel,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_pwm,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_pwm_channel,
            #[cfg(feature = "with-sd-card")]
            sd_card_block_device,
            #[cfg(feature = "with-motion-broadcast")]
            high_priority_core: hwa::NoDevice,
        }
    }

    /// Resets the MCU/CPU
    fn sys_reset() {
        hwa::info!("Finishing process. Native board has not the capacity to restart");
        std::process::exit(0);
    }

    /// Stop the MCU/CPU
    fn sys_stop() {
        hwa::info!("Sending terminate signal");
        TERMINATION.signal(true);
    }


    fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce() -> R
    {
        f()
    }

    #[cfg(feature = "with-motion")]
    fn pause_ticker() {
        hwa::info!("Ticker Paused");
        TICKER_SIGNAL.reset();
    }

    #[cfg(feature = "with-motion")]
    fn resume_ticker() {
        hwa::debug!("Ticker Resumed");
        TICKER_SIGNAL.signal(true);
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion-broadcast")] {

            type HighPriorityCore = hwa::NoDevice;

            fn launch_high_priotity<S: 'static + Sized + Send>(_core: Self::HighPriorityCore, token: embassy_executor::SpawnToken<S>) -> Result<(),()>
            {
                use std::sync::{Arc, Mutex};

                let tk = Arc::new(Mutex::new(Some(SendWrapper::new(token))));

                let _ = thread_priority::ThreadBuilder::default()
                    .name("Broadcaster")
                    .priority(thread_priority::ThreadPriority::Max)
                    .spawn(move |_result| {
                        // This is printed out from within the spawned thread.
                        let executor = hwa::make_static_ref!(
                            "Executor",
                            embassy_executor::Executor,
                            embassy_executor::Executor::new()
                        );
                        executor.run(move |_spawner: embassy_executor::Spawner| {
                            use std::ops::DerefMut;
                            let spawner = _spawner.make_send();
                            let mut mg = tk.lock().unwrap();
                            let t = mg.deref_mut().take().unwrap();
                            spawner.must_spawn(t.inner);
                        });
                    }).unwrap();
                Ok(())
            }
        }
    }
}

    //#region "Custom machinery"

pub(crate) static TERMINATION: hwa::PersistentState<hwa::AsyncCsMutexType, bool> = hwa::PersistentState::new();

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub(crate) static TICKER_SIGNAL: hwa::PersistentState<hwa::AsyncCsMutexType, bool> = hwa::PersistentState::new();

        pub fn launch_task_stepper_ticker()
        {
            use hwa::HwiContract;

            hwa::info!("[task_stepper_ticker] starting");

            let _ = thread_priority::ThreadBuilder::default().name("Ticker")
                .spawn(move |_result| {
                    let max_freq = Contract::STEP_PLANNER_CLOCK_FREQUENCY as f32;
                    let pulse_period = std::time::Duration::from_secs_f32(1.0/max_freq);
                    let mut next_frame = std::time::Instant::now();
                    // This is printed out from within the spawned thread.
                    loop {

                        if TERMINATION.signaled() {
                            break;
                        }
                        if TICKER_SIGNAL.signaled() {
                            unsafe {
                                do_tick();
                            }
                            if let Some(delay) = next_frame.checked_duration_since(
                                std::time::Instant::now()
                            ) {
                                std::thread::sleep(delay);
                            }
                            next_frame += pulse_period;
                        }
                        else {
                            std::thread::sleep(std::time::Duration::from_millis(500));
                            next_frame = std::time::Instant::now();
                        }
                    }
                }).unwrap();
        }

        extern "Rust" {fn do_tick();}
    }
}

//#endregion

cfg_if::cfg_if!{
    if #[cfg(feature = "with-motion-broadcast")] {
        struct SendWrapper<T> {
            pub inner: T
        }
        impl<T> SendWrapper<T> {
            const fn new (inner: T) -> Self {
                Self { inner }
            }
        }
        unsafe impl<T> Send for SendWrapper<T> {}
    }
}
