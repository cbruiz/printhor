#![allow(stable_features)]
#[allow(unused)]
use printhor_hwa_common as hwa;
use printhor_hwa_common::HwiContract;
mod board;

//#region "Mutex types for this board"

type EventBusLockType = hwa::SyncSendMutex;
type EventBusPubSubMutexType = hwa::SyncSendMutex;
#[allow(unused)]
type DeferChannelMutexType = hwa::NoopMutex;

type EventBusMutexStrategyType<L,D> = hwa::NotHoldable<L,D>;

//pub type EventBusChannelMutexStrategyType<D> = hwa::NotHoldable<hwa::NoopMutex, D>;

type WatchDogMutexStrategyType<D> = hwa::NotHoldable<hwa::NoopMutex, D>;
pub type MotionDriverMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type MotionPlannerMutexStrategyType<D> = hwa::NotHoldable<hwa::NoopMutex, D>;
pub type MotionConfigMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type MotionStatusMutexStrategyType<D> = hwa::NotHoldable<hwa::NoopMutex, D>;
pub type MotionRingBufferMutexStrategyType<D> = hwa::NotHoldable<hwa::NoopMutex, D>;
pub type MotionSignalMutexType = hwa::NoopMutex;

//pub type Pwm1MutexType = hwa::SyncSendMutex;
//pub type Pwm2MutexType = hwa::SyncSendMutex;
pub type Pwm1ControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type Pwm2ControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;

pub type Adc1ControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type AdcHotEndMutexStrategyType<D> = Adc1ControllerMutexStrategyType<D>;
pub type AdcHotBedMutexStrategyType<D> = Adc1ControllerMutexStrategyType<D>;

pub type Adc2ControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;

pub type ProbeServoControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;

pub type PwmProbeMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type PwmHotEndMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type PwmHotBedMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type PwmFanLayerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type PwmFanExtra1MutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type PwmLaserMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type PSOnMutexStrategyType<D> = hwa::NotHoldable<hwa::NoopMutex, D>;

pub type PrinterControllerSignalMutexType = hwa::NoopMutex;

pub type FanLayerControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type FanExtra1ControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type LaserControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type HotEndControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type HotBedControllerMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;

pub type Spi1MutexStrategyType<D> = hwa::Holdable<hwa::SyncSendMutex, D>;
pub type SDCardMutexStrategyType<D> = hwa::Holdable<hwa::SyncSendMutex, D>;
pub type SPIControllerMutexStrategyType<D> = hwa::Holdable<hwa::SyncSendMutex, D>;


pub type SerialPort1MutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type SerialPort2MutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;
pub type SerialUsbMutexStrategyType<D> = hwa::NotHoldable<hwa::SyncSendMutex, D>;

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

#[cfg(feature = "with-motion")]
extern "Rust" {fn do_tick();}

static TICKER_SIGNAL: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();

static TERMINATION: hwa::PersistentState<CriticalSectionRawMutex, bool> = hwa::PersistentState::new();

#[cfg(feature = "with-motion")]
#[embassy_executor::task]
pub async fn task_stepper_ticker()
{
    hwa::info!("[task_stepper_ticker] starting");
    let mut t = embassy_time::Ticker::every(embassy_time::Duration::from_micros((1_000_000 / Contract::STEP_PLANNER_CLOCK_FREQUENCY) as u64));
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

    #[cfg(feature = "with-serial-port-1")]
    type SerialPort1Tx = crate::SerialPort1MutexStrategyType<device::SerialPort1TxDevice>;

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

    fn init_logger() {
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


    async fn init(_spawner: embassy_executor::Spawner) ->  hwa::HwiContext<Self> {
        use board::mocked_peripherals;

        #[cfg(feature = "with-motion")]
        let _ = _spawner.spawn(task_stepper_ticker());

        let _pin_state = hwa::make_static_ref!(
            "GlobalPinState",
            mocked_peripherals::PinsCell<mocked_peripherals::PinState>,
            mocked_peripherals::PinsCell::new(mocked_peripherals::PinState::new())
        );

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-usb"))] {
                let (usb_tx_device, usb_rx_device) = device::SerialUsbDevice::new().split();
                let serial_usb_tx = hwa::make_static_controller!(
                    "UsbTx",
                    crate::SerialUsbMutexStrategyType<device::SerialUsbTxDevice>,
                    usb_tx_device
                );
                let serial_usb_rx_stream = hwa::HwiResource::new(device::SerialUsbInputStream::new(usb_rx_device));
            }
        }

        cfg_if::cfg_if!{
        if #[cfg(all(feature = "with-serial-port-1"))] {
            let (uart_port1_tx_device, uart_port1_rx_device) = device::SerialPort1Device::new(_spawner.make_send()).split();
            let serial_port1_tx = hwa::make_static_controller!(
                "UartPort1Tx",
                crate::SerialPort1MutexStrategyType<device::SerialPort1TxDevice>,
                uart_port1_tx_device
            );
            let serial_port1_rx_stream = hwa::HwiResource::new(device::SerialPort1InputStream::new(uart_port1_rx_device));
        }
    }

        cfg_if::cfg_if!{
        if #[cfg(all(feature = "with-serial-port-2"))] {
            let (uart_port2_tx_device, uart_port2_rx_device) = device::SerialPort2Device::new().split();
            let serial_port2_tx = hwa::make_static_controller!(
                "UartPort2Tx",
                crate::SerialPort2MutexStrategyType<device::SerialPort2TxDevice>,
                uart_port2_tx_device
            );
            let serial_port2_rx_stream = hwa::HwiResource::new(device::SerialPort2InputStream::new(uart_port2_rx_device));
        }
    }

        #[cfg(all(feature = "with-trinamic"))]
        let trinamic_uart = {
            device::TrinamicUart::new(
                TRINAMIC_UART_BAUD_RATE,
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
                        mocked_peripherals::MockedIOPin::new(0, _pin_state),
                        mocked_peripherals::MockedIOPin::new(1, _pin_state),
                        mocked_peripherals::MockedIOPin::new(2, _pin_state),
                        mocked_peripherals::MockedIOPin::new(3, _pin_state),
                    )
                )
            ).unwrap();
        }


        #[cfg(feature = "with-spi")]
        let spi1_device = hwa::make_static_controller!(
            "SPI1",
            crate::Spi1MutexStrategyType<device::Spi>,
            device::Spi::new()

        );

        #[cfg(feature = "with-spi")]
        hwa::debug!("SPI done");

        #[cfg(feature = "with-sd-card")]
        let sd_card_device = {
            device::SDCardBlockDevice::new("data/sdcard.img", false).unwrap()
        };

        #[cfg(feature = "with-motion")]
        let motion_devices = device::MotionDevice {
            #[cfg(feature = "with-trinamic")]
            trinamic_uart,
            motion_pins: device::MotionPins {
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
            }
        };
        #[cfg(feature = "with-motion")]
        hwa::debug!("motion_driver done");

        #[cfg(any(feature = "with-probe", feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))]
        let pwm_any = hwa::make_static_controller!(
            "Pwm1Controller",
            crate::Pwm1ControllerMutexStrategyType<device::PwmAny>,
            device::PwmAny::new(20, _pin_state)
        );

        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        let adc_any = hwa::make_static_controller!(
            "Adc1Controller",
            crate::Adc1ControllerMutexStrategyType<device::Adc1>,
            device::AdcHotEnd::new(0)
        );

        #[cfg(feature = "with-motion")]
        hwa::debug!("motion_planner done");

        #[cfg(feature = "with-ps-on")]
        let ps_on = hwa::make_static_controller!(
        "PSOn",
        crate::PSOnMutexStrategyType<device::PsOnPin>,
        device::PsOnPin::new(21, _pin_state)
    );

        let inst = device::WatchDog::new(_spawner.make_send(), Self::WATCHDOG_TIMEOUT_US);
        let sys_watch_dog = hwa::make_static_controller!(
        "WatchDog",
        crate::WatchDogMutexStrategyType<device::WatchDog>,
        inst
    );
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                let probe_power_pwm = pwm_any.clone();
                let probe_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-laser")] {
                let laser_power_pwm = pwm_any.clone();
                let laser_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-fan-layer")] {
                let fan_layer_power_pwm = pwm_any.clone();
                let fan_layer_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-fan-extra-1")] {
                let fan_extra1_power_pwm = pwm_any.clone();
                let fan_extra1_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-hot-end")] {
                let hot_end_adc = adc_any.clone();
                let hot_end_adc_pin = hwa::HwiResource::new(mocked_peripherals::MockedIOPin::new(23, _pin_state));
                let hot_end_power_pwm= pwm_any.clone();
                let hot_end_power_channel = hwa::HwiResource::new(0u8);
            }
        }

        /*

        hwa::MachineContext {
            controllers: Controllers {
                sys_watchdog: sys_watch_dog,
                #[cfg(feature = "with-serial-port-1")]
                serial_port1_tx,
                #[cfg(feature = "with-serial-port-2")]
                serial_port2_tx,
            },
            sys_devices: SysDevices {
                #[cfg(all(feature = "with-motion", feature="executor-interrupt"))]
                task_stepper_core: printhor_hwa_common::NoDevice::new(),
                #[cfg(feature = "with-ps-on")]
                ps_on
            },
            io_devices: IODevices {
                #[cfg(feature = "with-serial-port-1")]
                serial_port1_rx_stream,
                #[cfg(feature = "with-serial-port-2")]
                serial_port2_rx_stream,
                #[cfg(feature = "with-sd-card")]
                sd_card_device,
            },
            motion: MotionDevices {
                #[cfg(feature = "with-motion")]
                motion_devices
            },
            pwm: PwmDevices {
                #[cfg(feature = "with-probe")]
                probe: device::ProbePeripherals {
                    power_pwm: pwm_any.clone(),
                    power_channel: 0,
                },
                #[cfg(feature = "with-hot-end")]
                hot_end: device::HotEndPeripherals {
                    power_pwm: pwm_any.clone(),
                    power_channel: 1,
                    temp_adc: adc_any.clone(),
                    temp_pin: mocked_peripherals::MockedIOPin::new(23, _pin_state),
                    thermistor_properties: &HOT_END_THERMISTOR_PROPERTIES,
                },
                #[cfg(feature = "with-hot-bed")]
                hot_bed: device::HotBedPeripherals {
                    power_pwm: pwm_any.clone(),
                    power_channel: 2,
                    temp_adc: adc_any.clone(),
                    temp_pin: mocked_peripherals::MockedIOPin::new(24, _pin_state),
                    thermistor_properties: &HOT_BED_THERMISTOR_PROPERTIES,
                },
                #[cfg(feature = "with-fan-layer")]
                fan_layer: device::FanLayerPeripherals {
                    power_pwm: pwm_any.clone(),
                    power_channel: 4,
                },
                #[cfg(feature = "with-fan-extra-1")]
                fan_extra_1: device::FanExtra1Peripherals {
                    power_pwm: pwm_any.clone(),
                    power_channel: 5,
                },
                #[cfg(feature = "with-laser")]
                laser: device::LaserPeripherals {
                    power_pwm: pwm_any.clone(),
                    power_channel: 6,
                },
            }
        }


         */

        hwa::HwiContext {
            sys_watch_dog,
            #[cfg(feature="with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature="with-serial-port-1")]
            serial_port1_tx,
            #[cfg(feature="with-serial-port-2")]
            serial_port2_tx,
            #[cfg(feature="with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature="with-serial-port-1")]
            serial_port1_rx_stream,
            #[cfg(feature="with-serial-port-2")]
            serial_port2_rx_stream,
            #[cfg(feature="with-ps-on")]
            ps_on,
            #[cfg(feature="with-probe")]
            probe_power_pwm,
            #[cfg(feature="with-probe")]
            probe_power_channel,
            #[cfg(feature="with-laser")]
            laser_power_pwm,
            #[cfg(feature="with-laser")]
            laser_power_channel,
            #[cfg(feature="with-fan-layer")]
            fan_layer_power_pwm,
            #[cfg(feature="with-fan-layer")]
            fan_layer_power_channel,
            #[cfg(feature="with-fan-extra-1")]
            fan_extra1_power_pwm,
            #[cfg(feature="with-fan-extra-1")]
            fan_extra1_power_channel,
            #[cfg(feature="with-hot-end")]
            hot_end_adc,
            #[cfg(feature="with-hot-end")]
            hot_end_adc_pin,
            #[cfg(feature="with-hot-end")]
            hot_end_power_pwm,
            #[cfg(feature="with-hot-end")]
            hot_end_power_channel,
        }

    }


    fn sys_reset() {
        std::process::exit(0);
    }

    fn sys_stop() {
        hwa::info!("Sending terminate signal");
        TERMINATION.signal(true);
    }

    // Execute closure f in an interrupt-free context.
    // In native this is not required, so does nothing
    fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce() -> R,
    {
        f()
    }

    fn pause_ticker() {
        todo!()
    }

    fn resume_ticker() {
        todo!()
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