///  Native board implementation. For debugging/simulation purposes
#[allow(unused)]
use printhor_hwa_common as hwa;
use embassy_executor::Spawner;

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

pub const MACHINE_TYPE: &str = "Simulator/debugger";
pub const MACHINE_BOARD: &str = "PC";
pub const MACHINE_PROCESSOR: &str = std::env::consts::ARCH;
#[allow(unused)]
pub(crate) const PROCESSOR_SYS_CK_MHZ: u32 = 1_000_000_000;
pub const HEAP_SIZE_BYTES: usize = 1024;

pub const VREF_SAMPLE: u16 = 1210u16;
#[cfg(feature = "with-sdcard")]
pub const SDCARD_PARTITION: usize = 0;
// The bit-banging uart in native simulator is set to ultra low speed for obvious reasons
#[cfg(feature = "with-trinamic")]
pub const TRINAMIC_UART_BAUD_RATE: u32 = 100;


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
/// :
///
/// Defaulting to 10,000,000 micro-seconds (or 10 secs).
/// Modify this value as per the requirements of your specific application.
pub const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;

pub const ADC_START_TIME_US: u16 = 10;
pub const ADC_VREF_DEFAULT_MV: u16 = 1650;
#[allow(unused)]
pub const ADC_VREF_DEFAULT_SAMPLE: u16 = 2048;

#[const_env::from_env("STEPPER_PLANNER_MICROSEGMENT_FREQUENCY")]
pub const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 200;
#[const_env::from_env("STEPPER_PLANNER_CLOCK_FREQUENCY")]
pub const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;

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
    pub sys_watchdog: hwa::StaticController<crate::WatchdogMutexType, device::Watchdog>,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: hwa::StaticController<crate::SerialPort1MutexType, device::SerialPort1TxDevice>,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: hwa::StaticController<crate::SerialPort2MutexType, device::SerialPort2TxDevice>,
}

pub struct SysDevices {
    #[cfg(all(feature = "with-motion", feature="executor-interrupt"))]
    pub task_stepper_core: printhor_hwa_common::NoDevice,
    #[cfg(feature = "with-ps-on")]
    pub ps_on: hwa::StaticController<crate::PSOnMutexType, device::PsOnPin>,
}

pub struct IODevices {
    /// Only single owner allowed
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: device::UartPort1RxInputStream,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_rx_stream: device::UartPort2RxInputStream,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_device: device::SDCardBlockDevice,
}

pub struct PwmDevices {
    #[cfg(feature = "with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature = "with-hot-end")]
    pub hot_end: device::HotendPeripherals,
    #[cfg(feature = "with-hot-bed")]
    pub hot_bed: device::HotbedPeripherals,
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

pub struct HWIPeripherals {
}

#[inline]
pub fn init() -> HWIPeripherals {
    HWIPeripherals{}
}

pub async fn setup(_spawner: Spawner, _p: HWIPeripherals) -> hwa::MachineContext<Controllers, SysDevices, IODevices, MotionDevices, PwmDevices> {

    #[cfg(feature = "with-motion")]
    let _ = _spawner.spawn(task_stepper_ticker());

    let _pin_state = hwa::make_static_ref!(
        "GlobalPinState",
        mocked_peripherals::PinsCell<mocked_peripherals::PinState>,
        mocked_peripherals::PinsCell::new(mocked_peripherals::PinState::new())
    );

    cfg_if::cfg_if!{
        if #[cfg(all(feature = "with-serial-port-1"))] {
            let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(_spawner.make_send()).split();
            let serial_port1_tx = hwa::make_static_controller!(
                "UartPort1Tx",
                crate::SerialPort1MutexType,
                device::SerialPort1TxDevice,
                uart_port1_tx_device
            );
            let serial_port1_rx_stream = device::UartPort1RxInputStream::new(uart_port1_rx_device);
        }
    }

    cfg_if::cfg_if!{
        if #[cfg(all(feature = "with-serial-port-2"))] {
            let (uart_port2_tx_device, uart_port2_rx_device) = device::UartPort2Device::new().split();
            let serial_port2_tx = hwa::make_static_controller!(
                "UartPort2Tx",
                crate::SerialPort2MutexType,
                device::SerialPort2TxDevice,
                uart_port2_tx_device
            );
            let serial_port2_rx_stream = device::UartPort2RxInputStream::new(uart_port2_rx_device);
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


    #[allow(unused)]
    #[cfg(feature = "with-spi")]
    let spi1_device = {
        #[link_section = "__DATA,.bss"]
        static SPI1_INST: TrackedStaticCell<ControllerMutex<device::Spi>> = TrackedStaticCell::new();
        ControllerRef::new(SPI1_INST.init(
            "SPI1",
            ControllerMutex::new(
                device::Spi::new()
            )
        ))
    };
    #[cfg(feature = "with-spi")]
    hwa::debug!("SPI done");

    #[cfg(feature = "with-sdcard")]
    let sdcard_device = {
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

    #[cfg(feature = "with-hot-end")]
    static HOT_END_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_END_THERM_PULL_UP_RESISTANCE, HOT_END_THERM_NOMINAL_RESISTANCE, HOT_END_THERM_BETA);

    #[cfg(feature = "with-hot-bed")]
    static HOT_BED_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_BED_THERM_PULL_UP_RESISTANCE, HOT_BED_THERM_NOMINAL_RESISTANCE, HOT_BED_THERM_BETA);

    #[cfg(any(feature = "with-probe", feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))]
    let pwm_any = hwa::make_static_controller!(
        "PwmController",
        crate::CriticalSectionRawMutex,
        mocked_peripherals::MockedPwm,
        mocked_peripherals::MockedPwm::new(20, _pin_state)
    );

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    let adc_hotend_hotbed = hwa::make_static_controller!(
        "AdcHotendHotbed",
        crate::AdcHotEndMutexType,
        device::AdcHotendHotbed,
        device::AdcHotendHotbed::new(0)
    );

    #[cfg(feature = "with-motion")]
    hwa::debug!("motion_planner done");

    #[cfg(feature = "with-ps-on")]
    let ps_on = hwa::make_static_controller!(
        "PSOn",
        crate::PSOnMutexType,
        device::PsOnPin,
        mocked_peripherals::MockedIOPin::new(21, _pin_state)
    );

    let sys_watchdog = hwa::make_static_controller!(
        "WatchDog",
        crate::WatchdogMutexType,
        device::Watchdog,
        device::Watchdog::new(_spawner.make_send(), WATCHDOG_TIMEOUT_US)
    );

    hwa::MachineContext {
        controllers: Controllers {
            sys_watchdog,
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
            #[cfg(feature = "with-sdcard")]
            sdcard_device,
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
            hot_end: device::HotendPeripherals {
                power_pwm: pwm_any.clone(),
                power_channel: 1,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: mocked_peripherals::MockedIOPin::new(23, _pin_state),
                thermistor_properties: &HOT_END_THERMISTOR_PROPERTIES,
            },
            #[cfg(feature = "with-hot-bed")]
            hot_bed: device::HotbedPeripherals {
                power_pwm: pwm_any.clone(),
                power_channel: 2,
                temp_adc: adc_hotend_hotbed.clone(),
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
}
