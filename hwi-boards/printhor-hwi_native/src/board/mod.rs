///  Native board implementation. For debugging/simulation purposes
pub mod device;
pub mod io;
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

use embassy_executor::Spawner;
use printhor_hwa_common::{ControllerMutex, ControllerRef, TrackedStaticCell, MachineContext};

#[allow(unused)]
use crate::board::mocked_peripherals::MockedIOPin;

pub const MACHINE_TYPE: &str = "Simulator/debugger";
pub const MACHINE_BOARD: &str = "PC";
/// ARM Cortex M0+ @64MHZ, 144kB SRAM, 512kB Program
pub const MACHINE_PROCESSOR: &str = std::env::consts::ARCH;
#[allow(unused)]
pub(crate) const PROCESSOR_SYS_CK_MHZ: u32 = 1_000_000_000;
pub const HEAP_SIZE_BYTES: usize = 1024;
pub const MAX_STATIC_MEMORY: u32 = 8192;
pub const VREF_SAMPLE: u16 = 1210u16;
#[cfg(feature = "with-sdcard")]
pub const SDCARD_PARTITION: usize = 0;
// The bit-banging uart in native simulator is set to ultra low speed for obvious reasons
#[cfg(feature = "with-trinamic")]
pub(crate) const TRINAMIC_UART_BAUD_RATE: u32 = 8;
pub(crate) const WATCHDOG_TIMEOUT: u32 = 30_000_000;

pub const ADC_START_TIME_US: u16 = 10;
pub const ADC_VREF_DEFAULT_MV: u16 = 1650;
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
    pub sys_watchdog: ControllerRef<device::Watchdog>,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: device::UartPort1TxControllerRef,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: device::UartPort2TxControllerRef,
}

pub struct SysDevices {
    #[cfg(feature = "with-motion")]
    pub task_stepper_core: printhor_hwa_common::NoDevice,
    #[cfg(feature = "with-ps-on")]
    pub ps_on: device::PsOnRef,
}

pub struct IODevices {
    /// Only single owner allowed
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: device::UartPort1RxInputStream,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_rx_stream: device::UartPort2RxInputStream,
    #[cfg(feature  ="with-display")]
    pub display_device: device::DisplayDevice,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_device: device::SDCardBlockDevice,
}

pub struct PwmDevices {
    #[cfg(feature = "with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature = "with-hot-end")]
    pub hotend: device::HotendPeripherals,
    #[cfg(feature = "with-hot-bed")]
    pub hotbed: device::HotbedPeripherals,
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
    unsafe {
        core::ptr::read_volatile(core::ptr::addr_of!(printhor_hwa_common::COUNTER)) as u32
    }
}

pub struct HWIPeripherals {
}

#[inline]
pub fn init() -> HWIPeripherals {
    log::debug!("native init");
    HWIPeripherals{}
}

pub async fn setup(_spawner: Spawner, _p: HWIPeripherals) -> MachineContext<Controllers, SysDevices, IODevices, MotionDevices, PwmDevices> {

    let _pin_state = mocked_peripherals::init_pin_state();

    cfg_if::cfg_if!{
        if #[cfg(all(feature = "with-serial-port-1"))] {
            let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(_spawner.make_send()).split();
            static UART_PORT1_INS: TrackedStaticCell<ControllerMutex<device::UartPort1Tx>> = TrackedStaticCell::new();
            let serial_port1_tx = ControllerRef::new(
                UART_PORT1_INS.init("UartPort1Tx", ControllerMutex::new(uart_port1_tx_device))
            );
            let serial_port1_rx_stream = device::UartPort1RxInputStream::new(uart_port1_rx_device);
        }
    }

    cfg_if::cfg_if!{
        if #[cfg(all(feature = "with-serial-port-2"))] {
            let (uart_port2_tx_device, uart_port2_rx_device) = device::UartPort2Device::new().split();
            static UART_PORT2_INS: TrackedStaticCell<ControllerMutex<device::UartPort2Tx>> = TrackedStaticCell::new();
            let serial_port2_tx = ControllerRef::new(
                UART_PORT2_INS.init("UartPort1Tx", ControllerMutex::new(uart_port2_tx_device))
            );
            let serial_port2_rx_stream = device::UartPort2RxInputStream::new(uart_port2_rx_device);
        }
    }

    #[cfg(all(feature = "with-trinamic"))]
        let trinamic_uart = {
        device::TrinamicUart::new(
            TRINAMIC_UART_BAUD_RATE,
            MockedIOPin::new(0, _pin_state),
            MockedIOPin::new(1, _pin_state),
            MockedIOPin::new(2, _pin_state),
            MockedIOPin::new(3, _pin_state),
        )
    };

    #[cfg(all(feature = "with-trinamic"))]
    {
        static EXECUTOR: printhor_hwa_common::TrackedStaticCell<embassy_executor::Executor> = printhor_hwa_common::TrackedStaticCell::new();

        let builder = std::thread::Builder::new()
            .name("trinamic-uart-driver-sym".into());
        let _ = builder.spawn(move || {
            let executor: &'static mut embassy_executor::Executor = EXECUTOR.init("X", embassy_executor::Executor::new());
            executor.run(move |s| {
                s.spawn(
                    device::trinamic_driver_simulator(
                        device::MockedTrinamicDriver::new(
                            MockedIOPin::new(0, _pin_state),
                            MockedIOPin::new(1, _pin_state),
                            MockedIOPin::new(2, _pin_state),
                            MockedIOPin::new(3, _pin_state),
                        )
                    )
                ).unwrap();
            });
        });
    }


    #[allow(unused)]
    #[cfg(feature = "with-spi")]
    let spi1_device = {
        static SPI1_INST: TrackedStaticCell<ControllerMutex<device::Spi>> = TrackedStaticCell::new();
        ControllerRef::new(SPI1_INST.init(
            "SPI1",
            ControllerMutex::new(
                device::Spi::new()
            )
        ))
    };
    #[cfg(feature = "with-spi")]
    log::debug!("SPI done");

    #[cfg(feature = "with-sdcard")]
    let sdcard_device = {
        device::SDCardBlockDevice::new("data/sdcard.img", false).unwrap()
    };

    #[cfg(feature = "with-motion")]
    let motion_devices = device::MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart,
        motion_pins: device::MotionPins {
            x_enable_pin: MockedIOPin::new(4, _pin_state),
            y_enable_pin: MockedIOPin::new(5, _pin_state),
            z_enable_pin: MockedIOPin::new(6, _pin_state),
            e_enable_pin: MockedIOPin::new(7, _pin_state),
            x_endstop_pin: MockedIOPin::new(8, _pin_state),
            y_endstop_pin: MockedIOPin::new(9, _pin_state),
            z_endstop_pin: MockedIOPin::new(10, _pin_state),
            e_endstop_pin: MockedIOPin::new(11, _pin_state),
            x_step_pin: MockedIOPin::new(12, _pin_state),
            y_step_pin: MockedIOPin::new(13, _pin_state),
            z_step_pin: MockedIOPin::new(14, _pin_state),
            e_step_pin: MockedIOPin::new(15, _pin_state),
            x_dir_pin: MockedIOPin::new(16, _pin_state),
            y_dir_pin: MockedIOPin::new(17, _pin_state),
            z_dir_pin: MockedIOPin::new(18, _pin_state),
            e_dir_pin: MockedIOPin::new(19, _pin_state),
        }
    };
    #[cfg(feature = "with-motion")]
    log::debug!("motion_driver done");

    #[cfg(feature = "with-display")]
    let display_device = mocked_peripherals::SimulatorDisplayDevice::new();

    #[cfg(feature = "with-hot-end")]
    static HOT_END_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_END_THERM_PULL_UP_RESISTANCE, HOT_END_THERM_NOMINAL_RESISTANCE, HOT_END_THERM_BETA);

    #[cfg(feature = "with-hot-bed")]
    static HOT_BED_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_BED_THERM_PULL_UP_RESISTANCE, HOT_BED_THERM_NOMINAL_RESISTANCE, HOT_BED_THERM_BETA);

    #[cfg(any(feature = "with-probe", feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))]
    let pwm_any = {
        let pwm_any = mocked_peripherals::MockedPwm::new(20, _pin_state);
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmAny>> = TrackedStaticCell::new();
        ControllerRef::new(PWM_INST.init(
            "PwmAny",
            ControllerMutex::new(pwm_any)
        ))
    };

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    let adc_hotend_hotbed = {
        let adc_hotend_hotbed = device::AdcHotendHotbed::new(0);
        static ADC_INST: TrackedStaticCell<ControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();

        ControllerRef::new(ADC_INST.init(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed)
        ))
    };

    #[cfg(feature = "with-motion")]
    log::debug!("motion_planner done");

    #[cfg(feature = "with-ps-on")]
    let ps_on = {
        static PS_ON: TrackedStaticCell<ControllerMutex<device::PsOnPin>> = TrackedStaticCell::new();
        ControllerRef::new(
            PS_ON.init("", ControllerMutex::new(MockedIOPin::new(21, _pin_state)))
        )
    };

    static WD: TrackedStaticCell<ControllerMutex<device::Watchdog>> = TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init("watchdog", ControllerMutex::new(device::Watchdog::new(_spawner.make_send(), WATCHDOG_TIMEOUT))));

    MachineContext {
        controllers: Controllers {
            sys_watchdog,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_tx,
        },
        sys_devices: SysDevices {
            task_stepper_core: printhor_hwa_common::NoDevice::new(),
            #[cfg(feature = "with-ps-on")]
            ps_on
        },
        io_devices: IODevices {
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_rx_stream,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_rx_stream,
            #[cfg(feature = "with-display")]
            display_device,
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
            hotend: device::HotendPeripherals {
                power_pwm: pwm_any.clone(),
                power_channel: 1,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: MockedIOPin::new(23, _pin_state),
                thermistor_properties: &HOT_END_THERMISTOR_PROPERTIES,
            },
            #[cfg(feature = "with-hot-bed")]
            hotbed: device::HotbedPeripherals {
                power_pwm: pwm_any.clone(),
                power_channel: 2,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: MockedIOPin::new(24, _pin_state),
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
