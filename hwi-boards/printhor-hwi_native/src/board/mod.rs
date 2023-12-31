///  Native board implementation. For debugging/simulation purposes
pub mod device;
pub mod io;

mod mocked_peripherals;

use embassy_executor::Spawner;

#[cfg(feature = "with-usbserial")]
use device::*;
use printhor_hwa_common::{ControllerMutex, ControllerRef};
use printhor_hwa_common::{TrackedStaticCell, MachineContext};
#[cfg(feature = "with-trinamic")]
use device::Uart4;
#[cfg(feature = "with-motion")]
use device::{MotionDevice, MotionPins};
#[cfg(feature = "with-motion")]
use crate::board::mocked_peripherals::MockedOutputPin;
#[cfg(feature = "with-motion")]
use crate::board::mocked_peripherals::MockedInputPin;
use crate::board::mocked_peripherals::MockedPwm;

pub const MACHINE_TYPE: &str = "Simulator/debugger";
pub const MACHINE_BOARD: &str = "PC";
/// ARM Cortex M0+ @64MHZ, 144kB SRAM, 512kB Program
pub const MACHINE_PROCESSOR: &str = std::env::consts::ARCH;
#[allow(unused)]
pub(crate) const PROCESSOR_SYS_CK_MHZ: u32 = 1_000_000_000;
pub const HEAP_SIZE_BYTES: usize = 1024;
pub const MAX_STATIC_MEMORY: u32 = 4096;
pub const VREF_SAMPLE: u16 = 1210u16;
#[cfg(feature = "with-uart2")]
pub(crate) const UART2_BAUD_RATE: u32 = 115200;
#[cfg(feature = "with-sdcard")]
pub const SDCARD_PARTITION: usize = 0;
#[cfg(feature = "with-trinamic")]
pub(crate) const TRINAMIC_UART_BAUD_RATE: u32 = 115200;
pub(crate) const WATCHDOG_TIMEOUT: u32 = 30_000_000;

/// Shared controllers
pub struct Controllers {
    pub sys_watchdog: ControllerRef<device::Watchdog>,
    #[cfg(feature = "with-usbserial")]
    pub usbserial_tx: device::USBSerialTxControllerRef,
    #[cfg(feature = "with-uart-port-1")]
    pub uart_port1_tx: device::UartPort1TxControllerRef,
}

pub struct IODevices {
    #[cfg(feature = "with-usbserial")]
    pub usbserial_rx_stream: device::USBSerialDeviceInputStream,
    /// Only single owner allowed
    #[cfg(feature = "with-uart-port-1")]
    pub uart_port1_rx_stream: device::UartPort1RxInputStream,
    #[cfg(feature  ="with-display")]
    pub display_device: device::DisplayDevice,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_device: device::SDCardBlockDevice,
}

pub struct PwmDevices {
    #[cfg(feature = "with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature = "with-hotend")]
    pub hotend: device::HotendPeripherals,
    #[cfg(feature = "with-hotbed")]
    pub hotbed: device::HotbedPeripherals,
    #[cfg(feature = "with-fan0")]
    pub fan0: device::Fan0Peripherals,
    #[cfg(feature = "with-fan-layer")]
    pub layer_fan: device::FanLayerPeripherals,
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
        core::ptr::read_volatile(&printhor_hwa_common::COUNTER) as u32
    }
}

#[inline]
pub fn heap_current_usage_percentage() -> f32 {
    100.0f32 * (heap_current_size() as f32) / (HEAP_SIZE_BYTES as f32)
}

pub struct HWIPeripherals {
}

#[inline]
pub fn init() -> HWIPeripherals {
    log::debug!("native init");
    HWIPeripherals{}
}

pub async fn setup(_spawner: Spawner, _p: HWIPeripherals) -> printhor_hwa_common::MachineContext<Controllers, IODevices, MotionDevices, PwmDevices> {

    #[cfg(all(feature = "with-uart-port-1"))]
    let (uart_port1_tx, uart_port1_rx_stream) = {
        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(_spawner.make_send()).split();
        static UART_PORT1_INS: TrackedStaticCell<ControllerMutex<device::UartPort1Tx>> = TrackedStaticCell::new();
        (
            ControllerRef::new(
                UART_PORT1_INS.init("UartPort1Tx", ControllerMutex::new(uart_port1_tx_device))
            ),
            crate::device::UartPort1RxInputStream::new(uart_port1_rx_device)
        )
    };

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
    let motion_devices = MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart,
        motion_pins: MotionPins {
            x_enable_pin: MockedOutputPin::new(),
            y_enable_pin: MockedOutputPin::new(),
            z_enable_pin: MockedOutputPin::new(),
            e_enable_pin: MockedOutputPin::new(),
            x_endstop_pin: MockedInputPin::new(),
            y_endstop_pin: MockedInputPin::new(),
            z_endstop_pin: MockedInputPin::new(),
            e_endstop_pin: MockedInputPin::new(),
            x_step_pin: MockedOutputPin::new(),
            y_step_pin: MockedOutputPin::new(),
            z_step_pin: MockedOutputPin::new(),
            e_step_pin: MockedOutputPin::new(),
            x_dir_pin: MockedOutputPin::new(),
            y_dir_pin: MockedOutputPin::new(),
            z_dir_pin: MockedOutputPin::new(),
            e_dir_pin: MockedOutputPin::new(),
        }
    };
    #[cfg(feature = "with-motion")]
    log::debug!("motion_driver done");

        #[cfg(feature = "with-display")]
    let display_device = mocked_peripherals::SimulatorDisplayDevice::new();

    #[cfg(any(feature = "with-hotend", feature = "with-hotbed", feature = "with-fan0", feature = "with-fan-layer", feature = "with-laser"))]
    let pwm_fan0_fan1_hotend_hotbed_laser = {
        let pwm_fan0_fan1_hotend_hotbed_laser = mocked_peripherals::MockedPwm::new();
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmLayerFan>> = TrackedStaticCell::new();
        ControllerRef::new(PWM_INST.init(
            "PwmFanFan0HotendHotbed",
            ControllerMutex::new(pwm_fan0_fan1_hotend_hotbed_laser)
        ))
    };

    #[cfg(any(feature = "with-laser"))]
    let (pwm_laser, pwm_laser_channel) = {
        static PWM_LASER_INST: TrackedStaticCell<ControllerMutex<device::PwmLaser>> = TrackedStaticCell::new();
        (ControllerRef::new(PWM_LASER_INST.init(
                "LaserPwmController",
                ControllerMutex::new(
                    MockedPwm::new()
                )
            )),
            0
        )
    };

    #[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
    let adc_hotend_hotbed = {
        let adc_hotend_hotbed = device::AdcHotendHotbed::new(0);
        static ADC_INST: TrackedStaticCell<ControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();

        ControllerRef::new(ADC_INST.init(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed)
        ))
    };

    #[cfg(feature = "with-probe")]
    let probe_device = {
        let probe_pwm = device::PwmServo::new();
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmServo>> = TrackedStaticCell::new();

        crate::device::ProbePeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init("ProbePwm", ControllerMutex::new(probe_pwm))),
            power_channel: 0,
        }
    };

    #[cfg(feature = "with-motion")]
    log::debug!("motion_planner done");

    static WD: TrackedStaticCell<ControllerMutex<device::Watchdog>> = TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init("watchdog", ControllerMutex::new(device::Watchdog::new(_spawner.make_send(), WATCHDOG_TIMEOUT))));

    MachineContext {
        controllers: Controllers {
            sys_watchdog,
            #[cfg(feature = "with-usbserial")]
            usbserial_tx,
            #[cfg(feature = "with-uart-port-1")]
            uart_port1_tx,
        },
        devices: IODevices {
            #[cfg(feature = "with-usbserial")]
            usbserial_rx_stream,
            #[cfg(feature = "with-uart-port-1")]
            uart_port1_rx_stream,
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
            probe: probe_device,
            #[cfg(feature = "with-hotend")]
            hotend: device::HotendPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed_laser.clone(),
                power_channel: 0,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: MockedInputPin::new(),
            },
            #[cfg(feature = "with-hotbed")]
            hotbed: device::HotbedPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed_laser.clone(),
                power_channel: 0,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: MockedInputPin::new(),
            },
            #[cfg(feature = "with-fan0")]
            fan0: Fan0Peripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_fan0_channel,
            },
            #[cfg(feature = "with-fan-layer")]
            layer_fan: device::FanLayerPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed_laser.clone(),
                power_channel: 0u8,
            },
            #[cfg(feature = "with-laser")]
            laser: device::LaserPeripherals {
                power_pwm: pwm_laser.clone(),
                power_channel: pwm_laser_channel,
            },
        }
    }

}

#[allow(unused)]
pub mod consts {
    /// 50ms to enqueue ~28 motion gcodes at 115200 bps
    pub(crate) const LINGER_MS: u64 = 10000;
}