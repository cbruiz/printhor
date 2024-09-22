/// https://github.com/raspberrypi/pico-sdk/blob/1.5.0/src/rp2040/hardware_regs/rp2040.svd
pub mod device;
pub mod io;

use alloc_cortex_m::CortexMHeap;
use embassy_executor::Spawner;
use embassy_rp::config::Config;

#[cfg(any(feature = "with-serial-port-1", feature = "with-serial-port-2", feature="with-trinamic"))]
use embassy_rp::uart::{DataBits, Parity, StopBits};
#[allow(unused)]
use printhor_hwa_common::{ControllerMutex, ControllerRef, ControllerMutexType};
use printhor_hwa_common::{TrackedStaticCell, MachineContext, StandardControllerMutex};
#[cfg(feature = "with-motion")]
use device::{MotionDevice, MotionPins};

#[global_allocator]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "RP2040";

cfg_if::cfg_if! {
    if #[cfg(feature="tst-rp2040")] {
        /// ARM Cortex M0+ @133MHZ, 264kB SRAM, 2048kB Program
        pub const MACHINE_PROCESSOR: &str = "RP2040";
        pub const MACHINE_BOARD: &str = "TBD";
        #[allow(unused)]
        pub const PROCESSOR_SYS_CK_MHZ: &str = "133_000_000";
        pub const ADC_START_TIME_US: u16 = 10;
        pub const ADC_VREF_DEFAULT_MV: u16 = 1210;
    }
    else {
        compile_error!("No board specified");
    }
}

pub const HEAP_SIZE_BYTES: usize = 1024;
pub const MAX_STATIC_MEMORY: usize = 4096;
#[cfg(feature = "with-sdcard")]
pub const SDCARD_PARTITION: usize = 0;
pub(crate) const WATCHDOG_TIMEOUT_MS: u32 = 5_000;
#[cfg(feature = "with-spi")]
pub(crate) const SPI_FREQUENCY_HZ: u32 = 2_000_000;

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
    pub sys_watchdog: printhor_hwa_common::StandardControllerRef<device::Watchdog>,
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: device::USBSerialTxControllerRef,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: device::UartPort1TxControllerRef,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_tx: device::UartPort2TxControllerRef,
}

pub struct SysDevices {
    pub task_stepper_core: device::TaskStepperCore,
    #[cfg(feature = "with-ps-on")]
    pub ps_on: device::PsOnRef,
}

pub struct IODevices {
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_rx_stream: device::USBSerialDeviceInputStream,
    /// Only single owner allowed
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: device::UartPort1RxInputStream,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_rx_stream: device::UartPort2RxInputStream,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_device: device::SpiCardDeviceRef,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_cs_pin: device::SpiCardCSPin,
}

pub struct PwmDevices {
    #[cfg(feature="with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature="with-hot-end")]
    pub hotend: device::HotendPeripherals,
    #[cfg(feature="with-hot-bed")]
    pub hotbed: device::HotbedPeripherals,
    #[cfg(feature="with-laser")]
    pub laser: device::LaserPeripherals,
    #[cfg(feature="with-fan-layer")]
    pub fan_layer: device::FanLayerPeripherals,
    #[cfg(feature="with-fan-extra-1")]
    pub fan_extra1: device::FanExtra1Peripherals,
}

pub struct MotionDevices {
    #[cfg(feature = "with-motion")]
    pub motion_devices: device::MotionDevice,
}

pub fn heap_current_size() -> u32 {
    HEAP.used() as u32
}

#[inline]
pub fn stack_reservation_current_size() -> u32 {
    unsafe {
        core::ptr::read_volatile(core::ptr::addr_of!(printhor_hwa_common::COUNTER)) as u32
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        embassy_rp::bind_interrupts!(struct UsbIrqs {
            USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
        });
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-uart-buffered")] {
                embassy_rp::bind_interrupts!(struct UartPort1Irqs {
                    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<embassy_rp::peripherals::UART0>;
                });
            }
            else {
                embassy_rp::bind_interrupts!(struct UartPort1Irqs {
                    UART0_IRQ => embassy_rp::uart::InterruptHandler<embassy_rp::peripherals::UART0>;
                });
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-uart-buffered")] {
                embassy_rp::bind_interrupts!(struct UartPort2Irqs {
                    UART1_IRQ => embassy_rp::uart::BufferedInterruptHandler<embassy_rp::peripherals::UART1>;
                });
            }
            else {
                embassy_rp::bind_interrupts!(struct UartPort2Irqs {
                    UART1_IRQ => embassy_rp::uart::InterruptHandler<embassy_rp::peripherals::UART1>;
                });
            }
        }
    }
}

#[inline]
pub(crate) fn init_heap() -> () {
    use core::mem::MaybeUninit;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE_BYTES] = [MaybeUninit::uninit(); HEAP_SIZE_BYTES];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE_BYTES) }
}

#[inline]
pub fn init() -> embassy_rp::Peripherals {
    init_heap();
    #[cfg(feature="tst-rp2040")]
        let config = {
        let config = Config::default();
        config
    };
    embassy_rp::init(config)
}

pub async fn setup(_spawner: Spawner, p: embassy_rp::Peripherals) -> printhor_hwa_common::MachineContext<Controllers, SysDevices, IODevices, MotionDevices, PwmDevices> {

    #[cfg(feature = "with-serial-usb")]
    let (serial_usb_tx, serial_usb_rx_stream) = {
        defmt::info!("Creating USB Driver");
        let driver = embassy_rp::usb::Driver::new(p.USB, UsbIrqs);

        let mut usb_serial_device = io::usbserial::USBSerialDevice::new(driver);
        usb_serial_device.spawn(_spawner);
        let (usb_serial_rx_device, sender) = usb_serial_device.split();
        #[link_section = ".bss"]
        static USB_INST: TrackedStaticCell<StandardControllerMutex<device::USBSerialDeviceSender>> = TrackedStaticCell::new();
        let serial_usb_tx = ControllerRef::new(
            USB_INST.init::<{crate::MAX_STATIC_MEMORY}>("USBSerialTxController", ControllerMutex::new(sender))
        );
        (serial_usb_tx, device::USBSerialDeviceInputStream::new(usb_serial_rx_device))
    };

    #[cfg(feature = "with-serial-port-1")]
    let (serial_port1_tx, serial_port1_rx_stream) = {
        let mut cfg = embassy_rp::uart::Config::default();
        cfg.baudrate = crate::UART_PORT1_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;

        #[link_section = ".bss"]
        static RXB: TrackedStaticCell<[u8; 32]> =  TrackedStaticCell::new();
        let rxb = RXB.init::<{crate::MAX_STATIC_MEMORY}>("RXBuffer", [0u8; 32]);
        #[link_section = ".bss"]
        static TXB: TrackedStaticCell<[u8; 32]> =  TrackedStaticCell::new();
        let txb = TXB.init::<{crate::MAX_STATIC_MEMORY}>("TXBuffer", [0u8; 32]);

        let (uart_port1_rx_device, uart_port1_tx_device) = device::UartPort1Device::new_blocking(
            p.UART0, p.PIN_0, p.PIN_1, cfg
        ).into_buffered(UartPort1Irqs, txb, rxb).split();

        #[link_section = ".bss"]
        static UART_PORT1_INST: TrackedStaticCell<StandardControllerMutex<device::UartPort1TxDevice>> = TrackedStaticCell::new();
        let serial_port1_tx = ControllerRef::new(
            UART_PORT1_INST.init::<{crate::MAX_STATIC_MEMORY}>("UartPort1", ControllerMutex::new(uart_port1_tx_device))
        );
        (serial_port1_tx, device::UartPort1RxInputStream::new(uart_port1_rx_device))
    };

    #[cfg(feature = "with-serial-port-2")]
    let (serial_port2_tx, serial_port2_rx_stream) = {
        let mut cfg = embassy_rp::uart::Config::default();
        cfg.baudrate = crate::UART_PORT2_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;

        #[link_section = ".bss"]
        static RXB: TrackedStaticCell<[u8; 32]> =  TrackedStaticCell::new();
        let rxb = RXB.init::<{crate::MAX_STATIC_MEMORY}>("RXBuffer", [0u8; 32]);
        #[link_section = ".bss"]
        static TXB: TrackedStaticCell<[u8; 32]> =  TrackedStaticCell::new();
        let txb = TXB.init::<{crate::MAX_STATIC_MEMORY}>("TXBuffer", [0u8; 32]);

        let (uart_port2_rx_device, uart_port2_tx_device) = device::UartPort2Device::new_blocking(
            p.UART1, p.PIN_4, p.PIN_5, cfg
        ).into_buffered(UartPort2Irqs, txb, rxb).split();

        #[link_section = ".bss"]
        static UART_PORT2_INST: TrackedStaticCell<ControllerMutex<device::UartPort2TxDevice>> = TrackedStaticCell::new();
        let serial_port2_tx = ControllerRef::new(
            UART_PORT2_INST.init::<{crate::MAX_STATIC_MEMORY}>("UartPort1", ControllerMutex::new(uart_port2_tx_device))
        );
        (serial_port2_tx, device::UartPort2RxInputStream::new(uart_port2_rx_device))
    };

    #[cfg(feature = "with-spi")]
    {
        compile_error!("Not yet implemented")
    }

    #[cfg(feature = "with-sdcard")]
    {
        compile_error!("Not yet implemented")
    }

    #[cfg(feature = "with-motion")]
    let motion_devices = MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart,
        motion_pins: MotionPins {
            all_enable_pin: embassy_rp::gpio::Output::new(p.PIN_2, embassy_rp::gpio::Level::Low),
            x_endstop_pin: embassy_rp::gpio::Input::new(p.PIN_3, embassy_rp::gpio::Pull::Down),
            y_endstop_pin: embassy_rp::gpio::Input::new(p.PIN_6, embassy_rp::gpio::Pull::Down),
            z_endstop_pin: embassy_rp::gpio::Input::new(p.PIN_7, embassy_rp::gpio::Pull::Down),
            x_step_pin: embassy_rp::gpio::Output::new(p.PIN_8, embassy_rp::gpio::Level::Low),
            y_step_pin: embassy_rp::gpio::Output::new(p.PIN_9, embassy_rp::gpio::Level::Low),
            z_step_pin: embassy_rp::gpio::Output::new(p.PIN_10, embassy_rp::gpio::Level::Low),
            x_dir_pin: embassy_rp::gpio::Output::new(p.PIN_11, embassy_rp::gpio::Level::Low),
            y_dir_pin: embassy_rp::gpio::Output::new(p.PIN_12, embassy_rp::gpio::Level::Low),
            z_dir_pin: embassy_rp::gpio::Output::new(p.PIN_13, embassy_rp::gpio::Level::Low),
        }
    };

    #[cfg(feature = "with-probe")]
    {
        compile_error!("Not implemented")
    }

    #[cfg(feature = "with-hot-end")]
    static HOT_END_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_END_THERM_PULL_UP_RESISTANCE, HOT_END_THERM_NOMINAL_RESISTANCE, HOT_END_THERM_BETA);

    #[cfg(feature = "with-hot-bed")]
    static HOT_BED_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_BED_THERM_PULL_UP_RESISTANCE, HOT_BED_THERM_NOMINAL_RESISTANCE, HOT_BED_THERM_BETA);

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    {
        compile_error!("Not implemented")
    }

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    {
        compile_error!("Not yet implemented")
    }

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer"))]
    {
        compile_error!("Not yet implemented")
    }
    #[cfg(feature = "with-hot-end")]
    {
        compile_error!("Not yet implemented")
    }
    #[cfg(feature = "with-hot-end")]
    {
        compile_error!("Not yet implemented")
    }

    #[cfg(feature = "with-hot-bed")]
    {
        compile_error!("Not yet implemented")
    }

    #[cfg(feature = "with-laser")]
    {
        compile_error!("Not yet implemented")
    }

    #[cfg(feature = "with-fan-layer")]
    {
        compile_error!("Not yet implemented")
    }

    #[cfg(feature = "with-ps-on")]
        let ps_on = {
        #[link_section = ".bss"]
        static PS_ON: TrackedStaticCell<ControllerMutex<device::PsOnPin>> = TrackedStaticCell::new();
        ControllerRef::new(
            PS_ON.init::<{crate::MAX_STATIC_MEMORY}>("", ControllerMutex::new(
                embassy_rp::gpio::Output::new(p.PIN_14, embassy_rp::gpio::Level::Low)
            ))
        )
    };

    let watchdog = device::Watchdog::new(p.WATCHDOG);
    #[link_section = ".bss"]
    static WD: TrackedStaticCell<printhor_hwa_common::StandardControllerMutex<device::Watchdog>> = TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init::<{crate::MAX_STATIC_MEMORY}>("watchdog", ControllerMutex::new(watchdog)));

    MachineContext {
        controllers: Controllers {
            sys_watchdog,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_tx,
        },
        sys_devices: SysDevices {
            task_stepper_core: p.CORE1,
            #[cfg(feature = "with-ps-on")]
            ps_on,
        },
        io_devices: IODevices {
            #[cfg(feature = "with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_rx_stream,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_rx_stream,
            #[cfg(feature = "with-sdcard")]
            sdcard_device,
            #[cfg(feature = "with-sdcard")]
            sdcard_cs_pin,
        },
        motion: MotionDevices {
            #[cfg(feature = "with-motion")]
            motion_devices
        },
        pwm: PwmDevices {
            #[cfg(feature = "with-probe")]
            probe: probe_device,
            #[cfg(feature = "with-hot-end")]
            hotend: hotend_device,
            #[cfg(feature = "with-hot-bed")]
            hotbed: hotbed_device,
            #[cfg(feature = "with-laser")]
            laser: laser_device,
            #[cfg(feature = "with-fan-layer")]
            fan_layer: fan_layer_device,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1: fan_extra1_device,
        }
    }
}
