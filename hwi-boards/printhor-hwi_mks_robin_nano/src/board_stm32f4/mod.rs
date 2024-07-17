/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32F407VE.json
///
pub mod device;
pub mod io;
#[cfg(feature = "with-trinamic")]
pub mod comm;

use alloc_cortex_m::CortexMHeap;
use embassy_executor::Spawner;
use embassy_stm32::Config;
#[cfg(any(feature = "with-serial-usb", feature = "with-serial-port-1", feature="with-trinamic"))]
use embassy_stm32::{bind_interrupts};
#[cfg(any(feature = "with-serial-port-1"))]
use embassy_stm32::usart;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Level, Output, Speed, Pull};
#[allow(unused)]
use embassy_sync::mutex::Mutex;
#[cfg(any(feature = "with-serial-port-1"))]
use embassy_stm32::usart::{DataBits, Parity, StopBits};
#[cfg(feature = "with-serial-usb")]
use embassy_stm32::usb_otg;
#[cfg(feature = "with-spi")]
use embassy_stm32::spi;
#[allow(unused)]
use printhor_hwa_common::{ControllerMutex, ControllerRef, ControllerMutexType};
use printhor_hwa_common::{TrackedStaticCell, MachineContext};
use embassy_stm32::rcc::*;

#[cfg(feature = "with-motion")]
use device::{MotionDevice, MotionPins};

#[global_allocator]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "MKS";
pub const MACHINE_BOARD: &str = "SKR_ROBIN_NANO_V3.1";
/// ARM Cortex M4F @168MHZ, 192kB SRAM, 512kB Program
pub const MACHINE_PROCESSOR: &str = "STM32F407VET6";
#[allow(unused)]
pub(crate) const PROCESSOR_SYS_CK_MHZ: u32 = 168_000_000;
pub const HEAP_SIZE_BYTES: usize = 1024;
pub const MAX_STATIC_MEMORY: usize = 4096;
pub const VREF_SAMPLE: u16 = 1210u16;
#[cfg(feature = "with-sdcard")]
pub const SDCARD_PARTITION: usize = 0;
pub(crate) const WATCHDOG_TIMEOUT: u32 = 30_000_000;
#[cfg(feature = "with-spi")]
pub(crate) const SPI_FREQUENCY_HZ: u32 = 2_000_000;
#[cfg(feature = "with-trinamic")]
pub(crate) const TRINAMIC_UART_BAUD_RATE: u32 = 9600;

// https://www.st.com/resource/en/datasheet/stm32f405og.pdf
pub const ADC_START_TIME_US: u16 = 10;
// https://www.st.com/resource/en/datasheet/stm32f405og.pdf
pub const ADC_VREF_DEFAULT_MV: u16 = 1210;

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
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: device::USBSerialTxControllerRef,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx: device::UartPort1TxControllerRef,
}

pub struct SysDevices {
    #[cfg(feature = "with-motion")]
    pub task_stepper_core: printhor_hwa_common::NoDevice,
    #[cfg(feature = "with-ps-on")]
    pub ps_on: device::PsOnRef,
}

pub struct IODevices {
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_rx_stream: device::USBSerialDeviceInputStream,
    /// Only single owner allowed
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: device::UartPort1RxInputStream,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_device: device::SpiCardDeviceRef,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_cs_pin: device::SpiCardCSPin,
}

pub struct PwmDevices {
    #[cfg(feature = "with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer: device::FanLayerPeripherals,
    #[cfg(feature = "with-hot-end")]
    pub hotend: device::HotendPeripherals,
    #[cfg(feature = "with-hot-bed")]
    pub hotbed: device::HotbedPeripherals,
    #[cfg(feature = "with-laser")]
    pub laser: device::LaserPeripherals,
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

#[cfg(feature = "with-serial-usb")]
bind_interrupts!(struct UsbIrqs {
    OTG_FS => usb_otg::InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
});
#[cfg(feature = "with-serial-port-1")]
bind_interrupts!(struct UartPort1Irqs {
    USART1 => usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});

#[inline]
pub(crate) fn init_heap() -> () {
    use core::mem::MaybeUninit;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE_BYTES] = [MaybeUninit::uninit(); HEAP_SIZE_BYTES];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE_BYTES) }
}

#[inline]
pub fn init() -> embassy_stm32::Peripherals {
    init_heap();
    crate::info!("Initializing...");
    let mut config = Config::default();
    config.rcc.hse = Some(Hse {
        freq: embassy_stm32::time::Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL168,
        divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
        divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    embassy_stm32::init(config)
}

pub async fn setup(_spawner: Spawner, p: embassy_stm32::Peripherals) -> printhor_hwa_common::MachineContext<Controllers, SysDevices, IODevices, MotionDevices, PwmDevices> {

    #[cfg(feature = "with-serial-usb")]
    let (serial_usb_tx, serial_usb_rx_stream) = {
        static EP_OUT_BUFFER_INST: TrackedStaticCell<[u8; 256]> =  TrackedStaticCell::new();
        let ep_out_buffer = EP_OUT_BUFFER_INST.init::<{crate::MAX_STATIC_MEMORY}>("USBEPBuffer", [0u8; 256]);
        defmt::info!("Creating USB Driver");
        let mut config = embassy_stm32::usb_otg::Config::default();
        config.vbus_detection = false;

        // Maybe OTG_FS is not the right peripheral...
        // USB_OTG_FS is DM=PB14, DP=PB15
        let driver = usb_otg::Driver::new_fs(p.USB_OTG_FS, UsbIrqs, p.PA12, p.PA11,  ep_out_buffer, config);
        let mut usb_serial_device = device::USBSerialDevice::new(driver);
        usb_serial_device.spawn(_spawner);
        let (usb_serial_rx_device, sender) = usb_serial_device.split();
        static USB_INST: TrackedStaticCell<Mutex<ControllerMutexType, device::USBSerialDeviceSender>> = TrackedStaticCell::new();
        let serial_usb_tx = ControllerRef::new(
            USB_INST.init::<{crate::MAX_STATIC_MEMORY}>("USBSerialTxController", Mutex::<ControllerMutexType, _>::new(sender))
        );
        (serial_usb_tx, device::USBSerialDeviceInputStream::new(usb_serial_rx_device))
    };

    #[cfg(feature = "with-serial-port-1")]
    let (serial_port1_tx, serial_port1_rx_stream) = {
        let mut cfg = usart::Config::default();
        cfg.baudrate = crate::UART_PORT1_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;
        cfg.detect_previous_overrun = false;

        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(p.USART1,
                                                            p.PA10, p.PA9,
                                                            UartPort1Irqs,
                                                            p.DMA2_CH7, p.DMA2_CH5,
                                                            cfg).expect("Ready").split();

        static UART_PORT1_INST: TrackedStaticCell<ControllerMutex<device::UartPort1TxDevice>> = TrackedStaticCell::new();
        let serial_port1_tx = ControllerRef::new(
            UART_PORT1_INST.init::<{crate::MAX_STATIC_MEMORY}>("UartPort1", Mutex::<ControllerMutexType, _>::new(uart_port1_tx_device))
        );
        (serial_port1_tx, device::UartPort1RxInputStream::new(uart_port1_rx_device))
    };

    #[cfg(feature = "with-trinamic")]
    let trinamic_uart = {
        // TODO: WorkInProgress Trinamic UART (when needed) requires a software usart implementation because of the wiring
        device::TrinamicUart::new(TRINAMIC_UART_BAUD_RATE, p.PD5, p.PD1, p.PD4, p.PD9)
    };

    #[cfg(feature = "with-spi")]
    let spi1_device = {
        let mut cfg = spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
        static SPI1_INST: TrackedStaticCell<ControllerMutex<device::Spi1>> = TrackedStaticCell::new();
        ControllerRef::new(SPI1_INST.init::<{crate::MAX_STATIC_MEMORY}>(
            "SPI1",
            ControllerMutex::new(
                device::Spi1::new(p.SPI3, p.PC10, p.PC12, p.PC11,
                                  p.DMA1_CH5, p.DMA1_CH0, cfg
                )
            )
        ))
    };
    #[cfg(feature = "with-spi")]
    defmt::info!("SPI done");

    #[cfg(feature = "with-sdcard")]
    let (sdcard_device, sdcard_cs_pin) = {
        (spi1_device.clone(), Output::new(p.PC9, Level::High, Speed::VeryHigh))
    };
    #[cfg(feature = "with-sdcard")]
    defmt::info!("card_controller done");

    #[cfg(feature = "with-motion")]
    let motion_devices = MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart,
        motion_pins: MotionPins {
            x_enable_pin: Output::new(p.PE4, Level::Low, Speed::VeryHigh),
            y_enable_pin: Output::new(p.PE1, Level::Low, Speed::VeryHigh),
            z_enable_pin: Output::new(p.PB8, Level::Low, Speed::VeryHigh),
            e_enable_pin: Output::new(p.PB3, Level::Low, Speed::VeryHigh),
            x_endstop_pin: Input::new(p.PA15, Pull::Down),
            y_endstop_pin: Input::new(p.PD2, Pull::Down),
            z_endstop_pin: Input::new(p.PC8, Pull::Down),
            e_endstop_pin: Input::new(p.PC4, Pull::Down),
            x_step_pin: Output::new(p.PE3, Level::Low, Speed::VeryHigh),
            y_step_pin: Output::new(p.PE0, Level::Low, Speed::VeryHigh),
            z_step_pin: Output::new(p.PB5, Level::Low, Speed::VeryHigh),
            e_step_pin: Output::new(p.PD6, Level::Low, Speed::VeryHigh),
            x_dir_pin: Output::new(p.PE2, Level::Low, Speed::VeryHigh),
            y_dir_pin: Output::new(p.PB9, Level::Low, Speed::VeryHigh),
            z_dir_pin: Output::new(p.PB4, Level::Low, Speed::VeryHigh),
            e_dir_pin: Output::new(p.PD3, Level::Low, Speed::VeryHigh),
        }
    };
    #[cfg(feature = "with-motion")]
    defmt::info!("motion_driver done");

    #[cfg(feature = "with-probe")]
        let probe_device = {
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmServo>> = TrackedStaticCell::new();
        crate::device::ProbePeripherals {
            power_pwm: ControllerRef::new(
                PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>("PwmServo",
                              ControllerMutex::new(
                                  device::PwmServo::new(
                                      p.TIM1,
                                      Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PA8, embassy_stm32::gpio::OutputType::PushPull)),
                                      None,
                                      None,
                                      None,
                                      embassy_stm32::time::hz(50),
                                      embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
                                  )
                              ),
                )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
        }
    };

    #[cfg(feature = "with-fan-layer")]
        let fan_layer_device = {
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmFanLayer>> = TrackedStaticCell::new();

        let pwm_fan1 = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            p.TIM3,
            None,
            None,
            None,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PB1, embassy_stm32::gpio::OutputType::PushPull)),
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        crate::device::FanLayerPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>(
                "FanLayerControler",
                ControllerMutex::new(pwm_fan1)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
        }
    };

    #[cfg(feature = "with-hot-end")]
    static HOT_END_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_END_THERM_PULL_UP_RESISTANCE, HOT_END_THERM_NOMINAL_RESISTANCE, HOT_END_THERM_BETA);

    #[cfg(feature = "with-hot-bed")]
    static HOT_BED_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_BED_THERM_PULL_UP_RESISTANCE, HOT_BED_THERM_NOMINAL_RESISTANCE, HOT_BED_THERM_BETA);

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    let adc = {
        let mut adc_hotend_hotbed = device::AdcHotendHotbed::new(p.ADC1, &mut embassy_time::Delay);
        adc_hotend_hotbed.set_sample_time(embassy_stm32::adc::SampleTime::Cycles28);
        static ADC_INST: TrackedStaticCell<ControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();
        ControllerRef::new(ADC_INST.init::<{crate::MAX_STATIC_MEMORY}>(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed)
        ))
    };

    #[cfg(any(feature = "with-hot-end"))]
        let hotend_device = {

        let pwm_hotend = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            p.TIM9,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PE5, embassy_stm32::gpio::OutputType::PushPull)),
            None,
            None,
            None,
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmHotend>> = TrackedStaticCell::new();

        device::HotendPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>(
                "PwmFanFan0HotendHotbed",
                ControllerMutex::new(pwm_hotend)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PC1,
            thermistor_properties: &HOT_END_THERMISTOR_PROPERTIES,
        }
    };

    #[cfg(any(feature = "with-hot-bed"))]
        let hotbed_device = {

        let pwm_hotbed = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            p.TIM5,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PA0, embassy_stm32::gpio::OutputType::PushPull)),
            None,
            None,
            None,
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmHotbed>> = TrackedStaticCell::new();

        device::HotbedPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>(
                "PwmHotbed",
                ControllerMutex::new(pwm_hotbed)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PC0,
            thermistor_properties: &HOT_BED_THERMISTOR_PROPERTIES,
        }
    };

    #[cfg(any(feature = "with-laser"))]
        let laser_device = {

        let pwm_laser = device::PwmLaser::new(
            p.TIM13,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PA6, embassy_stm32::gpio::OutputType::PushPull)),
            None,
            None,
            None,
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmLaser>> = TrackedStaticCell::new();

        device::LaserPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>(
                "PwmLaser",
                ControllerMutex::new(pwm_laser)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
        }
    };

    #[cfg(feature = "with-ps-on")]
    let ps_on = {
        static PS_ON: TrackedStaticCell<ControllerMutex<device::PsOnPin>> = TrackedStaticCell::new();
        ControllerRef::new(
            PS_ON.init::<{crate::MAX_STATIC_MEMORY}>("", ControllerMutex::new(Output::new(p.PB2, Level::Low, Speed::Low)))
        )
    };

    #[cfg(feature = "with-motion")]
    defmt::info!("motion_planner done");

    static WD: TrackedStaticCell<ControllerMutex<device::Watchdog>> = TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init::<{crate::MAX_STATIC_MEMORY}>("watchdog", ControllerMutex::new(device::Watchdog::new(p.IWDG, WATCHDOG_TIMEOUT))));

    MachineContext {
        controllers: Controllers {
            sys_watchdog,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
        },
        sys_devices: SysDevices {
            #[cfg(feature = "with-motion")]
            task_stepper_core: printhor_hwa_common::NoDevice::new(),
            #[cfg(feature = "with-ps-on")]
            ps_on
        },
        io_devices: IODevices {
            #[cfg(feature = "with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_rx_stream,
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
            #[cfg(feature = "with-fan-layer")]
            fan_layer: fan_layer_device,
            #[cfg(feature = "with-hot-end")]
            hotend: hotend_device,
            #[cfg(feature = "with-hot-bed")]
            hotbed: hotbed_device,
            #[cfg(feature = "with-laser")]
            laser: laser_device,
        }
    }
}
