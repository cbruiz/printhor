//noinspection RsDetachedFile
/// <https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32G0B1RE.json>
pub mod device;
pub mod io;

use alloc_cortex_m::CortexMHeap;
#[cfg(feature = "with-motion")]
use device::{MotionDevice, MotionPins};
pub use embassy_executor::Spawner;
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
use embassy_stm32::adc::SampleTime;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
#[cfg(feature = "with-spi")]
use embassy_stm32::spi;
#[cfg(any(feature = "with-serial-port-1", feature = "with-trinamic"))]
use embassy_stm32::usart::{DataBits, Parity, StopBits};
#[cfg(feature = "with-serial-usb")]
use embassy_stm32::usb;
#[allow(unused)]
use embassy_sync::mutex::Mutex;


#[global_allocator]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "SKR";

pub const MACHINE_BOARD: &str = "SKR_MINI_E3_V3";
/// ARM Cortex M0+ @64MHZ, 144kB SRAM, 512kB Program
pub const MACHINE_PROCESSOR: &str = "STM32G0B1RET6";
pub const PROCESSOR_SYS_CK_MHZ: u32 = 64_000_000;

pub const MAX_STATIC_MEMORY: usize = 16386;
pub const HEAP_SIZE_BYTES: usize = 1024;

/// Micro-segment sampling frequency in Hz
pub const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 200;
/// Micro-segment clock frequency in Hz
pub const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 20_000;

// https://www.st.com/resource/en/datasheet/dm00748675.pdf
pub const ADC_START_TIME_US: u16 = 12;
// https://www.st.com/resource/en/datasheet/dm00748675.pdf
pub const ADC_VREF_DEFAULT_MV: u16 = 1212;

#[cfg(feature = "with-sd-card")]
pub const SDCARD_PARTITION: usize = 0;
#[cfg(feature = "with-trinamic")]
pub const TRINAMIC_UART_BAUD_RATE: u32 = 9600;
pub const WATCHDOG_TIMEOUT_US: u32 = 8_000_000;
#[cfg(feature = "with-spi")]
pub(crate) const SPI_FREQUENCY_HZ: u32 = 1_000_000;

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
    pub sys_watchdog: StandardControllerRef<device::Watchdog>,
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: device::USBSerialTxControllerRef,
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
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_rx_stream: device::USBSerialDeviceInputStream,
    /// Only single owner allowed
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: device::UartPort1RxInputStream,
    #[cfg(feature = "with-serial-port-2")]
    pub serial_port2_rx_stream: device::UartPort2RxInputStream,
    #[cfg(feature = "with-display")]
    pub display_device: DisplayDevice,
    #[cfg(feature = "with-sd-card")]
    pub sd_card_device: device::SpiCardDeviceRef,
    #[cfg(feature = "with-sd-card")]
    pub sd_card_cs_pin: device::SpiCardCSPin,
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
    #[cfg(feature = "with-laser")]
    pub laser: device::LaserPeripherals,
}

pub struct MotionDevices {
    #[cfg(feature = "with-motion")]
    pub motion_devices: MotionDevice,
}

pub fn heap_current_size() -> u32 {
    HEAP.used() as u32
}

pub fn stack_reservation_current_size() -> u32 {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(printhor_hwa_common::COUNTER)) as u32 }
}

// Interrupt mappings
#[cfg(feature = "with-serial-usb")]
embassy_stm32::bind_interrupts!(struct UsbIrqs {
    USB_UCPD1_2 => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB>;
});
#[cfg(feature = "with-serial-port-1")]
embassy_stm32::bind_interrupts!(struct UartPort1Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});
#[cfg(feature = "with-serial-port-2")]
embassy_stm32::bind_interrupts!(struct UartPort2Irqs {
    USART2_LPUART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});
#[cfg(feature = "with-trinamic")]
embassy_stm32::bind_interrupts!(struct TrinamicIrqs {
    USART3_4_5_6_LPUART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART4>;
});

#[inline]
pub(crate) fn init_heap() -> () {
    use core::mem::MaybeUninit;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE_BYTES] =
        [MaybeUninit::uninit(); HEAP_SIZE_BYTES];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE_BYTES) }
}

#[inline]
pub fn init() -> embassy_stm32::Peripherals {
    init_heap();
    #[allow(unused_mut)]
    let mut config = embassy_stm32::Config::default();

    cfg_if::cfg_if! {
        if #[cfg(not(feature = "without-bootloader"))] {

            // Reset bootloader state

            #[allow(unused)]
            use embassy_stm32::pac::*;
            #[allow(unused)]
            use embassy_stm32::rcc::*;

            // SYSCFG: Dead battery pull-down resistors functionality should be enabled by default on startup
            SYSCFG.cfgr1().modify(|w| {
                w.set_ucpd1_strobe(true);
                w.set_ucpd2_strobe(true);
            });

            // RCC: Enable HSI and wait for it to be ready
            RCC.cr().modify(|w| {
                w.set_hsion(true)
            });
            while !RCC.cr().read().hsirdy() {}

            // CFGR: Write its default
            RCC.cfgr().write(|_w| {
            });

            // RCC: Enable HSI only. Wait for PLL to be unready
            RCC.cr().write(|w| {
                w.set_hsion(true)
            });
            while RCC.cr().read().pllrdy() {}

            // Reset values from datasheet
            RCC.pllcfgr().write_value(rcc::regs::Pllcfgr(0x00001000));
            RCC.gpioenr().write_value(rcc::regs::Gpioenr(0x00001000));
            RCC.ahbenr().write_value(rcc::regs::Ahbenr(0x000000100));
            RCC.apbenr1().write_value(rcc::regs::Apbenr1(0x000000000));
            RCC.apbenr2().write_value(rcc::regs::Apbenr2(0x000000000));

            unsafe {
                defmt::info!("Setting VTOR...");
                #[allow(unused_mut)]
                let mut p = cortex_m::Peripherals::steal();
                defmt::trace!("VTOR WAS AT: {} ", p.SCB.vtor.read());
                p.SCB.vtor.write(0x2000);
                defmt::trace!("VTOR SET TO: {} ", p.SCB.vtor.read());
            }
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature="use-hsi")] {
            defmt::trace!("HSI...");

            config.rcc.mux = ClockSrc::PLL(
                PllConfig {
                    // HSI = 16MHz
                    source: PllSource::HSI,
                    m: Pllm::DIV1,
                    n: Plln::MUL12,
                    // SysClk = 16 / 1 * 12 / 3 = 64MHz
                    r: Pllr::DIV3,
                    // PLLQ = 16 / 1 * 12 / 4 = 48MHz
                    q: Some(Pllq::DIV2),
                    // PLLP = 16 / 1 * 12 / 3 = 64MHz
                    p: Some(Pllp::DIV3),
                }
            );
            config.rcc.usb_src = Some(UsbSrc::Hsi48(
                Hsi48Config {
                    sync_from_usb: true,
                    ..Default::default()
                }
            ));
        }
        else {
            defmt::trace!("PLL...");
            config.rcc.hsi = false;
            config.rcc.hse = Some(Hse{
                freq: embassy_stm32::time::Hertz(8_000_000),
                mode: HseMode::Oscillator
            });
            config.rcc.pll = Some(
                Pll {
                    // HSE = 8MHz
                    source: PllSource::HSE,
                    prediv: PllPreDiv::DIV1,
                    mul: PllMul::MUL24,
                    // SysClk = 8 / 1 * 24 / 3 = 64MHz
                    divr: Some(PllRDiv::DIV3),
                    // PLLQ = 8 / 1 * 24 / 4 = 48MHz
                    divq: Some(PllQDiv::DIV4),
                    // PLLP = 8 / 1 * 24 / 3 = 64MHz
                    divp: Some(PllPDiv::DIV3),
                }
            );
            config.rcc.sys = Sysclk::PLL1_R;
            // PllQ does not work for Usb in this board
            config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
            config.rcc.mux.usbsel = mux::Usbsel::HSI48;
            // HCLK = {Power, AHB bus, core, memory, DMA, System timer, FCLK} = 64MHz
            config.rcc.ahb_pre = AHBPrescaler::DIV1;
            // PCLK = APB peripheral clocks = 64MHz
            // TPCLK = APOB timer clocks = 64MHz
            config.rcc.ahb_pre = AHBPrescaler::DIV1;
            config.rcc.low_power_run = false;

        }
    }

    defmt::debug!("Initiallizing embassy_stm32...");
    embassy_stm32::init(config)
}

pub async fn setup(
    _spawner: Spawner,
    p: embassy_stm32::Peripherals,
) -> printhor_hwa_common::MachineContext<
    Controllers,
    SysDevices,
    IODevices,
    MotionDevices,
    PwmDevices,
> {
    defmt::debug!("Setting up...");

    embassy_stm32::pac::SYSCFG.cfgr1().write(|w| {
        // https://www.st.com/resource/en/reference_manual/rm0454-stm32g0x0-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
        //  set_pa11_rmp and set_pa12_rmp (bits 3 and 4)
        let val = true;
        let off = 3usize;
        w.0 = (w.0 & !(0x01 << off)) | (((val as u32) & 0x01) << off);
        let off = 4usize;
        w.0 = (w.0 & !(0x01 << off)) | (((val as u32) & 0x01) << off);
    });

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-spi")] {
            let mut cfg = spi::Config::default();
            cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
            #[link_section = ".bss"]
            static SPI1_INST: TrackedStaticCell<StandardControllerMutex<device::Spi1>> = TrackedStaticCell::new();
            let spi1_device = ControllerRef::new(
                SPI1_INST.init::<{crate::MAX_STATIC_MEMORY}>(
                    "SPI1",
                    ControllerMutex::new(
                        device::Spi1::new(
                            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH4, p.DMA1_CH3, cfg
                        )
                    )
                )
            );
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-usb")] {
            let (serial_usb_tx, serial_usb_rx_stream) = {
                defmt::info!("Creating USB Driver");
                let driver = usb::Driver::new(p.USB, UsbIrqs, p.PA12, p.PA11);
                let mut usb_serial_device = device::USBSerialDevice::new(driver);
                usb_serial_device.spawn(_spawner);
                let (usb_serial_rx_device, sender) = usb_serial_device.split();
                #[link_section = ".bss"]
                static USB_INST: TrackedStaticCell<Mutex<ControllerMutexType, device::USBSerialDeviceSender>> = TrackedStaticCell::new();
                let serial_usb_tx = ControllerRef::new(
                    USB_INST.init::<{crate::MAX_STATIC_MEMORY}>("USBSerialTxController", Mutex::<ControllerMutexType, _>::new(sender))
                );
                (serial_usb_tx, device::USBSerialDeviceInputStream::new(usb_serial_rx_device))
            };
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-1")] {
            let (serial_port1_tx, serial_port1_rx_stream) = {
                let mut cfg = embassy_stm32::usart::Config::default();
                cfg.baudrate = crate::UART_PORT1_BAUD_RATE;
                cfg.data_bits = DataBits::DataBits8;
                cfg.stop_bits = StopBits::STOP1;
                cfg.parity = Parity::ParityNone;
                cfg.detect_previous_overrun = false;

                let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(p.USART1,
                                                                    p.PA10, p.PA9,
                                                                    UartPort1Irqs,
                                                                    p.DMA2_CH2, p.DMA2_CH1,
                                                                    cfg).expect("Ready").split();

                #[link_section = ".bss"]
                static UART_PORT1_INST: TrackedStaticCell<ControllerMutex<printhor_hwa_common::SerialAsyncWrapper<device::UartPort1TxDevice>>> = TrackedStaticCell::new();
                let serial_port1_tx = ControllerRef::new(
                    UART_PORT1_INST.init::<{crate::MAX_STATIC_MEMORY}>("UartPort1", Mutex::<ControllerMutexType, _>::new(printhor_hwa_common::SerialAsyncWrapper::new(uart_port1_tx_device, crate::UART_PORT1_BAUD_RATE)))
                );
                (serial_port1_tx, device::UartPort1RxInputStream::new(uart_port1_rx_device))
            };
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-2")] {
            let (serial_port2_tx, serial_port2_rx_stream) = {

                let mut cfg = embassy_stm32::usart::Config::default();
                cfg.baudrate = crate::UART_PORT2_BAUD_RATE;
                cfg.data_bits = embassy_stm32::usart::DataBits::DataBits8;
                cfg.stop_bits = embassy_stm32::usart::StopBits::STOP1;
                cfg.parity = embassy_stm32::usart::Parity::ParityNone;
                cfg.detect_previous_overrun = false;

                let (uart_port2_tx_device, uart_port2_rx_device) = device::UartPort2Device::new(p.USART2,
                                                                    p.PA3, p.PA2,
                                                                    UartPort2Irqs,
                                                                    p.DMA2_CH4, p.DMA2_CH3,
                                                                    cfg).expect("Ready").split();
                #[link_section = ".bss"]
                static UART_PORT2_INST: TrackedStaticCell<StandardControllerMutex<printhor_hwa_common::SerialAsyncWrapper<device::UartPort2TxDevice>>> = TrackedStaticCell::new();
                let serial_port2_tx = ControllerRef::new(
                    UART_PORT2_INST.init::<{crate::MAX_STATIC_MEMORY}>("UartPort2", Mutex::<ControllerMutexType, _>::new(
                        printhor_hwa_common::SerialAsyncWrapper::new(uart_port2_tx_device, crate::UART_PORT2_BAUD_RATE)
                    ))
                );
                (serial_port2_tx, device::UartPort2RxInputStream::new(uart_port2_rx_device))
            };
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-trinamic")] {
            let trinamic_uart = {
                let mut cfg = embassy_stm32::usart::Config::default();
                cfg.baudrate = TRINAMIC_UART_BAUD_RATE;
                cfg.data_bits = DataBits::DataBits8;
                cfg.stop_bits = StopBits::STOP1;
                cfg.parity = Parity::ParityNone;
                cfg.detect_previous_overrun = false;


                device::TrinamicUartDevice::new(p.USART4, p.PC11, p.PC10,
                           TrinamicIrqs, p.DMA1_CH7, p.DMA1_CH6,
                           cfg).expect("Ready")
            };
        }
    }

    #[cfg(feature = "with-spi")]
    defmt::info!("SPI done");

    #[cfg(feature = "with-sd-card")]
    let (sd_card_device, sd_card_cs_pin) = {
        (
            spi1_device.clone(),
            embassy_stm32::gpio::Output::new(
                p.PA4,
                embassy_stm32::gpio::Level::High,
                embassy_stm32::gpio::Speed::VeryHigh,
            ),
        )
    };
    #[cfg(feature = "with-sd-card")]
    defmt::info!("card_controller done");

    #[cfg(feature = "with-display")]
    let display_device = device::DisplayDevice {
        interface: spi_device,
        rst: embassy_stm32::gpio::Output::new(p.PC1, Level::High, Speed::VeryHigh),
        cs: embassy_stm32::gpio::Output::new(p.PB0, Level::High, Speed::VeryHigh),
        dc: embassy_stm32::gpio::Output::new(p.PA4, Level::High, Speed::VeryHigh),
    };
    #[cfg(feature = "with-display")]
    crate::info!("display_device done");

    #[cfg(feature = "with-motion")]
    let motion_devices = MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart: io::TrinamicUartWrapper::new(trinamic_uart),
        motion_pins: MotionPins {
            x_enable_pin: embassy_stm32::gpio::Output::new(p.PB14, Level::High, Speed::VeryHigh),
            y_enable_pin: embassy_stm32::gpio::Output::new(p.PB11, Level::High, Speed::VeryHigh),
            z_enable_pin: embassy_stm32::gpio::Output::new(p.PB1, Level::High, Speed::VeryHigh),
            #[cfg(feature = "with-e-axis")]
            e_enable_pin: embassy_stm32::gpio::Output::new(p.PD1, Level::High, Speed::VeryHigh),
            x_endstop_pin: embassy_stm32::gpio::Input::new(p.PC0, Pull::Down),
            y_endstop_pin: embassy_stm32::gpio::Input::new(p.PC1, Pull::Down),
            z_endstop_pin: embassy_stm32::gpio::Input::new(p.PC2, Pull::Down),
            #[cfg(feature = "with-e-axis")]
            e_endstop_pin: embassy_stm32::gpio::Input::new(p.PC15, Pull::Down),
            x_step_pin: embassy_stm32::gpio::Output::new(p.PB13, Level::Low, Speed::VeryHigh),
            y_step_pin: embassy_stm32::gpio::Output::new(p.PB10, Level::Low, Speed::VeryHigh),
            z_step_pin: embassy_stm32::gpio::Output::new(p.PB0, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-e-axis")]
            e_step_pin: embassy_stm32::gpio::Output::new(p.PB3, Level::Low, Speed::VeryHigh),
            x_dir_pin: embassy_stm32::gpio::Output::new(p.PB12, Level::Low, Speed::VeryHigh),
            y_dir_pin: embassy_stm32::gpio::Output::new(p.PB2, Level::Low, Speed::VeryHigh),
            z_dir_pin: embassy_stm32::gpio::Output::new(p.PC5, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-e-axis")]
            e_dir_pin: embassy_stm32::gpio::Output::new(p.PB4, Level::Low, Speed::VeryHigh),
        },
    };

    #[cfg(feature = "with-hot-end")]
    static HOT_END_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties =
        printhor_hwa_common::ThermistorProperties::new(
            HOT_END_THERM_PULL_UP_RESISTANCE,
            HOT_END_THERM_NOMINAL_RESISTANCE,
            HOT_END_THERM_BETA,
        );

    #[cfg(feature = "with-hot-bed")]
    static HOT_BED_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties =
        printhor_hwa_common::ThermistorProperties::new(
            HOT_BED_THERM_PULL_UP_RESISTANCE,
            HOT_BED_THERM_NOMINAL_RESISTANCE,
            HOT_BED_THERM_BETA,
        );

    // PC7(Fan1) PC6(Fan0) PC8(HE_PWM) PC9(BED_PWM) PB15(Fan2) PA8(neo) PA1(Probe)
    // OK FOR: FAN0, FAN1, HE_PWM, BE_PWM
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer"))] {

            use embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts;

            let pwm_fan0_fan1_hotend_hotbed = {
                let pwm_fan0_fan1_hotend_hotbed = embassy_stm32::timer::simple_pwm::SimplePwm::new(
                    p.TIM3,
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PC6, embassy_stm32::gpio::OutputType::PushPull)), // PA6 | PB4 | PC6
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PC7, embassy_stm32::gpio::OutputType::PushPull)), // PA7 | PB5 | PC7
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch3(p.PC8, embassy_stm32::gpio::OutputType::PushPull)), // PB0 | PC8
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PC9, embassy_stm32::gpio::OutputType::PushPull)), // PB1 | PC9
                    embassy_stm32::time::hz(5_000),
                    CenterAlignedBothInterrupts,
                );
                #[link_section = ".bss"]
                static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmFan0Fan1HotendHotbed>> = TrackedStaticCell::new();
                ControllerRef::new(PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>(
                    "PwmFanFan0HotendHotbed",
                    ControllerMutex::new(pwm_fan0_fan1_hotend_hotbed)
                ))
            };
        }
    }

    #[cfg(feature = "with-fan-layer")]
    let pwm_fan1_channel = embassy_stm32::timer::Channel::Ch2;
    #[cfg(feature = "with-hot-end")]
    let pwm_hotend_channel = embassy_stm32::timer::Channel::Ch3;
    #[cfg(feature = "with-hot-bed")]
    let pwm_hotbed_channel = embassy_stm32::timer::Channel::Ch4;

    // V2 con PA8, T1 CH1
    #[cfg(any(feature = "with-laser"))]
    let (pwm_laser, pwm_laser_channel) = {
        use embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts;

        #[link_section = ".bss"]
        static PWM_LASER_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmLaser>> =
            TrackedStaticCell::new();
        (
            ControllerRef::new(PWM_LASER_INST.init::<{ crate::MAX_STATIC_MEMORY }>(
                "LaserPwmController",
                ControllerMutex::new(embassy_stm32::timer::simple_pwm::SimplePwm::new(
                    p.TIM16,
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(
                        p.PD0,
                        OutputType::PushPull,
                    )), // PA6 | PB8 | PD0
                    None, //
                    None, //
                    None, //
                    embassy_stm32::time::hz(5_000),
                    CenterAlignedBothInterrupts,
                )),
            )),
            embassy_stm32::timer::Channel::Ch1,
        )
    };

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    let adc_hotend_hotbed = {
        let mut adc_hotend_hotbed = device::AdcHotendHotbed::new(p.ADC1);
        adc_hotend_hotbed.set_sample_time(SampleTime::CYCLES7_5);
        #[link_section = ".bss"]
        static ADC_INST: TrackedStaticCell<InterruptControllerMutex<device::AdcHotendHotbed>> =
            TrackedStaticCell::new();

        ControllerRef::new(ADC_INST.init::<{ crate::MAX_STATIC_MEMORY }>(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed),
        ))
    };
    #[cfg(feature = "with-hot-end")]
    let adc_hotend_pin = p.PA0;
    #[cfg(feature = "with-hot-bed")]
    let adc_hotbed_pin = p.PC4;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            #[link_section = ".bss"]
            static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmServo>> = TrackedStaticCell::new();
            let pwm_dev: device::PwmServo = device::PwmServo::new(
                  p.TIM2,
                  None, // PA0 | PA15 | PA5 | PC4
                  Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PA1, OutputType::PushPull)), // PA1 | PB3 | PC5
                  None, // PA2 | PB10 | PC6
                  None, // PA3 | PB11 | PC7
                  embassy_stm32::time::hz(50),
                  embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
              );

            let power_pwm = ControllerRef::new(
                PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>("PwmServo", ControllerMutex::new(pwm_dev))
            );
        }
    }

    #[cfg(feature = "with-probe")]
    let probe_device = {
        crate::device::ProbePeripherals {
            power_pwm,
            power_channel: embassy_stm32::timer::Channel::Ch2,
        }
    };

    #[cfg(feature = "with-motion")]
    defmt::info!("motion_planner done");

    #[cfg(feature = "with-ps-on")]
    let ps_on = {
        #[link_section = ".bss"]
        static PS_ON: TrackedStaticCell<StandardControllerMutex<device::PsOnPin>> =
            TrackedStaticCell::new();
        ControllerRef::new(PS_ON.init::<{ crate::MAX_STATIC_MEMORY }>(
            "",
            ControllerMutex::new(Output::new(p.PC13, Level::Low, Speed::Low)),
        ))
    };

    #[link_section = ".bss"]
    static WD: TrackedStaticCell<StandardControllerMutex<device::Watchdog>> =
        TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init::<{ crate::MAX_STATIC_MEMORY }>(
        "watchdog",
        ControllerMutex::new(device::Watchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US)),
    ));

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            crate::setup_timer();
        }
    }
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
            #[cfg(feature = "with-motion")]
            task_stepper_core: printhor_hwa_common::NoDevice::new(),
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
            #[cfg(feature = "with-display")]
            display_device,
            #[cfg(feature = "with-sd-card")]
            sd_card_device,
            #[cfg(feature = "with-sd-card")]
            sd_card_cs_pin,
        },
        motion: MotionDevices {
            #[cfg(feature = "with-motion")]
            motion_devices,
        },
        pwm: PwmDevices {
            #[cfg(feature = "with-probe")]
            probe: probe_device,
            #[cfg(feature = "with-hot-end")]
            hotend: device::HotendPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_hotend_channel,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: adc_hotend_pin,
                thermistor_properties: &HOT_END_THERMISTOR_PROPERTIES,
            },
            #[cfg(feature = "with-hot-bed")]
            hotbed: device::HotbedPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_hotbed_channel,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: adc_hotbed_pin,
                thermistor_properties: &HOT_BED_THERMISTOR_PROPERTIES,
            },
            #[cfg(feature = "with-fan-layer")]
            fan_layer: device::FanLayerPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_fan1_channel,
            },
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra_1: device::FanExtra1Peripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_fan0_channel,
            },
            #[cfg(feature = "with-laser")]
            laser: device::LaserPeripherals {
                power_pwm: pwm_laser.clone(),
                power_channel: pwm_laser_channel,
            },
        },
    }
}
