/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32F103RC.json
pub mod device;
pub mod io;

use alloc_cortex_m::CortexMHeap;
pub use embassy_executor::Spawner;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Level, Output, Speed, Pull, OutputType};
#[allow(unused)]
use embassy_sync::mutex::Mutex;
#[cfg(any(feature = "with-serial-port-1", feature="with-trinamic"))]
use embassy_stm32::usart::{DataBits, Parity, StopBits};
#[cfg(feature = "with-serial-usb")]
use embassy_stm32::usb;
#[cfg(feature = "with-spi")]
use embassy_stm32::spi;
#[allow(unused)]
use printhor_hwa_common::{ControllerMutex, ControllerRef, ControllerMutexType};
use printhor_hwa_common::{TrackedStaticCell, MachineContext, StandardControllerRef};
#[cfg(feature = "with-motion")]
use device::{MotionDevice, MotionPins};
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
use embassy_stm32::adc::SampleTime;
use printhor_hwa_common::StandardControllerMutex;

#[global_allocator]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "SKR";
pub const MACHINE_BOARD: &str = "SKR_MINI_E3_V2";
/// ARM Cortex M3 @72MHZ, 48kB SRAM, 256kB Program
pub const MACHINE_PROCESSOR: &str = "STM32F103RCT6";
pub const PROCESSOR_SYS_CK_MHZ: u32 = 72_000_000;

pub const MAX_STATIC_MEMORY: usize = 8192;
pub const HEAP_SIZE_BYTES: usize = 512;

        /// Micro-segment sampling frequency in Hz
pub const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 72;
/// Micro-segment clock frequency in Hz
pub const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 72_000;

// https://www.st.com/resource/en/datasheet/CD00191185.pdf
pub const ADC_START_TIME_US: u16 = 17;
// https://www.st.com/resource/en/datasheet/CD00191185.pdf
pub const ADC_VREF_DEFAULT_MV: u16 = 1200;
cfg_if::cfg_if! {
    if #[cfg(feature="without-vref-int")] {
        // As VrefInt is not present in this board_stm32l4, the estimation is 1489 as ADC value for 1200 mV
        // So theoretically it will get 3300 mV for maximum ADC value (4095)
        pub const ADC_VREF_DEFAULT_SAMPLE: u16 = 1489;
    }
}

#[cfg(feature = "with-sdcard")]
pub const SDCARD_PARTITION: usize = 0;
#[cfg(feature = "with-trinamic")]
pub(crate) const TRINAMIC_UART_BAUD_RATE: u32 = 9600;
pub(crate) const WATCHDOG_TIMEOUT: u32 = 8_000_000;
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
    #[cfg(feature  ="with-display")]
    pub display_device: DisplayDevice,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_device: device::SpiCardDeviceRef,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_cs_pin: device::SpiCardCSPin,
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

#[inline]
pub fn stack_reservation_current_size() -> u32 {
    unsafe {
        core::ptr::read_volatile(core::ptr::addr_of!(printhor_hwa_common::COUNTER)) as u32
    }
}

// Interrupt mappings
#[cfg(feature = "with-serial-usb")]
embassy_stm32::bind_interrupts!(struct UsbIrqs {
    USB_LP_CAN1_RX0 => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB>;
});
#[cfg(feature = "with-serial-port-1")]
embassy_stm32::bind_interrupts!(struct UartPort1Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});
#[cfg(feature = "with-serial-port-2")]
embassy_stm32::bind_interrupts!(struct UartPort2Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});
#[cfg(feature = "with-trinamic")]
embassy_stm32::bind_interrupts!(struct TrinamicIrqs {
    UART4 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::UART4>;
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
    #[allow(unused_mut)]
    let mut config = embassy_stm32::Config::default();

    cfg_if::cfg_if! {
        if #[cfg(not(feature = "without-bootloader"))] {

            // Reset bootloader state

            #[allow(unused)]
            use embassy_stm32::pac::*;
            #[allow(unused)]
            use embassy_stm32::rcc::*;

            // RCC: Enable HSI and wait for it to be ready
            RCC.cr().modify(|w| {
                w.set_hsion(true)
            });
            while !RCC.cr().read().hsirdy() {}

            // RCC: Reset CFGR to defaults
            RCC.cfgr().modify(|w| {
                w.set_sw(rcc::vals::Sw::HSI);
                w.set_hpre(rcc::vals::Hpre::DIV1);
                w.set_ppre1(rcc::vals::Ppre::DIV1);
                w.set_ppre2(rcc::vals::Ppre::DIV1);
                w.set_adcpre(rcc::vals::Adcpre::DIV2);
                w.set_mcosel(rcc::vals::Mcosel::DISABLE);
            });

            // RCC: Enable HSI only. Wait for PLL to be unready
            RCC.cr().write(|w| {
                w.set_hsion(true)
            });
            while RCC.cr().read().pllrdy() {}

            // Reset values from datasheet
            //RCC.cir().write_value(rcc::regs::Cir(0x000000000));
            RCC.ahbenr().write_value(rcc::regs::Ahbenr(0x000000014));
            RCC.apb2enr().write_value(rcc::regs::Apb2enr(0x00000000));
            RCC.apb1enr().write_value(rcc::regs::Apb1enr(0x00000000));

            unsafe {
                defmt::info!("Setting VTOR...");
                #[allow(unused_mut)]
                let mut p = cortex_m::Peripherals::steal();
                defmt::trace!("VTOR WAS AT: {} ", p.SCB.vtor.read());
                p.SCB.vtor.write(0x7000);
                defmt::trace!("VTOR SET TO: {} ", p.SCB.vtor.read());
            }
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature="use-hsi")] {
            compile_error!("Not supported")
        }
        else {
            config.rcc.hse = Some(embassy_stm32::rcc::Hse {
                freq: embassy_stm32::time::Hertz(8_000_000),
                mode: embassy_stm32::rcc::HseMode::Oscillator,
            });
            config.rcc.pll = Some(embassy_stm32::rcc::Pll {
                src: embassy_stm32::rcc::PllSource::HSE,
                prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
                mul: embassy_stm32::rcc::PllMul::MUL9,
            });
            config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;
            config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
            config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV2;
            config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
            config.rcc.adc_pre = embassy_stm32::rcc::ADCPrescaler::DIV6;
        }
    }

    defmt::debug!("Initiallizing embassy_stm32...");
    embassy_stm32::init(config)
}

pub async fn setup(_spawner: Spawner, p: embassy_stm32::Peripherals) -> printhor_hwa_common::MachineContext<Controllers, SysDevices, IODevices, MotionDevices, PwmDevices> {

    defmt::debug!("Setting up...");

    #[cfg(feature = "with-spi")]
    let spi1_device = {

        let mut cfg = spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
        #[link_section = ".bss"]
        static SPI1_INST: TrackedStaticCell<StandardControllerMutex<device::Spi1>> = TrackedStaticCell::new();
        ControllerRef::new(SPI1_INST.init::<{crate::MAX_STATIC_MEMORY}>("SPI1",
            ControllerMutex::new(
                device::Spi1::new(p.SPI1, p.PA5, p.PA7, p.PA6,
                                  p.DMA1_CH3, p.DMA1_CH2, cfg
                )
            )
        ))
    };

    #[cfg(feature = "with-serial-usb")]
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

    #[cfg(feature = "with-serial-port-1")]
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
                                                            p.DMA1_CH4, p.DMA1_CH5,
                                                            cfg).expect("Ready").split();
        #[link_section = ".bss"]
        static UART_PORT1_INST: TrackedStaticCell<StandardControllerMutex<printhor_hwa_common::SerialAsyncWrapper<device::UartPort1TxDevice>>> = TrackedStaticCell::new();
        let serial_port1_tx = ControllerRef::new(
            UART_PORT1_INST.init::<{crate::MAX_STATIC_MEMORY}>("UartPort1", Mutex::<ControllerMutexType, _>::new(printhor_hwa_common::SerialAsyncWrapper::new(uart_port1_tx_device, crate::UART_PORT1_BAUD_RATE)))
        );
        (serial_port1_tx, device::UartPort1RxInputStream::new(uart_port1_rx_device))
    };

    #[cfg(feature = "with-serial-port-2")]
    let (serial_port2_tx, serial_port2_rx_stream) = {

        let mut cfg = embassy_stm32::usart::Config::default();
        cfg.baudrate = crate::UART_PORT2_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;
        cfg.detect_previous_overrun = false;

        let (uart_port2_tx_device, uart_port2_rx_device) = device::UartPort2Device::new(p.USART2,
                                                            p.PA3, p.PA2,
                                                            UartPort2Irqs,
                                                            p.DMA1_CH7, p.DMA1_CH6,
                                                            cfg).expect("Ready").split();
        #[link_section = ".bss"]
        static UART_PORT2_INST: TrackedStaticCell<StandardControllerMutex<printhor_hwa_common::SerialAsyncWrapper<device::UartPort2TxDevice>>> = TrackedStaticCell::new();
        let serial_port2_tx = ControllerRef::new(
            UART_PORT2_INST.init::<{crate::MAX_STATIC_MEMORY}>("UartPort2", Mutex::<ControllerMutexType, _>::new(printhor_hwa_common::SerialAsyncWrapper::new(uart_port2_tx_device, crate::UART_PORT2_BAUD_RATE)))
        );
        (serial_port2_tx, device::UartPort2RxInputStream::new(uart_port2_rx_device))
    };

    #[cfg(all(feature = "with-trinamic"))]
    let trinamic_uart = {
        let mut cfg = embassy_stm32::usart::Config::default();
        cfg.baudrate = TRINAMIC_UART_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;
        cfg.detect_previous_overrun = true;

        device::TrinamicUartDevice::new(p.UART4, p.PC11, p.PC10,
                   TrinamicIrqs, p.DMA2_CH5, p.DMA2_CH3, cfg).expect("Ready")
    };

    #[cfg(feature = "with-spi")]
    defmt::info!("SPI done");

    #[cfg(feature = "with-sdcard")]
    let (sdcard_device, sdcard_cs_pin) = {
        (spi1_device.clone(), Output::new(p.PA4, Level::High, Speed::VeryHigh))
    };
    #[cfg(feature = "with-sdcard")]
    defmt::info!("card_controller done");

    #[cfg(feature = "with-display")]
    let display_device = device::DisplayDevice {
        interface: spi_device,
        rst: Output::new(p.PC1, Level::High, Speed::VeryHigh),
        cs: Output::new(p.PB0, Level::High, Speed::VeryHigh),
        dc: Output::new(p.PA4, Level::High, Speed::VeryHigh),
    };
    #[cfg(feature = "with-display")]
    crate::info!("display_device done");

    #[cfg(feature = "with-motion")]
    let motion_devices = MotionDevice {

        #[cfg(feature = "with-trinamic")]
        trinamic_uart: io::TrinamicUartWrapper::new(trinamic_uart),
        motion_pins: MotionPins {
            #[cfg(feature = "with-x-axis")]
            x_enable_pin: Output::new(p.PB14, Level::High, Speed::VeryHigh),
            #[cfg(feature = "with-y-axis")]
            y_enable_pin: Output::new(p.PB11, Level::High, Speed::VeryHigh),
            #[cfg(feature = "with-z-axis")]
            z_enable_pin: Output::new(p.PB1, Level::High, Speed::VeryHigh),
            #[cfg(feature = "with-e-axis")]
            e_enable_pin: Output::new(p.PD1, Level::High, Speed::VeryHigh),
            #[cfg(feature = "with-x-axis")]
            x_endstop_pin: Input::new(p.PC0, Pull::Down),
            #[cfg(feature = "with-y-axis")]
            y_endstop_pin: Input::new(p.PC1, Pull::Down),
            #[cfg(feature = "with-z-axis")]
            z_endstop_pin: Input::new(p.PC2, Pull::Down),
            #[cfg(feature = "with-e-axis")]
            e_endstop_pin: Input::new(p.PC15, Pull::Down),
            #[cfg(feature = "with-x-axis")]
            x_step_pin: Output::new(p.PB13, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-y-axis")]
            y_step_pin: Output::new(p.PB10, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-z-axis")]
            z_step_pin: Output::new(p.PB0, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-e-axis")]
            e_step_pin: Output::new(p.PB3, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-x-axis")]
            x_dir_pin: Output::new(p.PB12, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-y-axis")]
            y_dir_pin: Output::new(p.PB2, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-z-axis")]
            z_dir_pin: Output::new(p.PC5, Level::Low, Speed::VeryHigh),
            #[cfg(feature = "with-e-axis")]
            e_dir_pin: Output::new(p.PB4, Level::Low, Speed::VeryHigh),
        }
    };
    #[cfg(feature = "with-motion")]
    defmt::info!("motion_driver done");

    // PC7(Fan1) PC6(Fan0) PC8(HE_PWM) PC9(BED_PWM) PB15(Fan2) PA8(neo) PA1(Probe)
    // OK FOR: FAN0, FAN1, HE_PWM, BE_PWM
    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer"))]
    let pwm_fan0_fan1_hotend_hotbed = {
        let pwm_fan0_fan1_hotend_hotbed = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            p.TIM3,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PC6, embassy_stm32::gpio::OutputType::PushPull)), // PA6 | PB4 | PC6
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PC7, embassy_stm32::gpio::OutputType::PushPull)), // PA7 | PB5 | PC7
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch3(p.PC8, embassy_stm32::gpio::OutputType::PushPull)), // PB0 | PC8
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PC9, embassy_stm32::gpio::OutputType::PushPull)), // PB1 | PC9
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
        );
        #[link_section = ".bss"]
        static PWM_INST: TrackedStaticCell<printhor_hwa_common::InterruptControllerMutex<device::PwmFan0Fan1HotendHotbed>> = TrackedStaticCell::new();
        ControllerRef::new(PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>(
            "PwmFanFan0HotendHotbed",
            ControllerMutex::new(pwm_fan0_fan1_hotend_hotbed)
        ))
    };
    #[cfg(feature = "with-fan-layer")]
    let pwm_fan1_channel = embassy_stm32::timer::Channel::Ch2;
    #[cfg(feature = "with-hot-end")]
    let pwm_hotend_channel = embassy_stm32::timer::Channel::Ch3;
    #[cfg(feature = "with-hot-bed")]
    let pwm_hotbed_channel = embassy_stm32::timer::Channel::Ch4;

    #[cfg(any(feature = "with-laser"))]
    let (pwm_laser, pwm_laser_channel) = {
        #[link_section = ".bss"]
        static PWM_LASER_INST: TrackedStaticCell<printhor_hwa_common::InterruptControllerMutex<device::PwmLaser>> = TrackedStaticCell::new();
        (
            ControllerRef::new(PWM_LASER_INST.init::<{crate::MAX_STATIC_MEMORY}>(
                "LaserPwmController",
                ControllerMutex::new(
                    embassy_stm32::timer::simple_pwm::SimplePwm::new(
                        p.TIM1,
                        Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PA8, OutputType::PushPull)),
                        None, //
                        None, //
                        None, //
                        embassy_stm32::time::hz(5_000),
                        embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                    )
                )
            )),
            embassy_stm32::timer::Channel::Ch1
        )
    };

    #[cfg(feature = "with-hot-end")]
    static HOT_END_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_END_THERM_PULL_UP_RESISTANCE, HOT_END_THERM_NOMINAL_RESISTANCE, HOT_END_THERM_BETA);

    #[cfg(feature = "with-hot-bed")]
    static HOT_BED_THERMISTOR_PROPERTIES: printhor_hwa_common::ThermistorProperties = printhor_hwa_common::ThermistorProperties::new(HOT_BED_THERM_PULL_UP_RESISTANCE, HOT_BED_THERM_NOMINAL_RESISTANCE, HOT_BED_THERM_BETA);

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    let adc_hotend_hotbed = {
        let mut adc_hotend_hotbed = device::AdcHotendHotbed::new(p.ADC1);
        adc_hotend_hotbed.set_sample_time(SampleTime::CYCLES7_5);
        #[link_section = ".bss"]
        static ADC_INST: TrackedStaticCell<printhor_hwa_common::InterruptControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();

        ControllerRef::new(ADC_INST.init::<{crate::MAX_STATIC_MEMORY}>(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed)
        ))
    };

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {
            let adc_hotend_pin = p.PA0;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-bed")] {
            let adc_hotbed_pin = p.PC3;
        }
    }

    #[cfg(feature = "with-probe")]
    let probe_device =  {
        #[link_section = ".bss"]
        static PWM_INST: TrackedStaticCell<printhor_hwa_common::InterruptControllerMutex<device::PwmServo>> = TrackedStaticCell::new();
        device::ProbePeripherals {
            power_pwm: ControllerRef::new(
                PWM_INST.init::<{crate::MAX_STATIC_MEMORY}>("PwmServo",
                              ControllerMutex::new(
                                  device::PwmServo::new(
                                      p.TIM2,
                                      None, // PA0 | PA15 | PA5 | PC4
                                      Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PA1, OutputType::PushPull)), // PA1 | PB3 | PC5
                                      None, // PA2 | PB10 | PC6
                                      None, // PA3 | PB11 | PC7
                                      embassy_stm32::time::hz(50),
                                      embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                                  )
                ),
            )),
            power_channel: embassy_stm32::timer::Channel::Ch2,
        }
    };

    #[cfg(feature = "with-motion")]
    defmt::info!("motion_planner done");
    #[cfg(feature = "with-ps-on")]
    let ps_on = {
        #[link_section = ".bss"]
        static PS_ON: TrackedStaticCell<StandardControllerMutex<device::PsOnPin>> = TrackedStaticCell::new();
        ControllerRef::new(
            PS_ON.init::<{crate::MAX_STATIC_MEMORY}>("", ControllerMutex::new(
                Output::new(p.PC13, Level::Low, Speed::Low)
            ))
        )
    };
    #[link_section = ".bss"]
    static WD: TrackedStaticCell<StandardControllerMutex<device::Watchdog>> = TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init::<{crate::MAX_STATIC_MEMORY}>("watchdog", ControllerMutex::new(device::Watchdog::new(p.IWDG, WATCHDOG_TIMEOUT))));

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
        }
    }
}
