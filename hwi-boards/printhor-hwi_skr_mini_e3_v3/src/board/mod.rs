/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32G0B1RE.json
///
pub mod device;
pub mod io;

use alloc_cortex_m::CortexMHeap;
use embassy_executor::Spawner;
use embassy_stm32::Config;
#[cfg(any(feature = "with-uart-port-1", feature = "with-usbserial", feature="with-trinamic"))]
use embassy_stm32::{bind_interrupts};
#[cfg(any(feature = "with-uart-port-1", feature="with-trinamic"))]
use embassy_stm32::usart;
use embassy_stm32::gpio::{Input, Level, Output, Speed, Pull};
#[cfg(any(feature = "with-probe", feature = "with-hotend", feature = "with-hotbed", feature = "with-fan0", feature = "with-fan1"))]
use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::{hz};
use embassy_sync::mutex::Mutex;
#[cfg(any(feature = "with-uart-port-1", feature="with-trinamic"))]
use embassy_stm32::usart::{DataBits, Parity, StopBits};
#[cfg(feature = "with-usbserial")]
use embassy_stm32::usb;
#[cfg(feature = "with-usbserial")]
use device::*;
#[cfg(feature = "with-spi")]
use embassy_stm32::spi;
use printhor_hwa_common::{ControllerMutex, ControllerRef, ControllerMutexType};
use printhor_hwa_common::{TrackedStaticCell, MachineContext};
use embassy_stm32::rcc::*;
#[cfg(feature = "with-trinamic")]
use device::Uart4;
#[cfg(any(feature = "with-probe", feature = "with-hotend", feature = "with-hotbed", feature = "with-fan0", feature = "with-fan1"))]
use embassy_stm32::timer::{CountingMode,simple_pwm::SimplePwm};
#[cfg(feature = "with-motion")]
use device::{MotionDevice, MotionPins};
#[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
use embassy_stm32::adc::SampleTime;

#[global_allocator]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "SKR";
pub const MACHINE_BOARD: &str = "SKR_MINI_E3_V3";
/// ARM Cortex M0+ @64MHZ, 144kB SRAM, 512kB Program
pub const MACHINE_PROCESSOR: &str = "STM32G0B1RE";
#[allow(unused)]
pub(crate) const PROCESSOR_SYS_CK_MHZ: u32 = 64_000_000;
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
#[cfg(feature = "with-spi")]
pub(crate) const SPI_FREQUENCY_HZ: u32 = 2_000_000;

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
    pub sdcard_device: device::SpiCardDeviceRef,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_cs_pin: device::SpiCardCSPin,
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
    #[cfg(feature = "with-fan1")]
    pub fan1: device::Fan1Peripherals,
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
        core::ptr::read_volatile(&printhor_hwa_common::COUNTER) as u32
    }
}

#[inline]
pub fn heap_current_usage_percentage() -> f32 {
    100.0f32 * (heap_current_size() as f32) / (HEAP_SIZE_BYTES as f32)
}

#[cfg(feature = "with-usbserial")]
bind_interrupts!(struct UsbIrqs {
    USB_UCPD1_2 => usb::InterruptHandler<embassy_stm32::peripherals::USB>;
});
#[cfg(feature = "with-uart-port-1")]
bind_interrupts!(struct UartPort1Irqs {
    USART2_LPUART2 => usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});
#[cfg(feature = "with-trinamic")]
bind_interrupts!(struct TrinamicIrqs {
    USART3_4_5_6_LPUART1 => usart::InterruptHandler<embassy_stm32::peripherals::USART4>;
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
    //crate::info!("Initializing...");
    let mut config = Config::default();
    config.rcc.mux = ClockSrc::PLL(
        // 8 / 1 * 24 / 3 = 64 MHz
        PllConfig {
            source: PllSource::HSE(hz(8_000_000)),
            m: Pllm::DIV1,
            n: Plln::MUL24,
            r: Pllr::DIV3,
            q: Some(Pllq::DIV2),
            p: Some(Pllp::DIV2),
        }
    );
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb_pre = APBPrescaler::DIV1;
    config.rcc.ls = LsConfig::default_lsi();
    config.rcc.low_power_run = false;

    embassy_stm32::init(config)
}

pub async fn setup(_spawner: Spawner, p: embassy_stm32::Peripherals) -> printhor_hwa_common::MachineContext<Controllers, IODevices, MotionDevices, PwmDevices> {

    #[cfg(feature = "with-usbserial")]
    let (usbserial_tx, usbserial_rx_stream) = {
        embassy_stm32::pac::RCC.cr().modify(|w| w.set_hsi48on(true));
        while !embassy_stm32::pac::RCC.cr().read().hsi48rdy() {}
        embassy_stm32::pac::RCC.ccipr2().write(|w| {
            w.set_usbsel(embassy_stm32::pac::rcc::vals::Usbsel::HSI48);
        });

        defmt::info!("Creating USB Driver");
        let driver = usb::Driver::new(p.USB, UsbIrqs, p.PA12, p.PA11);
        let mut usb_serial_device = USBSerialDevice::new(driver);
        usb_serial_device.spawn(_spawner);
        let (usb_serial_rx_device, sender) = usb_serial_device.split();
        static USB_INST: TrackedStaticCell<Mutex<ControllerMutexType, device::USBSerialDeviceSender>> = TrackedStaticCell::new();
        let usbserial_tx_controller = ControllerRef::new(
            USB_INST.init("USBSerialTxController", Mutex::<ControllerMutexType, _>::new(sender))
        );
        (usbserial_tx_controller, device::USBSerialDeviceInputStream::new(usb_serial_rx_device))
    };

    #[cfg(feature = "with-uart-port-1")]
    let (uart_port1_tx, uart_port1_rx_stream) = {
        let mut cfg = usart::Config::default();
        cfg.baudrate = crate::UART_PORT1_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;
        cfg.detect_previous_overrun = false;

        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(p.USART2,
                                                            p.PA3, p.PA2,
                                                            UartPort1Irqs,
                                                            p.DMA2_CH2, p.DMA2_CH1,
                                                            cfg).expect("Ready").split();

        static UART_PORT1_INST: TrackedStaticCell<ControllerMutex<device::UartPort1TxDevice>> = TrackedStaticCell::new();
        let uart_port1_tx = ControllerRef::new(
            UART_PORT1_INST.init("UartPort1", Mutex::<ControllerMutexType, _>::new(uart_port1_tx_device))
        );
        (uart_port1_tx, device::UartPort1RxInputStream::new(uart_port1_rx_device))
    };

    #[cfg(all(feature = "with-trinamic"))]
    let trinamic_uart = {
        let mut cfg = usart::Config::default();
        cfg.baudrate = TRINAMIC_UART_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;

        Uart4::new(p.USART4, p.PC11, p.PC10,
                   TrinamicIrqs, p.DMA1_CH7, p.DMA1_CH6,
                   cfg).expect("Ready")
    };

    #[cfg(feature = "with-spi")]
    let spi1_device = {
        let mut cfg = spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
        static SPI1_INST: TrackedStaticCell<ControllerMutex<device::Spi1>> = TrackedStaticCell::new();
        ControllerRef::new(SPI1_INST.init(
            "SPI1",
            ControllerMutex::new(
                device::Spi1::new(p.SPI1, p.PA5, p.PA7, p.PA6,
                                  p.DMA1_CH2, p.DMA1_CH1, cfg
                )
            )
        ))
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

    // PA1(PROBE) PC7(Fan1) PC6(Fan0) PC8(HE_PWM) PC9(BED_PWM) PB15(Fan2) PA8(neo)
    // OK for: NEO
    /*
    ComplementaryPwm::new(
        p.TIM1,
        None, // PA8 | PC8
        None, // PA7 | PB13 | PD2
        None, // PA9 | PB3 | PC9
        None, // PB0 | PB14 | PD3
        None, // PA10 | PB6 | PC10
        None, // PB1 | PB15 | PD4
        None, // PA11 | PC11
        None, //
        hz(5_000)

    );
    */


    // Timmer for drv: 4/6/7
    /*
    SimplePwm::new(
        p.TIM4,
        None, // PB6
        None, // PB7
        None, // PB8
        None, // PB9
        hz(5_000)
    );
     */

    // Timmer for drv: 6/7

    /*
    SimplePwm::new(
        p.TIM14,
        None, // PA4 | PA7 | PB1 | PC12 | PF0
        None, //
        None, //
        None, //
        hz(5_000)
    );
    */

    // PA1(PROBE) PC7(Fan1) PC6(Fan0) PC8(HE_PWM) PC9(BED_PWM) PB15(Fan2) PA8(neo)
    // OK: Fan2
    /*
    let t15 = SimplePwm::new(
        p.TIM15,
        None, // PA2 | PB14 | PC1
        None, // PA3 | PB15 | PC2
        None, //
        None, //
        hz(5_000)
    );
     */
    /*
    let t16 = SimplePwm::new(
        p.TIM16,
        None, // PA6 | PB8 | PD0
        None, //
        None, //
        None, //
        hz(5_000)
    );
    */
/*
    // PA1(PROBE) PC7(Fan1) PC6(Fan0) PC8(HE_PWM) PC9(BED_PWM) PB15(Fan2) PA8(neo)
    let pp = ControllerRef::new(hwi::pwm::init_pwm_tool(PwmTool::new(p.TIM1,
 Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)), // NEO / LASER
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    hz(20_000))));
*/

    #[cfg(feature = "with-motion")]
    let motion_devices = MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart,
        motion_pins: MotionPins {
            x_enable_pin: Output::new(p.PB14, Level::Low, Speed::VeryHigh),
            y_enable_pin: Output::new(p.PB11, Level::Low, Speed::VeryHigh),
            z_enable_pin: Output::new(p.PB1, Level::Low, Speed::VeryHigh),
            e_enable_pin: Output::new(p.PD1, Level::Low, Speed::VeryHigh),
            x_endstop_pin: Input::new(p.PC0, Pull::Down),
            y_endstop_pin: Input::new(p.PC1, Pull::Down),
            z_endstop_pin: Input::new(p.PC2, Pull::Down),
            e_endstop_pin: Input::new(p.PC15, Pull::Down),
            x_step_pin: Output::new(p.PB13, Level::Low, Speed::VeryHigh),
            y_step_pin: Output::new(p.PB10, Level::Low, Speed::VeryHigh),
            z_step_pin: Output::new(p.PB0, Level::Low, Speed::VeryHigh),
            e_step_pin: Output::new(p.PB3, Level::Low, Speed::VeryHigh),
            x_dir_pin: Output::new(p.PB12, Level::Low, Speed::VeryHigh),
            y_dir_pin: Output::new(p.PB2, Level::Low, Speed::VeryHigh),
            z_dir_pin: Output::new(p.PC5, Level::Low, Speed::VeryHigh),
            e_dir_pin: Output::new(p.PB4, Level::Low, Speed::VeryHigh),
        }
    };
    #[cfg(feature = "with-motion")]
    defmt::info!("motion_driver done");

    // PC7(Fan1) PC6(Fan0) PC8(HE_PWM) PC9(BED_PWM) PB15(Fan2) PA8(neo) PA1(Probe)
    // OK FOR: FAN0, FAN1, HE_PWM, BE_PWM
    #[cfg(any(feature = "with-hotend", feature = "with-hotbed", feature = "with-fan0", feature = "with-fan1"))]
    let pwm_fan0_fan1_hotend_hotbed = {
        let pwm_fan0_fan1_hotend_hotbed = SimplePwm::new(
            p.TIM3,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PC6, OutputType::PushPull)), // PA6 | PB4 | PC6
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PC7, OutputType::PushPull)), // PA7 | PB5 | PC7
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch3(p.PC8, OutputType::PushPull)), // PB0 | PC8
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PC9, OutputType::PushPull)), // PB1 | PC9
            hz(5_000),
            CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmFan0Fan1HotendHotbed>> = TrackedStaticCell::new();
        ControllerRef::new(PWM_INST.init(
            "PwmFanFan0HotendHotbed",
            ControllerMutex::new(pwm_fan0_fan1_hotend_hotbed)
        ))
    };

    #[cfg(feature = "with-fan0")]
    let pwm_fan0_channel = embassy_stm32::timer::Channel::Ch1;
    #[cfg(feature = "with-fan1")]
    let pwm_fan1_channel = embassy_stm32::timer::Channel::Ch2;
    #[cfg(feature = "with-hotend")]
    let pwm_hotend_channel = embassy_stm32::timer::Channel::Ch3;
    #[cfg(feature = "with-hotbed")]
    let pwm_hotbed_channel = embassy_stm32::timer::Channel::Ch4;

    #[cfg(any(feature = "with-neo"))]
        let pwm_neo = {
        let pwm_neo = SimplePwm::new(
            p.TIM1,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PA8, OutputType::PushPull)), // PA8
            None, //
            None, //
            None, //
            hz(5_000),
            CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_NEO_INST: TrackedStaticCell<ControllerMutex<device::PwmFan0Fan1HotendHotbed>> = TrackedStaticCell::new();
        ControllerRef::new(PWM_NEO_INST.init(
            "NeoController",
            ControllerMutex::new(pwm_fan0_fan1_hotend_hotbed)
        ))
    };

    #[cfg(any(feature = "with-laser"))]
    let (pwm_laser, pwm_laser_channel) = {
        static PWM_LASER_INST: TrackedStaticCell<ControllerMutex<device::PwmLaser>> = TrackedStaticCell::new();
        (
            ControllerRef::new(PWM_LASER_INST.init(
                "LaserPwmController",
                ControllerMutex::new(
                    SimplePwm::new(
                        p.TIM16,
                        Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PD0, OutputType::PushPull)), // PA6 | PB8 | PD0
                        None, //
                        None, //
                        None, //
                        hz(5_000),
                        CountingMode::CenterAlignedBothInterrupts,
                    )
                )
            )),
            embassy_stm32::timer::Channel::Ch1
        )
    };

    #[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
    let adc_hotend_hotbed = {
        let mut adc_hotend_hotbed = AdcHotendHotbed::new(p.ADC1, &mut embassy_time::Delay);
        adc_hotend_hotbed.set_sample_time(SampleTime::Cycles7_5);
        static ADC_INST: TrackedStaticCell<ControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();

        ControllerRef::new(ADC_INST.init(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed)
        ))
    };
    #[cfg(feature = "with-hotend")]
    let adc_hotend_pin = p.PA0;
    #[cfg(feature = "with-hotbed")]
    let adc_hotbed_pin = p.PC4;

    #[cfg(feature = "with-probe")]
    let probe_device = crate::device::ProbePeripherals {
        #[cfg(feature = "with-probe")]
        probe_pwm: SimplePwm::new(
            p.TIM2,
            None, // PA0 | PA15 | PA5 | PC4
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PA1, OutputType::PushPull)), // PA1 | PB3 | PC5
            None, // PA2 | PB10 | PC6
            None, // PA3 | PB11 | PC7
            hz(50),
            CountingMode::CenterAlignedBothInterrupts,
            ),
        #[cfg(feature = "with-probe")]
        probe_channel: embassy_stm32::timer::Channel::Ch2,
    };

    #[cfg(feature = "with-motion")]
    defmt::info!("motion_planner done");

    static WD: TrackedStaticCell<ControllerMutex<device::Watchdog>> = TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init("watchdog", ControllerMutex::new(device::Watchdog::new(p.IWDG, WATCHDOG_TIMEOUT))));

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
            #[cfg(feature = "with-hotend")]
            hotend: HotendPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_hotend_channel,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: adc_hotend_pin,
            },
            #[cfg(feature = "with-hotbed")]
            hotbed: HotbedPeripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_hotbed_channel,
                temp_adc: adc_hotend_hotbed.clone(),
                temp_pin: adc_hotbed_pin,
            },
            #[cfg(feature = "with-fan0")]
            fan0: Fan0Peripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_fan0_channel,
            },
            #[cfg(feature = "with-fan1")]
            fan1: Fan1Peripherals {
                power_pwm: pwm_fan0_fan1_hotend_hotbed.clone(),
                power_channel: pwm_fan1_channel,
            },
            #[cfg(feature = "with-laser")]
            laser: LaserPeripherals {
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