//noinspection RsDetachedFile
/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32L476RG.json
///
pub mod device;
pub mod io;

use alloc_cortex_m::CortexMHeap;
use embassy_executor::Spawner;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::Config;

#[cfg(feature = "with-spi")]
use embassy_stm32::spi;
#[cfg(any(feature = "with-serial-port-1", feature = "with-trinamic"))]
use embassy_stm32::usart::{DataBits, Parity, StopBits};
#[allow(unused)]
use printhor_hwa_common::{ControllerMutex, ControllerMutexType, ControllerRef};
use printhor_hwa_common::{InterruptControllerMutex, MachineContext, TrackedStaticCell};

#[global_allocator]
#[link_section = ".bss"]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "NUCLEO64";
pub const MACHINE_BOARD: &str = "NUCLEO64_Arduino_CNC_Hat_v3.x";

/// ARM Cortex M4F @80MHZ, 128kB SRAM, 1024kB Program
pub const MACHINE_PROCESSOR: &str = "STM32L476RG";
pub const PROCESSOR_SYS_CK_MHZ: u32 = 80_000_000;
// http://www.st.com/resource/en/datasheet/DM00108832.pdf
pub const ADC_START_TIME_US: u16 = 12;
// http://www.st.com/resource/en/datasheet/DM00108832.pdf
pub const ADC_VREF_DEFAULT_MV: u16 = 1212;

/// Micro-segment sampling frequency in Hz
pub const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 100;
/// Micro-segment clock frequency in Hz
pub const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;

pub const HEAP_SIZE_BYTES: usize = 1024;
pub const MAX_STATIC_MEMORY: usize = 4096;
#[cfg(feature = "with-sd-card")]
pub const SDCARD_PARTITION: usize = 0;
pub const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;
#[cfg(feature = "with-spi")]
pub const SPI_FREQUENCY_HZ: u32 = 2_000_000;

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
    pub sys_watchdog: ControllerRef<ControllerMutexType, device::Watchdog>,
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
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_rx_stream: device::UartPort1RxInputStream,
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
    #[cfg(feature = "with-laser")]
    pub laser: device::LaserPeripherals,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer: device::FanLayerPeripherals,
    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra1: device::FanExtra1Peripherals,
}

pub struct MotionDevices {
    #[cfg(feature = "with-motion")]
    pub motion_devices: device::MotionDevice,
}

pub fn heap_current_size() -> u32 {
    HEAP.used() as u32
}

pub fn stack_reservation_current_size() -> u32 {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(printhor_hwa_common::COUNTER)) as u32 }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        embassy_stm32::bind_interrupts!(struct UsbIrqs {
            OTG_FS => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
        });
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        embassy_stm32::bind_interrupts!(struct UartPort1Irqs {
            USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
        });
    }
}

#[inline]
pub(crate) fn init_heap() -> () {
    use core::mem::MaybeUninit;
    #[link_section = ".bss"]
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE_BYTES] =
        [MaybeUninit::uninit(); HEAP_SIZE_BYTES];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE_BYTES) }
}

#[inline]
pub fn init() -> embassy_stm32::Peripherals {
    init_heap();

    let config = {
        let mut config = Config::default();

        config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;

        /*
        config.rcc.hsi = false;
        config.rcc.hse = Some(embassy_stm32::rcc::Hse {
            freq: embassy_stm32::time::Hertz(24_000_000),
            mode: embassy_stm32::rcc::HseMode::Oscillator,
        });
        config.rcc.pll = Some(embassy_stm32::rcc::Pll {
            source: embassy_stm32::rcc::PllSource::HSI,
            prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
            mul: embassy_stm32::rcc::PllMul::MUL20,
            divp: None,
            divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLL1_R 80Mhz (8 / 1 * 20 / 2)
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLQ 80Mhz (8 / 1 * 20 / 2)
        });
        config.rcc.pllsai1 = Some(embassy_stm32::rcc::Pll {
            source: embassy_stm32::rcc::PllSource::HSE,
            prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
            mul: embassy_stm32::rcc::PllMul::MUL12,
            divp: None,
            divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLLSAl1R 48Mhz (8 / 1 * 12 / 2)
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLSAl1Q 48Mhz (8 / 1 * 12 / 2)
        });
         */

        config.rcc.hsi = true;
        config.rcc.pll = Some(embassy_stm32::rcc::Pll {
            source: embassy_stm32::rcc::PllSource::HSI,
            prediv: embassy_stm32::rcc::PllPreDiv::DIV2,
            mul: embassy_stm32::rcc::PllMul::MUL20,
            divp: None,
            divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLL1_R 80Mhz (16 / 2 * 20 / 2)
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLQ 80Mhz (16 / 2 * 20 / 2)
        });
        config.rcc.pllsai1 = Some(embassy_stm32::rcc::Pll {
            source: embassy_stm32::rcc::PllSource::HSI,
            prediv: embassy_stm32::rcc::PllPreDiv::DIV2,
            mul: embassy_stm32::rcc::PllMul::MUL12,
            divp: None,
            divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLLSAl1R 48Mhz (16 / 2 * 12 / 2)
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLSAl1Q 48Mhz (16 / 2 * 12 / 2)
        });

        config.rcc.mux.clk48sel = embassy_stm32::rcc::mux::Clk48sel::PLLSAI1_Q;
        config.rcc.mux.adcsel = embassy_stm32::rcc::mux::Adcsel::SYS;
        config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
        config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
        config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;

        config
    };
    defmt::info!("embassy init...");
    let p = embassy_stm32::init(config);
    defmt::info!("embassy init done");
    p
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
    defmt::info!("Setting up...");
    //embassy_time::Timer::after_secs(5).await;

    #[cfg(feature = "with-serial-usb")]
    let (serial_usb_tx, serial_usb_rx_stream) = {
        defmt::info!("USB...");
        #[link_section = ".bss"]
        static EP_OUT_BUFFER_INST: TrackedStaticCell<[u8; 256]> = TrackedStaticCell::new();
        let ep_out_buffer = EP_OUT_BUFFER_INST.init::<MAX_STATIC_MEMORY>("USBEPBuffer", [0u8; 256]);
        defmt::info!("Creating USB Driver");
        let mut config = embassy_stm32::usb::Config::default();
        config.vbus_detection = true;

        // Maybe OTG_FS is not the right peripheral...
        // USB_OTG_FS is DM=PB14, DP=PB15
        let driver = embassy_stm32::usb::Driver::new_fs(
            p.USB_OTG_FS,
            UsbIrqs,
            p.PA12,
            p.PA11,
            ep_out_buffer,
            config,
        );
        let mut usb_serial_device = io::usbserial::USBSerialDevice::new(driver);
        usb_serial_device.spawn(_spawner);
        let (usb_serial_rx_device, sender) = usb_serial_device.split();
        #[link_section = ".bss"]
        static USB_INST: TrackedStaticCell<
            InterruptControllerMutex<device::USBSerialDeviceSender>,
        > = TrackedStaticCell::new();
        let serial_usb_tx = ControllerRef::new(
            USB_INST
                .init::<MAX_STATIC_MEMORY>("USBSerialTxController", ControllerMutex::new(sender)),
        );
        (
            serial_usb_tx,
            device::USBSerialDeviceInputStream::new(usb_serial_rx_device),
        )
    };

    #[cfg(feature = "with-serial-port-1")]
    let (serial_port1_tx, serial_port1_rx_stream) = {
        defmt::info!("Serial port 1...");
        let mut cfg = embassy_stm32::usart::Config::default();
        cfg.baudrate = crate::UART_PORT1_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;
        cfg.detect_previous_overrun = false;

        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(
            p.USART2,
            p.PA3,
            p.PA2,
            UartPort1Irqs,
            p.DMA1_CH7,
            p.DMA1_CH6,
            cfg,
        )
        .expect("Ready")
        .split();

        #[link_section = ".bss"]
        static UART_PORT1_INST: TrackedStaticCell<
            ControllerMutex<
                ControllerMutexType,
                printhor_hwa_common::SerialAsyncWrapper<device::UartPort1TxDevice>,
            >,
        > = TrackedStaticCell::new();
        let serial_port1_tx =
            ControllerRef::new(UART_PORT1_INST.init::<{ crate::MAX_STATIC_MEMORY }>(
                "UartPort1",
                ControllerMutex::new(printhor_hwa_common::SerialAsyncWrapper::new(
                    uart_port1_tx_device,
                    crate::UART_PORT1_BAUD_RATE,
                )),
            ));
        defmt::info!("seems legit");
        (
            serial_port1_tx,
            device::UartPort1RxInputStream::new(uart_port1_rx_device),
        )
    };

    #[cfg(feature = "with-spi")]
    let spi1_device = {
        defmt::info!("SPI...");
        let mut cfg = spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
        #[link_section = ".bss"]
        static SPI_INST: TrackedStaticCell<InterruptControllerMutex<device::SpiCardDevice>> =
            TrackedStaticCell::new();
        ControllerRef::new(SPI_INST.init::<{ crate::MAX_STATIC_MEMORY }>(
            "SPI",
            ControllerMutex::new(device::SpiCardDevice::new(
                p.SPI3, p.PC10, p.PC12, p.PC11, p.DMA2_CH2, p.DMA2_CH1, cfg,
            )),
        ))
    };

    #[cfg(feature = "with-sd-card")]
    let (sd_card_device, sd_card_cs_pin) = {
        (
            spi1_device.clone(),
            Output::new(p.PC4, Level::High, Speed::VeryHigh),
        )
    };
    #[cfg(feature = "with-sd-card")]
    defmt::info!("card_controller done");

    #[cfg(feature = "with-motion")]
    let motion_devices = device::MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart,
        motion_pins: device::MotionPins {
            all_enable_pin: Output::new(p.PA9, Level::Low, Speed::VeryHigh),
            x_endstop_pin: Input::new(p.PC7, Pull::Down),
            y_endstop_pin: Input::new(p.PB6, Pull::Down),
            z_endstop_pin: Input::new(p.PA7, Pull::Down),
            x_step_pin: Output::new(p.PA10, Level::Low, Speed::VeryHigh),
            y_step_pin: Output::new(p.PB3, Level::Low, Speed::VeryHigh),
            z_step_pin: Output::new(p.PB5, Level::Low, Speed::VeryHigh),
            x_dir_pin: Output::new(p.PB4, Level::Low, Speed::VeryHigh),
            y_dir_pin: Output::new(p.PB10, Level::Low, Speed::VeryHigh),
            z_dir_pin: Output::new(p.PA8, Level::Low, Speed::VeryHigh),
        },
    };

    #[cfg(feature = "with-motion")]
    defmt::info!("motion_driver done");

    #[cfg(feature = "with-probe")]
    let probe_device = {
        #[link_section = ".bss"]
        static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmServo>> =
            TrackedStaticCell::new();
        crate::device::ProbePeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init::<{ crate::MAX_STATIC_MEMORY }>(
                "PwmServo",
                ControllerMutex::new(device::PwmServo::new(
                    p.TIM3,
                    None,
                    None,
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch3(
                        p.PC8,
                        embassy_stm32::gpio::OutputType::PushPull,
                    )),
                    None,
                    embassy_stm32::time::hz(50),
                    embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                )),
            )),
            power_channel: embassy_stm32::timer::Channel::Ch3,
        }
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

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
            let adc = {
                let mut adc_hotend_hotbed = device::AdcHotendHotbed::new(p.ADC1);
                adc_hotend_hotbed.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES12_5);
                #[link_section = ".bss"]
                static ADC_INST: TrackedStaticCell<InterruptControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();
                ControllerRef::new(ADC_INST.init::<{MAX_STATIC_MEMORY}>(
                    "HotendHotbedAdc",
                    ControllerMutex::new(adc_hotend_hotbed)
                ))
            };
        }
    }

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    let pwm_hotend_hotbed = {
        let pwm = device::PwmHotendHotbed::new(
            p.TIM15,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(
                p.PB14,
                embassy_stm32::gpio::OutputType::PushPull,
            )),
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(
                p.PB15,
                embassy_stm32::gpio::OutputType::PushPull,
            )),
            None,
            None,
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
        );
        #[link_section = ".bss"]
        static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmHotendHotbed>> =
            TrackedStaticCell::new();

        ControllerRef::new(
            PWM_INST
                .init::<{ crate::MAX_STATIC_MEMORY }>("PwmHotendHotbed", ControllerMutex::new(pwm)),
        )
    };

    #[cfg(feature = "with-hot-end")]
    let hotend_device = {
        device::HotendPeripherals {
            power_pwm: pwm_hotend_hotbed.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PC2,
            thermistor_properties: &HOT_END_THERMISTOR_PROPERTIES,
        }
    };

    #[cfg(feature = "with-hot-bed")]
    let hotbed_device = {
        device::HotbedPeripherals {
            power_pwm: pwm_hotend_hotbed.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch2,
            temp_adc: adc.clone(),
            temp_pin: p.PC3,
            thermistor_properties: &HOT_BED_THERMISTOR_PROPERTIES,
        }
    };

    #[cfg(feature = "with-laser")]
    let laser_device = {
        let pwm = device::PwmLaser::new(
            p.TIM8,
            None,
            None,
            None,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(
                p.PC9,
                embassy_stm32::gpio::OutputType::PushPull,
            )),
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
        );
        #[link_section = ".bss"]
        static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmLaser>> =
            TrackedStaticCell::new();

        device::LaserPeripherals {
            power_pwm: ControllerRef::new(
                PWM_INST
                    .init::<{ crate::MAX_STATIC_MEMORY }>("PwmLaser", ControllerMutex::new(pwm)),
            ),
            power_channel: embassy_stm32::timer::Channel::Ch4,
        }
    };

    #[cfg(feature = "with-fan-layer")]
    let fan_layer_device = {
        let pwm = device::PwmFanLayer::new(
            p.TIM2,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(
                p.PA15,
                embassy_stm32::gpio::OutputType::PushPull,
            )),
            None,
            None,
            None,
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
        );
        #[link_section = ".bss"]
        static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmFanLayer>> =
            TrackedStaticCell::new();

        device::FanLayerPeripherals {
            power_pwm: ControllerRef::new(
                PWM_INST
                    .init::<{ crate::MAX_STATIC_MEMORY }>("PwmLayer", ControllerMutex::new(pwm)),
            ),
            power_channel: embassy_stm32::timer::Channel::Ch1,
        }
    };

    #[cfg(feature = "with-ps-on")]
    let ps_on = {
        #[link_section = ".bss"]
        static PS_ON: TrackedStaticCell<ControllerMutex<ControllerMutexType, device::PsOnPin>> =
            TrackedStaticCell::new();
        ControllerRef::new(PS_ON.init::<{ crate::MAX_STATIC_MEMORY }>(
            "",
            ControllerMutex::new(Output::new(p.PA4, Level::Low, Speed::Low)),
        ))
    };

    defmt::info!("watchdog...");

    #[link_section = ".bss"]
    static WD: TrackedStaticCell<ControllerMutex<ControllerMutexType, device::Watchdog>> =
        TrackedStaticCell::new();
    let sys_watchdog = ControllerRef::new(WD.init::<{ crate::MAX_STATIC_MEMORY }>(
        "watchdog",
        ControllerMutex::new(device::Watchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US)),
    ));

    defmt::info!("Setup timer...");
    crate::setup_timer();

    defmt::info!("All done");
    MachineContext {
        controllers: Controllers {
            sys_watchdog,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
        },
        sys_devices: SysDevices {
            task_stepper_core: printhor_hwa_common::NoDevice::new(),
            #[cfg(feature = "with-ps-on")]
            ps_on,
        },
        io_devices: IODevices {
            #[cfg(feature = "with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_rx_stream,
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
            hotend: hotend_device,
            #[cfg(feature = "with-hot-bed")]
            hotbed: hotbed_device,
            #[cfg(feature = "with-laser")]
            laser: laser_device,
            #[cfg(feature = "with-fan-layer")]
            fan_layer: fan_layer_device,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1: fan_extra1_device,
        },
    }
}
