/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32F410RB.json
///
#[allow(unused)]
use printhor_hwa_common as hwa;
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

#[global_allocator]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "NUCLEO64";
pub const MACHINE_BOARD: &str = "NUCLEO64_Arduino_CNC_Hat_v3.x";

/// ARM Cortex M4F @100MHZ, 32kB SRAM, 128kB Program
pub const MACHINE_PROCESSOR: &str = "STM32F410RB";
#[allow(unused)]
pub const PROCESSOR_SYS_CK_MHZ: u32 = 100_000_000;
// https://www.st.com/resource/en/datasheet/DM00214043.pdf
pub const ADC_START_TIME_US: u16 = 10;
// https://www.st.com/resource/en/datasheet/DM00214043.pdf
pub const ADC_VREF_DEFAULT_MV: u16 = 1210;
/// Micro-segment sampling frequency in Hz
pub const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 400;
pub const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 40_000;

pub const HEAP_SIZE_BYTES: usize = 256;
pub const MAX_STATIC_MEMORY: usize = 4096;
#[cfg(feature = "with-sd-card")]
pub const SDCARD_PARTITION: usize = 0;
pub(crate) const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;
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
    pub sys_watchdog: hwa::StaticController<crate::WatchdogMutexType, device::Watchdog>,
    #[cfg(feature = "with-serial-usb")]
    pub serial_usb_tx: device::USBSerialTxControllerRef,
    #[cfg(feature = "with-serial-port-1")]
    pub serial_port1_tx:
        hwa::StaticController<crate::SerialPort1MutexType, device::SerialPort1TxDevice>,
}

pub struct SysDevices {
    #[cfg(feature = "with-motion")]
    pub task_stepper_core: hwa::NoDevice,
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
    pub sd-card_cs_pin: device::SpiCardCSPin,
}

pub struct PwmDevices {
    #[cfg(feature = "with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature = "with-hot-end")]
    pub hot_end: device::HotEndPeripherals,
    #[cfg(feature = "with-hot-bed")]
    pub hot_bed: device::HotBedPeripherals,
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

#[inline]
pub fn stack_reservation_current_size() -> u32 {
    hwa::stack_allocation_get() as u32
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        embassy_stm32::bind_interrupts!(struct UsbIrqs {
            OTG_FS => embassy_stm32::usb_otg::InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
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

pub fn init() -> embassy_stm32::Peripherals {
    init_heap();

    let config = {
        let mut config = Config::default();
        config.rcc.hsi = true;
        config.rcc.hse = None;
        config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;
        config.rcc.pll_src = embassy_stm32::rcc::PllSource::HSI;
        config.rcc.pll = Some(embassy_stm32::rcc::Pll {
            prediv: embassy_stm32::rcc::PllPreDiv::DIV16,
            mul: embassy_stm32::rcc::PllMul::MUL200,
            divp: Some(embassy_stm32::rcc::PllPDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
        config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV2;
        config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
        config
    };
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
    //defmt::warn!("Wait a few...");
    //embassy_time::Timer::after_secs(5).await;

    #[cfg(feature = "with-serial-usb")]
    let (serial_usb_tx, serial_usb_rx_stream) = {
        #[link_section = ".bss"]
        static EP_OUT_BUFFER_INST: TrackedStaticCell<[u8; 256]> = TrackedStaticCell::new();
        let ep_out_buffer = EP_OUT_BUFFER_INST.init("USBEPBuffer", [0u8; 256]);
        defmt::info!("Creating USB Driver");
        let mut config = embassy_stm32::usb_otg::Config::default();
        config.vbus_detection = true;

        // Maybe OTG_FS is not the right peripheral...
        // USB_OTG_FS is DM=PB14, DP=PB15
        let driver = embassy_stm32::usb_otg::Driver::new_fs(
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
        static USB_INST: TrackedStaticCell<ControllerMutex<device::USBSerialDeviceSender>> =
            TrackedStaticCell::new();
        let serial_usb_tx = ControllerRef::new(
            USB_INST.init("USBSerialTxController", ControllerMutex::new(sender)),
        );
        (
            serial_usb_tx,
            device::USBSerialDeviceInputStream::new(usb_serial_rx_device),
        )
    };

    #[cfg(feature = "with-serial-port-1")]
    let (serial_port1_tx, serial_port1_rx_stream) = {
        let mut cfg = embassy_stm32::usart::Config::default();
        cfg.baudrate = crate::UART_PORT1_BAUD_RATE;
        cfg.data_bits = DataBits::DataBits8;
        cfg.stop_bits = StopBits::STOP1;
        cfg.parity = Parity::ParityNone;
        cfg.detect_previous_overrun = true;

        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(
            p.USART2,
            p.PA3,
            p.PA2,
            UartPort1Irqs,
            p.DMA1_CH6,
            p.DMA1_CH7,
            cfg,
        )
        .expect("Ready")
        .split();

        let serial_port1_tx = hwa::make_static_controller!(
            "UartPort1",
            crate::SerialPort1MutexType,
            hwa::SerialAsyncWrapper<device::UartPort1TxDevice>,
            hwa::SerialAsyncWrapper::new(uart_port1_tx_device, crate::UART_PORT1_BAUD_RATE)
        );

        (
            serial_port1_tx,
            device::UartPort1RxInputStream::new(uart_port1_rx_device),
        )
    };

    #[cfg(feature = "with-spi")]
    let spi1_device = {
        let mut cfg = spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
        #[link_section = ".bss"]
        static SPI_INST: TrackedStaticCell<InterruptControllerMutex<device::Spi1>> =
            TrackedStaticCell::new();

        let spi = ControllerRef::new(SPI_INST.init::<MAX_STATIC_MEMORY>(
            "SPI",
            ControllerMutex::new(device::Spi1::new(
                p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH4, p.DMA1_CH3, cfg,
            )),
        ));
        spi
    };
    #[cfg(feature = "with-spi")]
    defmt::info!("SPI done");

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

    cfg_if::cfg_if! {
        if #[cfg(feature ="with-probe")] {

            let probe_device = {
                #[link_section = ".bss"]
                static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmServo>> = TrackedStaticCell::new();
                device::ProbePeripherals {
                    power_pwm: ControllerRef::new(
                        PWM_INST.init::<MAX_STATIC_MEMORY>("PwmServo",
                                      ControllerMutex::new(
                                          device::PwmServo::new(
                                              p.TIM11,
                                              Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PB9, embassy_stm32::gpio::OutputType::PushPull)),
                                              None,
                                              None,
                                              None,
                                              embassy_stm32::time::hz(50),
                                              embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                                          )
                                      ),
                        )),
                    power_channel: embassy_stm32::timer::Channel::Ch1,
                }
            };
        }
    }

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
                adc_hotend_hotbed.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES15);
                #[link_section = ".bss"]
                static ADC_INST: TrackedStaticCell<InterruptControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();
                ControllerRef::new(ADC_INST.init::<{MAX_STATIC_MEMORY}>(
                    "HotendHotbedAdc",
                    ControllerMutex::new(adc_hotend_hotbed)
                ))
            };
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer", feature = "with-fan-extra-1"))] {

            let pwm_hotend_hotbed_layer = {
                let pwm = device::PwmHotendHotbedLayer::new(
                    p.TIM5,
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PB12, embassy_stm32::gpio::OutputType::PushPull)),
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PA1, embassy_stm32::gpio::OutputType::PushPull)),
                    None,
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PB11, embassy_stm32::gpio::OutputType::PushPull)),
                    embassy_stm32::time::hz(5_000),
                    embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                );
                #[link_section = ".bss"]
                static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmHotendHotbedLayer>> = TrackedStaticCell::new();

                ControllerRef::new(PWM_INST.init::<MAX_STATIC_MEMORY>(
                    "PwmHotendHotbedLayer",
                    ControllerMutex::new(pwm)
                ))
            };
        }
    }

    #[cfg(feature = "with-hot-end")]
    let hotend_device = {
        device::HotendPeripherals {
            power_pwm: pwm_hotend_hotbed_layer.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PB0,
            thermistor_properties: &HOT_END_THERMISTOR_PROPERTIES,
        }
    };

    #[cfg(feature = "with-hot-bed")]
    let hotbed_device = {
        device::HotbedPeripherals {
            power_pwm: pwm_hotend_hotbed_layer.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch4,
            temp_adc: adc.clone(),
            temp_pin: p.PB1,
            thermistor_properties: &HOT_BED_THERMISTOR_PROPERTIES,
        }
    };

    cfg_if::cfg_if! {
        if #[cfg(feature ="with-laser")] {
            let laser_device = {
                let pwm = device::PwmLaser::new(
                    p.TIM1,
                    None,
                    None,
                    None,
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PA11, embassy_stm32::gpio::OutputType::PushPull)),
                    embassy_stm32::time::hz(5_000),
                    embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                );
                #[link_section = ".bss"]
                static PWM_INST: TrackedStaticCell<InterruptControllerMutex<device::PwmLaser>> = TrackedStaticCell::new();

                device::LaserPeripherals {
                    power_pwm: ControllerRef::new(PWM_INST.init::<MAX_STATIC_MEMORY>(
                        "PwmLaser",
                        ControllerMutex::new(pwm)
                    )),
                    power_channel: embassy_stm32::timer::Channel::Ch4,
                }
            };
        }
    }

    #[cfg(feature = "with-fan-layer")]
    let fan_layer_device = {
        device::FanLayerPeripherals {
            power_pwm: pwm_hotend_hotbed_layer.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch2,
        }
    };

    #[cfg(feature = "with-ps-on")]
    let ps_on = {
        #[link_section = ".bss"]
        static PS_ON: TrackedStaticCell<ControllerMutex<ControllerMutexType, device::PsOnPin>> =
            TrackedStaticCell::new();
        ControllerRef::new(PS_ON.init::<{ MAX_STATIC_MEMORY }>(
            "PSOn",
            ControllerMutex::new(Output::new(p.PA4, Level::Low, Speed::Low)),
        ))
    };

    let sys_watchdog = hwa::make_static_controller!(
        "Watchdog",
        crate::WatchdogMutexType,
        device::Watchdog,
        device::Watchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US)
    );

    crate::setup_timer();

    hwa::MachineContext {
        controllers: Controllers {
            sys_watchdog,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
        },
        sys_devices: SysDevices {
            #[cfg(feature = "with-motion")]
            task_stepper_core: hwa::NoDevice::new(),
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
