/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32F410RB.json
/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32L476RG.json
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
#[allow(unused)]
use embassy_stm32::gpio::{Input, Level, Output, Speed, Pull};

use embassy_sync::mutex::Mutex;
#[cfg(any(feature = "with-uart-port-1", feature="with-trinamic"))]
use embassy_stm32::usart::{DataBits, Parity, StopBits};
#[cfg(feature = "with-usbserial")]
use embassy_stm32::usb_otg;
#[cfg(feature = "with-usbserial")]
use device::*;
#[cfg(feature = "with-spi")]
use embassy_stm32::spi;
#[allow(unused)]
use embassy_stm32::exti::ExtiInput;
use printhor_hwa_common::{ControllerMutex, ControllerRef, ControllerMutexType};
use printhor_hwa_common::{TrackedStaticCell, MachineContext};


#[cfg(feature = "with-motion")]
use device::{MotionDevice, MotionPins};

#[global_allocator]
static HEAP: CortexMHeap = CortexMHeap::empty();

pub const MACHINE_TYPE: &str = "NUCLEO64";
pub const MACHINE_BOARD: &str = "NUCLEO64_Arduino_CNC_Hat_v3.x";

#[cfg(feature="nucleo64-f410rb")]
/// ARM Cortex M4F @100MHZ, 32kB SRAM, 128kB Program
pub const MACHINE_PROCESSOR: &str = "STM32F410RB";
#[cfg(feature="nucleo64-l476rg")]
/// ARM Cortex M4F @80MHZ, 128kB SRAM, 1024kB Program
pub const MACHINE_PROCESSOR: &str = "STM32L476RG";
#[cfg(feature="nucleo64-f410rb")]
#[allow(unused)]
pub const PROCESSOR_SYS_CK_MHZ: &str = "100_000_000";
#[cfg(feature="nucleo64-l476rg")]
#[allow(unused)]
pub const PROCESSOR_SYS_CK_MHZ: &str = "80_000_000";

pub const HEAP_SIZE_BYTES: usize = 1024;
pub const MAX_STATIC_MEMORY: u32 = 4096;
pub const VREF_SAMPLE: u16 = 1210u16;
#[cfg(feature = "with-sdcard")]
pub const SDCARD_PARTITION: usize = 0;
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
    #[cfg(feature = "with-sdcard")]
    pub sdcard_device: device::SpiCardDeviceRef,
    #[cfg(feature = "with-sdcard")]
    pub sdcard_cs_pin: device::SpiCardCSPin,
}

pub struct PwmDevices {
    #[cfg(feature="with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature="with-hotend")]
    pub hotend: device::HotendPeripherals,
    #[cfg(feature="with-hotbed")]
    pub hotbed: device::HotbedPeripherals,
    #[cfg(feature="with-laser")]
    pub laser: device::LaserPeripherals,
    #[cfg(feature="with-layer-fan")]
    pub layer_fan: device::LayerFanPeripherals,
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
    OTG_FS => usb_otg::InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
});
#[cfg(all(feature = "nucleo64-l476rg", feature = "with-uart-port-1"))]
bind_interrupts!(struct UartPort1Irqs {
    USART2 => usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});
#[cfg(all(feature = "nucleo64-f410rb", feature = "with-uart-port-1"))]
bind_interrupts!(struct UartPort1Irqs {
    USART2 => usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
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
    #[cfg(feature="nucleo64-f410rb")]
        let config = {
        let mut config = Config::default();
        config.rcc.hsi = true;
        config.rcc.hse = None;
        config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;
        config.rcc.pll_src = embassy_stm32::rcc::PllSource::HSI;
        config.rcc.pll = Some(embassy_stm32::rcc::Pll{
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
    #[cfg(feature="nucleo64-l476rg")]
    let config = {
        let mut config = Config::default();

        config.rcc.mux = embassy_stm32::rcc::ClockSrc::PLL1_R;
        config.rcc.hsi = true;
        /*
        config.rcc.hse = None;
        config.rcc.hse = Some(embassy_stm32::rcc::Hse {
            freq: embassy_stm32::time::Hertz(24_000_000),
            mode: embassy_stm32::rcc::HseMode::Oscillator,
        });
         */
        config.rcc.pll = Some(embassy_stm32::rcc::Pll {
            source: embassy_stm32::rcc::PllSource::HSI,
            prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
            mul: embassy_stm32::rcc::PllMul::MUL10,
            divp: None, //Some(embassy_stm32::rcc::PllPDiv::DIV2),
            divq: None, // Some(embassy_stm32::rcc::PllQDiv::DIV2),
            divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // sysclk 80Mhz (16 / 1 * 10 / 2)
        });
        config
    };
    embassy_stm32::init(config)
}

pub async fn setup(_spawner: Spawner, p: embassy_stm32::Peripherals) -> printhor_hwa_common::MachineContext<Controllers, IODevices, MotionDevices, PwmDevices> {

    #[cfg(feature = "with-usbserial")]
    let (usbserial_tx, usbserial_rx_stream) = {
        static EP_OUT_BUFFER_INST: TrackedStaticCell<[u8; 256]> =  TrackedStaticCell::new();
        let ep_out_buffer = EP_OUT_BUFFER_INST.init("USBEPBuffer", [0u8; 256]);
        defmt::info!("Creating USB Driver");
        let mut config = embassy_stm32::usb_otg::Config::default();
        config.vbus_detection = true;

                // Maybe OTG_FS is not the right peripheral...
        // USB_OTG_FS is DM=PB14, DP=PB15
        let driver = usb_otg::Driver::new_fs(p.USB_OTG_FS, UsbIrqs, p.PA12, p.PA11,  ep_out_buffer, config);
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
        cfg.detect_previous_overrun = true;

        /*
        static RXB: TrackedStaticCell<[u8; 32]> =  TrackedStaticCell::new();
        let rxb = RXB.init("RXBuffer", [0u8; 32]);
        static TXB: TrackedStaticCell<[u8; 32]> =  TrackedStaticCell::new();
        let txb = TXB.init("TXBuffer", [0u8; 32]);
         */

        #[cfg(feature = "nucleo64-f410rb")]
        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(
            p.USART2, p.PA3, p.PA2, UartPort1Irqs, p.DMA1_CH6, p.DMA1_CH5, cfg
        ).expect("Ready").split();
        #[cfg(feature = "nucleo64-l476rg")]
        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(
            p.USART2, p.PA3, p.PA2, UartPort1Irqs,
            p.DMA1_CH7, p.DMA1_CH6, cfg
        ).expect("Ready").split();

        static UART_PORT1_INST: TrackedStaticCell<ControllerMutex<device::UartPort1TxDevice>> = TrackedStaticCell::new();
        let uart_port1_tx = ControllerRef::new(
            UART_PORT1_INST.init("UartPort1", Mutex::<ControllerMutexType, _>::new(uart_port1_tx_device))
        );
        (uart_port1_tx, device::UartPort1RxInputStream::new(uart_port1_rx_device))
    };

    #[cfg(feature = "with-spi")]
    let spi1_device = {
        let mut cfg = spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
        static SPI_INST: TrackedStaticCell<ControllerMutex<device::Spi1>> = TrackedStaticCell::new();
        #[cfg(feature="nucleo64-l476rg")]
        let spi = ControllerRef::new(SPI_INST.init(
            "SPI",
            ControllerMutex::new(
                device::Spi1::new(p.SPI3, p.PC10, p.PC12, p.PC11,
                                  p.DMA2_CH2, p.DMA2_CH1, cfg
                )
            )
        ));
        #[cfg(feature="nucleo64-f410rb")]
        let spi = ControllerRef::new(SPI_INST.init(
            "SPI",
            ControllerMutex::new(
                device::Spi1::new(p.SPI2, p.PB13, p.PB15, p.PB14,
                                  p.DMA1_CH4, p.DMA1_CH3, cfg
                )
            )
        ));
        spi
    };
    #[cfg(feature = "with-spi")]
    defmt::info!("SPI done");

    #[cfg(feature = "with-sdcard")]
    let (sdcard_device, sdcard_cs_pin) = {
        (spi1_device.clone(), Output::new(p.PC4, Level::High, Speed::VeryHigh))
    };
    #[cfg(feature = "with-sdcard")]
    defmt::info!("card_controller done");

    #[cfg(feature = "with-motion")]
    let motion_devices = MotionDevice {
        #[cfg(feature = "with-trinamic")]
        trinamic_uart,
        motion_pins: MotionPins {
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
        }
    };

    #[cfg(feature = "with-motion")]
    defmt::info!("motion_driver done");

    #[cfg(all(feature = "with-probe", feature ="nucleo64-l476rg"))]
        let probe_device = {
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmServo>> = TrackedStaticCell::new();
        crate::device::ProbePeripherals {
            power_pwm: ControllerRef::new(
                PWM_INST.init("PwmServo",
                              ControllerMutex::new(
                                  device::PwmServo::new(
                                      p.TIM3,
                                      None,
                                      None,
                                      Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch3(p.PC8, embassy_stm32::gpio::OutputType::PushPull)),
                                      None,
                                      embassy_stm32::time::hz(50),
                                      embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
                                  )
                              ),
                )),
            power_channel: embassy_stm32::timer::Channel::Ch3,
        }
    };
    #[cfg(all(feature = "with-probe", feature ="nucleo64-f410rb"))]
        let probe_device = {
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmServo>> = TrackedStaticCell::new();
        crate::device::ProbePeripherals {
            power_pwm: ControllerRef::new(
                PWM_INST.init("PwmServo",
                              ControllerMutex::new(
                                  device::PwmServo::new(
                                      p.TIM11,
                                      Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PB9, embassy_stm32::gpio::OutputType::PushPull)),
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

    #[cfg(feature = "with-motion")]
    defmt::info!("motion_planner done");

    #[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
        let adc = {
        let mut adc_hotend_hotbed = device::AdcHotendHotbed::new(p.ADC1, &mut embassy_time::Delay);
        #[cfg(feature = "nucleo64-l476rg")]
        adc_hotend_hotbed.set_sample_time(embassy_stm32::adc::SampleTime::Cycles12_5);
        #[cfg(feature = "nucleo64-f410rb")]
        adc_hotend_hotbed.set_sample_time(embassy_stm32::adc::SampleTime::Cycles15);
        static ADC_INST: TrackedStaticCell<ControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();
        ControllerRef::new(ADC_INST.init(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed)
        ))
    };

    #[cfg(all(feature ="nucleo64-l476rg", any(feature = "with-hotend", feature = "with-hotbed")))]
        let pwm_hotend_hotbed = {
            let pwm = device::PwmHotendHotbed::new(
            p.TIM15,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PB14, embassy_stm32::gpio::OutputType::PushPull)),
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PB15, embassy_stm32::gpio::OutputType::PushPull)),
            None,
            None,
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmHotendHotbed>> = TrackedStaticCell::new();

        ControllerRef::new(PWM_INST.init(
            "PwmHotendHotbed",
            ControllerMutex::new(pwm)
        ))
    };

    #[cfg(all(feature ="nucleo64-f410rb", any(feature = "with-hotend", feature = "with-hotbed", feature = "with-layer-fan")))]
        let pwm_hotend_hotbed_layer = {
            let pwm = device::PwmHotendHotbedLayer::new(
            p.TIM5,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PB12, embassy_stm32::gpio::OutputType::PushPull)),
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PA1, embassy_stm32::gpio::OutputType::PushPull)),
            None,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PB11, embassy_stm32::gpio::OutputType::PushPull)),
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmHotendHotbedLayer>> = TrackedStaticCell::new();

        ControllerRef::new(PWM_INST.init(
            "PwmHotendHotbedLayer",
            ControllerMutex::new(pwm)
        ))
    };

    #[cfg(all(feature = "nucleo64-l476rg", feature = "with-hotend"))]
        let hotend_device = {

        device::HotendPeripherals {
            power_pwm: pwm_hotend_hotbed.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PC2,
        }
    };
    #[cfg(all(feature = "nucleo64-f410rb", feature = "with-hotend"))]
        let hotend_device = {

        device::HotendPeripherals {
            power_pwm: pwm_hotend_hotbed_layer.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PB0,
        }
    };

    #[cfg(all(feature="nucleo64-l476rg", feature = "with-hotbed"))]
        let hotbed_device = {

        device::HotbedPeripherals {
            power_pwm: pwm_hotend_hotbed.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch2,
            temp_adc: adc.clone(),
            temp_pin: p.PC3,
        }
    };
    #[cfg(all(feature="nucleo64-f410rb", feature = "with-hotbed"))]
        let hotbed_device = {

        device::HotbedPeripherals {
            power_pwm: pwm_hotend_hotbed_layer.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch4,
            temp_adc: adc.clone(),
            temp_pin: p.PB1,
        }
    };

    #[cfg(all(feature="nucleo64-l476rg", feature = "with-laser"))]
        let laser_device = {
        let pwm = device::PwmLaser::new(
            p.TIM8,
            None,
            None,
            None,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PC9, embassy_stm32::gpio::OutputType::PushPull)),
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmLaser>> = TrackedStaticCell::new();

        device::LaserPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init(
                "PwmLaser",
                ControllerMutex::new(pwm)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch4,
        }
    };

    #[cfg(all(feature="nucleo64-f410rb", feature = "with-laser"))]
        let laser_device = {
            let pwm = device::PwmLaser::new(
            p.TIM1,
            None,
            None,
            None,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PA11, embassy_stm32::gpio::OutputType::PushPull)),
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmLaser>> = TrackedStaticCell::new();

        device::LaserPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init(
                "PwmLaser",
                ControllerMutex::new(pwm)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch4,
        }
    };

    #[cfg(all(feature="nucleo64-l476rg", feature = "with-layer-fan"))]
        let layer_fan_device = {
        let pwm = device::PwmLayerFan::new(
            p.TIM2,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PA15, embassy_stm32::gpio::OutputType::PushPull)),
            None,
            None,
            None,
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmLayerFan>> = TrackedStaticCell::new();

        device::LayerFanPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init(
                "PwmLayer",
                ControllerMutex::new(pwm)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
        }
    };

    #[cfg(all(feature="nucleo64-f410rb", feature = "with-layer-fan"))]
        let layer_fan_device = {

        device::LayerFanPeripherals {
            power_pwm: pwm_hotend_hotbed_layer.clone(),
            power_channel: embassy_stm32::timer::Channel::Ch2,
        }
    };


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
            hotend: hotend_device,
            #[cfg(feature = "with-hotbed")]
            hotbed: hotbed_device,
            #[cfg(feature = "with-laser")]
            laser: laser_device,
            #[cfg(feature = "with-layer-fan")]
            layer_fan: layer_fan_device,
        }
    }

}

#[allow(unused)]
pub mod consts {
    /// 50ms to enqueue ~28 motion gcodes at 115200 bps
    pub(crate) const LINGER_MS: u64 = 10000;
}