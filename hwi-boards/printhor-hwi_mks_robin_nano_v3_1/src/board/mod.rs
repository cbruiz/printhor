/// https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32F407VE.json
///
pub mod device;
pub mod io;
#[cfg(feature = "with-trinamic")]
pub mod comm;

use alloc_cortex_m::CortexMHeap;
use embassy_executor::Spawner;
use embassy_stm32::Config;
#[cfg(any(feature = "with-uart-port-1", feature = "with-usbserial", feature="with-trinamic"))]
use embassy_stm32::{bind_interrupts};
#[cfg(any(feature = "with-uart-port-1"))]
use embassy_stm32::usart;
use embassy_stm32::gpio::{Input, Level, Output, Speed, Pull};

use embassy_sync::mutex::Mutex;
#[cfg(any(feature = "with-uart-port-1"))]
use embassy_stm32::usart::{DataBits, Parity, StopBits};
#[cfg(feature = "with-usbserial")]
use embassy_stm32::usb_otg;
#[cfg(feature = "with-usbserial")]
use device::*;
#[cfg(feature = "with-spi")]
use embassy_stm32::spi;
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
    #[cfg(feature = "with-probe")]
    pub probe: device::ProbePeripherals,
    #[cfg(feature = "with-fan-layer-fan1")]
    pub layer_fan: device::LayerFanPeripherals,
    #[cfg(feature = "with-hotend")]
    pub hotend: device::HotendPeripherals,
    #[cfg(feature = "with-hotbed")]
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
#[cfg(feature = "with-uart-port-1")]
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

pub async fn setup(_spawner: Spawner, p: embassy_stm32::Peripherals) -> printhor_hwa_common::MachineContext<Controllers, IODevices, MotionDevices, PwmDevices> {

    #[cfg(feature = "with-usbserial")]
    let (usbserial_tx, usbserial_rx_stream) = {
        static EP_OUT_BUFFER_INST: TrackedStaticCell<[u8; 256]> =  TrackedStaticCell::new();
        let ep_out_buffer = EP_OUT_BUFFER_INST.init("USBEPBuffer", [0u8; 256]);
        defmt::info!("Creating USB Driver");
        let mut config = embassy_stm32::usb_otg::Config::default();
        config.vbus_detection = false;

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
        cfg.detect_previous_overrun = false;

        let (uart_port1_tx_device, uart_port1_rx_device) = device::UartPort1Device::new(p.USART1,
                                                            p.PA10, p.PA9,
                                                            UartPort1Irqs,
                                                            p.DMA2_CH7, p.DMA2_CH5,
                                                            cfg).expect("Ready").split();

        static UART_PORT1_INST: TrackedStaticCell<ControllerMutex<device::UartPort1TxDevice>> = TrackedStaticCell::new();
        let uart_port1_tx = ControllerRef::new(
            UART_PORT1_INST.init("UartPort1", Mutex::<ControllerMutexType, _>::new(uart_port1_tx_device))
        );
        (uart_port1_tx, device::UartPort1RxInputStream::new(uart_port1_rx_device))
    };

    #[cfg(feature = "with-trinamic")]
    let trinamic_uart = {
        // TODO: WorkInProgress Trinamic UART (when needed) requires a software usar implementation because of the wiring

        //use printhor_hwa_common::soft_uart::{AsyncRead, AsyncWrite};



        //let _ = uart_e0.write(0b10101010u8).await;
        //let _ = uart_e0.read().await;

        crate::device::UartTrinamic::new(p.PD5, p.PD7, p.PD4, p.PD9)
    };

    #[cfg(feature = "with-spi")]
    let spi1_device = {
        let mut cfg = spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(SPI_FREQUENCY_HZ);
        static SPI1_INST: TrackedStaticCell<ControllerMutex<device::Spi1>> = TrackedStaticCell::new();
        ControllerRef::new(SPI1_INST.init(
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
                PWM_INST.init("PwmServo",
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

    // FAN0 = PC14 // Requires a soft pwm [Not implemented (yet)]
    // FAN1 = PB1 // T3.CH4
    // HE0 = PE5 // T9.Ch1
    // HE1 = PB0 // T3.3 | T1.2N
    // BE0 = PA0 // T2.1 | T5.1
    // LASER = PA6 // T13.1

    #[cfg(feature = "with-fan-layer-fan1")]
        let layer_device = {
        static PWM_INST: TrackedStaticCell<ControllerMutex<device::PwmLayerFan>> = TrackedStaticCell::new();

        let pwm_fan1 = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            p.TIM3,
            None,
            None,
            None,
            Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PB1, embassy_stm32::gpio::OutputType::PushPull)),
            embassy_stm32::time::hz(5_000),
            embassy_stm32::timer::CountingMode::CenterAlignedBothInterrupts,
        );
        crate::device::LayerFanPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init(
                "PwmFan",
                ControllerMutex::new(pwm_fan1)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
        }
    };

    #[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
    let adc = {
        let mut adc_hotend_hotbed = device::AdcHotendHotbed::new(p.ADC1, &mut embassy_time::Delay);
        adc_hotend_hotbed.set_sample_time(embassy_stm32::adc::SampleTime::Cycles15);
        static ADC_INST: TrackedStaticCell<ControllerMutex<device::AdcHotendHotbed>> = TrackedStaticCell::new();
        ControllerRef::new(ADC_INST.init(
            "HotendHotbedAdc",
            ControllerMutex::new(adc_hotend_hotbed)
        ))
    };

    #[cfg(any(feature = "with-hotend"))]
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

        HotendPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init(
                "PwmFanFan0HotendHotbed",
                ControllerMutex::new(pwm_hotend)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PC1,
        }
    };

    #[cfg(any(feature = "with-hotbed"))]
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

        HotbedPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init(
                "PwmHotbed",
                ControllerMutex::new(pwm_hotbed)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
            temp_adc: adc.clone(),
            temp_pin: p.PC0,
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

        LaserPeripherals {
            power_pwm: ControllerRef::new(PWM_INST.init(
                "PwmLaser",
                ControllerMutex::new(pwm_laser)
            )),
            power_channel: embassy_stm32::timer::Channel::Ch1,
        }
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
            #[cfg(feature = "with-fan-layer-fan1")]
            layer_fan: layer_device,
            #[cfg(feature = "with-hotend")]
            hotend: hotend_device,
            #[cfg(feature = "with-hotbed")]
            hotbed: hotbed_device,
            #[cfg(feature = "with-laser")]
            laser: laser_device,
        }
    }

}

#[allow(unused)]
pub mod consts {
    /// 50ms to enqueue ~28 motion gcodes at 115200 bps
    pub(crate) const LINGER_MS: u64 = 10000;
}