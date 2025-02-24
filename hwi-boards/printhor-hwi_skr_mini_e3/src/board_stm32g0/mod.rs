//! Hardware Interface implementation for SKR Mini E3 V3 (STM32G0B1RE)
//!
//! <https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32G0B1RE.json>

use printhor_hwa_common as hwa;
use hwa::HwiContract;

mod device;
mod types;

pub(crate) mod io;

#[global_allocator]
static HEAP: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub struct Contract;
impl HwiContract for Contract {

    //#region "Board specific constants"

    const MACHINE_TYPE: &'static str = "SKR";
    const MACHINE_BOARD: &'static str = "SKR_MINI_E3_V3";
    /// ARM Cortex M0+ @64MHZ, 144kB SRAM, 512kB Program
    const MACHINE_PROCESSOR: &'static str = "STM32G0B1RET6";

    const PROCESSOR_SYS_CK_MHZ: u32 = 64_000_000;

    /// The target [hwa::CommChannel] for M117 (display)
    const DISPLAY_CHANNEL: hwa::CommChannel = hwa::CommChannel::SerialPort2;

    /// The target [hwa::CommChannel] for M118 (host)
    const HOST_CHANNEL: hwa::CommChannel = hwa::CommChannel::SerialUsb;

    //#endregion

    //#region "Watchdog settings"

    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 8_000_000;

    //#endregion

    //#region "Memory management/tracking settings"

    #[const_env::from_env("MAX_HEAP_SIZE_BYTES")]
    const MAX_HEAP_SIZE_BYTES: usize = 512;

    #[const_env::from_env("MAX_EXPECTED_STATIC_ALLOC_BYTES")]
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize = 16384;

    fn heap_current_size() -> usize {
        HEAP.used()
    }

    //#endregion

    //#region "Feature [with-motion] settings"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            cfg_if::cfg_if! {
                if #[cfg(feature = "with-motion-cartessian-kinematics")] {
                     const DEFAULT_WORLD_SIZE_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=220.0, y=220.0, z=220.0)
                    };

                    const DEFAULT_WORLD_CENTER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=105.0, y=105.0, z=105.0)
                    };

                    const DEFAULT_MAX_SPEED_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=600.0, y=600.0, z=100.0, e=300.0)
                    };

                    const DEFAULT_MAX_ACCEL_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=2400.0, y=2400.0, z=2400.0, e=2400.0)
                    };

                    const DEFAULT_MAX_JERK_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=4800.0, y=4800.0, z=4800.0, e=4800.0)
                    };

                    const DEFAULT_TRAVEL_SPEED_PS: hwa::math::Real = const {
                        hwa::make_real!(600.0)
                    };

                    const DEFAULT_UNITS_PER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=10.0, y=10.0, z=50.0, e=50.0)
                    };

                    const DEFAULT_MICRO_STEPS_PER_AXIS: hwa::math::TVector<u16> = const {
                        hwa::make_vector!(x=8, y=8, z=8, e=8)
                    };

                    #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
                    const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 100;

                    #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
                    const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;

                    #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
                    const SEGMENT_QUEUE_SIZE: u8 = 40;
                }
                else if #[cfg(feature = "with-motion-core-xy-kinematics")] {
                    const DEFAULT_WORLD_SIZE_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=220.0, y=220.0, z=220.0)
                    };

                    const DEFAULT_WORLD_CENTER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=105.0, y=105.0, z=105.0)
                    };

                    const DEFAULT_MAX_SPEED_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=600.0, y=600.0, z=100.0, e=300.0)
                    };

                    const DEFAULT_MAX_ACCEL_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=9800.0, y=9800.0, z=4800.0, e=9800.0)
                    };

                    const DEFAULT_MAX_JERK_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=19600.0, y=19600.0, z=9600.0, e=19600.0)
                    };

                    const DEFAULT_TRAVEL_SPEED_PS: hwa::math::Real = const {
                        hwa::make_real!(600.0)
                    };

                    const DEFAULT_UNITS_PER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=10.0, y=10.0, z=50.0, e=50.0)
                    };


                    const DEFAULT_MICRO_STEPS_PER_AXIS: hwa::math::TVector<u16> = const {
                        hwa::make_vector!(x=8, y=8, z=8, e=8)
                    };

                    #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
                    const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 100;

                    #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
                    const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 10_000;

                    #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
                    const SEGMENT_QUEUE_SIZE: u8 = 40;

                }
                else if #[cfg(feature = "with-motion-delta-kinematics")] {
                    compile_error!("Not yet implemented");
                }
                else {
                    compile_error!("You didn't specify any supported kinematics");
                }
            }
        }
    }

    //#endregion

    //#region Constant settings for with-hot-end feature [...]

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {

            #[const_env::from_env("HOT_END_ADC_V_REF_DEFAULT_MV")]
            const HOT_END_ADC_V_REF_DEFAULT_MV: u16 = todo!("fill me");

            #[const_env::from_env("HOT_END_ADC_V_REF_DEFAULT_SAMPLE")]
            const HOT_END_ADC_V_REF_DEFAULT_SAMPLE: u16 = todo!("fill me");

            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_BETA: f32 = todo!("fill me");

            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_NOMINAL_RESISTANCE: f32 = todo!("fill me");

            #[const_env::from_env("HOT_END_THERM_PULL_UP_RESISTANCE")]
            const HOT_END_THERM_PULL_UP_RESISTANCE: f32 = todo!("fill me");
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-bed")] {

            #[const_env::from_env("HOT_BED_ADC_V_REF_DEFAULT_MV")]
            const HOT_BED_ADC_V_REF_DEFAULT_MV: u16 = todo!("fill me");

            #[const_env::from_env("HOT_BED_ADC_V_REF_DEFAULT_SAMPLE")]
            const HOT_BED_ADC_V_REF_DEFAULT_SAMPLE: u16 = todo!("fill me");

            #[const_env::from_env("HOT_BED_THERM_BETA")]
            const HOT_BED_THERM_BETA: f32 = todo!("fill me");

            #[const_env::from_env("HOT_BED_THERM_NOMINAL_RESISTANCE")]
            const HOT_BED_THERM_NOMINAL_RESISTANCE: f32 = todo!("fill me");

            #[const_env::from_env("HOT_BED_THERM_PULL_UP_RESISTANCE")]
            const HOT_BED_THERM_PULL_UP_RESISTANCE: f32 = todo!("fill me");
        }
    }

    //#endregion

    //#region Customization of Mutex types [...]

    type EventBusPubSubMutexType = types::EventBusPubSubMutexType;
    type WatchDogMutexStrategy = types::WatchDogMutexStrategy;

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            type DeferChannelMutexType = types::DeferChannelMutexType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            type StepActuatorMutexType = types::StepActuatorMutexType;
            type MotionSignalMutexType = types::MotionSignalMutexType;
            type MotionRingBufferMutexType = types::MotionRingBufferMutexType;
            type MotionConfigMutexType = types::MotionConfigMutexType;
            type MotionStatusMutexType = types::MotionStatusMutexType;
            type MotionDriverMutexType = types::MotionDriverMutexType;
        }
    }

    //#endregion

    //#region Locking strategies for shared devices/controllers/adaptors [...]
    type EventBusMutexStrategy = types::EventBusMutexStrategy;

    //#endregion

    //#region "devices declarations and constant parameters"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-usb")] {
            #[const_env::from_env("SERIAL_USB_PACKET_SIZE")]
            const SERIAL_USB_PACKET_SIZE: usize = 64;

            type SerialUsbTx = types::SerialUsbTxMutexStrategy;
            type SerialUsbRx = io::serial_usb::SerialUsbRxInputStream;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-1")] {

            #[const_env::from_env("SERIAL_PORT1_BAUD_RATE")]
            const SERIAL_PORT1_BAUD_RATE: u32 = 115200;
            #[const_env::from_env("SERIAL_PORT1_RX_BUFFER_SIZE")]
            const SERIAL_PORT1_RX_BUFFER_SIZE: usize = 128;

            type SerialPort1Tx = types::SerialPort1TxMutexStrategy;
            type SerialPort1Rx = device::SerialPort1Rx;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-2")] {
            #[const_env::from_env("SERIAL_PORT2_BAUD_RATE")]
            const SERIAL_PORT2_BAUD_RATE: u32 = 115200;
            #[const_env::from_env("SERIAL_PORT2_RX_BUFFER_SIZE")]
            const SERIAL_PORT2_RX_BUFFER_SIZE: usize = 128;

            type SerialPort2Tx = types::SerialPort2TxMutexStrategy;
            type SerialPort2Rx = device::SerialPort2Rx;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-spi")] {
            #[const_env::from_env("SPI_FREQUENCY")]
            const SPI_FREQUENCY: u32 = 20_000_000;
            type SpiController = types::Spi1MutexStrategyType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-i2c")] {
            #[const_env::from_env("I2C_FREQUENCY")]
            const I2C_FREQUENCY: u32 = 100_000;
            type I2cMotionMutexStrategy = types::I2cMutexStrategyType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            type StepActuatorMutexStrategy = types::StepActuatorMuxtexStrategy;
            type StepActuator = device::StepActuator;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
            type MotionBroadcastChannelMutexType = types::MotionBroadcastChannelMutexType;

            type MotionSenderMutexStrategy = types::MotionSenderMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-ps-on")] {
            type PSOnMutexStrategy = types::PSOnMutexStrategy;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            type ProbePwm = types::ProbePwmMutexStrategy;
            type ProbePwmChannel = hwa::HwiResource<device::PwmProbeChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-end")] {
            type HotEndAdc = types::HotEndAdcMutexStrategy;
            type HotEndAdcPin = hwa::HwiResource<device::HotEndAdcPin>;
            type HotEndPwm = types::HotEndPwmMutexStrategy;
            type HotEndPwmChannel = hwa::HwiResource<device::HotEndPwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-bed")] {
            type HotBedAdc = types::HotBedAdcMutexStrategy;
            type HotBedAdcPin = hwa::HwiResource<device::HotBedAdcPin>;
            type HotBedPwm = types::HotBedPwmMutexStrategy;
            type HotBedPwmChannel = hwa::HwiResource<device::HotBedPwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-layer")] {
            type FanLayerPwm = types::FanLayerPwmMutexStrategy;
            type FanLayerPwmChannel = hwa::HwiResource<device::FanLayerPwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-laser")] {
            type LaserPwm = types::LaserPwmMutexStrategy;
            type LaserPwmChannel = hwa::HwiResource<device::PwmLaserChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-extra-1")] {
            type FanExtra1Pwm = types::FanExtra1PwmMutexStrategy;
            type FanExtra1PwmChannel = hwa::HwiResource<device::FanExtra1PwmChannel>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-sd-card")] {
            const SD_CARD_MAX_FILES: usize = 4;
            const SD_CARD_MAX_DIRS: usize = 4;
            type SDCardBlockDevice = types::SDCardBlockDevice;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-print-job")] {
            type PrinterControllerSignalMutexType = types::PrinterControllerSignalMutexType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-trinamic")] {
            const TRINAMIC_UART_BAUD_RATE: u32 = 4096;
            type TrinamicUartDevice = types::TrinamicUartDevice;

        }
    }

    //#endregion

    fn init_heap() {
        hwa::info!("Initializing heap ({}) bytes", Contract::MAX_HEAP_SIZE_BYTES);
        use core::mem::MaybeUninit;
        #[link_section = ".bss"]
        static mut HEAP_MEM: [MaybeUninit<u8>; Contract::MAX_HEAP_SIZE_BYTES] =
            [MaybeUninit::uninit(); Contract::MAX_HEAP_SIZE_BYTES];
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, Contract::MAX_HEAP_SIZE_BYTES)
        }
    }

    async fn init(_spawner: embassy_executor::Spawner) -> hwa::HwiContext<Self> {

        //#region "Bootloader reset"

        cfg_if::cfg_if! {
            if #[cfg(not(feature = "without-bootloader"))] {

                // Reset bootloader state

                // SYSCFG: Dead battery pull-down resistors functionality should be enabled by default on startup
                embassy_stm32::pac::SYSCFG.cfgr1().modify(|w| {
                    w.set_ucpd1_strobe(true);
                    w.set_ucpd2_strobe(true);
                });

                // RCC: Enable HSI and wait for it to be ready
                embassy_stm32::pac::RCC.cr().modify(|w| {
                    w.set_hsion(true)
                });
                while !embassy_stm32::pac::RCC.cr().read().hsirdy() {}

                // CFGR: Write its default
                embassy_stm32::pac::RCC.cfgr().write(|_w| {
                });

                // RCC: Enable HSI only. Wait for PLL to be unready
                embassy_stm32::pac::RCC.cr().write(|w| {
                    w.set_hsion(true)
                });
                while embassy_stm32::pac::RCC.cr().read().pllrdy() {}

                // Reset values from datasheet
                embassy_stm32::pac::RCC.pllcfgr().write_value(embassy_stm32::pac::rcc::regs::Pllcfgr(0x00001000));
                embassy_stm32::pac::RCC.gpioenr().write_value(embassy_stm32::pac::rcc::regs::Gpioenr(0x00001000));
                embassy_stm32::pac::RCC.ahbenr().write_value(embassy_stm32::pac::rcc::regs::Ahbenr(0x000000100));
                embassy_stm32::pac::RCC.apbenr1().write_value(embassy_stm32::pac::rcc::regs::Apbenr1(0x000000000));
                embassy_stm32::pac::RCC.apbenr2().write_value(embassy_stm32::pac::rcc::regs::Apbenr2(0x000000000));

                unsafe {
                    hwa::trace!("Setting VTOR...");
                    #[allow(unused_mut)]
                    let mut p = cortex_m::Peripherals::steal();
                    hwa::trace!("VTOR WAS AT: {} ", p.SCB.vtor.read());
                    p.SCB.vtor.write(0x2000);
                    hwa::trace!("VTOR SET TO: {} ", p.SCB.vtor.read());
                }
            }
            else {
                compile_error!("You should not compile without bootloader")
            }
        }
        //#endregion

        //#region "RCC setup"

        let mut config = embassy_stm32::Config::default();

        config.rcc.hsi = None;
        config.rcc.hse = Some(
            embassy_stm32::rcc::Hse {
                freq: embassy_stm32::time::Hertz(8_000_000),
                mode: embassy_stm32::rcc::HseMode::Oscillator
            }
        );
        config.rcc.pll = Some(
            embassy_stm32::rcc::Pll {
                // HSE = 8MHz
                source: embassy_stm32::rcc::PllSource::HSE,
                prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
                mul: embassy_stm32::rcc::PllMul::MUL24,
                // SysClk = 8 / 1 * 24 / 3 = 64MHz
                divr: Some(embassy_stm32::rcc::PllRDiv::DIV3),
                // PLLQ = 8 / 1 * 24 / 4 = 48MHz
                divq: Some(embassy_stm32::rcc::PllQDiv::DIV4),
                // PLLP = 8 / 1 * 24 / 3 = 64MHz
                divp: Some(embassy_stm32::rcc::PllPDiv::DIV3),
            }
        );
        config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;
        // PllQ does not work for Usb in this board (so far)
        config.rcc.hsi48 = Some(embassy_stm32::rcc::Hsi48Config { sync_from_usb: true });
        config.rcc.mux.usbsel = embassy_stm32::rcc::mux::Usbsel::HSI48;
        // HCLK = {Power, AHB bus, core, memory, DMA, System timer, FCLK} = 64MHz
        config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
        // PCLK = APB peripheral clocks = 64MHz
        // TPCLK = APOB timer clocks = 64MHz
        config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
        config.rcc.low_power_run = false;
        let p = embassy_stm32::init(config);

        //#endregion

        embassy_stm32::pac::SYSCFG.cfgr1().write(|w| {
            // https://www.st.com/resource/en/reference_manual/rm0454-stm32g0x0-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
            //  set_pa11_rmp and set_pa12_rmp (bits 3 and 4)
            let val = true;
            let off = 3usize;
            w.0 = (w.0 & !(0x01 << off)) | (((val as u32) & 0x01) << off);
            let off = 4usize;
            w.0 = (w.0 & !(0x01 << off)) | (((val as u32) & 0x01) << off);
        });

        //#region "with-serial-usb"

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-serial-usb")] {

                embassy_stm32::bind_interrupts!(struct UsbIrqs {
                    USB_UCPD1_2 => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB>;
                });

                {
                    use embassy_stm32::interrupt::InterruptExt;
                    embassy_stm32::interrupt::USB_UCPD1_2.set_priority(embassy_stm32::interrupt::Priority::P3);
                }

                hwa::info!("Creating USB Driver");
                let mut usb_serial_device = io::serial_usb::SerialUsbDevice::new(
                    embassy_stm32::usb::Driver::new(p.USB, UsbIrqs, p.PA12, p.PA11)
                );
                hwa::info!("Spawning USB Driver Task");
                usb_serial_device.spawn(_spawner);

                let (usb_serial_rx_device, sender) = usb_serial_device.split();
                let serial_usb_tx = hwa::make_static_async_controller!(
                    "UsbSerialTxController",
                    types::SerialUsbTxMutexStrategy,
                    sender
                );

                let serial_usb_rx_stream = device::SerialUsbRxInputStream::new(usb_serial_rx_device);
            }
        }

        //#endregion

        //#region "with-serial-port-1"

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-1"))] {

                embassy_stm32::bind_interrupts!(struct SerialPort1IRQs {
                    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
                });

                {
                    use embassy_stm32::interrupt::InterruptExt;
                    embassy_stm32::interrupt::USART1.set_priority(embassy_stm32::interrupt::Priority::P3);
                    embassy_stm32::interrupt::DMA1_CHANNEL2_3.set_priority(embassy_stm32::interrupt::Priority::P3);
                }

                let mut cfg = embassy_stm32::usart::Config::default();
                cfg.baudrate = Self::SERIAL_PORT1_BAUD_RATE;
                cfg.data_bits = embassy_stm32::usart::DataBits::DataBits8;
                cfg.stop_bits = embassy_stm32::usart::StopBits::STOP1;
                cfg.parity = embassy_stm32::usart::Parity::ParityNone;
                cfg.detect_previous_overrun = true;

                let (serial_port_1_tx_device, serial_port_1_rx_device) = embassy_stm32::usart::Uart::new(
                    p.USART1, p.PA10, p.PA9,
                    SerialPort1IRQs,
                    p.DMA2_CH3, p.DMA2_CH2,
                    cfg,
                ).expect("Ready").split();
                let serial_port_1_tx = hwa::make_static_async_controller!(
                    "SerialPort1Tx",
                    types::SerialPort1TxMutexStrategy,
                    hwa::SerialTxWrapper::new(serial_port_1_tx_device, Self::SERIAL_PORT1_BAUD_RATE)
                );
                let serial_port_1_rx_stream = device::SerialPort1Rx::new(serial_port_1_rx_device);
            }
        }

        //#endregion

        //#region "with-serial-port-2"

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-2"))] {

                embassy_stm32::bind_interrupts!(struct SerialPort2IRQs {
                    USART2_LPUART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
                });

                {
                    use embassy_stm32::interrupt::InterruptExt;
                    embassy_stm32::interrupt::USART2_LPUART2.set_priority(embassy_stm32::interrupt::Priority::P3);
                    embassy_stm32::interrupt::DMA1_CH4_7_DMA2_CH1_5_DMAMUX1_OVR.set_priority(embassy_stm32::interrupt::Priority::P3);
                }

                let mut cfg = embassy_stm32::usart::Config::default();
                cfg.baudrate = Self::SERIAL_PORT2_BAUD_RATE;
                cfg.data_bits = embassy_stm32::usart::DataBits::DataBits8;
                cfg.stop_bits = embassy_stm32::usart::StopBits::STOP1;
                cfg.parity = embassy_stm32::usart::Parity::ParityNone;
                cfg.detect_previous_overrun = true;

                let (serial_port_2_tx_device, serial_port_2_rx_device) = embassy_stm32::usart::Uart::new(
                    p.USART2, p.PA3, p.PA2,
                    SerialPort2IRQs,
                    p.DMA2_CH5, p.DMA2_CH4,
                    cfg,
                ).expect("Ready").split();
                let serial_port_2_tx = hwa::make_static_async_controller!(
                    "SerialPort2Tx",
                    types::SerialPort2TxMutexStrategy,
                    hwa::SerialTxWrapper::new(serial_port_2_tx_device, Self::SERIAL_PORT2_BAUD_RATE)
                );
                let serial_port_2_rx_stream = device::SerialPort2Rx::new(serial_port_2_rx_device);
            }
        }

        //#endregion

        //#region "with-motion"

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                let motion_pins = device::StepActuator {
                    #[cfg(feature = "with-x-axis")]
                    x_enable_pin: embassy_stm32::gpio::Output::new(p.PB14, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-y-axis")]
                    y_enable_pin: embassy_stm32::gpio::Output::new(p.PB11, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-z-axis")]
                    z_enable_pin: embassy_stm32::gpio::Output::new(p.PB1, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-e-axis")]
                    e_enable_pin: embassy_stm32::gpio::Output::new(p.PD1, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-x-axis")]
                    x_endstop_pin: embassy_stm32::gpio::Input::new(p.PC0, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-y-axis")]
                    y_endstop_pin: embassy_stm32::gpio::Input::new(p.PC1, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-z-axis")]
                    z_endstop_pin: embassy_stm32::gpio::Input::new(p.PC2, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-e-axis")]
                    e_endstop_pin: embassy_stm32::gpio::Input::new(p.PC15, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-x-axis")]
                    x_step_pin: embassy_stm32::gpio::Output::new(p.PB13, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-y-axis")]
                    y_step_pin: embassy_stm32::gpio::Output::new(p.PB10, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-z-axis")]
                    z_step_pin: embassy_stm32::gpio::Output::new(p.PB0, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-e-axis")]
                    e_step_pin: embassy_stm32::gpio::Output::new(p.PB3, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-x-axis")]
                    x_dir_pin: embassy_stm32::gpio::Output::new(p.PB12, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-y-axis")]
                    y_dir_pin: embassy_stm32::gpio::Output::new(p.PB2, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-z-axis")]
                    z_dir_pin: embassy_stm32::gpio::Output::new(p.PC5, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-e-axis")]
                    e_dir_pin: embassy_stm32::gpio::Output::new(p.PB4, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                };
            }
        }

        //#endregion

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-spi")] {

                {
                    use embassy_stm32::interrupt::InterruptExt;
                    embassy_stm32::interrupt::SPI1.set_priority(embassy_stm32::interrupt::Priority::P4);
                    embassy_stm32::interrupt::DMA1_CHANNEL2_3.set_priority(embassy_stm32::interrupt::Priority::P4);
                }

                let spi = {
                    let mut cfg = embassy_stm32::spi::Config::default();
                    cfg.frequency = embassy_stm32::time::Hertz(Self::SPI_FREQUENCY);
                    hwa::make_static_async_controller!(
                        "SPI1Controller",
                        types::Spi1MutexStrategyType,
                        device::Spi1::new(
                            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, cfg
                        )
                    )
                };
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-sd-card")] {

                let sd_card_block_device = hwa::sd_card_spi::SPIAdapter::new(
                    spi.clone(),
                    embassy_stm32::gpio::Output::new(p.PA4,
                        embassy_stm32::gpio::Level::High,
                        embassy_stm32::gpio::Speed::VeryHigh
                    )
                );
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-trinamic")] {
                embassy_stm32::bind_interrupts!(struct TrinamicIrqs {
                    USART3_4_5_6_LPUART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART4>;
                });

                let trinamic_uart = {
                    let mut cfg = embassy_stm32::usart::Config::default();
                    cfg.baudrate = Self::TRINAMIC_UART_BAUD_RATE;
                    cfg.data_bits = embassy_stm32::usart::DataBits::DataBits8;
                    cfg.stop_bits = embassy_stm32::usart::StopBits::STOP1;
                    cfg.parity = embassy_stm32::usart::Parity::ParityNone;
                    cfg.detect_previous_overrun = false;
                    io::TrinamicUartWrapper::new(
                        device::TrinamicUart::new(p.USART4, p.PC11, p.PC10,
                            TrinamicIrqs, p.DMA1_CH7, p.DMA1_CH6,
                            cfg).expect("Ready")
                        )
                };
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-ps-on")] {
                let ps_on = hwa::make_static_sync_controller!(
                    "PSOn",
                    types::PSOnMutexStrategy,
                    device::PsOnPin::new(p.PC13, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::Low)
                );
            }
        }
        
        cfg_if::cfg_if!{
            if #[cfg(any(feature = "with-probe"))] {
                let pwm_servo = hwa::make_static_sync_controller!(
                    "PwmServo",
                    types::ServoPwmMutexStrategy,
                    device::PwmServo::new(
                        p.TIM2,
                        None, // PA0 | PA15 | PA5 | PC4
                        Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch2(p.PA1, embassy_stm32::gpio::OutputType::PushPull)), // PA1 | PB3 | PC5
                        None, // PA2 | PB10 | PC6
                        None, // PA3 | PB11 | PC7
                        embassy_stm32::time::hz(50),
                        embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                    )
                );
                
            }
        }

        cfg_if::cfg_if!{
            if #[cfg(any(feature = "with-probe"))] {
                let probe_pwm = pwm_servo.clone();
                let probe_pwm_channel = hwa::HwiResource::new(device::PwmProbeChannel::Ch2);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                setup_timer();
            }
        }

        // Return the HWI Context with HWI peripherals and controllers

        hwa::HwiContext {
            sys_watch_dog : hwa::make_static_async_controller!(
                "WatchDogController",
                types::WatchDogMutexStrategy,
                device::Watchdog::new(p.IWDG, Self::WATCHDOG_TIMEOUT_US),
            ),
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature = "with-serial-port-1")]
            serial_port_1_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port_1_rx_stream,
            #[cfg(feature = "with-serial-port-2")]
            serial_port_2_tx,
            #[cfg(feature = "with-serial-port-2")]
            serial_port_2_rx_stream,
            #[cfg(feature = "with-spi")]
            spi,
            #[cfg(feature = "with-sd-card")]
            sd_card_block_device,
            #[cfg(feature = "with-i2c")]
            i2c,
            #[cfg(feature = "with-ps-on")]
            ps_on,
            #[cfg(feature = "with-motion")]
            motion_pins,
            #[cfg(feature = "with-trinamic")]
            trinamic_uart,
            #[cfg(feature = "with-motion-broadcast")]
            motion_sender,
            #[cfg(feature = "with-probe")]
            probe_pwm,
            #[cfg(feature = "with-probe")]
            probe_pwm_channel,
            #[cfg(feature = "with-laser")]
            laser_pwm,
            #[cfg(feature = "with-laser")]
            laser_pwm_channel,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_pwm,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_pwm_channel,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_pwm,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_pwm_channel,
            #[cfg(feature = "with-hot-end")]
            hot_end_adc,
            #[cfg(feature = "with-hot-end")]
            hot_end_adc_pin,
            #[cfg(feature = "with-hot-end")]
            hot_end_pwm,
            #[cfg(feature = "with-hot-end")]
            hot_end_pwm_channel,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_adc,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_adc_pin,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_pwm,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_pwm_channel,
            #[cfg(feature = "with-motion-broadcast")]
            high_priority_core: hwa::NoDevice,
        }
    }

    fn sys_reset() {
        cortex_m::peripheral::SCB::sys_reset();
    }

    fn sys_stop() {
        // Not needed
    }

    // Execute closure f in an interrupt-free context.
    // In native this is not required, so does nothing
    fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce() -> R,
    {
        f()
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            fn pause_ticker() {
                unsafe {
                    let p = cortex_m::Peripherals::steal();
                    let mut syst = p.SYST;
                    syst.disable_counter();
                    syst.disable_interrupt();
                }
                hwa::info!("Ticker Paused");
            }

            fn resume_ticker() {
                hwa::debug!("Ticker Resumed");
                unsafe {
                    let p = cortex_m::Peripherals::steal();
                    let mut syst = p.SYST;
                    syst.enable_counter();
                    syst.enable_interrupt();
                }
                hwa::info!("Ticker Resumed");
            }
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion-broadcast")] {

            type HighPriorityCore = hwa::NoDevice;

            fn launch_high_priotity<S: 'static + Sized + Send>(_core: Self::HighPriorityCore, _token: embassy_executor::SpawnToken<S>) -> Result<(),()>
            {
                pub static EXECUTOR_HIGH: embassy_executor::InterruptExecutor = embassy_executor::InterruptExecutor::new();

                #[interrupt]
                unsafe fn RTC_ALARM() {
                    EXECUTOR_HIGH.on_interrupt()
                }

                use embassy_stm32::interrupt;
                use embassy_stm32::interrupt::InterruptExt;
                interrupt::RTC_ALARM.set_priority(interrupt::Priority::P2);

                let spawner = EXECUTOR_HIGH.start(interrupt::RTC_ALARM);
                spawner.spawn(_token).map_err(|_| ())
            }
        }
    }

}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        extern "Rust" {
            fn do_tick();
        }

        use cortex_m_rt::exception;
        #[exception]
        fn SysTick() {
            #[cfg(feature = "with-motion")]
            unsafe {
                do_tick();
            }
        }

        pub fn setup_timer() {
            unsafe {
                let p = cortex_m::Peripherals::steal();
                let mut syst = p.SYST;
                syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
                let reload: u32 = ((Contract::PROCESSOR_SYS_CK_MHZ / Contract::STEP_PLANNER_CLOCK_FREQUENCY) - 1).max(1);
                hwa::info!(
                    "SYST reload set to {} ({} Hz)",
                    reload,
                    Contract::STEP_PLANNER_CLOCK_FREQUENCY
                );
                syst.set_reload(reload);
            }
        }
    }
}
