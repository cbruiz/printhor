//! HWI module for MKS Robin Nano v3.1
//!
//! <https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32F407VE.json>
//!

use hwa::HwiContract;
use printhor_hwa_common as hwa;

mod device;
mod io;
mod types;
//mod comm;

#[global_allocator]
static HEAP: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub struct Contract;
impl HwiContract for Contract {
    //#region "Board specific constants"

    const MACHINE_TYPE: &'static str = "MKS";
    const MACHINE_BOARD: &'static str = "SKR_ROBIN_NANO_V3.1";
    /// ARM Cortex M4F @168MHZ, 192kB SRAM, 512kB Program
    const MACHINE_PROCESSOR: &'static str = "STM32F407VET6";

    const PROCESSOR_SYS_CK_MHZ: u32 = 168_000_000;

    //#endregion

    //#region "Watchdog settings"

    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;

    //#endregion

    //#region "Memory management/tracking settings"

    #[const_env::from_env("MAX_HEAP_SIZE_BYTES")]
    const MAX_HEAP_SIZE_BYTES: usize = 1024;

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
                        hwa::make_vector_real!(x=200.0, y=200.0, z=200.0)
                    };

                    const DEFAULT_WORLD_CENTER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=100.0, y=100.0, z=100.0)
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
                        hwa::make_vector_real!(x=50.0, y=50.0, z=10.0, e=50.0)
                    };


                    const DEFAULT_MICRO_STEPS_PER_AXIS: hwa::math::TVector<u16> = const {
                        hwa::make_vector!(x=2, y=2, z=2, e=2)
                    };

                    #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
                    const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 100;

                    #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
                    const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;

                    #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
                    const SEGMENT_QUEUE_SIZE: u8 = 10;
                }
                else {
                    compile_error!("kinematics not implemented");
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
            type MotionPinsMutexType = types::MotionPinsMutexType;
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

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-usb")] {
            #[const_env::from_env("SERIAL_USB_RX_BUFFER_SIZE")]
            const SERIAL_USB_RX_BUFFER_SIZE: usize = 128;

            type SerialUsbTx = types::SerialUsbTxMutexStrategy;
            type SerialUsbRx = io::usb_serial::SerialUsbInputStream;
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
            compile_error!("Not implemented");
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-spi")] {
            #[const_env::from_env("SPI_FREQUENCY")]
            const SPI_FREQUENCY: u32 = 20_000_000;
            type SpiController = types::Spi3MutexStrategyType;
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
            type MotionPinsMutexStrategy = types::MotionPinsMuxtexStrategy;
            type MotionPins = device::MotionPins;
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
        }
    }

    //#endregion

    fn init_heap() {
        hwa::info!(
            "Initializing heap ({}) bytes",
            Contract::MAX_HEAP_SIZE_BYTES
        );
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
        hwa::info!("Initializing...");

        // Init
        cfg_if::cfg_if! {
            if #[cfg(not(feature = "without-bootloader"))] {
                // Reset bootloader state

                // RCC: Enable HSI and wait for it to be ready
                embassy_stm32::pac::RCC.cr().modify(|w| {
                    w.set_hsion(true)
                });
                while !embassy_stm32::pac::RCC.cr().read().hsirdy() {}

                // RCC: Reset CFGR to defaults
                embassy_stm32::pac::RCC.cfgr().modify(|w| {
                    w.set_sw(embassy_stm32::pac::rcc::vals::Sw::HSI);
                    w.set_sws(embassy_stm32::pac::rcc::vals::Sw::HSI);
                    w.set_hpre(embassy_stm32::pac::rcc::vals::Hpre::DIV1);
                    w.set_ppre1(embassy_stm32::pac::rcc::vals::Ppre::DIV1);
                    w.set_ppre2(embassy_stm32::pac::rcc::vals::Ppre::DIV1);
                    w.set_mco1en(false);
                    w.set_mco2en(false);
                });

                // RCC: Enable HSI only. Wait for PLL to be unready
                embassy_stm32::pac::RCC.cr().write(|w| {
                    w.set_hsion(true)
                });
                while embassy_stm32::pac::RCC.cr().read().pllrdy() {}

                // Reset values from datasheet
                embassy_stm32::pac::RCC.pllcfgr().write_value(embassy_stm32::pac::rcc::regs::Pllcfgr(0x24003010));
                embassy_stm32::pac::RCC.cir().write_value(embassy_stm32::pac::rcc::regs::Cir(0x000000000));
                embassy_stm32::pac::RCC.ahb1enr().write_value(embassy_stm32::pac::rcc::regs::Ahb1enr(0x00100000));
                embassy_stm32::pac::RCC.ahb2enr().write_value(embassy_stm32::pac::rcc::regs::Ahb2enr(0x000000000));
                embassy_stm32::pac::RCC.ahb3enr().write_value(embassy_stm32::pac::rcc::regs::Ahb3enr(0x000000000));
                embassy_stm32::pac::RCC.apb1enr().write_value(embassy_stm32::pac::rcc::regs::Apb1enr(0x00000000));
                embassy_stm32::pac::RCC.apb2enr().write_value(embassy_stm32::pac::rcc::regs::Apb2enr(0x00000000));
                embassy_stm32::pac::RCC.ahb1lpenr().write_value(embassy_stm32::pac::rcc::regs::Ahb1lpenr(0x7E6791FF));
                embassy_stm32::pac::RCC.ahb2lpenr().write_value(embassy_stm32::pac::rcc::regs::Ahb2lpenr(0x000000F1));
                embassy_stm32::pac::RCC.ahb3lpenr().write_value(embassy_stm32::pac::rcc::regs::Ahb3lpenr(0x00000001));
                embassy_stm32::pac::RCC.apb1lpenr().write_value(embassy_stm32::pac::rcc::regs::Apb1lpenr(0x36FEC9FF));
                embassy_stm32::pac::RCC.apb2lpenr().write_value(embassy_stm32::pac::rcc::regs::Apb2lpenr(0x00075F33));
                embassy_stm32::pac::RCC.plli2scfgr().write_value(embassy_stm32::pac::rcc::regs::Plli2scfgr(0x20003000));

                unsafe {
                    hwa::info!("Setting VTOR...");
                    #[allow(unused_mut)]
                    let mut p = cortex_m::Peripherals::steal();
                    hwa::trace!("VTOR WAS AT: {} ", p.SCB.vtor.read());
                    p.SCB.vtor.write(0x7000);
                    hwa::trace!("VTOR SET TO: {} ", p.SCB.vtor.read());
                }
            }
        }

        // RCC
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(embassy_stm32::rcc::Hse {
            freq: embassy_stm32::time::Hertz(8_000_000),
            mode: embassy_stm32::rcc::HseMode::Oscillator,
        });
        config.rcc.pll_src = embassy_stm32::rcc::PllSource::HSE;
        config.rcc.pll = Some(embassy_stm32::rcc::Pll {
            prediv: embassy_stm32::rcc::PllPreDiv::DIV4,
            mul: embassy_stm32::rcc::PllMul::MUL168,
            divp: Some(embassy_stm32::rcc::PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
        config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV2;
        config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = embassy_stm32::rcc::mux::Clk48sel::PLL1_Q;

        let p = embassy_stm32::init(config);

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-serial-usb")] {

                embassy_stm32::bind_interrupts!(struct UsbIrqs {
                    OTG_FS => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
                });

                hwa::info!("Creating USB Driver");
                let usb_cfg = embassy_stm32::usb::Config::default();

                type EndpointBufferType = [u8; 256];
                let buff = hwa::make_static_ref!(
                    "USB Endpoint buffer",
                    EndpointBufferType,
                    [0; 256]
                );
                let drv = embassy_stm32::usb::Driver::new_fs(p.USB_OTG_FS, UsbIrqs, p.PA12, p.PA11, buff, usb_cfg);
                let mut usb_serial_device = io::usb_serial::SerialUsbDevice::new(
                    drv
                );
                hwa::info!("Spawning USB Driver Task");
                usb_serial_device.spawn(_spawner);

                let (usb_serial_rx_device, sender) = usb_serial_device.split();
                let serial_usb_tx = hwa::make_static_async_controller!(
                    "UsbSerialTxController",
                    types::SerialUsbTxMutexStrategy,
                    sender
                );

                let serial_usb_rx_stream = device::SerialUsbInputStream::new(usb_serial_rx_device);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-spi")] {
                let spi = {
                    let mut cfg = embassy_stm32::spi::Config::default();
                    cfg.frequency = embassy_stm32::time::Hertz(Self::SPI_FREQUENCY);
                    hwa::make_static_async_controller!(
                        "SPI1Controller",
                        types::Spi3MutexStrategyType,
                        device::Spi3::new(
                            p.SPI3, p.PC10, p.PC12, p.PC11, p.DMA1_CH5, p.DMA1_CH0, cfg,
                        )
                    )
                };
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-sd-card")] {

                let sd_card_block_device = hwa::sd_card_spi::SPIAdapter::new(
                    spi.clone(),
                    embassy_stm32::gpio::Output::new(p.PC9,
                        embassy_stm32::gpio::Level::High,
                        embassy_stm32::gpio::Speed::VeryHigh
                    )
                );
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {

                let motion_pins = device::MotionPins {
                    #[cfg(feature = "with-x-axis")]
                    x_enable_pin: embassy_stm32::gpio::Output::new(p.PE4, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-y-axis")]
                    y_enable_pin: embassy_stm32::gpio::Output::new(p.PE1, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-z-axis")]
                    z_enable_pin: embassy_stm32::gpio::Output::new(p.PB8, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-e-axis")]
                    e_enable_pin: embassy_stm32::gpio::Output::new(p.PB3, embassy_stm32::gpio::Level::High, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-x-axis")]
                    x_endstop_pin: embassy_stm32::gpio::Input::new(p.PA15, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-y-axis")]
                    y_endstop_pin: embassy_stm32::gpio::Input::new(p.PD2, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-z-axis")]
                    z_endstop_pin: embassy_stm32::gpio::Input::new(p.PC8, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-e-axis")]
                    e_endstop_pin: embassy_stm32::gpio::Input::new(p.PC4, embassy_stm32::gpio::Pull::Down),
                    #[cfg(feature = "with-x-axis")]
                    x_step_pin: embassy_stm32::gpio::Output::new(p.PE3, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-y-axis")]
                    y_step_pin: embassy_stm32::gpio::Output::new(p.PE0, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-z-axis")]
                    z_step_pin: embassy_stm32::gpio::Output::new(p.PB5, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-e-axis")]
                    e_step_pin: embassy_stm32::gpio::Output::new(p.PD6, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-x-axis")]
                    x_dir_pin: embassy_stm32::gpio::Output::new(p.PE2, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-y-axis")]
                    y_dir_pin: embassy_stm32::gpio::Output::new(p.PB9, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-z-axis")]
                    z_dir_pin: embassy_stm32::gpio::Output::new(p.PB4, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    #[cfg(feature = "with-e-axis")]
                    e_dir_pin: embassy_stm32::gpio::Output::new(p.PD3, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                };
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                setup_timer();
            }
        }

        hwa::HwiContext {
            sys_watch_dog: hwa::make_static_async_controller!(
                "WatchDogController",
                types::WatchDogMutexStrategy,
                device::Watchdog::new(p.IWDG, Self::WATCHDOG_TIMEOUT_US)
            ),
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx: todo!("fill me"),
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_tx: todo!("fill me"),
            #[cfg(feature = "with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_rx_stream,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_rx_stream,
            #[cfg(feature = "with-spi")]
            spi,
            #[cfg(feature = "with-i2c")]
            i2c: todo!("fill me"),
            #[cfg(feature = "with-sd-card")]
            sd_card_block_device,
            #[cfg(feature = "with-ps-on")]
            ps_on: todo!("fill me"),
            #[cfg(feature = "with-motion")]
            motion_pins,
            #[cfg(feature = "with-motion-broadcast")]
            motion_sender,
            #[cfg(feature = "with-probe")]
            probe_pwm: todo!("fill me"),
            #[cfg(feature = "with-probe")]
            probe_pwm_channel: todo!("fill me"),
            #[cfg(feature = "with-laser")]
            laser_pwm: todo!("fill me"),
            #[cfg(feature = "with-laser")]
            laser_pwm_channel: todo!("fill me"),
            #[cfg(feature = "with-fan-layer")]
            fan_layer_pwm: todo!("fill me"),
            #[cfg(feature = "with-fan-layer")]
            fan_layer_pwm_channel: todo!("fill me"),
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_pwm: todo!("fill me"),
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_pwm_channel: todo!("fill me"),
            #[cfg(feature = "with-hot-end")]
            hot_end_adc: todo!("fill me"),
            #[cfg(feature = "with-hot-end")]
            hot_end_adc_pin: todo!("fill me"),
            #[cfg(feature = "with-hot-end")]
            hot_end_pwm: todo!("fill me"),
            #[cfg(feature = "with-hot-end")]
            hot_end_pwm_channel: todo!("fill me"),
            #[cfg(feature = "with-hot-bed")]
            hot_bed_adc: todo!("fill me"),
            #[cfg(feature = "with-hot-bed")]
            hot_bed_adc_pin: todo!("fill me"),
            #[cfg(feature = "with-hot-bed")]
            hot_bed_pwm: todo!("fill me"),
            #[cfg(feature = "with-hot-bed")]
            hot_bed_pwm_channel: todo!("fill me"),
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
                todo!()
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
                // Target: 0.000010 seg (10us)
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
