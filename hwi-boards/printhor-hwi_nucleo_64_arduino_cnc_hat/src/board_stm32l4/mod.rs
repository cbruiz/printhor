//! Hardware Interface implementation for nucleo64-L476RG (STM32L476RG)
//!
//! https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32L476RG.json
use printhor_hwa_common as hwa;
use hwa::HwiContract;

mod device;
mod types;

pub(crate) mod io;

#[global_allocator]
static HEAP: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub struct Contract;
impl HwiContract for Contract {
    //#region "Common constants"

    const MACHINE_TYPE: &'static str = "NUCLEO64";
    const MACHINE_BOARD: &'static str = "NUCLEO64-L476RG+Arduino_CNC_Hat_v3.x";
    /// ARM Cortex M4F @80MHZ, 128kB SRAM, 1024kB Program
    const MACHINE_PROCESSOR: &'static str = "STM32L476RG";

    cfg_if::cfg_if! {
        if #[cfg(feature="with-motion")] {
            #[const_env::from_env("PROCESSOR_SYS_CK_MHZ")]
            const PROCESSOR_SYS_CK_MHZ: u32 = 80_000_000;
        }
    }

    //#enregion

    //#region Hard-coded settings [...]

    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;

    #[const_env::from_env("MAX_HEAP_SIZE_BYTES")]
    const MAX_HEAP_SIZE_BYTES: usize = 1024;

    #[const_env::from_env("MAX_EXPECTED_STATIC_ALLOC_BYTES")]
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize = 12384;

    //#endregion

    //#region "Memory management"

    fn heap_current_size() -> usize {
        HEAP.used()
    }

    //#endregion

    //#region Constant settings for with-motion feature [...]

    cfg_if::cfg_if! {
        if #[cfg(feature="with-motion")] {
            #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
            const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 400;

            #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
            const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 40_000;

            #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
            const SEGMENT_QUEUE_SIZE: u8 = 10;
        }
    }

    //#endregion

    //#region Constant settings for with-hot-end feature [...]

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {

            #[const_env::from_env("HOT_END_ADC_V_REF_DEFAULT_MV")]
            const HOT_END_ADC_V_REF_DEFAULT_MV: u16 = 4096;

            #[const_env::from_env("HOT_END_ADC_V_REF_DEFAULT_SAMPLE")]
            const HOT_END_ADC_V_REF_DEFAULT_SAMPLE: u16 = 4096;

            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_BETA: f32 = 3950.0;

            #[const_env::from_env("HOT_END_THERM_NOMINAL_RESISTANCE")]
            const HOT_END_THERM_NOMINAL_RESISTANCE: f32 = 100000.0;

            #[const_env::from_env("HOT_END_THERM_PULL_UP_RESISTANCE")]
            const HOT_END_THERM_PULL_UP_RESISTANCE: f32 = 4685.0;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-bed")] {

            #[const_env::from_env("HOT_BED_ADC_V_REF_DEFAULT_MV")]
            const HOT_BED_ADC_V_REF_DEFAULT_MV: u16 = 4096;

            #[const_env::from_env("HOT_BED_ADC_V_REF_DEFAULT_SAMPLE")]
            const HOT_BED_ADC_V_REF_DEFAULT_SAMPLE: u16 = 4096;

            #[const_env::from_env("HOT_BED_THERM_BETA")]
            const HOT_BED_THERM_BETA: f32 = 3950.0;

            #[const_env::from_env("HOT_BED_THERM_NOMINAL_RESISTANCE")]
            const HOT_BED_THERM_NOMINAL_RESISTANCE: f32 = 100000.0;

            #[const_env::from_env("HOT_BED_THERM_PULL_UP_RESISTANCE")]
            const HOT_BED_THERM_PULL_UP_RESISTANCE: f32 = 4685.0;
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
            compile_error!("not implemented");
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
            type SpiController = types::Spi1MutexStrategyType;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            type MotionPinsMutexStrategy = types::MotionPinsMuxtexStrategy;
            type MotionPins = device::MotionPins;
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

        //#region "RCC"
        let config = {
            let mut config = embassy_stm32::Config::default();

            config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;

            config.rcc.hsi = true;
            config.rcc.msi = Some(embassy_stm32::rcc::MSIRange::RANGE16M);
            config.rcc.pll = Some(embassy_stm32::rcc::Pll {
                source: embassy_stm32::rcc::PllSource::MSI, // HSI: 16Mhz
                prediv: embassy_stm32::rcc::PllPreDiv::DIV2,
                mul: embassy_stm32::rcc::PllMul::MUL20,
                divp: None,
                divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLL1_R 80Mhz (16 / 2 * 20 / 2)
                divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLQ 80Mhz (16 / 2 * 20 / 2)
            });
            config.rcc.pllsai1 = Some(embassy_stm32::rcc::Pll {
                source: embassy_stm32::rcc::PllSource::MSI, // HSI: 16Mhz
                prediv: embassy_stm32::rcc::PllPreDiv::DIV2,
                mul: embassy_stm32::rcc::PllMul::MUL12,
                divp: None,
                divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLLSAl1R 48Mhz (16 / 2 * 12 / 2)
                divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLSAl1Q 48Mhz (16 / 2 * 12 / 2)
            });

            config.rcc.mux.clk48sel = embassy_stm32::rcc::mux::Clk48sel::PLLSAI1_Q;
            config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
            config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
            config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
            cfg_if::cfg_if! {
                if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
                    config.rcc.mux.adcsel = embassy_stm32::rcc::mux::Adcsel::SYS;
                }
            }
            config
        };
        //#endregion
        defmt::info!("embassy init...");
        let p = embassy_stm32::init(config);
        defmt::info!("embassy init done");

        //#region "Prepare shared peripherals"

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-1"))] {

                embassy_stm32::bind_interrupts!(struct UartPort1Irqs {
                    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
                });

                let mut cfg = embassy_stm32::usart::Config::default();
                cfg.baudrate = Self::SERIAL_PORT1_BAUD_RATE;
                cfg.data_bits = embassy_stm32::usart::DataBits::DataBits8;
                cfg.stop_bits = embassy_stm32::usart::StopBits::STOP1;
                cfg.parity = embassy_stm32::usart::Parity::ParityNone;
                cfg.detect_previous_overrun = true;

                let (uart_port1_tx_device, uart_port1_rx_device) = embassy_stm32::usart::Uart::new(
                    p.USART2,
                    p.PA3, p.PA2,
                    UartPort1Irqs,
                    p.DMA1_CH7, p.DMA1_CH6, cfg,
                ).expect("Ready").split();
                let serial_port1_tx = hwa::make_static_async_controller!(
                    "UartPort1Tx",
                    types::SerialPort1TxMutexStrategy,
                    hwa::SerialTxWrapper::new(uart_port1_tx_device, Self::SERIAL_PORT1_BAUD_RATE)
                );
                let serial_port1_rx_stream = device::SerialPort1Rx::new(uart_port1_rx_device);
            }
        }

        #[cfg(all(feature = "with-trinamic"))]
        let trinamic_uart = {
            todo!("Not yet implemented")
        };

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-spi")] {
                let mut cfg = embassy_stm32::spi::Config::default();
                cfg.frequency = embassy_stm32::time::Hertz(Contract::SPI_FREQUENCY);

                let _spi1_device = hwa::make_static_async_controller!(
                    "SPI3",
                    types::Spi1MutexStrategyType,
                    device::Spi::new(
                        p.SPI3, p.PC10, p.PC12, p.PC11, p.DMA2_CH2, p.DMA2_CH1, cfg,
                    )
                );
                hwa::debug!("SPI done");
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-sd-card")] {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-spi")] {
                        let sd_card_device = _spi1_device.clone();
                    }
                    else {
                        compile_error!("with-sd-card requires with-spi in this board");
                    }
                }
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                let motion_pins = device::MotionPins {
                    all_enable_pin: embassy_stm32::gpio::Output::new(p.PA9, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    x_endstop_pin: embassy_stm32::gpio::Input::new(p.PC7, embassy_stm32::gpio::Pull::Down),
                    y_endstop_pin: embassy_stm32::gpio::Input::new(p.PB6, embassy_stm32::gpio::Pull::Down),
                    z_endstop_pin: embassy_stm32::gpio::Input::new(p.PA7, embassy_stm32::gpio::Pull::Down),
                    x_step_pin: embassy_stm32::gpio::Output::new(p.PA10, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    y_step_pin: embassy_stm32::gpio::Output::new(p.PB3, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    z_step_pin: embassy_stm32::gpio::Output::new(p.PB5, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    x_dir_pin: embassy_stm32::gpio::Output::new(p.PB4, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    y_dir_pin: embassy_stm32::gpio::Output::new(p.PB10, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                    z_dir_pin: embassy_stm32::gpio::Output::new(p.PA8, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::VeryHigh),
                };
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-i2c-pwm-motion")] {

                        embassy_stm32::bind_interrupts!(struct I2C1Irqs {
                            I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
                            I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
                        });

                        let i2c_freq = embassy_stm32::time::hz(16_000);
                        let i2c_conf = embassy_stm32::i2c::Config::default();

                        let _i2c_dev = embassy_stm32::i2c::I2c::new(
                            p.I2C1,
                            // SCL
                            p.PB8,
                            // SDA
                            p.PB9,
                            I2C1Irqs,
                            p.DMA2_CH7,
                            p.DMA2_CH6,
                            i2c_freq,
                            i2c_conf,
                        );
                    }
                }
                hwa::debug!("motion_driver done");
            }
        }

        cfg_if::cfg_if!{
            if #[cfg(feature = "with-fan-layer")] {
                let fan_layer_pwm = device::PwmFanLayer::new(
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
            }
        }

        #[cfg(feature = "with-ps-on")]
        let ps_on = hwa::make_static_sync_controller!(
            "PSOn",
            types::PSOnMutexStrategy,
            device::PsOnPin::new(p.PA4, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::Low)
        );

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                let probe_pwm = hwa::make_static_sync_controller!(
                    "ProbeController",
                    types::ProbePwmMutexStrategy,
                    device::PwmProbe::new(
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
                    ),
                );
                let probe_pwm_channel = hwa::HwiResource::new(embassy_stm32::timer::Channel::Ch3);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-laser")] {
                let laser_pwm_device = device::PwmLaser::new(
                    p.TIM8,
                    None,
                    None,
                    None,
                    Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch4(p.PC9, embassy_stm32::gpio::OutputType::PushPull)),
                    embassy_stm32::time::hz(5_000),
                    embassy_stm32::timer::low_level::CountingMode::CenterAlignedBothInterrupts,
                );

                let laser_pwm = hwa::make_static_sync_controller!(
                    "LaserController",
                    types::LaserPwmMutexStrategy,
                    laser_pwm_device
                );

                let laser_pwm_channel = hwa::HwiResource::new(embassy_stm32::timer::Channel::Ch4);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-fan-extra-1")] {
                compile_error!("Not implemented");
            }
        }

        cfg_if::cfg_if!{
            if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
                use embassy_stm32::adc::AdcChannel;
                let hotend_hotbed_adc = hwa::make_static_async_controller!(
                    "AdcHotendHotbedController",
                    types::HotEndHotBedAdcMutexStrategy,
                    device::HotEndHotBedAdc::new(p.ADC1, p.DMA1_CH1)
                );
                let hotend_hotbed_pwm = hwa::make_static_sync_controller!(
                    "PwmHotendHotbedController",
                    types::HotEndHotBedPwmMutexStrategy,
                    device::HotEndHotBedPwm::new(
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
                    ),
                );
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-hot-end")] {
                let hot_end_adc = hotend_hotbed_adc.clone();
                let hot_end_adc_pin = hwa::HwiResource::new(
                    p.PC2.degrade_adc()
                );
                let hot_end_pwm= hotend_hotbed_pwm.clone();
                let hot_end_pwm_channel = hwa::HwiResource::new(embassy_stm32::timer::Channel::Ch1);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-hot-bed")] {
                let hot_bed_adc = hotend_hotbed_adc.clone();
                let hot_bed_adc_pin = hwa::HwiResource::new(
                    p.PC3.degrade_adc()
                );
                let hot_bed_pwm= hotend_hotbed_pwm.clone();
                let hot_bed_pwm_channel = hwa::HwiResource::new(embassy_stm32::timer::Channel::Ch2);
            }
        }

        //#endregion

        // Return the HWI Context with HWI peripherals and controllers

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                setup_timer();
            }
        }

        hwa::HwiContext {
            sys_watch_dog : hwa::make_static_async_controller!(
                "WatchDogController",
                types::WatchDogMutexStrategy,
                device::Watchdog::new(p.IWDG, Self::WATCHDOG_TIMEOUT_US)
            ),
            #[cfg(feature = "with-serial-usb")]
            serial_usb_tx,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_tx,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_tx,
            #[cfg(feature = "with-serial-usb")]
            serial_usb_rx_stream,
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_rx_stream,
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_rx_stream,
            #[cfg(feature = "with-ps-on")]
            ps_on,
            #[cfg(feature = "with-motion")]
            motion_pins,
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
        cortex_m::interrupt::free(|_| f())
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
                hwa::debug!("Ticker Paused");
            }

            fn resume_ticker() {
                hwa::debug!("Ticker Resumed");
                unsafe {
                    let p = cortex_m::Peripherals::steal();
                    let mut syst = p.SYST;
                    syst.enable_interrupt();
                    syst.enable_counter();
                }
                hwa::debug!("Ticker Resumed");
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
