//! Hardware Interface implementation for nucleo64-L476RG (STM32L476RG)
//!
//! https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32L476RG.json
//!
//! https://os.mbed.com/platforms/ST-Nucleo-L476RG/

use embassy_time::Duration;
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

    const MACHINE_TYPE: &'static str = "NUCLEO64";
    const MACHINE_BOARD: &'static str = "NUCLEO64-L476RG+Arduino_CNC_Hat_v3.x";
    /// ARM Cortex M4F @80MHZ, 128kB SRAM, 1024kB Program
    const MACHINE_PROCESSOR: &'static str = "STM32L476RG";
    const PROCESSOR_SYS_CK_MHZ: u32 = 80_000_000;

    //#endregion

    //#region "Watchdog settings"

    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;

    //#endregion

    //#region "Memory management/tracking settings"

    #[const_env::from_env("MAX_HEAP_SIZE_BYTES")]
    const MAX_HEAP_SIZE_BYTES: usize = 1024;

    #[const_env::from_env("MAX_EXPECTED_STATIC_ALLOC_BYTES")]
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize = 8192;

    fn heap_current_size() -> usize {
        HEAP.used()
    }

    //#endregion

    //#region "Feature [with-motion] settings"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            cfg_if::cfg_if! {
                if #[cfg(feature = "with-motion-anthropomorphic-kinematics")] {

                    #[const_env::from_env("SPACE_UNIT_MAGNITUDE")]
                    const SPACE_UNIT_MAGNITUDE: &'static str = "º";

                    #[const_env::from_env("PHYSICAL_UNIT_MAGNITUDE")]
                    const PHYSICAL_UNIT_MAGNITUDE: &'static str = "º";

                    const DEFAULT_WORLD_SIZE_WU: hwa::math::TVector<hwa::math::Real> = const {
                        ANTHROPOMORFIC_WORLD_SIZE_WU
                    };

                    const DEFAULT_WORLD_CENTER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        ANTHROPOMORFIC_WORLD_CENTER_WU
                    };

                    const DEFAULT_WORLD_HOMING_POINT_WU: hwa::math::TVector<hwa::math::Real> = const {
                       hwa::math::TVector::new_with_coord(hwa::math::CoordSel::all_axis(), hwa::make_optional_real!(0.0))
                    };

                    fn project_to_space(&self, world_pos: &hwa::math::TVector<hwa::math::Real>) -> Result<hwa::math::TVector<hwa::math::Real>, ()> {
                        use hwa::kinematics::WorldToSpaceTransformer;
                        let traslated_world_pos = world_pos.clone() + ANTHROPOMORFIC_WORLD_CENTER_WU;
                        ANTHROPOMORFIC_TRANSFORMER.project_to_space(&(traslated_world_pos))
                    }

                    fn project_to_world(&self, space_pos: &hwa::math::TVector<hwa::math::Real>) -> Result<hwa::math::TVector<hwa::math::Real>, ()> {
                        use hwa::kinematics::WorldToSpaceTransformer;
                        ANTHROPOMORFIC_TRANSFORMER.project_to_world(&(*space_pos)).and_then(|pos|
                            Ok(pos - ANTHROPOMORFIC_WORLD_CENTER_WU)
                        )
                    }
                    
                    /// Apply calculated max feed rate boundaries
                    const CLAMP_MAX_FEED_RATE: bool = false;

                    /// Default max speed in Physical Units / second
                    /// Reference: MG90S Analog Servo: 0.1s/60º @4.8Volt => 600.240096 º/seg
                    const DEFAULT_MAX_SPEED_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::math::TVector::new_with_coord(hwa::math::CoordSel::all_axis(), hwa::make_optional_real!(2400.0))
                    };
                    /// Default max speed in physical units by second
                    const DEFAULT_MAX_ACCEL_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::math::TVector::new_with_coord(hwa::math::CoordSel::all_axis(), hwa::make_optional_real!(9600.0))
                    };
                    /// Default max speed in physical units by second
                    const DEFAULT_MAX_JERK_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::math::TVector::new_with_coord(hwa::math::CoordSel::all_axis(), hwa::make_optional_real!(19200.0))
                    };
                    /// Default max speed in physical units by second
                    const DEFAULT_TRAVEL_SPEED_PS: hwa::math::Real = const {
                        hwa::make_real!(1200.0)
                    };

                    /// Default units per workspace unit
                    ///
                    /// Reference: MG90S Analog Servo and PCA9685 (12 bit counter).
                    ///
                    /// pwm count by angle is 1.1375, which is higher than unit, so we are just the finest we can
                    const DEFAULT_UNITS_PER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::math::TVector::new_with_coord(hwa::math::CoordSel::all_axis(), hwa::make_optional_real!(1.0))
                    };

                    /// Default micro-steps per axis
                    ///
                    /// Reference: MG90S Analog Servo: Dead-band width: 5 µs:
                    ///
                    /// With PCA9685, which has 12 bits counter, it's 0.005000 period secs per count. Much higher than dead-band width.
                    const DEFAULT_MICRO_STEPS_PER_AXIS: hwa::math::TVector<u16> = const {
                        hwa::math::TVector::new_with_coord(hwa::math::CoordSel::all_axis(), Some(2))
                    };

                    #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
                    const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 50;

                    #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
                    const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 100;

                    #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
                    const SEGMENT_QUEUE_SIZE: u8 = 10;

                }
                else if #[cfg(feature = "with-motion-delta-kinematics")] {
                    compile_error!("Work in progress");
                }
                else { // Implicitly Assume with-motion-cartessian-kinematic


                    const DEFAULT_WORLD_SIZE_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=200.0, y=200.0, z=200.0, a=200.0, b=200.0, c=200.0)
                    };

                    const DEFAULT_WORLD_CENTER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=100.0, y=100.0, z=100.0, a=100.0, b=100.0, c=100.0)
                    };

                    const DEFAULT_MAX_SPEED_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=600.0, y=600.0, z=100.0, e=300.0, a=1200.0, b=1200.0, c=1200.0)
                    };

                    const DEFAULT_MAX_ACCEL_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=9800.0, y=9800.0, z=4800.0, e=9800.0, a=9800.0, b=9800.0, c=9800.0)
                    };

                    const DEFAULT_MAX_JERK_PS: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=19600.0, y=19600.0, z=9600.0, e=19600.0, a=19600.0, b=19600.0, c=19600.0)
                    };

                    const DEFAULT_TRAVEL_SPEED_PS: hwa::math::Real = const {
                        hwa::make_real!(600.0)
                    };

                    const DEFAULT_UNITS_PER_WU: hwa::math::TVector<hwa::math::Real> = const {
                        hwa::make_vector_real!(x=50.0, y=50.0, z=10.0, e=50.0, a=1.0, b=1.0, c=1.0)
                    };


                    const DEFAULT_MICRO_STEPS_PER_AXIS: hwa::math::TVector<u16> = const {
                        hwa::make_vector!(x=2, y=2, z=2, e=2, a=10, b=10, c=10)
                    };

                    #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
                    const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 100;

                    #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
                    const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 100_000;

                    #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
                    const SEGMENT_QUEUE_SIZE: u8 = 10;

                }
            }
        }
    }

    //#endregion

    //#region "Constant settings for with-hot-end feature [...]"

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
            const SERIAL_PORT1_RX_BUFFER_SIZE: usize = 512;

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
        if #[cfg(feature = "with-i2c")] {
            #[const_env::from_env("I2C_FREQUENCY")]
            const I2C_FREQUENCY: u32 = 500_000;
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

            cfg_if::cfg_if! {
                if #[cfg(feature="hse-stlink-mso")] {

                    config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;

                    config.rcc.hsi = false;
                    config.rcc.hse = Some(
                        embassy_stm32::rcc::Hse {
                            mode: embassy_stm32::rcc::HseMode::Bypass,
                            freq: embassy_stm32::time::Hertz(8_000_000),
                        }
                    );
                    config.rcc.msi = None;
                    config.rcc.pll = Some(embassy_stm32::rcc::Pll {
                        source: embassy_stm32::rcc::PllSource::HSE, // HSE: 8Mhz
                        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
                        mul: embassy_stm32::rcc::PllMul::MUL20,
                        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7), // PLLP 22.85Mhz (8 / 1 * 20 / 7)
                        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLL1_R 80Mhz (8 / 1 * 20 / 2)
                        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLQ 80Mhz (8 / 1 * 20 / 2)
                    });
                    config.rcc.pllsai1 = Some(embassy_stm32::rcc::Pll {
                        source: embassy_stm32::rcc::PllSource::HSE, // HSE: 8Mhz
                        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
                        mul: embassy_stm32::rcc::PllMul::MUL12,
                        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7), // PLLSAl1P 13.71Mhz (8 / 1 * 12 / 7)
                        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLLSAl1R 48Mhz (8 / 1 * 12 / 2)
                        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLSAl1Q 48Mhz (8 / 1 * 12 / 2)
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
                }
                else {
                    config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;

                    config.rcc.hsi = true;
                    config.rcc.hse = None;
                    config.rcc.msi = Some(embassy_stm32::rcc::MSIRange::RANGE16M);
                    config.rcc.pll = Some(embassy_stm32::rcc::Pll {
                        source: embassy_stm32::rcc::PllSource::HSI, // HSI: 16Mhz
                        prediv: embassy_stm32::rcc::PllPreDiv::DIV2,
                        mul: embassy_stm32::rcc::PllMul::MUL20,
                        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7), // PLLP 22.85Mhz (16 / 2 * 20 / 7)
                        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLL1_R 80Mhz (16 / 2 * 20 / 2)
                        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLQ 80Mhz (16 / 2 * 20 / 2)
                    });
                    config.rcc.pllsai1 = Some(embassy_stm32::rcc::Pll {
                        source: embassy_stm32::rcc::PllSource::HSI, // HSI: 16Mhz
                        prediv: embassy_stm32::rcc::PllPreDiv::DIV2,
                        mul: embassy_stm32::rcc::PllMul::MUL12,
                        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7), // PLLSAl1P 13.71Mhz (16 / 2 * 12 / 7)
                        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // PLLSAl1R 48Mhz (16 / 2 * 12 / 2)
                        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // PLLSAl1Q 48Mhz (16 / 2 * 12 / 2)
                    });

                    //config.rcc.mux.clk48sel = embassy_stm32::rcc::mux::Clk48sel::PLLSAI1_Q;
                    config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
                    config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
                    config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
                    cfg_if::cfg_if! {
                        if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
                            config.rcc.mux.adcsel = embassy_stm32::rcc::mux::Adcsel::SYS;
                        }
                    }
                }
            }

            config
        };

        //#endregion
        hwa::info!("embassy init...");
        let p = embassy_stm32::init(config);
        embassy_time::Timer::after(Duration::from_millis(1000)).await;
        hwa::info!("embassy init done");

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
                cfg.frequency = embassy_stm32::time::Hertz(Self::SPI_FREQUENCY);

                let spi = hwa::make_static_async_controller!(
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
                        let sd_card_device = spi.clone();
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
                    if #[cfg(any(feature = "with-i2c", feature = "with-motion-broadcast"))] {

                        embassy_stm32::bind_interrupts!(struct I2C1Irqs {
                            I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
                            I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
                        });

                        let i2c_freq = embassy_stm32::time::hz(Self::I2C_FREQUENCY);
                        let mut i2c_conf = embassy_stm32::i2c::Config::default();
                        i2c_conf.timeout = Duration::from_millis(20);

                        let i2c = hwa::make_static_async_controller!(
                            "I2C1",
                            types::I2cMutexStrategyType,

                            io::MotionI2c::new(embassy_stm32::i2c::I2c::new(
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
                            )).await
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
            #[cfg(feature = "with-spi")]
            spi,
            #[cfg(feature = "with-i2c")]
            i2c: i2c.clone(),
            #[cfg(feature = "with-ps-on")]
            ps_on,
            #[cfg(feature = "with-motion")]
            motion_pins,
            #[cfg(feature = "with-motion-broadcast")]
            motion_sender: i2c,
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
        // TODO: Thinking it's causing issues
        //cortex_m::interrupt::free(|_| f())
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

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion-anthropomorphic-kinematics")] {

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis"))] {
                // If ANY OF x, y, z is set...
                cfg_if::cfg_if! {
                    if #[cfg(not(all(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis")))] {
                        // ALL x, y, z must be set...
                        compile_error!("with-motion-anthropomorphic-kinematics requires all (x, y, z) axes if any of them is set");
                    }
                }
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "with-a-axis", feature = "with-b-axis", feature = "with-c-axis"))] {
                // If ANY OF a, b, c is set...
                cfg_if::cfg_if! {
                    if #[cfg(not(all(feature = "with-a-axis", feature = "with-b-axis", feature = "with-c-axis")))] {
                        // ALL a, c, c must be set...
                        compile_error!("with-motion-anthropomorphic-kinematics requires all (a, b, c) axes if any of them is set");
                    }
                }
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "with-i-axis", feature = "with-j-axis", feature = "with-k-axis"))] {
                // If ANY OF i, j, k is set...
                cfg_if::cfg_if! {
                    if #[cfg(not(all(feature = "with-i-axis", feature = "with-j-axis", feature = "with-k-axis")))] {
                        // ALL i, j, k must be set...
                        compile_error!("with-motion-anthropomorphic-kinematics requires all (i, j, k) axes if any of them is set");
                    }
                }
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "with-u-axis", feature = "with-v-axis", feature = "with-w-axis"))] {
                // If ANY OF u, v, w is set...
                cfg_if::cfg_if! {
                    if #[cfg(not(all(feature = "with-u-axis", feature = "with-v-axis", feature = "with-w-axis")))] {
                        // ALL u, v, w must be set...
                        compile_error!("with-motion-anthropomorphic-kinematics requires all (u, v, w) axes if any of them is set");
                    }
                }
            }
        }

        const ANTHROPOMORFIC_WORLD_CENTER_WU: hwa::math::TVector<hwa::math::Real> = hwa::make_vector_real!(
            x=55.8614311, y=55.8614311, z=0.0,
            a=55.8614311, b=55.8614311, c=0.0,
            i=55.8614311, j=55.8614311, k=0.0,
            u=55.8614311, v=55.8614311, w=0.0
        );

        pub(crate) const ANTHROPOMORFIC_SPACE_CALIBRATION: hwa::math::TVector<hwa::math::Real> = hwa::make_vector_real!(
            x=0.0, y=10.0, z=10.0,
            a=0.0, b=-15.0, c=40.0,
            i=0.0, j=0.0, k=0.0,
            u=0.0, v=0.0, w=0.0
        );
        pub(crate) const ANTHROPOMORFIC_SPACE_DIR: hwa::math::TVector<hwa::math::Real> = hwa::make_vector_real!(
            x=1.0, y=-1.0, z=1.0,
            a=1.0, b=1.0, c=-1.0,
            i=1.0, j=1.0, k=-1.0,
            u=1.0, v=-1.0, w=1.0
        );

        const ANTHROPOMORFIC_WORLD_SIZE_WU: hwa::math::TVector<hwa::math::Real> = hwa::make_vector_real!(
            x=250.0, y=250.0, z=140.0,
            a=250.0, b=250.0, c=140.0,
            i=250.0, j=250.0, k=140.0,
            u=250.0, v=250.0, w=140.0,
        );

        pub(crate) const ANTHROPOMORFIC_TRANSFORMER: hwa::kinematics::anthropomorphic_3dof::Quadruped =
            hwa::kinematics::anthropomorphic_3dof::Quadruped::new(
                #[cfg(all(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis"))]
                hwa::kinematics::anthropomorphic_3dof::SingleActuator::new(
                    hwa::make_real!(70.0),
                    hwa::make_real!(24.0),
                    hwa::make_real!(55.0),
                    hwa::make_real!(70.0),
                    // Initial theta
                    hwa::make_real!(45.0),
                    // Initial phi
                    hwa::make_real!(0.0),
                    // Initial psi
                    hwa::make_real!(90.0),
                    hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z,
                    hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z,
                ),
            #[cfg(all(feature = "with-a-axis", feature = "with-b-axis", feature = "with-c-axis"))]
                hwa::kinematics::anthropomorphic_3dof::SingleActuator::new(
                    hwa::make_real!(70.0),
                    hwa::make_real!(24.0),
                    hwa::make_real!(55.0),
                    hwa::make_real!(70.0),
                    // Initial theta
                    hwa::make_real!(45.0),
                    // Initial phi
                    hwa::make_real!(0.0),
                    // Initial psi
                    hwa::make_real!(90.0),
                    hwa::CoordSel::A, hwa::CoordSel::B, hwa::CoordSel::C,
                    hwa::CoordSel::A, hwa::CoordSel::B, hwa::CoordSel::C,
                ),
                #[cfg(all(feature = "with-i-axis", feature = "with-j-axis", feature = "with-k-axis"))]
                hwa::kinematics::anthropomorphic_3dof::SingleActuator::new(
                    hwa::make_real!(70.0),
                    hwa::make_real!(24.0),
                    hwa::make_real!(55.0),
                    hwa::make_real!(70.0),
                    // Initial theta
                    hwa::make_real!(45.0),
                    // Initial phi
                    hwa::make_real!(0.0),
                    // Initial psi
                    hwa::make_real!(90.0),
                    hwa::CoordSel::I, hwa::CoordSel::J, hwa::CoordSel::K,
                    hwa::CoordSel::I, hwa::CoordSel::J, hwa::CoordSel::K,
                ),
                #[cfg(all(feature = "with-u-axis", feature = "with-v-axis", feature = "with-w-axis"))]
                hwa::kinematics::anthropomorphic_3dof::SingleActuator::new(
                    hwa::make_real!(70.0),
                    hwa::make_real!(24.0),
                    hwa::make_real!(55.0),
                    hwa::make_real!(70.0),
                    // Initial theta
                    hwa::make_real!(45.0),
                    // Initial phi
                    hwa::make_real!(0.0),
                    // Initial psi
                    hwa::make_real!(90.0),
                    hwa::CoordSel::U, hwa::CoordSel::V, hwa::CoordSel::W,
                    hwa::CoordSel::U, hwa::CoordSel::V, hwa::CoordSel::W,
                ),
            );
    }
}