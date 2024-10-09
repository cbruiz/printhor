use printhor_hwa_common as hwa;
//use printhor_hwa_common::SerialTxWrapper;
//use printhor_hwa_utils::HwiResource;

pub mod device;
mod types;

pub(crate) mod io;

#[global_allocator]
static HEAP: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub struct Contract();
impl hwa::HwiContract for Contract {

    //#region "Common constants"

    const MACHINE_TYPE: &'static str = "NUCLEO64";
    const MACHINE_BOARD: &'static str = "NUCLEO64_Arduino_CNC_Hat_v3.x";
    /// ARM Cortex M4F @100MHZ, 32kB SRAM, 128kB Program
    const MACHINE_PROCESSOR: &'static str = "STM32F410RB";

    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 10_000_000;

    #[const_env::from_env("HEAP_SIZE_BYTES")]
    const HEAP_SIZE_BYTES: usize = 1024;

    #[const_env::from_env("MAX_EXPECTED_STATIC_ALLOC_BYTES")]
    const MAX_EXPECTED_STATIC_ALLOC_BYTES: usize = 12384;

    //#endregion

    //#region "Memory management"

    fn heap_current_size() -> usize {
        HEAP.used()
    }

    //#endregion

    //#region "Motion constants"

    cfg_if::cfg_if! {
        if #[cfg(feature="with-motion")] {
            #[const_env::from_env("MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY")]
            const MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY: u32 = 400;

            #[const_env::from_env("STEP_PLANNER_CLOCK_FREQUENCY")]
            const STEP_PLANNER_CLOCK_FREQUENCY: u32 = 40_000;

            #[const_env::from_env("SEGMENT_QUEUE_SIZE")]
            const SEGMENT_QUEUE_SIZE: usize = 0;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature="with-hot-end")] {
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
            #[const_env::from_env("HOT_BED_THERM_BETA")]
            const HOT_BED_THERM_BETA: f32 = 3950.0;

            #[const_env::from_env("HOT_BED_THERM_NOMINAL_RESISTANCE")]
            const HOT_BED_THERM_NOMINAL_RESISTANCE: f32 = 100000.0;

            #[const_env::from_env("HOT_BED_THERM_PULL_UP_RESISTANCE")]
            const HOT_BED_THERM_PULL_UP_RESISTANCE: f32 = 4685.0;
        }
    }

    //#endregion

    type EventBusPubSubMutexType = types::EventBusPubSubMutexType;

    // To build eventbus

    #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))]
    type DeferChannelMutexType = types::DeferChannelMutexType;

    type WatchDogMutexStrategy = types::WatchDogMutexStrategy;
    type EventBusMutexStrategy = types::EventBusMutexStrategy;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-port-1")] {

            #[const_env::from_env("SERIAL_PORT1_BAUD_RATE")]
            const SERIAL_PORT1_BAUD_RATE: u32 = 115200;
            const SERIAL_PORT1_RX_BUFFER_SIZE: usize = 128;

            type SerialPort1Tx = types::SerialPort1TxMutexStrategy;
            type SerialPort1Rx = hwa::HwiResource<super::io::uart_port1::UartPort1RxInputStream>;
        }
    }


    #[cfg(feature = "with-serial-usb")]
    type SerialUsbRx = hwa::HwiResource<crate::device::SerialUsbInputStream>;

    #[cfg(feature = "with-serial-usb")]
    type SerialUsbTx = crate::SerialPort1MutexStrategyType<device::SerialUsbTxDevice>;

    #[cfg(feature = "with-serial-port-2")]
    type SerialPort2Tx = crate::SerialPort2MutexStrategyType<device::SerialPort2TxDevice>;

    #[cfg(feature = "with-serial-port-2")]
    type SerialPort2Rx = hwa::HwiResource<crate::device::SerialPort2InputStream>;
    #[cfg(feature = "with-ps-on")]
    type PSOnMutexStrategy = crate::PSOnMutexStrategyType<crate::device::PsOnPin>;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            type ProbePowerPwm = crate::PwmProbeMutexStrategyType<device::PwmProbe>;
            type ProbePowerPwmChannel = hwa::HwiResource<crate::device::PwmProbeChannel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-laser")] {
            type LaserPowerPwm = crate::PwmLaserMutexStrategyType<device::PwmLaser>;
            type LaserPowerPwmChannel = hwa::HwiResource<crate::device::PwmLaserChannel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-layer")] {
            type FanLayerPowerPwm = crate::PwmLaserMutexStrategyType<device::PwmFanLayer>;
            type FanLayerPowerPwmChannel = hwa::HwiResource<crate::device::PwmFanLayerChannel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-extra-1")] {
            type FanExtra1PowerPwm = crate::PwmFanLayerMutexStrategyType<device::PwmFanExtra1>;
            type FanExtra1PowerPwmChannel = hwa::HwiResource<crate::device::PwmFanExtra1Channel>;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-end")] {
            type HotEndAdc = crate::AdcHotEndMutexStrategyType<device::AdcHotEnd>;
            type HotEndAdcPin = hwa::HwiResource<crate::device::AdcHotEndPin>;
            type HotEndPowerPwm = crate::PwmHotEndMutexStrategyType<device::PwmHotEnd>;
            type HotEndPwmChannel = hwa::HwiResource<crate::device::PwmHotEndChannel>;
        }
    }

    fn init_logger() {}

    fn init_heap() {
        hwa::info!("Initializing heap ({}) bytes", <Contract as hwa::HwiContract>::HEAP_SIZE_BYTES);
        use core::mem::MaybeUninit;
        #[link_section = ".bss"]
        static mut HEAP_MEM: [MaybeUninit<u8>; <Contract as hwa::HwiContract>::HEAP_SIZE_BYTES] =
            [MaybeUninit::uninit(); <Contract as hwa::HwiContract>::HEAP_SIZE_BYTES];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, <Contract as hwa::HwiContract>::HEAP_SIZE_BYTES) }
    }

    async fn init(_spawner: embassy_executor::Spawner) -> hwa::HwiContext<Self> {


        //#region "RCC"
        let config = {
            let mut config = embassy_stm32::Config::default();
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
        let p = embassy_stm32::init(config);

        //#region "Prepare shared peripherals"

        #[cfg(feature = "with-serial-port-1")]
        let (serial_port1_tx, serial_port1_rx_stream) = {

            embassy_stm32::bind_interrupts!(struct UartPort1Irqs {
                USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
            });

            let mut cfg = embassy_stm32::usart::Config::default();
            cfg.baudrate = Self::SERIAL_PORT1_BAUD_RATE;
            cfg.data_bits = embassy_stm32::usart::DataBits::DataBits8;
            cfg.stop_bits = embassy_stm32::usart::StopBits::STOP1;
            cfg.parity = embassy_stm32::usart::Parity::ParityNone;
            cfg.detect_previous_overrun = true;

            let (uart_port1_tx, uart_port1_rx) = embassy_stm32::usart::Uart::new(
                p.USART2,
                p.PA3, p.PA2,
                UartPort1Irqs,
                p.DMA1_CH6, p.DMA1_CH7, cfg,
            ).expect("Ready").split();
            (
                hwa::make_static_controller!(
                    "UartPort1",
                    types::SerialPort1TxMutexStrategy,
                    hwa::SerialTxWrapper::new(uart_port1_tx, Self::SERIAL_PORT1_BAUD_RATE)
                ),
                hwa::HwiResource::new(io::uart_port1::UartPort1RxInputStream::new(uart_port1_rx)),
            )
        };

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-2"))] {
                let (uart_port2_tx_device, uart_port2_rx_device) = device::SerialPort2Device::new().split();
                let serial_port2_tx = hwa::make_static_controller!(
                    "UartPort2Tx",
                    crate::SerialPort2MutexStrategyType<device::SerialPort2TxDevice>,
                    uart_port2_tx_device
                );
                let serial_port2_rx_stream = hwa::HwiResource::new(device::SerialPort2InputStream::new(uart_port2_rx_device));
            }
        }

        #[cfg(all(feature = "with-trinamic"))]
        let trinamic_uart = {
            device::TrinamicUart::new(
                TRINAMIC_UART_BAUD_RATE,
                mocked_peripherals::MockedIOPin::new(0, _pin_state),
                mocked_peripherals::MockedIOPin::new(1, _pin_state),
                mocked_peripherals::MockedIOPin::new(2, _pin_state),
                mocked_peripherals::MockedIOPin::new(3, _pin_state),
            )
        };

        #[cfg(all(feature = "with-trinamic"))]
        {
            _spawner
                .spawn(device::trinamic_driver_simulator(
                    device::MockedTrinamicDriver::new(
                        mocked_peripherals::MockedIOPin::new(0, _pin_state),
                        mocked_peripherals::MockedIOPin::new(1, _pin_state),
                        mocked_peripherals::MockedIOPin::new(2, _pin_state),
                        mocked_peripherals::MockedIOPin::new(3, _pin_state),
                    ),
                ))
                .unwrap();
        }

        #[cfg(feature = "with-spi")]
        let spi1_device = hwa::make_static_controller!(
            "SPI1",
            crate::Spi1MutexStrategyType<device::Spi>,
            device::Spi::new()
        );

        #[cfg(feature = "with-spi")]
        hwa::debug!("SPI done");

        #[cfg(feature = "with-sd-card")]
        let sd_card_device = { device::SDCardBlockDevice::new("data/sdcard.img", false).unwrap() };

        #[cfg(feature = "with-motion")]
        let motion_devices = device::MotionDevice {
            #[cfg(feature = "with-trinamic")]
            trinamic_uart,
            motion_pins: device::MotionPins {
                x_enable_pin: mocked_peripherals::MockedIOPin::new(4, _pin_state),
                y_enable_pin: mocked_peripherals::MockedIOPin::new(5, _pin_state),
                z_enable_pin: mocked_peripherals::MockedIOPin::new(6, _pin_state),
                e_enable_pin: mocked_peripherals::MockedIOPin::new(7, _pin_state),
                x_endstop_pin: mocked_peripherals::MockedIOPin::new(8, _pin_state),
                y_endstop_pin: mocked_peripherals::MockedIOPin::new(9, _pin_state),
                z_endstop_pin: mocked_peripherals::MockedIOPin::new(10, _pin_state),
                e_endstop_pin: mocked_peripherals::MockedIOPin::new(11, _pin_state),
                x_step_pin: mocked_peripherals::MockedIOPin::new(12, _pin_state),
                y_step_pin: mocked_peripherals::MockedIOPin::new(13, _pin_state),
                z_step_pin: mocked_peripherals::MockedIOPin::new(14, _pin_state),
                e_step_pin: mocked_peripherals::MockedIOPin::new(15, _pin_state),
                x_dir_pin: mocked_peripherals::MockedIOPin::new(16, _pin_state),
                y_dir_pin: mocked_peripherals::MockedIOPin::new(17, _pin_state),
                z_dir_pin: mocked_peripherals::MockedIOPin::new(18, _pin_state),
                e_dir_pin: mocked_peripherals::MockedIOPin::new(19, _pin_state),
            },
        };
        #[cfg(feature = "with-motion")]
        hwa::debug!("motion_driver done");

        #[cfg(any(
            feature = "with-probe",
            feature = "with-hot-end",
            feature = "with-hot-bed",
            feature = "with-fan-layer",
            feature = "with-fan-extra-1",
            feature = "with-laser"
        ))]
        let pwm_any = hwa::make_static_controller!(
            "Pwm1Controller",
            crate::Pwm1ControllerMutexStrategyType<device::PwmAny>,
            device::PwmAny::new(20, _pin_state)
        );

        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        let adc_any = hwa::make_static_controller!(
            "Adc1Controller",
            crate::Adc1ControllerMutexStrategyType<device::Adc1>,
            device::AdcHotEnd::new(0)
        );

        #[cfg(feature = "with-motion")]
        hwa::debug!("motion_planner done");

        #[cfg(feature = "with-ps-on")]
        let ps_on = hwa::make_static_controller!(
            "PSOn",
            crate::PSOnMutexStrategyType<device::PsOnPin>,
            device::PsOnPin::new(21, _pin_state)
        );

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                let probe_power_pwm = pwm_any.clone();
                let probe_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-laser")] {
                let laser_power_pwm = pwm_any.clone();
                let laser_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-fan-layer")] {
                let fan_layer_power_pwm = pwm_any.clone();
                let fan_layer_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-fan-extra-1")] {
                let fan_extra1_power_pwm = pwm_any.clone();
                let fan_extra1_power_channel = hwa::HwiResource::new(0u8);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-hot-end")] {
                let hot_end_adc = adc_any.clone();
                let hot_end_adc_pin = hwa::HwiResource::new(mocked_peripherals::MockedIOPin::new(23, _pin_state));
                let hot_end_power_pwm= pwm_any.clone();
                let hot_end_power_channel = hwa::HwiResource::new(0u8);
            }
        }

        // Return the HWI Context with HWI peripherals and controllers

        hwa::HwiContext {
            sys_watch_dog : hwa::make_static_controller!(
                "Watchdog",
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
            #[cfg(feature = "with-probe")]
            probe_power_pwm,
            #[cfg(feature = "with-probe")]
            probe_power_channel,
            #[cfg(feature = "with-laser")]
            laser_power_pwm,
            #[cfg(feature = "with-laser")]
            laser_power_channel,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_power_pwm,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_power_channel,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_power_pwm,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1_power_channel,
            #[cfg(feature = "with-hot-end")]
            hot_end_adc,
            #[cfg(feature = "with-hot-end")]
            hot_end_adc_pin,
            #[cfg(feature = "with-hot-end")]
            hot_end_power_pwm,
            #[cfg(feature = "with-hot-end")]
            hot_end_power_channel,
        }
    }

    fn sys_reset() {}

    fn sys_stop() {}

    // Execute closure f in an interrupt-free context.
    // In native this is not required, so does nothing
    fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce() -> R,
    {
        f()
    }

    fn pause_ticker() {
        todo!()
    }

    fn resume_ticker() {
        todo!()
    }
}

// As of not, the AdcTraits must be exported this way

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        pub use device::AdcTrait as HotEndAdcTrait;
        pub use device::AdcPinTrait as HotEndAdcPinTrait;

    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-bed")] {
        pub use device::AdcTrait as HotBedAdcTrait;
        pub use device::AdcPinTrait as HotBedAdcPinTrait;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {

    }
}

