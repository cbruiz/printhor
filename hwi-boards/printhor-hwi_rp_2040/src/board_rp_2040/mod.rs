//! HWI module for RPI Pico (RP 2040)
//!
//! https://github.com/raspberrypi/pico-sdk/blob/1.5.0/src/rp2040/hardware_regs/rp2040.svd

use printhor_hwa_common as hwa;
use hwa::HwiContract;
use printhor_hwa_common::SerialTxWrapper;

mod device;
mod types;

pub(crate) mod io;

#[global_allocator]
static HEAP: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub struct Contract;
impl HwiContract for Contract {

    //#region "Board specific constants"

    const MACHINE_TYPE: &'static str = "RP2040";
    const MACHINE_BOARD: &'static str = "TBD";
    /// ARM Cortex M0+ @133MHZ, 264kB SRAM, 2048kB Program
    const MACHINE_PROCESSOR: &'static str = "RP2040";

    const PROCESSOR_SYS_CK_MHZ: u32 = 133_000_000;

    /// The target [hwa::CommChannel] for M117 (display)
    const DISPLAY_CHANNEL: hwa::CommChannel = hwa::CommChannel::Internal;

    /// The target [hwa::CommChannel] for M118 (host)
    const HOST_CHANNEL: hwa::CommChannel = hwa::CommChannel::Internal;

    //#endregion

    //#region "Watchdog settings"

    #[const_env::from_env("WATCHDOG_TIMEOUT_US")]
    const WATCHDOG_TIMEOUT_US: u32 = 5_000_000;

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
                if #[cfg(feature = "with-motion-core-xy-kinematics")] {
                    compile_error!("Not implemented");
                }
                else if #[cfg(feature = "with-motion-delta-kinematics")] {
                    compile_error!("Not implemented");
                }
                else if #[cfg(feature = "with-motion-anthropomorphic-kinematics")] {
                    compile_error!("Not implemented");
                }
                else {
                    // Assuming #[cfg(feature = "with-motion-cartessian-kinematics")]
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

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-serial-usb")] {
            #[const_env::from_env("SERIAL_USB_PACKET_SIZE")]
            const SERIAL_USB_PACKET_SIZE: usize = 64;

            type SerialUsbTx = types::SerialUsbTxMutexStrategy;
            type SerialUsbRx = io::serial_usb::SerialUsbInputStream;
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
        }
    }

    //#endregion

    fn init_heap() {
        hwa::info!(
            "Initializing heap ({}) bytes",
            Contract::MAX_HEAP_SIZE_BYTES
        );
        use core::mem::MaybeUninit;
        #[unsafe(link_section = ".bss")]
        static mut HEAP_MEM: [MaybeUninit<u8>; Contract::MAX_HEAP_SIZE_BYTES] =
            [MaybeUninit::uninit(); Contract::MAX_HEAP_SIZE_BYTES];
        unsafe {
            #[allow(static_mut_refs)]
            HEAP.init(HEAP_MEM.as_ptr() as usize, Contract::MAX_HEAP_SIZE_BYTES)
        }
    }

    async fn init(_spawner: embassy_executor::Spawner) -> hwa::HwiContext<Self> {

        //#region "Bootloader reset"

        //#endregion

        //#region "RCC setup"

        let config = {
            let config = embassy_rp::config::Config::default();
            config
        };

        let p = embassy_rp::init(config);

        let watchdog = device::Watchdog::new(p.WATCHDOG);

        //#endregion

        //#region "with-serial-usb"

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-serial-usb")] {

                embassy_rp::bind_interrupts!(struct UsbIrqs {
                    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
                });

                hwa::info!("Creating USB Driver");

                let mut usb_serial_device = io::serial_usb::SerialUsbDevice::new(
                    device::SerialUsbDriver::new(p.USB, UsbIrqs)
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

        //#endregion

        //#region "with-serial-port-1"

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-1"))] {

                embassy_rp::bind_interrupts!(struct SerialPort1IRQs {
                    UART0_IRQ => embassy_rp::uart::InterruptHandler<embassy_rp::peripherals::UART0>;
                });

                let mut cfg = embassy_rp::uart::Config::default();
                cfg.baudrate = Self::SERIAL_PORT1_BAUD_RATE;
                cfg.data_bits = embassy_rp::uart::DataBits::DataBits8;
                cfg.stop_bits = embassy_rp::uart::StopBits::STOP1;
                cfg.parity = embassy_rp::uart::Parity::ParityNone;

                let dev = embassy_rp::uart::Uart::new(p.UART0,
                    p.PIN_0, p.PIN_1,
                    SerialPort1IRQs,
                    p.DMA_CH0,
                    p.DMA_CH1,
                    cfg,
                );
                let (serial_port_1_tx_device, serial_port1_rx_device) =
                    dev.split();

                let serial_port_1_tx = hwa::make_static_async_controller!(
                    "SerialPort1Tx",
                    types::SerialPort1TxMutexStrategy,
                    SerialTxWrapper::new(io::serial_port_1::SerialPort1TxAdapter::new(serial_port_1_tx_device), Self::SERIAL_PORT1_BAUD_RATE),
                );
                let serial_port_1_rx_stream = io::serial_port_1::SerialPort1RxInputStream::new(serial_port1_rx_device);
            }
        }

        //#endregion

        //#region "with-serial-port-1"

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-serial-port-2"))] {

                embassy_rp::bind_interrupts!(struct SerialPort2IRQs {
                    UART1_IRQ => embassy_rp::uart::InterruptHandler<embassy_rp::peripherals::UART1>;
                });

                let mut cfg = embassy_rp::uart::Config::default();
                cfg.baudrate = Self::SERIAL_PORT1_BAUD_RATE;
                cfg.data_bits = embassy_rp::uart::DataBits::DataBits8;
                cfg.stop_bits = embassy_rp::uart::StopBits::STOP1;
                cfg.parity = embassy_rp::uart::Parity::ParityNone;

                let dev = embassy_rp::uart::Uart::new(p.UART1,
                    p.PIN_4, p.PIN_5,
                    SerialPort2IRQs,
                    p.DMA_CH2,
                    p.DMA_CH3,
                    cfg,
                );
                let (serial_port_2_tx_device, serial_port2_rx_device) =
                    dev.split();

                let serial_port_2_tx = hwa::make_static_async_controller!(
                    "SerialPort2Tx",
                    types::SerialPort2TxMutexStrategy,
                    SerialTxWrapper::new(io::serial_port_2::SerialPort2TxAdapter::new(serial_port_2_tx_device), Self::SERIAL_PORT2_BAUD_RATE),
                );
                let serial_port_2_rx_stream = io::serial_port_2::SerialPort2RxInputStream::new(serial_port2_rx_device);
            }
        }

        //#endregion

        //#region "with-spi"

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-spi")] {
                let spi1 = compile_error!("Provide me")
            }
        }

        //#endregion

        //#region "with-sd-card"

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-sd-card")] {
                let sd_card_block_device = hwa::sd_card_spi::SPIAdapter::new(
                    compile_error!("Provide me")
                );
            }
        }

        //#endregion

        //#region "with-trinamic"

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-trinamic")] {
                let trinamic_uart = compile_error!("Provide me");
            }
        }

        //#endregion

        //#region "with-motion"

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {

                let motion_pins = device::StepActuator {
                    #[cfg(any(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis", feature = "with-e-axis"))]
                    all_enable_pin: embassy_rp::gpio::Output::new(p.PIN_2, embassy_rp::gpio::Level::High),
                    #[cfg(feature = "with-x-axis")]
                    x_endstop_pin: embassy_rp::gpio::Input::new(p.PIN_3, embassy_rp::gpio::Pull::Down),
                    #[cfg(feature = "with-y-axis")]
                    y_endstop_pin: embassy_rp::gpio::Input::new(p.PIN_6, embassy_rp::gpio::Pull::Down),
                    #[cfg(feature = "with-z-axis")]
                    z_endstop_pin: embassy_rp::gpio::Input::new(p.PIN_7, embassy_rp::gpio::Pull::Down),
                    #[cfg(feature = "with-e-axis")]
                    e_endstop_pin: compile_error!("Provide me"),
                    #[cfg(feature = "with-x-axis")]
                    x_step_pin: embassy_rp::gpio::Output::new(p.PIN_8, embassy_rp::gpio::Level::Low),
                    #[cfg(feature = "with-y-axis")]
                    y_step_pin: embassy_rp::gpio::Output::new(p.PIN_9, embassy_rp::gpio::Level::Low),
                    #[cfg(feature = "with-z-axis")]
                    z_step_pin: embassy_rp::gpio::Output::new(p.PIN_10, embassy_rp::gpio::Level::Low),
                    #[cfg(feature = "with-e-axis")]
                    e_step_pin: compile_error!("Provide me"),
                    #[cfg(feature = "with-x-axis")]
                    x_dir_pin: embassy_rp::gpio::Output::new(p.PIN_11, embassy_rp::gpio::Level::Low),
                    #[cfg(feature = "with-y-axis")]
                    y_dir_pin: embassy_rp::gpio::Output::new(p.PIN_12, embassy_rp::gpio::Level::Low),
                    #[cfg(feature = "with-z-axis")]
                    z_dir_pin: embassy_rp::gpio::Output::new(p.PIN_13, embassy_rp::gpio::Level::Low),
                    #[cfg(feature = "with-e-axis")]
                    e_dir_pin: compile_error!("Provide me"),
                };
            }
        }

        //#endregion

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-ps-on")] {
                let ps_on = hwa::make_static_sync_controller!(
                    "PSOn",
                    types::PSOnMutexStrategy,
                    device::PsOnPin::new(p.PIN_14, embassy_rp::gpio::Level::Low)
                );
            }
        }

        hwa::HwiContext {
            sys_watch_dog : hwa::make_static_async_controller!(
                "WatchDogController",
                types::WatchDogMutexStrategy,
                watchdog
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
            #[cfg(feature = "with-i2c")]
            i2c,
            #[cfg(feature = "with-ps-on")]
            ps_on,
            #[cfg(feature = "with-motion")]
            motion_pins,
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
        cortex_m::peripheral::SCB::sys_reset();
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
            
            fn setup_ticker() {
                unsafe {
                    let mut p = cortex_m::Peripherals::steal();
                    let mut syst = p.SYST;
                    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
                    let reload: u32 = ((Contract::PROCESSOR_SYS_CK_MHZ / Contract::STEP_PLANNER_CLOCK_FREQUENCY) - 1).max(1);
                    hwa::info!(
                        "SYST reload set to {} ({} Hz)",
                        reload,
                        Contract::STEP_PLANNER_CLOCK_FREQUENCY
                    );
                    syst.set_reload(reload);
                    syst.clear_current();
                    p.SCB.set_priority(cortex_m::peripheral::scb::SystemHandler::SysTick, 2);
                }
            }
            
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
                    syst.clear_current();
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
                use embassy_executor::Executor;
                use embassy_rp::multicore::{spawn_core1, Stack};
                use printhor_hwa_common::TrackedStaticCell;
                
                #[unsafe(link_section = ".bss")]
                static CORE1_STACK: TrackedStaticCell<Stack<4096>> = TrackedStaticCell::new();
                #[unsafe(link_section = ".bss")]
                static EXECUTOR_HIGH: TrackedStaticCell<Executor> = TrackedStaticCell::new();
                
                struct TokenHolder<S> {
                    token: embassy_executor::SpawnToken<S>,
                }
                
                unsafe impl<S> Sync for TokenHolder<S> {}
                unsafe impl<S> Send for TokenHolder<S> {}
                
                
                // TODO: There must be a better way to tackle this
                let r = Box::new(TokenHolder { token });
                let stack = CORE1_STACK.init::<{ crate::MAX_STATIC_MEMORY }>("executor1::stack", Stack::new());
            
                spawn_core1(core, stack, || {
                    let executor1 =
                        EXECUTOR_HIGH.init::<{ crate::MAX_STATIC_MEMORY }>("executor1", Executor::new());
                    executor1.run(|spawner| unwrap!(spawner.spawn(r.token)))
                });
                Ok(())
                
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        unsafe extern "Rust" {
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
    }
}
