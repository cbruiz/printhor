cfg_if::cfg_if! {
    if #[cfg(feature="tst-rp2040")] {

        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-port-1")] {
                type UartPort1Peri = embassy_rp::peripherals::UART0;
                pub type UartPort1Device = embassy_rp::uart::Uart<'static, UartPort1Peri, embassy_rp::uart::Blocking>;
                pub type UartPort1TxDevice = embassy_rp::uart::BufferedUartTx<'static, UartPort1Peri>;
                pub type UartPort1RxDevice = embassy_rp::uart::BufferedUartRx<'static, UartPort1Peri>;
                pub type UartPort1RxBufferedDevice = embassy_rp::uart::BufferedUartRx<'static, UartPort1Peri>;

                pub type UartPort1TxControllerRef = crate::board::ControllerRef<UartPort1TxDevice>;
                pub use crate::board::io::uart_port1::UartPort1RxInputStream;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-port-2")] {
                type UartPort2Peri = embassy_rp::peripherals::UART1;
                pub type UartPort2Device = embassy_rp::uart::Uart<'static, UartPort2Peri, embassy_rp::uart::Blocking>;
                pub type UartPort2TxDevice = embassy_rp::uart::BufferedUartTx<'static, UartPort2Peri>;
                pub type UartPort2RxDevice = embassy_rp::uart::BufferedUartRx<'static, UartPort2Peri>;
                pub type UartPort2RxBufferedDevice = embassy_rp::uart::BufferedUartRx<'static, UartPort2Peri>;

                pub type UartPort2TxControllerRef = crate::board::ControllerRef<UartPort2TxDevice>;
                pub use crate::board::io::uart_port2::UartPort2RxInputStream;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-usb")] {
                pub type USBDrv = embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>;
                pub use crate::board::io::usbserial::*;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-trinamic")] {
                compile_error("Not yet implemented")
            }
        }

        pub type TaskStepperCore = embassy_rp::peripherals::CORE1;

        cfg_if::cfg_if! {
            if #[cfg(feature="with-ps-on")] {
                pub type PsOnPin = embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_14>;
                pub type PsOnRef = printhor_hwa_common::ControllerRef<PsOnPin>;
            }
        }

    }
}

/// Wraps the gap between stm32 and rp8020 Watchdog API specs
pub struct WatchdogAdapter {
    inner: embassy_rp::watchdog::Watchdog,
}

impl WatchdogAdapter {
    pub fn new(peri: embassy_rp::peripherals::WATCHDOG) -> Self {
        Self {
            inner: embassy_rp::watchdog::Watchdog::new(peri)
        }
    }
    #[inline(always)]
    pub fn unleash(&mut self) {
        self.inner.start(embassy_time::Duration::from_millis(crate::board::WATCHDOG_TIMEOUT_MS as u64))
    }
    #[inline(always)]
    pub fn pet(&mut self) {
        self.inner.feed();
    }
}

pub type Watchdog = WatchdogAdapter;

cfg_if::cfg_if! {
    if #[cfg(feature="upstream-embassy")] {
        cfg_if::cfg_if! {
            if #[cfg(feature="with-motion")] {
                pub struct MotionPins {
                    pub all_enable_pin: Output<'static>,

                    pub x_endstop_pin: Input<'static>,
                    pub y_endstop_pin: Input<'static>,
                    pub z_endstop_pin: Input<'static>,

                    pub x_step_pin: Output<'static>,
                    pub y_step_pin: Output<'static>,
                    pub z_step_pin: Output<'static>,

                    pub x_dir_pin: Output<'static>,
                    pub y_dir_pin: Output<'static>,
                    pub z_dir_pin: Output<'static>,
                }
            }
        }
    }
    else {
        cfg_if::cfg_if! {
            if #[cfg(feature="with-motion")] {
                // TODO: Arbitrarily chosen!!
                pub struct MotionPins {
                    pub all_enable_pin: embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_2>,

                    pub x_endstop_pin: embassy_rp::gpio::Input<'static, embassy_rp::peripherals::PIN_3>,
                    pub y_endstop_pin: embassy_rp::gpio::Input<'static, embassy_rp::peripherals::PIN_6>,
                    pub z_endstop_pin: embassy_rp::gpio::Input<'static, embassy_rp::peripherals::PIN_7>,

                    pub x_step_pin: embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_8>,
                    pub y_step_pin: embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_9>,
                    pub z_step_pin: embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_10>,

                    pub x_dir_pin: embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_11>,
                    pub y_dir_pin: embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_12>,
                    pub z_dir_pin: embassy_rp::gpio::Output<'static, embassy_rp::peripherals::PIN_13>,
                }
            }
        }
    }
}

#[cfg(feature = "with-motion")]
impl MotionPins {
    #[inline]
    pub fn enable_x_stepper(&mut self) {
        self.all_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_y_stepper(&mut self) {
        self.all_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_z_stepper(&mut self) {
        self.all_enable_pin.set_low();
    }
    #[inline]
    pub fn disable_x_stepper(&mut self) {
        self.all_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_y_stepper(&mut self) {
        self.all_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_z_stepper(&mut self) {
        self.all_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_e_stepper(&mut self) { self.all_enable_pin.set_high(); }
    #[inline]
    pub fn disable_all_steppers(&mut self) {
        self.all_enable_pin.set_high();
    }
    #[inline]
    pub fn enable_all_steppers(&mut self) { self.all_enable_pin.set_low(); }
}

#[cfg(feature = "with-motion")]
pub struct MotionDevice {

    #[cfg(feature = "with-trinamic")]
    pub trinamic_uart: UartTrinamic,

    pub motion_pins: MotionPins,
}
