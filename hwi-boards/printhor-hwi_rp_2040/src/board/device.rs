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
    if #[cfg(feature = "with-motion")] {

        pub struct MotionPins {
            pub x_enable_pin: Output<'static>,
            pub y_enable_pin: Output<'static>,
            pub z_enable_pin: Output<'static>,
            pub e_enable_pin: Output<'static>,

            pub x_endstop_pin: Input<'static>,
            pub y_endstop_pin: Input<'static>,
            pub z_endstop_pin: Input<'static>,
            pub e_endstop_pin: Input<'static>,

            pub x_step_pin: Output<'static>,
            pub y_step_pin: Output<'static>,
            pub z_step_pin: Output<'static>,
            pub e_step_pin: Output<'static>,

            pub x_dir_pin: Output<'static>,
            pub y_dir_pin: Output<'static>,
            pub z_dir_pin: Output<'static>,
            pub e_dir_pin: Output<'static>,
        }

        impl MotionPins {

            pub fn disable(&mut self, _channels: printhor_hwa_common::StepperChannel)
            {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::X) {
                    self.x_enable_pin.set_high();
                }
                else {
                    self.x_enable_pin.set_low();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Y) {
                    self.y_enable_pin.set_high();
                }
                else {
                    self.y_enable_pin.set_low();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Z) {
                    self.z_enable_pin.set_high();
                }
                else {
                    self.z_enable_pin.set_low();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::E) {
                    self.e_enable_pin.set_high();
                }
                else {
                    self.e_enable_pin.set_low();
                }
            }

            pub fn enable(&mut self, _channels: printhor_hwa_common::StepperChannel)
            {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::X) {
                    self.x_enable_pin.set_low();
                }
                else {
                    self.x_enable_pin.set_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Y) {
                    self.y_enable_pin.set_low();
                }
                else {
                    self.y_enable_pin.set_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Z) {
                    self.z_enable_pin.set_low();
                }
                else {
                    self.z_enable_pin.set_high();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::E) {
                    self.e_enable_pin.set_low();
                }
                else {
                    self.e_enable_pin.set_high();
                }
            }

            pub fn set_forward_direction(&mut self, _channels: printhor_hwa_common::StepperChannel)
            {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::X) {
                    self.x_dir_pin.set_high();
                }
                else {
                    self.x_dir_pin.set_low();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Y) {
                    self.y_dir_pin.set_high();
                }
                else {
                    self.y_dir_pin.set_low();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Z) {
                    self.z_dir_pin.set_high();
                }
                else {
                    self.z_dir_pin.set_low();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::E) {
                    self.e_dir_pin.set_high();
                }
                else {
                    self.e_dir_pin.set_low();
                }
            }

            pub fn step_toggle(&mut self, _channels: printhor_hwa_common::StepperChannel)
            {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::X) {
                    self.x_step_pin.toggle();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Y) {
                    self.y_step_pin.toggle();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Z) {
                    self.z_step_pin.toggle();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::E) {
                    self.e_step_pin.toggle();
                }
            }

            pub fn step_high(&mut self, _channels: printhor_hwa_common::StepperChannel)
            {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::X) {
                    self.x_step_pin.set_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Y) {
                    self.y_step_pin.set_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Z) {
                    self.z_step_pin.set_high();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::E) {
                    self.e_step_pin.set_high();
                }
            }

            pub fn step_low(&mut self, _channels: printhor_hwa_common::StepperChannel)
            {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::X) {
                    self.x_step_pin.set_low();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Y) {
                    self.y_step_pin.set_low();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Z) {
                    self.z_step_pin.set_low();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::E) {
                    self.e_step_pin.set_low();
                }
            }

            pub fn endstop_triggered(&mut self, _channels: printhor_hwa_common::StepperChannel) -> bool
            {
                #[allow(unused_mut)]
                let mut triggered = false;
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::X) {
                    triggered |= self.x_endstop_pin.is_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Y) {
                    triggered |= self.y_endstop_pin.is_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(printhor_hwa_common::StepperChannel::Z) {
                    triggered |= self.z_endstop_pin.is_high();
                }
                triggered
            }
        }

        pub struct MotionDevice {
            #[cfg(feature = "with-trinamic")]
            pub trinamic_uart: TrinamicUart,

            pub motion_pins: MotionPins,
        }
    }
}
