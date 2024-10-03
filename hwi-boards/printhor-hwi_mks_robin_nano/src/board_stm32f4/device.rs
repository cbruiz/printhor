#[allow(unused)]
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::wdg;

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type USBDrv = embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>;
        pub use crate::board::io::usbserial::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-1")] {
        pub type UartPort1Device = embassy_stm32::usart::Uart<'static, embassy_stm32::mode::Async>;
        pub type UartPort1RingBufferedRxDevice = embassy_stm32::usart::RingBufferedUartRx<'static>;
        pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>;
        pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>;

        pub type UartPort1TxControllerRef = printhor_hwa_common::StandardControllerRef<printhor_hwa_common::SerialAsyncWrapper<UartPort1TxDevice>>;
        pub use crate::board::io::uart_port1::UartPort1RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-trinamic")] {
        pub type TrinamicUart = crate::board::comm::SingleWireSoftwareUart;
        pub use crate::board::comm::AxisChannel;
        pub type TMCUartCh1Pin = embassy_stm32::peripherals::PD5;
        pub type TMCUartCh2Pin = embassy_stm32::peripherals::PD1;
        pub type TMCUartCh3Pin = embassy_stm32::peripherals::PD4;
        pub type TMCUartCh4Pin = embassy_stm32::peripherals::PD9;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        pub type PsOnPin = Output<'static>;
        pub type PsOnRef = printhor_hwa_common::StandardControllerRef<PsOnPin>;
    }
}

#[cfg(feature = "with-spi")]
pub(crate) type Spi1 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;

#[cfg(feature = "with-spi")]
pub type SpiCardDevice = Spi1;

#[cfg(feature = "with-spi")]
pub type Spi = Spi1;

#[cfg(feature = "with-spi")]
pub type SpiDeviceRef = printhor_hwa_common::InterruptControllerRef<Spi>;

#[cfg(feature = "with-sd-card")]
pub type SpiCardDeviceRef = printhor_hwa_common::InterruptControllerRef<Spi>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub type SpiCardCSPin = Output<'static>;
    }
}

pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
pub use embassy_stm32::adc::AdcChannel as AdcPinTrait;
pub use embassy_stm32::adc::Instance as AdcTrait;
pub use embassy_stm32::adc::VrefInt;
pub type AdcHotendHotbedPeripheral = embassy_stm32::peripherals::ADC1;
pub type AdcHotendHotbed = AdcImpl<AdcHotendHotbedPeripheral>;
pub type AdcHotendPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotbedPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotend = AdcHotendHotbed;
pub type AdcHotbed = AdcHotendHotbed;
pub type AdcHotendPin = embassy_stm32::peripherals::PC1;
pub type AdcHotbedPin = embassy_stm32::peripherals::PC0;

pub use embassy_stm32::timer::GeneralInstance4Channel as PwmTrait;

pub type PwmImpl<TimPeri> = embassy_stm32::timer::simple_pwm::SimplePwm<'static, TimPeri>;

pub type PwmServo = SimplePwm<'static, embassy_stm32::peripherals::TIM1>;
pub type PwmFanLayer = SimplePwm<'static, embassy_stm32::peripherals::TIM3>;
pub type PwmHotend = SimplePwm<'static, embassy_stm32::peripherals::TIM9>;
pub type PwmHotbed = SimplePwm<'static, embassy_stm32::peripherals::TIM5>;
pub type PwmLaser = SimplePwm<'static, embassy_stm32::peripherals::TIM13>;

pub type PwmChannel = embassy_stm32::timer::Channel;

pub type Watchdog = wdg::IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>;

#[cfg(feature = "with-probe")]
pub struct ProbePeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmServo>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-hot-end")]
pub struct HotendPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmHotend>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::InterruptControllerRef<AdcHotend>,
    pub temp_pin: AdcHotendPin,
    pub thermistor_properties: &'static printhor_hwa_common::ThermistorProperties,
}

#[cfg(feature = "with-hot-bed")]
pub struct HotbedPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmHotbed>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::InterruptControllerRef<AdcHotbed>,
    pub temp_pin: AdcHotbedPin,
    pub thermistor_properties: &'static printhor_hwa_common::ThermistorProperties,
}

#[cfg(feature = "with-fan-layer")]
pub struct FanLayerPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmFanLayer>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmLaser>,
    pub power_channel: PwmChannel,
}

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

#[cfg(feature = "with-sd-card")]
pub struct CardDevice {
    pub card_spi: SpiCardDevice,
    pub card_cs: SpiCardCSPin,
}
