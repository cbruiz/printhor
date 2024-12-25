use printhor_hwa_common as hwa;

pub type WatchDog = embassy_stm32::wdg::IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>;

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-1")] {

        // The HWI Device types
        pub type UartPort1Device = embassy_stm32::usart::Uart<'static, embassy_stm32::mode::Async>;
        pub type UartPort1RingBufferedRxDevice = embassy_stm32::usart::RingBufferedUartRx<'static>;
        pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>;
        pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>;

        // The device type exported to HWA
        pub type SerialPort1TxDevice = hwa::SerialAsyncWrapper<UartPort1TxDevice>;
        pub use crate::board::io::uart_port1::UartPort1RxInputStream;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type USBDrv = embassy_stm32::usb_otg::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>;
        pub use crate::board::io::usbserial::*;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-trinamic")] {
        type UartTrinamicPeri = embassy_stm32::peripherals::USART4;
        type UartTrinamicTxDma = embassy_stm32::peripherals::DMA1_CH7;
        type UartTrinamicRxDma = embassy_stm32::peripherals::DMA1_CH6;
        pub type UartTrinamic = crate::board::usart::Uart<'static,UartTrinamicPeri, UartTrinamicTxDma, UartTrinamicRxDma>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-ps-on")] {
        pub type PsOnPin = Output<'static>;

        pub type PsOnRef = printhor_hwa_common::ControllerRef<ControllerMutexType, PsOnPin>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-spi")] {
        pub type Spi1 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
    }
}

#[cfg(feature = "with-spi")]
pub type SpiCardDevice = Spi1;

#[cfg(feature = "with-spi")]
pub type Spi = Spi1;

#[cfg(feature = "with-spi")]
pub type SpiDeviceRef = printhor_hwa_common::InterruptControllerRef<Spi>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub type SpiCardDeviceRef = printhor_hwa_common::InterruptControllerRef<Spi>;

        pub type SpiCardCSPin = Output<'static>;
    }
}

pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
pub use embassy_stm32::adc::Instance as AdcTrait;

pub use embassy_stm32::adc::AdcChannel as AdcPinTrait;
pub use embassy_stm32::adc::VrefInt;

pub type AdcHotendHotbedPeripheral = embassy_stm32::peripherals::ADC1;
pub type AdcHotendHotbed = AdcImpl<AdcHotendHotbedPeripheral>;
pub type AdcHotendPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotbedPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotend = AdcHotendHotbed;
pub type AdcHotbed = AdcHotendHotbed;
pub type AdcHotendPin = embassy_stm32::peripherals::PB0;
pub type AdcHotbedPin = embassy_stm32::peripherals::PB1;

pub type PwmServo = SimplePwm<'static, embassy_stm32::peripherals::TIM11>;

pub type PwmHotendHotbedLayer = SimplePwm<'static, embassy_stm32::peripherals::TIM5>;

pub type PwmFanLayer = PwmHotendHotbedLayer;

pub type PwmHotend = PwmHotendHotbedLayer;

pub type PwmHotbed = PwmHotendHotbedLayer;

pub type PwmLaser = SimplePwm<'static, embassy_stm32::peripherals::TIM1>;

pub type PwmChannel = embassy_stm32::timer::Channel;

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

#[cfg(feature = "with-fan-extra-1")]
pub struct FanExtra1Peripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmFanExtra1>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmLaser>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-fan-layer")]
pub struct LayerFanPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmFanLayer>,
    pub power_channel: PwmChannel,
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-motion")] {
        pub struct MotionPins {
            pub all_enable_pin: Output<'static>, // D8

            pub x_endstop_pin: Input<'static>, // D9
            pub y_endstop_pin: Input<'static>, // D10
            pub z_endstop_pin: Input<'static>, // D11

            pub x_step_pin: Output<'static>, // D2
            pub y_step_pin: Output<'static>, // D3
            pub z_step_pin: Output<'static>, // D4

            pub x_dir_pin: Output<'static>, // D5
            pub y_dir_pin: Output<'static>, // D6
            pub z_dir_pin: Output<'static>, // D7
        }
    }

}

#[cfg(feature = "with-motion")]
impl MotionPins {
    pub fn disable(&mut self, _channels: printhor_hwa_common::StepperChannel) {
        self.all_enable_pin.set_high();
    }

    pub fn enable(&mut self, _channels: printhor_hwa_common::StepperChannel) {
        self.all_enable_pin.set_low();
    }

    pub fn set_forward_direction(&mut self, _channels: printhor_hwa_common::StepperChannel) {
        cfg_if::cfg_if! {
            if #[cfg(not(feature="debug-signals"))] {
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
        }
    }

    pub fn step_toggle(&mut self, _channels: printhor_hwa_common::StepperChannel) {
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

    pub fn step_high(&mut self, _channels: printhor_hwa_common::StepperChannel) {
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

    pub fn step_low(&mut self, _channels: printhor_hwa_common::StepperChannel) {
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

    pub fn endstop_triggered(&mut self, _channels: printhor_hwa_common::StepperChannel) -> bool {
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

#[cfg(feature = "with-motion")]
pub struct MotionDevice {
    #[cfg(feature = "with-trinamic")]
    pub trinamic_uart: UartTrinamic,

    pub motion_pins: MotionPins,
}

#[cfg(feature = "with-sd-card")]
pub struct CardDevice {
    pub card_spi: SpiCardDevice,
    pub card_cs: SpiCardCSPin,
}



/*

#[allow(unused)]
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Output},
    timer::simple_pwm::SimplePwm,
    wdg,
};

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
        /*
        // For UARTBuffered
        pub type UartPort1Device = embassy_stm32::usart::BufferedUart<'static>;
        pub type UartPort1TxDevice = embassy_stm32::usart::BufferedUartTx<'static>;
        pub type UartPort1RxDevice = embassy_stm32::usart::BufferedUartRx<'static>;
         */

        pub type UartPort1TxControllerRef = printhor_hwa_common::StandardControllerRef<printhor_hwa_common::SerialAsyncWrapper<UartPort1TxDevice>>;
        pub use crate::board::io::uart_port1::UartPort1RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-trinamic")] {
        type UartTrinamicPeri = embassy_stm32::peripherals::USART4;
        type UartTrinamicTxDma = embassy_stm32::peripherals::DMA1_CH7;
        type UartTrinamicRxDma = embassy_stm32::peripherals::DMA1_CH6;
        pub type UartTrinamic = crate::board::usart::Uart<'static,UartTrinamicPeri, UartTrinamicTxDma, UartTrinamicRxDma>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-ps-on")] {
        pub type PsOnPin = Output<'static>;
        pub type PsOnRef = printhor_hwa_common::StandardControllerRef<PsOnPin>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-spi")] {
        pub type Spi1 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
    }
}

#[cfg(feature = "with-sd-card")]
pub type SpiCardDevice = Spi1;

#[cfg(feature = "with-spi")]
pub type Spi = Spi1;

#[cfg(feature = "with-spi")]
pub type SpiDeviceRef = printhor_hwa_common::InterruptControllerRef<Spi>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub type SpiCardDeviceRef = printhor_hwa_common::InterruptControllerRef<SpiCardDevice>;

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
pub type AdcHotendPin = embassy_stm32::peripherals::PC2;
pub type AdcHotbedPin = embassy_stm32::peripherals::PC3;

pub type PwmServo = SimplePwm<'static, embassy_stm32::peripherals::TIM3>;

pub type PwmHotendHotbed = SimplePwm<'static, embassy_stm32::peripherals::TIM15>;

pub type PwmFanLayer = SimplePwm<'static, embassy_stm32::peripherals::TIM2>;

pub type PwmHotend = PwmHotendHotbed;

pub type PwmHotbed = PwmHotendHotbed;

pub type PwmLaser = SimplePwm<'static, embassy_stm32::peripherals::TIM8>;

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

#[cfg(feature = "with-fan-extra-1")]
pub struct FanExtra1Peripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmFanExtra1>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmLaser>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-fan-layer")]
pub struct LayerFanPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmFanLayer>,
    pub power_channel: PwmChannel,
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-motion")] {
        pub struct MotionPins {
            pub all_enable_pin: Output<'static>, // D8

            pub x_endstop_pin: Input<'static>, // D9
            pub y_endstop_pin: Input<'static>, // D10
            pub z_endstop_pin: Input<'static>, // D11

            pub x_step_pin: Output<'static>, // D2
            pub y_step_pin: Output<'static>, // D3
            pub z_step_pin: Output<'static>, // D4

            pub x_dir_pin: Output<'static>, // D5
            pub y_dir_pin: Output<'static>, // D6
            pub z_dir_pin: Output<'static>, // D7
        }
    }
}

#[cfg(feature = "with-motion")]
impl MotionPins {
    pub fn disable(&mut self, _channels: printhor_hwa_common::StepperChannel) {
        self.all_enable_pin.set_high();
    }

    pub fn enable(&mut self, _channels: printhor_hwa_common::StepperChannel) {
        self.all_enable_pin.set_low();
    }

    pub fn set_forward_direction(&mut self, _channels: printhor_hwa_common::StepperChannel) {
        cfg_if::cfg_if! {
            if #[cfg(not(feature="debug-signals"))] {
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
        }
    }

    pub fn step_toggle(&mut self, _channels: printhor_hwa_common::StepperChannel) {
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

    pub fn step_high(&mut self, _channels: printhor_hwa_common::StepperChannel) {
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

    pub fn step_low(&mut self, _channels: printhor_hwa_common::StepperChannel) {
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

    pub fn endstop_triggered(&mut self, _channels: printhor_hwa_common::StepperChannel) -> bool {
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

#[cfg(feature = "with-motion")]
pub struct MotionDevice {
    #[cfg(feature = "with-trinamic")]
    pub trinamic_uart: UartTrinamic,

    pub motion_pins: MotionPins,
}

#[cfg(feature = "with-sd-card")]
pub struct CardDevice {
    pub card_spi: SpiCardDevice,
    pub card_cs: SpiCardCSPin,
}
*/