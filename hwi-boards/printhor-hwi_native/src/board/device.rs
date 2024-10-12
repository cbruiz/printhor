#[allow(unused)]
use printhor_hwa_common as hwa;
#[allow(unused)]
use crate::board as board;
#[allow(unused)]
use hwa::HwiContract;


pub type WatchDog = board::mocked_peripherals::MockedWatchdog<'static, u8>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        // The HWI Device types
        pub type SerialUsbDevice = board::mocked_peripherals::MockedUartUnixSocket;
        pub type SerialUsbTx = board::mocked_peripherals::MockedUartNamedPipeTx;
        pub type SerialUsbRx = board::mocked_peripherals::MockedUartNamedPipeRxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        // The HWI Device types
        pub type SerialPort1Device = board::mocked_peripherals::MockedUart;
        pub type SerialPort1Rx = board::mocked_peripherals::MockedUartRxInputStream<{crate::Contract::SERIAL_PORT1_RX_BUFFER_SIZE}>;
        pub type SerialPort1Tx = board::mocked_peripherals::MockedUartTx;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type SerialPort2Device = board::mocked_peripherals::MockedUartSink;
        pub type SerialPort2Rx = board::mocked_peripherals::MockedUartSinkRxInputStream;
        pub type SerialPort2Tx = board::mocked_peripherals::MockedUartSinkTx;

    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        pub type PsOnPin = board::mocked_peripherals::MockedIOPin;
    }
}

#[cfg(feature = "with-motion")]
pub struct MotionPins {
    pub x_enable_pin: board::mocked_peripherals::MockedIOPin,
    pub y_enable_pin: board::mocked_peripherals::MockedIOPin,
    pub z_enable_pin: board::mocked_peripherals::MockedIOPin,
    pub e_enable_pin: board::mocked_peripherals::MockedIOPin,

    pub x_endstop_pin: board::mocked_peripherals::MockedIOPin,
    pub y_endstop_pin: board::mocked_peripherals::MockedIOPin,
    pub z_endstop_pin: board::mocked_peripherals::MockedIOPin,
    pub e_endstop_pin: board::mocked_peripherals::MockedIOPin,

    pub x_step_pin: board::mocked_peripherals::MockedIOPin,
    pub y_step_pin: board::mocked_peripherals::MockedIOPin,
    pub z_step_pin: board::mocked_peripherals::MockedIOPin,
    pub e_step_pin: board::mocked_peripherals::MockedIOPin,

    pub x_dir_pin: board::mocked_peripherals::MockedIOPin,
    pub y_dir_pin: board::mocked_peripherals::MockedIOPin,
    pub z_dir_pin: board::mocked_peripherals::MockedIOPin,
    pub e_dir_pin: board::mocked_peripherals::MockedIOPin,
}

#[cfg(feature = "with-motion")]
impl MotionPins {
    #[inline]
    pub fn enable_x_stepper(&mut self) {
        self.x_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_y_stepper(&mut self) {
        self.y_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_z_stepper(&mut self) {
        self.z_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_e_stepper(&mut self) { self.e_enable_pin.set_low(); }
    #[inline]
    pub fn disable_x_stepper(&mut self) {
        self.x_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_y_stepper(&mut self) {
        self.y_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_z_stepper(&mut self) {
        self.z_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_e_stepper(&mut self) { self.e_enable_pin.set_high(); }
    #[inline]
    pub fn disable_all_steppers(&mut self) {
        self.x_enable_pin.set_high();
        self.y_enable_pin.set_high();
        self.z_enable_pin.set_high();
        self.e_enable_pin.set_high();
    }
    #[inline]
    pub fn enable_all_steppers(&mut self) {
        self.x_enable_pin.set_low();
        self.y_enable_pin.set_low();
        self.z_enable_pin.set_low();
        self.e_enable_pin.set_low();
    }

    pub fn disable(&mut self, _channels: hwa::StepperChannel)
    {

    }

    pub fn enable(&mut self, _channels: hwa::StepperChannel)
    {

    }

    pub fn set_forward_direction(&mut self, _channels: hwa::StepperChannel)
    {

    }

    pub fn step_toggle(&mut self, _channels: hwa::StepperChannel)
    {

    }


    pub fn step_high(&mut self, _channels: hwa::StepperChannel)
    {

    }

    pub fn step_low(&mut self, _channels: hwa::StepperChannel)
    {

    }


    pub fn endstop_triggered(&mut self, _channels: hwa::StepperChannel) -> bool
    {
        false
    }

}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-probe", feature = "with-hot-end",feature = "with-hot-bed",
        feature = "with-laser", feature = "with-fan-layer", feature = "with-fan-extra-1"))] {

        pub type Pwm1 = board::mocked_peripherals::MockedPwm;
        pub type Pwm2 = board::mocked_peripherals::MockedPwm;

    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end",feature = "with-hot-bed"))] {

        pub type Adc1 = board::mocked_peripherals::MockedAdc<u8>;
        pub type Adc2 = board::mocked_peripherals::MockedAdc<u8>;

    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        pub type ProbePwm = Pwm1;
        pub type ProbePwmChannel = <ProbePwm as hwa::traits::Pwm>::Channel;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        pub type HotEndAdc = Adc1;
        pub type HotEndAdcPin = board::mocked_peripherals::MockedIOPin;
        pub type HotEndPwm = Pwm1;
        pub type HotEndPwmChannel = <HotEndPwm as hwa::traits::Pwm>::Channel;

    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-bed")] {
        pub type HotBedAdc = Adc1;
        pub type HotBedAdcPin = board::mocked_peripherals::MockedIOPin;
        pub type HotBedPwm = Pwm1;
        pub type HotBedPwmChannel = <HotBedPwm as hwa::traits::Pwm>::Channel;

    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-layer")] {
        pub type FanLayerPwm = Pwm1;
        pub type FanLayerPwmChannel = <FanLayerPwm as hwa::traits::Pwm>::Channel;

    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub type SDCardBlockDevice = board::mocked_peripherals::MockedSDCardBlockDevice;

    }
}

#[cfg(feature = "with-trinamic")]
pub type TrinamicUart = board::comm::SingleWireSoftwareUart;

#[cfg(feature = "with-trinamic")]
pub use board::mocked_peripherals::{MockedTrinamicDriver, trinamic_driver_simulator};

#[cfg(feature = "with-trinamic")]
pub use board::comm::AxisChannel;


#[cfg(feature = "with-trinamic")]
pub type TMCUartCh1Pin = board::mocked_peripherals::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh2Pin = board::mocked_peripherals::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh3Pin = board::mocked_peripherals::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh4Pin = board::mocked_peripherals::MockedIOPin;



#[cfg(feature = "with-spi")]
pub type Spi = board::mocked_peripherals::MockedSpi;



#[cfg(feature = "with-display")]
compile_error!("To recover");


#[cfg(feature = "with-sd-card")]
pub struct CardDevice {

    pub card_spi: SDCardBlockDevice,
}

#[cfg(feature = "with-fan-extra-1")]
pub struct FanExtra1Peripherals {
    pub power_pwm: hwa::StaticAsyncController<crate::PwmFanExtra1MutexStrategyType<device::PwmFanExtra1>>,
    pub power_channel: <device::PwmFanExtra1 as embedded_hal_02::Pwm>::Channel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: hwa::StaticAsyncController<crate::PwmLaserMutexStrategyType<device::PwmLaser>>,
    pub power_channel: <device::PwmLaser as embedded_hal_02::Pwm>::Channel,
}
