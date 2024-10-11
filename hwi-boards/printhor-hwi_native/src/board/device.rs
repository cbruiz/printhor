#[allow(unused)]
use printhor_hwa_common as hwa;
#[allow(unused)]
use crate::board as board;
#[allow(unused)]
use crate::device as device;

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
        pub type SerialPort1Rx = board::mocked_peripherals::MockedUartRxInputStream<{Contract::SERIAL_PORT1_RX_BUFFER_SIZE}>;
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

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub type AdcImpl<T> = board::mocked_peripherals::MockedAdc<T>;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub trait AdcTrait {}

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub trait AdcPinTrait<T> {
    fn is_internal(&self) -> bool {
        false
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        pub struct VrefInt;
        impl<T> AdcPinTrait<T> for VrefInt {
            fn is_internal(&self) -> bool {
                true
            }
        }
    }
}

#[cfg(feature = "with-trinamic")]
pub type TrinamicUart = board::comm::SingleWireSoftwareUart;

#[cfg(feature = "with-trinamic")]
pub use board::mocked_peripherals::{MockedTrinamicDriver, trinamic_driver_simulator};

#[cfg(feature = "with-trinamic")]
pub use board::comm::AxisChannel;
use printhor_hwa_common::HwiContract;
use crate::Contract;

#[cfg(feature = "with-trinamic")]
pub type TMCUartCh1Pin = board::mocked_peripherals::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh2Pin = board::mocked_peripherals::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh3Pin = board::mocked_peripherals::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh4Pin = board::mocked_peripherals::MockedIOPin;

#[cfg(feature = "with-ps-on")]
pub type PsOnPin = board::mocked_peripherals::MockedIOPin;

#[cfg(feature = "with-spi")]
pub type Spi = board::mocked_peripherals::MockedSpi;

#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
pub type PwmAny = board::mocked_peripherals::MockedPwm;

#[cfg(feature = "with-fan-layer")]
pub type PwmFanLayer = PwmAny;

#[cfg(feature = "with-fan-extra-1")]
pub type PwmFanExtra1 = PwmAny;

#[cfg(feature = "with-probe")]
pub type PwmProbe = PwmAny;

#[cfg(feature = "with-probe")]
pub type PwmProbeChannel = u8;
#[cfg(feature = "with-laser")]
pub type PwmLaserChannel = u8;
#[cfg(feature = "with-fan-layer")]
pub type PwmFanLayerChannel = u8;
#[cfg(feature = "with-fan-extra-1")]
pub type PwmFanExtra1Channel = u8;

#[cfg(feature = "with-hot-end")]
pub type PwmHotEndChannel = u8;

#[cfg(feature = "with-hot-bed")]
pub type PwmHotBedChannel = u8;

#[cfg(feature = "with-hot-end")]
pub type PwmHotEnd = PwmAny;

#[cfg(feature = "with-hot-bed")]
pub type PwmHotBed = PwmAny;

#[cfg(feature = "with-laser")]
pub type PwmLaser = PwmAny;

#[cfg(feature = "with-sd-card")]
pub type SDCardBlockDevice = board::mocked_peripherals::MockledSDCardBlockDevice;

#[cfg(feature = "with-hot-end")]
pub type AdcHotEndPeripheral = u8;

#[cfg(feature = "with-hot-end")]
pub type Adc1 = AdcImpl<u8>;

#[cfg(feature = "with-hot-end")]
pub type AdcHotEnd = Adc1;
#[cfg(feature = "with-hot-bed")]
pub type AdcHotbed = Adc1;

#[cfg(feature = "with-hot-end")]
pub type AdcHotEndPin = board::mocked_peripherals::MockedIOPin;

#[cfg(feature = "with-hot-bed")]
pub type AdcHotBedPeripheral = u8;

#[cfg(feature = "with-hot-bed")]
pub type AdcHotBedPin = board::mocked_peripherals::MockedIOPin;

#[cfg(feature = "with-display")]
compile_error!("To recover");


#[cfg(feature = "with-sd-card")]
pub struct CardDevice {

    pub card_spi: SDCardBlockDevice,
}

#[cfg(feature = "with-probe")]
pub struct ProbePeripherals {
    pub power_pwm: hwa::StaticAsyncController<crate::PwmProbeMutexStrategyType<device::PwmProbe>>,
    pub power_channel: <device::PwmProbe as embedded_hal_02::Pwm>::Channel,
}

#[cfg(feature = "with-hot-end")]
pub struct HotEndPeripherals {
    pub power_pwm: hwa::StaticAsyncController<crate::PwmHotEndMutexStrategyType<device::PwmHotEnd>>,
    pub power_channel: <device::PwmHotEnd as embedded_hal_02::Pwm>::Channel,
    pub temp_adc: hwa::StaticAsyncController<crate::AdcHotEndMutexStrategyType<device::AdcHotEnd>>,
    pub temp_pin: board::mocked_peripherals::MockedIOPin,
    pub thermistor_properties: &'static hwa::ThermistorProperties,
}

#[cfg(feature = "with-hot-bed")]
pub struct HotBedPeripherals {
    pub power_pwm: hwa::StaticAsyncController<crate::PwmHotBedMutexStrategyType<device::PwmHotBed>>,
    pub power_channel: <device::PwmHotBed as embedded_hal_02::Pwm>::Channel,
    pub temp_adc: hwa::StaticAsyncController<crate::AdcHotBedMutexStrategyType<device::AdcHotbed>>,
    pub temp_pin: board::mocked_peripherals::MockedIOPin,
    pub thermistor_properties: &'static hwa::ThermistorProperties,
}

#[cfg(feature = "with-fan-layer")]
pub struct FanLayerPeripherals {
    pub power_pwm: hwa::StaticAsyncController<crate::PwmLaserMutexStrategyType<device::PwmFanLayer>>,
    pub power_channel: <device::PwmFanLayer as embedded_hal_02::Pwm>::Channel,
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
