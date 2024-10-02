#[allow(unused)]
use printhor_hwa_common as hwa;
#[allow(unused)]
use crate::board as board;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-usb")] {
        pub type SerialUSBDevice = ();
        pub type SerialUSBTxDevice = ();
        pub type SerialUSBRxDevice = ();
        pub type SerialUSBRxInputStream = ();

        // The device type exported to HWA
        pub type SerialUSBTxDevice = ();
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {

        // The HWI Device types
        pub type UartPort1Device = board::mocked_peripherals::MockedUart;
        pub type UartPort1TxDevice = board::mocked_peripherals::MockedUartTx;
        pub type UartPort1RxDevice = board::mocked_peripherals::MockedUartRx;
        pub type UartPort1RxInputStream = board::mocked_peripherals::MockedUartRxInputStream;

        // The device type exported to HWA
        pub type SerialPort1TxDevice = UartPort1TxDevice;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type UartPort2Device = board::mocked_peripherals::MockedUartSink;
        pub type UartPort2TxDevice = board::mocked_peripherals::MockedUartSinkTx;
        pub type UartPort2RxDevice = board::mocked_peripherals::MockedUartSinkRx;
        pub type UartPort2RxInputStream = board::mocked_peripherals::MockedUartSinkRxInputStream;

        pub type SerialPort2TxDevice = UartPort2TxDevice;
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
pub type PwmServo = PwmAny;

#[cfg(feature = "with-hot-end")]
pub type PwmHotEnd = PwmAny;

#[cfg(feature = "with-hot-bed")]
pub type PwmHotBed = PwmAny;

#[cfg(feature = "with-laser")]
pub type PwmLaser = PwmAny;

#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
pub use board::mocked_peripherals::PwmChannel;

#[cfg(feature = "with-sdcard")]
pub type SDCardBlockDevice = board::mocked_peripherals::MockledSDCardBlockDevice;

#[cfg(feature = "with-sdcard")]
pub type SDCardBlockDeviceRef = hwa::StaticController<hwa::SyncSendMutex, SDCardBlockDevice>;

#[cfg(feature = "with-hot-end")]
pub type AdcHotEndPeripheral = u8;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub type AdcHotendHotbed = AdcImpl<u8>;

#[cfg(feature = "with-hot-end")]
pub type AdcHotEndPin = board::mocked_peripherals::MockedIOPin;

#[cfg(feature = "with-hot-bed")]
pub type AdcHotBedPeripheral = u8;

#[cfg(feature = "with-hot-bed")]
pub type AdcHotBedPin = board::mocked_peripherals::MockedIOPin;

pub type Watchdog = board::mocked_peripherals::MockedWatchdog<'static, u8>;

#[cfg(feature = "with-display")]
compile_error!("To recover");

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


#[cfg(feature = "with-motion")]
pub struct MotionDevice {

    #[cfg(feature = "with-trinamic")]
    pub trinamic_uart: TrinamicUart,

    pub motion_pins: MotionPins,
}


#[cfg(feature = "with-sdcard")]
pub struct CardDevice {

    pub card_spi: SDCardBlockDevice,
}

#[cfg(feature = "with-probe")]
pub struct ProbePeripherals {
    pub power_pwm: hwa::StaticController<crate::ProbeMutexType, PwmServo>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-hot-end")]
pub struct HotendPeripherals {
    pub power_pwm: hwa::StaticController<crate::PwmHotEndMutexType, PwmHotEnd>,
    pub power_channel: PwmChannel,
    pub temp_adc: hwa::StaticController<crate::AdcHotEndMutexType, AdcHotendHotbed>,
    pub temp_pin: board::mocked_peripherals::MockedIOPin,
    pub thermistor_properties: &'static hwa::ThermistorProperties,
}

#[cfg(feature = "with-hot-bed")]
pub struct HotbedPeripherals {
    pub power_pwm: hwa::StaticController<crate::PwmHotBedMutexType, PwmHotBed>,
    pub power_channel: PwmChannel,
    pub temp_adc: hwa::StaticController<crate::AdcHotBedMutexType, AdcHotendHotbed>,
    pub temp_pin: board::mocked_peripherals::MockedIOPin,
    pub thermistor_properties: &'static hwa::ThermistorProperties,
}

#[cfg(feature = "with-fan-layer")]
pub struct FanLayerPeripherals {
    pub power_pwm: hwa::StaticController<crate::PwmLaserMutexType, PwmFanLayer>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-fan-extra-1")]
pub struct FanExtra1Peripherals {
    pub power_pwm: hwa::StaticController<crate::PwmFanExtra1MutexType, PwmFanExtra1>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: hwa::StaticController<crate::PwmLaserMutexType, PwmLaser>,
    pub power_channel: PwmChannel,
}
