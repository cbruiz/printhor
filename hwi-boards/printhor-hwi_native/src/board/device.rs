#[allow(unused)]
use printhor_hwa_common as hwa;
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {

        // The HWI Device types
        pub type UartPort1Device = crate::board::mocked_peripherals::MockedUart;
        pub type UartPort1TxDevice = crate::board::mocked_peripherals::MockedUartTx;
        pub type UartPort1RxDevice = crate::board::mocked_peripherals::MockedUartRx;
        pub type UartPort1RxInputStream = crate::board::mocked_peripherals::MockedUartRxInputStream;

        // The device type exported to HWA
        pub type SerialPort1TxDevice = UartPort1TxDevice;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type UartPort2Device = crate::board::mocked_peripherals::MockedUartSink;
        pub type UartPort2Tx = crate::board::mocked_peripherals::MockedUartSinkTx;
        pub type UartPort2Rx = crate::board::mocked_peripherals::MockedUartSinkRx;
        pub type UartPort2TxControllerRef = crate::board::ControllerRef<UartPort2Tx>;
        pub type UartPort2RxInputStream = crate::board::mocked_peripherals::MockedUartSinkRxInputStream;
    }
}

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub type AdcImpl<T> = crate::board::mocked_peripherals::MockedAdc<T>;

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
pub type TrinamicUart = crate::board::comm::SingleWireSoftwareUart;

#[cfg(feature = "with-trinamic")]
pub use crate::board::mocked_peripherals::{MockedTrinamicDriver, trinamic_driver_simulator};

#[cfg(feature = "with-trinamic")]
pub use crate::board::comm::AxisChannel;

#[cfg(feature = "with-trinamic")]
pub type TMCUartCh1Pin = crate::board::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh2Pin = crate::board::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh3Pin = crate::board::MockedIOPin;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh4Pin = crate::board::MockedIOPin;

#[cfg(feature = "with-ps-on")]
pub type PsOnPin = crate::board::mocked_peripherals::MockedIOPin;
#[cfg(feature = "with-ps-on")]
pub type PsOnRef = printhor_hwa_common::StandardControllerRef<PsOnPin>;

#[cfg(feature = "with-spi")]
pub type Spi = crate::board::mocked_peripherals::MockedSpi;

#[cfg(feature = "with-spi")]
pub type SpiDeviceRef = printhor_hwa_common::ControllerRef<Spi>;

#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
pub type PwmAny = crate::board::mocked_peripherals::MockedPwm;

#[cfg(feature = "with-fan-layer")]
pub type PwmFanLayer = PwmAny;

#[cfg(feature = "with-fan-extra-1")]
pub type PwmFanExtra1 = PwmAny;

#[cfg(feature = "with-probe")]
pub type PwmServo = PwmAny;

#[cfg(feature = "with-hot-end")]
pub type PwmHotend = PwmAny;

#[cfg(feature = "with-hot-bed")]
pub type PwmHotbed = PwmAny;

#[cfg(feature = "with-laser")]
pub type PwmLaser = PwmAny;

#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
pub use crate::board::mocked_peripherals::PwmChannel;

#[cfg(feature = "with-sdcard")]
pub type SDCardBlockDevice = crate::board::mocked_peripherals::MockledSDCardBlockDevice;

#[cfg(feature = "with-sdcard")]
pub type SDCardBlockDeviceRef = hwa::StaticController<hwa::SyncSendMutex, SDCardBlockDevice>;

#[cfg(feature = "with-hot-end")]
pub type AdcHotendPeripheral = u8;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub type AdcHotendHotbed = AdcImpl<u8>;

#[cfg(feature = "with-hot-end")]
pub type AdcHotendPin = crate::board::mocked_peripherals::MockedIOPin;

#[cfg(feature = "with-hot-bed")]
pub type AdcHotbedPeripheral = u8;

#[cfg(feature = "with-hot-bed")]
pub type AdcHotbedPin = crate::board::mocked_peripherals::MockedIOPin;

pub type Watchdog = crate::board::mocked_peripherals::MockedWatchdog<'static, u8>;

#[cfg(feature = "with-display")]
pub type DisplayDevice = crate::board::mocked_peripherals::SimulatorDisplayDevice;
#[cfg(feature = "with-display")]
pub type DisplayScreen<UI> = crate::board::mocked_peripherals::SimulatorDisplayScreen<UI>;

#[cfg(feature = "with-motion")]
pub struct MotionPins {
    pub x_enable_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub y_enable_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub z_enable_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub e_enable_pin: crate::board::mocked_peripherals::MockedIOPin,

    pub x_endstop_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub y_endstop_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub z_endstop_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub e_endstop_pin: crate::board::mocked_peripherals::MockedIOPin,

    pub x_step_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub y_step_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub z_step_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub e_step_pin: crate::board::mocked_peripherals::MockedIOPin,

    pub x_dir_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub y_dir_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub z_dir_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub e_dir_pin: crate::board::mocked_peripherals::MockedIOPin,
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

    pub fn disable(&mut self, _channels: printhor_hwa_common::StepperChannel)
    {

    }

    pub fn enable(&mut self, _channels: printhor_hwa_common::StepperChannel)
    {

    }

    pub fn set_forward_direction(&mut self, _channels: printhor_hwa_common::StepperChannel)
    {

    }

    pub fn step_toggle(&mut self, _channels: printhor_hwa_common::StepperChannel)
    {

    }


    pub fn step_high(&mut self, _channels: printhor_hwa_common::StepperChannel)
    {

    }

    pub fn step_low(&mut self, _channels: printhor_hwa_common::StepperChannel)
    {

    }


    pub fn endstop_triggered(&mut self, _channels: printhor_hwa_common::StepperChannel) -> bool
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
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmServo>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-hot-end")]
pub struct HotendPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmHotend>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::ControllerRef<AdcHotendHotbed>,
    pub temp_pin: crate::board::mocked_peripherals::MockedIOPin,
    pub thermistor_properties: &'static printhor_hwa_common::ThermistorProperties,
}

#[cfg(feature = "with-hot-bed")]
pub struct HotbedPeripherals {
    pub power_pwm: printhor_hwa_common::InterruptControllerRef<PwmHotbed>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::InterruptControllerRef<AdcHotendHotbed>,
    pub temp_pin: crate::board::mocked_peripherals::MockedIOPin,
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