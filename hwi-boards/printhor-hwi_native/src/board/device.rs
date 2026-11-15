#[allow(unused)]
use printhor_hwa_common as hwa;
#[allow(unused)]
use crate::board as board;
#[allow(unused)]
use hwa::HwiContract;
#[allow(unused)]
use crate::simulation;

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
    if #[cfg(feature = "with-spi")] {
        pub type Spi = board::mocked_peripherals::MockedSpi;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-i2c")] {
        pub type I2c = board::mocked_peripherals::MockedI2c;
    }
}


cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        pub type PsOnPin = board::mocked_peripherals::MockedIOPin;
    }
}

#[cfg(feature = "with-motion")]
pub struct StepActuator {
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
impl hwa::traits::StepActuatorTrait for StepActuator {
    fn set_enabled(&mut self, _channels: hwa::CoordSel, _enabled: bool) {
        #[cfg(feature = "with-x-axis")]
        if _channels.contains(hwa::CoordSel::X) {
            if _enabled {
                self.x_enable_pin.set_low();
            }
            else {
                self.x_enable_pin.set_high();
            }
        }
        #[cfg(feature = "with-y-axis")]
        if _channels.contains(hwa::CoordSel::Y) {
            if _enabled {
                self.y_enable_pin.set_low();
            }
            else {
                self.y_enable_pin.set_high();
            }
        }
        #[cfg(feature = "with-z-axis")]
        if _channels.contains(hwa::CoordSel::Z) {
            if _enabled {
                self.z_enable_pin.set_low();
            }
            else {
                self.z_enable_pin.set_high();
            }
        }
        #[cfg(feature = "with-e-axis")]
        if _channels.contains(hwa::CoordSel::E) {
            if _enabled {
                self.e_enable_pin.set_low();
            }
            else {
                self.e_enable_pin.set_high();
            }
        }
    }

    fn set_forward_direction(&mut self, _channels: hwa::CoordSel, _mask: hwa::CoordSel) {
        #[cfg(feature = "with-x-axis")]
        if _mask.contains(hwa::CoordSel::X) {
            if _channels.contains(hwa::CoordSel::X) {
                self.x_dir_pin.set_high();
            }
            else {
                self.x_dir_pin.set_low();
            }
            simulation::set_direction(simulation::Axis::X, self.x_dir_pin.is_high());
        }
        #[cfg(feature = "with-y-axis")]
        if _mask.contains(hwa::CoordSel::Y) {
            if _channels.contains(hwa::CoordSel::Y) {
                self.y_dir_pin.set_high();
            }
            else {
                self.y_dir_pin.set_low();
            }
            simulation::set_direction(simulation::Axis::Y, self.y_dir_pin.is_high());
        }
        #[cfg(feature = "with-z-axis")]
        if _mask.contains(hwa::CoordSel::Z) {
            if _channels.contains(hwa::CoordSel::Z) {
                self.z_dir_pin.set_high();
            }
            else {
                self.z_dir_pin.set_low();
            }
            simulation::set_direction(simulation::Axis::Z, self.z_dir_pin.is_high());
        }
        #[cfg(feature = "with-e-axis")]
        if _mask.contains(hwa::CoordSel::E) {
            if _channels.contains(hwa::CoordSel::E) {
                self.e_dir_pin.set_high();
            }
            else {
                self.e_dir_pin.set_low();
            }
        }
    }

    fn step_toggle(&mut self, _channels: hwa::CoordSel) {
        #[cfg(feature = "with-x-axis")]
        if _channels.contains(hwa::CoordSel::X) {
            self.x_step_pin.toggle();
            simulation::step(simulation::Axis::X);
        }
        #[cfg(feature = "with-y-axis")]
        if _channels.contains(hwa::CoordSel::Y) {
            self.y_step_pin.toggle();
            simulation::step(simulation::Axis::Y);
        }
        #[cfg(feature = "with-z-axis")]
        if _channels.contains(hwa::CoordSel::Z) {
            self.z_step_pin.toggle();
            simulation::step(simulation::Axis::Z);
        }
        #[cfg(feature = "with-e-axis")]
        if _channels.contains(hwa::CoordSel::E) {
            self.e_step_pin.toggle();
        }
    }

    fn step_high(&mut self, _channels: hwa::CoordSel) {
        #[cfg(feature = "with-x-axis")]
        if _channels.contains(hwa::CoordSel::X) {
            // A step is the rising edge of the pulse
            if self.x_step_pin.is_low() {
                simulation::step(simulation::Axis::X);
            }
            self.x_step_pin.set_high();
        }
        #[cfg(feature = "with-y-axis")]
        if _channels.contains(hwa::CoordSel::Y) {
            // A step is the rising edge of the pulse
            if self.y_step_pin.is_low() {
                simulation::step(simulation::Axis::Y);
            }
            self.y_step_pin.set_high();
        }
        #[cfg(feature = "with-z-axis")]
        if _channels.contains(hwa::CoordSel::Z) {
            // A step is the rising edge of the pulse
            if self.z_step_pin.is_low() {
                simulation::step(simulation::Axis::Z);
            }
            self.z_step_pin.set_high();
        }
        #[cfg(feature = "with-e-axis")]
        if _channels.contains(hwa::CoordSel::E) {
            self.e_step_pin.set_high();
        }
    }

    fn step_low(&mut self, _channels: hwa::CoordSel) {
        #[cfg(feature = "with-x-axis")]
        if _channels.contains(hwa::CoordSel::X) {
            self.x_step_pin.set_low();
        }
        #[cfg(feature = "with-y-axis")]
        if _channels.contains(hwa::CoordSel::Y) {
            self.y_step_pin.set_low();
        }
        #[cfg(feature = "with-z-axis")]
        if _channels.contains(hwa::CoordSel::Z) {
            self.z_step_pin.set_low();
        }
        #[cfg(feature = "with-e-axis")]
        if _channels.contains(hwa::CoordSel::E) {
            self.e_step_pin.set_low();
        }
    }

    fn endstop_triggered(&mut self, _channels: hwa::CoordSel) -> bool {
        #[allow(unused_mut)]
        let mut triggered = false;
        #[cfg(feature = "with-x-axis")]
        if _channels.contains(hwa::CoordSel::X) {
            if simulation::end_stop_active(simulation::Axis::X) {
                self.x_endstop_pin.set_high();
            } else {
                self.x_endstop_pin.set_low();
            }
            triggered |= self.x_endstop_pin.is_high();
        }
        #[cfg(feature = "with-y-axis")]
        if _channels.contains(hwa::CoordSel::Y) {
            if simulation::end_stop_active(simulation::Axis::Y) {
                self.y_endstop_pin.set_high();
            } else {
                self.y_endstop_pin.set_low();
            }
            triggered |= self.y_endstop_pin.is_high();
        }
        #[cfg(feature = "with-z-axis")]
        if _channels.contains(hwa::CoordSel::Z) {
            if simulation::end_stop_active(simulation::Axis::Z) {
                self.z_endstop_pin.set_high();
            } else {
                self.z_endstop_pin.set_low();
            }
            triggered |= self.z_endstop_pin.is_high();
        }
        #[cfg(feature = "with-e-axis")]
        if _channels.contains(hwa::CoordSel::E) {
            triggered |= self.e_endstop_pin.is_high();
        }
        triggered
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
        pub type MotionSender = board::mocked_peripherals::MockedI2c;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-probe", feature = "with-hot-end",feature = "with-hot-bed",
        feature = "with-laser", feature = "with-fan-layer", feature = "with-fan-extra-1"))] {

        pub type Pwm1 = board::mocked_peripherals::MockedPwm;
        //pub type Pwm2 = board::mocked_peripherals::MockedPwm;

    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end",feature = "with-hot-bed"))] {

        pub type Adc1 = board::mocked_peripherals::MockedAdc<u8>;
        //pub type Adc2 = board::mocked_peripherals::MockedAdc<u8>;

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
        //pub type HotEndAdc = Adc1;
        pub type HotEndAdcPin = board::mocked_peripherals::MockedIOPin;
        pub type HotEndPwm = Pwm1;
        pub type HotEndPwmChannel = <HotEndPwm as hwa::traits::Pwm>::Channel;

    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-bed")] {
        //pub type HotBedAdc = Adc1;
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

cfg_if::cfg_if! {
    if #[cfg(feature = "with-laser")] {
        pub type LaserPwm = Pwm1;
        pub type LaserPwmChannel = <LaserPwm as hwa::traits::Pwm>::Channel;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-extra-1")] {
        pub type FanExtra1Pwm = Pwm1;
        pub type FanExtra1PwmChannel = <FanExtra1Pwm as hwa::traits::Pwm>::Channel;

    }
}

#[cfg(feature = "with-eeprom")]
pub type Eeprom = board::mocked_peripherals::MockedEeprom;

#[cfg(feature = "with-trinamic")]
pub type TrinamicUart = board::mocked_peripherals::MockedTrinamicUart;

