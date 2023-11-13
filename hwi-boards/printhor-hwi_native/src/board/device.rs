
#[cfg(feature = "with-uart-port-1")]
pub(crate) type UartPort1Device = crate::board::mocked_peripherals::MockedUart;

#[cfg(feature = "with-uart-port-1")]
pub type UartPort1Tx = crate::board::mocked_peripherals::MockedUartTx;

#[cfg(feature = "with-uart-port-1")]
pub type UartPort1Rx = crate::board::mocked_peripherals::MockedUartRx;

#[cfg(feature = "with-uart-port-1")]
pub type UartPort1TxControllerRef = crate::board::ControllerRef<UartPort1Tx>;
#[cfg(feature = "with-uart-port-1")]
pub type UartPort1RxInputStream = crate::board::mocked_peripherals::MockedUartRxInputStream;


#[cfg(feature = "with-trinamic")]
pub type Uart4 = crate::board::mocked_peripherals::MockedUart;

#[cfg(feature = "with-trinamic")]
pub type UartTrinamic = Uart4;

#[cfg(feature = "with-spi")]
pub type Spi = crate::board::mocked_peripherals::MockedSpi;

#[cfg(feature = "with-spi")]
pub type SpiDeviceRef = crate::board::ControllerRef<Spi>;

#[cfg(feature = "with-sdcard")]
pub type SDCardBlockDevice = crate::board::mocked_peripherals::MockledSDCardBlockDevice;

#[cfg(feature = "with-sdcard")]
pub type SDCardBlockDeviceRef = crate::board::ControllerRef<SDCardBlockDevice>;

pub type Watchdog = crate::board::mocked_peripherals::MockedWatchdog<'static, u8>;

#[cfg(feature = "with-display")]
pub type DisplayDevice = crate::board::mocked_peripherals::SimulatorDisplayDevice;
#[cfg(feature = "with-display")]
pub type DisplayScreen<UI> = crate::board::mocked_peripherals::SimulatorDisplayScreen<UI>;

#[cfg(feature = "with-motion")]
pub struct MotionPins {
    pub x_enable_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub y_enable_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub z_enable_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub e_enable_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,

    pub x_endstop_pin: crate::board::mocked_peripherals::MockedInputPin<'static, u8>,
    pub y_endstop_pin: crate::board::mocked_peripherals::MockedInputPin<'static, u8>,
    pub z_endstop_pin: crate::board::mocked_peripherals::MockedInputPin<'static, u8>,
    pub e_endstop_pin: crate::board::mocked_peripherals::MockedInputPin<'static, u8>,

    pub x_step_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub y_step_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub z_step_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub e_step_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,

    pub x_dir_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub y_dir_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub z_dir_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
    pub e_dir_pin: crate::board::mocked_peripherals::MockedOutputPin<'static, u8>,
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
    pub fn enable_e_stepper(&mut self) {
        self.e_enable_pin.set_low();
    }
    #[inline]
    pub fn disable_all_steppers(&mut self) {
        self.x_enable_pin.set_high();
        self.y_enable_pin.set_high();
        self.z_enable_pin.set_high();
        self.e_enable_pin.set_high();
    }
}


#[cfg(feature = "with-motion")]
pub struct MotionDevice {

    #[cfg(feature = "with-trinamic")]
    pub trinamic_uart: UartTrinamic,

    pub motion_pins: MotionPins,
}


#[cfg(feature = "with-sdcard")]
pub struct CardDevice {

    pub card_spi: SDCardBlockDevice,
}
