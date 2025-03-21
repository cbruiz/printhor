//! This specialization exists because there are several ways to communicate with stepper's Trinamic UART

#[cfg(feature = "with-trinamic")]
pub(crate) mod trinamic {
    #[allow(unused)]
    use printhor_hwa_common as hwa;
    use embassy_stm32::gpio::{Flex, Pull, Speed};
    use hwa::{
        uart::SerialError,
        soft_uart,
    };
    use hwa::CoordSel;
    use hwa::traits::TrinamicUartTrait;

    pub struct AnyPinWrapper(pub Flex<'static>);

    impl soft_uart::IOPin for AnyPinWrapper {
        fn set_output(&mut self) {
            self.0.set_as_output(Speed::VeryHigh);
        }
        fn set_input(&mut self) {
            self.0.set_as_input(Pull::Up);
        }
        fn is_high(&mut self) -> bool {
            self.0.is_high()
        }
        fn set_high(&mut self) {
            self.0.set_high()
        }
        fn set_low(&mut self) {
            self.0.set_low()
        }
        fn set_open_drain(&mut self) {
            self.0.set_as_input_output(Speed::VeryHigh)
        }
    }

    pub struct SingleWireSoftwareUart {
        selected: Option<CoordSel>,
        #[cfg(feature = "with-x-axis")]
        x_stepper_uart: soft_uart::HalfDuplexSerial<AnyPinWrapper>,
        #[cfg(feature = "with-y-axis")]
        y_stepper_uart: soft_uart::HalfDuplexSerial<AnyPinWrapper>,
        #[cfg(feature = "with-z-axis")]
        z_stepper_uart: soft_uart::HalfDuplexSerial<AnyPinWrapper>,
        #[cfg(feature = "with-e-axis")]
        e_stepper_uart: soft_uart::HalfDuplexSerial<AnyPinWrapper>,
    }

    impl SingleWireSoftwareUart {
        pub fn new(
            baud_rate: u32,
            #[cfg(feature = "with-x-axis")]
            x_stepper_pin: AnyPinWrapper,
            #[cfg(feature = "with-y-axis")]
            y_stepper_pin: AnyPinWrapper,
            #[cfg(feature = "with-z-axis")]
            z_stepper_pin: AnyPinWrapper,
            #[cfg(feature = "with-e-axis")]
            e_stepper_pin: AnyPinWrapper,
        ) -> Self {
            Self {
                #[cfg(feature = "with-x-axis")]
                x_stepper_uart: soft_uart::HalfDuplexSerial::new(x_stepper_pin, baud_rate),
                #[cfg(feature = "with-y-axis")]
                y_stepper_uart: soft_uart::HalfDuplexSerial::new(y_stepper_pin, baud_rate),
                #[cfg(feature = "with-z-axis")]
                z_stepper_uart: soft_uart::HalfDuplexSerial::new(z_stepper_pin, baud_rate),
                #[cfg(feature = "with-e-axis")]
                e_stepper_uart: soft_uart::HalfDuplexSerial::new(e_stepper_pin, baud_rate),
                selected: None,
            }
        }
    }

    impl TrinamicUartTrait for SingleWireSoftwareUart {
        fn select_stepper_of_axis(&mut self, channel: CoordSel) -> Result<(), ()> {
            if channel.is_empty() {
                self.selected = None;
                return Ok(());
            }
            #[cfg(feature = "with-x-axis")]
            if channel == CoordSel::X {
                self.selected = Some(channel);
                return Ok(());
            }
            #[cfg(feature = "with-y-axis")]
            if channel == CoordSel::Y {
                self.selected = Some(channel);
                return Ok(());
            }
            #[cfg(feature = "with-z-axis")]
            if channel == CoordSel::Z {
                self.selected = Some(channel);
                return Ok(());
            }
            #[cfg(feature = "with-e-axis")]
            if channel == CoordSel::E {
                self.selected = Some(channel);
                return Ok(());
            }
            hwa::error!("Invalid stepper channel selection: {:?}", channel);
            Err(())
        }

        fn get_tmc_address(&self, _channel: CoordSel) -> Result<u8, ()> {
            #[cfg(feature = "with-x-axis")]
            if _channel == CoordSel::X {
                return Ok(0);
            }
            #[cfg(feature = "with-y-axis")]
            if _channel == CoordSel::Y {
                return Ok(0);
            }
            #[cfg(feature = "with-z-axis")]
            if _channel == CoordSel::Z {
                return Ok(0);
            }
            #[cfg(feature = "with-e-axis")]
            if _channel == CoordSel::E {
                return Ok(0);
            }
            hwa::error!("Invalid stepper channel address requested: {:?}", _channel);
            Err(())
        }

        async fn read_until_idle(&mut self, _buffer: &mut [u8]) -> Result<usize, SerialError> {

            use printhor_hwa_common::soft_uart::AsyncRead;

            let uart = match self.selected {
                #[cfg(feature = "with-x-axis")]
                Some(CoordSel::X) => &mut self.x_stepper_uart,
                #[cfg(feature = "with-y-axis")]
                Some(CoordSel::Y) => &mut self.y_stepper_uart,
                #[cfg(feature = "with-z-axis")]
                Some(CoordSel::Z) => &mut self.z_stepper_uart,
                #[cfg(feature = "with-e-axis")]
                Some(CoordSel::E) => &mut self.e_stepper_uart,
                _ => return Err(SerialError::Framing)
            };

            uart.set_timeout(Some(uart.word_transfer_duration() * 10));

            uart.set_read_mode().await;

            let mut nb = 0;
            while nb < _buffer.len() {
                match uart.read().await {
                    Ok(_br) => {
                        _buffer[nb] = _br;
                        nb += 1;
                    }
                    Err(_e) => {
                        return if nb > 0 {
                            Ok(nb)
                        } else {
                            Err(SerialError::Timeout)
                        }
                    }
                }
            }
            Ok(nb)
        }

        async fn write(&mut self, _buffer: &[u8]) -> Result<(), SerialError> {
            use printhor_hwa_common::soft_uart::AsyncWrite;

            let uart = match self.selected {
                #[cfg(feature = "with-x-axis")]
                Some(CoordSel::X) => &mut self.x_stepper_uart,
                #[cfg(feature = "with-y-axis")]
                Some(CoordSel::Y) => &mut self.y_stepper_uart,
                #[cfg(feature = "with-z-axis")]
                Some(CoordSel::Z) => &mut self.z_stepper_uart,
                #[cfg(feature = "with-e-axis")]
                Some(CoordSel::E) => &mut self.e_stepper_uart,
                _ => return Err(SerialError::Framing)
            };

            uart.set_write_mode().await;

            for b in _buffer {
                let _x = uart.write(*b).await;
            }
            Ok(())
        }

        async fn flush(&mut self) -> Result<(), SerialError> {
            // The software uart is synchronous by nature
            Ok(())
        }
    }

}

