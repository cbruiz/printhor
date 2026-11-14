#[cfg(feature = "with-serial-usb")]
pub mod serial_usb {
    use printhor_hwa_common as hwa;
    use hwa::HwiContract;
    use super::super::device::SerialUsbDriver;

    pub type SerialUsbSender = embassy_usb::class::cdc_acm::Sender<'static, SerialUsbDriver>;
    pub type SerialUsbReceiver = embassy_usb::class::cdc_acm::Receiver<'static, SerialUsbDriver>;

    pub struct SerialUsbDevice {
        pub builder: Option<embassy_usb::Builder<'static, SerialUsbDriver>>,
        pub sender: SerialUsbSender,
        pub receiver: SerialUsbReceiver,
    }

    #[embassy_executor::task(pool_size = 1)]
    pub async fn serial_usb_task(mut usb: embassy_usb::UsbDevice<'static, SerialUsbDriver>) -> ! {
        hwa::debug!("Running serial usb task...");
        usb.run().await;
    }

    impl SerialUsbDevice {
        pub fn new(driver: SerialUsbDriver) -> Self {
            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Printhor");
            config.product = Some("Printhor USBSerial");
            config.serial_number = Some("printhor0");

            config.device_class = 0xEF;
            config.device_sub_class = 0x02;
            config.device_protocol = 0x01;
            config.composite_with_iads = true;
            type UsbConfigDescriptorType = [u8; 256];
            let config_descriptor =
                hwa::make_static_ref!("UsbConfigDescriptor", UsbConfigDescriptorType, [0; 256]);
            type UsbBOSDescriptorType = [u8; 256];
            let bos_descriptor =
                hwa::make_static_ref!("UsbBOSDescriptor", UsbBOSDescriptorType, [0; 256]);
            type UsbMSOSDescriptorType = [u8; 256];
            let m_sos_descriptor =
                hwa::make_static_ref!("UsbMSOSDescriptor", UsbMSOSDescriptorType, [0; 256]);
            type UsbControlBuffType = [u8; 64];
            let control_buf =
                hwa::make_static_ref!("UsbMSOSDescriptor", UsbControlBuffType, [0; 64]);

            let state = hwa::make_static_ref!(
                "UsbCDCACMState",
                embassy_usb::class::cdc_acm::State,
                embassy_usb::class::cdc_acm::State::new(),
            );

            let mut builder = embassy_usb::Builder::new(
                driver,
                config,
                config_descriptor,
                bos_descriptor,
                m_sos_descriptor,
                control_buf,
            );

            // Create classes on the builder.
            let class = embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 64);

            let (sender, receiver) = class.split();

            Self {
                builder: Some(builder),
                sender,
                receiver,
            }
        }

        pub fn spawn(&mut self, spawner: embassy_executor::Spawner) {
            self.builder
                .take()
                .map(|builder| match serial_usb_task(builder.build()) {
                    Ok(token) => spawner.spawn(token),
                    Err(_) => {
                        panic!("Unable to spawn USB task")
                    }
                });
        }

        pub fn split(self) -> (SerialUsbReceiver, SerialUsbSender) {
            (self.receiver, self.sender)
        }
    }

    pub struct SerialUsbInputStream {
        receiver: embassy_usb::class::cdc_acm::Receiver<'static, SerialUsbDriver>,
        buffer: [u8; <crate::Contract as HwiContract>::SERIAL_USB_PACKET_SIZE],
        bytes_read: u8,
        current_byte_index: u8,
    }

    impl SerialUsbInputStream {
        pub fn new(receiver: SerialUsbReceiver) -> Self {
            Self {
                receiver,
                buffer: [0; <crate::Contract as HwiContract>::SERIAL_USB_PACKET_SIZE],
                bytes_read: 0,
                current_byte_index: 0,
            }
        }
    }

    impl async_gcode::ByteStream for SerialUsbInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            if self.current_byte_index < self.bytes_read {
                let byte = self.buffer[self.current_byte_index as usize];
                self.current_byte_index += 1;
                Some(Ok(byte))
            } else {
                self.current_byte_index = 0;
                self.bytes_read = 0;
                self.receiver.wait_connection().await;

                let r = self.receiver.read_packet(&mut self.buffer).await;
                match r {
                    Ok(n) => {
                        self.bytes_read = n as u8;
                        if n > 0 {
                            let byte = self.buffer[self.current_byte_index as usize];
                            self.current_byte_index += 1;
                            Some(Ok(byte))
                        } else {
                            None
                        }
                    }
                    Err(_e) => None,
                }
            }
        }

        async fn recovery_check(&mut self) {

        }
    }
}

#[cfg(feature = "with-serial-port-1")]
pub mod serial_port_1 {
    use printhor_hwa_common as hwa;
    use hwa::HwiContract;

    pub struct SerialPort1RxInputStream {
        receiver: embassy_stm32::usart::RingBufferedUartRx<'static>,
    }

    impl SerialPort1RxInputStream {
        pub fn new(receiver: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>) -> Self {
            type BufferType = [u8; <crate::Contract as HwiContract>::SERIAL_PORT1_RX_BUFFER_SIZE];

            Self {
                receiver: receiver.into_ring_buffered(hwa::make_static_ref!(
                    "SerialPort1RXRingBuff",
                    BufferType,
                    [0; <crate::Contract as HwiContract>::SERIAL_PORT1_RX_BUFFER_SIZE]
                )),
            }
        }
    }

    impl async_gcode::ByteStream for SerialPort1RxInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            use embedded_io_async::Read;

            let mut buff: [u8; 1] = [0; 1];
            match self.receiver.read_exact(&mut buff).await {
                Ok(_r) => {
                    hwa::trace!("read {}", buff[0] as char);
                    Some(Ok(buff[0]))
                },
                Err(_e) => None,
            }
        }

        async fn recovery_check(&mut self) {
            // Optional. Needed when io is susceptible to need reestablished or reset
        }
    }
}
#[cfg(feature = "with-serial-port-2")]
pub mod serial_port_2 {
    use printhor_hwa_common as hwa;
    use hwa::HwiContract;

    pub struct SerialPort2RxInputStream {
        receiver: embassy_stm32::usart::RingBufferedUartRx<'static>,
    }

    impl SerialPort2RxInputStream {
        pub fn new(receiver: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>) -> Self {
            type BufferType = [u8; <crate::Contract as HwiContract>::SERIAL_PORT2_RX_BUFFER_SIZE];

            Self {
                receiver: receiver.into_ring_buffered(hwa::make_static_ref!(
                    "UartPort2RXRingBuff",
                    BufferType,
                    [0; <crate::Contract as HwiContract>::SERIAL_PORT2_RX_BUFFER_SIZE]
                )),
            }
        }
    }

    impl async_gcode::ByteStream for SerialPort2RxInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            use embedded_io_async::Read;
            let mut buff: [u8; 1] = [0; 1];
            match self.receiver.read_exact(&mut buff).await {
                Ok(_r) => {
                    hwa::trace!("read {}", buff[0] as char);
                    Some(Ok(buff[0]))
                },
                Err(_e) => None,
            }
        }

        async fn recovery_check(&mut self) {
            // Optional. Needed when io is susceptible to need reestablished or reset
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        /// ADC for the thermistors (TH1: PC1, TB: PC0).
        ///
        /// The thermistors are sampled every few hundred milliseconds, so blocking conversions
        /// (a few microseconds each, averaged in software) are simpler than DMA.
        pub mod adc {
            use printhor_hwa_common as hwa;

            /// Number of conversions averaged per sample (the F4 ADC has no hardware oversampling)
            const AVERAGED_SAMPLES: u32 = 8;

            pub struct AdcWrapper<T>
            where
                T: embassy_stm32::adc::Instance<Regs = embassy_stm32::pac::adc::Adc>,
            {
                adc: embassy_stm32::adc::Adc<'static, T>,
            }

            impl<T> AdcWrapper<T>
            where
                T: embassy_stm32::adc::Instance<Regs = embassy_stm32::pac::adc::Adc>,
            {
                pub fn new(adc_peripheral: embassy_stm32::Peri<'static, T>) -> Self {
                    // The ADC prescaler is derived from PCLK2 to keep the kernel clock in range
                    let mut config = embassy_stm32::adc::AdcConfig::default();
                    config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
                    Self {
                        adc: embassy_stm32::adc::Adc::new_with_config(adc_peripheral, config),
                    }
                }
            }

            impl<T> hwa::traits::UnifiedAdc16 for AdcWrapper<T>
            where
                T: embassy_stm32::adc::Instance<Regs = embassy_stm32::pac::adc::Adc>,
            {
                type VRefPin = ();
                type SamplePin = embassy_stm32::adc::AnyAdcChannel<'static, T>;

                fn read_adc(&mut self, pin: &mut Self::SamplePin) -> impl core::future::Future<Output = u16> {
                    // Long sample time: the thermistor inputs are high impedance (4.7K pull-up + RC)
                    let mut sum = 0u32;
                    for _ in 0..AVERAGED_SAMPLES {
                        sum += self.adc.blocking_read(pin, embassy_stm32::adc::SampleTime::CYCLES480) as u32;
                    }
                    let sample = (sum / AVERAGED_SAMPLES) as u16;
                    async move { sample }
                }
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-extra-1")] {
        /// On/off output with the PWM interface, for outputs without a timer channel (FAN1 is on
        /// PC14, which has no timer). Any duty greater than zero turns the output on.
        pub struct OnOffPwm {
            pin: embassy_stm32::gpio::Output<'static>,
            duty: u32,
            enabled: bool,
        }

        impl OnOffPwm {
            pub fn new(pin: embassy_stm32::gpio::Output<'static>) -> Self {
                Self { pin, duty: 0, enabled: false }
            }

            fn apply(&mut self) {
                if self.enabled && self.duty > 0 {
                    self.pin.set_high();
                } else {
                    self.pin.set_low();
                }
            }
        }

        impl printhor_hwa_common::traits::Pwm for OnOffPwm {
            type Channel = ();
            type Time = u32;
            type Duty = u32;

            fn disable(&mut self, _channel: Self::Channel) {
                self.enabled = false;
                self.apply();
            }

            fn enable(&mut self, _channel: Self::Channel) {
                self.enabled = true;
                self.apply();
            }

            fn get_period(&self) -> Self::Time {
                0
            }

            fn get_duty(&self, _channel: Self::Channel) -> Self::Duty {
                self.duty
            }

            fn get_max_duty(&self) -> Self::Duty {
                100
            }

            fn set_duty(&mut self, _channel: Self::Channel, duty: Self::Duty) {
                self.duty = duty.min(100);
                self.apply();
            }

            fn set_period<P>(&mut self, _period: P)
            where
                P: Into<Self::Time>,
            {
            }
        }
    }
}
