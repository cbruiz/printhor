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
                .map(|builder| match spawner.spawn(serial_usb_task(builder.build())) {
                    Ok(_) => (),
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
    use embedded_io_async::ErrorType;
    use printhor_hwa_common as hwa;
    #[allow(unused)]
    use hwa::HwiContract;

    type UartRxDevice = embassy_rp::uart::UartRx<'static,
        embassy_rp::peripherals::UART0,
        embassy_rp::uart::Async>;

    type UartTxDevice = embassy_rp::uart::UartTx<'static,
        embassy_rp::peripherals::UART0,
        embassy_rp::uart::Async>;

    pub struct SerialPort1TxAdapter {
        sender: UartTxDevice,
    }

    impl SerialPort1TxAdapter {
        pub const fn new(sender: UartTxDevice) -> Self {
            Self {
                sender
            }
        }
    }

    impl ErrorType for SerialPort1TxAdapter { type Error = embassy_rp::uart::Error; }

    impl embedded_io_async::Write for SerialPort1TxAdapter {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let len = buf.len();
            self.sender.write(buf).await?;
            Ok(len)
        }
    }

    pub struct SerialPort1RxInputStream {
        receiver: UartRxDevice,
    }

    impl SerialPort1RxInputStream {
        pub fn new(receiver: UartRxDevice) -> Self {

            Self {
                receiver,
            }
        }
    }

    impl async_gcode::ByteStream for SerialPort1RxInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            //use embedded_io_async::Read;

            let mut buff: [u8; 1] = [0; 1];
            match self.receiver.read(&mut buff).await {
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
    use embedded_io_async::ErrorType;
    use printhor_hwa_common as hwa;
    #[allow(unused)]
    use hwa::HwiContract;

    type UartRxDevice = embassy_rp::uart::UartRx<'static,
        embassy_rp::peripherals::UART1,
        embassy_rp::uart::Async>;

    type UartTxDevice = embassy_rp::uart::UartTx<'static,
        embassy_rp::peripherals::UART1,
        embassy_rp::uart::Async>;

    pub struct SerialPort2TxAdapter {
        sender: UartTxDevice,
    }
    
    impl SerialPort2TxAdapter {
        pub const fn new(sender: UartTxDevice) -> Self {
            Self {
                sender
            }
        }
    }

    impl ErrorType for SerialPort2TxAdapter { type Error = embassy_rp::uart::Error; }

    impl embedded_io_async::Write for SerialPort2TxAdapter {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let len = buf.len();
            self.sender.write(buf).await?;
            Ok(len)
        }
    }

    pub struct SerialPort2RxInputStream {
        receiver: UartRxDevice,
    }

    impl SerialPort2RxInputStream {
        pub fn new(receiver: UartRxDevice) -> Self {
            Self {
                receiver,
            }
        }
    }

    impl async_gcode::ByteStream for SerialPort2RxInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            #[allow(unused)]
            use embedded_io_async::Read;
            let mut buff: [u8; 1] = [0; 1];
            match self.receiver.read(&mut buff).await {
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