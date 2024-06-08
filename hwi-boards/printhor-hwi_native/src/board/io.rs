#[cfg(feature = "with-serial-port-usb")]
pub mod usbserial {
    use crate::board::device::USBDrv;
    pub type USBSerialDeviceSender = embassy_usb::class::cdc_acm::Sender<'static, USBDrv>;
    pub type USBSerialDeviceReceiver = embassy_usb::class::cdc_acm::Receiver<'static, USBDrv>;
    pub type USBSerialTxControllerRef = crate::board::ControllerRef<USBSerialDeviceSender>;

    pub struct USBSerialDevice {
        pub builder: Option<embassy_usb::Builder<'static, USBDrv>>,
        pub sender: USBSerialDeviceSender,
        pub receiver: USBSerialDeviceReceiver,
    }

    #[embassy_executor::task(pool_size=1)]
    pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, USBDrv>) -> ! {
        defmt::info!("Running usb task...");
        usb.run().await;
        unreachable!("usb task ended")
    }

    impl USBSerialDevice {
        pub fn new(driver: USBDrv) -> Self {

            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Printor");
            config.product = Some("Printor-USBSerial");
            config.serial_number = Some("");

            config.device_class = 0xEF;
            config.device_sub_class = 0x02;
            config.device_protocol = 0x01;
            config.composite_with_iads = true;
            static DEVICE_DESCRIPTOR_ST: crate::board::TrackedStaticCell<[u8; 256]> = crate::board::TrackedStaticCell::new();
            let device_descriptor = DEVICE_DESCRIPTOR_ST.init("", [0; 256]);
            static CONFIG_DESCRIPTOR_ST: crate::board::TrackedStaticCell<[u8; 256]> = crate::board::TrackedStaticCell::new();
            let config_descriptor = CONFIG_DESCRIPTOR_ST.init("", [0; 256]);
            static BOS_DESCRIPTOR_ST: crate::board::TrackedStaticCell<[u8; 256]> = crate::board::TrackedStaticCell::new();
            let bos_descriptor = BOS_DESCRIPTOR_ST.init("", [0; 256]);
            static CONTROL_BUF_ST: crate::board::TrackedStaticCell<[u8; 64]> = crate::board::TrackedStaticCell::new();
            let control_buf = CONTROL_BUF_ST.init("", [0; 64]);

            static STATE_ST: crate::board::TrackedStaticCell<embassy_usb::class::cdc_acm::State> = crate::board::TrackedStaticCell::new();
            let state = STATE_ST.init("", embassy_usb::class::cdc_acm::State::new());
            let mut builder = embassy_usb::Builder::new(
                driver,
                config,
                device_descriptor,
                config_descriptor,
                bos_descriptor,
                control_buf,
            );

            //crate::info!("Creating USB CLASS");

            // Create classes on the builder.
            let class = embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 64);

            let (sender, receiver) = class.split();

            Self {
                builder: Some(builder),
                sender,
                receiver,
            }
        }

        pub fn spawn(&mut self, spawner: crate::board::Spawner) {
            self.builder.take().map(|builder| {
                match spawner.spawn(usb_task(builder.build())) {
                    Ok(_) => {
                        ()
                    }
                    Err(_) => {
                        panic!("Unable to spawn USB task")
                    }
                }
            });
        }

        pub fn split(self) -> (USBSerialDeviceReceiver, USBSerialDeviceSender) {
            (
                self.receiver,
                self.sender,
            )
        }
    }

    pub struct USBSerialDeviceInputStream {
        receiver: embassy_usb::class::cdc_acm::Receiver<'static, crate::board::device::USBDrv>,
        buffer: [u8; crate::USBSERIAL_BUFFER_SIZE],
        bytes_read: u8,
        current_byte_index: u8,
        connected: bool,
    }

    #[cfg(feature = "with-serial-port-usb")]
    impl USBSerialDeviceInputStream {
        pub fn new(receiver: USBSerialDeviceReceiver) -> Self {
            Self {
                receiver,
                buffer: [0; crate::USBSERIAL_BUFFER_SIZE],
                bytes_read: 0,
                current_byte_index: 0,
                connected: false,
            }
        }
    }

    #[cfg(feature = "with-serial-port-usb")]
    impl async_gcode::AsyncRead for USBSerialDeviceInputStream
    {
        async fn read_byte(&mut self) -> Option<Result<u8, async_gcode::Error>> {
            if self.current_byte_index < self.bytes_read {
                let byte = self.buffer[self.current_byte_index as usize];
                self.current_byte_index += 1;
                Some(Ok(byte))
            } else {
                self.current_byte_index = 0;
                self.bytes_read = 0;
                if !self.connected {
                    self.receiver.wait_connection().await;
                    self.connected = true;
                }
                match self.receiver.read_packet(&mut self.buffer).await {
                    Ok(n) => {
                        self.bytes_read = n as u8;
                        if n > 0 {
                            let byte = self.buffer[self.current_byte_index as usize];
                            self.current_byte_index = 1;
                            Some(Ok(byte))
                        } else {
                            None
                        }
                    }
                    Err(_) => {
                        self.connected = false;
                        //crate::error!("got error!");
                        Some(Err(async_gcode::Error::InvalidUTF8String))
                    }
                }
            }
        }

        #[inline]
        fn push_back(&mut self, _b: u8) {
            if self.current_byte_index > 0 {
                self.current_byte_index -= 1;
            }
        }

        async fn close(&mut self) {}
    }
}

#[cfg(feature = "with-serial-port-2")]
pub mod uart2 {
    use crate::device::Usart2Rx;

    pub struct Uart2DeviceInputStream {
        pub receiver: Usart2Rx,
        buffer: [u8; crate::USART2_BUFFER_SIZE],
        bytes_read: u8,
        current_byte_index: u8,
    }

    impl Uart2DeviceInputStream {
        pub fn new(receiver: Usart2Rx) -> Self {
            Self {
                receiver,
                buffer: [0; crate::USART2_BUFFER_SIZE],
                bytes_read: 0,
                current_byte_index: 0,
            }
        }
    }

    impl async_gcode::AsyncRead for Uart2DeviceInputStream
    {
        async fn read_byte(&mut self) -> Option<Result<u8, async_gcode::Error>> {
            if self.current_byte_index < self.bytes_read {
                let byte = self.buffer[self.current_byte_index as usize];
                self.current_byte_index += 1;
                Some(Ok(byte))
            }
            else {
                self.current_byte_index = 0;
                self.bytes_read = 0;
                match self.receiver.read_until_idle(&mut self.buffer).await {
                    Ok(n) => {
                        self.bytes_read = n as u8;
                        if n > 0 {
                            let byte = self.buffer[self.current_byte_index as usize];
                            self.current_byte_index = 1;
                            Some(Ok(byte))
                        }
                        else {
                            None
                        }
                    }
                    Err(_) => {
                        Some(Err(async_gcode::Error::InvalidUTF8String))
                    }
                }
            }
        }

        #[inline]
        fn push_back(&mut self, _b: u8)  {
            if self.current_byte_index > 0 {
                self.current_byte_index -= 1;
            }
        }

        async fn close(&mut self) {

        }
    }

}