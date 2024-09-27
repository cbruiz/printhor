#[cfg(feature = "with-serial-usb")]
pub mod usbserial {
    use crate::board::device::USBDrv;

    pub type USBSerialDeviceSender = embassy_usb::class::cdc_acm::Sender<'static, USBDrv>;
    pub type USBSerialDeviceReceiver = embassy_usb::class::cdc_acm::Receiver<'static, USBDrv>;
    pub type USBSerialTxControllerRef = printhor_hwa_common::InterruptControllerRef<USBSerialDeviceSender>;

    pub struct USBSerialDevice {
        pub builder: Option<embassy_usb::Builder<'static, USBDrv>>,
        pub sender: USBSerialDeviceSender,
        pub receiver: USBSerialDeviceReceiver,
    }

    #[embassy_executor::task(pool_size=1)]
    pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, USBDrv>) -> ! {
        defmt::debug!("Running usb task...");
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
            #[link_section = ".bss"]
            static CONFIG_DESCRIPTOR_ST: printhor_hwa_common::TrackedStaticCell<[u8; 256]> = printhor_hwa_common::TrackedStaticCell::new();
            let config_descriptor = CONFIG_DESCRIPTOR_ST.init::<{crate::MAX_STATIC_MEMORY}>("UsbConfigDescriptor", [0; 256]);
            #[link_section = ".bss"]
            static BOS_DESCRIPTOR_ST: printhor_hwa_common::TrackedStaticCell<[u8; 256]> = printhor_hwa_common::TrackedStaticCell::new();
            let bos_descriptor = BOS_DESCRIPTOR_ST.init::<{crate::MAX_STATIC_MEMORY}>("UsbBOSDescriptor", [0; 256]);
            #[link_section = ".bss"]
            static MSOS_DESCRIPTOR_ST: printhor_hwa_common::TrackedStaticCell<[u8; 256]> = printhor_hwa_common::TrackedStaticCell::new();
            let msos_descriptor = MSOS_DESCRIPTOR_ST.init::<{crate::MAX_STATIC_MEMORY}>("UsbMSOSDescriptor", [0; 256]);

            #[link_section = ".bss"]
            static CONTROL_BUF_ST: printhor_hwa_common::TrackedStaticCell<[u8; 64]> = printhor_hwa_common::TrackedStaticCell::new();
            let control_buf = CONTROL_BUF_ST.init::<{crate::MAX_STATIC_MEMORY}>("UsbControlBuff", [0; 64]);

            #[link_section = ".bss"]
            static STATE_ST: printhor_hwa_common::TrackedStaticCell<embassy_usb::class::cdc_acm::State> = printhor_hwa_common::TrackedStaticCell::new();
            let state = STATE_ST.init::<{crate::MAX_STATIC_MEMORY}>("UsbCDCACMState", embassy_usb::class::cdc_acm::State::new());
            let mut builder = embassy_usb::Builder::new(
                driver,
                config,
                config_descriptor,
                bos_descriptor,
                msos_descriptor,
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
    }

    impl USBSerialDeviceInputStream {
        pub fn new(receiver: USBSerialDeviceReceiver) -> Self {
            Self {
                receiver,
                buffer: [0; crate::USBSERIAL_BUFFER_SIZE],
                bytes_read: 0,
                current_byte_index: 0,
            }
        }
    }

    impl async_gcode::ByteStream for USBSerialDeviceInputStream
    {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {

            if self.current_byte_index < self.bytes_read {
                let byte = self.buffer[self.current_byte_index as usize];
                self.current_byte_index += 1;
                Some(Ok(byte))
            }
            else {
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
                        }
                        else {
                            None
                        }
                    }
                    Err(_e) => {
                        None
                    }
                }
            }
        }
    }
}

#[cfg(feature = "with-serial-port-1")]
pub mod uart_port1 {
    use crate::board::device::UartPort1RxDevice;
    use crate::board::device::UartPort1RingBufferedRxDevice;
    use printhor_hwa_common::TrackedStaticCell;

    pub struct UartPort1RxInputStream {
        receiver: UartPort1RingBufferedRxDevice,
    }

    impl UartPort1RxInputStream {
        pub fn new(receiver: UartPort1RxDevice) -> Self {
            #[link_section =".bss"]
            static BUFF: TrackedStaticCell<[u8; crate::UART_PORT1_BUFFER_SIZE]> = TrackedStaticCell::new();
            let buffer = BUFF.init::<{crate::board::MAX_STATIC_MEMORY}>("UartPort1RXRingBuff", [0; crate::UART_PORT1_BUFFER_SIZE]);

            Self {
                receiver: receiver.into_ring_buffered(buffer),
            }
        }
    }

    impl async_gcode::ByteStream for UartPort1RxInputStream
    {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {

            let mut buff: [u8; 1] = [0; 1];

            match self.receiver.read(&mut buff).await {
                Ok(_r) => Some(Ok(buff[0])),
                Err(_e) => None,
            }
        }
    }
}
#[cfg(feature = "with-serial-port-2")]
compiler_error!("Not implemented");