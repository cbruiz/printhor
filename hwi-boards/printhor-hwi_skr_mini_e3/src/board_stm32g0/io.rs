//! IO wrappers for SKR Mini E3 V3 (STM32G0B1RE)
#[allow(unused)]
use printhor_hwa_common as hwa;
#[cfg(feature = "with-serial-usb")]
pub(crate) mod serial_usb {
    use printhor_hwa_common as hwa;
    use hwa::HwiContract;

    pub type SerialUsbDeviceSender = embassy_usb::class::cdc_acm::Sender<'static, crate::board::device::SerialUsbDriver>;
    pub type SerialUsbDeviceReceiver = embassy_usb::class::cdc_acm::Receiver<'static, crate::board::device::SerialUsbDriver>;

    pub struct SerialUsbDevice {
        pub builder: Option<embassy_usb::Builder<'static, crate::board::device::SerialUsbDriver>>,
        pub sender: SerialUsbDeviceSender,
        pub receiver: SerialUsbDeviceReceiver,
    }

    #[embassy_executor::task(pool_size = 1)]
    pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, crate::board::device::SerialUsbDriver>) -> ! {
        hwa::debug!("Running usb task...");
        usb.run().await;
    }

    impl SerialUsbDevice {
        pub fn new(driver: crate::board::device::SerialUsbDriver) -> Self {
            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Printhor");
            config.product = Some("Printhor-USBSerial");
            config.serial_number = Some("");

            config.device_class = 0xEF;
            config.device_sub_class = 0x02;
            config.device_protocol = 0x01;
            config.composite_with_iads = true;
            type UsbConfigDescriptorType = [u8; 256];
            let config_descriptor = hwa::make_static_ref!(
                "UsbConfigDescriptor",
                UsbConfigDescriptorType,
                [0; 256]
            );
            type UsbBOSDescriptorType = [u8; 256];
            let bos_descriptor = hwa::make_static_ref!(
                "UsbBOSDescriptor",
                UsbBOSDescriptorType,
                [0; 256]
            );
            type UsbMSOSDescriptorType = [u8; 256];
            let m_sos_descriptor = hwa::make_static_ref!(
                "UsbMSOSDescriptor",
                UsbMSOSDescriptorType,
                [0; 256]
            );
            type UsbControlBuffType = [u8; 64];
            let control_buf = hwa::make_static_ref!(
                "UsbMSOSDescriptor",
                UsbControlBuffType,
                [0; 64]
            );

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
                .map(|builder| match spawner.spawn(usb_task(builder.build())) {
                    Ok(_) => (),
                    Err(_) => {
                        panic!("Unable to spawn USB task")
                    }
                });
        }

        pub fn split(self) -> (SerialUsbDeviceReceiver, SerialUsbDeviceSender) {
            (self.receiver, self.sender)
        }
    }

    pub struct SerialUsbRxInputStream {
        receiver: embassy_usb::class::cdc_acm::Receiver<'static, super::super::device::SerialUsbDriver>,
        buffer: [u8; <crate::Contract as HwiContract>::SERIAL_USB_RX_BUFFER_SIZE],
        bytes_read: u8,
        current_byte_index: u8,
    }

    impl SerialUsbRxInputStream {
        pub fn new(receiver: SerialUsbDeviceReceiver) -> Self {
            Self {
                receiver,
                buffer: [0; <crate::Contract as HwiContract>::SERIAL_USB_RX_BUFFER_SIZE],
                bytes_read: 0,
                current_byte_index: 0,
            }
        }
    }

    impl async_gcode::ByteStream for SerialUsbRxInputStream {
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

        async fn recovery_check(&mut self) { }
    }
}

#[cfg(feature = "with-serial-port-1")]
pub(crate) mod serial_port_1 {
    #[allow(unused)]
    use embedded_io_async::{BufRead, Read};
    use printhor_hwa_common as hwa;
    #[allow(unused)]
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

            let mut buff: [u8; 1] = [0; 1];

            match self.receiver.read_exact(&mut buff[..]).await {
                Ok(_r) => {
                    Some(Ok(buff[0]))
                },
                Err(_e) => None,
            }
        }

        async fn recovery_check(&mut self) {

        }
    }
}

#[cfg(feature = "with-serial-port-2")]
pub(crate) mod serial_port_2 {
    #[allow(unused)]
    use embedded_io_async::{BufRead, Read};
    use printhor_hwa_common as hwa;
    #[allow(unused)]
    use hwa::HwiContract;

    pub struct SerialPort2RxInputStream {
        receiver: embassy_stm32::usart::RingBufferedUartRx<'static>,
    }

    impl SerialPort2RxInputStream {
        pub fn new(receiver: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>) -> Self {
            type BufferType = [u8; <crate::Contract as HwiContract>::SERIAL_PORT2_RX_BUFFER_SIZE];

            Self {
                receiver: receiver.into_ring_buffered(hwa::make_static_ref!(
                    "SerialPort2RXRingBuff",
                    BufferType,
                    [0; <crate::Contract as HwiContract>::SERIAL_PORT2_RX_BUFFER_SIZE]
                )),
            }
        }
    }

    impl async_gcode::ByteStream for SerialPort2RxInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            let mut buff: [u8; 1] = [0; 1];

            match self.receiver.read_exact(&mut buff[..]).await {
                Ok(_r) => {
                    Some(Ok(buff[0]))
                },
                Err(_e) => None,
            }
        }

        async fn recovery_check(&mut self) {

        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-trinamic")] {

        pub struct TrinamicUartWrapper {
            rx: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>,
            tx: embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>,
        }

        impl TrinamicUartWrapper {
            pub fn new(inner: crate::board::device::TrinamicUart) -> Self {

                let (tx, rx) = inner.split();
                Self {
                    rx,
                    tx,
                }
            }
        }
        impl hwa::traits::TrinamicUartTrait for TrinamicUartWrapper {

            fn get_tmc_address(&self, _coord: hwa::CoordSel) -> Result<u8, ()> {
                #[cfg(feature="with-x-axis")]
                if _coord == hwa::CoordSel::X {
                    return Ok(0)
                }
                #[cfg(feature="with-y-axis")]
                if _coord == hwa::CoordSel::Y {
                    return Ok(1)
                }
                #[cfg(feature="with-z-axis")]
                if _coord == hwa::CoordSel::Z {
                    return Ok(2)
                }
                #[cfg(feature="with-e-axis")]
                if _coord == hwa::CoordSel::E {
                    return Ok(3)
                }
                hwa::error!("Uknown TMC stepper axis activation requested: {:?}", _coord);
                return Err(())
            }

            async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, printhor_hwa_common::uart::SerialError> {
                match embassy_time::with_timeout(embassy_time::Duration::from_secs(1), self.rx.read_until_idle(buffer)).await {
                    Ok(Ok(x)) => {
                        hwa::info!("Ok read: {} bytes", x);
                        Ok(x)
                    },
                    Ok(Err(_error)) => {
                        hwa::error!("Error: {}", _error);
                        Err(hwa::uart::SerialError::Framing)
                    },
                    Err(_) => {
                        hwa::warn!("Timeout");
                        Err(hwa::uart::SerialError::Timeout)
                    }
                 }
            }

            async fn write(&mut self,buffer: &[u8]) -> Result<(), hwa::uart::SerialError> {
                Ok(self.tx.write(buffer).await.map_err(|_| {hwa::uart::SerialError::Framing})?)
            }

            async fn flush(&mut self) -> Result<(), hwa::uart::SerialError> {
                Ok(self.tx.flush().await.map_err(|_| {hwa::uart::SerialError::Framing})?)
            }
        }
    }
}
