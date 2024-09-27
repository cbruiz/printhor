#[cfg(feature = "with-serial-port-usb")]

#[cfg(feature = "with-serial-port-1")]
pub mod uart1 {
    use crate::device::Usart1Rx;

    pub struct Uart1DeviceInputStream {
        pub receiver: Usart1Rx,
        buffer: [u8; crate::USART1_BUFFER_SIZE],
        bytes_read: u8,
        current_byte_index: u8,
    }

    impl Uart1DeviceInputStream {
        pub fn new(receiver: Usart1Rx) -> Self {
            Self {
                receiver,
                buffer: [0; crate::USART1_BUFFER_SIZE],
                bytes_read: 0,
                current_byte_index: 0,
            }
        }
    }

    impl async_gcode::AsyncRead for Uart1DeviceInputStream
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