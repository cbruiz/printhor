
#[cfg(feature = "with-serial-usb")]
compile_error!("Not supported");

#[cfg(feature = "with-serial-port-1")]
pub(crate) mod uart_port1 {
    use printhor_hwa_common as hwa;
    use hwa::HwiContract;

    pub struct UartPort1RxInputStream {
        receiver: embassy_stm32::usart::RingBufferedUartRx<'static>,
    }

    impl UartPort1RxInputStream {
        pub fn new(receiver: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>) -> Self {
            type BufferType = [u8; <crate::Contract as HwiContract>::SERIAL_PORT1_RX_BUFFER_SIZE];

            Self {
                receiver: receiver.into_ring_buffered(hwa::make_static_ref!(
                    "UartPort1RXRingBuff",
                    BufferType,
                    [0; <crate::Contract as HwiContract>::SERIAL_PORT1_RX_BUFFER_SIZE]
                )),
            }
        }
    }

    impl async_gcode::ByteStream for UartPort1RxInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            // DMA1 REQ:
            // USART_2 RX ->    CHANNEL_4, STREAM_5 [CH: I2C1_RX, DAC1]
            //                  CHANNEL_6 - STREAM_7 [CH: I2C1_TX, I2C4_TX, I2C2_TX]
            // USART_2 TX ->    CHANNEL_4 STREAM_6 [CH: I2C1_TX, TIM5_UP, DAC2]

            let mut buff: [u8; 1] = [0; 1];

            match self.receiver.read(&mut buff).await {
                Ok(_r) => Some(Ok(buff[0])),
                Err(_e) => None,
            }
        }
    }
}

#[cfg(feature = "with-serial-port-2")]
compile_error!("Not implemented");
