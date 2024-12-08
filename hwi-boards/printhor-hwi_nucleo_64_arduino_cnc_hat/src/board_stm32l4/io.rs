//! IO device wrappers for nucleo64-L476RG (STM32L476RG)
#[cfg(feature = "with-serial-usb")]
compile_error!("Not supported");

#[cfg(feature = "with-serial-port-1")]
pub mod uart_port1 {
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

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub mod adc {

    // Hard-coded ADC1 sample time (6.5 cycles): Around 1us
    const ADC1_SAMPLE_TIME:embassy_stm32::adc::SampleTime = embassy_stm32::adc::SampleTime::CYCLES6_5;

    use printhor_hwa_common as hwa;

    pub struct AdcWrapper<T, RxDma>
    where
        T: embassy_stm32::adc::Instance,
        RxDma: embassy_stm32::adc::RxDma<T>,
    {
        adc: embassy_stm32::adc::Adc<'static, T>,
        dma: RxDma
    }

    impl<T, RxDma> AdcWrapper<T, RxDma>
    where
        T: embassy_stm32::adc::Instance,
        RxDma: embassy_stm32::adc::RxDma<T>,
    {
        pub fn new(adc_peripheral: T, dma: RxDma) -> Self {
            let mut adc = embassy_stm32::adc::Adc::new(adc_peripheral);
            adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
            adc.set_sample_time(ADC1_SAMPLE_TIME);
            Self {
                adc,
                dma,
            }
        }
    }

    impl<T, RxDma> hwa::traits::UnifiedAdc16 for AdcWrapper<T, RxDma>
    where
        T: embassy_stm32::adc::Instance,
        RxDma: embassy_stm32::adc::RxDma<T>,
    {
        type VRefPin = ();
        type SamplePin = embassy_stm32::adc::AnyAdcChannel<T>;

        fn read_adc(&mut self, _pin: &mut Self::SamplePin) -> impl core::future::Future<Output=u16> {
            async {
                let sequence = [(_pin, ADC1_SAMPLE_TIME)];
                let mut readings = [0u16];
                self.adc.read(
                    &mut self.dma,
                    sequence.into_iter(),
                    &mut readings
                ).await;
                readings[0]
            }
        }
    }
}