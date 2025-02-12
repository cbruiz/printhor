//! IO device wrappers for nucleo64-L476RG (STM32L476RG)
use printhor_hwa_common as hwa;
#[allow(unused)]
use hwa::math;
use hwa::HwiContract;

#[cfg(feature = "with-serial-usb")]
compile_error!("Not supported");

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-serial-port-1", not(feature = "uart-uses-ring-buffer")))] {
        pub mod uart_port1 {

            compile_error!("not proper");

            pub struct UartPort1RxInputStream {
                receiver: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>,
            }

            impl UartPort1RxInputStream {
                pub fn new(receiver: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>) -> Self {

                    Self {
                        receiver,
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
                    let mut buff = [0u8];

                    printhor_hwa_common::info!("serial read");

                    match self.receiver.read_until_idle(&mut buff).await {
                        Ok(_r) => Some(Ok(buff[0])),
                        Err(_e) => None,
                    }
                }
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-serial-port-1", feature = "uart-uses-ring-buffer"))] {
        pub mod uart_port1 {
            use printhor_hwa_common as hwa;
            use hwa::HwiContract;

            pub struct UartPort1RxInputStream {
                receiver: embassy_stm32::usart::RingBufferedUartRx<'static>,
            }

            impl UartPort1RxInputStream {
                pub fn new(receiver: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>) -> Self {
                    type BufferType = [u8; <crate::Contract as HwiContract>::SERIAL_PORT1_RX_BUFFER_SIZE];

                    hwa::info!("Creating ring_buffered uart");

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
                    use embedded_io_async::Read;
                    // DMA1 REQ:
                    // USART_2 RX ->    CHANNEL_4, STREAM_5 [CH: I2C1_RX, DAC1]
                    //                  CHANNEL_6 - STREAM_7 [CH: I2C1_TX, I2C4_TX, I2C2_TX]
                    // USART_2 TX ->    CHANNEL_4 STREAM_6 [CH: I2C1_TX, TIM5_UP, DAC2]

                    let mut buff: [u8; 1] = [0; 1];
                    match self.receiver.read_exact(&mut buff).await {
                        Ok(_r) => Some(Ok(buff[0])),
                        Err(_e) => None,
                    }
                }

                async fn recovery_check(&mut self) {
                    //TODO
                }
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

#[cfg(feature = "with-i2c")]
pub struct MotionI2c {
    pwm: pwm_pca9685::Pca9685<embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async>>,
    state: [pwm_pca9685::ChannelOnOffControl; 16],
}

#[cfg(feature = "with-i2c")]
impl MotionI2c {
    pub async fn new(pwm: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async>) -> Self {

        let address = pwm_pca9685::Address::from((false, false, false, false, false, false));
        let pwm = pwm_pca9685::Pca9685::new(pwm, address).unwrap();

        const PRESCALE:u8 = 126;
        hwa::info!("I2C Prescale: {}", PRESCALE);

        let mut instance = Self {
            pwm,
            state: [pwm_pca9685::ChannelOnOffControl{
                on: 0,
                off: 0,
                full_on: false,
                full_off: false,
            }; 16],
        };

        let world_center = crate::Contract::DEFAULT_WORLD_HOMING_POINT_WU;
        hwa::info!("I2C: Proyecting {:?}", world_center);
        let space_center = crate::Contract.project_to_space(&world_center).unwrap();
        hwa::info!("I2C: Got {:?}", space_center);
        space_center.foreach_values(|_c, _v| {
            instance.set_angle(_c, _v);
        });
        let cmd_timeout = embassy_time::Duration::from_millis(500);

        let got_timeout =
            if embassy_time::with_timeout(cmd_timeout, instance.pwm.disable()).await.is_err() {
                true
            }
            else {
                if embassy_time::with_timeout(cmd_timeout, instance.pwm.set_prescale(PRESCALE)).await.is_err() {
                    true
                }
                else {
                    if embassy_time::with_timeout(cmd_timeout, instance.pwm.enable()).await.is_err() {
                        true
                    }
                    else {
                        if embassy_time::with_timeout(cmd_timeout, instance.apply()).await.is_err() {
                            true
                        }
                        else {
                            false
                        }
                    }
                }
            };
        if got_timeout {
            hwa::warn!("Timeout initializing I2C");
        }
        instance
    }

    pub fn set_angle(&mut self, axis: hwa::math::CoordSel, angle: &hwa::math::Real) -> bool {

        let calibrated_angle = (
            (*angle)
            + crate::board_stm32l4::ANTHROPOMORFIC_SPACE_CALIBRATION.get_coord(axis).unwrap_or(math::ZERO)
        ) * crate::board_stm32l4::ANTHROPOMORFIC_SPACE_DIR.get_coord(axis).unwrap_or(math::ZERO);


        //Theoretically, should be:
        // cnt_min (1ms := -90º) = (4096 / 20) + 0.5 - 1 = 204.3
        // cnt_max (2ms := +90º) = 2 * (4095) / 20 + 0.5 -1 = 409.0
        // but your mileage may vary, because (100, ..., 300, .. 500) is the best accuracy result
        // with some analog servos (tested with TowerPro SG90 and brand-less MG90S)
        const PULSE_CENTER: hwa::math::Real = hwa::make_real!(309.0);
        const CNT_BY_ANGLE: hwa::math::Real = hwa::make_real!(2.1944444444);
        let pwm = ((calibrated_angle * CNT_BY_ANGLE) + PULSE_CENTER).round().to_i32().unwrap()
            .max(110).min(500) as u16;
        let index = axis.index();

        if index < 16 {
            #[cfg(feature = "debug-motion-broadcast")]
            hwa::info!("[task_motion_broadcast] set PWM [{:?}, {:?}] angle: ({:?})->{:?} : {:?} -> {:?}", axis, index, angle, calibrated_angle, self.state[index].off, pwm );
            if self.state[index].off != pwm {
                self.state[index].off = pwm;
                true
            }
            else {
                false
            }
        }
        else {
            false
        }
    }

    pub async fn apply(&mut self) {
        #[cfg(feature = "verbose-timings")]
        let t0 = embassy_time::Instant::now();
        let _ = self.pwm.set_all_channels(&self.state).await;
        #[cfg(feature = "verbose-timings")]
        hwa::info!("I2C update took {:?} us", t0.elapsed().as_micros());
    }

    pub async fn reset(&mut self) {
    }
}