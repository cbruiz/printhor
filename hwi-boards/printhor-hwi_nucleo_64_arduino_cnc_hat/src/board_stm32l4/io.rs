//! IO device wrappers for nucleo64-L476RG (STM32L476RG)
use printhor_hwa_common as hwa;
#[allow(unused)]
use hwa::math;
#[allow(unused)]
use hwa::HwiContract;

#[cfg(feature = "with-serial-usb")]
compile_error!("Not supported");

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

    impl async_gcode::ByteStream for SerialPort1RxInputStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            use embedded_io_async::Read;

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

            hwa::info!("Creating ring_buffered uart");

            Self {
                receiver: receiver.into_ring_buffered(hwa::make_static_ref!(
                    "UartPort1RXRingBuff",
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
                Ok(_r) => Some(Ok(buff[0])),
                Err(_e) => None,
            }
        }

        async fn recovery_check(&mut self) {
            //TODO
        }
    }
}

/// ADC for the thermistors.
///
/// Conversions go through DMA (DMA1_CH1, with its own interrupt line), so the temperature task
/// yields while the ADC converts instead of busy-waiting and stalling the other tasks of the
/// executor (e.g. the homing step train).
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub mod adc {
    use printhor_hwa_common as hwa;

    /// Long sample time: thermistor inputs are high impedance (pull-up + RC filter)
    const ADC1_SAMPLE_TIME: embassy_stm32::adc::SampleTime =
        embassy_stm32::adc::SampleTime::CYCLES247_5;

    embassy_stm32::bind_interrupts!(struct AdcDmaIrqs {
        DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH1>;
    });

    pub struct AdcWrapper<T>
    where
        T: embassy_stm32::adc::Instance<Regs = embassy_stm32::pac::adc::Adc>,
    {
        adc: embassy_stm32::adc::Adc<'static, T>,
        dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA1_CH1>,
    }

    impl<T> AdcWrapper<T>
    where
        T: embassy_stm32::adc::Instance<Regs = embassy_stm32::pac::adc::Adc>,
    {
        pub fn new(
            adc_peripheral: embassy_stm32::Peri<'static, T>,
            dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA1_CH1>,
        ) -> Self {
            let mut config = embassy_stm32::adc::AdcConfig::default();
            config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
            Self {
                adc: embassy_stm32::adc::Adc::new_with_config(adc_peripheral, config),
                dma,
            }
        }
    }

    impl<T> hwa::traits::UnifiedAdc16 for AdcWrapper<T>
    where
        T: embassy_stm32::adc::Instance<Regs = embassy_stm32::pac::adc::Adc>,
        embassy_stm32::peripherals::DMA1_CH1: embassy_stm32::adc::RxDma<T>,
    {
        type VRefPin = ();
        type SamplePin = embassy_stm32::adc::AnyAdcChannel<'static, T>;

        fn read_adc(&mut self, pin: &mut Self::SamplePin) -> impl core::future::Future<Output = u16> {
            async move {
                let mut reading = [0u16; 1];
                self.adc
                    .read(
                        self.dma.reborrow(),
                        AdcDmaIrqs,
                        [(pin, ADC1_SAMPLE_TIME)].into_iter(),
                        &mut reading,
                    )
                    .await;
                reading[0]
            }
        }
    }
}

#[cfg(feature = "with-i2c")]
pub struct MotionI2c {
    pwm: pwm_pca9685::Pca9685<embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>>,
    state: [pwm_pca9685::ChannelOnOffControl; 16],
}

#[cfg(feature = "with-i2c")]
impl MotionI2c {
    pub async fn new(pwm: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>) -> Self {

        let address = pwm_pca9685::Address::from((false, false, false, false, false, false));
        let pwm = pwm_pca9685::Pca9685::new(pwm, address).unwrap();

        const PRESCALE:u8 = 126;
        hwa::info!("[MotionI2c] I2C Prescale: {}", PRESCALE);

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
        hwa::info!("[MotionI2c] I2C: Proyecting {:?}", world_center);
        let space_center = crate::Contract.project_to_space(&world_center).unwrap();
        hwa::info!("[MotionI2c] I2C: Got {:?}", space_center);
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
            hwa::warn!("[MotionI2c] Timeout initializing I2C");
        }
        instance
    }

    pub fn set_angle(&mut self, axis: hwa::math::CoordSel, angle: &hwa::math::Real) -> bool {

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion-anthropomorphic-kinematics")] {
                let calibrated_angle = (
                    (*angle)
                    + crate::board_stm32l4::ANTHROPOMORFIC_SPACE_CALIBRATION.get_coord(axis).unwrap_or(math::ZERO)
                ) * crate::board_stm32l4::ANTHROPOMORFIC_SPACE_DIR.get_coord(axis).unwrap_or(math::ZERO);
            }
            else {
                let calibrated_angle = *angle;
            }
        }

        //Theoretically, should be:
        // cnt_min (1ms := -90º) = (4096 / 20) + 0.5 - 1 = 204.3
        // cnt_max (2ms := +90º) = 2 * (4095) / 20 + 0.5 -1 = 409.0
        // but your mileage may vary, because (100, ..., 300, .. 500) is the best accuracy result
        // with some analog servos (tested with TowerPro SG90 and brand-less MG90S)
        const PULSE_CENTER: hwa::math::Real = hwa::make_real!(309.0);
        const CNT_BY_ANGLE: hwa::math::Real = hwa::make_real!(2.1944444444);
        // Saturating: an out of range or NaN angle never panics (NaN is 0, then clamped)
        let pwm = ((calibrated_angle * CNT_BY_ANGLE) + PULSE_CENTER).round().int()
            .clamp(110, 500) as u16;
        let index = axis.index();

        if index < 16 {
            #[cfg(feature = "debug-motion-broadcast")]
            hwa::info!("[MotionI2c] set PWM [{:?}, {:?}] angle: ({:?})->{:?} : {:?} -> {:?}", axis, index, angle, calibrated_angle, self.state[index].off, pwm );
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
        hwa::info!("[MotionI2c] I2C update took {:?} us", t0.elapsed().as_micros());
    }

    pub async fn reset(&mut self) {
    }
}