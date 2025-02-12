//! IO device wrappers for nucleo64-F410RB (STM32F410RB)

use printhor_hwa_common as hwa;
#[allow(unused)]
use hwa::math;
#[cfg(feature = "with-serial-usb")]
compile_error!("Not supported");

#[cfg(feature = "with-serial-port-1")]
pub(crate) mod uart_port1 {
    #[allow(unused)]
    use embedded_io_async::{BufRead, Read};
    use printhor_hwa_common as hwa;
    #[allow(unused)]
    use hwa::HwiContract;

    pub struct UartPort1RxInputStream {
        receiver: embassy_stm32::usart::RingBufferedUartRx<'static>,
    }

    impl UartPort1RxInputStream {
        pub fn new(receiver: embassy_stm32::usart::RingBufferedUartRx<'static>) -> Self {

            Self {
                receiver
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

            match self.receiver.read_exact(&mut buff[..]).await {
                Ok(_r) => {
                    Some(Ok(buff[0]))
                },
                Err(_e) => None,
            }
        }

        async fn recovery_check(&mut self) {
            hwa::warn!("Internal reset");
            // TODO
        }
    }
}

#[cfg(feature = "with-serial-port-2")]
compile_error!("Not implemented");

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

        // For PCA9685 with internal RC resonator (innacurate), according to specs,
        // PRESCALE can be computed as:
        //  PRESCALE = (25_000_000f32 - (25.0f32 * 4096.0f32)) / (50.0f32 * 4096.0f32);
        // Ideally PRESCALE IS 121.57031250, so 121 rounding
        // However, measuring with oscilloscope, prescale=126 shows a constant 20.07ms period
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
        for _servo in hwa::math::CoordSel::all_axis().iter() {
            instance.set_angle(_servo, &math::ZERO);
        }
        let cmd_timeout = embassy_time::Duration::from_millis(10);

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
        //Theoretically, should be:
        // cnt_min (1ms := -90º) = (4096 / 20) + 0.5 - 1 = 204.3
        // cnt_max (2ms := +90º) = 2 * (4095) / 20 + 0.5 -1 = 409.0
        // but your mileage may vary, because (100, ..., 300, .. 500) is the best accuracy result
        // with some analog servos (tested with TowerPro SG90 and brand-less MG90S)
        const PULSE_CENTER: hwa::math::Real = hwa::make_real!(309.0);
        const CNT_BY_ANGLE: hwa::math::Real = hwa::make_real!(2.1944444444);
        let pwm = (((*angle) * CNT_BY_ANGLE) + PULSE_CENTER).round().to_i32().unwrap()
            .max(110).min(500) as u16;
        let index = axis.index();
        if index < 16 {
            #[cfg(feature = "debug-motion-broadcast")]
            hwa::info!("[task_motion_broadcast] set PWM [{:?}] [{:?}] = {:?} -> {:?}", angle, index, self.state[index].off, pwm );
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
        let _ = self.pwm.set_all_channels(&self.state).await;
    }
}