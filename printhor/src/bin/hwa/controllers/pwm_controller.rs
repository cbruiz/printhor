//! TODO: This feature is still in incubation

use crate::hwa;
use embedded_hal_02::Pwm;
use hwa::MutexStrategy;
use hwa::StaticAsyncController;

/// A controller for managing PWM (Pulse-Width Modulation).
///
/// # Type Parameters
///
/// * `H` - A type that implements the `MutexStrategy` trait and is 'static.
///
/// # Fields
///
/// * `pwm` - A reference to an interrupt controller managing the PWM peripheral.
/// * `pwm_chan` - The specific PWM channel being controlled.
pub struct PwmController<H>
where
    H: MutexStrategy + Send + 'static,
    H::Resource: Pwm + Send + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty:
        core::fmt::Debug + Copy + core::cmp::Ord + Into<u32> + From<u16> + TryFrom<u32>,
{
    pwm: StaticAsyncController<H>,
    pwm_chan: <H::Resource as Pwm>::Channel,
}

impl<H> PwmController<H>
where
    H: MutexStrategy + Send + 'static,
    H::Resource: Pwm + Send + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty:
        core::fmt::Debug + Copy + core::cmp::Ord + Into<u32> + From<u16> + TryFrom<u32>,
{
    pub fn new(pwm: StaticAsyncController<H>, pwm_chan: <H::Resource as Pwm>::Channel) -> Self {
        Self { pwm, pwm_chan }
    }

    // Sets the applied power in scale between 0 and 100
    pub async fn set_power(&mut self, power: u8) {
        let mut mg = self.pwm.lock().await;
        if power > 0 {
            let max_duty = mg.get_max_duty();
            let duty_result: Result<u32, _> =
                (((power as u32) * max_duty.into()) / 100u32).try_into();
            match duty_result {
                Ok(duty) => {
                    let d: <H::Resource as Pwm>::Duty = duty.try_into().unwrap_or(max_duty);
                    hwa::trace!("Set duty: {:?}", d);
                    mg.set_duty(self.pwm_chan, d.min(max_duty));
                    mg.enable(self.pwm_chan);
                }
                _ => {
                    mg.disable(self.pwm_chan);
                    hwa::error!("Unable to set power");
                }
            }
        } else {
            mg.disable(self.pwm_chan);
        }
    }

    // Gets the applied power in scale between 0.0 and 1.0
    #[allow(unused)]
    pub async fn get_power(&mut self) -> f32 {
        let mg = self.pwm.lock().await;
        let d: u32 = mg.get_duty(self.pwm_chan).into();
        let duty_result: Result<f32, _> =
            ((d as f32 * 100.0f32) / (mg.get_max_duty().into() as f32)).try_into();
        hwa::debug!(
            "Computing power: ({} * {}) / {:?} = {:?}",
            mg.get_duty(self.pwm_chan).into() as f32,
            100f32,
            mg.get_max_duty(),
            duty_result,
        );
        duty_result.unwrap_or(0.0f32)
    }
}
