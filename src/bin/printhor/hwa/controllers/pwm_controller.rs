//! TODO: This feature is still in incubation
use crate::hwa;
use hwa::StaticController;
use embedded_hal_02::Pwm;

/// A controller for managing PWM (Pulse-Width Modulation).
///
/// # Type Parameters
///
/// * `TimPeri` - A type that implements the `Pwm` trait and is 'static.
///
/// # Fields
///
/// * `pwm` - A reference to an interrupt controller managing the PWM peripheral.
/// * `pwm_chan` - The specific PWM channel being controlled.
pub struct PwmController<PwmMutex, TimPeri>
where
    PwmMutex: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    TimPeri: Pwm + 'static,
{
    pwm: StaticController<PwmMutex, TimPeri>,
    pwm_chan: <TimPeri as Pwm>::Channel,
}

impl<PwmMutex, TimPeri> PwmController<PwmMutex, TimPeri>
where
    PwmMutex: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    TimPeri: Pwm<Duty = u32> + 'static,
    <TimPeri as Pwm>::Channel: Copy,
{
    pub fn new(pwm: StaticController<PwmMutex, TimPeri>, pwm_chan: <TimPeri as Pwm>::Channel) -> Self {
        Self { pwm, pwm_chan }
    }

    // Sets the applied power in scale between 0 and 100
    #[allow(unused)]
    pub async fn set_power(&mut self, power: u8) {
        let mut mg = self.pwm.lock().await;
        if power > 0 {
            let max_duty = mg.get_max_duty();
            let duty_result: Result<u32, _> = (((power as u32) * (max_duty)) / 100u32).try_into();
            match duty_result {
                Ok(duty) => {
                    hwa::trace!("Set duty: {}", duty);
                    mg.set_duty(self.pwm_chan, duty.min(max_duty) as <TimPeri as Pwm>::Duty);
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
        let duty_result: Result<f32, _> = ((mg.get_duty(self.pwm_chan) as f32 * 100.0f32)
            / (mg.get_max_duty() as f32))
            .try_into();
        hwa::debug!(
            "Computing power: ({} * {}) / {} = {:?}",
            mg.get_duty(self.pwm_chan) as f32,
            100f32,
            mg.get_max_duty(),
            duty_result,
        );
        duty_result.unwrap_or(0.0f32)
    }
}
