//! TODO: This feature is still in incubation
use embedded_hal_02::Pwm;
use printhor_hwa_common::ControllerRef;
use crate::hwa;

pub struct PwmController<TimPeri>
where TimPeri: Pwm + 'static
{
    pwm: ControllerRef<TimPeri>,
    pwm_chan: <TimPeri as Pwm>::Channel,
}

impl<TimPeri> PwmController<TimPeri>
    where TimPeri: Pwm<Duty=u16> + 'static,
          <TimPeri as Pwm>::Channel: Copy
{
    pub fn new(pwm: ControllerRef<TimPeri>, pwm_chan: <TimPeri as Pwm>::Channel) -> Self {
        Self {
            pwm,
            pwm_chan,
        }
    }

    // Gets the applied power in scale between 0 and 100
    pub async fn set_power(&mut self, power: u8)
    {
        let mut mg = self.pwm.lock().await;
        if power > 0 {
            let duty_result: Result<u16, _> = ((power as u32 * (mg.get_max_duty() as u32)) / 100u32).try_into();
            match duty_result {
                Ok(duty) => {
                    hwa::trace!("Set duty: {}", duty);
                    mg.set_duty(self.pwm_chan, duty as <TimPeri as Pwm>::Duty);
                    mg.enable(self.pwm_chan);
                }
                _ => {
                    mg.disable(self.pwm_chan);
                    hwa::error!("Unable to set power");
                }
            }
        }
        else {
            mg.disable(self.pwm_chan);
        }
    }

    // Gets the applied power in scale between 0.0 and 1.0
    pub async fn get_power(&mut self) -> f32
    {
        let mg = self.pwm.lock().await;
        let duty_result: Result<f32, _> = ((mg.get_duty(self.pwm_chan) as f32 * 100.0f32) / (mg.get_max_duty() as f32)).try_into();
        hwa::debug!("Computing power: ({} * {}) / {} = {:?}",
            mg.get_duty(self.pwm_chan) as f32,
            100f32, mg.get_max_duty(), duty_result,
        );
        duty_result.unwrap_or(0.0f32)
    }
}
