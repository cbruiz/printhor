use embedded_hal::Pwm;
use printhor_hwa_common::ControllerRef;
use crate::hwa;

pub struct PwmController<TimPeri>
where TimPeri: Pwm + 'static
{
    pwm: ControllerRef<TimPeri>,
    pwm_chan: <TimPeri as Pwm>::Channel,
    enabled: bool,
}

impl<TimPeri> PwmController<TimPeri>
    where TimPeri: Pwm<Duty=u16> + 'static,
          <TimPeri as Pwm>::Channel: Copy
{
    pub fn new(pwm: ControllerRef<TimPeri>, pwm_chan: <TimPeri as Pwm>::Channel) -> Self {
        Self {
            pwm,
            pwm_chan,
            enabled: false,
        }
    }

    #[allow(unused)]
    #[inline]
    pub async fn set_power(&mut self, power: u8)
    {
        let mut mg = self.pwm.lock().await;
        if power > 0 {
            let duty_result: Result<u16, _> = ((power as u32 * (mg.get_max_duty() as u32)) / 255u32).try_into();
            match duty_result {
                Ok(duty) => {
                    mg.set_duty(self.pwm_chan, duty as <TimPeri as Pwm>::Duty);
                    mg.enable(self.pwm_chan);
                    self.enabled = true;
                }
                _ => {
                    mg.disable(self.pwm_chan);
                    self.enabled = false;
                    hwa::error!("Unable to set power");
                }
            }
        }
        else {
            mg.disable(self.pwm_chan);
            self.enabled = false;
        }
    }

    #[inline]
    #[allow(unused)]
    pub fn is_on(&self) -> bool {
        self.enabled
    }
}
