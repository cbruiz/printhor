//! TODO: This feature is still very experimental
//! TODO: Pending to review after intense refactor
use embedded_hal::Pwm;
use printhor_hwa_common::ControllerRef;

pub struct PwmController<TimPeri>
where TimPeri: Pwm + 'static
{
    pwm: ControllerRef<TimPeri>,
    pwm_chan: <TimPeri as Pwm>::Channel,
    enabled: bool,
}

impl<TimPeri> PwmController<TimPeri>
    where TimPeri: Pwm + 'static,
          <TimPeri as Pwm>::Channel: Copy
{
    pub fn new(pwm: ControllerRef<TimPeri>, pwm_chan: <TimPeri as Pwm>::Channel) -> Self {
        Self {
            pwm,
            pwm_chan,
            enabled: false,
        }
    }

    pub async fn set_power(&self, _power: f32) {
        let mut x = self.pwm.lock().await;
        x.disable(self.pwm_chan.clone());
    }

    #[inline]
    pub fn is_on(&self) -> bool {
        self.enabled
    }
}
