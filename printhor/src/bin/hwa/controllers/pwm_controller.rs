//! TODO: This feature is still in incubation

use crate::hwa;
use hwa::traits::Pwm;

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
pub struct GenericPwmController<H>
where
    H: hwa::SyncMutexStrategy + 'static,
    H::Resource: Pwm + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty:
        core::fmt::Debug + Copy + Ord + Into<u32> + From<u16> + TryFrom<u32>,
{
    pwm: hwa::StaticSyncController<H>,
    pwm_chan: <H::Resource as Pwm>::Channel,
}

impl<H> GenericPwmController<H>
where
    H: hwa::SyncMutexStrategy + 'static,
    H::Resource: Pwm + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty:
        core::fmt::Debug + Copy + Ord + Into<u32> + From<u16> + TryFrom<u32>,
{
    pub fn new(pwm: hwa::StaticSyncController<H>, pwm_chan: <H::Resource as Pwm>::Channel) -> Self {
        Self { pwm, pwm_chan }
    }

    // Sets the applied power in scale between 0 and 100
    pub fn set_power(&mut self, power: u8) {
        self.pwm.apply_mut(|pwm| {
            if power > 0 {
                let max_duty = pwm.get_max_duty();
                let duty_result: Result<u32, _> =
                    (((power as u32) * max_duty.into()) / 100u32).try_into();
                match duty_result {
                    Ok(duty) => {
                        let d: <H::Resource as Pwm>::Duty = duty.try_into().unwrap_or(max_duty);
                        hwa::trace!("Set duty: {:?}", d);
                        pwm.set_duty(self.pwm_chan, d.min(max_duty));
                        pwm.enable(self.pwm_chan);
                    }
                    _ => {
                        pwm.disable(self.pwm_chan);
                        hwa::error!("Unable to set power");
                    }
                }
            } else {
                pwm.disable(self.pwm_chan);
            }
        });
    }

    // Gets the applied power in scale between 0.0 and 1.0
    pub fn get_power(&mut self) -> f32 {
        self.pwm.apply(|pwm| {
            let d: u32 = pwm.get_duty(self.pwm_chan).into();
            let duty_result: Result<f32, _> =
                ((d as f32 * 100.0f32) / (pwm.get_max_duty().into() as f32)).try_into();
            hwa::debug!(
                "Computing power: ({} * {}) / {:?} = {:?}",
                pwm.get_duty(self.pwm_chan).into() as f32,
                100f32,
                pwm.get_max_duty(),
                duty_result,
            );
            duty_result.unwrap_or(0.0f32)
        })
    }
}
