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

/// Another controller of the same PWM channel (the PWM mutex is shared). Used by the step
/// executor to switch the tool outputs from the ISR (see `ToolOutputs`)
impl<H> Clone for GenericPwmController<H>
where
    H: hwa::SyncMutexStrategy + 'static,
    H::Resource: Pwm + 'static,
    <H::Resource as Pwm>::Channel: Copy,
    <H::Resource as Pwm>::Duty:
        core::fmt::Debug + Copy + Ord + Into<u32> + From<u16> + TryFrom<u32>,
{
    fn clone(&self) -> Self {
        Self {
            pwm: self.pwm.clone(),
            pwm_chan: self.pwm_chan,
        }
    }
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
        self.set_power_permille(u16::from(power) * 10);
    }

    /// Sets the applied power in scale between 0 and 1000 (per mille): a finer resolution, for
    /// the laser dynamic power (`M4`), updated by the step executor along the motion
    pub fn set_power_permille(&mut self, permille: u16) {
        let permille = permille.min(1000);
        self.pwm.apply_mut(|pwm| {
            if permille > 0 {
                let max_duty = pwm.get_max_duty();
                let max_duty_u32: u32 = max_duty.into();
                let duty_result: Result<u32, _> =
                    (((permille as u64) * (max_duty_u32 as u64)) / 1000u64).try_into();
                match duty_result {
                    Ok(duty) => {
                        let d: <H::Resource as Pwm>::Duty = duty.try_into().unwrap_or(max_duty);
                        hwa::trace!("Set duty: {}", Into::<u32>::into(d));
                        pwm.set_duty(self.pwm_chan, d.min(max_duty));
                        pwm.enable(self.pwm_chan);
                    }
                    _ => {
                        pwm.disable(self.pwm_chan);
                        hwa::error!("Unable to set power");
                    }
                }
            } else {
                // Duty 0 first, so that the output is driven low (heater MOSFET gate off)
                pwm.set_duty(self.pwm_chan, 0u16.into());
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
                pwm.get_max_duty().into(),
                duty_result,
            );
            duty_result.unwrap_or(0.0f32)
        })
    }
}
