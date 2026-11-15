//! Tool actions: the output changes requested by `M106`/`M107` (fans), `M3`/`M4`/`M5` (laser or
//! spindle) and `M7`/`M8`/`M9` (coolant), queued with the motion.
//!
//! A tool action takes effect at the point of the motion queue where it was issued: right after
//! the motion queued before it, and right before the motion queued after it. It never takes a
//! slot of the motion queue: it is attached to the newest queued entry (see
//! `RingBuffer::post_tools`), so it neither shortens the look-ahead nor breaks the motion
//! chaining. When that entry has been executed, the step executor (ISR) applies the actions
//! between its last step window and the first window of the next motion (see
//! [super::StepPlan::marker]): no time is added between both, and the switch happens within one
//! tick of the motion reaching the junction.
#[allow(unused_imports)]
use crate::hwa;

/// Laser or spindle state (`M3`/`M4`/`M5`). The power is a percentage, `[0, 100]`
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum CutterState {
    /// `M5`
    Off,
    /// `M3`: spindle on, clockwise. Laser on at constant power (as GRBL laser mode): off during
    /// the rapids (`G0`) and the homing, back at the programmed power with the next cutting move.
    /// Kept while stopped (a dwell burns a spot)
    Clockwise(u8),
    /// `M4`: spindle on, counter-clockwise. Laser on in dynamic power mode (as GRBL and Marlin):
    /// along the motion, the power is scaled by the speed relative to the programmed one (the
    /// `F` of each move), so that the energy per unit of length stays the same while the motion
    /// accelerates, brakes or turns a corner. Off while stopped, during the rapids (`G0`) and the
    /// homing (see [dynamic_power_permille])
    CounterClockwise(u8),
}

/// The power (per mille) of the laser in dynamic mode (`M4` at `power` percent) over a step
/// window whose speed is `speed_ratio` per mille of the programmed one: `power · speed_ratio`,
/// capped at full power (the ratio exceeds 1 with an M220 speed rate above 100%). Off without a
/// programmed speed (`None`: rapids, homing, no motion)
pub const fn dynamic_power_permille(power: u8, speed_ratio: Option<u16>) -> u16 {
    match speed_ratio {
        None => 0,
        Some(ratio) => {
            let p = power as u32 * ratio as u32 / 100;
            if p > 1000 { 1000 } else { p as u16 }
        }
    }
}

impl CutterState {
    /// The power applied, `[0, 100]`
    pub const fn power(&self) -> u8 {
        match self {
            CutterState::Off => 0,
            CutterState::Clockwise(p) | CutterState::CounterClockwise(p) => *p,
        }
    }

    /// Whether the cutter is on (with some power)
    pub const fn is_on(&self) -> bool {
        self.power() > 0
    }
}

/// The output changes requested at the same point of the motion queue. Each field is the new
/// state of an output, `None` when unchanged. Since they apply at the same instant, only the
/// last request of each output matters (see [ToolActions::merge]).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub struct ToolActions {
    /// Layer fan power `[0, 100]` (`M106 P0`, `M107`)
    pub fan_layer: Option<u8>,
    /// Extra fan power `[0, 100]` (`M106 P1`)
    pub fan_extra1: Option<u8>,
    /// Laser or spindle (`M3`, `M4`, `M5`)
    pub cutter: Option<CutterState>,
    /// Mist coolant (`M7` on, `M9` off)
    pub coolant_mist: Option<bool>,
    /// Flood coolant (`M8` on, `M9` off)
    pub coolant_flood: Option<bool>,
}

impl ToolActions {
    /// No change
    pub const NONE: Self = Self {
        fan_layer: None,
        fan_extra1: None,
        cutter: None,
        coolant_mist: None,
        coolant_flood: None,
    };

    /// Everything that may hurt or burn off: laser/spindle and coolant (emergency stop)
    pub const CUTTER_AND_COOLANT_OFF: Self = Self {
        fan_layer: None,
        fan_extra1: None,
        cutter: Some(CutterState::Off),
        coolant_mist: Some(false),
        coolant_flood: Some(false),
    };

    /// Whether nothing changes
    pub const fn is_empty(&self) -> bool {
        self.fan_layer.is_none()
            && self.fan_extra1.is_none()
            && self.cutter.is_none()
            && self.coolant_mist.is_none()
            && self.coolant_flood.is_none()
    }

    /// Adds the `newer` changes, requested after these ones at the same point of the queue: per
    /// output, the newest request wins
    pub fn merge(&mut self, newer: &ToolActions) {
        if newer.fan_layer.is_some() {
            self.fan_layer = newer.fan_layer;
        }
        if newer.fan_extra1.is_some() {
            self.fan_extra1 = newer.fan_extra1;
        }
        if newer.cutter.is_some() {
            self.cutter = newer.cutter;
        }
        if newer.coolant_mist.is_some() {
            self.coolant_mist = newer.coolant_mist;
        }
        if newer.coolant_flood.is_some() {
            self.coolant_flood = newer.coolant_flood;
        }
    }

    /// The changes allowed in dry run (`M37 S1`): the motion is virtual, so the laser/spindle is
    /// never turned on (a laser on a still head burns a single spot). Turning it off is kept
    pub fn for_dry_run(mut self) -> Self {
        if self.cutter.is_some_and(|c| c.is_on()) {
            self.cutter = None;
        }
        self
    }
}

/// The outputs the step executor drives (see `SoftTimer::setup_tools`). Each one shares the
/// PWM of its controller: the PWM mutex of a board must be a critical section one
/// (`SyncCsMutexType`) for any output listed here, as the step executor runs in the ISR.
pub struct ToolOutputs {
    #[cfg(feature = "with-fan-layer")]
    fan_layer: hwa::types::InnerFanLayerController,
    #[cfg(feature = "with-fan-extra-1")]
    fan_extra1: hwa::types::InnerFanExtra1Controller,
    #[cfg(feature = "with-laser")]
    laser: hwa::types::InnerLaserController,
}

impl ToolOutputs {
    pub const fn new(
        #[cfg(feature = "with-fan-layer")] fan_layer: hwa::types::InnerFanLayerController,
        #[cfg(feature = "with-fan-extra-1")] fan_extra1: hwa::types::InnerFanExtra1Controller,
        #[cfg(feature = "with-laser")] laser: hwa::types::InnerLaserController,
    ) -> Self {
        Self {
            #[cfg(feature = "with-fan-layer")]
            fan_layer,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra1,
            #[cfg(feature = "with-laser")]
            laser,
        }
    }

    /// Applies the changes to the outputs. Without a coolant output, `M7`/`M8`/`M9` only change
    /// the tool state (see `SoftTimer::applied_tools`)
    pub fn apply(&mut self, _actions: &ToolActions) {
        #[cfg(feature = "with-fan-layer")]
        if let Some(power) = _actions.fan_layer {
            self.fan_layer.set_power(power);
        }
        #[cfg(feature = "with-fan-extra-1")]
        if let Some(power) = _actions.fan_extra1 {
            // The manual setting the hot-end auto fan restores once cold (see `task_temperature`)
            crate::processing::FAN_EXTRA1_MANUAL_POWER
                .store(power, core::sync::atomic::Ordering::Relaxed);
            self.fan_extra1.set_power(power);
        }
        #[cfg(feature = "with-laser")]
        if let Some(cutter) = _actions.cutter {
            match cutter {
                // Dynamic power: off until the motion sets it (see `set_laser_permille`)
                CutterState::CounterClockwise(_) => self.laser.set_power(0),
                _ => self.laser.set_power(cutter.power()),
            }
        }
    }

    /// The laser power (per mille) along the motion (`M3` rapids off, `M4` dynamic power), set by
    /// the step executor at the start of every step window
    #[allow(unused)]
    pub fn set_laser_permille(&mut self, _permille: u16) {
        #[cfg(feature = "with-laser")]
        self.laser.set_power_permille(_permille);
    }
}

/// Test instrumentation of the native simulator: the tool actions applied by the step executor,
/// with the step position (see [super::step_trace]) of the motors at that instant
#[cfg(all(test, feature = "native"))]
pub mod tool_trace {
    use super::ToolActions;
    use crate::hwa::controllers::step_trace;
    use std::sync::Mutex;

    static LOG: Mutex<Option<std::vec::Vec<(ToolActions, step_trace::Steps)>>> = Mutex::new(None);

    /// Starts recording
    pub fn start() {
        *LOG.lock().unwrap() = Some(std::vec::Vec::new());
    }

    /// Stops recording and returns the actions applied since [start], in order
    pub fn stop() -> std::vec::Vec<(ToolActions, step_trace::Steps)> {
        LOG.lock().unwrap().take().unwrap_or_default()
    }

    pub(crate) fn record(actions: &ToolActions) {
        if let Some(log) = LOG.lock().unwrap().as_mut() {
            log.push((*actions, step_trace::current()));
        }
    }

    static LASER: Mutex<Option<std::vec::Vec<(u16, step_trace::Steps)>>> = Mutex::new(None);

    /// Starts recording the laser dynamic power (`M4`)
    pub fn start_laser() {
        *LASER.lock().unwrap() = Some(std::vec::Vec::new());
    }

    /// Stops recording and returns every change of the laser dynamic power (per mille) since
    /// [start_laser], with the step position where it was applied
    pub fn stop_laser() -> std::vec::Vec<(u16, step_trace::Steps)> {
        LASER.lock().unwrap().take().unwrap_or_default()
    }

    pub(crate) fn record_laser(permille: u16) {
        if let Some(log) = LASER.lock().unwrap().as_mut() {
            log.push((permille, step_trace::current()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_set_changes_nothing() {
        assert!(ToolActions::NONE.is_empty());
        assert!(ToolActions::default().is_empty());
        assert!(!ToolActions::CUTTER_AND_COOLANT_OFF.is_empty());
        let fan = ToolActions { fan_layer: Some(0), ..ToolActions::NONE };
        assert!(!fan.is_empty());
    }

    /// Requests at the same point of the queue apply at the same instant: the newest one of each
    /// output wins, the other outputs keep their own request
    #[test]
    fn the_newest_request_of_each_output_wins() {
        let mut a = ToolActions {
            fan_layer: Some(100),
            cutter: Some(CutterState::Clockwise(40)),
            coolant_mist: Some(true),
            ..ToolActions::NONE
        };
        a.merge(&ToolActions {
            fan_layer: Some(0),
            coolant_flood: Some(true),
            ..ToolActions::NONE
        });
        a.merge(&ToolActions::NONE);
        assert_eq!(a.fan_layer, Some(0));
        assert_eq!(a.fan_extra1, None);
        assert_eq!(a.cutter, Some(CutterState::Clockwise(40)));
        assert_eq!(a.coolant_mist, Some(true));
        assert_eq!(a.coolant_flood, Some(true));
        a.merge(&ToolActions { cutter: Some(CutterState::Off), ..ToolActions::NONE });
        assert_eq!(a.cutter, Some(CutterState::Off));
    }

    #[test]
    fn the_cutter_power() {
        assert_eq!(CutterState::Off.power(), 0);
        assert!(!CutterState::Off.is_on());
        assert_eq!(CutterState::Clockwise(30).power(), 30);
        assert_eq!(CutterState::CounterClockwise(70).power(), 70);
        assert!(CutterState::CounterClockwise(70).is_on());
        assert!(!CutterState::Clockwise(0).is_on());
    }

    /// Dynamic power: the power scaled by the speed ratio, capped at full power, off without a
    /// programmed speed
    #[test]
    fn the_dynamic_power_follows_the_speed() {
        assert_eq!(dynamic_power_permille(100, Some(1000)), 1000);
        assert_eq!(dynamic_power_permille(100, Some(500)), 500);
        assert_eq!(dynamic_power_permille(40, Some(1000)), 400);
        assert_eq!(dynamic_power_permille(40, Some(250)), 100);
        assert_eq!(dynamic_power_permille(80, Some(0)), 0);
        // M220 above 100%: faster than programmed, more power, at most full power
        assert_eq!(dynamic_power_permille(50, Some(1500)), 750);
        assert_eq!(dynamic_power_permille(100, Some(1500)), 1000);
        assert_eq!(dynamic_power_permille(100, Some(u16::MAX)), 1000);
        assert_eq!(dynamic_power_permille(100, None), 0);
    }

    /// In dry run the laser/spindle is never turned on, but it is turned off
    #[test]
    fn dry_run_never_turns_the_cutter_on() {
        let on = ToolActions {
            fan_layer: Some(50),
            cutter: Some(CutterState::Clockwise(80)),
            ..ToolActions::NONE
        };
        let dry = on.for_dry_run();
        assert_eq!(dry.cutter, None);
        assert_eq!(dry.fan_layer, Some(50));
        let off = ToolActions { cutter: Some(CutterState::Off), ..ToolActions::NONE };
        assert_eq!(off.for_dry_run(), off);
    }
}
