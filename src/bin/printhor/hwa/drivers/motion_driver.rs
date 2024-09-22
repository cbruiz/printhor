#[cfg(all(feature = "native", feature = "plot-timings"))]
use super::timing_monitor::*;
#[allow(unused)]
#[cfg(feature = "with-probe")]
use crate::hwa::controllers::ProbeTrait;
use crate::math;
use crate::math::Real;
use crate::tgeo::{ArithmeticOps, CoordSel, TVector};
#[allow(unused)]
use crate::{hwa, hwi};
use core::ops::Neg;
use embassy_time::Duration;

use hwa::controllers::motion;
use hwa::{InterruptControllerRef, StepperChannel};

pub type MotionDriverRef = InterruptControllerRef<MotionDriver>;

#[cfg(feature = "with-motion")]
pub struct MotionDriverParams {
    pub motion_device: hwi::device::MotionDevice,
    /// Motion config is only used here to submit a copy to trinamic controller
    #[cfg(feature = "with-trinamic")]
    pub motion_config: hwa::controllers::MotionConfigRef,
    #[cfg(feature = "with-probe")]
    pub probe_controller: InterruptControllerRef<hwa::controllers::ServoController>,
    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra_1_controller: InterruptControllerRef<hwa::controllers::FanExtra1PwmController>,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_controller: InterruptControllerRef<hwa::controllers::FanLayerPwmController>,
    #[cfg(feature = "with-laser")]
    pub laser_controller: InterruptControllerRef<hwa::controllers::LaserPwmController>,
}

pub struct MotionDriver {
    #[cfg(feature = "with-motion")]
    pub pins: hwa::device::MotionPins,
    #[cfg(feature = "with-trinamic")]
    pub trinamic_controller: hwa::controllers::TrinamicController,
    #[cfg(feature = "with-probe")]
    pub probe_controller: InterruptControllerRef<hwa::controllers::ServoController>,
    #[cfg(feature = "with-fan-layer")]
    #[allow(unused)]
    pub fan_layer_controller: InterruptControllerRef<hwa::controllers::FanLayerPwmController>,
    #[cfg(feature = "with-fan-extra-1")]
    #[allow(unused)]
    pub fan_extra_1_controller: InterruptControllerRef<hwa::controllers::FanExtra1PwmController>,
    #[cfg(feature = "with-laser")]
    pub _laser_controller: InterruptControllerRef<hwa::controllers::LaserPwmController>,
    #[cfg(all(feature = "native", feature = "plot-timings"))]
    tmon: TimingsMonitor,
}

#[cfg(feature = "with-motion")]
impl MotionDriver {
    pub fn new(params: MotionDriverParams) -> Self {
        Self {
            pins: params.motion_device.motion_pins,
            #[cfg(feature = "with-trinamic")]
            trinamic_controller: hwa::controllers::TrinamicController::new(
                params.motion_device.trinamic_uart,
                params.motion_config,
            ),
            #[cfg(feature = "with-probe")]
            probe_controller: params.probe_controller,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_controller: params.fan_layer_controller,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra_1_controller: params.fan_extra_1_controller,
            #[cfg(feature = "with-laser")]
            _laser_controller: params.laser_controller,
            #[cfg(all(feature = "native", feature = "plot-timings"))]
            tmon: TimingsMonitor::new(),
        }
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn update_clock(&mut self, real_time: embassy_time::Instant) {
        self.tmon.set_clock(real_time)
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn start_segment(
        &mut self,
        ref_time: embassy_time::Instant,
        real_time: embassy_time::Instant,
    ) {
        self.tmon.reset(ref_time, real_time)
    }
    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn end_segment(&mut self) {
        self.tmon.commit()
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn mark_microsegment(&mut self) {
        self.tmon.swap(PinState::USCLK)
    }

    #[inline(always)]
    pub fn disable_steppers(&mut self, channels: StepperChannel) {
        self.pins.disable(channels);
    }

    #[inline(always)]
    pub fn enable_steppers(&mut self, channels: StepperChannel) {
        self.pins.enable(channels);
    }

    #[inline(always)]
    pub fn set_forward_direction(&mut self, channels: StepperChannel) {
        self.pins.set_forward_direction(channels);
    }

    #[allow(unused)]
    #[inline(always)]
    pub fn step_toggle(&mut self, channels: StepperChannel) {
        self.pins.step_toggle(channels);
    }

    #[allow(unused)]
    #[inline(always)]
    pub fn step_high(&mut self, channels: StepperChannel) {
        self.pins.step_high(channels);
    }

    #[allow(unused)]
    #[inline(always)]
    pub fn step_low(&mut self, channels: StepperChannel) {
        self.pins.step_low(channels);
    }

    #[inline(always)]
    pub fn endstop_triggered(&mut self, coordsel: StepperChannel) -> bool {
        self.pins.endstop_triggered(coordsel)
    }

    pub fn enable_and_set_dir(&mut self, vdir: &TVector<Real>) {
        self.enable_steppers(StepperChannel::all());

        let mut dir_fwd = StepperChannel::empty();

        #[cfg(feature = "with-x-axis")]
        if vdir.x.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(StepperChannel::X, true)
        }
        #[cfg(feature = "with-y-axis")]
        if vdir.y.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(StepperChannel::Y, true)
        }
        #[cfg(feature = "with-z-axis")]
        if vdir.z.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(StepperChannel::Z, true)
        }
        #[cfg(feature = "with-e-axis")]
        if vdir.e.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(StepperChannel::E, true)
        }
        self.set_forward_direction(dir_fwd);
    }

    
    /// This method performs the homing action for the stepper motors.
    ///
    /// The homing action involves moving the stepper motors to a known reference 
    /// position. This is typically done by moving the motors towards the end-stop 
    /// switches and then setting the current position to zero, which serves as a 
    /// reference point for future movements.
    ///
    /// # Arguments
    ///
    /// * `motion_config_ref` - A reference to the motion configuration which contains
    ///                         parameters such as units per millimeter, micro-steps 
    ///                         per axis, and machine bounds.
    ///
    /// # Returns
    ///
    /// A `Result` with the new position as `TVector<Real>`, or an error with the 
    /// homing position if the homing action fails.
    ///
    /// # Examples
    ///
    /// ```
    /// let result = controller.homing_action(&motion_config_ref).await;
    /// match result {
    ///     Ok(position) => println!("New homing position: {:?}", position),
    ///     Err(err_position) => eprintln!("Homing failed at position: {:?}", err_position),
    /// }
    /// ```
    ///
    /// # Errors
    ///
    /// This function will return an error if it is unable to complete the homing
    /// process, such as if an end-stop switch is not triggered within the expected 
    /// range of motion.
    // TODO: Carefully review
    pub async fn homing_action(
        &mut self,
        motion_config_ref: &motion::MotionConfigRef,
    ) -> Result<TVector<Real>, TVector<Real>> {

        hwa::debug!("Homing");

        let mut homming_position = TVector::zero();

        let motion_config = motion_config_ref.lock().await;
        let steps_per_mm = motion_config.units_per_mm
            * TVector::from_coords(
            Some(Real::from_lit(motion_config.micro_steps_per_axis[0].into(), 0)),
            Some(Real::from_lit(motion_config.micro_steps_per_axis[1].into(), 0)),
            Some(Real::from_lit(motion_config.micro_steps_per_axis[2].into(), 0)),
            None,
        );
        hwa::info!("Steps per mm: {}", steps_per_mm);
        let machine_bounds = motion_config.machine_bounds;
        drop(motion_config);

        hwa::info!("Machine bounds: {}", machine_bounds);

        // Raise Z axis 10mm to avoid obstacles during X and Y homing
        hwa::info!("Raise Z +10mm");
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE), None),
            Real::from_lit(10, 0),
            steps_per_mm,
            2000,
            false,
            Some(&mut homming_position),
        ).await;

        hwa::debug!("ADV: {}", homming_position);

        // Home the X axis quickly
        hwa::info!("Quick Homing X axis with a max of -{} mm", machine_bounds.x.unwrap_or(math::ZERO));
        self.shabbily_move_to(
            TVector::from_coords(Some(math::ONE.neg()), None, None, None),
            machine_bounds.x.unwrap_or(Real::zero()),
            steps_per_mm,
            2000,
            true,
            None,
        ).await;

        hwa::info!("Quick Separate X axis 5 mm");
        // Home the X axis precisely
        self.shabbily_move_to(
            TVector::from_coords(Some(math::ONE), None, None, None),
            Real::from_lit(5, 0),
            steps_per_mm,
            1000,
            false,
            Some(&mut homming_position),
        ).await;
        hwa::info!("Slow approximate X axis up to 6 mm");
        // Home the X axis precisely
        self.shabbily_move_to(
            TVector::from_coords(Some(math::ONE.neg()), None, None, None),
            Real::from_lit(6, 0),
            steps_per_mm,
            1000,
            true,
            Some(&mut homming_position),
        ).await;

        hwa::debug!("ADV: {}", homming_position);

        // Home the Y axis quickly
        hwa::info!("Quick Homing Y axis with a max of -{} mm", machine_bounds.y.unwrap_or(math::ZERO));
        self.shabbily_move_to(
            TVector::from_coords(None, Some(math::ONE.neg()), None, None),
            machine_bounds.y.unwrap_or(Real::zero()),
            steps_per_mm,
            2000,
            true,
            None,
        ).await;

        hwa::info!("Quick Separate Y axis 5 mm");
        // Home the Y axis precisely
        self.shabbily_move_to(
            TVector::from_coords(None, Some(math::ONE), None, None),
            Real::from_lit(5, 0),
            steps_per_mm,
            2000,
            false,
            Some(&mut homming_position),
        ).await;

        hwa::info!("Slow approximate Y axis up to 6 mm");
        // Home the X axis precisely
        self.shabbily_move_to(
            TVector::from_coords(None, Some(math::ONE.neg()), None, None),
            Real::from_lit(6, 0),
            steps_per_mm,
            1000,
            true,
            Some(&mut homming_position),
        ).await;

        hwa::info!("ADV: {}", homming_position);

        // TODO:
        // Go to center


        hwa::info!("Upper Z + 10mm");
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE.neg()), None),
            Real::from_lit(10, 0),
            steps_per_mm,
            1000,
            false,
            Some(&mut homming_position),
        ).await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                self.probe_controller.lock().await.probe_pin_down(100).await;
            }
        }

        // FIXME: As of now, until tested in real hardware, for safety
        let zbounds = machine_bounds.z.unwrap_or(math::ONE)  / Real::from_lit(16, 0);

        hwa::info!("Lower Z up to {} mm", zbounds); // reduced for safety
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE.neg()), None),
            zbounds,
            steps_per_mm,
            1000,
            true,
            Some(&mut homming_position),
        ).await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                self.probe_controller.lock().await.probe_pin_up(100).await;
            }
        }

        hwa::info!("Upper Z + 10mm");
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE), None),
            Real::from_lit(10, 0),
            steps_per_mm,
            2000,
            false,
            Some(&mut homming_position),
        ).await;

        Ok(homming_position)
    }

    async fn shabbily_move_to(
        &mut self,
        vdir: TVector<Real>,
        module: Real,
        steps_per_mm: TVector<Real>,
        step_frequency: u64,
        check_endstops: bool,
        position: Option<&mut TVector<Real>>,
    ) -> TVector<u32> {
        let steps_to_advance: TVector<u32> = (vdir * module * steps_per_mm)
            .abs()
            .round()
            .map_coords(|c| c.to_i32().and_then(|c| Some(c as u32)));
        let mut steps_advanced: TVector<u32> = TVector::zero();

        self.enable_and_set_dir(&vdir);

        let mut channel = StepperChannel::empty();
        let mut coord_sel = CoordSel::empty();
        steps_to_advance.apply_coords(|(coord, v)| {
            let applied = v.is_defined_positive();
            if coord.contains(CoordSel::X) {
                channel.set(StepperChannel::X, applied);
                coord_sel.set(CoordSel::X, applied);
            }
            if coord.contains(CoordSel::Y) {
                channel.set(StepperChannel::Y, applied);
                coord_sel.set(CoordSel::Y, applied);
            }
            if coord.contains(CoordSel::Z) {
                channel.set(StepperChannel::Z, applied);
                coord_sel.set(CoordSel::Z, applied);
            }
        });

        let mut ticker = embassy_time::Ticker::every(Duration::from_hz(step_frequency));

        loop {
            let mut completed = false;
            if check_endstops && self.endstop_triggered(channel) {
                hwa::debug!("ENDSTOP TRIGGERED");
                completed = true;
            }
            if !steps_advanced.bounded_by(&steps_to_advance) {
                hwa::debug!("FULL ADV");
                completed = true;
            }
            if completed {
                match position {
                    Some(mut _p) => {
                        *_p += vdir
                            * steps_advanced.map_coords(|c| Some(Real::from_lit(c.into(), 0)))
                            / steps_per_mm;
                    }
                    None => {}
                }

                return steps_advanced;
            }
            cfg_if::cfg_if! {
                if #[cfg(feature="pulsed")] {
                    compile_error!("Not yet implemented");
                    self.step_high(channel);
                    // TODO
                    //crate::control::motion_timing::s_block_for(Duration::from_micros(1));
                    self.step_low(channel);

                }
                else {
                    self.step_toggle(channel);
                }
            }
            steps_advanced.increment(coord_sel, 1);
            ticker.next().await;
        }
    }
}
