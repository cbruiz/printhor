use embassy_time::Duration;
#[allow(unused)]
use crate::{hwa, hwi};
use crate::control::motion_timing::s_block_for;
#[allow(unused)]
use crate::hwa::ControllerRef;
use crate::hwa::controllers::{ MotionConfigRef};
#[allow(unused)]
#[cfg(feature = "with-probe")]
use crate::hwa::controllers::ProbeTrait;
use crate::math::{Real, ONE};
use crate::tgeo::{ArithmeticOps, CoordSel, TVector};
#[cfg(all(feature = "native", feature = "plot-timings"))]
use super::timing_monitor::*;
use core::ops::Neg;

#[cfg(feature = "with-motion")]
pub struct MotionDriverParams {
    pub motion_device: hwi::device::MotionDevice,
    pub motion_config: hwa::controllers::MotionConfigRef,
    #[cfg(feature = "with-probe")]
    pub probe_controller: ControllerRef<hwa::controllers::ServoController>,
    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra_1_controller: ControllerRef<hwa::controllers::FanExtra1PwmController>,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_controller: ControllerRef<hwa::controllers::FanLayerPwmController>,
    #[cfg(feature = "with-laser")]
    pub laser_controller: ControllerRef<hwa::controllers::LaserPwmController>,
}

pub struct MotionDriver {

    #[cfg(feature = "with-motion")]
    pub pins: hwi::device::MotionPins,
    #[cfg(feature = "with-trinamic")]
    pub trinamic_controller: hwa::controllers::TrinamicController,
    #[cfg(feature = "with-probe")]
    pub probe_controller: ControllerRef<hwa::controllers::ServoController>,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_controller: ControllerRef<hwa::controllers::FanLayerPwmController>,
    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra_1_controller: ControllerRef<hwa::controllers::FanExtra1PwmController>,
    #[cfg(feature = "with-laser")]
    pub laser_controller: ControllerRef<hwa::controllers::LaserPwmController>,
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
            laser_controller: params.laser_controller,
            #[cfg(all(feature = "native", feature = "plot-timings"))]
            tmon: TimingsMonitor::new(),
        }
    }

    #[cfg(all(feature = "native", feature = "plot-timings", feature = "no-real-time"))]
    #[inline]
    pub fn update_clock(&mut self, real_time: embassy_time::Instant) {
        self.tmon.set_clock(real_time)
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn start_segment(&mut self, ref_time: embassy_time::Instant, real_time: embassy_time::Instant) {
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
    pub fn enable_x_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::X_ENA, false);
        self.pins.enable_x_stepper()
    }

    #[allow(unused)]
    #[inline(always)]
    pub fn disable_x_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::X_ENA, true);
        self.pins.disable_x_stepper()
    }

    #[inline(always)]
    pub fn enable_y_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Y_ENA, false);
        self.pins.enable_y_stepper()
    }

    #[allow(unused)]
    #[inline(always)]
    pub fn disable_y_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Y_ENA, true);
        self.pins.disable_y_stepper()
    }

    #[inline(always)]
    pub fn enable_z_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Z_ENA, false);
        self.pins.enable_z_stepper()
    }

    #[allow(unused)]
    #[inline(always)]
    pub fn disable_z_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Z_ENA, true);
        self.pins.disable_z_stepper()
    }

    #[cfg(feature = "with-hot-end")]
    #[inline(always)]
    pub fn enable_e_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::E_ENA, false);
        self.pins.enable_e_stepper()
    }


    #[cfg(feature = "with-hot-end")]
    #[allow(unused)]
    #[inline(always)]
    pub fn disable_e_stepper(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::E_ENA, true);
        self.pins.disable_e_stepper()
    }

    #[inline(always)]
    pub fn x_dir_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::X_DIR, true);
        self.pins.x_dir_pin.set_high();
    }

    #[inline(always)]
    pub fn y_dir_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Y_DIR, true);
        self.pins.y_dir_pin.set_high();
    }

    #[inline(always)]
    pub fn z_dir_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Z_DIR, true);
        self.pins.z_dir_pin.set_high();
    }

    #[cfg(feature = "with-hot-end")]
    #[inline(always)]
    pub fn e_dir_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::E_DIR, true);
        self.pins.e_dir_pin.set_high();
    }

    #[inline(always)]
    pub fn x_dir_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::X_DIR, false);
        self.pins.x_dir_pin.set_low();
    }

    #[inline(always)]
    pub fn y_dir_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Y_DIR, false);
        self.pins.y_dir_pin.set_low();
    }

    #[inline(always)]
    pub fn z_dir_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Z_DIR, false);
        self.pins.z_dir_pin.set_low();
    }

    #[cfg(feature = "with-hot-end")]
    #[inline(always)]
    pub fn e_dir_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::E_DIR, false);
        self.pins.e_dir_pin.set_low();
    }

    #[inline(always)]
    pub fn x_step_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::X_STEP, true);
        self.pins.x_step_pin.set_high();
    }

    #[inline(always)]
    pub fn y_step_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Y_STEP, true);
        self.pins.y_step_pin.set_high();
    }

    #[inline(always)]
    pub fn z_step_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Z_STEP, true);
        self.pins.z_step_pin.set_high();
    }

    #[cfg(feature = "with-hot-end")]
    #[inline(always)]
    pub fn e_step_pin_high(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::E_STEP, true);
        self.pins.e_step_pin.set_high();
    }

    #[inline(always)]
    pub fn x_step_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::X_STEP, false);
        self.pins.x_step_pin.set_low();
    }

    #[inline(always)]
    pub fn y_step_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Y_STEP, false);
        self.pins.y_step_pin.set_low();
    }

    #[inline(always)]
    pub fn z_step_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::Z_STEP, false);
        self.pins.z_step_pin.set_low();
    }

    #[cfg(feature = "with-hot-end")]
    #[inline(always)]
    pub fn e_step_pin_low(&mut self) {
        #[cfg(all(feature = "native", feature = "plot-timings"))]
        self.tmon.update(PinState::E_STEP, false);
        self.pins.e_step_pin.set_low();
    }

    pub fn endstop_triggered(&self, coordsel: CoordSel) -> bool {
        let mut triggered = false;
        if coordsel.contains(CoordSel::X) {
            triggered |= self.pins.x_endstop_pin.is_high();
        }
        if coordsel.contains(CoordSel::Y) {
            triggered |= self.pins.y_endstop_pin.is_high();
        }
        if coordsel.contains(CoordSel::Z) {
            triggered |= self.pins.z_endstop_pin.is_high();
        }
        triggered
    }

    #[inline]
    pub fn enable_and_set_dir(&mut self, vdir: &TVector<Real>) {
        self.enable_x_stepper();
        self.enable_y_stepper();
        self.enable_z_stepper();
        #[cfg(feature = "with-hot-end")]
        self.enable_e_stepper();

        if vdir.x.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            self.x_dir_pin_high();
        }
        else {
            self.x_dir_pin_low();
        }
        if vdir.y.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            self.y_dir_pin_high();
        }
        else {
            self.y_dir_pin_low();
        }
        if vdir.z.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            self.z_dir_pin_high();
        }
        else {
            self.z_dir_pin_low();
        }
        #[cfg(feature = "with-hot-end")]
        if vdir.e.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            self.e_dir_pin_high();
        }
        else {
            self.e_dir_pin_low();
        }
    }

    pub async fn homing_action(&mut self, motion_config_ref: &MotionConfigRef) -> Result<(), ()>{
        hwa::info!("Homing");

        let motion_config = motion_config_ref.lock().await;
        let steps_per_mm = motion_config.mm_per_unit * TVector::from_coords(
            Some(Real::from_lit(motion_config.usteps[0].into(),0 )),
            Some(Real::from_lit(motion_config.usteps[1].into(), 0)),
            Some(Real::from_lit(motion_config.usteps[2].into(), 0)),
            None,
        );
        hwa::info!("Steps per mm: {}", steps_per_mm);
        let machine_bounds = motion_config.machine_bounds;
        drop(motion_config);

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        hwa::info!("Machine bounds: {}", machine_bounds );

        hwa::info!("Homing X axis");
        let advanced = self.shabbily_move_to(
            TVector::from_coords(Some(ONE.neg()), None, None, None),
            machine_bounds.x.unwrap_or(Real::zero()),
            steps_per_mm,
            1000,
            true,
        ).await;

        hwa::info!("{}", advanced);

        hwa::info!("Homing Y axis");
        let advanced = self.shabbily_move_to(
            TVector::from_coords(None, Some(ONE.neg()), None, None),
            machine_bounds.y.unwrap_or(Real::zero()),
            steps_per_mm,
            1000,
            true,
        ).await;

        hwa::info!("{}", advanced);

        /*

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                hwa::info!("probe down...");
                self.probe_controller.lock().await.probe_pin_down(100).await;
                hwa::info!("TODO: Homing Z axis");
                embassy_time::Timer::after_secs(1).await;
                self.probe_controller.lock().await.probe_pin_up(100).await;
            }
        }

        //#[cfg(all(feature = "native", feature = "plot-timings"))]
        //self.end_segment().await;

         */

        Ok(())
    }

    async fn shabbily_move_to(&mut self, vdir: TVector<Real>, module: Real, steps_per_mm: TVector<Real>, step_frequency: u64, check_endstops: bool) -> TVector<u32>
    {
        let steps_to_advance: TVector<u32> = (vdir * module * steps_per_mm).abs().round().map_coords(|c| { c.to_i32().and_then(|c| Some(c as u32))});
        let mut steps_advanced: TVector<u32> = TVector::zero();

        self.enable_and_set_dir(&vdir);
        let mut coordsel = CoordSel::empty();
        steps_to_advance.apply_coords(|(coord, v)| {
            if v.is_defined_positive() {
                coordsel.set(coord, true);
            }
        });

        let mut ticker = embassy_time::Ticker::every(
            Duration::from_hz(step_frequency)
        );

        loop {
            if check_endstops && self.endstop_triggered(coordsel) {
                hwa::info!("ENDSTOP TRIGGERED");
                return steps_advanced;
            }
            if !steps_advanced.bounded_by(&steps_to_advance) {
                hwa::info!("FULL ADV");
                return steps_advanced;
            }
            if coordsel.contains(CoordSel::X) {
                self.x_step_pin_high();
            }
            if coordsel.contains(CoordSel::Y) {
                self.y_step_pin_high();
            }
            if coordsel.contains(CoordSel::Z) {
                self.z_step_pin_high();
            }

            s_block_for(Duration::from_micros(1));
            if coordsel.contains(CoordSel::X) {
                self.x_step_pin_low();
            }
            if coordsel.contains(CoordSel::Y) {
                self.y_step_pin_low();
            }
            if coordsel.contains(CoordSel::Z) {
                self.z_step_pin_low();
            }
            steps_advanced.increment(coordsel.clone(), 1);

            ticker.next().await;
        }
    }

}