use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering::SeqCst;
use embassy_time::Duration;
#[allow(unused)]
use crate::{hwa, hwi};
use crate::control::motion_timing::s_block_for;
#[allow(unused)]
use crate::hwa::ControllerRef;
#[allow(unused)]
#[cfg(feature = "with-probe")]
use crate::hwa::controllers::ProbeTrait;
use crate::math::Real;
use crate::tgeo::{CoordSel, TVector};
#[cfg(all(feature = "native", feature = "plot-timings"))]
use super::timing_monitor::*;

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

    pub async fn homing_action(&mut self) -> Result<(), ()>{
        hwa::info!("Homing");

        compiler_fence(SeqCst);

        let mm_per_second = Real::new(50, 0);
        let microsteps = Real::new(16, 0);
        let axis_pulses_width = TVector::<Real>::one() * Real::from_lit(1000000, 0) / (TVector::<Real>::one() * mm_per_second * microsteps);
        let bounds = TVector::one() * Real::from_lit(200, 0) * microsteps;
        let mut advanced = TVector::zero();

        hwa::info!("Homing speed: {}", axis_pulses_width );
        hwa::info!("Homing bounds: {}", bounds );

        #[cfg(feature="with-laser")]
        self.laser_controller.lock().await.set_power(0).await;

        hwa::info!("Homing X axis");
        self.enable_x_stepper();
        self.x_dir_pin_low();
        let mut ticker = embassy_time::Ticker::every(
            embassy_time::Duration::from_micros(axis_pulses_width.x.unwrap().to_i64().unwrap() as u64)
        );
        while self.pins.x_endstop_pin.is_low() {
            if !advanced.bounded_by(&bounds) {
                hwa::warn!("Missing X endstop after {}", advanced);
                return Err(());
            }
            self.x_step_pin_high();
            s_block_for(Duration::from_micros(10));
            self.x_step_pin_low();
            s_block_for(Duration::from_micros(10));
            advanced.increment(CoordSel::X, Real::one());
            ticker.next().await;
        }
        hwa::info!("{}", advanced);
        advanced.set_coord(CoordSel::X, Some(Real::zero()));

        hwa::info!("Homing Y axis");
        self.enable_y_stepper();
        self.y_dir_pin_low();
        let mut ticker = embassy_time::Ticker::every(
            Duration::from_micros(axis_pulses_width.y.unwrap().to_i64().unwrap() as u64)
        );
        while self.pins.y_endstop_pin.is_low() {
            if !advanced.bounded_by(&bounds) {
                hwa::warn!("Missing Y endstop after {}", advanced);
                return Err(());
            }
            self.y_step_pin_high();
            s_block_for(Duration::from_micros(10));
            self.y_step_pin_low();
            s_block_for(Duration::from_micros(10));
            advanced.increment(CoordSel::Y, Real::one());
            ticker.next().await;
        }

        hwa::info!("{}", advanced);
        advanced.set_coord(CoordSel::Y, Some(Real::zero()));

        hwa::info!("Move Z up");
        self.enable_z_stepper();
        self.z_dir_pin_high();
        ticker = embassy_time::Ticker::every(
            Duration::from_micros(axis_pulses_width.z.unwrap().to_i64().unwrap() as u64)
        );
        for _i in 0 .. 16 * 10 {
            self.z_step_pin_high();
            s_block_for(Duration::from_micros(10));
            advanced.increment(CoordSel::Z, Real::one());
            self.z_step_pin_low();
            s_block_for(Duration::from_micros(10));
            ticker.next().await;
        }

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

        Ok(())
    }

}