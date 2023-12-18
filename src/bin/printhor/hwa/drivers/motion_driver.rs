#[allow(unused)]
use crate::{hwa, hwi};
#[allow(unused)]
use crate::hwa::ControllerRef;
#[cfg(feature = "with-probe")]
use crate::hwa::controllers::ProbeTrait;
#[cfg(feature = "native")]
use super::timing_monitor::*;

#[cfg(feature = "with-motion")]
pub struct MotionDriverParams {
    pub(crate) motion_device: hwi::device::MotionDevice,
    #[cfg(feature = "with-probe")]
    pub(crate) probe_controller: ControllerRef<hwa::controllers::ServoController>,
    #[cfg(feature = "with-fan0")]
    pub(crate) fan0_controller: ControllerRef<hwa::controllers::Fan0PwmController>,
    #[cfg(feature = "with-fan-layer")]
    pub(crate) layer_fan_controller: ControllerRef<hwa::controllers::LayerPwmController>,
    #[cfg(feature = "with-laser")]
    pub(crate) laser_controller: ControllerRef<hwa::controllers::LaserPwmController>,
}

pub struct MotionDriver {

    #[cfg(feature = "with-motion")]
    pub pins: hwi::device::MotionPins,
    #[cfg(feature = "with-trinamic")]
    pub trinamic_controller: hwa::controllers::TrinamicController,
    #[cfg(feature = "with-probe")]
    pub probe_controller: ControllerRef<hwa::controllers::ServoController>,
    #[cfg(feature = "with-fan0")]
    pub fan0_controller: ControllerRef<hwa::controllers::Fan0PwmController>,
    #[cfg(feature = "with-fan-layer")]
    pub layer_fan_controller: ControllerRef<hwa::controllers::LayerPwmController>,
    #[cfg(feature = "with-laser")]
    pub laser_controller: ControllerRef<hwa::controllers::LaserPwmController>,
    #[cfg(feature = "native")]
    tmon: TimingsMonitor,
}

#[cfg(feature = "with-motion")]
impl MotionDriver {
    pub fn new(params: MotionDriverParams) -> Self {
        Self {
            pins: params.motion_device.motion_pins,
            #[cfg(feature = "with-trinamic")]
            trinamic_controller: hwa::controllers::TrinamicController::new(params.motion_device.trinamic_uart),
            #[cfg(feature = "with-probe")]
            probe_controller: params.probe_controller,
            #[cfg(feature = "with-fan0")]
            fan0_controller: params.fan0_controller,
            #[cfg(feature = "with-fan-layer")]
            layer_fan_controller: params.layer_fan_controller,
            #[cfg(feature = "with-laser")]
            laser_controller: params.laser_controller,
            #[cfg(feature = "native")]
            tmon: TimingsMonitor::new(),
        }
    }

    #[cfg(feature = "native")]
    #[inline]
    pub fn start_segment(&mut self, ref_time: embassy_time::Instant) {
        self.tmon.reset(ref_time)
    }
    #[cfg(feature = "native")]
    #[inline]
    pub fn end_segment(&mut self) {
        self.tmon.commit()
    }

    #[cfg(feature = "native")]
    #[inline]
    pub fn mark_microsegment(&mut self) {
        self.tmon.swap(PinState::USCLK)
    }

    #[inline(always)]
    pub fn enable_x_stepper(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::X_ENA, true);
        self.pins.enable_x_stepper()
    }

    #[inline(always)]
    pub fn enable_y_stepper(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Y_ENA, true);
        self.pins.enable_y_stepper()
    }

    #[inline(always)]
    pub fn enable_z_stepper(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Z_ENA, true);
        self.pins.enable_z_stepper()
    }

    #[cfg(feature = "has-extruder")]
    #[inline(always)]
    pub fn enable_e_stepper(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::E_ENA, true);
        self.pins.enable_e_stepper()
    }

    #[inline(always)]
    pub fn x_dir_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::X_DIR, true);
        self.pins.x_dir_pin.set_high();
    }

    #[inline(always)]
    pub fn y_dir_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Y_DIR, true);
        self.pins.y_dir_pin.set_high();
    }

    #[inline(always)]
    pub fn z_dir_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Z_DIR, true);
        self.pins.z_dir_pin.set_high();
    }

    #[cfg(feature = "has-extruder")]
    #[inline(always)]
    pub fn e_dir_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::E_DIR, true);
        self.pins.e_dir_pin.set_high();
    }

    #[inline(always)]
    pub fn x_dir_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::X_DIR, false);
        self.pins.x_dir_pin.set_low();
    }

    #[inline(always)]
    pub fn y_dir_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Y_DIR, false);
        self.pins.y_dir_pin.set_low();
    }

    #[inline(always)]
    pub fn z_dir_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Z_DIR, false);
        self.pins.z_dir_pin.set_low();
    }

    #[cfg(feature = "has-extruder")]
    #[inline(always)]
    pub fn e_dir_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::E_DIR, false);
        self.pins.e_dir_pin.set_low();
    }

    #[inline(always)]
    pub fn x_step_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::X_STEP, true);
        self.pins.x_step_pin.set_high();
    }

    #[inline(always)]
    pub fn y_step_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Y_STEP, true);
        self.pins.y_step_pin.set_high();
    }

    #[inline(always)]
    pub fn z_step_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Z_STEP, true);
        self.pins.z_step_pin.set_high();
    }

    #[cfg(feature = "has-extruder")]
    #[inline(always)]
    pub fn e_step_pin_high(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::E_STEP, true);
        self.pins.e_step_pin.set_high();
    }

    #[inline(always)]
    pub fn x_step_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::X_STEP, false);
        self.pins.x_step_pin.set_low();
    }

    #[inline(always)]
    pub fn y_step_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Y_STEP, false);
        self.pins.y_step_pin.set_low();
    }

    #[inline(always)]
    pub fn z_step_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::Z_STEP, false);
        self.pins.z_step_pin.set_low();
    }

    #[cfg(feature = "has-extruder")]
    #[inline(always)]
    pub fn e_step_pin_low(&mut self) {
        #[cfg(feature = "native")]
        self.tmon.update(PinState::E_STEP, false);
        self.pins.e_step_pin.set_low();
    }

    pub async fn homing_action(&mut self) -> Result<(), ()>{
        hwa::debug!("Do homing");

        #[cfg(feature="with-laser")]
        let _on = self.laser_controller.lock().await.is_on();

        self.pins.enable_z_stepper();

        let mut num_pulses = 0u32;
        let mut reached = false;

        self.pins.z_dir_pin.set_high();

        // Move up a little bit
        for _i in 0..2 {
            self.pins.z_step_pin.set_high();
            embassy_time::Timer::after(embassy_time::Duration::from_micros(10)).await;
            self.pins.z_step_pin.set_low();
            embassy_time::Timer::after(embassy_time::Duration::from_micros(1000)).await;
        }

        // Move down until endtop hit
        self.pins.z_dir_pin.set_low();
        #[cfg(feature = "with-probe")]
        self.probe_controller.lock().await.probe_pin_down(300).await;
        for _i in 0..10 * 10 {
            if self.pins.z_endstop_pin.is_high() {
                hwa::info!(" - R");
                reached = true;
                break;
            }
            num_pulses += 1;
            self.pins.z_step_pin.set_high();
            embassy_time::Timer::after(embassy_time::Duration::from_micros(10)).await;
            self.pins.z_step_pin.set_low();
            embassy_time::Timer::after(embassy_time::Duration::from_micros(1000)).await;
        }
        hwa::debug!("1.ZDone");
        #[cfg(feature = "with-probe")]
        self.probe_controller.lock().await.probe_pin_up(300).await;
        if ! reached{
            hwa::debug!("not reached in {}", num_pulses);
            Err(())
        }
        else {
            Ok(())
        }
    }


}