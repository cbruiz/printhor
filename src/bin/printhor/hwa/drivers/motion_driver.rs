#[allow(unused)]
use crate::{hwa, hwi};
#[cfg(feature = "with-probe")]
use crate::hwa::{ControllerRef};
#[cfg(feature = "with-probe")]
use crate::hwa::controllers::ProbeTrait;

#[cfg(feature = "with-motion")]
pub struct MotionDriverParams {
    pub(crate) motion_device: hwi::device::MotionDevice,
    #[cfg(feature = "with-probe")]
    pub(crate) probe_controller: ControllerRef<hwa::controllers::ServoController>,
    #[cfg(feature = "with-fan0")]
    pub(crate) fan0_controller: ControllerRef<hwa::controllers::Fan0PwmController>,
    #[cfg(feature = "with-fan1")]
    pub(crate) fan1_controller: ControllerRef<hwa::controllers::Fan1PwmController>,
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
    #[cfg(feature = "with-fan1")]
    pub fan1_controller: ControllerRef<hwa::controllers::Fan1PwmController>,
    #[cfg(feature = "with-laser")]
    pub laser_controller: ControllerRef<hwa::controllers::LaserPwmController>,
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
            #[cfg(feature = "with-fan1")]
            fan1_controller: params.fan1_controller,
            #[cfg(feature = "with-laser")]
            laser_controller: params.laser_controller,
        }
    }

    pub async fn homing_action(&mut self) -> Result<(), ()>{
        hwa::info!("Do homing");

        #[cfg(feature="with-laser")]
        let _on = self.laser_controller.lock().await.is_on();

        self.pins.z_enable_pin.set_low();

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
        hwa::info!("1.ZDone");
        #[cfg(feature = "with-probe")]
        self.probe_controller.lock().await.probe_pin_up(300).await;
        if ! reached{
            hwa::info!("not reached in {}", num_pulses);
            Err(())
        }
        else {
            Ok(())
        }
    }
}