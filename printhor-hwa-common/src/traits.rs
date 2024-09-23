
/*

use crate::StepperChannel;


pub trait BoardTrait: Sized {
    type Controllers;
    type SysDevices;
    #[cfg(feature = "with-motion")]
    type MotionDevices;
    type IODevices;
    type PwmDevices;

    type Spawner;

    type Periperals;

    fn init_heap();
    fn init();

    async fn setup(spawner: Self::Spawner, peripherals: Self::Periperals) -> MachineContext<Self>;


}


pub trait MotionTrait {
    fn enable(&mut self, channels: crate::StepperChannel);

    fn disable(&mut self, channels: crate::StepperChannel);

    fn set_forward_direction(&mut self, channels: StepperChannel);

    fn step_high(&mut self, channels: StepperChannel);

    fn step_low(&mut self, channels: StepperChannel);

    fn step_toggle(&mut self, channels: StepperChannel);

    fn endstop_triggered(&mut self, channels: StepperChannel) -> bool;
}
*/