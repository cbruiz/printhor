

pub struct MachineContext<C, S, I, M, P> {
    pub controllers: C,
    pub sys_devices: S,
    pub io_devices: I,
    pub motion: M,
    pub pwm: P,
}
/*
use core::marker::PhantomData;
use crate::traits::BoardTrait;
pub struct Board<B>
where B: BoardTrait
{
    phantom_data: PhantomData<B>,
}

impl<B> Board<B>
where B: BoardTrait
{
    pub fn new() -> Self {
        Self { phantom_data: PhantomData }
    }
}

pub struct MachineContext<B: BoardTrait> {
    pub controllers: B::Controllers,
    pub sys_devices: B::SysDevices,
    pub io_devices: B::IODevices,
    pub motion: B::MotionDevices,
    pub pwm: B::PwmDevices,
}
 */