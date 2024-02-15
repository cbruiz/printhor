pub struct MachineContext<C, S, I, M, P> {
    pub controllers: C,
    pub sys_devices: S,
    pub io_devices: I,
    pub motion: M,
    pub pwm: P,
}