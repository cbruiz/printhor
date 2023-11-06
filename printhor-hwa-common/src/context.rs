pub struct MachineContext<C, D, M, P> {
    pub controllers: C,
    pub devices: D,
    pub motion: M,
    pub pwm: P,
}