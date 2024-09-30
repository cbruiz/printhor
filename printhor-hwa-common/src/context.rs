/// `MachineContext` struct holds various components required for machine operation.
///
/// # Type Parameters
///
/// * `C` - Type representing the controllers in the machine context.
/// * `S` - Type representing the system devices in the machine context.
/// * `I` - Type representing the input/output devices in the machine context.
/// * `M` - Type representing the motion-related components in the machine context.
/// * `P` - Type representing the PWM (Pulse Width Modulation) components in the machine context.
pub struct MachineContext<C, S, I, M, P> {
    /// Contains the controllers responsible for managing various machine operations.
    pub controllers: C,
    /// Contains the system devices necessary for the machine's overall functionality.
    pub sys_devices: S,
    /// Contains the input/output devices for data exchange and interactions.
    pub io_devices: I,
    /// Holds components related to motion control, such as motors or actuators.
    pub motion: M,
    /// Holds the Pulse Width Modulation (PWM) components for controlling signal output.
    pub pwm: P,
}

#[cfg(test)]
mod tests {
    use crate::MachineContext;

    #[test]
    fn dummy() {
        let _context = MachineContext {
            controllers: (),
            sys_devices: (),
            io_devices: (),
            motion: (),
            pwm: (),
        };
    }
}
