/// Stores the physical configuration of the thermistor sensor electronics.
pub struct ThermistorProperties {
    // Pull-up resistor in Ohms
    pub r_pullup: f32,
    // Thermistor
    pub r_nominal: f32,
    // Thermistor Beta value
    pub beta: f32
}

impl ThermistorProperties {
    pub const fn new(r_pullup: f32, r_nominal: f32, beta: f32) -> Self  {
        Self {
            r_pullup,
            r_nominal,
            beta,
        }
    }
}