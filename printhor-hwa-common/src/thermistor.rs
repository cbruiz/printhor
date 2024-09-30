//! This module provides the `ThermistorProperties` struct, which holds essential parameters
//! for calculating the temperature from a thermistor sensor. The parameters include the pull-up resistor value,
//! the thermistor's nominal resistance, and the thermistor's Beta value.
//!
//! # Overview
//!
//! A thermistor is a type of resistor whose resistance varies significantly with temperature.
//! The `ThermistorProperties` struct encapsulates three critical attributes necessary
//! for calculating temperature from a thermistor: the pull-up resistor, the thermistor's
//! nominal resistance, and its Beta constant. These parameters are typically used in
//! temperature calculation formulas such as the Steinhart-Hart equation.
//!
//! # Example
//!
//! ```rust
//! use printhor_hwa_common as hwa;
//! use hwa::ThermistorProperties;
//!
//! // Create a new instance of ThermistorProperties with specific values
//! let thermistor = ThermistorProperties::new(10000.0, 10000.0, 3950.0);
//!
//! println!("Pull-up Resistor: {} Ohms", thermistor.r_pullup);
//! println!("Nominal Resistance: {} Ohms", thermistor.r_nominal);
//! println!("Beta Value: {}", thermistor.beta);
//! ```
//!
//! # Fields
//!
//! * `r_pullup`: The resistance of the pull-up resistor in Ohms. This resistor is part of a voltage divider circuit with the thermistor.
//! * `r_nominal`: The nominal resistance of the thermistor at a specific temperature (usually 25°C) in Ohms.
//! * `beta`: The Beta constant of the thermistor, which is used in the Steinhart-Hart equation or Beta parameterization to calculate the temperature.

/// The `ThermistorProperties` struct holds essential parameters for calculating the temperature
/// from a thermistor sensor, including the pull-up resistor value, the thermistor's nominal
/// resistance, and the thermistor's Beta value.
///
/// # Fields
///
/// * `r_pullup`: The resistance of the pull-up resistor in Ohms. This resistor is part of a
/// voltage divider circuit with the thermistor.
/// * `r_nominal`: The nominal resistance of the thermistor at a specific temperature (usually 25°C)
/// in Ohms.
/// * `beta`: The Beta constant of the thermistor, which is used in the Steinhart-Hart equation or
/// Beta parameterization to calculate the temperature.
pub struct ThermistorProperties {
    // Pull-up resistor in Ohms
    pub r_pullup: f32,
    // The nominal resistance of the thermistor in Ohms at a specific temperature (usually 25°C)
    pub r_nominal: f32,
    // Thermistor Beta value
    pub beta: f32,
}

impl ThermistorProperties {
    pub const fn new(r_pullup: f32, r_nominal: f32, beta: f32) -> Self {
        Self {
            r_pullup,
            r_nominal,
            beta,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate as hwa;
    use hwa::ThermistorProperties;
    #[test]
    fn test_thermistor_properties() {
        let thermistor = ThermistorProperties::new(10000.0, 10000.0, 3950.0);
        log::info!("Pull-up Resistor: {} Ohms", thermistor.r_pullup);
        log::info!("Nominal Resistance: {} Ohms", thermistor.r_nominal);
        log::info!("Beta Value: {}", thermistor.beta);
        assert_eq!(thermistor.r_pullup, 10000.0);
        assert_eq!(thermistor.r_nominal, 10000.0);
        assert_eq!(thermistor.beta, 3950.0);
    }
}
