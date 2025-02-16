#![doc = r"
This module provides serial communication (UART) elements.

## Enums

### `SerialError`
Represents possible errors that can occur during serial communication.
- `Timeout`: Indicates a read or write operation has timed out.
- `Framing`: Indicates framing error, where the stop bit was not as expected.
"]

//! TODO: [Work In progress] Software serial communication (UART)
#[allow(unused)]
use crate as hwa;

/// Serial communication error type
#[derive(Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum SerialError {
    /// Timeout
    Timeout,
    /// Framing error
    Framing,
}
