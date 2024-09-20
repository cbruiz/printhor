//! A module for tracking static cell memory usage.
//!
//! This module defines a simple monitor to track the amount of memory used for controllers and machinery
//! using static cells. It provides a `TrackedStaticCell` wrapper around `StaticCell` that tracks memory
//! usage when initializing static cells. The memory usage is tracked by incrementing a global counter.
//!
//! # Example
//! ```
//! use printhor_hwa_common as hwa;
//! use hwa::TrackedStaticCell;
//!
//! static TRACKED_CELL: TrackedStaticCell<i32> = TrackedStaticCell::new();
//!
//! fn main() {
//!     let value = TRACKED_CELL.init::<100>("my_element", 42);
//!     assert_eq!(*value, 42);
//! }
//! ```
//!
//! # Features
//! - `with-log`: Enables logging of allocation details when enabled.

use static_cell::StaticCell;

/// Global counter to keep track of memory allocation.
pub static mut COUNTER: usize = 0;

/// Simple monitor to track how much mem we are using for controllers and machinery.
pub struct TrackedStaticCell<T>(StaticCell<T>);

impl<T> TrackedStaticCell<T> {
    /// Unfortunately as new() is and must be const, there is no way to track mem
    /// of not explicitly initialized cells.
    pub const fn new() -> Self {
        Self {
            0: StaticCell::new(),
        }
    }

    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn init<const MAX_SIZE: usize>(&'static self, element_name: &str, val: T) -> &'static mut T {
        stack_reservation_increment::<MAX_SIZE>(element_name, core::mem::size_of::<T>());
        self.0.init(val)
    }
}

/// Increments the stack reservation counter.
///
/// This function checks if the number of bytes `nbytes` exceeds the `MAX_SIZE`.
/// If it does, it panics. Otherwise, it increments a global counter by `nbytes`.
///
/// # Panics
///
/// Panics if `nbytes` is greater than `MAX_SIZE`.
pub(self) fn stack_reservation_increment<const MAX_SIZE: usize>(_element_name: &str, nbytes: usize) {
    #[cfg(feature = "with-log")]
    crate::debug!("D; statically allocated {} bytes for {}", nbytes, _element_name);
    if nbytes > MAX_SIZE {
        panic!("Too much allocation! {} / {}", nbytes, MAX_SIZE)
    }
    unsafe {
        COUNTER += nbytes
    }
}
