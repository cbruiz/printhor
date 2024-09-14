use static_cell::StaticCell;
/// Simple monitor to track how much mem we are using for controllers and machinery
pub struct TrackedStaticCell<T>(StaticCell<T>);

impl<T> TrackedStaticCell<T> {
    /// Unfortunately as new() is and must be const, there is no way to track mem
    /// of not explicitly initialized cells
    pub const fn new() -> Self {
        Self {
            0: StaticCell::new(),
        }
    }

    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub fn init<const MAX_SIZE: usize>(&'static self, element_name: &str, val: T) -> &'static mut T {
        stack_reservation_increment::<MAX_SIZE>(element_name , core::mem::size_of::<T>());
        self.0.init(val)
    }

}

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
pub static mut COUNTER: usize = 0;