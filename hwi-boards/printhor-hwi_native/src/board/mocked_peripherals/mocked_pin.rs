pub(crate) type PinsCell<T> = std::sync::Mutex<T>;

pub type PinStateRef = &'static PinsCell<PinState>;

const NUM_PINS: usize = 100usize;
/// Pin state for state persistence, inter-connection, automation and monitoring
/// Purpose: simulation only
pub(crate) struct PinState {
    digital: [bool; NUM_PINS],
}
impl PinState {
   pub(crate)  const fn new() -> Self {
        Self {
            digital: [false; NUM_PINS]
        }
    }

    pub(crate) fn set(&mut self, id: u8, state: bool) {
        self.digital[id as usize] = state
    }

    pub(crate) fn get(&self, id: u8) -> bool {
        self.digital[id as usize]
    }
}

pub struct MockedIOPin {
    id: u8,
    global_state: PinStateRef,
}

#[allow(unused)]
impl MockedIOPin {
    pub(crate) const fn new(id: u8, global_state: PinStateRef) -> Self {
        Self { id, global_state }
    }

    pub fn set_high(&mut self) {
        loop {
            if let Ok(mut mg) = self.global_state.try_lock() {
                return mg.set(self.id, true);
            }
        }
    }

    pub fn set_low(&mut self) {
        loop {
            if let Ok(mut mg) = self.global_state.try_lock() {
                return mg.set(self.id, false);
            }
        }
    }

    pub fn toggle(&mut self) {
        let state = self.is_high();
        loop {
            if let Ok(mut mg) = self.global_state.try_lock() {
                return mg.set(self.id, !state);
            }
        }
    }

    #[allow(unused)]
    pub fn is_low(&self) -> bool {
        loop {
            if let Ok(mg) = self.global_state.try_lock() {
                return mg.get(self.id) == false
            }
        }
    }

    #[allow(unused)]
    pub fn is_high(&self) -> bool {
        loop {
            if let Ok(mg) = self.global_state.try_lock() {
                return mg.get(self.id) == true
            }
        }
    }
}


cfg_if::cfg_if! {
    if #[cfg(feature = "with-spi")] {
        use embedded_hal_0::digital::v2;
        impl v2::OutputPin for MockedIOPin {
            type Error = core::convert::Infallible;
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }

            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }
    }
}
