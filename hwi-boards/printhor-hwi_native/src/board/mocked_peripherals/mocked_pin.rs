use embassy_time::{Duration, Timer};

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

    #[inline]
    pub(crate) fn set(&mut self, id: u8, state: bool) {
        self.digital[id as usize] = state
    }

    #[inline]
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
            match self.global_state.try_lock() {
                Ok(mut g) => return g.set(self.id, true),
                Err(_e) => {
                    //panic!("Error setting high: {:?}", _e)
                }
            }
        }
    }

    pub fn set_low(&mut self) {
        loop {
            match self.global_state.try_lock() {
                Ok(mut g) => return g.set(self.id, false),
                Err(_e) => {
                    //panic!("Error setting low: {:?}", _e)
                }
            }
        }
    }

    pub fn toggle(&mut self) {
        let state = self.is_high();
        loop {
            match self.global_state.try_lock() {
                Ok(mut g) => return g.set(self.id, !state),
                Err(_e) => {
                    //panic!("Error setting low: {:?}", _e)
                }
            }
        }
    }

    pub fn is_set_high(&mut self) -> bool {
        self.is_high()
    }

    pub fn is_set_low(&mut self) -> bool {
        self.is_low()
    }

    #[allow(unused)]
    pub fn is_low(&self) -> bool {
        loop {
            match self.global_state.try_lock() {
                Ok(g) => return g.get(self.id) == false,
                Err(_e) => {
                    //panic!("Error getting low: {:?}", _e)
                }
            }
        }
    }

    #[allow(unused)]
    pub fn is_high(&self) -> bool {
        loop {
            match self.global_state.try_lock() {
                Ok(g) => return g.get(self.id) == true,
                Err(_e) => {
                    //panic!("Error getting high: {:?}", _e)
                }
            }
        }
    }

    #[allow(unused)]
    pub async fn wait_for_any_edge<'b>(&'b mut self) {
        //println!("wait_edge");
        Timer::after(Duration::from_secs(10)).await;
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
