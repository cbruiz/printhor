use embassy_time::{Duration, Timer};
use printhor_hwa_common::TrackedStaticCell;
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
use crate::device::AdcPinTrait;

type PinsCell<T> = std::sync::Mutex<T>;

pub type PinStateRef = &'static PinsCell<PinState>;

const NUM_PINS: usize = 100usize;
/// Pin state for state persistence, inter-connection, automation and monitoring
/// Purpose: simulation only
pub(crate) struct PinState {
    digital: [bool; NUM_PINS],
}
impl PinState {
    const fn new() -> Self {
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

static GLOBAL_PIN_STATE: TrackedStaticCell<PinsCell<PinState>> = TrackedStaticCell::new();

#[inline]
pub(crate) fn init_pin_state() -> PinStateRef {
    GLOBAL_PIN_STATE.init::<{crate::MAX_STATIC_MEMORY}>("GlobaPinState", PinsCell::new(PinState::new()))
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

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
impl<T> AdcPinTrait<crate::board::mocked_peripherals::MockedAdc<T>> for MockedIOPin {

}

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
impl<T> AdcPinTrait<crate::board::mocked_peripherals::MockedAdc<T>> for u8 {

}

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
impl AdcPinTrait<u8> for u8 {

}

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
impl AdcPinTrait<u8> for MockedIOPin {

}

#[cfg(feature = "with-hot-bed")]
impl<'a, ADC, Word, PIN> embedded_hal_02::adc::OneShot<ADC, Word, PIN> for MockedIOPin
where PIN: embedded_hal_02::adc::Channel<ADC>
{

    type Error = u8;
    fn read(&mut self, _: &mut PIN) -> Result<Word, nb::Error<u8>> {
        todo!()
    }

}


#[cfg(feature = "with-spi")]
impl<T> embedded_hal::digital::v2::OutputPin for MockedIOPin<'_, T> {
    type Error = core::convert::Infallible;
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}