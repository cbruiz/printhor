use std::marker::PhantomData;
use embassy_executor::SendSpawner;

pub struct MockedWatchdog<'a, T> {
    _spawner: SendSpawner,
    _timeout: u32,
    p: PhantomData<&'a T>,
}
impl<'a, T> MockedWatchdog<'a, T> {

    pub(crate) const fn new(spawner: SendSpawner, timeout: u32) -> Self {
        Self {
            _spawner: spawner,
            _timeout: timeout,
            p: PhantomData,
        }
    }

    pub fn unleash(&mut self) {
    }

    pub fn pet(&mut self) {
        //println!("pet!");
    }
}