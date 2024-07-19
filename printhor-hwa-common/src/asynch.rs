
pub trait AsyncWrapper<E> {
    async fn wrapped_write(&mut self, data: &[u8]) -> Result<usize, E>;
    async fn wrapped_flush(&mut self);
}


pub struct SerialAsyncWrapper<P> {
    peri: P,
    ticks_by_word: u64,
    last_write_len: usize,
}

impl<P> SerialAsyncWrapper<P> {
    pub const fn new(peri: P, baud_rate: u32) -> Self {
        let ticks_by_word = embassy_time::Duration::from_micros(((1000000 / baud_rate) * 10) as u64).as_ticks();
        Self {
            peri,
            ticks_by_word,
            last_write_len: 0,
        }
    }
}

impl<P, E> AsyncWrapper<E> for SerialAsyncWrapper<P>
where P: embedded_io_async::Write<Error = E>
{
    async fn wrapped_write(&mut self, data: &[u8]) -> Result<usize, E> {
        self.last_write_len = data.len();
        self.peri.write(data).await
    }
    async fn wrapped_flush(&mut self) {
        if self.last_write_len > 0 {
            let d = embassy_time::Duration::from_ticks(self.ticks_by_word * (self.last_write_len as u64));
            //defmt::info!("flushing, awaiting {} us", d.as_micros());
            embassy_time::Timer::after(d).await;
            self.last_write_len = 0;
        }
    }
}