pub trait AsyncWrapper<E> {
    fn wrapped_write(
        &mut self,
        data: &[u8],
    ) -> impl core::future::Future<Output = Result<usize, E>>;

    fn wrapped_flush(&mut self) -> impl core::future::Future<Output = ()>;
}

pub struct SerialAsyncWrapper<P> {
    peri: P,
    ticks_by_word: u64,
    last_write_len: usize,
}

impl<P> SerialAsyncWrapper<P> {
    pub const fn new(peri: P, baud_rate: u32) -> Self {
        let ticks_by_word =
            embassy_time::Duration::from_micros(((1000000 / baud_rate) * 10) as u64).as_ticks();
        Self {
            peri,
            ticks_by_word,
            last_write_len: 0,
        }
    }
}

impl<P, E> AsyncWrapper<E> for SerialAsyncWrapper<P>
where
    P: embedded_io_async::Write<Error = E>,
{
    fn wrapped_write(
        &mut self,
        data: &[u8],
    ) -> impl core::future::Future<Output = Result<usize, E>> {
        async {
            self.last_write_len = data.len();
            self.peri.write(data).await
        }
    }
    fn wrapped_flush(&mut self) -> impl core::future::Future<Output = ()> {
        async {
            if self.last_write_len > 0 {
                let d = embassy_time::Duration::from_ticks(
                    self.ticks_by_word * (self.last_write_len as u64),
                );
                embassy_time::Timer::after(d).await;
                self.last_write_len = 0;
            }
        }
    }
}

#[cfg(test)]
mod test {
    use crate as hwa;
    use crate::AsyncWrapper;

    #[futures_test::test]
    async fn serial_wrapper_works() {
        struct DummyDevice {}
        impl DummyDevice {
            pub fn new() -> Self {
                Self {}
            }
        }
        impl embedded_io_async::ErrorType for DummyDevice {
            type Error = embedded_io_async::ErrorKind;
        }
        impl embedded_io_async::Write for DummyDevice {
            async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                // Nothing
                Ok(buf.len())
            }
        }

        let mut async_wrapper = hwa::SerialAsyncWrapper::new(DummyDevice::new(), 115200);

        async_wrapper.wrapped_write(b"hello world").await.unwrap();
        async_wrapper.wrapped_flush().await;
    }
}
