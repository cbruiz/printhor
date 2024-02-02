use futures::Stream;
use core::pin::Pin;
use futures::task::Context;
use futures::task::Poll;

// A mocked Uart which does nothing
pub struct MockedUartSink {
}

impl MockedUartSink {
    pub(crate) fn new() -> Self {
        Self {
        }
    }

    pub(crate) fn split(&self) -> (MockedUartSinkTx, MockedUartSinkRx) {
        (MockedUartSinkTx::new(), MockedUartSinkRx::new())
    }
}

pub struct MockedUartSinkRx {
}

impl MockedUartSinkRx {
    pub(crate) fn new() -> Self {
        Self {
        }
    }
}

pub struct MockedUartSinkTx {
}
impl MockedUartSinkTx {
    pub(crate) const fn new() -> Self {
        Self {}
    }
    pub fn blocking_flush(&mut self) {}

    pub async fn write(&mut self, _b: &[u8]) {}
}

pub struct MockedUartSinkRxInputStream {
    pub receiver: MockedUartSinkRx,
}

impl MockedUartSinkRxInputStream {
    pub fn new(receiver: MockedUartSinkRx) -> Self {
        Self {
            receiver
        }
    }
}

impl Stream for MockedUartSinkRxInputStream
{
    type Item = Result<u8, async_gcode::Error>;

    fn poll_next(self: Pin<&mut Self>, _ctx: &mut Context<'_>) -> Poll<Option<<Self as futures::Stream>::Item>> {
        Poll::Pending
    }
}
