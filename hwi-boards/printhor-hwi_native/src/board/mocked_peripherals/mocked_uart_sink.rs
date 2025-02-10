use core::future;

// A mocked Uart which does nothing
pub struct MockedUartSink {
}

impl MockedUartSink {
    pub fn new() -> Self {
        Self {
        }
    }

    pub fn split(&self) -> (MockedUartSinkTx, MockedUartSinkRx) {
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

    pub async fn write_packet(&mut self, _b: &[u8]) {}

    pub async fn wrapped_flush(&mut self) {}
    pub async fn wrapped_write(&mut self, _b: &[u8]) {}
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

impl async_gcode::ByteStream for MockedUartSinkRxInputStream
{
    type Item = Result<u8, async_gcode::Error>;


    async fn next(&mut self) -> Option<Self::Item> {
        future::pending().await
    }

    async fn recovery_check(&mut self) {}
}
