use std::io::{stdout, Write};
use embassy_executor::SendSpawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use futures::Stream;
use core::pin::Pin;
use futures::task::Context;
use futures::task::Poll;
use futures::Future;
use async_std::io::ReadExt;

pub(crate) static SERIAL_PIPE: Pipe<CriticalSectionRawMutex, 256> = Pipe::<CriticalSectionRawMutex, 256>::new();

#[embassy_executor::task(pool_size=1)]
pub async fn processor() {

    let mut stream = async_std::io::stdin();

    loop {
        let mut buf = [0u8; 256];
        let n = stream.read(&mut buf).await.unwrap();
        SERIAL_PIPE.write(&buf[..n]).await;
    }
}

pub struct MockedUart {
    spawner: SendSpawner,
}

impl MockedUart {
    pub(crate) fn new(spawner: SendSpawner) -> Self {

        spawner.spawn(processor()).unwrap();

        Self {
            spawner,
        }
    }

    pub(crate) fn split(&self) -> (MockedUartTx, MockedUartRx) {
        (MockedUartTx::new(), MockedUartRx::new(self.spawner))
    }
}

pub struct MockedUartRx {
}

impl MockedUartRx {
    pub(crate) fn new(_spawner: SendSpawner) -> Self {
        Self {
        }
    }
    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, ()> {
        Ok(SERIAL_PIPE.read(buffer).await)
    }
}

pub struct MockedUartTx {
}
impl MockedUartTx {
    pub(crate) const fn new() -> Self {
        Self {
        }
    }
    pub fn blocking_flush(&mut self) {
    }
    pub async fn write(&mut self, b: &[u8]) {
        print!("{}", std::str::from_utf8(b).unwrap());
        let _ = stdout().flush().ok();
    }
}



pub struct MockedUartRxInputStream {
    pub receiver: MockedUartRx,
    buffer: [u8; crate::UART_PORT1_BUFFER_SIZE],
    bytes_read: u8,
    current_byte_index: u8,
}

impl MockedUartRxInputStream {
    pub fn new(receiver: MockedUartRx) -> Self {
        Self {
            receiver,
            buffer: [0; crate::UART_PORT1_BUFFER_SIZE],
            bytes_read: 0,
            current_byte_index: 0,
        }
    }
}

impl Stream for MockedUartRxInputStream
{
    type Item = Result<u8, async_gcode::Error>;

    fn poll_next(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Option<<Self as futures::Stream>::Item>> {

        let this = self.get_mut();

        if this.current_byte_index < this.bytes_read {
            let byte = this.buffer[this.current_byte_index as usize];
            this.current_byte_index += 1;
            Poll::Ready(Some(Ok(byte)))
        }
        else {
            this.current_byte_index = 0;
            this.bytes_read = 0;
            let r = core::pin::pin!(
                this.receiver.read_until_idle(&mut this.buffer)
            ).poll(ctx);
            match r {
                Poll::Ready(rst) => {
                    match rst {
                        Ok(n) => {
                            this.bytes_read = n as u8;
                            if n > 0 {
                                let byte = this.buffer[this.current_byte_index as usize];
                                this.current_byte_index += 1;
                                Poll::Ready(Some(Ok(byte)))
                            }
                            else {
                                Poll::Ready(None)
                            }
                        }
                        Err(_e) => {
                            log::error!("Error: {:?}", _e);
                            Poll::Ready(None)
                        }
                    }
                }
                Poll::Pending => {
                    Poll::Pending
                }
            }
        }
    }
}
