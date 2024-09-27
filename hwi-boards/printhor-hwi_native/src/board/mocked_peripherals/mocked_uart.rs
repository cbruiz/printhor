use std::io::{stdout, Write};
use embassy_executor::SendSpawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_time::{Duration, with_timeout};
use async_std::io::ReadExt;
use log::info;
use crate::TERMINATION;

pub static SERIAL_PIPE: Pipe<CriticalSectionRawMutex, {crate::UART_PORT1_BUFFER_SIZE}> = Pipe::<CriticalSectionRawMutex, {crate::UART_PORT1_BUFFER_SIZE}>::new();

#[embassy_executor::task(pool_size=1)]
pub async fn task_mocked_uart() {

    info!("[task_mocked_uart] starting");

    // byte-to-byte reading is required for stdin for it to work unbuffered with simple I/O management.
    // Another solution could be leveraging a wrapper/crate on top of native select() with the proper ioctl on stdin, but is not worthy in this case.
    // Unbuffered I/O requirement for piping simulator with Universal Gcode Sender and others by socat.

    let mut stream = async_std::io::stdin();
    let mut buf = [0u8; 1];

    loop {
        match with_timeout(
            Duration::from_secs(60),
            stream.read_exact(&mut buf)
        ).await {
            Err(_) => {
                if TERMINATION.signaled() {
                    info!("[task_mocked_uart] Ending gracefully");
                    return ();
                }
            }
            Ok(Ok(_)) => {
                SERIAL_PIPE.write(&buf[..1]).await;
            }
            Ok(Err(_e)) => {
                panic!("Serial EOF");
            }
        }
    }
}

pub struct MockedUart {
    spawner: SendSpawner,
}

impl MockedUart {
    pub(crate) fn new(spawner: SendSpawner) -> Self {

        spawner.spawn(task_mocked_uart()).unwrap();

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
        log::trace!("Reading from pipe");
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
    pub async fn wrapped_flush(&mut self) {
    }
    pub async fn wrapped_write(&mut self, b: &[u8]) {
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

impl async_gcode::ByteStream for MockedUartRxInputStream
{
    type Item = Result<u8, async_gcode::Error>;

    async fn next(&mut self) -> Option<Self::Item> {

        if self.current_byte_index < self.bytes_read {
            let byte = self.buffer[self.current_byte_index as usize];
            self.current_byte_index += 1;
            Some(Ok(byte))
        }
        else {
            self.current_byte_index = 0;
            self.bytes_read = 0;
            let r = self.receiver.read_until_idle(&mut self.buffer).await;
            match r {
                Ok(n) => {
                    self.bytes_read = n as u8;
                    if n > 0 {
                        let byte = self.buffer[self.current_byte_index as usize];
                        self.current_byte_index += 1;
                        Some(Ok(byte))
                    }
                    else {
                        log::error!("0 bytes read. EOF?");
                        None
                    }
                }
                Err(_e) => {
                    log::error!("Error: {:?}", _e);
                    None
                }
            }
        }
    }

}
