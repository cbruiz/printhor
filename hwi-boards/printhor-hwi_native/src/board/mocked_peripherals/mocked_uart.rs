use printhor_hwa_common as hwa;

use std::io::{stdout, Write};
use embassy_executor::SendSpawner;
use embassy_sync::pipe::Pipe;
use embassy_time::{Duration, with_timeout};
use async_std::io::ReadExt;
use printhor_hwa_common::HwiContract;
use crate::board::TERMINATION;

pub static SERIAL_PIPE: Pipe<hwa::AsyncCsMutexType, {crate::Contract::SERIAL_PORT1_RX_BUFFER_SIZE}> = Pipe::<hwa::AsyncCsMutexType, {crate::Contract::SERIAL_PORT1_RX_BUFFER_SIZE}>::new();
#[embassy_executor::task(pool_size=1)]
pub async fn task_mocked_uart() {

    hwa::info!("[task_mocked_uart] starting");

    // byte-to-byte reading is required for stdin for it to work unbuffered with simple I/O management.
    // Another solution could be leveraging a wrapper/crate on top of native select() with the proper ioctl on stdin, but is not worthy in this case.
    // Unbuffered I/O requirement for piping simulator with Universal Gcode Sender and others by socat.

    let mut stream = async_std::io::stdin();
    let mut buf = [0u8; 1];
    let mut error_reading = false;

    loop {
        match with_timeout(
            Duration::from_secs(60),
            stream.read_exact(&mut buf)
        ).await {
            Err(_) => {
                if TERMINATION.signaled() {
                    hwa::info!("[task_mocked_uart] Ending gracefully");
                    return ();
                }
            }
            Ok(Ok(_)) => {
                error_reading = false; // Recovered
                SERIAL_PIPE.write(&buf[..1]).await;
            }
            Ok(Err(_e)) => { // Error reading from stdin: Closed or temporary unavailable
                if !error_reading {
                    hwa::trace!("[task_mocked_uart] Error reading from stdin {:?}", _e);
                    error_reading = true;
                    if TERMINATION.signaled() {
                        hwa::info!("[task_mocked_uart] Ending gracefully");
                        return ();
                    }
                }
                else {
                    // Could happen. For instance in CI.
                    // In this case we will wait to avoid respawning fast and keep trying in case it
                    // can be recovered (practically improbable)
                    embassy_time::Timer::after(Duration::from_secs(10)).await;
                }
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
        hwa::trace!("Reading from pipe");
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

    pub async fn write_packet(&mut self, b: &[u8]) {
        self.wrapped_write(b).await;
    }
    pub async fn wrapped_flush(&mut self) {
    }
    pub async fn wrapped_write(&mut self, b: &[u8]) {
        print!("{}", std::str::from_utf8(b).unwrap());
        let _ = stdout().flush().ok();
    }
}

pub struct MockedUartRxInputStream<const BUF_SIZE: usize> {
    pub receiver: MockedUartRx,
    buffer: [u8; BUF_SIZE],
    bytes_read: u8,
    current_byte_index: u8,
}

impl<const BUFF_SIZE: usize> MockedUartRxInputStream<BUFF_SIZE>
{
    pub fn new(receiver: MockedUartRx) -> Self {
        Self {
            receiver,
            buffer: [0; BUFF_SIZE],
            bytes_read: 0,
            current_byte_index: 0,
        }
    }
}

impl<const BUFF_SIZE: usize> async_gcode::ByteStream for MockedUartRxInputStream<BUFF_SIZE>
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
                        hwa::error!("0 bytes read. EOF?");
                        None
                    }
                }
                Err(_e) => {
                    hwa::error!("Error: {:?}", _e);
                    None
                }
            }
        }
    }

    async fn recovery_check(&mut self) {
    }

}
