use printhor_hwa_common as hwa;
use hwa::HwiContract;
use crate::Contract;
use smol::net::unix::{UnixListener, UnixStream};
use smol::io::{AsyncReadExt, AsyncWriteExt};
use std::io::ErrorKind;
use std::net::Shutdown;

/// A quick and dirty approach to get it done with the unix sockets communication
/// The approach is thought to be full duplex in relation with an external process

pub struct RawConnectionManager {
    /// Accepts are serialized by the `reader` lock (only the Rx side accepts connections)
    /// None when the socket could not be bound: the port never gets a connection
    listener: Option<&'static UnixListener>,
    reader: &'static hwa::AsyncMutex<hwa::AsyncNoopMutexType, Option<UnixStream>>,
    writer: &'static hwa::AsyncMutex<hwa::AsyncNoopMutexType, Option<UnixStream>>,
}

pub struct MockedUartUnixSocket {
    manager: &'static RawConnectionManager,
}

impl MockedUartUnixSocket {
    pub async fn new(socket_path: String) -> Self {

        match smol::fs::remove_file(&socket_path).await {
            Ok(_) => {
                // Fine
            }
            Err(e) => {
                match e.kind() {
                    ErrorKind::NotFound => {
                        // Fine
                    }
                    _other => {
                        hwa::error!("[mocked-unix-socket] Unable to remove prior socket file. Binding will probably fail due to error: {:?}", e);
                    }
                }
            }
        }

        // Address is in use, not enough permissions to do it [...]: reported, and the port stays
        // disconnected (never panics)
        let listener = match UnixListener::bind(socket_path) {
            Ok(instance) => Some(&*hwa::make_static_ref! {
                "Listener",
                UnixListener,
                instance
            }),
            Err(_e) => {
                hwa::error!("[mocked-unix-socket] Can't bind to socket path: {:?}. Port disabled", _e);
                None
            }
        };

        Self {
            manager: hwa::make_static_ref! {
                "UnixConnection",
                RawConnectionManager,
                RawConnectionManager {
                    listener,
                    reader: hwa::make_static_ref! {
                        "UnixSocketReader",
                        hwa::AsyncMutex<hwa::AsyncNoopMutexType, Option<UnixStream>>,
                        hwa::AsyncMutex::new(None),
                    },
                    writer: hwa::make_static_ref! {
                        "UnixSocketWriter",
                        hwa::AsyncMutex<hwa::AsyncNoopMutexType, Option<UnixStream>>,
                        hwa::AsyncMutex::new(None),
                    }
                }
            },
        }
    }

    pub fn split(self) -> (MockedUartNamedPipeTx, MockedUartNamedPipeRx) {
        (
            MockedUartNamedPipeTx::new(self.manager),
            MockedUartNamedPipeRx::new(self.manager)
        )
    }
}

pub struct MockedUartNamedPipeRx {
    manager: &'static RawConnectionManager,
    // A silly flag to not report constantly the same
    is_already_trying_to_listen: bool,
}

impl MockedUartNamedPipeRx {
    pub fn new(manager: &'static RawConnectionManager) -> Self {
        Self {
            manager,
            is_already_trying_to_listen: false,
        }
    }

    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, ()> {

        let mut rg = self.manager.reader.lock().await;
        let reader_instance = match rg.as_mut() {
            None => {
                // Wait for another accept
                if !self.is_already_trying_to_listen {
                    hwa::info!("[mocked-unix-socket] Waiting for connection");
                    self.is_already_trying_to_listen = true;
                }
                let listener = match self.manager.listener {
                    Some(listener) => listener,
                    // Disabled port: no data, ever
                    None => core::future::pending().await,
                };
                let (stream, _addr) = match listener.accept().await {
                    Ok(accepted) => accepted,
                    Err(_e) => {
                        hwa::error!("[mocked-unix-socket] Accept failed: {:?}", _e);
                        return Err(());
                    }
                };
                let mut wg = self.manager.writer.lock().await;
                wg.replace(stream.clone());
                rg.replace(stream);
                hwa::info!("[mocked-unix-socket] Connection established");
                match rg.as_mut() {
                    // Not expected (just replaced). Reported as a read error, never panics
                    None => return Err(()),
                    Some(guarded_instance) => { guarded_instance }
                }
            }
            Some(guarded_reference) => {
                guarded_reference
            }
        };
        let read_result = match reader_instance.read(buffer).await {
            Ok(n) => {
                if n == 0 {
                    hwa::warn!("[mocked-unix-socket] Got EOF. Will drop connection");
                    Err(())
                }
                else {
                    Ok(n)
                }
            }
            Err(_e) => {
                hwa::error!("[mocked-unix-socket] Error reading from socket: {:?}", _e);
                Err(())
            }
        };
        if read_result.is_err() {
            // Drop connection
            if let Some(_in) = rg.take() {
                let _ = _in.shutdown(Shutdown::Both);
            }
            let mut wg = self.manager.writer.lock().await;
            if let Some(_out) = wg.take() {
                let _ = _out.shutdown(Shutdown::Both);
            }
            self.is_already_trying_to_listen = false;
            hwa::info!("[mocked-unix-socket] Connection dropped");
        }
        // Finally return the read result
        read_result
    }
}

pub struct MockedUartNamedPipeTx {
    manager: &'static RawConnectionManager,
}
impl MockedUartNamedPipeTx {
    pub fn new(manager: &'static RawConnectionManager) -> Self {
        Self {
            manager,
        }
    }

    pub async fn write_packet(&mut self, b: &[u8]) {
        self.wrapped_write(b).await;
    }
    pub async fn wrapped_flush(&mut self) {
    }
    pub async fn wrapped_write(&mut self, b: &[u8]) {
        let mut mgw = self.manager.writer.lock().await;
        if let Some(writer) = mgw.as_mut() {
            match writer.write_all(b).await {
                Ok(_) => {}
                Err(_e) => {
                    hwa::error!("[mocked-unix-socket] Error writing to socket: {:?}", _e);
                }
            }
        }
    }
}

pub struct MockedUartNamedPipeRxInputStream {
    pub receiver: MockedUartNamedPipeRx,
    buffer: [u8; Contract::SERIAL_USB_PACKET_SIZE],
    bytes_read: u8,
    current_byte_index: u8,
}

impl MockedUartNamedPipeRxInputStream {
    pub fn new(receiver: MockedUartNamedPipeRx) -> Self {
        Self {
            receiver,
            buffer: [0; Contract::SERIAL_USB_PACKET_SIZE],
            bytes_read: 0,
            current_byte_index: 0,
        }
    }
}

impl async_gcode::ByteStream for MockedUartNamedPipeRxInputStream
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
                    // Should be already reported by the inner layer
                    None
                }
            }
        }
    }

    async fn recovery_check(&mut self) {
    }

}
