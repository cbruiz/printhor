//! A module to inject gcode similar way [crate::processing] machinery does
use std::collections::VecDeque;

pub(crate) struct GCodeBuffer {
    buffer: VecDeque<u8>,
}

impl GCodeBuffer {
    pub const fn new() -> Self {
        Self {
            buffer: VecDeque::new(),
        }
    }
    pub fn pop_front(&mut self) -> Option<u8> {
        self.buffer.pop_front()
    }

    pub fn append(&mut self, data: &str) {
        self.buffer.extend(data.as_bytes());
    }
}

pub(crate) struct BufferStream {
    inner: GCodeBuffer,
}

impl BufferStream {
    pub(crate) const fn new(buff: GCodeBuffer) -> Self {
        Self { inner: buff }
    }
}

impl async_gcode::ByteStream for BufferStream {
    type Item = Result<u8, async_gcode::Error>;

    async fn next(&mut self) -> Option<Self::Item> {
        match self.inner.pop_front() {
            None => None,
            Some(_b) => Some(Ok(_b)),
        }
    }

    async fn recovery_check(&mut self) {}
}
