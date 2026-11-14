//! A mocked EEPROM (24C32-like, 4KB) for the native simulator.
//!
//! The content lives in memory (erased to 0xFF on start), so every run starts with the default
//! settings. When the `PRINTHOR_EEPROM_FILE` environment variable is set, the content is loaded
//! from (and saved to) that file, so the settings survive restarts as in a real board.
use printhor_hwa_common as hwa;
use hwa::traits::{AsyncEeprom, EepromError};

pub struct MockedEeprom {
    mem: std::vec::Vec<u8>,
    path: Option<std::string::String>,
}

impl MockedEeprom {
    pub const CAPACITY: usize = 4096;

    pub fn new() -> Self {
        let path = std::env::var("PRINTHOR_EEPROM_FILE").ok();
        let mut mem = std::vec![0xFFu8; Self::CAPACITY];
        if let Some(p) = &path {
            if let Ok(content) = std::fs::read(p) {
                let len = content.len().min(Self::CAPACITY);
                mem[..len].copy_from_slice(&content[..len]);
                hwa::info!("[MockedEeprom] Loaded {} bytes from {}", len, p);
            }
        }
        Self { mem, path }
    }
}

impl AsyncEeprom for MockedEeprom {
    fn capacity(&self) -> usize {
        Self::CAPACITY
    }

    async fn read(&mut self, offset: usize, buffer: &mut [u8]) -> Result<(), EepromError> {
        let end = offset.checked_add(buffer.len()).ok_or(EepromError::OutOfBounds)?;
        buffer.copy_from_slice(self.mem.get(offset..end).ok_or(EepromError::OutOfBounds)?);
        Ok(())
    }

    async fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), EepromError> {
        let end = offset.checked_add(data.len()).ok_or(EepromError::OutOfBounds)?;
        self.mem
            .get_mut(offset..end)
            .ok_or(EepromError::OutOfBounds)?
            .copy_from_slice(data);
        if let Some(p) = &self.path {
            std::fs::write(p, &self.mem).map_err(|_| EepromError::Bus)?;
        }
        Ok(())
    }
}
