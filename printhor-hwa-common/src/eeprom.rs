//! Generic async driver for the I2C 24Cxx EEPROM family (AT24C32 and larger: 16-bit memory
//! addresses), as found in many 3D printer boards (SKR Mini E3 V3, MKS Robin Nano V3.x...).
//!
//! The driver is fully async: bus transfers are performed through
//! [embedded_hal_async::i2c::I2c] and the internal write cycle of the chip is awaited with an
//! async delay ([embassy_time::Delay] by default), so the executor is never blocked.
//!
//! Writes are page oriented and differential: each page is read first and only rewritten when
//! its content changes. This reduces both the wear of the cells and the latency of storing the
//! settings (an unchanged `M500` costs just the reads). Every written page is read back to
//! verify it, which also polls the end of the internal write cycle.
use crate::traits::{AsyncEeprom, EepromError};
use embedded_hal_async::delay::DelayNs;

/// The default 7-bit I2C address of a 24Cxx chip with A0..A2 tied to ground
pub const DEFAULT_I2C_24CXX_ADDRESS: u8 = 0x50;

/// Maximum time (ms) a write cycle may last before giving up (datasheets: 5ms-10ms)
const WRITE_CYCLE_TIMEOUT_MS: u64 = 20;

/// Async driver for an I2C 24Cxx EEPROM with 16-bit memory addresses.
///
/// * `SIZE`: capacity in bytes (e.g. 4096 for an AT24C32)
/// * `PAGE`: write page size in bytes (e.g. 32 for an AT24C32). At most 64.
/// * `D`: the async delay used to await the write cycles
pub struct I2cEeprom24x<I2C, const SIZE: usize, const PAGE: usize, D = embassy_time::Delay>
where
    I2C: embedded_hal_async::i2c::I2c,
    D: DelayNs,
{
    i2c: I2C,
    address: u8,
    delay: D,
}

/// AT24C32 (4KB, 32-byte pages)
pub type At24c32<I2C> = I2cEeprom24x<I2C, 4096, 32>;

/// AT24C64 (8KB, 32-byte pages)
pub type At24c64<I2C> = I2cEeprom24x<I2C, 8192, 32>;

impl<I2C, const SIZE: usize, const PAGE: usize> I2cEeprom24x<I2C, SIZE, PAGE>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self::new_with_delay(i2c, address, embassy_time::Delay)
    }
}

impl<I2C, const SIZE: usize, const PAGE: usize, D> I2cEeprom24x<I2C, SIZE, PAGE, D>
where
    I2C: embedded_hal_async::i2c::I2c,
    D: DelayNs,
{
    const VALID_PAGE: () = assert!(PAGE > 0 && PAGE <= 64 && SIZE % PAGE == 0 && SIZE <= 65536);

    pub const fn new_with_delay(i2c: I2C, address: u8, delay: D) -> Self {
        #[allow(clippy::let_unit_value)]
        let _ = Self::VALID_PAGE;
        Self { i2c, address, delay }
    }

    async fn raw_read(&mut self, offset: usize, buffer: &mut [u8]) -> Result<(), EepromError> {
        let addr = (offset as u16).to_be_bytes();
        self.i2c
            .write_read(self.address, &addr, buffer)
            .await
            .map_err(|_| EepromError::Bus)
    }

    /// Writes (at most) one page and waits for the write cycle to complete, verifying the data
    async fn write_page(&mut self, offset: usize, data: &[u8]) -> Result<(), EepromError> {
        let mut frame = [0u8; 66];
        frame[..2].copy_from_slice(&(offset as u16).to_be_bytes());
        frame[2..2 + data.len()].copy_from_slice(data);
        self.i2c
            .write(self.address, &frame[..2 + data.len()])
            .await
            .map_err(|_| EepromError::Bus)?;
        // While the internal write cycle is in progress the chip does not acknowledge its
        // address: the read-back fails until the cycle ends
        let mut readback = [0u8; 64];
        let readback = &mut readback[..data.len()];
        for _ in 0..WRITE_CYCLE_TIMEOUT_MS {
            self.delay.delay_ms(1).await;
            if self.raw_read(offset, readback).await.is_ok() {
                return match readback == data {
                    true => Ok(()),
                    false => Err(EepromError::Bus),
                };
            }
        }
        Err(EepromError::Timeout)
    }
}

impl<I2C, const SIZE: usize, const PAGE: usize, D> AsyncEeprom for I2cEeprom24x<I2C, SIZE, PAGE, D>
where
    I2C: embedded_hal_async::i2c::I2c,
    D: DelayNs,
{
    fn capacity(&self) -> usize {
        SIZE
    }

    async fn read(&mut self, offset: usize, buffer: &mut [u8]) -> Result<(), EepromError> {
        if offset.checked_add(buffer.len()).is_none_or(|end| end > SIZE) {
            return Err(EepromError::OutOfBounds);
        }
        // Sequential reads have no page boundaries, but some I2C peripherals limit the length
        // of a single transfer
        let mut done = 0;
        for chunk in buffer.chunks_mut(128) {
            self.raw_read(offset + done, chunk).await?;
            done += chunk.len();
        }
        Ok(())
    }

    async fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), EepromError> {
        if offset.checked_add(data.len()).is_none_or(|end| end > SIZE) {
            return Err(EepromError::OutOfBounds);
        }
        let mut current = [0u8; 64];
        let mut done = 0;
        while done < data.len() {
            let at = offset + done;
            // A page write must not cross a page boundary (it would wrap around the page)
            let len = (PAGE - at % PAGE).min(data.len() - done);
            let chunk = &data[done..done + len];
            let current = &mut current[..len];
            // Differential write: unchanged pages are not rewritten
            if self.raw_read(at, current).await.is_err() || current != chunk {
                self.write_page(at, chunk).await?;
            }
            done += len;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_async::i2c::{ErrorType, Operation, SevenBitAddress};

    /// A simulated 24C32: a write cycle keeps the chip busy (NACK) for `busy_polls` accesses
    struct FakeChip {
        mem: [u8; 4096],
        pointer: usize,
        busy: u8,
        busy_polls: u8,
        page_writes: usize,
    }

    #[derive(Debug)]
    struct Nack;
    impl embedded_hal_async::i2c::Error for Nack {
        fn kind(&self) -> embedded_hal_async::i2c::ErrorKind {
            embedded_hal_async::i2c::ErrorKind::NoAcknowledge(
                embedded_hal_async::i2c::NoAcknowledgeSource::Address,
            )
        }
    }

    impl ErrorType for FakeChip {
        type Error = Nack;
    }

    impl embedded_hal_async::i2c::I2c<SevenBitAddress> for FakeChip {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            assert_eq!(address, DEFAULT_I2C_24CXX_ADDRESS);
            if self.busy > 0 {
                self.busy -= 1;
                return Err(Nack);
            }
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        self.pointer = u16::from_be_bytes([bytes[0], bytes[1]]) as usize;
                        if bytes.len() > 2 {
                            let page = self.pointer - self.pointer % 32;
                            for (i, b) in bytes[2..].iter().enumerate() {
                                // Wraps around the page, as the real chip does
                                self.mem[page + (self.pointer % 32 + i) % 32] = *b;
                            }
                            self.page_writes += 1;
                            self.busy = self.busy_polls;
                        }
                    }
                    Operation::Read(buffer) => {
                        for b in buffer.iter_mut() {
                            *b = self.mem[self.pointer % 4096];
                            self.pointer += 1;
                        }
                    }
                }
            }
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    fn chip() -> I2cEeprom24x<FakeChip, 4096, 32, NoDelay> {
        I2cEeprom24x::new_with_delay(
            FakeChip {
                mem: [0xFF; 4096],
                pointer: 0,
                busy: 0,
                busy_polls: 3,
                page_writes: 0,
            },
            DEFAULT_I2C_24CXX_ADDRESS,
            NoDelay,
        )
    }

    #[futures_test::test]
    async fn write_read_across_pages() {
        let mut e = chip();
        assert_eq!(e.capacity(), 4096);
        let data: [u8; 100] = core::array::from_fn(|i| i as u8);
        // Unaligned: spans 5 pages (20 + 32 + 32 + 16)
        e.write(12, &data).await.unwrap();
        let mut back = [0u8; 100];
        e.read(12, &mut back).await.unwrap();
        assert_eq!(back, data);
        assert_eq!(e.i2c.page_writes, 4);
        // Rewriting the same content does not write any page
        e.write(12, &data).await.unwrap();
        assert_eq!(e.i2c.page_writes, 4);
        // A single changed byte rewrites a single page
        let mut changed = data;
        changed[50] = 0;
        e.write(12, &changed).await.unwrap();
        assert_eq!(e.i2c.page_writes, 5);
    }

    #[futures_test::test]
    async fn out_of_bounds_and_timeout() {
        let mut e = chip();
        assert_eq!(e.write(4090, &[0u8; 7]).await, Err(EepromError::OutOfBounds));
        let mut b = [0u8; 2];
        assert_eq!(e.read(4095, &mut b).await, Err(EepromError::OutOfBounds));
        e.i2c.busy_polls = 100;
        assert_eq!(e.write(0, &[1u8]).await, Err(EepromError::Timeout));
    }
}
