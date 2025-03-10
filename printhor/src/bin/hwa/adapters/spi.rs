//! Implementation borrowed and adapted from embedded_sdmmc to provide a pseudo-async approach

use crate::hwa;
use core::cell::RefCell;
#[cfg(feature = "with-sd-card")]
use embedded_hal_02::prelude::_embedded_hal_blocking_delay_DelayUs;
#[cfg(feature = "with-sd-card")]
use embedded_sdmmc::sdcard::AcquireOpts;
#[cfg(feature = "with-sd-card")]
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};

#[cfg(feature = "with-sd-card")]
use embedded_sdmmc::sdcard::proto::*;

/// Adapter to manage multiple drivers
pub struct SPIAdapter<PIN: embedded_hal_02::digital::v2::OutputPin> {
    spi: hwa::device::SpiDeviceRef,
    cs: RefCell<PIN>,
    card_type: RefCell<Option<CardType>>,
    options: AcquireOpts,
}

impl<PIN: embedded_hal_02::digital::v2::OutputPin> SPIAdapter<PIN> {
    pub fn new(spi: hwa::device::SpiDeviceRef, cs: PIN) -> Self {
        Self {
            spi,
            cs: RefCell::new(cs),
            card_type: RefCell::new(None),
            options: AcquireOpts::default(),
        }
    }

    #[inline]
    pub async fn retain(&self) {
        #[cfg(feature = "sd-card-uses-spi")]
        self.spi.retain().await;
    }

    #[inline]
    pub async fn release(&self) {
        #[cfg(feature = "sd-card-uses-spi")]
        self.spi.release().await;
    }

    /// Read the 'card specific data' block.
    fn read_csd(&self) -> Result<Csd, Error> {
        match *self.card_type.borrow() {
            Some(CardType::SD1) => {
                let mut csd = CsdV1::new();
                if self.card_command(CMD9, 0)? != 0 {
                    return Err(Error::RegisterReadError);
                }
                self.read_data(&mut csd.data)?;
                Ok(Csd::V1(csd))
            }
            Some(CardType::SD2 | CardType::SDHC) => {
                let mut csd = CsdV2::new();
                if self.card_command(CMD9, 0)? != 0 {
                    return Err(Error::RegisterReadError);
                }
                self.read_data(&mut csd.data)?;
                Ok(Csd::V2(csd))
            }
            None => Err(Error::CardNotFound),
        }
    }

    /// Read an arbitrary number of bytes from the card using the SD Card
    /// protocol and an optional CRC. Always fills the given buffer, so make
    /// sure it's the right size.
    fn read_data(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // Get first non-FF byte.
        let mut delay = Delay::new_read();
        let status = loop {
            let s = self.read_byte()?;
            if s != 0xFF {
                break s;
            }
            delay.delay(Error::TimeoutReadBuffer)?;
        };
        if status != DATA_START_BLOCK {
            return Err(Error::ReadError);
        }

        for b in buffer.iter_mut() {
            *b = 0xFF;
        }
        self.transfer_bytes(buffer)?;

        // These two bytes are always sent. They are either a valid CRC, or
        // junk, depending on whether CRC mode was enabled.
        let mut crc_bytes = [0xFF; 2];
        self.transfer_bytes(&mut crc_bytes)?;
        if self.options.use_crc {
            let crc = u16::from_be_bytes(crc_bytes);
            let calc_crc = crc16(buffer);
            if crc != calc_crc {
                return Err(Error::CrcError(crc, calc_crc));
            }
        }

        Ok(())
    }

    fn cs_high(&self) -> Result<(), Error> {
        self.cs
            .borrow_mut()
            .set_high()
            .map_err(|_| Error::GpioError)
    }

    fn cs_low(&self) -> Result<(), Error> {
        self.cs.borrow_mut().set_low().map_err(|_| Error::GpioError)
    }

    /// Check the card is initialised.
    fn check_init(&self) -> Result<(), Error> {
        if self.card_type.borrow().is_none() {
            // If we don't know what the card type is, try and initialise the
            // card. This will tell us what type of card it is.
            self.acquire()
        } else {
            Ok(())
        }
    }

    fn acquire(&self) -> Result<(), Error> {
        defmt::info!("CSCARD_SPI: Acquiring");
        let f = |s: &Self| {
            // Assume it hasn't worked
            let mut card_type;
            defmt::trace!("CSCARD_SPI: Reset card..");
            // Supply minimum of 74 clock cycles without CS asserted.
            s.cs_high()?;
            s.write_bytes(&[0xFF; 10])?;
            // Assert CS
            s.cs_low()?;
            // Enter SPI mode.
            let mut delay = Delay::new_command();
            for _attempts in 1.. {
                defmt::trace!("Enter SPI mode, attempt: {}..", _attempts);
                match s.card_command(CMD0, 0) {
                    Err(Error::TimeoutCommand(0)) => {
                        defmt::trace!("\tEnter SPI mode, timeout");
                        // Try again?
                        //warn!("Timed out, trying again...");
                        // Try flushing the card as done here: https://github.com/greiman/SdFat/blob/master/src/SdCard/SdSpiCard.cpp#L170,
                        // https://github.com/rust-embedded-community/embedded-sdmmc-rs/pull/65#issuecomment-1270709448
                        for _ in 0..0xFF {
                            s.write_byte(0xFF)?;
                        }
                    }
                    Err(e) => {
                        defmt::trace!("\tEnter SPI mode: ERROR");
                        return Err(e);
                    }
                    Ok(R1_IDLE_STATE) => {
                        defmt::trace!("\tEnter SPI mode: IDLE");
                        break;
                    }
                    Ok(_r) => {
                        // Try again
                        defmt::trace!("\tEnter SPI mode, got response {:x}. Trying again", _r);
                        //warn!("Got response: {:x}, trying again..", _r);
                    }
                }

                delay.delay(Error::CardNotFound)?;
            }
            // Enable CRC
            //debug!("Enable CRC: {}", s.options.use_crc);
            // "The SPI interface is initialized in the CRC OFF mode in default"
            // -- SD Part 1 Physical Layer Specification v9.00, Section 7.2.2 Bus Transfer Protection
            if s.options.use_crc && s.card_command(CMD59, 1)? != R1_IDLE_STATE {
                return Err(Error::CantEnableCRC);
            }
            // Check card version
            let mut delay = Delay::new_command();
            let arg = loop {
                if s.card_command(CMD8, 0x1AA)? == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
                    card_type = CardType::SD1;
                    break 0;
                }
                let mut buffer = [0xFF; 4];
                s.transfer_bytes(&mut buffer)?;
                let status = buffer[3];
                if status == 0xAA {
                    card_type = CardType::SD2;
                    break 0x4000_0000;
                }
                delay.delay(Error::TimeoutCommand(CMD8))?;
            };

            let mut delay = Delay::new_command();
            while s.card_acmd(ACMD41, arg)? != R1_READY_STATE {
                delay.delay(Error::TimeoutACommand(ACMD41))?;
            }

            if card_type == CardType::SD2 {
                if s.card_command(CMD58, 0)? != 0 {
                    return Err(Error::Cmd58Error);
                }
                let mut buffer = [0xFF; 4];
                s.transfer_bytes(&mut buffer)?;
                if (buffer[0] & 0xC0) == 0xC0 {
                    card_type = CardType::SDHC;
                }
                // Ignore the other three bytes
            }
            hwa::info!("Card type: {:?}", card_type);
            s.card_type.borrow_mut().replace(card_type);
            Ok(())
        };
        let result = f(self);
        self.cs_high()?;
        let _ = self.read_byte();
        result
    }

    /// Perform a function that might error with the chipselect low.
    /// Always releases the chipselect, even if the function errors.
    fn with_chip_select<F, T>(&self, func: F) -> Result<T, Error>
    where
        F: FnOnce(&Self) -> Result<T, Error>,
    {
        self.cs_low()?;
        let result = func(self);
        self.cs_high()?;
        result
    }
    fn card_acmd(&self, command: u8, arg: u32) -> Result<u8, Error> {
        self.card_command(CMD55, 0)?;
        self.card_command(command, arg)
    }

    /// Perform a command.
    fn card_command(&self, command: u8, arg: u32) -> Result<u8, Error> {
        if command != CMD0 && command != CMD12 {
            self.wait_not_busy(Delay::new_command())?;
        }

        let mut buf = [
            0x40 | command,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        self.write_bytes(&buf)?;

        // skip stuff byte for stop read
        if command == CMD12 {
            let _result = self.read_byte()?;
        }

        let mut delay = Delay::new_command();
        loop {
            let result = self.read_byte()?;
            if (result & 0x80) == ERROR_OK {
                return Ok(result);
            }
            delay.delay(Error::TimeoutCommand(command))?;
        }
    }

    /// Receive a byte from the SPI bus by clocking out an 0xFF byte.
    fn read_byte(&self) -> Result<u8, Error> {
        self.transfer_byte(0xFF)
    }

    /// Send a byte over the SPI bus and ignore what comes back.
    fn write_byte(&self, out: u8) -> Result<(), Error> {
        let _ = self.transfer_byte(out)?;
        Ok(())
    }

    /// Send one byte and receive one byte over the SPI bus.
    fn transfer_byte(&self, out: u8) -> Result<u8, Error> {
        let mut received: [u8; 1] = [0];
        Ok(self
            .spi
            .apply_result(
                |spi| {
                    spi.blocking_transfer(&mut received, &[out])
                        .map_err(|_| Error::Transport)
                },
                Error::Transport,
            )
            .map(|_| received[0])?)
    }

    /// Send mutiple bytes and ignore what comes back over the SPI bus.
    fn write_bytes(&self, out: &[u8]) -> Result<(), Error> {
        self.spi.apply_result(
            |spi| {
                defmt::debug!("SPI: write_bytes ....");
                spi.blocking_write(out).map_err(|_e| Error::Transport)
            },
            Error::Transport,
        )
    }

    /// Send multiple bytes and replace them with what comes back over the SPI bus.
    fn transfer_bytes(&self, in_out: &mut [u8]) -> Result<(), Error> {
        self.spi.apply_result(
            |spi| {
                spi.blocking_transfer_in_place(in_out)
                    .map_err(|_e| Error::Transport)
            },
            Error::Transport,
        )
    }

    /// Spin until the card returns 0xFF, or we spin too many times and
    /// timeout.
    fn wait_not_busy(&self, mut delay: Delay) -> Result<(), Error> {
        //crate::info!("wait_not_busy()");
        loop {
            let s = self.read_byte()?;
            if s == 0xFF {
                break;
            }
            delay.delay(Error::TimeoutWaitNotBusy)?;
        }
        Ok(())
    }
}

impl<PIN: embedded_hal_02::digital::v2::OutputPin> BlockDevice for SPIAdapter<PIN> {
    type Error = Error;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
        _reason: &str,
    ) -> Result<(), Self::Error> {
        self.check_init()?;
        let start_idx = match *self.card_type.borrow() {
            Some(CardType::SD1 | CardType::SD2) => start_block_idx.0 * 512,
            Some(CardType::SDHC) => start_block_idx.0,
            None => return Err(Error::CardNotFound),
        };
        self.with_chip_select(|s| {
            if blocks.len() == 1 {
                // Start a single-block read
                s.card_command(CMD17, start_idx)?;
                s.read_data(&mut blocks[0].contents)?;
            } else {
                // Start a multi-block read
                s.card_command(CMD18, start_idx)?;
                for block in blocks.iter_mut() {
                    s.read_data(&mut block.contents)?;
                }
                // Stop the read
                s.card_command(CMD12, 0)?;
            }
            Ok(())
        })
    }

    fn write(&self, _blocks: &[Block], _start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        todo!()
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        let num_blocks = self.with_chip_select(|s| {
            let csd = s.read_csd()?;
            //debug!("CSD: {:?}", csd);
            match csd {
                Csd::V1(ref contents) => Ok(contents.card_capacity_blocks()),
                Csd::V2(ref contents) => Ok(contents.card_capacity_blocks()),
            }
        })?;
        Ok(BlockCount(num_blocks))
    }
}

#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum Error {
    /// We got an error from the SPI peripheral
    Transport,
    /// We failed to enable CRC checking on the SD card
    CantEnableCRC,
    /// We didn't get a response when reading data from the card
    TimeoutReadBuffer,
    /// We didn't get a response when waiting for the card to not be busy
    TimeoutWaitNotBusy,
    /// We didn't get a response when executing this command
    TimeoutCommand(u8),
    /// We didn't get a response when executing this application-specific command
    TimeoutACommand(u8),
    /// We got a bad response from Command 58
    Cmd58Error,
    /// We failed to read the Card Specific Data register
    RegisterReadError,
    /// We got a CRC mismatch (card gave us, we calculated)
    CrcError(u16, u16),
    /// Error reading from the card
    ReadError,

    /// Error writing to the card
    #[allow(unused)]
    WriteError,

    /// Can't perform this operation with the card in this state
    #[allow(unused)]
    BadState,

    /// Couldn't find the card
    CardNotFound,
    /// Couldn't set a GPIO pin
    GpioError,
}

#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CardType {
    /// A standard-capacity SD Card supporting v1.x of the standard.
    ///
    /// Uses byte-addressing internally, so limited to 2GiB in size.
    SD1,
    /// A standard-capacity SD Card supporting v2.x of the standard.
    ///
    /// Uses byte-addressing internally, so limited to 2GiB in size.
    SD2,
    /// A high-capacity 'SDHC' Card.
    ///
    /// Uses block-addressing internally to support capacities above 2GiB.
    SDHC,
}

/// This an object you can use to busy-wait with a timeout.
///
/// Will let you call `delay` up to `max_retries` times before `delay` returns
/// an error.
struct Delay {
    retries_left: u32,
}

impl Delay {
    /// The default number of retries for a read operation.
    ///
    /// At ~10us each this is ~100ms.
    ///
    /// See `Part1_Physical_Layer_Simplified_Specification_Ver9.00-1.pdf` Section 4.6.2.1
    pub const DEFAULT_READ_RETRIES: u32 = 10_000;

    /// The default number of retries for a write operation.
    ///
    /// At ~10us each this is ~500ms.
    ///
    /// See `Part1_Physical_Layer_Simplified_Specification_Ver9.00-1.pdf` Section 4.6.2.2
    #[allow(unused)]
    pub const DEFAULT_WRITE_RETRIES: u32 = 50_000;

    /// The default number of retries for a control command.
    ///
    /// At ~10us each this is ~100ms.
    ///
    /// No value is given in the specification, so we pick the same as the read timeout.
    pub const DEFAULT_COMMAND_RETRIES: u32 = 10_000;

    /// Create a new Delay object with the given maximum number of retries.
    fn new(max_retries: u32) -> Delay {
        Delay {
            retries_left: max_retries,
        }
    }

    /// Create a new Delay object with the maximum number of retries for a read operation.
    fn new_read() -> Delay {
        Delay::new(Self::DEFAULT_READ_RETRIES)
    }

    /// Create a new Delay object with the maximum number of retries for a write operation.
    #[allow(unused)]
    fn new_write() -> Delay {
        Delay::new(Self::DEFAULT_WRITE_RETRIES)
    }

    /// Create a new Delay object with the maximum number of retries for a command operation.
    fn new_command() -> Delay {
        Delay::new(Self::DEFAULT_COMMAND_RETRIES)
    }

    /// Wait for a while.
    ///
    /// Checks the retry counter first, and if we hit the max retry limit, the
    /// value `err` is returned. Otherwise, we wait for 10us and then return
    /// `Ok(())`.
    fn delay(&mut self, err: Error) -> Result<(), Error> {
        if self.retries_left == 0 {
            Err(err)
        } else {
            //block_on(embassy_time::Timer::after(Duration::from_micros(10)));

            let mut delay = embassy_time::Delay;
            delay.delay_us(10u8);
            self.retries_left -= 1;
            Ok(())
        }
    }
}
