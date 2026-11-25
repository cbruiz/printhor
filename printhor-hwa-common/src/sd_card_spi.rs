//! Implementation borrowed and adapted from embedded_sdmmc to provide a pseudo-async approach

use crate as hwa;
use core::cell::RefCell;
#[cfg(feature = "with-sd-card")]
use embedded_hal_0::prelude::_embedded_hal_blocking_delay_DelayUs;
#[cfg(feature = "with-sd-card")]
use embedded_sdmmc::sdcard::spi::AcquireOpts;
#[cfg(feature = "with-sd-card")]
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};

#[cfg(feature = "with-sd-card")]
use embedded_sdmmc::embedded_sdmmc_types::sdcard::{
    DATA_RES_ACCEPTED, DATA_RES_MASK, DATA_START_BLOCK, ERROR_OK, R1_IDLE_STATE,
    R1_ILLEGAL_COMMAND, R1_READY_STATE, STOP_TRAN_TOKEN, WRITE_MULTIPLE_TOKEN, crc7, crc16,
    csd::Csd,
};

/// Highest SPI clock allowed while the card is initialized (SD Physical Layer spec: 100-400 kHz
/// until the end of the identification). Many cards do not answer at all at a higher one.
#[cfg(feature = "with-sd-card")]
pub const SD_INIT_FREQUENCY: u32 = 400_000;
#[cfg(feature = "with-sd-card")]
use sd_cmd::*;

/// SD command indices (SD Physical Layer spec). embedded-sdmmc >= 0.10 no longer exports them as
/// plain constants (it uses `CmdId`/`AcmdId`), so they are kept here with the same values.
#[allow(dead_code)]
mod sd_cmd {
    /// GO_IDLE_STATE - init card in spi mode if CS low
    pub const CMD0: u8 = 0x00;
    /// SEND_IF_COND - verify SD Memory Card interface operating condition
    pub const CMD8: u8 = 0x08;
    /// SEND_CSD - read the Card Specific Data (CSD register)
    pub const CMD9: u8 = 0x09;
    /// STOP_TRANSMISSION - end multiple block read sequence
    pub const CMD12: u8 = 0x0C;
    /// READ_SINGLE_BLOCK - read a single data block from the card
    pub const CMD17: u8 = 0x11;
    /// READ_MULTIPLE_BLOCK - read a multiple data blocks from the card
    pub const CMD18: u8 = 0x12;
    /// SEND_STATUS - read the card status register
    pub const CMD13: u8 = 0x0D;
    /// WRITE_BLOCK - write a single data block to the card
    pub const CMD24: u8 = 0x18;
    /// WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION
    pub const CMD25: u8 = 0x19;
    /// SET_WR_BLK_ERASE_COUNT - number of blocks to pre-erase before a multiple block write
    pub const ACMD23: u8 = 0x17;
    /// APP_CMD - escape for application specific command
    pub const CMD55: u8 = 0x37;
    /// READ_OCR - read the OCR register of a card
    pub const CMD58: u8 = 0x3A;
    /// CRC_ON_OFF - enable or disable CRC checking
    pub const CMD59: u8 = 0x3B;
    /// SD_SEND_OP_COMD - Sends host capacity support information and activates the card's
    /// initialization process
    pub const ACMD41: u8 = 0x29;
}


/// Adapter to manage multiple drivers
pub struct SPIAdapter<H, PIN>
where
    H: hwa::AsyncMutexStrategy + 'static,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Transfer<u8>,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Write<u8>,
    PIN: embedded_hal_0::digital::v2::OutputPin,
{
    spi: hwa::StaticAsyncController<H>,
    cs: RefCell<PIN>,
    card_type: RefCell<Option<CardType>>,
    options: AcquireOpts,
    /// Sets the SPI clock (Hz) of the bus, and the clock to use once the card is initialized (see
    /// [Self::with_frequency_control])
    frequency: Option<(fn(&mut <H as hwa::AsyncMutexStrategy>::Resource, u32), u32)>,
}

impl<H, PIN> SPIAdapter<H, PIN>
where
    H: hwa::AsyncMutexStrategy + 'static,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Transfer<u8>,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Write<u8>,
    PIN: embedded_hal_0::digital::v2::OutputPin,
{
    pub fn new(spi: hwa::StaticAsyncController<H>, cs: PIN) -> Self {
        Self {
            spi,
            cs: RefCell::new(cs),
            card_type: RefCell::new(None),
            options: AcquireOpts::default(),
            frequency: None,
        }
    }

    /// The card is initialized at [SD_INIT_FREQUENCY] and then used at `frequency`: `set` changes
    /// the clock of the SPI bus (a board function, e.g. `Spi::set_config`). Without it the card is
    /// initialized at the clock the bus was configured with, which many cards do not accept.
    pub fn with_frequency_control(
        mut self,
        set: fn(&mut <H as hwa::AsyncMutexStrategy>::Resource, u32),
        frequency: u32,
    ) -> Self {
        self.frequency = Some((set, frequency));
        self
    }

    fn set_frequency(&self, hz: u32) {
        if let Some((set, _)) = self.frequency {
            let _ = self.spi.apply_or_error(
                |spi| {
                    set(spi, hz);
                    Ok(())
                },
                (),
            );
        }
    }

    pub async fn retain(&self) {
        let _ = self.spi.retain().await;
    }

    pub fn release(&self) {
        let _ = self.spi.release();
    }

    /// Read the 'card specific data' block.
    fn read_csd(&self) -> Result<Csd, Error> {
        if self.card_type.borrow().is_none() {
            return Err(Error::CardNotFound);
        }
        if self.card_command(CMD9, 0)? != 0 {
            return Err(Error::RegisterReadError);
        }
        // The CSD layout is selected from its CSD_STRUCTURE field (as embedded-sdmmc 0.10 does)
        let mut csd_raw = [0u8; 16];
        self.read_data(&mut csd_raw)?;
        Csd::new(&csd_raw).map_err(|_| Error::RegisterReadError)
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
        hwa::debug!("CSCARD_SPI: Acquiring");
        let f = |s: &Self| {
            // Assume it hasn't worked
            let mut card_type;
            hwa::debug!("CSCARD_SPI: Reset card..");
            // Supply minimum of 74 clock cycles without CS asserted.
            s.cs_high()?;
            s.write_bytes(&[0xFF; 10])?;
            // Assert CS
            s.cs_low()?;
            // Enter SPI mode.
            let mut delay = Delay::new_command();
            for _attempts in 1.. {
                hwa::debug!("Enter SPI mode, attempt: {}..", _attempts);
                match s.card_command(CMD0, 0) {
                    Err(Error::TimeoutCommand(0)) => {
                        hwa::debug!("\tEnter SPI mode, timeout");
                        // Try again?
                        //warn!("Timed out, trying again...");
                        // Try flushing the card as done here: https://github.com/greiman/SdFat/blob/master/src/SdCard/SdSpiCard.cpp#L170,
                        // https://github.com/rust-embedded-community/embedded-sdmmc-rs/pull/65#issuecomment-1270709448
                        for _ in 0..0xFF {
                            s.write_byte(0xFF)?;
                        }
                    }
                    Err(e) => {
                        hwa::debug!("\tEnter SPI mode: ERROR");
                        return Err(e);
                    }
                    Ok(R1_IDLE_STATE) => {
                        hwa::debug!("\tEnter SPI mode: IDLE");
                        break;
                    }
                    Ok(_r) => {
                        // Try again
                        hwa::debug!("\tEnter SPI mode, got response {:x}. Trying again", _r);
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
            hwa::debug!("Card type: {:?}", card_type);
            s.card_type.borrow_mut().replace(card_type);
            Ok(())
        };
        // The identification runs at the low clock; the bus goes back to its working clock
        // whatever the result (it is shared)
        self.set_frequency(SD_INIT_FREQUENCY);
        let result = f(self);
        let cs = self.cs_high();
        let _ = self.read_byte();
        if let Some((_, hz)) = self.frequency {
            self.set_frequency(hz);
        }
        cs?;
        result
    }

    /// Write an arbitrary number of bytes to the card using the SD protocol and an optional CRC
    fn write_data(&self, token: u8, buffer: &[u8]) -> Result<(), Error> {
        self.write_byte(token)?;
        self.write_bytes(buffer)?;
        let crc_bytes = if self.options.use_crc {
            crc16(buffer).to_be_bytes()
        } else {
            [0xFF, 0xFF]
        };
        // These two bytes are always sent. They are either a valid CRC, or junk, depending on
        // whether CRC mode was enabled.
        self.write_bytes(&crc_bytes)?;
        let status = self.read_byte()?;
        if (status & DATA_RES_MASK) != DATA_RES_ACCEPTED {
            Err(Error::WriteError)
        } else {
            Ok(())
        }
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
        // embedded-sdmmc >= 0.10 returns the raw 7-bit CRC: the byte is the CRC and the end bit
        // (0.8 returned that byte). With the raw CRC every command but CMD0 (still in SD mode
        // only for some cards) is rejected with a CRC error, e.g. CMD8: the card never opened
        buf[5] = (crc7(&buf[0..5]) << 1) | 1;

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
        let mut in_out: [u8; 1] = [out];
        let _result = self.spi.apply_or_error(
            |spi| {
                use embedded_hal_0::blocking::spi::Transfer;
                spi.transfer(&mut in_out).map_err(|_| Error::Transport)
            },
            Error::Transport,
        )?;
        Ok(in_out[0])
    }

    /// Send mutiple bytes and ignore what comes back over the SPI bus.
    fn write_bytes(&self, out: &[u8]) -> Result<(), Error> {
        self.spi.apply_or_error(
            |spi| {
                use embedded_hal_0::blocking::spi::Write;
                spi.write(out).map_err(|_e| Error::Transport)
            },
            Error::Transport,
        )
    }

    /// Send multiple bytes and replace them with what comes back over the SPI bus.
    fn transfer_bytes<'a>(&self, in_out: &'a mut [u8]) -> Result<&'a [u8], Error> {
        self.spi.apply_or_error(
            move |spi| {
                use embedded_hal_0::blocking::spi::Transfer;
                spi.transfer(in_out).map_err(|_e| Error::Transport)
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

impl<H, PIN> BlockDevice for SPIAdapter<H, PIN>
where
    H: hwa::AsyncMutexStrategy,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Transfer<u8>,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Write<u8>,
    PIN: embedded_hal_0::digital::v2::OutputPin,
{
    type Error = Error;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
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

    /// Writes the blocks (as embedded-sdmmc's `SdCard`): CMD24 for one block, ACMD23 + CMD25
    /// for several
    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        self.check_init()?;
        let start_idx = match *self.card_type.borrow() {
            Some(CardType::SD1 | CardType::SD2) => start_block_idx.0 * 512,
            Some(CardType::SDHC) => start_block_idx.0,
            None => return Err(Error::CardNotFound),
        };
        self.with_chip_select(|s| {
            if blocks.len() == 1 {
                s.card_command(CMD24, start_idx)?;
                s.write_data(DATA_START_BLOCK, &blocks[0].contents)?;
                s.wait_not_busy(Delay::new_write())?;
                if s.card_command(CMD13, 0)? != 0x00 {
                    return Err(Error::WriteError);
                }
                if s.read_byte()? != 0x00 {
                    return Err(Error::WriteError);
                }
            } else {
                // Pre-erase (faster multiple block writes on some cards)
                s.card_acmd(ACMD23, blocks.len() as u32)?;
                s.wait_not_busy(Delay::new_write())?;
                s.card_command(CMD25, start_idx)?;
                for block in blocks.iter() {
                    s.wait_not_busy(Delay::new_write())?;
                    s.write_data(WRITE_MULTIPLE_TOKEN, &block.contents)?;
                }
                s.wait_not_busy(Delay::new_write())?;
                s.write_byte(STOP_TRAN_TOKEN)?;
            }
            Ok(())
        })
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        let num_blocks = self.with_chip_select(|s| {
            let csd = s.read_csd()?;
            //debug!("CSD: {:?}", csd);
            Ok(csd.card_capacity_blocks())
        })?;
        Ok(BlockCount(num_blocks))
    }
}

impl<H, PIN> hwa::traits::AsyncSDBlockDevice for SPIAdapter<H, PIN>
where
    H: hwa::AsyncMutexStrategy,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Transfer<u8>,
    <H as hwa::AsyncMutexStrategy>::Resource: embedded_hal_0::blocking::spi::Write<u8>,
    PIN: embedded_hal_0::digital::v2::OutputPin,
{
    type Lock = hwa::AsyncMutexGuard<
        'static,
        <H as hwa::AsyncMutexStrategy>::AsyncMutexType,
        <H as hwa::AsyncMutexStrategy>::Resource,
    >;

    /// Locks the SPI bus (its static mutex) without borrowing `self`. It used to retain a clone
    /// of the controller, whose own holder dropped the guard right away: `self.spi` was never
    /// retained and every transfer failed with [Error::Transport] (the volume never opened)
    fn do_lock(&self) -> impl core::future::Future<Output = Option<Self::Lock>> + use<H, PIN> {
        self.spi.lock_owned()
    }

    fn do_hold(&self, lock: Self::Lock) -> Result<(), ()> {
        self.spi.hold(lock)
    }

    fn do_release(&self) -> Result<(), ()> {
        self.release();
        Ok(())
    }

    fn reset_card(&mut self) {
        self.card_type.replace(None);
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

    /// Error writing to the card (rejected data block or bad status after the write)
    WriteError,

    /// Can't perform this operation with the card in this state
    #[allow(unused)]
    BadState,

    /// Couldn't find the card
    CardNotFound,
    /// Couldn't set a GPIO pin
    GpioError,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

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
