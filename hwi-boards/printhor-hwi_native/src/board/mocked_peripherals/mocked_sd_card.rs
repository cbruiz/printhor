use printhor_hwa_common as hwa;
use std::cell::RefCell;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;
use hwa::traits::{AsyncSDBlockDevice, SDBlockDevice};
use embedded_sdmmc::{Block, BlockCount, BlockIdx};

#[allow(unused)]
#[derive(Debug, Clone)]
enum Error {
    #[allow(unused)]
    Filesystem(embedded_sdmmc::Error<embedded_sdmmc::SdCardError>),
    #[allow(unused)]
    Disk(embedded_sdmmc::SdCardError),
}

impl From<embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> for Error {
    fn from(value: embedded_sdmmc::Error<embedded_sdmmc::SdCardError>) -> Error {
        Error::Filesystem(value)
    }
}

impl From<embedded_sdmmc::SdCardError> for Error {
    fn from(value: embedded_sdmmc::SdCardError) -> Error {
        Error::Disk(value)
    }
}

pub struct MockedSDCardBlockDevice {
    /// None when the image could not be opened: the card behaves as not inserted
    file: RefCell<Option<File>>,
    /// Blocks written (copy on write): the image is never modified
    written: RefCell<std::collections::HashMap<u32, [u8; 512]>>,
}

impl MockedSDCardBlockDevice {
    /// Never fails: a missing image is reported and then behaves as a card not inserted (every
    /// access fails), so the SD commands report errors instead of the firmware panicking
    pub fn new<P>(image_path: P) -> MockedSDCardBlockDevice
        where P: AsRef<Path>,
    {
        let file = match File::open(image_path) {
            Ok(f) => Some(f),
            Err(_e) => {
                hwa::error!("[sdcard] Unable to open the card image: {:?}. No card", _e);
                None
            }
        };
        MockedSDCardBlockDevice { file: RefCell::new(file), written: RefCell::new(Default::default()) }
    }

    fn no_card() -> std::io::Error {
        std::io::Error::new(std::io::ErrorKind::NotFound, "no card")
    }
}

impl AsyncSDBlockDevice for MockedSDCardBlockDevice {
    type Lock = ();

    fn do_lock(&self) -> impl core::future::Future<Output = Option<()>> + use<> {
        core::future::ready(Some(()))
    }
    fn do_hold(&self, _lock: ()) -> Result<(), ()> {
        Ok(())
    }
    fn do_release(&self) -> Result<(), ()> {
        Ok(())
    }
}

impl SDBlockDevice for MockedSDCardBlockDevice {
    type Error = std::io::Error;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), Self::Error> {
        let mut guard = self.file.borrow_mut();
        let file = guard.as_mut().ok_or_else(Self::no_card)?;
        let written = self.written.borrow();
        for (i, block) in blocks.iter_mut().enumerate() {
            let idx = start_block_idx.0 + i as u32;
            match written.get(&idx) {
                Some(contents) => block.contents = *contents,
                None => {
                    file.seek(SeekFrom::Start(BlockIdx(idx).into_bytes()))?;
                    file.read_exact(&mut block.contents)?;
                }
            }
        }
        Ok(())
    }

    /// Kept in memory (copy on write): the card image of the repository is never modified
    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        if self.file.borrow().is_none() {
            return Err(Self::no_card());
        }
        let mut written = self.written.borrow_mut();
        for (i, block) in blocks.iter().enumerate() {
            written.insert(start_block_idx.0 + i as u32, block.contents);
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        let guard = self.file.borrow();
        let file = guard.as_ref().ok_or_else(Self::no_card)?;
        let num_blocks = file.metadata()?.len() / 512;
        Ok(BlockCount(num_blocks as u32))
    }
}
