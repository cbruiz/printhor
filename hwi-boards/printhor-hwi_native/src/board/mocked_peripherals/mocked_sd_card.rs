use printhor_hwa_common as hwa;
use std::cell::RefCell;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::Path;
use hwa::traits::{AsyncSDBlockDevice, SDBlockDevice};
use embedded_sdmmc::{Block, BlockCount, BlockIdx};

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
    file: RefCell<File>,
}

impl MockedSDCardBlockDevice {
    pub fn new<P>(device_name: P) -> Result<MockedSDCardBlockDevice, std::io::Error>
        where P: AsRef<Path>,
    {
        hwa::debug!("Cwd: Opening {}/{:?}",
            std::env::current_dir().as_ref().unwrap().to_str().unwrap(),
            device_name.as_ref().to_str().unwrap());

        Ok(MockedSDCardBlockDevice {
            file: RefCell::new(File::open(device_name)?),
        })
    }
}

impl AsyncSDBlockDevice for MockedSDCardBlockDevice {
    async fn do_retain(&self) -> Result<(), ()> {
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
        _reason: &str,
    ) -> Result<(), Self::Error> {
        self.file
            .borrow_mut()
            .seek(SeekFrom::Start(start_block_idx.into_bytes()))?;
        for block in blocks.iter_mut() {
            self.file.borrow_mut().read_exact(&mut block.contents)?;
        }
        Ok(())
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        self.file
            .borrow_mut()
            .seek(SeekFrom::Start(start_block_idx.into_bytes()))?;
        for block in blocks.iter() {
            self.file.borrow_mut().write_all(&block.contents)?;
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        let num_blocks = self.file.borrow().metadata().unwrap().len() / 512;
        Ok(BlockCount(num_blocks as u32))
    }
}
