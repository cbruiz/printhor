use crate as hwa;
use embedded_sdmmc::{RawDirectory, RawVolume, TimeSource, Timestamp, VolumeIdx, VolumeManager};

/// Represents various errors that can occur while interacting with an SD card.
#[cfg_attr(feature = "with-log", derive(Debug))]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum SDCardError {
    /// Error indicating that no volume could be found.
    NoSuchVolume,

    /// A general internal error.
    InternalError,

    /// Error indicating that no directory was specified.
    NoDirectorySpecified,

    /// Error for features that are not yet implemented.
    NotYetImplemented,

    /// Error indicating that the maximum number of open directories has been reached.
    MaxOpenDirs,

    /// Error indicating an inconsistency within the filesystem or state.
    InconsistencyError,

    /// Error indicating that there are trailing entries in the path.
    TrailingEntries,

    /// Error indicating that the specified item was not found.
    NotFound,
}

pub struct DummyTimeSource {}
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub struct DirectoryRef {
    idx: u8,
}

/// Helper adaption for embedded_sdmmc::VolumeManager to manage open counts and resolve paths
pub struct SDStateManager<D, const MAX_DIRS: usize, const MAX_FILES: usize>
where
    D: hwa::traits::AsyncSDBlockDevice //+ hwa::AsyncMutexStrategy + 'static,
{
    pub mgr: VolumeManager<D, DummyTimeSource, MAX_DIRS, MAX_FILES>,
    pub vol: Option<RawVolume>,
    pub opened_dir_slots: heapless::Vec<Option<RawDirectory>, MAX_DIRS>,
    pub opened_dir_refcount: heapless::Vec<u8, MAX_DIRS>,
    /// Full paths as of now...
    pub opened_dir_names: heapless::Vec<Option<alloc::string::String>, MAX_DIRS>,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize> SDStateManager<D, MAX_DIRS, MAX_FILES>
where
    D: hwa::traits::AsyncSDBlockDevice, //+ hwa::AsyncMutexStrategy + 'static
{
    pub async fn new(device: D, partition: usize) -> Self
    where
        D: hwa::traits::AsyncSDBlockDevice,
    {
        let mut card = VolumeManager::new_with_limits(device, DummyTimeSource {}, /* u32 */ 0);
        //#[cfg(feature = "sd-card-uses-spi")]
        let _ = card.device().retain().await;
        let vol = card.open_raw_volume(VolumeIdx(partition));
        //#[cfg(feature = "sd-card-uses-spi")]
        let _ = card.device().release();
        let mut opened_dir_slots = heapless::Vec::new();
        let mut opened_dir_refcount = heapless::Vec::new();
        let mut opened_dir_names = heapless::Vec::new();
        for _ in 0..MAX_DIRS {
            opened_dir_slots.push(None).unwrap();
            opened_dir_refcount.push(0).unwrap();
            opened_dir_names.push(None).unwrap();
        }
        Self {
            mgr: card,
            vol: vol.ok(),
            opened_dir_slots,
            opened_dir_refcount,
            opened_dir_names,
        }
    }

    pub async fn retain_device(&mut self) -> Result<(), ()> {
        self.mgr.device().retain().await
    }

    pub fn release_device(&mut self) -> Result<(), ()> {
        self.mgr.device().release()
    }

    pub fn open_dir_path(&mut self, path: &str) -> Result<heapless::Vec<DirectoryRef, MAX_DIRS>, SDCardError> {
        // First, is mandatory to open or refcount inc the root dir.
        match self.raw_open_root_dir() {
            Ok(root_dir) => {
                // A path_stack of current nested opened sub-dirs.
                let mut path_stack: heapless::Vec<DirectoryRef, MAX_DIRS> = heapless::Vec::new();
                if path_stack.push(root_dir).is_err() {
                    return Err(SDCardError::MaxOpenDirs);
                }
                for sub_dir_entry_name in path.trim_start_matches('/').split('/') {
                    if !sub_dir_entry_name.is_empty() {
                        if sub_dir_entry_name == "." {
                            continue;
                        } else if sub_dir_entry_name == ".." {
                            // as we are descending, we need to release current dir in our depth
                            // unless it's the root
                            if path_stack.len() > 1 {
                                if let Some(current_dir) = path_stack.pop() {
                                    self.close_dir(current_dir);
                                }
                            }
                            continue;
                        }
                        hwa::debug!("---- Opening {}", sub_dir_entry_name);
                        if let Some(&ref current_dir) = path_stack.last() {
                            match self.raw_open_dir_entry(&current_dir, sub_dir_entry_name) {
                                Ok(sub_dir) => {
                                    if let Err(_dir) = path_stack.push(sub_dir) {
                                        self.close_dir(_dir);
                                        self.release_path(&mut path_stack);
                                        return Err(SDCardError::MaxOpenDirs);
                                    }
                                }
                                Err(_e) => {
                                    self.release_path(&mut path_stack);
                                    return Err(_e);
                                }
                            }
                        }
                    }
                }
                // Return the stack of retained opened entries
                Ok(path_stack)
            }
            Err(_e) => {
                Err(_e)
            }
        }
    }

    pub fn release_path(&mut self, _path_stack: &mut heapless::Vec<DirectoryRef, MAX_DIRS>) {
        loop {
            match _path_stack.pop() {
                Some(dir) => {
                    self.close_dir(dir);
                }
                None => break
            }
        }
    }

    pub fn close_dir(&mut self, dir_ref: DirectoryRef) {
        match self.opened_dir_slots[dir_ref.idx as usize] {
            Some(dir) => {
                match self.mgr.close_dir(dir) {
                    Ok(_) => {},
                    Err(_e) => {
                        hwa::error!("Inconsistency error closing dir. Continuing anyway");
                    }
                }
            }
            None => {
                // Theoretically impossible to happen unless there is a bug in the code
                hwa::error!("Inconsistency error. Unable to close dir slot. Continuing anyway");
            }
        }
        self.opened_dir_refcount[dir_ref.idx as usize] -= 1;
        self.opened_dir_slots[dir_ref.idx as usize] = None;
        self.opened_dir_names[dir_ref.idx as usize] = None;
    }

    fn raw_open_root_dir(&mut self) -> Result<DirectoryRef, SDCardError> {

        match self.vol.as_ref() {
            Some(vol) => match self.mgr.open_root_dir(*vol) {
                Ok(directory) => {
                    let mut idx = 0u8;
                    for refcount in &self.opened_dir_refcount {
                        if *refcount == 0 {
                            break;
                        }
                        idx += 1;
                    }
                    if idx < (self.opened_dir_refcount.len() as u8) {
                        self.opened_dir_refcount[idx as usize] += 1;
                        self.opened_dir_slots[idx as usize] = Some(directory);
                        self.opened_dir_names[idx as usize] = None;
                        Ok(DirectoryRef { idx })
                    } else {
                        let _ = self.mgr.close_dir(directory);
                        Err(SDCardError::MaxOpenDirs)
                    }
                }
                Err(reason) => match reason {
                    embedded_sdmmc::Error::DirAlreadyOpen => {
                        let mut idx = 0u8;
                        for dirname in &self.opened_dir_names {
                            if dirname.is_none() {
                                break;
                            }
                            idx += 1;
                        }
                        if idx < (self.opened_dir_refcount.len() as u8) {
                            self.opened_dir_refcount[idx as usize] += 1;
                            Ok(DirectoryRef { idx })
                        } else {
                            Err(SDCardError::InconsistencyError)
                        }
                    }
                    _ => Err(SDCardError::InternalError),
                },
            },
            None => Err(SDCardError::NoSuchVolume),
        }
    }

    fn raw_open_dir_entry(&mut self, parent_ref: &DirectoryRef, entry_name: &str) -> Result<DirectoryRef, SDCardError> {
        match self.opened_dir_slots[parent_ref.idx as usize].as_ref() {
            Some(parent) => {
                match self.mgr.open_dir(*parent, entry_name) {
                    Ok(sub_dir) => {
                        let mut new_idx = 0u8;
                        for refcount in &self.opened_dir_refcount {
                            if *refcount == 0 {
                                break;
                            }
                            new_idx += 1;
                        }
                        if new_idx < (self.opened_dir_refcount.len() as u8) {
                            self.opened_dir_refcount[new_idx as usize] += 1;
                            self.opened_dir_slots[new_idx as usize] = Some(sub_dir);
                            self.opened_dir_names[new_idx as usize] = None;
                            Ok(DirectoryRef { idx: new_idx })
                        } else {
                            Err(SDCardError::MaxOpenDirs)
                        }
                    }
                    Err(reason) => match reason {
                        embedded_sdmmc::Error::DirAlreadyOpen => {
                            todo!("")
                        }
                        _ => Err(SDCardError::InternalError),
                    },
                }
            }
            None => {
                panic!("TODO")
            }
        }
    }

    pub fn list_dir<F>(&mut self, dir_ref: &DirectoryRef, func: F)
    where F: FnMut(&embedded_sdmmc::DirEntry)
    {
        if let Some(dir) = self.opened_dir_slots[dir_ref.idx as usize] {
            let _ = self.mgr.iterate_dir(dir, func);
        }
    }

}

#[cfg(test)]
mod test {
    use crate as hwa;
    use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};
    use hwa::sd_card::SDStateManager;
    use printhor_hwa_utils::StaticAsyncController;
    use crate::sd_card::SDCardError;

    struct DummyDevice {}

    impl DummyDevice {
        pub fn new() -> Self {
            Self {}
        }
    }

    impl BlockDevice for DummyDevice {
        type Error = hwa::sd_card::SDCardError;

        fn read(&self, _blocks: &mut [Block], _start_block_idx: BlockIdx, _reason: &str) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write(&self, _blocks: &[Block], _start_block_idx: BlockIdx) -> Result<(), Self::Error> {
            todo!()
        }

        fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
            Ok(BlockCount(0))
        }
    }


    /// Needed for AsyncStandardStrategy to implement SDBlockDevice trait
    struct BlockDeviceAdaptor<H>
    where
        H: hwa::AsyncMutexStrategy + 'static,
        <H as hwa::AsyncMutexStrategy>::Resource: hwa::traits::SDBlockDevice,
        <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error: From<SDCardError>,
    {
        dev: StaticAsyncController<H>
    }

    impl<H> hwa::traits::AsyncSDBlockDevice for BlockDeviceAdaptor<H>
    where
        H: hwa::AsyncMutexStrategy + 'static,
        <H as hwa::AsyncMutexStrategy>::Resource: hwa::traits::SDBlockDevice,
        <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error: From<SDCardError>,
    {
        async fn retain(&self) -> Result<(), ()> {
            self.dev.retain().await
        }

        fn release(&self) -> Result<(), ()> {
            self.dev.release()
        }
    }

    impl<H> hwa::traits::SDBlockDevice for BlockDeviceAdaptor<H>
    where
        H: hwa::AsyncMutexStrategy,
        <H as hwa::AsyncMutexStrategy>::Resource: hwa::traits::SDBlockDevice,
        <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error: From<SDCardError>
    {
        type Error = <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error;

        fn read(&self, blocks: &mut [Block], start_block_idx: BlockIdx, reason: &str) -> Result<(), Self::Error> {
            self.dev.apply_or_error(|dev| {
                dev.read(blocks, start_block_idx, reason)
            }, SDCardError::InternalError.into())
        }

        fn write(&self, _blocks: &[Block], _start_block_idx: BlockIdx) -> Result<(), Self::Error> {
            Ok(())
        }

        fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
            Ok(BlockCount(0))
        }
    }

    #[futures_test::test]
    async fn test() {

        type CardDev = DummyDevice;
        type MutexType = hwa::AsyncNoopMutexType;
        type MutexStrategyType = hwa::AsyncHoldableStrategy<MutexType, CardDev>;

        let dev = hwa::make_static_async_controller!(
            "CardDev",
            MutexStrategyType,
            DummyDevice::new()
        );

        let a: BlockDeviceAdaptor<MutexStrategyType> = BlockDeviceAdaptor{
            dev
        };


        let mut _card: SDStateManager<_, 2, 2> = SDStateManager::new(a, 0).await;


    }
}
