use crate as hwa;
use embedded_sdmmc::{
    Mode, RawDirectory, RawFile, RawVolume, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};

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

#[derive(Clone, Copy)]
pub struct EntryRef(u8);

pub enum Entry {
    Directory(RawDirectory),
    File(RawFile),
}

struct RefCounts {
    ref_count: u8,
    parent_idx: u8,
    entry_name: alloc::string::String,
}

impl RefCounts {
    const fn empty() -> Self {
        Self {
            ref_count: 0,
            parent_idx: 0,
            entry_name: alloc::string::String::new(),
        }
    }
    const fn new(parent_idx: u8, entry_name: alloc::string::String) -> Self {
        Self {
            ref_count: 1,
            parent_idx,
            entry_name,
        }
    }
}

/// Helper adaption for embedded_sdmmc::VolumeManager to manage open counts and resolve paths
pub struct SDStateManager<
    D,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_OPENED_ENTRIES: usize,
> where
    D: hwa::traits::AsyncSDBlockDevice,
{
    mgr: VolumeManager<D, DummyTimeSource, MAX_DIRS, MAX_FILES>,
    vol: Option<RawVolume>,
    opened_entries: heapless::Vec<Option<Entry>, MAX_OPENED_ENTRIES>,
    opened_entries_ref_counts: heapless::Vec<RefCounts, MAX_OPENED_ENTRIES>,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize>
    SDStateManager<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice,
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
        let mut opened_dirs = heapless::Vec::new();
        let mut opened_dirs_ref_counts = heapless::Vec::new();
        for _ in 0..MAX_DIRS {
            let _ = opened_dirs.push(None);
            let _ = opened_dirs_ref_counts.push(RefCounts::empty());
        }
        Self {
            mgr: card,
            vol: vol.ok(),
            opened_entries: opened_dirs,
            opened_entries_ref_counts: opened_dirs_ref_counts,
        }
    }

    pub async fn retain_device(&mut self) -> Result<(), ()> {
        self.mgr.device().retain().await
    }

    pub fn release_device(&mut self) -> Result<(), ()> {
        self.mgr.device().release()
    }

    pub fn open_path(
        &mut self,
        path: &str,
    ) -> Result<heapless::Vec<EntryRef, MAX_OPENED_ENTRIES>, SDCardError>
    {
        // First, it's mandatory to open or increment the reference count of the root directory.

        match self.raw_open_root_dir() {
            Ok(root_dir) => {
                // A path_stack of current nested opened sub-dirs.
                let mut path_stack: heapless::Vec<EntryRef, MAX_OPENED_ENTRIES> =
                    heapless::Vec::new();
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
                                    self.close_entry(current_dir);
                                }
                            }
                            continue;
                        }
                        hwa::debug!("---- Opening {}", sub_dir_entry_name);
                        if let Some(&ref current_dir) = path_stack.last() {
                            match self.raw_open_entry(&current_dir, sub_dir_entry_name) {
                                Ok(sub_dir) => {
                                    if let Err(_dir) = path_stack.push(sub_dir) {
                                        self.close_entry(_dir);
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
            Err(_e) => Err(_e),
        }
    }

    pub fn release_path(&mut self, _path_stack: &mut heapless::Vec<EntryRef, MAX_OPENED_ENTRIES>) {
        loop {
            match _path_stack.pop() {
                Some(dir) => {
                    self.close_entry(dir);
                }
                None => break,
            }
        }
    }

    pub fn close_entry(&mut self, entry_ref: EntryRef) {
        let ref_counts: &mut RefCounts = &mut self.opened_entries_ref_counts[entry_ref.0 as usize];
        if ref_counts.ref_count > 1 {
            ref_counts.ref_count -= 1;
        } else {
            self.opened_entries_ref_counts[entry_ref.0 as usize] = RefCounts::empty();
            let dir_entry: Option<Entry> = self.opened_entries[entry_ref.0 as usize].take();
            match dir_entry {
                Some(Entry::Directory(directory)) => match self.mgr.close_dir(directory) {
                    Ok(_) => {}
                    Err(_e) => {
                        hwa::error!("Inconsistency error closing dir. Continuing anyway");
                    }
                },
                Some(Entry::File(file)) => match self.mgr.close_file(file) {
                    Ok(_) => {
                        hwa::info!("Closed file");
                    }
                    Err(_e) => {
                        hwa::error!("Inconsistency error closing file. Continuing anyway");
                    }
                },
                None => {
                    hwa::error!("Inconsistency error closing dir. Continuing anyway");
                }
            }
        }
    }

    fn raw_open_root_dir(&mut self) -> Result<EntryRef, SDCardError> {
        match self.vol.as_ref() {
            Some(vol) => {
                let mut cached_entry_ref = None;

                for _i in 0..self.opened_entries_ref_counts.len() as u8 {
                    let d: &mut RefCounts = &mut self.opened_entries_ref_counts[_i as usize];
                    if d.ref_count > 0 && d.entry_name.is_empty() {
                        d.ref_count += 1;
                        cached_entry_ref = Some(EntryRef(_i.into()));
                        break;
                    }
                }
                match cached_entry_ref {
                    None => match self.mgr.open_root_dir(*vol) {
                        Ok(directory) => {
                            let mut new_idx = 0u8;
                            for _entry in &self.opened_entries {
                                if _entry.is_some() {
                                    new_idx += 1;
                                } else {
                                    break;
                                }
                            }
                            if new_idx < (self.opened_entries.len() as u8) {
                                self.opened_entries[new_idx as usize] =
                                    Some(Entry::Directory(directory));
                                self.opened_entries_ref_counts[new_idx as usize] =
                                    RefCounts::new(new_idx, alloc::string::String::new());
                                Ok(EntryRef(new_idx))
                            } else {
                                let _ = self.mgr.close_dir(directory);
                                Err(SDCardError::MaxOpenDirs)
                            }
                        }
                        Err(reason) => match reason {
                            _e => Err(SDCardError::InternalError),
                        },
                    },
                    Some(entry_ref) => Ok(entry_ref),
                }
            }
            None => Err(SDCardError::NoSuchVolume),
        }
    }

    fn raw_open_entry(
        &mut self,
        parent_ref: &EntryRef,
        entry_name: &str,
    ) -> Result<EntryRef, SDCardError>
    {
        if self.opened_entries_ref_counts[parent_ref.0 as usize].ref_count == 0 {
            return Err(SDCardError::InconsistencyError);
        }
        match &self.opened_entries[parent_ref.0 as usize] {
            Some(Entry::Directory(parent)) => {
                // Needs to have a parent already opened
                let mut cached_entry_ref = None;

                for _i in 0..self.opened_entries_ref_counts.len() as u8 {
                    let d: &mut RefCounts = &mut self.opened_entries_ref_counts[_i as usize];
                    if d.ref_count > 0
                        && d.entry_name.eq(entry_name)
                        && d.parent_idx == parent_ref.0
                    {
                        d.ref_count += 1;
                        cached_entry_ref = Some(EntryRef(_i));
                        break;
                    }
                }
                match cached_entry_ref {
                    Some(entry_ref) => Ok(entry_ref),
                    None => match self.mgr.find_directory_entry(*parent, entry_name) {
                        Ok(dir_entry) => {
                            let mut new_idx = 0u8;
                            for entry in &self.opened_entries {
                                if entry.is_some() {
                                    new_idx += 1;
                                } else {
                                    break;
                                }
                            }
                            if dir_entry.attributes.is_directory() {
                                match self.mgr.open_dir(*parent, entry_name) {
                                    Ok(directory) => {
                                        if new_idx < (self.opened_entries.len() as u8) {
                                            self.opened_entries[new_idx as usize] =
                                                Some(Entry::Directory(directory));
                                            self.opened_entries_ref_counts[new_idx as usize] =
                                                RefCounts::new(
                                                    parent_ref.0,
                                                    alloc::string::String::from(entry_name),
                                                );
                                            Ok(EntryRef(new_idx))
                                        } else {
                                            let _ = self.mgr.close_dir(directory);
                                            Err(SDCardError::MaxOpenDirs)
                                        }
                                    }
                                    Err(_reason) => {
                                        #[cfg(feature = "with-defmt")]
                                        hwa::error!("unable to open entry: {:?}", _reason);
                                        Err(SDCardError::InternalError)
                                    }
                                }
                            } else if dir_entry.attributes.is_archive() {
                                match self
                                    .mgr
                                    .open_file_in_dir(*parent, entry_name, Mode::ReadOnly)
                                {
                                    Ok(file) => {
                                        if new_idx < (self.opened_entries.len() as u8) {
                                            self.opened_entries[new_idx as usize] =
                                                Some(Entry::File(file));
                                            self.opened_entries_ref_counts[new_idx as usize] =
                                                RefCounts::new(
                                                    parent_ref.0,
                                                    alloc::string::String::from(entry_name),
                                                );
                                            Ok(EntryRef(new_idx))
                                        } else {
                                            let _ = self.mgr.close_file(file);
                                            Err(SDCardError::MaxOpenDirs)
                                        }
                                    }
                                    Err(_reason) => {
                                        hwa::error!("unable to open file: {:?}", _reason);
                                        Err(SDCardError::InternalError)
                                    }
                                }
                            } else {
                                hwa::error!("Unsuported entry type");
                                return Err(SDCardError::InconsistencyError);
                            }
                        }
                        Err(_) => Err(SDCardError::NotFound),
                    },
                }
            }
            _ => {
                hwa::error!("Unable to open entry. Parent is not an opened directory");
                Err(SDCardError::InconsistencyError)
            }
        }
    }

    pub fn list_dir<F>(&mut self, dir_ref: &EntryRef, func: F)
    where
        F: FnMut(&embedded_sdmmc::DirEntry),
    {
        match &self.opened_entries[dir_ref.0 as usize] {
            Some(Entry::Directory(dir)) => {
                let _ = self.mgr.iterate_dir(*dir, func);
            }
            Some(Entry::File(_)) => {
                hwa::error!("Unable to list a file entry")
            }
            _ => {
                hwa::error!("Unable to list a directory which is not open")
            }
        }
    }

    pub fn open_file_path(
        &mut self,
        _path: &str,
    ) -> Result<heapless::Vec<EntryRef, MAX_OPENED_ENTRIES>, SDCardError>
    {
        match self.open_path(_path) {
            Ok(_path_stack) => Ok(_path_stack),
            Err(_e) => Err(_e),
        }
    }

    pub fn read(&mut self, entry_ref: EntryRef, buffer: &mut [u8]) -> Result<usize, SDCardError> {
        match &self.opened_entries[entry_ref.0 as usize] {
            Some(Entry::Directory(_)) => {
                hwa::error!("Unable to read bytes from a directory");
                Err(SDCardError::InconsistencyError)
            }
            Some(Entry::File(_file)) => Ok(self
                .mgr
                .read(*_file, buffer)
                .map_err(|_e| SDCardError::InternalError)?),
            _ => {
                hwa::error!("Unable to list a directory which is not open");
                Err(SDCardError::InconsistencyError)
            }
        }
    }
}

#[cfg(test)]
mod test {
    use crate as hwa;
    use crate::sd_card::SDCardError;
    use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};
    use hwa::sd_card::SDStateManager;
    use printhor_hwa_utils::StaticAsyncController;

    struct DummyDevice {}

    impl DummyDevice {
        pub fn new() -> Self {
            Self {}
        }
    }

    impl BlockDevice for DummyDevice {
        type Error = hwa::sd_card::SDCardError;

        fn read(
            &self,
            _blocks: &mut [Block],
            _start_block_idx: BlockIdx,
            _reason: &str,
        ) -> Result<(), Self::Error> {
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
        <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error:
            From<SDCardError>,
    {
        dev: StaticAsyncController<H>,
    }

    impl<H> hwa::traits::AsyncSDBlockDevice for BlockDeviceAdaptor<H>
    where
        H: hwa::AsyncMutexStrategy + 'static,
        <H as hwa::AsyncMutexStrategy>::Resource: hwa::traits::SDBlockDevice,
        <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error:
            From<SDCardError>,
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
        <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error:
            From<SDCardError>,
    {
        type Error =
            <<H as hwa::AsyncMutexStrategy>::Resource as hwa::traits::SDBlockDevice>::Error;

        fn read(
            &self,
            blocks: &mut [Block],
            start_block_idx: BlockIdx,
            reason: &str,
        ) -> Result<(), Self::Error> {
            self.dev.apply_or_error(
                |dev| dev.read(blocks, start_block_idx, reason),
                SDCardError::InternalError.into(),
            )
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

        let dev =
            hwa::make_static_async_controller!("CardDev", MutexStrategyType, DummyDevice::new());

        let a: BlockDeviceAdaptor<MutexStrategyType> = BlockDeviceAdaptor { dev };

        let mut _card: SDStateManager<_, 2, 2, 4> = SDStateManager::new(a, 0).await;
    }
}
