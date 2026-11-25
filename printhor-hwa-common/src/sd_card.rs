//! SD-Card management

use crate as hwa;
use embedded_sdmmc::{
    DirEntry, LfnBuffer, Mode, RawDirectory, RawFile, RawVolume, ShortFileName, TimeSource,
    Timestamp, VolumeIdx, VolumeManager,
};

/// Max length (UTF-8 bytes) of a long file name (VFAT): a longer one is presented by its short
/// name. As long as the longest text of a G-code (`MAX_TEXT_LENGTH`), so that any long name
/// listed can be sent back (e.g. `M23 <long name>`)
pub const LONG_NAME_LEN: usize = 128;

/// A short (8.3) name as text: `NAME.EXT`
pub type ShortName = heapless::String<12>;

/// A long file name (see [LONG_NAME_LEN])
pub type LongName = heapless::String<LONG_NAME_LEN>;

/// The text of a short (8.3) name (`NAME.EXT`)
pub fn short_name(name: &ShortFileName) -> ShortName {
    let mut text = ShortName::new();
    let _ = core::fmt::Write::write_fmt(&mut text, format_args!("{}", name));
    text
}

/// Represents various errors that can occur while interacting with an SD card.
#[derive(Debug)]
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

    /// The name is not a valid FAT 8.3 file name
    InvalidName,

    /// Writing to the card failed (or the file is not open for writing)
    WriteError,

    /// The resource is in use (an open file, or entries still open on the volume)
    Busy,
}

/// Size of the text kept with the reason why the volume could not be opened
pub const MOUNT_ERROR_LEN: usize = 64;

/// A `core::fmt::Write` sink into a fixed buffer that keeps what fits (truncates, never fails)
struct Truncating<'a, const N: usize>(&'a mut heapless::String<N>);

impl<const N: usize> core::fmt::Write for Truncating<'_, N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for c in s.chars() {
            if self.0.push(c).is_err() {
                break;
            }
        }
        Ok(())
    }
}

impl core::fmt::Display for SDCardError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for SDCardError {}

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
    /// The partition opened as the volume
    partition: usize,
    /// Why the volume could not be opened (empty when it is, or was not tried yet)
    mount_error: heapless::String<MOUNT_ERROR_LEN>,
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
        let card = VolumeManager::new_with_limits(device, DummyTimeSource {}, /* u32 */ 0);
        let mut opened_dirs = heapless::Vec::new();
        let mut opened_dirs_ref_counts = heapless::Vec::new();
        for _ in 0..MAX_OPENED_ENTRIES {
            let _ = opened_dirs.push(None);
            let _ = opened_dirs_ref_counts.push(RefCounts::empty());
        }
        let mut manager = Self {
            mgr: card,
            vol: None,
            partition,
            mount_error: heapless::String::new(),
            opened_entries: opened_dirs,
            opened_entries_ref_counts: opened_dirs_ref_counts,
        };
        // A card not inserted (or not ready) is not an error here: it is mounted later on demand
        let _ = manager.mount().await;
        manager
    }

    /// Opens the volume when it is not open yet: the card is initialized again (it may have been
    /// inserted or replaced after the start). On failure, [Self::mount_error] tells why.
    pub async fn mount(&mut self) -> Result<(), SDCardError> {
        if self.vol.is_some() {
            return Ok(());
        }
        self.mgr.device(|d| d.reset_card());
        let _ = self.retain_device().await;
        hwa::debug!("Opening volume...");
        let vol = self.mgr.open_raw_volume(VolumeIdx(self.partition));
        let _ = self.mgr.device(|d| d.do_release());
        self.mount_error.clear();
        match vol {
            Ok(vol) => {
                hwa::info!("[sd_card] Volume {} opened", self.partition);
                self.vol = Some(vol);
                Ok(())
            }
            Err(_e) => {
                use core::fmt::Write;
                let _ = write!(Truncating(&mut self.mount_error), "{}", _e);
                hwa::warn!("[sd_card] Unable to open the volume: {}", self.mount_error.as_str());
                Err(SDCardError::NoSuchVolume)
            }
        }
    }

    /// Closes the volume (M22: the card can then be removed). Refused while entries are open.
    pub fn unmount(&mut self) -> Result<(), SDCardError> {
        match self.vol {
            None => Ok(()),
            Some(_) if self.opened_entries_count() > 0 => Err(SDCardError::Busy),
            Some(vol) => {
                let result = self.mgr.close_volume(vol).map_err(|_| SDCardError::InternalError);
                self.vol = None;
                self.mount_error.clear();
                result
            }
        }
    }

    /// Whether the volume is open
    pub fn is_mounted(&self) -> bool {
        self.vol.is_some()
    }

    /// Why the volume could not be opened (empty when it is open, or it was not tried)
    pub fn mount_error(&self) -> &str {
        self.mount_error.as_str()
    }

    pub async fn retain_device(&mut self) -> Result<(), ()> {
        let lock = self.mgr.device(|d| d.do_lock());
        match lock.await {
            Some(lock) => self.mgr.device(|d| d.do_hold(lock)),
            None => Err(()),
        }
    }

    pub fn release_device(&mut self) -> Result<(), ()> {
        self.mgr.device(|d| d.do_release())
    }

    pub fn open_path(
        &mut self,
        path: &str,
    ) -> Result<heapless::Vec<EntryRef, MAX_OPENED_ENTRIES>, SDCardError> {
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

    /// Number of entries (directories and files) currently opened on the card
    pub fn opened_entries_count(&self) -> usize {
        self.opened_entries.iter().filter(|e| e.is_some()).count()
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
                        hwa::debug!("Closed file");
                    }
                    Err(_e) => {
                        hwa::error!("Inconsistency error closing file. Continuing anyway");
                    }
                },
                None => {
                    hwa::error!(
                        "[sd_card] Inconsistency closing entry #{}: it is not open. Continuing anyway",
                        entry_ref.0
                    );
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
    ) -> Result<EntryRef, SDCardError> {
        if self.opened_entries_ref_counts[parent_ref.0 as usize].ref_count == 0 {
            return Err(SDCardError::InconsistencyError);
        }
        match &self.opened_entries[parent_ref.0 as usize] {
            Some(Entry::Directory(parent)) => {
                // The entry may be named by its short name or by its long name (case-insensitive):
                // from now on, by its short name (the cache key and the name to open it)
                let canonical = match self.find_entry(*parent, entry_name, None) {
                    Some(entry) => short_name(&entry.name),
                    None => return Err(SDCardError::NotFound),
                };
                let entry_name = canonical.as_str();
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
                                    Err(_e) => {
                                        hwa::error!("unable to open entry: (TBD)");
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
                                        hwa::error!("unable to open file: (TBD)");
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

    /// The entry of `dir` named `name`: by its short name or by its long name (ASCII
    /// case-insensitive). When `long` is given, the long name of the entry (empty when it has
    /// none, or it is longer than [LONG_NAME_LEN]) is copied there.
    fn find_entry(
        &self,
        dir: RawDirectory,
        name: &str,
        mut long: Option<&mut LongName>,
    ) -> Option<DirEntry> {
        // A short name: found without reading the long names
        if long.is_none() {
            if let Ok(entry) = self.mgr.find_directory_entry(dir, name) {
                return Some(entry);
            }
        }
        let mut storage = [0u8; LONG_NAME_LEN];
        let mut lfn = LfnBuffer::new(&mut storage);
        let mut found = None;
        let _ = self.mgr.iterate_dir_lfn(dir, &mut lfn, |entry, long_name| {
            let matches = !entry.attributes.is_volume()
                && (short_name(&entry.name).eq_ignore_ascii_case(name)
                    || long_name.is_some_and(|l| l.eq_ignore_ascii_case(name)));
            if !matches {
                return core::ops::ControlFlow::Continue(());
            }
            if let Some(out) = long.as_deref_mut() {
                out.clear();
                let _ = out.push_str(long_name.unwrap_or(""));
            }
            found = Some(entry.clone());
            core::ops::ControlFlow::Break(())
        });
        found
    }

    /// Lists a directory with the long names of its entries (`None` when an entry has none, or
    /// it is longer than [LONG_NAME_LEN]). `func` may stop the listing (`ControlFlow::Break`).
    pub fn list_dir_lfn<F>(&mut self, dir_ref: &EntryRef, mut func: F)
    where
        F: FnMut(&DirEntry, Option<&str>) -> core::ops::ControlFlow<()>,
    {
        match &self.opened_entries[dir_ref.0 as usize] {
            Some(Entry::Directory(dir)) => {
                let mut storage = [0u8; LONG_NAME_LEN];
                let mut lfn = LfnBuffer::new(&mut storage);
                let _ = self.mgr.iterate_dir_lfn(*dir, &mut lfn, |entry, long_name| {
                    func(entry, long_name)
                });
            }
            Some(Entry::File(_)) => {
                hwa::error!("Unable to list a file entry")
            }
            _ => {
                hwa::error!("Unable to list a directory which is not open")
            }
        }
    }

    /// The long path of `path` (named by short or long names): `/<name>/<name>...`, each entry
    /// by its long name, or by its short name when it has none (Marlin's `M33`). Truncated when
    /// it does not fit `out`.
    pub fn long_path<const N: usize>(
        &mut self,
        path: &str,
        out: &mut heapless::String<N>,
    ) -> Result<(), SDCardError> {
        let mut stack = self.open_path("")?;
        let mut result = Ok(());
        let mut segments = path
            .split('/')
            .filter(|segment| !segment.is_empty() && *segment != ".")
            .peekable();
        while let Some(segment) = segments.next() {
            let Some(&dir_ref) = stack.last() else {
                break;
            };
            let dir = match &self.opened_entries[dir_ref.0 as usize] {
                Some(Entry::Directory(dir)) => *dir,
                _ => {
                    result = Err(SDCardError::NotFound);
                    break;
                }
            };
            let mut long = LongName::new();
            let Some(entry) = self.find_entry(dir, segment, Some(&mut long)) else {
                result = Err(SDCardError::NotFound);
                break;
            };
            let short = short_name(&entry.name);
            let name = if long.is_empty() { short.as_str() } else { long.as_str() };
            let _ = out.push('/');
            for c in name.chars() {
                if out.push(c).is_err() {
                    break;
                }
            }
            if segments.peek().is_none() {
                break;
            }
            // A file in the middle of the path
            if !entry.attributes.is_directory() {
                result = Err(SDCardError::NotFound);
                break;
            }
            match self.raw_open_entry(&dir_ref, short.as_str()) {
                Ok(sub_dir) => {
                    if let Err(sub_dir) = stack.push(sub_dir) {
                        self.close_entry(sub_dir);
                        result = Err(SDCardError::MaxOpenDirs);
                        break;
                    }
                }
                Err(e) => {
                    result = Err(e);
                    break;
                }
            }
        }
        if result.is_ok() && out.is_empty() {
            let _ = out.push('/');
        }
        self.release_path(&mut stack);
        result
    }

    pub fn list_dir<F>(&mut self, dir_ref: &EntryRef, mut func: F)
    where
        F: FnMut(&embedded_sdmmc::DirEntry),
    {
        match &self.opened_entries[dir_ref.0 as usize] {
            Some(Entry::Directory(dir)) => {
                let _ = self.mgr.iterate_dir(*dir, |entry| {
                    func(entry);
                    core::ops::ControlFlow::Continue(())
                });
            }
            Some(Entry::File(_)) => {
                hwa::error!("Unable to list a file entry")
            }
            _ => {
                hwa::error!("Unable to list a directory which is not open")
            }
        }
    }

    /// Creates (or truncates) the file at `path` and opens it for writing. Returns the stack of
    /// opened entries (parent directories, then the file), released with [Self::release_path]
    /// (closing the file flushes it).
    pub fn create_file(
        &mut self,
        path: &str,
    ) -> Result<heapless::Vec<EntryRef, MAX_OPENED_ENTRIES>, SDCardError> {
        // rsplit_once: slicing (`&path[..i]`) links the str panic formatting (unicode tables)
        let (dir, name) = path.rsplit_once('/').unwrap_or(("", path));
        if name.is_empty() || name == "." || name == ".." {
            return Err(SDCardError::InvalidName);
        }
        let mut path_stack = self.open_path(dir)?;
        let parent = match path_stack.last() {
            Some(parent) => *parent,
            None => return Err(SDCardError::InconsistencyError),
        };
        let parent_dir = match &self.opened_entries[parent.0 as usize] {
            Some(Entry::Directory(directory)) => *directory,
            _ => {
                self.release_path(&mut path_stack);
                return Err(SDCardError::NotFound);
            }
        };
        let Some(new_idx) = self.opened_entries.iter().position(|e| e.is_none()) else {
            self.release_path(&mut path_stack);
            return Err(SDCardError::MaxOpenDirs);
        };
        // An existing file may be named by its long name: it is truncated (by its short name).
        // A new one needs a short name (long names can not be created)
        let existing = self
            .find_entry(parent_dir, name, None)
            .filter(|entry| !entry.attributes.is_directory())
            .map(|entry| short_name(&entry.name));
        let name = existing.as_ref().map_or(name, |n| n.as_str());
        match self
            .mgr
            .open_file_in_dir(parent_dir, name, Mode::ReadWriteCreateOrTruncate)
        {
            Ok(file) => {
                self.opened_entries[new_idx] = Some(Entry::File(file));
                self.opened_entries_ref_counts[new_idx] =
                    RefCounts::new(parent.0, alloc::string::String::from(name));
                if let Err(entry) = path_stack.push(EntryRef(new_idx as u8)) {
                    self.close_entry(entry);
                    self.release_path(&mut path_stack);
                    return Err(SDCardError::MaxOpenDirs);
                }
                Ok(path_stack)
            }
            Err(_e) => {
                self.release_path(&mut path_stack);
                Err(match _e {
                    embedded_sdmmc::Error::FilenameError(_) => SDCardError::InvalidName,
                    embedded_sdmmc::Error::FileAlreadyOpen => SDCardError::Busy,
                    embedded_sdmmc::Error::TooManyOpenFiles => SDCardError::MaxOpenDirs,
                    _ => SDCardError::WriteError,
                })
            }
        }
    }

    /// Appends `data` to a file opened with [Self::create_file]
    pub fn write(&mut self, entry_ref: EntryRef, data: &[u8]) -> Result<(), SDCardError> {
        match &self.opened_entries[entry_ref.0 as usize] {
            Some(Entry::File(file)) => self
                .mgr
                .write(*file, data)
                .map_err(|_| SDCardError::WriteError),
            _ => Err(SDCardError::InconsistencyError),
        }
    }

    pub fn open_file_path(
        &mut self,
        _path: &str,
    ) -> Result<heapless::Vec<EntryRef, MAX_OPENED_ENTRIES>, SDCardError> {
        match self.open_path(_path) {
            Ok(_path_stack) => Ok(_path_stack),
            Err(_e) => Err(_e),
        }
    }

    /// The size in bytes of an opened file
    pub fn file_length(&self, entry_ref: EntryRef) -> Result<u32, SDCardError> {
        match &self.opened_entries[entry_ref.0 as usize] {
            Some(Entry::File(_file)) => self
                .mgr
                .file_length(*_file)
                .map_err(|_e| SDCardError::InternalError),
            Some(Entry::Directory(_)) => Err(SDCardError::InconsistencyError),
            _ => Err(SDCardError::InconsistencyError),
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
        type Lock = hwa::AsyncMutexGuard<
            'static,
            <H as hwa::AsyncMutexStrategy>::AsyncMutexType,
            <H as hwa::AsyncMutexStrategy>::Resource,
        >;

        fn do_lock(&self) -> impl core::future::Future<Output = Option<Self::Lock>> + use<H> {
            self.dev.lock_owned()
        }

        fn do_hold(&self, lock: Self::Lock) -> Result<(), ()> {
            self.dev.hold(lock)
        }

        fn do_release(&self) -> Result<(), ()> {
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
        ) -> Result<(), Self::Error> {
            self.dev.apply_or_error(
                |dev| dev.read(blocks, start_block_idx),
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
        hwa::info!("Expecting Err: the dummy device has no filesystem, it can not be mounted");
        type CardDev = DummyDevice;
        type MutexType = hwa::AsyncNoopMutexType;
        type MutexStrategyType = hwa::AsyncHoldableStrategy<MutexType, CardDev>;

        let dev =
            hwa::make_static_async_controller!("CardDev", MutexStrategyType, DummyDevice::new());

        let a: BlockDeviceAdaptor<MutexStrategyType> = BlockDeviceAdaptor { dev };

        let mut _card: SDStateManager<_, 2, 2, 4> = SDStateManager::new(a, 0).await;
    }
}

#[cfg(test)]
mod coverage_tests;
