//! TODO: This feature is still in incubation

use crate::hwa;
use embedded_sdmmc::RawFile;
use hwa::StaticAsyncController;
#[allow(unused)]
use hwa::Contract;
use hwa::sd_card::DirectoryRef;
#[allow(unused)]
use crate::alloc::string::ToString;

/*
impl<D, const MAX_DIRS: usize, const MAX_FILES: usize> hwa::sd_card::SDCard<D, MAX_DIRS, MAX_FILES>
where
    D: hwa::traits::SDBlockDevice + 'static,
{
    pub async fn retain(&mut self) {
        #[cfg(feature = "sd-card-uses-spi")]
        self.mgr.device().retain().await;
    }

    pub async fn release(&mut self) {
        #[cfg(feature = "sd-card-uses-spi")]
        self.mgr.device().release().await;
    }

    pub fn open_root_dir(&mut self) -> Result<DirectoryRef, hwa::sd_card::SDCardError> {
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
                        self.mgr.close_dir(directory);
                        Err(hwa::sd_card::SDCardError::MaxOpenDirs)
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
                            Err(hwa::sd_card::SDCardError::InconsistencyError)
                        }
                    }
                    _ => Err(hwa::sd_card::SDCardError::InternalError),
                },
            },
            None => Err(hwa::sd_card::SDCardError::NoSuchVolume),
        }
    }

    pub fn open_dir(
        &mut self,
        parent_dir_ref: &DirectoryRef,
        name: &str,
    ) -> Result<DirectoryRef, hwa::sd_card::SDCardError> {
        match self.vol.as_ref() {
            Some(vol) => {
                let parent_idx = parent_dir_ref.idx as usize;
                match &self.opened_dir_slots[parent_idx] {
                    Some(parent_dir) => {
                        hwa::debug!("Found parent at idx {}", parent_idx);
                        match self.mgr.open_dir(*parent_dir, name) {
                            Ok(directory) => {
                                hwa::debug!("Looking for a place...");
                                let mut idx = 0u8;
                                for refcount in &self.opened_dir_refcount {
                                    if *refcount == 0 {
                                        break;
                                    }
                                    idx += 1;
                                }
                                hwa::debug!("Will get idx {}... Len is {}", idx, MAX_DIRS);
                                if idx < (self.opened_dir_refcount.len() as u8) {
                                    self.opened_dir_refcount[idx as usize] += 1;
                                    self.opened_dir_slots[idx as usize] = Some(directory);
                                    self.opened_dir_names[idx as usize] = None;
                                    Ok(DirectoryRef { idx })
                                } else {
                                    self.mgr.close_dir(directory);
                                    Err(hwa::sd_card::SDCardError::MaxOpenDirs)
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
                                    if idx < (MAX_DIRS as u8) {
                                        self.opened_dir_refcount[idx as usize] += 1;
                                        Ok(DirectoryRef { idx })
                                    } else {
                                        Err(hwa::sd_card::SDCardError::InconsistencyError)
                                    }
                                }
                                _ => Err(hwa::sd_card::SDCardError::InternalError),
                            },
                        }
                    }
                    None => Err(hwa::sd_card::SDCardError::InconsistencyError),
                }
            }
            None => Err(hwa::sd_card::SDCardError::NoSuchVolume),
        }
    }

    pub fn close_dir(&mut self, dir_ref: DirectoryRef) {
        match self.vol.as_ref() {
            Some(vol) => {
                let idx = dir_ref.idx as usize;
                if self.opened_dir_refcount[idx] > 0 {
                    self.opened_dir_refcount[idx] -= 1;
                    if self.opened_dir_refcount[idx] == 0 {
                        hwa::debug!("Refcount of {} went to 0", idx);
                        if let Some(dir) = self.opened_dir_slots[idx].take() {
                            self.mgr.close_dir(dir);
                            let _ = self.opened_dir_names[idx].take();
                        }
                    }
                }
            }
            None => {
                todo!("No volume")
                //Err(SDCardError::NoSuchVolume)
            }
        }
    }

    pub fn ckeck_is_dir(
        &mut self,
        parent_dir_ref: &DirectoryRef,
        entry_name: &str,
    ) -> Result<bool, hwa::sd_card::SDCardError> {
        match self.vol.as_ref() {
            Some(vol) => {
                let idx = parent_dir_ref.idx as usize;
                if self.opened_dir_refcount[idx] > 0 {
                    match &self.opened_dir_slots[idx] {
                        Some(parent_dir) => {
                            match self.mgr.find_directory_entry(*parent_dir, entry_name) {
                                Ok(dir_entry) => Ok(dir_entry.attributes.is_directory()),
                                Err(_e) => match _e {
                                    embedded_sdmmc::Error::NoSuchVolume => {
                                        Err(hwa::sd_card::SDCardError::NoSuchVolume)
                                    }
                                    embedded_sdmmc::Error::NotFound => Err(hwa::sd_card::SDCardError::NotFound),
                                    _ => Err(hwa::sd_card::SDCardError::InternalError),
                                },
                            }
                        }
                        None => {
                            todo!("hodor")
                        }
                    }
                } else {
                    Err(hwa::sd_card::SDCardError::InconsistencyError)
                }
            }
            None => {
                todo!("No volume")
                //Err(SDCardError::NoSuchVolume)
            }
        }
    }

    /***
    This is quite slow but safe as we are holding refcounts, so it's not possible to get inconsistencies
     */
    pub fn list_dir<F>(&mut self, dir: &DirectoryRef, func: F) -> Result<(), hwa::sd_card::SDCardError>
    where
        F: FnMut(&DirEntry),
    {
        match self.vol.as_ref() {
            Some(vol) => {
                if self.opened_dir_refcount[dir.idx as usize] == 0 {
                    Err(hwa::sd_card::SDCardError::InternalError)
                } else {
                    match &self.opened_dir_slots[dir.idx as usize] {
                        Some(dir) => self.mgr.iterate_dir(*dir, func).map_err(|e| match e {
                            _ => hwa::sd_card::SDCardError::InternalError,
                        }),
                        None => Err(hwa::sd_card::SDCardError::InternalError),
                    }
                }
            }
            None => {
                todo!("No volume")
                //Err(SDCardError::NoSuchVolume)
            }
        }
    }

    pub async fn open_file(
        &mut self,
        parent_dir_ref: &DirectoryRef,
        file_name: &str,
    ) -> Result<RawFile, hwa::sd_card::SDCardError> {
        match self.vol.as_mut() {
            Some(vol) => {
                let idx = parent_dir_ref.idx as usize;
                if self.opened_dir_refcount[idx] > 0 {
                    match &self.opened_dir_slots[idx] {
                        Some(parent_dir) => {
                            match self
                                .mgr
                                .open_file_in_dir(*parent_dir, file_name, Mode::ReadOnly)
                            {
                                Ok(file) => Ok(file),
                                Err(_e) => {
                                    hwa::error!("Error opening file in directory. CLUE: File releasing is still incompleted :)");
                                    todo!("hodor")
                                }
                            }
                        }
                        None => {
                            hwa::error!(
                                "TODO Logic error. CLUE: File releasing is still incompleted :)"
                            );
                            todo!("hodor")
                        }
                    }
                } else {
                    Err(hwa::sd_card::SDCardError::InconsistencyError)
                }
            }
            None => {
                todo!("No volume")
                //Err(SDCardError::NoSuchVolume)
            }
        }
    }

    pub async fn close_file(&mut self, file: RawFile) -> Result<(), hwa::sd_card::SDCardError> {
        match self.vol.as_ref() {
            Some(vol) => match self.mgr.close_file(file) {
                Ok(()) => Ok(()),
                Err(_e) => {
                    panic!("hodor")
                }
            },
            None => {
                todo!("No volume")
                //Err(SDCardError::NoSuchVolume)
            }
        }
    }

    pub async fn read(
        &mut self,
        file: &mut RawFile,
        buffer: &mut [u8],
    ) -> Result<usize, hwa::sd_card::SDCardError> {
        match self.vol.as_ref() {
            Some(vol) => Ok(self.mgr.read(*file, buffer).map_err(|e| match e {
                _ => hwa::sd_card::SDCardError::InternalError,
            })?),
            None => {
                todo!("No volume")
                //Err(SDCardError::NoSuchVolume)
            }
        }
    }
}

 */

pub struct GenericSDCardController<D, const MAX_DIRS: usize, const MAX_FILES: usize>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    card: StaticAsyncController<hwa::AsyncStandardStrategy<hwa::AsyncNoopMutexType, hwa::sd_card::SDStateManager<D, MAX_DIRS, MAX_FILES>>>,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize> GenericSDCardController<D, MAX_DIRS, MAX_FILES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    pub async fn new(card: StaticAsyncController<hwa::AsyncStandardStrategy<hwa::AsyncNoopMutexType, hwa::sd_card::SDStateManager<D, MAX_DIRS, MAX_FILES>>>) -> Self {
        Self {
            card
        }
    }

    pub async fn dir_iterator(&self, full_path: &str) -> Result<CardAsyncDirIterator<D, MAX_DIRS, MAX_FILES>, hwa::sd_card::SDCardError> {
        hwa::debug!("Locking card");
        let mut card = self.card.lock().await;
        hwa::debug!("Retaining device");
        let _r = card.retain_device().await;
        hwa::debug!("opening root dir");
        let dir = card.open_dir_path(full_path);
        let _ = card.release_device();
        Ok(CardAsyncDirIterator::new(self.clone(), dir?))
    }

    pub async fn close_path(&self, path: &mut heapless::Vec<DirectoryRef, MAX_DIRS>) {
        let mut card = self.card.lock().await;
        hwa::debug!("Retaining device");
        let _r = card.retain_device().await;
        hwa::debug!("opening root dir");
        card.release_path(path);
        let _ = card.release_device();
    }

    pub async fn list_dir<F>(&self, dir: &DirectoryRef, func: F)
    where F:  FnMut(&embedded_sdmmc::DirEntry)
    {
        let mut card = self.card.lock().await;
        let _r = card.retain_device().await;
        card.list_dir(dir, func);
        let _ = card.release_device();
    }

    #[allow(unused)]
    fn internal_close_dir(&self, card: &mut hwa::sd_card::SDStateManager<D, MAX_DIRS, MAX_FILES>,
                 path: &mut heapless::Vec<hwa::sd_card::DirectoryRef, MAX_DIRS>, just: Option<usize>) {

        for _ in 0..just.unwrap_or(path.len()) {
            if let Some(d) = path.pop() {
                card.close_dir(d);
            }
        }
    }

    pub async fn new_stream(&self, _file_path: &str) -> Result<SDCardStream<D, MAX_FILES, MAX_DIRS>, hwa::sd_card::SDCardError> {
        todo!("");
        /*
        let mut path: heapless::Vec<DirectoryRef, MAX_DIRS> = heapless::Vec::new();
        let mut card = self.instance.lock().await;
        card.retain().await;
        let dir = card.open_root_dir()?;
        path.push(dir).map_err(|dr| {
            card.close_dir(dr);
            SDCardError::MaxOpenDirs
        })?;
        hwa::debug!("Opened root dir");
        let mut file: Option<RawFile> = None;
        for next_entry in file_path.trim_start_matches('/').split('/') {
            match file.take() {
                // If already got a file but willing to deep into tree... consume the file and fail
                Some(file) => {
                    card.close_file(file).await.map_err(|_d| {
                        hwa::error!("Unexpected error closing file");
                        SDCardError::MaxOpenDirs
                    })?;
                    return Err(SDCardError::TrailingEntries);
                }
                None => {}
            }
            if !next_entry.is_empty() {
                if next_entry == "." {
                    continue;
                } else if next_entry == ".." {
                    if let Some(last_dir) = path.pop() {
                        card.close_dir(last_dir);
                        continue;
                    } else {
                        return Err(SDCardError::InconsistencyError);
                    }
                }
                hwa::debug!("---- Opening {}", next_entry);
                if let Some(last_dir) = path.last() {
                    if card.ckeck_is_dir(last_dir, next_entry)? {
                        path.push(card.open_dir(last_dir, next_entry)?)
                            .map_err(|dir_ref| {
                                hwa::error!("Error opening subdir: push failed");
                                card.close_dir(dir_ref);
                                SDCardError::MaxOpenDirs
                            })?;
                    } else {
                        // File found -> Open it
                        file.replace(
                            card.open_file(last_dir, next_entry)
                                .await
                                .map_err(|_e| SDCardError::InternalError)?,
                        );
                    }
                }
            }
        }
        card.release().await;
        match file {
            Some(f) => Ok(SDCardStream::new(self.clone(), path, f)),
            None => Err(SDCardError::NotFound),
        }

         */
    }

    pub async fn read(
        &mut self,
        _file: &mut RawFile,
        _buffer: &mut [u8],
    ) -> Result<usize, hwa::sd_card::SDCardError> {
        todo!("");
        /*
        let mut card = self.instance.lock().await;
        card.retain().await;
        let result = card.read(file, buffer).await;
        card.release().await;
        result

         */
    }

    pub async fn close_file(&mut self, _file: RawFile) -> Result<(), hwa::sd_card::SDCardError> {
        todo!("");
        /*
        let mut card = self.instance.lock().await;
        card.retain().await;
        let result = card.close_file(file).await;
        card.release().await;
        result

         */
    }
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize> Clone for GenericSDCardController<D, MAX_DIRS, MAX_FILES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    fn clone(&self) -> Self {
        Self {
            card : self.card.clone(),
        }
    }
}

pub enum SDEntryType {
    FILE,
    DIRECTORY,
}

pub struct SDDirEntry {
    pub name: alloc::string::String,
    pub entry_type: SDEntryType,
    pub size: u32,
}

pub struct CardAsyncDirIterator<D, const MAX_DIRS: usize, const MAX_FILES: usize>
where D: hwa::traits::AsyncSDBlockDevice + 'static
{
    controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES>,
    path: heapless::Vec<DirectoryRef, MAX_DIRS>,
    current_index: usize,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize> CardAsyncDirIterator<D, MAX_DIRS, MAX_FILES>
where D: hwa::traits::AsyncSDBlockDevice + 'static
{
    pub fn new(
        instance: GenericSDCardController<D, MAX_DIRS, MAX_FILES>,
        path: heapless::Vec<DirectoryRef, MAX_DIRS>,
    ) -> Self {
        Self {
            controller: instance,
            path,
            current_index: 0,
        }
    }
    pub async fn next(&mut self) -> Result<Option<SDDirEntry>, hwa::sd_card::SDCardError> {
        match self.path.last() {
            Some(dir) => {
                let mut idx = 0;
                let mut entry = None;
                self.controller.list_dir(dir, |dir_entry| {
                    if self.current_index == idx {
                        let name: alloc::string::String = match dir_entry.name.extension().is_empty() {
                            true => alloc::string::String::from_utf8_lossy(dir_entry.name.base_name())
                                .to_string(),
                            false => {
                                alloc::format!(
                                    "{}.{}",
                                    alloc::string::String::from_utf8_lossy(dir_entry.name.base_name())
                                        .to_string()
                                        .as_str(),
                                    alloc::string::String::from_utf8_lossy(dir_entry.name.extension())
                                        .to_string()
                                        .as_str()
                                )
                            }
                        };
                        entry = Some(SDDirEntry {
                            name,
                            entry_type: match dir_entry.attributes.is_directory() {
                                true => SDEntryType::DIRECTORY,
                                false => SDEntryType::FILE,
                            },
                            size: dir_entry.size,
                        });
                    }
                    idx += 1;
                }).await;
                if entry.is_some() {
                    self.current_index += 1;
                }
                Ok(entry)
            }
            None => {
                Err(hwa::sd_card::SDCardError::NoDirectorySpecified)
            }
        }
    }

    pub async fn close(&mut self) {
        self.controller.close_path(&mut self.path).await;
    }
}

const BSIZE: usize = 32;

pub struct SDCardStream<D, const MAX_DIRS: usize, const MAX_FILES: usize>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    card_controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES>,
    file: Option<RawFile>,
    #[allow(unused)]
    path: heapless::Vec<DirectoryRef, MAX_DIRS>,
    buffer: [u8; BSIZE],
    bytes_read: u8,
    current_byte_index: u8,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize> SDCardStream<D, MAX_DIRS, MAX_FILES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    pub fn new(
        card_controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES>,
        path: heapless::Vec<DirectoryRef, MAX_DIRS>,
        file: RawFile,
    ) -> Self {
        Self {
            card_controller,
            file: Some(file),
            path,
            buffer: [0; BSIZE],
            bytes_read: 0,
            current_byte_index: 0,
        }
    }
}

impl<D, const MAX_FILES: usize, const MAX_DIRS: usize> hwa::traits::GCodeByteStream for SDCardStream<D, MAX_FILES, MAX_DIRS>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    type Item = Result<u8, async_gcode::Error>;

    async fn next(&mut self) -> Option<Self::Item> {
        if self.current_byte_index < self.bytes_read {
            let byte = self.buffer[self.current_byte_index as usize];
            self.current_byte_index += 1;
            Some(Ok(byte))
        } else {
            self.current_byte_index = 0;
            self.bytes_read = 0;
            let result = match self.file.as_mut() {
                None => Err(hwa::sd_card::SDCardError::NotFound),
                Some(f) => self.card_controller.read(f, &mut self.buffer).await,
            };
            match result {
                Ok(bytes_read) => {
                    self.bytes_read = bytes_read as u8;
                    if bytes_read > 0 {
                        let byte = self.buffer[self.current_byte_index as usize];
                        self.current_byte_index = 1;
                        Some(Ok(byte))
                    } else {
                        // FIXME not correct
                        self.bytes_read = 0;
                        self.current_byte_index = 0;
                        None
                    }
                }
                Err(_) => Some(Err(async_gcode::Error::NumberOverflow)),
            }
        }
    }
}
