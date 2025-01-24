//! This module provides functionality for managing SD card operations asynchronously
//! using a `GenericSDCardController`. It is designed to work with SD block devices that
//! implement the `AsyncSDBlockDevice` trait.
//!
//! ## Overview
//!
//! The main struct in this module is `GenericSDCardController`, which allows asynchronous
//! handling of directories and files on an SD card. It provides methods for creating new
//! instances, iterating over directories, closing paths, listing directories, opening file streams,
//! and reading file entries.
//!
//! It leverages an `SDStateManager` to manage the state of the SD card and ensures safe
//! concurrent access using `AsyncNoopMutexType`.
//!
//! ## Structures and Enums
//!
//! - `GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>`: Controls the SD card operations.
//! - `SDEntryType`: Enum representing the type of SD card entries (File or Directory).
//! - `SDDirEntry`: Struct representing a directory entry.
//! - `CardAsyncDirIterator<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>`: Iterator for async directory listing.
//!
//! ## Examples
//!
//! ```rust
//! use crate::hwa;
//! use hwa::StaticAsyncController;
//!
//! async fn example<D>(card: StaticAsyncController<hwa::AsyncStandardStrategy<hwa::AsyncNoopMutexType, hwa::sd_card::SDStateManager<D, 10, 10, 5>>>)
//! where
//!     D: hwa::traits::AsyncSDBlockDevice + 'static,
//! {
//!     let controller = GenericSDCardController::<D, 10, 10, 5>::new(card).await;
//! }
//! ```
//!
//! The above example shows how to create a new `GenericSDCardController` instance and
//! customize it for specific SD card configurations.
//!
#[allow(unused)]
use crate::alloc::string::ToString;
use crate::hwa;
#[allow(unused)]
use hwa::Contract;
use hwa::StaticAsyncController;

/// `GenericSDCardController` is a controller for handling SD card operations
/// asynchronously.
///
/// # Type Parameters
///
/// * `D` - Represents the SD block device that must implement the `AsyncSDBlockDevice` trait.
/// * `MAX_DIRS` - Maximum number of directories that can be handled.
/// * `MAX_FILES` - Maximum number of files that can be handled.
/// * `MAX_OPENED_ENTRIES` - Maximum number of opened entries that the controller can manage.
///
/// # Fields
///
/// * `card` - A `StaticAsyncController` managing the `AsyncStandardStrategy` for the SD card.
///
/// # Example
///
/// ```rust
/// use crate::hwa;
/// use hwa::StaticAsyncController;
///
/// // Example usage of GenericSDCardController
/// async fn example<D>(card: StaticAsyncController<hwa::AsyncStandardStrategy<hwa::AsyncNoopMutexType, hwa::sd_card::SDStateManager<D, 10, 10, 5>>>)
/// where
///     D: hwa::traits::AsyncSDBlockDevice + 'static,
/// {
///     let controller = GenericSDCardController::<D, 10, 10, 5>::new(card).await;
/// }
/// ```
pub struct GenericSDCardController<
    D,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_OPENED_ENTRIES: usize,
> where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    card: StaticAsyncController<
        hwa::AsyncStandardStrategy<
            hwa::AsyncNoopMutexType,
            hwa::sd_card::SDStateManager<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
        >,
    >,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize>
    GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    pub async fn new(
        card: StaticAsyncController<
            hwa::AsyncStandardStrategy<
                hwa::AsyncNoopMutexType,
                hwa::sd_card::SDStateManager<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
            >,
        >,
    ) -> Self {
        Self { card }
    }

    pub async fn dir_iterator(
        &self,
        full_path: &str,
    ) -> Result<
        CardAsyncDirIterator<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
        hwa::sd_card::SDCardError,
    > {
        hwa::debug!("Locking card");
        let mut card = self.card.lock().await;
        hwa::debug!("Retaining device");
        let _r = card.retain_device().await;
        hwa::debug!("opening root dir");
        let dir = card.open_path(full_path);
        let _ = card.release_device();
        Ok(CardAsyncDirIterator::new(self.clone(), dir?))
    }

    pub async fn close_path(
        &self,
        path: &mut heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
    ) {
        let mut card = self.card.lock().await;
        hwa::debug!("Retaining device");
        let _r = card.retain_device().await;
        hwa::info!("releasing path");
        card.release_path(path);
        let _ = card.release_device();
    }

    pub async fn list_dir<F>(&self, dir: &hwa::sd_card::EntryRef, func: F)
    where
        F: FnMut(&embedded_sdmmc::DirEntry),
    {
        let mut card = self.card.lock().await;
        let _r = card.retain_device().await;
        card.list_dir(dir, func);
        let _ = card.release_device();
    }

    #[allow(unused)]
    fn internal_close_dir(
        &self,
        card: &mut hwa::sd_card::SDStateManager<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
        path: &mut heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
        just: Option<usize>,
    ) {
        for _ in 0..just.unwrap_or(path.len()) {
            if let Some(d) = path.pop() {
                card.close_entry(d);
            }
        }
    }

    pub async fn new_stream(
        &self,
        _file_path: &str,
    ) -> Result<SDCardStream<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>, hwa::sd_card::SDCardError>
    {
        let mut card = self.card.lock().await;
        let _ = card.retain_device().await;
        hwa::debug!("Opened root dir");
        let result = card.open_file_path(_file_path);
        let _ = card.release_device();
        match result {
            Ok(path) => Ok(SDCardStream::new(self.clone(), path)),
            Err(_e) => Err(_e),
        }
    }

    pub async fn read(
        &mut self,
        _entry: hwa::sd_card::EntryRef,
        buffer: &mut [u8],
    ) -> Result<usize, hwa::sd_card::SDCardError> {
        let mut card = self.card.lock().await;
        let _ = card.retain_device().await;
        let result = card.read(_entry, buffer);
        let _ = card.release_device();
        result
    }
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize> Clone
    for GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    fn clone(&self) -> Self {
        Self {
            card: self.card.clone(),
        }
    }
}

#[derive(Debug, Clone)]
pub enum SDEntryType {
    FILE,
    DIRECTORY,
}

/// A structure representing an entry in the SD card directory.
///
/// This can be either a file or a directory and contains metadata about the entry.
#[derive(Debug, Clone)]
pub struct SDDirEntry {
    /// The name of the entry (file or directory).
    pub name: alloc::string::String,

    /// The type of the entry, indicating whether it's a file or a directory.
    pub entry_type: SDEntryType,

    /// The size of the file in bytes. For directories, this might be 0 or some other relevant value.
    pub size: u32,
}

/// An asynchronous directory iterator for the SD card.
///
/// This struct allows iterating over the directory entries of an SD card in an asynchronous manner.
pub struct CardAsyncDirIterator<
    D,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_OPENED_ENTRIES: usize,
> where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
    path: heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
    current_index: usize,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize>
    CardAsyncDirIterator<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    /// Creates a new asynchronous directory iterator.
    ///
    /// # Parameters
    ///
    /// - `instance`: An instance of `GenericSDCardController`.
    /// - `path`: A vector of `EntryRef` representing the path to iterate over.
    ///
    /// # Returns
    ///
    /// - A new `CardAsyncDirIterator` instance.
    pub fn new(
        instance: GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
        path: heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
    ) -> Self {
        Self {
            controller: instance,
            path,
            current_index: 0,
        }
    }

    /// Asynchronously retrieves the next entry in the directory.
    ///
    /// # Returns
    ///
    /// - `Ok(Some(SDDirEntry))` if an entry is available.
    /// - `Ok(None)` if no more entries are available.
    /// - `Err(SDCardError)` if an error occurs.
    pub async fn next(&mut self) -> Result<Option<SDDirEntry>, hwa::sd_card::SDCardError> {
        match self.path.last() {
            Some(dir) => {
                let mut idx = 0;
                let mut entry = None;
                self.controller
                    .list_dir(dir, |dir_entry| {
                        if self.current_index == idx {
                            let name: alloc::string::String =
                                match dir_entry.name.extension().is_empty() {
                                    true => alloc::string::String::from_utf8_lossy(
                                        dir_entry.name.base_name(),
                                    )
                                    .to_string(),
                                    false => {
                                        alloc::format!(
                                            "{}.{}",
                                            alloc::string::String::from_utf8_lossy(
                                                dir_entry.name.base_name()
                                            )
                                            .to_string()
                                            .as_str(),
                                            alloc::string::String::from_utf8_lossy(
                                                dir_entry.name.extension()
                                            )
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
                    })
                    .await;
                if entry.is_some() {
                    self.current_index += 1;
                }
                Ok(entry)
            }
            None => Err(hwa::sd_card::SDCardError::NoDirectorySpecified),
        }
    }

    /// Asynchronously closes the directory iterator.
    ///
    /// This will release any resources associated with the directory.
    pub async fn close(&mut self) {
        self.controller.close_path(&mut self.path).await;
    }
}

const BSIZE: usize = 32;

pub struct SDCardStream<
    D,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_OPENED_ENTRIES: usize,
> where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    card_controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
    #[allow(unused)]
    path: heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
    buffer: [u8; BSIZE],
    bytes_read: u8,
    current_byte_index: u8,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize>
    SDCardStream<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    pub fn new(
        card_controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
        path: heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
    ) -> Self {
        Self {
            card_controller,
            path,
            buffer: [0; BSIZE],
            bytes_read: 0,
            current_byte_index: 0,
        }
    }
}

impl<D, const MAX_FILES: usize, const MAX_DIRS: usize, const MAX_OPENED_ENTRIES: usize>
    hwa::traits::GCodeByteStream for SDCardStream<D, MAX_FILES, MAX_DIRS, MAX_OPENED_ENTRIES>
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
            let result = match self.path.last() {
                None => Err(hwa::sd_card::SDCardError::NotFound),
                Some(entry) => self.card_controller.read(*entry, &mut self.buffer).await,
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

    async fn recovery_check(&mut self) {
        //TODO
    }
}
