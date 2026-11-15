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
//! Warning: Reference releasing is still uncompleted
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
        // A card inserted (or not ready) after the start is mounted now
        let _ = card.mount().await;
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
        hwa::debug!("releasing path");
        card.release_path(path);
        hwa::debug!("Releasing device");
        let _ = card.release_device();
    }

    /// Number of entries (directories and files) currently opened on the card
    #[allow(unused)]
    pub async fn opened_entries_count(&self) -> usize {
        self.card.lock().await.opened_entries_count()
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
        let _ = card.mount().await;
        let _ = card.retain_device().await;
        hwa::debug!("Opened root dir");
        let result = card.open_file_path(_file_path);
        // The size is read here, with the card already locked: the users get it without awaiting
        // while holding the (large) stream
        let length = match &result {
            Ok(path) => path.last().map_or(0, |entry| card.file_length(*entry).unwrap_or(0)),
            Err(_) => 0,
        };
        let _ = card.release_device();
        match result {
            Ok(path) => Ok(SDCardStream::new(self.clone(), path, length)),
            Err(_e) => Err(_e),
        }
    }

    /// Mounts the card (M21): initializes it again when the volume is not open. On failure, the
    /// reason (see [Self::mount_error]).
    pub async fn mount(&self) -> Result<(), hwa::sd_card::SDCardError> {
        self.card.lock().await.mount().await
    }

    /// Releases the card (M22). Refused while files or directories are open.
    pub async fn unmount(&self) -> Result<(), hwa::sd_card::SDCardError> {
        self.card.lock().await.unmount()
    }

    /// Why the card could not be mounted (empty when it is mounted)
    pub async fn mount_error(
        &self,
    ) -> crate::processing::LineBuf<{ hwa::sd_card::MOUNT_ERROR_LEN }> {
        crate::processing::LineBuf::from(self.card.lock().await.mount_error())
    }

    /// Creates (or truncates) a file and opens it for writing (M28). Released with
    /// [Self::close_path], which flushes it.
    pub async fn create_file(
        &self,
        file_path: &str,
    ) -> Result<heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>, hwa::sd_card::SDCardError>
    {
        let mut card = self.card.lock().await;
        let _ = card.mount().await;
        let _ = card.retain_device().await;
        let result = card.create_file(file_path);
        let _ = card.release_device();
        result
    }

    /// Appends `data` to a file opened with [Self::create_file]
    pub async fn write(
        &self,
        entry: hwa::sd_card::EntryRef,
        data: &[u8],
    ) -> Result<(), hwa::sd_card::SDCardError> {
        let mut card = self.card.lock().await;
        let _ = card.retain_device().await;
        let result = card.write(entry, data);
        let _ = card.release_device();
        result
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

/// A FAT name part as text (FAT 8.3 names are ASCII)
fn ascii(bytes: &[u8]) -> &str {
    core::str::from_utf8(bytes).unwrap_or("?")
}

/// A structure representing an entry in the SD card directory.
///
/// This can be either a file or a directory and contains metadata about the entry.
#[derive(Debug, Clone)]
pub struct SDDirEntry {
    /// The name of the entry (file or directory): a FAT 8.3 name (no heap)
    pub name: crate::processing::LineBuf<12>,

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
                            // FAT 8.3 names are ASCII: 8 + '.' + 3 bytes
                            let mut name = crate::processing::LineBuf::<12>::new();
                            name.push_str(ascii(dir_entry.name.base_name()));
                            if !dir_entry.name.extension().is_empty() {
                                name.push_str(".");
                                name.push_str(ascii(dir_entry.name.extension()));
                            }
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

/// Read chunk of the G-code stream: one SD block. Every chunk locks the card controller and
/// retains/releases the device (the SPI bus on real boards), so small chunks multiply that cost
/// (32-byte chunks were ~16x more calls for the same bytes).
const BSIZE: usize = 512;
// BSIZE must fit in u16 because SDCardStream::bytes_read is u16.
const _: () = assert!(BSIZE <= u16::MAX as usize, "BSIZE exceeds u16::MAX; update bytes_read field type");

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
    #[cfg(feature = "native")]
    buff_str: String,
    bytes_read: u16,
    current_byte_index: u16,
    /// Bytes delivered so far (the position in the file)
    position: u32,
    /// Size of the file in bytes
    length: u32,
    /// Publishes `position` to [crate::hwa::controllers::JOB_PROGRESS] (the stream of the job)
    track_progress: bool,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize>
    SDCardStream<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    pub fn new(
        card_controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
        path: heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
        length: u32,
    ) -> Self {
        Self {
            card_controller,
            path,
            buffer: [0; BSIZE],
            #[cfg(feature = "native")]
            buff_str: String::new(),
            bytes_read: 0,
            current_byte_index: 0,
            position: 0,
            length,
            track_progress: false,
        }
    }

    /// The size of the file in bytes (0 if unknown)
    pub fn length(&self) -> u32 {
        self.length
    }

    /// From now on, the position is published as the job progress
    pub fn track_progress(&mut self) {
        self.track_progress = true;
    }

    #[inline]
    fn advance(&mut self) {
        self.position = self.position.saturating_add(1);
        #[cfg(feature = "with-print-job")]
        if self.track_progress {
            // Plain store (no read-modify-write): available on every target, Cortex-M0+ included
            crate::hwa::controllers::JOB_PROGRESS.set_position(self.position);
        }
    }
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize>
    SDCardStream<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    /// Closes the stream, releasing the opened entries of its path
    pub async fn close(self) {
        self.into_closer().close().await;
    }

    /// Keeps only what closing needs (a few bytes), dropping the stream and its block buffer.
    /// To be called before awaiting the close, so a task does not hold the whole stream (~0.5 KB)
    /// in its state while closing.
    pub fn into_closer(self) -> SDCardStreamCloser<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES> {
        SDCardStreamCloser {
            card_controller: self.card_controller,
            path: self.path,
        }
    }
}

/// What closing a [SDCardStream] needs: see [SDCardStream::into_closer]
pub struct SDCardStreamCloser<
    D,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_OPENED_ENTRIES: usize,
> where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    card_controller: GenericSDCardController<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>,
    path: heapless::Vec<hwa::sd_card::EntryRef, MAX_OPENED_ENTRIES>,
}

impl<D, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_OPENED_ENTRIES: usize>
    SDCardStreamCloser<D, MAX_DIRS, MAX_FILES, MAX_OPENED_ENTRIES>
where
    D: hwa::traits::AsyncSDBlockDevice + 'static,
{
    /// Releases the opened entries of the path
    pub async fn close(mut self) {
        self.card_controller.close_path(&mut self.path).await;
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
            self.advance();
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
                    #[cfg(feature = "native")]
                    {
                        self.buff_str =
                            String::from_utf8_lossy(&self.buffer[..bytes_read]).into_owned();
                    }
                    self.bytes_read = bytes_read as u16;
                    if bytes_read > 0 {
                        let byte = self.buffer[self.current_byte_index as usize];
                        self.current_byte_index = 1;
                        self.advance();
                        Some(Ok(byte))
                    } else {
                        // EOF: no more data in this block
                        hwa::debug!("SDCardStream: EOF reached");
                        None
                    }
                }
                Err(e) => {
                    hwa::error!("SDCardStream: read error: {:?}", e);
                    None
                }
            }
        }
    }

    async fn recovery_check(&mut self) {
        self.bytes_read = 0;
        self.current_byte_index = 0;
    }
}
