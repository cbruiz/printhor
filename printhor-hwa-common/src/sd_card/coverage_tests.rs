//! Tests for `SDStateManager` against an in-memory FAT16 volume.
//!
//! Disk layout (512-byte blocks, sparse: missing blocks read as zeros):
//! - block 0: MBR with one FAT16 partition starting at block 1 (4200 blocks)
//! - volume block 0: BPB (1 block/cluster, 1 reserved block, 1 FAT of 17 blocks, 16 root entries)
//! - volume blocks 1..=17: FAT; block 18: root directory; block 19+: data (cluster 2 = block 19)
//!
//! Content: `/GCODE/` (dir, cluster 2) holding `PART.GCO` (cluster 4), `/BENCHY.GCO` (cluster 3)
//! `/READONLY.TXT` (read-only attribute only: neither a directory nor an archive) and
//! `/BAD.GCO`, whose data block cannot be read from the device.
use super::*;
use core::sync::atomic::{AtomicUsize, Ordering};
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};
use std::collections::BTreeMap;
use std::string::{String, ToString};
use std::sync::Arc;
use std::vec::Vec;

const LBA_START: u32 = 1;
const VOLUME_BLOCKS: u32 = 4200;
const FAT_BLOCKS: u32 = 17;
const ROOT_DIR_BLOCK: u32 = LBA_START + 1 + FAT_BLOCKS;
const FIRST_DATA_BLOCK: u32 = ROOT_DIR_BLOCK + 1;
const BENCHY: &[u8] = b"G28\nG1 X10 Y10\n";
const PART: &[u8] = b"M115\n";

struct RamDisk {
    blocks: core::cell::RefCell<BTreeMap<u32, [u8; 512]>>,
    /// Whether the card is inserted: otherwise every access fails
    present: Arc<core::sync::atomic::AtomicBool>,
    /// Number of `reset_card` calls
    resets: Arc<AtomicUsize>,
    bad_block: u32,
    retained: Arc<AtomicUsize>,
    released: Arc<AtomicUsize>,
}

impl BlockDevice for RamDisk {
    type Error = SDCardError;

    fn read(&self, blocks: &mut [Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        if !self.present.load(Ordering::SeqCst) {
            return Err(SDCardError::NotFound);
        }
        for (i, b) in blocks.iter_mut().enumerate() {
            if start_block_idx.0 + i as u32 == self.bad_block {
                return Err(SDCardError::InternalError);
            }
            b.contents = self
                .blocks
                .borrow()
                .get(&(start_block_idx.0 + i as u32))
                .copied()
                .unwrap_or([0u8; 512]);
        }
        Ok(())
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        if !self.present.load(Ordering::SeqCst) {
            return Err(SDCardError::NotFound);
        }
        let mut disk = self.blocks.borrow_mut();
        for (i, b) in blocks.iter().enumerate() {
            disk.insert(start_block_idx.0 + i as u32, b.contents);
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        Ok(BlockCount(LBA_START + VOLUME_BLOCKS))
    }
}

impl hwa::traits::AsyncSDBlockDevice for RamDisk {
    type Lock = ();

    fn do_lock(&self) -> impl core::future::Future<Output = Option<()>> + use<> {
        async { Some(()) }
    }

    fn do_hold(&self, _lock: ()) -> Result<(), ()> {
        self.retained.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn do_release(&self) -> Result<(), ()> {
        self.released.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn reset_card(&mut self) {
        self.resets.fetch_add(1, Ordering::SeqCst);
    }
}

fn put_u16(b: &mut [u8], at: usize, v: u16) {
    b[at..at + 2].copy_from_slice(&v.to_le_bytes());
}

fn put_u32(b: &mut [u8], at: usize, v: u32) {
    b[at..at + 4].copy_from_slice(&v.to_le_bytes());
}

/// Writes a 32-byte short-name directory entry.
fn dir_entry(b: &mut [u8], slot: usize, name: &[u8; 11], attributes: u8, cluster: u16, size: u32) {
    let e = &mut b[slot * 32..slot * 32 + 32];
    e[..11].copy_from_slice(name);
    e[11] = attributes;
    put_u16(e, 26, cluster);
    put_u32(e, 28, size);
}

fn cluster_block(cluster: u32) -> u32 {
    FIRST_DATA_BLOCK + cluster - 2
}

/// Builds the disk. With `with_mbr == false` the disk is blank (no partition table).
fn disk(with_mbr: bool) -> RamDisk {
    let mut blocks = BTreeMap::new();
    if with_mbr {
        let mut mbr = [0u8; 512];
        mbr[446 + 4] = 0x06; // FAT16
        put_u32(&mut mbr, 446 + 8, LBA_START);
        put_u32(&mut mbr, 446 + 12, VOLUME_BLOCKS);
        put_u16(&mut mbr, 510, 0xAA55);
        blocks.insert(0, mbr);

        let mut bpb = [0u8; 512];
        bpb[..3].copy_from_slice(&[0xEB, 0x3C, 0x90]);
        bpb[3..11].copy_from_slice(b"PRINTHOR");
        put_u16(&mut bpb, 11, 512); // bytes per block
        bpb[13] = 1; // blocks per cluster
        put_u16(&mut bpb, 14, 1); // reserved blocks
        bpb[16] = 1; // number of FATs
        put_u16(&mut bpb, 17, 16); // root entries
        put_u16(&mut bpb, 19, VOLUME_BLOCKS as u16);
        bpb[21] = 0xF8; // media
        put_u16(&mut bpb, 22, FAT_BLOCKS as u16);
        bpb[43..54].copy_from_slice(b"PRINTHOR   ");
        put_u16(&mut bpb, 510, 0xAA55);
        blocks.insert(LBA_START, bpb);

        let mut fat = [0u8; 512];
        for (cluster, value) in [(0, 0xFFF8u16), (1, 0xFFFF), (2, 0xFFFF), (3, 0xFFFF), (4, 0xFFFF), (6, 0xFFFF)] {
            put_u16(&mut fat, cluster * 2, value);
        }
        blocks.insert(LBA_START + 1, fat);

        let mut root = [0u8; 512];
        dir_entry(&mut root, 0, b"GCODE      ", 0x10, 2, 0);
        dir_entry(&mut root, 1, b"BENCHY  GCO", 0x20, 3, BENCHY.len() as u32);
        dir_entry(&mut root, 2, b"READONLYTXT", 0x01, 5, 0);
        dir_entry(&mut root, 3, b"BAD     GCO", 0x20, 6, 10);
        blocks.insert(ROOT_DIR_BLOCK, root);

        let mut gcode = [0u8; 512];
        dir_entry(&mut gcode, 0, b".          ", 0x10, 2, 0);
        dir_entry(&mut gcode, 1, b"..         ", 0x10, 0, 0);
        dir_entry(&mut gcode, 2, b"PART    GCO", 0x20, 4, PART.len() as u32);
        blocks.insert(cluster_block(2), gcode);

        let mut benchy = [0u8; 512];
        benchy[..BENCHY.len()].copy_from_slice(BENCHY);
        blocks.insert(cluster_block(3), benchy);

        let mut part = [0u8; 512];
        part[..PART.len()].copy_from_slice(PART);
        blocks.insert(cluster_block(4), part);
    }
    RamDisk {
        blocks: core::cell::RefCell::new(blocks),
        present: Arc::new(core::sync::atomic::AtomicBool::new(true)),
        resets: Arc::new(AtomicUsize::new(0)),
        bad_block: cluster_block(6),
        retained: Arc::new(AtomicUsize::new(0)),
        released: Arc::new(AtomicUsize::new(0)),
    }
}

fn read_all<const D: usize, const F: usize, const E: usize>(
    card: &mut SDStateManager<RamDisk, D, F, E>,
    entry: EntryRef,
) -> Vec<u8> {
    let mut out = Vec::new();
    let mut buf = [0u8; 4];
    loop {
        let n = card.read(entry, &mut buf).unwrap();
        if n == 0 {
            break out;
        }
        out.extend_from_slice(&buf[..n]);
    }
}

fn total_refs<const D: usize, const F: usize, const E: usize>(
    card: &SDStateManager<RamDisk, D, F, E>,
) -> usize {
    card.opened_entries_ref_counts
        .iter()
        .map(|r| r.ref_count as usize)
        .sum()
}

fn names_in<const D: usize, const F: usize, const E: usize>(
    card: &mut SDStateManager<RamDisk, D, F, E>,
    dir: EntryRef,
) -> Vec<String> {
    let mut names = Vec::new();
    card.list_dir(&dir, |e| names.push(e.name.to_string()));
    names
}

#[test]
fn errors_display_their_name_and_the_time_source_is_the_epoch() {
    assert_eq!(std::format!("{}", SDCardError::NotFound), "NotFound");
    assert_eq!(std::format!("{}", SDCardError::MaxOpenDirs), "MaxOpenDirs");
    let e: &dyn core::error::Error = &SDCardError::NoSuchVolume;
    assert_eq!(e.to_string(), "NoSuchVolume");

    let t = DummyTimeSource {}.get_timestamp();
    assert_eq!(
        (t.year_since_1970, t.zero_indexed_month, t.zero_indexed_day, t.hours, t.minutes, t.seconds),
        (0, 0, 0, 0, 0, 0)
    );
}

#[futures_test::test]
async fn device_retain_and_release_are_forwarded() {
    let d = disk(true);
    let (retained, released) = (d.retained.clone(), d.released.clone());
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(d, 0).await;
    // Mounting retains and releases the device once
    assert_eq!((retained.load(Ordering::SeqCst), released.load(Ordering::SeqCst)), (1, 1));
    assert_eq!(card.retain_device().await, Ok(()));
    assert_eq!(card.release_device(), Ok(()));
    assert_eq!((retained.load(Ordering::SeqCst), released.load(Ordering::SeqCst)), (2, 2));
}

#[futures_test::test]
async fn open_path_walks_directories_and_reads_files() {
    crate::info!("Expecting Err: reading a directory and listing a file are rejected");
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(true), 0).await;

    let root_only = card.open_path("/").unwrap();
    assert_eq!(root_only.len(), 1);
    let mut names = names_in(&mut card, root_only[0]);
    names.sort();
    assert_eq!(names, ["BAD.GCO", "BENCHY.GCO", "GCODE", "READONLY.TXT"]);
    card.release_path(&mut root_only.clone());

    let mut stack = card.open_file_path("/GCODE/PART.GCO").unwrap();
    assert_eq!(stack.len(), 3);
    let file = *stack.last().unwrap();
    assert_eq!(read_all(&mut card, file), PART);
    // Directories cannot be read and files cannot be listed
    assert!(matches!(card.read(stack[1], &mut [0u8; 8]), Err(SDCardError::InconsistencyError)));
    assert!(names_in(&mut card, file).is_empty());
    card.release_path(&mut stack);
    assert!(stack.is_empty());
    assert_eq!(total_refs(&card), 0, "everything closed");
    // Closed entries can be neither read nor listed
    assert!(matches!(card.read(file, &mut [0u8; 8]), Err(SDCardError::InconsistencyError)));
    assert!(names_in(&mut card, file).is_empty());

    // `.`, `..` (never above the root) and repeated separators are resolved while walking
    for path in [
        "GCODE/./PART.GCO",
        "//GCODE//PART.GCO",
        "/GCODE/../GCODE/PART.GCO",
        "/../GCODE/PART.GCO",
    ] {
        let mut stack = card.open_path(path).unwrap();
        assert_eq!(stack.len(), 3, "{}", path);
        assert_eq!(read_all(&mut card, *stack.last().unwrap()), PART, "{}", path);
        card.release_path(&mut stack);
        assert_eq!(total_refs(&card), 0, "{}", path);
    }

    let mut stack = card.open_file_path("/BENCHY.GCO").unwrap();
    assert_eq!(read_all(&mut card, stack[1]), BENCHY);
    card.release_path(&mut stack);
}

#[futures_test::test]
async fn entries_are_reference_counted_and_shared_between_paths() {
    crate::info!("Expecting Err: closing an entry twice is reported as an inconsistency");
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(true), 0).await;
    let mut first = card.open_path("/GCODE").unwrap();
    let mut second = card.open_path("/GCODE").unwrap();
    assert_eq!((first[0].0, first[1].0), (second[0].0, second[1].0), "cached entries");
    assert_eq!(card.opened_entries_ref_counts[first[1].0 as usize].ref_count, 2);

    card.release_path(&mut first);
    // Still open through the second path
    assert_eq!(names_in(&mut card, second[1]).len(), 3, ". .. PART.GCO");
    card.release_path(&mut second);
    assert_eq!(total_refs(&card), 0);
    assert!(card.opened_entries.iter().all(|e| e.is_none()));

    // Closing a slot that holds nothing is tolerated
    card.close_entry(EntryRef(3));
    assert_eq!(total_refs(&card), 0);
}

#[futures_test::test]
async fn failed_lookups_release_the_partial_path() {
    crate::info!("Expecting Err: paths through missing directories or files are rejected");
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(card.open_path("/MISSING.GCO"), Err(SDCardError::NotFound)));
    assert!(matches!(card.open_path("/GCODE/MISSING.GCO"), Err(SDCardError::NotFound)));
    // A file used as a directory
    assert!(matches!(card.open_path("/BENCHY.GCO/X"), Err(SDCardError::InconsistencyError)));
    // Neither a directory nor an archive
    assert!(matches!(card.open_path("/READONLY.TXT"), Err(SDCardError::InconsistencyError)));
    assert_eq!(total_refs(&card), 0, "nothing leaked");
    assert!(card.opened_entries.iter().all(|e| e.is_none()));

    // Device read errors surface as internal errors, and the file can still be closed
    let mut bad = card.open_file_path("/BAD.GCO").unwrap();
    assert!(matches!(card.read(bad[1], &mut [0u8; 8]), Err(SDCardError::InternalError)));
    card.release_path(&mut bad);
    assert_eq!(total_refs(&card), 0);
}

#[futures_test::test]
async fn missing_volumes_are_reported() {
    crate::info!("Expecting Err: the partitions without a filesystem can not be mounted");
    // Partition 2 is empty
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(true), 1).await;
    assert!(matches!(card.open_path("/"), Err(SDCardError::NoSuchVolume)));
    // Blank disk: no MBR signature
    let mut blank: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(false), 0).await;
    assert!(matches!(blank.open_file_path("/BENCHY.GCO"), Err(SDCardError::NoSuchVolume)));
}

#[futures_test::test]
async fn capacity_limits_are_reported_without_leaking_entries() {
    crate::info!("Expecting Err: the entries that do not fit in the tables are rejected");
    // Entry table of 2 slots: root + GCODE fill it, the file does not fit
    let mut small: SDStateManager<_, 2, 2, 2> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(small.open_path("/GCODE/PART.GCO"), Err(SDCardError::MaxOpenDirs)));
    assert_eq!(total_refs(&small), 0);
    // A single slot: only the root fits
    let mut tiny: SDStateManager<_, 1, 1, 1> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(tiny.open_path("/BENCHY.GCO"), Err(SDCardError::MaxOpenDirs)));
    assert_eq!(total_refs(&tiny), 0);

    // The volume manager allows a single open file
    let mut one_file: SDStateManager<_, 4, 1, 4> = SDStateManager::new(disk(true), 0).await;
    let mut benchy = one_file.open_path("/BENCHY.GCO").unwrap();
    assert!(matches!(one_file.open_path("/GCODE/PART.GCO"), Err(SDCardError::InternalError)));
    one_file.release_path(&mut benchy);
    assert_eq!(total_refs(&one_file), 0);
    let mut part = one_file.open_path("/GCODE/PART.GCO").unwrap();
    assert_eq!(read_all(&mut one_file, part[2]), PART);
    one_file.release_path(&mut part);

    // No entry slot at all: not even the root directory fits
    let mut none: SDStateManager<_, 1, 1, 0> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(none.open_path("/"), Err(SDCardError::MaxOpenDirs)));
    // A single slot (taken by the root) but room for two directories in the volume manager
    let mut one_slot: SDStateManager<_, 2, 2, 1> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(one_slot.open_path("/GCODE"), Err(SDCardError::MaxOpenDirs)));
    assert_eq!(total_refs(&one_slot), 0);
    // The volume manager cannot open any directory: the root cannot be opened
    let mut no_dirs: SDStateManager<_, 0, 1, 4> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(no_dirs.open_path("/"), Err(SDCardError::InternalError)));

    // The volume manager allows a single open directory (taken by the root)
    let mut one_dir: SDStateManager<_, 1, 2, 4> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(one_dir.open_path("/GCODE"), Err(SDCardError::InternalError)));
    assert_eq!(total_refs(&one_dir), 0);
}

/// The firmware instantiates `SDStateManager<D, MAX_DIRS, MAX_FILES, MAX_DIRS + MAX_FILES>`, but
/// `new()` fills the entry table with `MAX_DIRS` slots only, so directories and files together
/// can never exceed `MAX_DIRS`: here 2 directories + 1 file (within every limit) do not fit.
#[futures_test::test]
async fn entry_table_holds_max_opened_entries() {
    let mut card: SDStateManager<_, 2, 2, 4> = SDStateManager::new(disk(true), 0).await;
    assert_eq!(card.opened_entries.len(), 4);
    let mut stack = card.open_path("/GCODE/PART.GCO").unwrap();
    assert_eq!(read_all(&mut card, stack[2]), PART);
    card.release_path(&mut stack);
}


#[futures_test::test]
async fn the_volume_is_mounted_on_demand_and_reports_why_it_is_not() {
    crate::info!("Expecting Err: without a card (or a formatted one) the volume can not be mounted");
    // No card when the firmware starts
    let d = disk(true);
    let (present, resets) = (d.present.clone(), d.resets.clone());
    present.store(false, Ordering::SeqCst);
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(d, 0).await;
    assert!(!card.is_mounted());
    assert!(card.mount_error().contains("NotFound"), "{}", card.mount_error());
    assert!(matches!(card.open_path("/"), Err(SDCardError::NoSuchVolume)));
    assert!(matches!(card.mount().await, Err(SDCardError::NoSuchVolume)));

    // Inserted later: the next mount initializes the card again and opens the volume
    present.store(true, Ordering::SeqCst);
    let before = resets.load(Ordering::SeqCst);
    assert!(card.mount().await.is_ok());
    assert_eq!(resets.load(Ordering::SeqCst), before + 1, "the card is initialized again");
    assert!(card.is_mounted() && card.mount_error().is_empty());
    let mut stack = card.open_file_path("/BENCHY.GCO").unwrap();
    // Mounting an open volume does nothing; releasing it is refused while entries are open
    assert!(card.mount().await.is_ok());
    assert_eq!(resets.load(Ordering::SeqCst), before + 1);
    assert!(matches!(card.unmount(), Err(SDCardError::Busy)));
    card.release_path(&mut stack);
    assert!(card.unmount().is_ok() && !card.is_mounted());
    assert!(card.unmount().is_ok(), "releasing twice does nothing");
    assert!(card.mount().await.is_ok());

    // An unsupported format tells it (no MBR)
    let blank: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(false), 0).await;
    assert!(blank.mount_error().contains("MBR"), "{}", blank.mount_error());
}

#[test]
fn the_mount_error_is_truncated_to_its_buffer() {
    use core::fmt::Write;
    let mut text: heapless::String<8> = heapless::String::new();
    assert!(write!(Truncating(&mut text), "{}", "a long description").is_ok());
    assert_eq!(text.as_str(), "a long d");
}

#[futures_test::test]
async fn files_are_created_written_and_read_back() {
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(true), 0).await;
    let content: Vec<u8> = (0..1300u32).map(|i| b"G1 X1\n"[(i % 6) as usize]).collect();
    for (path, file_at) in [("/NEW.GCO", 1usize), ("GCODE/NEW2.GCO", 2)] {
        let mut stack = card.create_file(path).unwrap();
        assert_eq!(stack.len(), file_at + 1, "{}", path);
        let file = *stack.last().unwrap();
        // In pieces, across several blocks
        for chunk in content.chunks(97) {
            card.write(file, chunk).unwrap();
        }
        // Directories can not be written
        assert!(matches!(card.write(stack[0], b"x"), Err(SDCardError::InconsistencyError)));
        card.release_path(&mut stack);
        assert_eq!(total_refs(&card), 0, "{}", path);

        let mut stack = card.open_file_path(path).unwrap();
        assert_eq!(read_all(&mut card, *stack.last().unwrap()), content, "{}", path);
        card.release_path(&mut stack);
    }
    // Truncated when created again
    let mut stack = card.create_file("/NEW.GCO").unwrap();
    card.write(*stack.last().unwrap(), b"M115\n").unwrap();
    card.release_path(&mut stack);
    let mut stack = card.open_file_path("/NEW.GCO").unwrap();
    assert_eq!(read_all(&mut card, stack[1]), b"M115\n");
    card.release_path(&mut stack);
    let root = card.open_path("/").unwrap();
    assert!(names_in(&mut card, root[0]).contains(&"NEW.GCO".to_string()));
    card.release_path(&mut root.clone());
}

#[futures_test::test]
async fn invalid_file_creations_are_reported_without_leaking_entries() {
    crate::info!("Expecting Err: invalid file creations are rejected");
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(true), 0).await;
    for (path, expected) in [
        ("/", "InvalidName"),
        ("/GCODE/", "InvalidName"),
        ("..", "InvalidName"),
        ("/TOOLONGNAME.GCODE", "InvalidName"),
        ("/MISSING/X.GCO", "NotFound"),
        // A file used as a directory
        ("/BENCHY.GCO/X.GCO", "NotFound"),
    ] {
        let r = card.create_file(path);
        assert_eq!(r.err().map(|e| std::format!("{}", e)).as_deref(), Some(expected), "{}", path);
        assert_eq!(total_refs(&card), 0, "{}", path);
    }
    // A file open for reading can not be created again
    let mut benchy = card.open_file_path("/BENCHY.GCO").unwrap();
    assert!(matches!(card.create_file("/BENCHY.GCO"), Err(SDCardError::Busy)));
    card.release_path(&mut benchy);
    assert_eq!(total_refs(&card), 0);
    // Not mounted
    let mut blank: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(false), 0).await;
    assert!(matches!(blank.create_file("/X.GCO"), Err(SDCardError::NoSuchVolume)));
    // No free entry slot for the file: root + GCODE fill a table of 2
    let mut small: SDStateManager<_, 2, 2, 2> = SDStateManager::new(disk(true), 0).await;
    assert!(matches!(small.create_file("/GCODE/X.GCO"), Err(SDCardError::MaxOpenDirs)));
    assert_eq!(total_refs(&small), 0);
    // Device write errors (card removed)
    let d = disk(true);
    let present = d.present.clone();
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(d, 0).await;
    let mut stack = card.create_file("/X.GCO").unwrap();
    present.store(false, Ordering::SeqCst);
    let big = [b'G'; 1024];
    assert!(matches!(card.write(*stack.last().unwrap(), &big), Err(SDCardError::WriteError)));
    card.release_path(&mut stack);
}

// ── Long file names (VFAT) ─────────────────────────────────────────────────────────────────────

/// The VFAT entries of `long` (last segment first, as on disk) for the short name `short`
fn lfn_entries(long: &str, short: &[u8; 11]) -> Vec<[u8; 32]> {
    let checksum = short
        .iter()
        .fold(0u8, |sum, b| ((sum & 1) << 7).wrapping_add(sum >> 1).wrapping_add(*b));
    let mut units: Vec<u16> = long.encode_utf16().collect();
    if units.len() % 13 != 0 {
        units.push(0);
    }
    while units.len() % 13 != 0 {
        units.push(0xFFFF);
    }
    let count = units.len() / 13;
    (0..count)
        .rev()
        .map(|i| {
            let mut e = [0u8; 32];
            e[0] = (i as u8 + 1) | if i + 1 == count { 0x40 } else { 0 };
            e[11] = 0x0F;
            e[13] = checksum;
            let chunk = &units[i * 13..i * 13 + 13];
            for (k, unit) in chunk.iter().enumerate() {
                let at = match k {
                    0..=4 => 1 + 2 * k,
                    5..=10 => 14 + 2 * (k - 5),
                    _ => 28 + 2 * (k - 11),
                };
                put_u16(&mut e, at, *unit);
            }
            e
        })
        .collect()
}

/// Writes the long name entries and the short one from `slot`. Returns the next free slot
fn named_entry(b: &mut [u8], slot: usize, long: &str, short: &[u8; 11], attributes: u8, cluster: u16, size: u32) -> usize {
    let mut slot = slot;
    for e in lfn_entries(long, short) {
        b[slot * 32..slot * 32 + 32].copy_from_slice(&e);
        slot += 1;
    }
    dir_entry(b, slot, short, attributes, cluster, size);
    slot + 1
}

const LONG_DIR: &str = "Long Folder Name";
const LONG_FILE: &str = "A really long file name.gcode";
const LONG_ROOT_FILE: &str = "My Benchy Print.gcode";
const LONG_CONTENT: &[u8] = b"G28\nM117 long\n";

/// The disk with long names: `/My Benchy Print.gcode` (`MYBENC~1.GCO`, the content of
/// `BENCHY.GCO`) and `/Long Folder Name/` (`LONGFO~1`, cluster 7) holding
/// `A really long file name.gcode` (`AREALL~1.GCO`, cluster 8)
fn lfn_disk() -> RamDisk {
    let d = disk(true);
    {
        let mut blocks = d.blocks.borrow_mut();
        let root = blocks.get_mut(&ROOT_DIR_BLOCK).unwrap();
        let slot = named_entry(root, 4, LONG_ROOT_FILE, b"MYBENC~1GCO", 0x20, 3, BENCHY.len() as u32);
        named_entry(root, slot, LONG_DIR, b"LONGFO~1   ", 0x10, 7, 0);
        let fat = blocks.get_mut(&(LBA_START + 1)).unwrap();
        put_u16(fat, 7 * 2, 0xFFFF);
        put_u16(fat, 8 * 2, 0xFFFF);
        let mut dir = [0u8; 512];
        dir_entry(&mut dir, 0, b".          ", 0x10, 7, 0);
        dir_entry(&mut dir, 1, b"..         ", 0x10, 0, 0);
        named_entry(&mut dir, 2, LONG_FILE, b"AREALL~1GCO", 0x20, 8, LONG_CONTENT.len() as u32);
        blocks.insert(cluster_block(7), dir);
        let mut content = [0u8; 512];
        content[..LONG_CONTENT.len()].copy_from_slice(LONG_CONTENT);
        blocks.insert(cluster_block(8), content);
    }
    d
}

fn long_names_in<const D: usize, const F: usize, const E: usize>(
    card: &mut SDStateManager<RamDisk, D, F, E>,
    dir: EntryRef,
) -> Vec<(String, Option<String>)> {
    let mut names = Vec::new();
    card.list_dir_lfn(&dir, |e, long| {
        names.push((short_name(&e.name).to_string(), long.map(String::from)));
        core::ops::ControlFlow::Continue(())
    });
    names
}

#[futures_test::test]
async fn long_names_are_listed() {
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(lfn_disk(), 0).await;
    let mut root = card.open_path("/").unwrap();
    let names = long_names_in(&mut card, root[0]);
    assert!(names.contains(&("BENCHY.GCO".to_string(), None)), "{:?}", names);
    assert!(names.contains(&("MYBENC~1.GCO".to_string(), Some(LONG_ROOT_FILE.to_string()))), "{:?}", names);
    assert!(names.contains(&("LONGFO~1".to_string(), Some(LONG_DIR.to_string()))), "{:?}", names);
    // The listing can stop early
    let mut seen = 0;
    card.list_dir_lfn(&root[0], |_, _| {
        seen += 1;
        core::ops::ControlFlow::Break(())
    });
    assert_eq!(seen, 1);
    card.release_path(&mut root);
    assert_eq!(total_refs(&card), 0);
}

#[futures_test::test]
async fn paths_are_resolved_by_short_or_long_names() {
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(lfn_disk(), 0).await;
    for path in [
        "/LONGFO~1/AREALL~1.GCO",
        "/Long Folder Name/A really long file name.gcode",
        // Case-insensitive, as FAT
        "/long folder name/a REALLY long FILE name.GCODE",
        "/longfo~1/A really long file name.gcode",
        "Long Folder Name/./AREALL~1.GCO",
    ] {
        let mut stack = card.open_file_path(path).unwrap_or_else(|e| panic!("{}: {}", path, e));
        assert_eq!(stack.len(), 3, "{}", path);
        assert_eq!(read_all(&mut card, *stack.last().unwrap()), LONG_CONTENT, "{}", path);
        card.release_path(&mut stack);
        assert_eq!(total_refs(&card), 0, "{}", path);
    }
    // The same entry, named both ways, is opened once (shared)
    let mut by_long = card.open_path("/Long Folder Name").unwrap();
    let mut by_short = card.open_path("/LONGFO~1").unwrap();
    assert_eq!(by_long[1].0, by_short[1].0);
    card.release_path(&mut by_long);
    card.release_path(&mut by_short);
    assert_eq!(total_refs(&card), 0);
    // A root file by its long name
    let mut stack = card.open_file_path("/my benchy print.gcode").unwrap();
    assert_eq!(read_all(&mut card, stack[1]), BENCHY);
    card.release_path(&mut stack);
    // Unknown names
    crate::info!("Expecting Err: unknown long names are not found");
    for path in ["/Long Folder/A really long file name.gcode", "/Long Folder Name/missing file.gcode"] {
        assert!(matches!(card.open_file_path(path), Err(SDCardError::NotFound)), "{}", path);
        assert_eq!(total_refs(&card), 0, "{}", path);
    }
}

#[futures_test::test]
async fn the_long_path_of_a_path() {
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(lfn_disk(), 0).await;
    let long = |card: &mut SDStateManager<_, 4, 4, 8>, path: &str| {
        let mut out = heapless::String::<256>::new();
        card.long_path(path, &mut out).map(|_| out.to_string())
    };
    let expected = std::format!("/{}/{}", LONG_DIR, LONG_FILE);
    assert_eq!(long(&mut card, "/LONGFO~1/AREALL~1.GCO").unwrap(), expected);
    assert_eq!(long(&mut card, "LONGFO~1/AREALL~1.GCO").unwrap(), expected);
    assert_eq!(long(&mut card, "/Long Folder Name/AREALL~1.GCO").unwrap(), expected);
    // Entries without a long name keep their short one
    assert_eq!(long(&mut card, "/GCODE/PART.GCO").unwrap(), "/GCODE/PART.GCO");
    assert_eq!(long(&mut card, "/BENCHY.GCO").unwrap(), "/BENCHY.GCO");
    assert_eq!(long(&mut card, "/MYBENC~1.GCO").unwrap(), std::format!("/{}", LONG_ROOT_FILE));
    assert_eq!(long(&mut card, "/LONGFO~1/").unwrap(), std::format!("/{}", LONG_DIR));
    assert_eq!(long(&mut card, "/").unwrap(), "/");
    assert_eq!(total_refs(&card), 0);
    crate::info!("Expecting Err: the long path of a missing entry, or of a file used as a directory");
    assert!(matches!(long(&mut card, "/LONGFO~1/MISSING.GCO"), Err(SDCardError::NotFound)));
    assert!(matches!(long(&mut card, "/BENCHY.GCO/X.GCO"), Err(SDCardError::NotFound)));
    assert_eq!(total_refs(&card), 0);
    // Truncated when it does not fit
    let mut out = heapless::String::<8>::new();
    assert!(card.long_path("/MYBENC~1.GCO", &mut out).is_ok());
    assert_eq!(out.as_str(), "/My Benc");
}

#[futures_test::test]
async fn an_existing_file_named_by_its_long_name_is_truncated() {
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(lfn_disk(), 0).await;
    let path = std::format!("/{}/{}", LONG_DIR, LONG_FILE);
    let mut stack = card.create_file(&path).unwrap();
    card.write(*stack.last().unwrap(), b"M115\n").unwrap();
    card.release_path(&mut stack);
    let mut stack = card.open_file_path("/LONGFO~1/AREALL~1.GCO").unwrap();
    assert_eq!(read_all(&mut card, *stack.last().unwrap()), b"M115\n");
    card.release_path(&mut stack);
    // A new file needs a short name
    crate::info!("Expecting Err: a new long name can not be created");
    assert!(matches!(card.create_file("/Another long name.gcode"), Err(SDCardError::InvalidName)));
    assert_eq!(total_refs(&card), 0);
}
