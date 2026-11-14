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
    blocks: BTreeMap<u32, [u8; 512]>,
    bad_block: u32,
    retained: Arc<AtomicUsize>,
    released: Arc<AtomicUsize>,
}

impl BlockDevice for RamDisk {
    type Error = SDCardError;

    fn read(&self, blocks: &mut [Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        for (i, b) in blocks.iter_mut().enumerate() {
            if start_block_idx.0 + i as u32 == self.bad_block {
                return Err(SDCardError::InternalError);
            }
            b.contents = self
                .blocks
                .get(&(start_block_idx.0 + i as u32))
                .copied()
                .unwrap_or([0u8; 512]);
        }
        Ok(())
    }

    fn write(&self, _blocks: &[Block], _start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        Err(SDCardError::NotYetImplemented)
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        Ok(BlockCount(LBA_START + VOLUME_BLOCKS))
    }
}

impl hwa::traits::AsyncSDBlockDevice for RamDisk {
    fn do_retain(&self) -> impl core::future::Future<Output = Result<(), ()>> + use<> {
        let retained = self.retained.clone();
        async move {
            retained.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
    }

    fn do_release(&self) -> Result<(), ()> {
        self.released.fetch_add(1, Ordering::SeqCst);
        Ok(())
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
        blocks,
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
    // Partition 2 is empty
    let mut card: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(true), 1).await;
    assert!(matches!(card.open_path("/"), Err(SDCardError::NoSuchVolume)));
    // Blank disk: no MBR signature
    let mut blank: SDStateManager<_, 4, 4, 8> = SDStateManager::new(disk(false), 0).await;
    assert!(matches!(blank.open_file_path("/BENCHY.GCO"), Err(SDCardError::NoSuchVolume)));
}

#[futures_test::test]
async fn capacity_limits_are_reported_without_leaking_entries() {
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
