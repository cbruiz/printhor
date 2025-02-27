use crate::hwa;
use hwa::HwiContract;
use hwa::controllers::{PlanEntry, motion};

/// A ring buffer data structure designed for storing `PlanEntry` items efficiently.
///
/// The `RingBuffer` maintains a circular buffer of fixed size. The `head` and `used` variables keep
/// track of the current position in the buffer and the number of items stored, respectively.
///
/// # Fields
///
/// * `data` - An array of `PlanEntry` elements representing the buffer storage.
/// * `head` - An index pointing to the start of the buffer.
/// * `used` - The number of elements currently stored in the buffer.
pub struct RingBuffer {
    /// An array of `PlanEntry` elements representing the buffer storage.
    pub data: [PlanEntry; hwa::Contract::SEGMENT_QUEUE_SIZE as usize],
    /// An index pointing to the start of the buffer.
    pub head: u8,
    /// The number of elements currently stored in the buffer
    pub used: u8,
}

impl RingBuffer {
    /// Creates a new `RingBuffer` with an initial empty state.
    pub const fn new() -> Self {
        Self {
            data: [PlanEntry::Empty; hwa::Contract::SEGMENT_QUEUE_SIZE as usize],
            head: 0,
            used: 0,
        }
    }

    /// Returns the index of an element from the tail of the buffer based on the given offset.
    ///
    /// # Arguments
    ///
    /// * `offset` - The relative offset from the tail of the buffer.
    ///
    /// # Returns
    ///
    /// * `Ok(u8)` - The index of the element from the tail.
    /// * `Err(())` - If the offset is greater than the number of used elements.
    ///
    /// # Examples
    /// TBD
    pub fn index_from_tail(&self, offset: u8) -> Result<u8, ()> {
        if offset > self.used {
            Err(())
        } else {
            let absolute_offset = self.head as u16 + self.used as u16 - offset as u16;
            let len = self.data.len() as u16;
            match absolute_offset < len {
                true => Ok(absolute_offset as u8),
                false => Ok((absolute_offset - len) as u8),
            }
        }
    }

    /// Returns an immutable reference to an entry from the tail of the buffer based on the given offset.
    ///
    /// # Arguments
    ///
    /// * `offset` - The relative offset from the tail of the buffer.
    ///
    /// # Returns
    ///
    /// * `Option<&PlanEntry>` - The reference to the entry from the tail.
    pub fn entry_from_tail(&self, offset: u8) -> Option<&PlanEntry> {
        let absolute_offset = self.head as u16 + self.used as u16 - offset as u16;
        let len = self.data.len() as u16;

        let index = match absolute_offset < len {
            true => absolute_offset as u8,
            false => (absolute_offset - len) as u8,
        };
        self.data.get(index as usize)
    }

    /// Returns a mutable reference to an entry from the tail of the buffer based on the given offset.
    ///
    /// # Arguments
    ///
    /// * `offset` - The relative offset from the tail of the buffer.
    ///
    /// # Returns
    ///
    /// * `Option<&mut PlanEntry>` - The mutable reference to the entry from the tail.
    #[allow(unused)]
    pub fn mut_entry_from_tail(&mut self, offset: u8) -> Option<&mut PlanEntry> {
        match self.index_from_tail(offset) {
            Ok(index) => self.data.get_mut(index as usize),
            Err(_e) => None,
        }
    }

    /// Returns a mutable reference to a `motion::Segment` from the tail of the buffer based on the given offset.
    ///
    /// # Arguments
    ///
    /// * `offset` - The relative offset from the tail of the buffer.
    ///
    /// # Returns
    ///
    /// * `Result<&mut motion::Segment, ()>` - The mutable reference to the segment or an error if the entry is not a `PlannedMove`.
    #[allow(unused)]
    pub fn mut_planned_segment_from_tail(
        &mut self,
        offset: u8,
    ) -> Result<&mut motion::Segment, ()> {
        match self.mut_entry_from_tail(offset) {
            Some(PlanEntry::PlannedMove(_s, _, _, _, _)) => Ok(_s),
            _ => Err(()),
        }
    }

    /// Returns an immutable reference to a `motion::Segment` from the tail of the buffer based on the given offset.
    ///
    /// # Arguments
    ///
    /// * `offset` - The relative offset from the tail of the buffer.
    ///
    /// # Returns
    ///
    /// * `Result<&motion::Segment, ()>` - The reference to the segment or an error if the entry is not a `PlannedMove`.
    pub fn planned_segment_from_tail(&self, offset: u8) -> Result<&motion::Segment, ()> {
        match self.entry_from_tail(offset) {
            Some(PlanEntry::PlannedMove(_s, _, _, _, _)) => Ok(_s),
            _ => Err(()),
        }
    }

    /// Returns mutable references to two entries from the tail of the buffer based on the given offsets.
    ///
    /// # Arguments
    ///
    /// * `offset1` - The relative offset for the first entry.
    /// * `offset2` - The relative offset for the second entry.
    ///
    /// # Returns
    ///
    /// * `(Option<&mut PlanEntry>, Option<&mut PlanEntry>)` -
    ///   A tuple containing the mutable references to the entries from the tail.
    #[allow(unused)]
    pub fn entries_from_tail(
        &mut self,
        offset1: u8,
        offset2: u8,
    ) -> (Option<&mut PlanEntry>, Option<&mut PlanEntry>) {
        let len = self.data.len();
        let absolute_offset1 = self.head as usize + self.used as usize - offset1 as usize;
        let index1 = match absolute_offset1 < len {
            true => absolute_offset1,
            false => absolute_offset1 - len,
        };
        let absolute_offset2 = self.head as usize + self.used as usize - offset2 as usize;
        let index2 = match absolute_offset2 < len {
            true => absolute_offset2,
            false => absolute_offset2 - len,
        };
        if index1 == index2 {
            (self.data.get_mut(index1), None)
        } else if index1 < index2 {
            let (l, r) = self.data.split_at_mut(index2);
            (l.get_mut(index1), r.get_mut(0))
        } else {
            let (l, r) = self.data.split_at_mut(index1);
            (r.get_mut(0), l.get_mut(index2))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hwa::controllers::MovType;
    use printhor_hwa_common::CommChannel;

    #[test]
    fn test_index_from_tail() {
        let mut rb = RingBuffer::new();
        rb.head = 1;
        rb.used = 3;

        assert_eq!(rb.index_from_tail(0), Ok(4));
        assert_eq!(rb.index_from_tail(1), Ok(3));
        assert_eq!(rb.index_from_tail(2), Ok(2));
        assert_eq!(rb.index_from_tail(3), Ok(1));
        assert_eq!(rb.index_from_tail(4), Err(()));
    }

    #[test]
    fn test_entry_from_tail() {
        let mut rb = RingBuffer::new();
        rb.head = 1;
        rb.used = 1;
        rb.data[0] = PlanEntry::Executing(MovType::Homing(CommChannel::Internal), true, 1);

        let t = rb.planned_segment_from_tail(1);
        assert!(t.is_err(), "No planned entry");

        let t = rb.mut_entry_from_tail(1);
        drop(t);
        let t = rb.mut_entry_from_tail(1);
        drop(t);
        let _t = rb.entries_from_tail(0, 1);
        let _t = rb.planned_segment_from_tail(0);
        /*
        let mut rb = RingBuffer::new();
        rb.head = 1;
        rb.used = 3;
        rb.data[1] = PlanEntry::Executing(MovType::Homing(CommChannel::Internal), true);
        rb.data[2] = PlanEntry::Dwell(CommChannel::Internal, true);
        rb.data[3] = PlanEntry::Homing(CommChannel::Internal, true);

        assert!(matches!(rb.entry_from_tail(0), Some(PlanEntry::Executing(..))));
        assert!(matches!(rb.entry_from_tail(1), Some(PlanEntry::Dwell(..))));
        assert!(matches!(rb.entry_from_tail(2), Some(PlanEntry::Homing(..))));
        assert!(rb.entry_from_tail(3).is_none());

         */
    }
}
