//! Motion Ring buffer (planning -> execution)
use crate::hwa;
use hwa::HwiContract;
use hwa::controllers::{PlanEntry, motion_control};

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
    /// Tool actions to apply right after the entry of the same slot was executed (see
    /// `MotionPlanner::schedule_tool_action`). Kept apart from the entries: they never take a
    /// slot, so they neither shorten the look-ahead nor break the motion chaining
    pub post_tools: [hwa::controllers::ToolActions; hwa::Contract::SEGMENT_QUEUE_SIZE as usize],
    /// An index pointing to the start of the buffer.
    pub head: u8,
    /// The number of elements currently stored in the buffer
    pub used: u8,
    /// Newest moves appended since the last motion chaining (see `MotionPlanner`: the chaining is
    /// batched)
    #[cfg(feature = "with-motion-chaining")]
    pub unchained: u8,
    /// Non-blocking issuer (a host over a serial port) whose last command filled the queue: it is
    /// answered (`DeferEvent::Completed`) as soon as a slot is freed, see `MotionPlanner`
    pub slot_waiter: Option<(hwa::DeferAction, hwa::CommChannel, u32)>,
    /// Instant (µs since boot, wrapping) the last move was appended; 0 when the next interval
    /// must not be measured (start, queue full, non-move entry). See [RingBuffer::sample_move_period]
    #[cfg(feature = "with-motion-chaining")]
    pub last_move_us: u32,
    /// Sum (µs) and count of the intervals of the current block
    #[cfg(feature = "with-motion-chaining")]
    period_acc_us: u32,
    #[cfg(feature = "with-motion-chaining")]
    period_n: u8,
    /// Average time (µs) the planner takes to produce a move, 0 when unknown (throughput slowdown,
    /// see `motion::throughput_speed_cap`)
    #[cfg(feature = "with-motion-chaining")]
    pub move_period_us: u32,
    /// Target speed the throughput slowdown gave to the last move, 0 when it was not slowed down
    /// (see `MotionPlanner`: it rises by at most 2% per move)
    #[cfg(feature = "with-motion-chaining")]
    pub slowdown_speed: hwa::math::Real,
}

impl RingBuffer {
    /// Creates a new `RingBuffer` with an initial empty state.
    pub const fn new() -> Self {
        Self {
            data: [PlanEntry::Empty; hwa::Contract::SEGMENT_QUEUE_SIZE as usize],
            post_tools: [hwa::controllers::ToolActions::NONE;
                hwa::Contract::SEGMENT_QUEUE_SIZE as usize],
            head: 0,
            used: 0,
            #[cfg(feature = "with-motion-chaining")]
            unchained: 0,
            slot_waiter: None,
            #[cfg(feature = "with-motion-chaining")]
            last_move_us: 0,
            #[cfg(feature = "with-motion-chaining")]
            period_acc_us: 0,
            #[cfg(feature = "with-motion-chaining")]
            period_n: 0,
            #[cfg(feature = "with-motion-chaining")]
            move_period_us: 0,
            #[cfg(feature = "with-motion-chaining")]
            slowdown_speed: hwa::math::ZERO,
        }
    }

    /// Longest interval between two appended moves taken as a production time: a longer one is
    /// an idle gap (no G-code to plan), not the cost of planning a move
    #[cfg(feature = "with-motion-chaining")]
    const MAX_MOVE_PERIOD_US: u32 = 50_000;

    /// Updates the average production time of a move with the interval since the previous move
    /// was appended (`now_us`: µs since boot, wrapping). The interval is the whole cost of a move
    /// as the planner sees it (G-code input and parsing, planning, chaining, and the CPU the
    /// stepper task and the interrupts take meanwhile), unless it was broken (see
    /// [RingBuffer::break_move_period]) or is an idle gap.
    ///
    /// The intervals are averaged over blocks of `block` moves (the chaining batch: one move of
    /// each batch also pays the chaining, a periodic pattern that a plain exponential average
    /// turns into a speed ripple), and the block means are averaged exponentially (⅛, ~64 moves:
    /// an occasional long chaining, 40+ ms on the SKR Mini E3, is spread instead of making the
    /// speed dip)
    #[cfg(feature = "with-motion-chaining")]
    pub fn sample_move_period(&mut self, now_us: u32, block: u8) {
        let dt = now_us.wrapping_sub(self.last_move_us);
        if self.last_move_us != 0 && dt <= Self::MAX_MOVE_PERIOD_US {
            self.period_acc_us += dt;
            self.period_n += 1;
            if self.period_n >= block.max(1) {
                let mean = self.period_acc_us / self.period_n as u32;
                self.move_period_us = match self.move_period_us {
                    0 => mean,
                    p => p - p / 8 + mean / 8,
                };
                self.period_acc_us = 0;
                self.period_n = 0;
            }
        }
        self.last_move_us = now_us.max(1);
    }

    /// The next interval between appended moves is not a production time (the planner waited on
    /// a full queue, or something else than a move was queued)
    #[cfg(feature = "with-motion-chaining")]
    pub fn break_move_period(&mut self) {
        self.last_move_us = 0;
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
        if offset > self.used {
            return None;
        }
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
    /// * `Result<&mut motion_control::Segment, ()>` - The mutable reference to the segment or an error if the entry is not a `PlannedMove`.
    #[allow(unused)]
    pub fn mut_planned_segment_from_tail(
        &mut self,
        offset: u8,
    ) -> Result<&mut motion_control::Segment, ()> {
        match self.mut_entry_from_tail(offset) {
            Some(PlanEntry::PlannedMove(_s, _, _, _, _)) => Ok(_s),
            _ => Err(()),
        }
    }

    /// Returns an immutable reference to a `motion_control::Segment` from the tail of the buffer based on the given offset.
    ///
    /// # Arguments
    ///
    /// * `offset` - The relative offset from the tail of the buffer.
    ///
    /// # Returns
    ///
    /// * `Result<&motion_control::Segment, ()>` - The reference to the segment or an error if the entry is not a `PlannedMove`.
    pub fn planned_segment_from_tail(&self, offset: u8) -> Result<&motion_control::Segment, ()> {
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

    /// The production time is the block mean of the intervals (a periodic pattern does not leak
    /// into it), broken intervals and idle gaps are skipped, and the time may wrap
    #[cfg(feature = "with-motion-chaining")]
    #[test]
    fn test_sample_move_period() {
        let mut rb = RingBuffer::new();
        let mut t = u32::MAX - 20_000;
        rb.sample_move_period(t, 4);
        for dt in [1_000u32, 1_000, 1_000, 9_000] {
            t = t.wrapping_add(dt);
            rb.sample_move_period(t, 4);
        }
        assert_eq!(rb.move_period_us, 3_000);
        // Broken interval and idle gap: not counted
        rb.break_move_period();
        t = t.wrapping_add(500);
        rb.sample_move_period(t, 4);
        t = t.wrapping_add(200_000);
        rb.sample_move_period(t, 4);
        for _ in 0..4 {
            t = t.wrapping_add(5_000);
            rb.sample_move_period(t, 4);
        }
        // 3000 + (5000 - 3000) / 8
        assert_eq!(rb.move_period_us, 3_250);
    }

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
