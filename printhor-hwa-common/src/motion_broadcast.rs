//! # Motion Broadcast Module
//!
//! This module [...]

use crate as hwa;
use crate::math;
use crate::math::TVector;
use embassy_sync::channel::Channel;
//#region "MotionBroadcastEvent"

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum MotionBroadcastEvent {
    Reset,
    Delta(MotionDelta),
}

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub struct MotionDelta {
    pub order_num: u32,
    pub micro_segment_id: u32,
    /// The microsegment relative time in seconds
    pub micro_segment_time: hwa::math::Real,
    /// The instant position in steps
    pub pos_wu: TVector<hwa::math::Real>,
    /// The instant position in steps
    pub pos_steps: TVector<i32>,
}

impl MotionDelta {
    pub const fn new() -> Self {
        Self {
            order_num: 0,
            micro_segment_id: 0,
            micro_segment_time: math::ZERO,
            pos_wu: TVector::new(),
            pos_steps: TVector::new(),
        }
    }
}

//#endregion

//#region "Motion Broadcast Channel"

/// The queue size of the motion broadcast channel
#[const_env::from_env("MOTION_BROADCAST_CHANNEL_SIZE")]
pub const MOTION_BROADCAST_CHANNEL_SIZE: usize = 1;

pub type MotionBroadcastChannelType<M> =
    Channel<M, MotionBroadcastEvent, MOTION_BROADCAST_CHANNEL_SIZE>;

pub struct GenericMotionBroadcastChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    channel: &'static MotionBroadcastChannelType<M>,
}

impl<M> GenericMotionBroadcastChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    pub const fn new(channel: &'static MotionBroadcastChannelType<M>) -> Self {
        GenericMotionBroadcastChannel { channel }
    }
}
impl<M> core::ops::Deref for GenericMotionBroadcastChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    type Target = MotionBroadcastChannelType<M>;

    fn deref(&self) -> &Self::Target {
        self.channel
    }
}

impl<M> Clone for GenericMotionBroadcastChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    fn clone(&self) -> Self {
        GenericMotionBroadcastChannel::new(self.channel)
    }
}

//#endregion
