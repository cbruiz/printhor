//! Miscellaneous utils
use crate::hwa;

/// Converts an `async-gcode` decimal representation in the format of tuple (signed_decimal: i32, number_of_decimal_digits: u8) to [hwa::math::Real]
#[inline]
pub fn to_fixed(val: (i32, u8)) -> hwa::math::Real {
    hwa::math::Real::new(val.0.into(), val.1 as u32)
}

/// Waits until `flags` are set (`set` = true) or cleared, whatever the `SYS_ALARM` status: used
/// by the startup synchronization, so an alarm raised at startup (e.g. a heater fault) does not
/// stop the boot. The system starts in alarm mode, recoverable with `M999`. Never fails.
pub async fn wait_for_flags(event_bus: &hwa::types::EventBus, flags: hwa::EventFlags, set: bool) {
    let mut warned = false;
    loop {
        let status = event_bus.get_status().await;
        if status.contains(flags) == set {
            return;
        }
        if !warned && status.contains(hwa::EventFlags::SYS_ALARM) {
            hwa::warn!("SYS_ALARM raised at startup: starting in alarm mode (M999 to recover)");
            warned = true;
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
    }
}

/// The flags the motion (moves and homing) requires: the drivers configured ([hwa::EventFlags::TMC_OK],
/// always set without `with-trinamic`) and, with `with-ps-on`, the power on.
#[cfg(feature = "with-motion")]
pub const MOTION_READY_FLAGS: hwa::EventFlags = {
    #[cfg(feature = "with-ps-on")]
    {
        hwa::EventFlags::ATX_ON.union(hwa::EventFlags::TMC_OK)
    }
    #[cfg(not(feature = "with-ps-on"))]
    {
        hwa::EventFlags::TMC_OK
    }
};

/// The longest wait for [MOTION_READY_FLAGS] (e.g. the drivers being configured right after `M80`),
/// before the move (or homing) is rejected. Bounded: nothing waits forever for a state that may
/// never come.
#[cfg(feature = "with-motion")]
#[const_env::env_item("MOTION_READY_TIMEOUT_MS")]
pub const MOTION_READY_TIMEOUT_MS: u64 = 5000;

/// Polling period of [wait_for_flags_timeout]
const FLAGS_POLL_PERIOD_MS: u64 = 10;

/// Waits until every flag of `flags` is set, for `timeout` at most.
///
/// Fails at once when `SYS_ALARM` is set (unless `flags` include it): nothing that would set them
/// runs in alarm mode. Polls the status (no subscriber is taken: their number is a static limit).
/// The common case (already set) does not wait.
pub async fn wait_for_flags_timeout(
    event_bus: &hwa::types::EventBus,
    flags: hwa::EventFlags,
    timeout: embassy_time::Duration,
) -> Result<(), ()> {
    let deadline = embassy_time::Instant::now() + timeout;
    loop {
        let status = event_bus.get_status().await;
        if status.contains(flags) {
            return Ok(());
        }
        if !flags.contains(hwa::EventFlags::SYS_ALARM)
            && status.contains(hwa::EventFlags::SYS_ALARM)
        {
            return Err(());
        }
        if embassy_time::Instant::now() >= deadline {
            return Err(());
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(FLAGS_POLL_PERIOD_MS)).await;
    }
}
