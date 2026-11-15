//! Supervision of the Trinamic drivers (as Klipper does): configures them whenever they can be
//! configured (powered) and are not, and checks them periodically. [EventFlags::TMC_OK] reports
//! that every driver is configured and healthy: the motion (moves and homing) waits for it.
//!
//! Every wait of this task is bounded: it never holds the motion driver for longer than
//! [TMC_OPERATION_TIMEOUT_MS], and never waits for it longer than [TMC_LOCK_TIMEOUT_MS] (e.g.
//! while a homing holds it, the check is just skipped).
//!
//! The health check reads one driver per period (round robin) and, by default, only while the
//! machine is idle ([TMC_CHECK_WHILE_MOVING]): on a bit-banged UART (MKS Robin Nano, 1200 baud)
//! a read takes ~100 ms of a task polling the line every few tens of microseconds, which would
//! compete with the motion planning (and its jitter would corrupt the bits). A short check also
//! keeps short the hold of the motion driver, which the motion start (unpark) and homing need.
use crate::hwa;
use crate::processing;
use embassy_time::{Duration, Timer, with_timeout};
use hwa::{EventFlags, EventStatus};

/// Period of the health check of a driver while the drivers are configured: one driver per
/// period (with 4 drivers, each one is checked every 2 s)
#[const_env::env_item("TMC_CHECK_PERIOD_MS")]
const TMC_CHECK_PERIOD_MS: u64 = 500;

/// Whether the drivers are checked while the machine moves. Off by default: see the module doc
#[const_env::env_item("TMC_CHECK_WHILE_MOVING")]
const TMC_CHECK_WHILE_MOVING: bool = false;

/// The machine is busy (not idle) while any of these is set
const MOTION_BUSY_FLAGS: EventFlags = EventFlags::MOVING.union(EventFlags::HOMING);

/// Period between configuration attempts while a driver can not be configured
#[const_env::env_item("TMC_RETRY_PERIOD_MS")]
const TMC_RETRY_PERIOD_MS: u64 = 1000;

/// Period of the status watch while the drivers are not powered (or in alarm mode)
const TMC_IDLE_PERIOD_MS: u64 = 50;

/// Longest wait for the motion driver (which holds the drivers UART)
const TMC_LOCK_TIMEOUT_MS: u64 = 1000;

/// Longest configuration (or check) of every driver. A driver not answering takes
/// `INIT_ATTEMPTS` read deadlines (1 s) to be given up.
const TMC_OPERATION_TIMEOUT_MS: u64 = 15_000;

/// Longest check of a driver (two register reads at most, each bounded to 1 s)
const TMC_CHECK_TIMEOUT_MS: u64 = 3000;

/// Consecutive checks a driver must not answer before it is considered lost: a single UART
/// failure (noise) must not stop the motion
const TMC_CHECK_MAX_UNANSWERED: u8 = 3;

/// Set when the drivers have to be configured again (power cycle, new settings), so that a
/// configuration in progress (begun before) does not report them configured
static CONFIGURATION_REQUESTED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Requests the drivers to be configured again (e.g. `M80`, `M502`). The caller clears
/// [EventFlags::TMC_OK] right after: the task configures them as soon as they are powered.
pub fn request_configuration() {
    CONFIGURATION_REQUESTED.store(true, core::sync::atomic::Ordering::Release);
}

/// The outcome of an operation on the drivers, bounded by the timeouts
enum Outcome<T> {
    /// Completed
    Done(T),
    /// The motion driver was busy (e.g. homing): not attempted
    Busy,
    /// Took too long
    Timeout,
}

#[embassy_executor::task]
pub async fn task_trinamic(processor: processing::GCodeProcessor) {
    let event_bus = processor.event_bus.clone();
    // Whether a configuration failure has already been reported (once per failure streak)
    let mut failure_reported = false;
    // Consecutive unanswered checks of each driver (index of [TrinamicController::axes])
    let mut unanswered_checks = [0u8; 4];
    // The next driver to check (index of [TrinamicController::axes])
    let mut next_check: usize = 0;

    hwa::info!(
        "[task_trinamic] Started (health check: {})",
        if hwa::controllers::TrinamicController::can_read() { "on" } else { "off" }
    );

    loop {
        let status = event_bus.get_status().await;
        #[cfg(feature = "with-ps-on")]
        let powered = status.contains(EventFlags::ATX_ON);
        #[cfg(not(feature = "with-ps-on"))]
        let powered = true;

        if !powered {
            // The drivers lose their configuration without power
            if status.contains(EventFlags::TMC_OK) {
                hwa::info!("[task_trinamic] Drivers not powered");
                clear_tmc_ok(&processor).await;
            }
            failure_reported = false;
            unanswered_checks = [0; 4];
            Timer::after(Duration::from_millis(TMC_IDLE_PERIOD_MS)).await;
            continue;
        }

        if !status.contains(EventFlags::TMC_OK) {
            unanswered_checks = [0; 4];
            // Not configured in alarm mode (e.g. after a driver fault): M999 first
            if status.contains(EventFlags::SYS_ALARM) {
                Timer::after(Duration::from_millis(TMC_IDLE_PERIOD_MS)).await;
                continue;
            }
            // Requests made from now on are not covered by this configuration
            CONFIGURATION_REQUESTED.store(false, core::sync::atomic::Ordering::Release);
            match configure(&processor).await {
                Outcome::Done(Ok(())) => {
                    // Unless the status changed meanwhile (power off, a new M80/M502 request)
                    if still_powered(&processor).await
                        && !CONFIGURATION_REQUESTED.load(core::sync::atomic::Ordering::Acquire)
                    {
                        hwa::info!("[task_trinamic] Drivers configured");
                        event_bus
                            .publish_event(EventStatus::containing(EventFlags::TMC_OK))
                            .await;
                        if failure_reported {
                            processor.write_all("echo: TMC drivers configured\n").await;
                        }
                        failure_reported = false;
                    }
                }
                Outcome::Done(Err(e)) => {
                    if !failure_reported {
                        report_configuration_failure(&processor, Some(e)).await;
                        failure_reported = true;
                    }
                    Timer::after(Duration::from_millis(TMC_RETRY_PERIOD_MS)).await;
                }
                Outcome::Timeout => {
                    if !failure_reported {
                        report_configuration_failure(&processor, None).await;
                        failure_reported = true;
                    }
                    Timer::after(Duration::from_millis(TMC_RETRY_PERIOD_MS)).await;
                }
                Outcome::Busy => {
                    Timer::after(Duration::from_millis(TMC_IDLE_PERIOD_MS)).await;
                }
            }
            continue;
        }

        // Configured: periodic health check. The status is watched meanwhile: a configuration
        // request (M80, M502) or a power off is attended at once
        if !sleep_while_configured(&processor, TMC_CHECK_PERIOD_MS).await
            || !hwa::controllers::TrinamicController::can_read()
        {
            continue;
        }
        let status = event_bus.get_status().await;
        let idle = status.contains(EventFlags::MOV_QUEUE_EMPTY) && !status.intersects(MOTION_BUSY_FLAGS);
        if !idle && !TMC_CHECK_WHILE_MOVING {
            continue;
        }
        let axes = hwa::controllers::TrinamicController::axes();
        if axes.is_empty() {
            continue;
        }
        let index = next_check % axes.len();
        let axis = axes[index];
        next_check = (index + 1) % axes.len();
        match check(&processor, axis).await {
            Outcome::Done(health) => {
                if !health.fault.is_empty() {
                    // As Klipper: a driver shut down by a fault halts the machine
                    hwa::error!("[task_trinamic] Driver fault: {:?}", health.fault);
                    clear_tmc_ok(&processor).await;
                    processor.write_all("Error:TMC driver fault\n").await;
                    processor.emergency_stop("TMC driver fault").await;
                } else if !health.reset.is_empty() {
                    // The configuration is lost: configured again right away
                    hwa::warn!("[task_trinamic] Driver reset: {:?}. Configuring the drivers again", health.reset);
                    clear_tmc_ok(&processor).await;
                } else if !health.unanswered.is_empty() {
                    if count_unanswered(&mut unanswered_checks, index) {
                        hwa::error!("[task_trinamic] Driver not answering: {:?}", health.unanswered);
                        clear_tmc_ok(&processor).await;
                    }
                } else if let Some(streak) = unanswered_checks.get_mut(index) {
                    *streak = 0;
                }
            }
            Outcome::Timeout => {
                if count_unanswered(&mut unanswered_checks, index) {
                    hwa::error!("[task_trinamic] Driver check timed out: {:?}", axis);
                    clear_tmc_ok(&processor).await;
                }
            }
            // Retried in the next period
            Outcome::Busy => {}
        }
    }
}

/// Counts an unanswered check of the driver at `index`. Returns whether it is considered lost
/// ([TMC_CHECK_MAX_UNANSWERED] in a row)
fn count_unanswered(streaks: &mut [u8; 4], index: usize) -> bool {
    match streaks.get_mut(index) {
        Some(streak) => {
            *streak = streak.saturating_add(1);
            *streak >= TMC_CHECK_MAX_UNANSWERED
        }
        None => false,
    }
}

/// Sleeps `period_ms` while the drivers stay configured and powered. Returns whether they did
async fn sleep_while_configured(processor: &processing::GCodeProcessor, period_ms: u64) -> bool {
    let deadline = embassy_time::Instant::now() + Duration::from_millis(period_ms);
    while embassy_time::Instant::now() < deadline {
        Timer::after(Duration::from_millis(TMC_IDLE_PERIOD_MS)).await;
        if !processor.event_bus.has_flags(EventFlags::TMC_OK).await
            || !still_powered(processor).await
        {
            return false;
        }
    }
    true
}

/// Whether the drivers are (still) powered
async fn still_powered(_processor: &processing::GCodeProcessor) -> bool {
    #[cfg(feature = "with-ps-on")]
    {
        _processor.event_bus.has_flags(EventFlags::ATX_ON).await
    }
    #[cfg(not(feature = "with-ps-on"))]
    {
        true
    }
}

async fn clear_tmc_ok(processor: &processing::GCodeProcessor) {
    processor
        .event_bus
        .publish_event(EventStatus::not_containing(EventFlags::TMC_OK))
        .await;
}

/// Configures every driver, bounded (see [Outcome])
async fn configure(
    processor: &processing::GCodeProcessor,
) -> Outcome<Result<(), hwa::controllers::TrinamicError>> {
    let motion_driver = processor.motion_planner.motion_driver();
    let mut driver = match with_timeout(
        Duration::from_millis(TMC_LOCK_TIMEOUT_MS),
        motion_driver.lock(),
    )
    .await
    {
        Ok(driver) => driver,
        Err(_) => return Outcome::Busy,
    };
    match with_timeout(
        Duration::from_millis(TMC_OPERATION_TIMEOUT_MS),
        driver.trinamic_controller.init(),
    )
    .await
    {
        Ok(result) => Outcome::Done(result),
        Err(_) => Outcome::Timeout,
    }
}

/// Checks the driver of `axis`, bounded (see [Outcome])
async fn check(
    processor: &processing::GCodeProcessor,
    axis: hwa::CoordSel,
) -> Outcome<hwa::controllers::TrinamicHealth> {
    let motion_driver = processor.motion_planner.motion_driver();
    let mut driver = match with_timeout(
        Duration::from_millis(TMC_LOCK_TIMEOUT_MS),
        motion_driver.lock(),
    )
    .await
    {
        Ok(driver) => driver,
        Err(_) => return Outcome::Busy,
    };
    match with_timeout(
        Duration::from_millis(TMC_CHECK_TIMEOUT_MS),
        driver.trinamic_controller.check(axis),
    )
    .await
    {
        Ok(health) => Outcome::Done(health),
        Err(_) => Outcome::Timeout,
    }
}

/// Reports to every channel the drivers that could not be configured (`None`: timed out)
async fn report_configuration_failure(
    processor: &processing::GCodeProcessor,
    error: Option<hwa::controllers::TrinamicError>,
) {
    let mut msg = processing::LineBuf::<64>::new();
    msg.push_str("echo: TMC drivers not configured:");
    match error {
        Some(hwa::controllers::TrinamicError::Unconfigured(axes)) => {
            for axis in hwa::CoordSel::all_axis().iter() {
                if axes.contains(axis) {
                    msg.push_str(" ");
                    msg.push_str(axis.name());
                }
            }
        }
        Some(hwa::controllers::TrinamicError::BadSettings) => msg.push_str(" bad settings"),
        Some(_) => msg.push_str(" UART"),
        None => msg.push_str(" timeout"),
    }
    msg.push_str(". Retrying\n");
    hwa::error!("[task_trinamic] {}", msg.as_str());
    processor.write_all(msg.as_str()).await;
}
