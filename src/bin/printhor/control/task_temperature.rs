//! The Temperature controller task

use crate::hwa;
use crate::hwa::controllers::HeaterController;
use embassy_time::{Duration, Ticker};
use hwa::DeferAction;
use hwa::{EventFlags, EventStatus};
#[cfg(not(feature = "native"))]
use num_traits::float::FloatCore;
use num_traits::ToPrimitive;
use printhor_hwa_utils::{MutexStrategy, StaticController};
use std::ops::Deref;

///
/// # Asynchronous Task Entry Point
///
/// This function serves as the entry point for the temperature task that runs asynchronously.
///
/// # Arguments
///
/// * `event_bus` - A reference to the event bus.
///
/// `#[cfg(feature = "with-hot-end")]` - If enabled, receives a reference to the hot end controller.
///
/// `#[cfg(feature = "with-hot-bed")]` - If enabled, receives a reference to the hot bed controller.
///
/// # Description
///
/// This method initializes and runs an infinite loop where the state machines for the hot end and hot bed (if enabled) are updated.
/// Within the loop:
///
/// 1. It waits for an interval of 2 seconds.
/// 2. It calls the `update` method of the heater state machines (hot end and hot bed) if they are enabled, providing references to their respective controllers and the event bus.
///
/// Note that this function uses async/await to coordinate asynchronous operations.
#[embassy_executor::task(pool_size = 1)]
pub async fn task_temperature(
    event_bus: hwa::types::EventBus,
    #[cfg(feature = "with-hot-end")] hot_end_controller: hwa::types::HotEndController,
    #[cfg(feature = "with-hot-bed")] hot_bed_controller: hwa::types::HotBedController,
) {
    hwa::info!("[task_temperature] Started");

    let mut ticker = Ticker::every(Duration::from_secs(2));

    loop {
        // TODO: Park on SYS_ALARM
        ticker.next().await;

        #[cfg(test)]
        if crate::control::task_integration::INTEGRATION_STATUS.signaled() {
            hwa::info!("[task_temperature] Ending gracefully");
            return ();
        }
        #[cfg(feature = "with-hot-end")]
        hot_end_controller.lock().await.update(&event_bus).await;

        #[cfg(feature = "with-hot-bed")]
        hot_end_controller.lock().await.update(&event_bus).await;
    }
}
