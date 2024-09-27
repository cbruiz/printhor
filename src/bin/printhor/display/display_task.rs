use crate::display::ui::MainUI;
use crate::hwa;
use embassy_time;
use printhor_hwa_common::EventBusRef;

#[embassy_executor::task(pool_size = 1)]
pub async fn task_display(display_dev: hwa::display::DisplayDevice, event_bus: EventBusRef) {
    hwa::info!("Display task started");
    let t0 = embassy_time::Instant::now();
    let main_ui = MainUI::new(event_bus.clone()).await;
    let mut tft = hwa::device::DisplayScreen::new(display_dev, main_ui).await;
    hwa::info!("D; display init done in {} ms ", t0.elapsed().as_millis());

    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(1000));
    #[cfg(feature = "with-lvgl")]
    let mut ticks = tft.ticks();
    let mut _t1 = embassy_time::Instant::now();
    loop {
        #[cfg(test)]
        if crate::control::task_integration::INTEGRATION_STATUS.signaled() {
            hwa::info!("task_display ending");
            return ();
        }
        hwa::debug!("D; lvgl display refresh last: {}", t0.elapsed().as_millis());
        #[cfg(feature = "with-lvgl")]
        ticks.inc(_t1.elapsed().as_millis() as u32);
        _t1 = embassy_time::Instant::now();
        tft.update().await;
        ticker.next().await;
    }
}
