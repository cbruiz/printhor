#[cfg(feature = "with-lvgl")]
pub mod lvgl_ui;

//#[cfg(not(feature = "with-lvgl"))]
pub mod eg_ui;

pub type MainUI = eg_ui::EmbeddedGraphicsUI;

