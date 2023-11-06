// TODO: Intense refactor and cleanup
//use std::mem::MaybeUninit;

//use embedded_graphics_core::geometry::{Dimensions, Point};
use embedded_graphics_core::geometry::Size;
#[allow(unused)]
use embedded_graphics_simulator::{OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window};
use embedded_graphics_core::pixelcolor::{Rgb565};
//use embedded_graphics_core::Drawable;
//use embedded_graphics::mono_font::{mapping::StrGlyphMapping, DecorationDimensions, MonoFont, MonoTextStyle};
//use embedded_graphics::mono_font::ascii::FONT_6X10;
//use embedded_graphics::text::Text;
//use embedded_graphics_core::draw_target::DrawTarget;
//use embedded_graphics_core::primitives::Rectangle;

#[cfg(feature = "with-lvgl")]
use lvgl::core::{Display, Lvgl, ObjExt, Screen, Ticks};
//use printhor_hwa_common::{EventBusRef, EventBusSubscriber, EventFlags, DisplayScreenUI};
use printhor_hwa_common::DisplayScreenUI;

//use embedded_graphics_core::Pixel;

const WIDTH: u32 = 240;
const HEIGHT: u32 = 320;
#[cfg(feature = "lvgl")]
const LVGL_BUFFER_LEN: usize = 16usize * WIDTH as usize * 2usize;

pub type PixelColor = Rgb565;

pub struct SimulatorDisplayDevice {

}

impl SimulatorDisplayDevice {
    pub fn new() -> Self {
        Self {}
    }
}

pub struct SimulatorDisplayScreen<UI>
where
    UI: DisplayScreenUI,
{
    _device: SimulatorDisplayDevice,
    raw_display: SimulatorDisplay<PixelColor>,
    main_ui: UI,
    simulator_window: Window,
}

impl<UI: DisplayScreenUI> SimulatorDisplayScreen<UI>
{
    pub async fn new(_device: SimulatorDisplayDevice, main_ui: UI) -> Self {
        let output_settings = OutputSettingsBuilder::new().build();
        let raw_display = SimulatorDisplay::new(Size::new(WIDTH, HEIGHT));
        Self {
            _device,
            main_ui,
            raw_display: raw_display,
            simulator_window: Window::new("PrinThor TFT", &output_settings),
        }
    }

    pub async fn update(&mut self) {
        #[cfg(feature = "with-lvgl")]
        self.main_ui.context().as_mut().unwrap().refresh().await;
        #[cfg(not(feature = "with-lvgl"))]
        self.main_ui.refresh(&mut self.raw_display).await;
        #[cfg(feature = "with-lvgl")]
        self.lvgl.run_tasks();
        #[cfg(feature = "with-lvgl")]
        self.simulator_window.update(&self.lvgl_display);
        #[cfg(not(feature = "with-lvgl"))]
        self.simulator_window.update(&self.raw_display);

        self.simulator_window.events().for_each(|_e| {
            //println!("Event! {:?}", _e)
        });
    }

    pub async fn retain(&mut self)  {

    }

    pub async fn release(&mut self)  {

    }
}

/*

TODO: Obsolete code pending to refactor/readapt

pub struct TFTDisplay {
    #[cfg(feature = "with-lvgl")]
    lvgl: Lvgl,
    simulator_window: Window,
    #[cfg(feature = "with-lvgl")]
    lvgl_display: Display<SimulatorDisplay<PixelColor>>,
    #[cfg(not(feature = "with-lvgl"))]
    raw_display: SimulatorDisplay<PixelColor>,
    #[cfg(feature = "with-lvgl")]
    main_ui: Screen<lvgl_ui::MotionScreen>,
    #[cfg(not(feature = "with-lvgl"))]
    main_ui: DirectUI,
}

impl TFTDisplay {
    pub async fn new(_display_dev: hwa::display::DisplayDevice, event_bus: EventBusRef) -> Self {

        info!("BUFF prec size:{}", core::mem::size_of::<PixelColor>() * LVGL_BUFFER_LEN);

        // Display init with its draw buffer
        static mut DRAW_BUFFER: [MaybeUninit<PixelColor>; LVGL_BUFFER_LEN] =
            [MaybeUninit::<PixelColor>::uninit(); LVGL_BUFFER_LEN];

        let output_settings = OutputSettingsBuilder::new().build();

        #[cfg(feature = "with-lvgl")]
            let mut lvgl = Lvgl::new();
        #[cfg(feature = "with-lvgl")]
        lvgl.register_logger(|s| info!("LVGL: {}", s));

        let mut raw_display = SimulatorDisplay::new(Size::new(WIDTH, HEIGHT));

        #[cfg(feature = "with-lvgl")]
            let mut lvgl_display = Display::new(&lvgl, raw_display, unsafe { &mut DRAW_BUFFER });

        #[cfg(feature = "with-lvgl")]
            let mut main_ui = lvgl_ui::new_screen(&lvgl_display, |screen| {
            lvgl_ui::MotionScreen::new(screen)
        });

        #[cfg(feature = "with-lvgl")]
        lvgl_display.load_screen(&mut main_ui);

        Self {
            #[cfg(feature = "with-lvgl")]
            lvgl,
            simulator_window: Window::new("PrinThor TFT", &output_settings),
            #[cfg(feature = "with-lvgl")]
            lvgl_display,
            #[cfg(not(feature = "with-lvgl"))]
            raw_display,
            #[cfg(feature = "with-lvgl")]
            main_ui,
            #[cfg(not(feature = "with-lvgl"))]
            main_ui: DirectUI::new(event_bus).await,
        }
    }
    #[allow(unused)]
    pub async fn init(&self) {

    }

    #[cfg(feature = "with-lvgl")]
    pub fn ticks(&self) -> Ticks {
        self.lvgl.ticks()
    }

    pub async fn update(&mut self) {

    }
}
*/