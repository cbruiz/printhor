#[cfg(feature = "with-lvgl")]
use crate::display::ui;
#[cfg(feature = "with-lvgl")]
use crate::display::ui::MotionScreen;
use crate::hwa::EventBusRef;
use crate::hwi;
use crate::hwi::device::SPIDevice;
use crate::hwi::{ControllerMutexType, IODevices, SpiControllerRef};
use crate::machine::MACHINE_INFO;
/// Based on https://github.com/yuri91/ili9341-rs
use crate::{hwa, info};
use core::cell::RefCell;
use core::cell::RefMut;
use core::iter::once;
use core::mem::MaybeUninit;
use core::slice::Iter;
use embassy_futures::block_on;
use embassy_sync::mutex::MutexGuard;
use embassy_time::{Duration, Timer};
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::geometry::Size;
use embedded_graphics_core::geometry::{Dimensions, OriginDimensions, Point};
use embedded_graphics_core::pixelcolor::raw::{RawU16, ToBytes};
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics_core::prelude::RawData;
use embedded_graphics_core::primitives::{PointsIter, Rectangle};
use embedded_graphics_core::Pixel;
#[cfg(feature = "with-lvgl")]
use lvgl::core::ObjExt;
#[cfg(feature = "with-lvgl")]
use lvgl::core::Ticks;
#[cfg(feature = "with-lvgl")]
use lvgl::core::{Display, Lvgl, Screen};

const WIDTH: u32 = 240;
const HEIGHT: u32 = 320;
#[cfg(feature = "with-lvgl")]
const LVGL_BUFFER_LEN: usize = (240 * 320) / 10;

pub type PixelColor = Rgb565;

type DisplayDevice = crate::hwa::display::DisplayDevice;
use crate::hwi::nucleo_f410rb::device::InterfaceType;
use display_interface_parallel_gpio::{DataFormat, WriteOnlyDataCommand};
use ili9341::{DisplaySize240x320, Ili9341, Orientation};

pub struct TFTDisplay {
    #[cfg(feature = "with-lvgl")]
    lvgl: Lvgl,
    #[cfg(feature = "with-lvgl")]
    lvgl_display: Display<RawDisplay>,
    #[cfg(not(feature = "with-lvgl"))]
    raw_display: RawDisplay,
    #[cfg(feature = "with-lvgl")]
    main_ui: Screen<MotionScreen>,
    #[cfg(not(feature = "with-lvgl"))]
    main_ui: Screen<MotionScreen>,
}

impl TFTDisplay {
    pub async fn new(display_dev: DisplayDevice, event_bus: EventBusRef) -> Self {
        #[cfg(feature = "with-lvgl")]
        let mut lvgl = Lvgl::new();
        #[cfg(feature = "with-lvgl")]
        lvgl.register_logger(|s| info!("LVGL: {}", s));

        #[cfg(feature = "with-lvgl")]
        info!(
            "BUFF prec size:{}",
            core::mem::size_of::<PixelColor>() * LVGL_BUFFER_LEN
        );

        // Display init with its draw buffer
        #[cfg(feature = "with-lvgl")]
        static mut DRAW_BUFFER: [MaybeUninit<PixelColor>; LVGL_BUFFER_LEN] =
            [MaybeUninit::<PixelColor>::uninit(); LVGL_BUFFER_LEN];

        info!("create raw display");
        let mut raw_display = RawDisplay::new(display_dev, event_bus).await;
        raw_display.init().await;
        let _ = raw_display.clear(PixelColor::new(255, 0, 0));

        #[cfg(feature = "with-lvgl")]
        let mut lvgl_display = Display::new(&lvgl, raw_display, unsafe { &mut DRAW_BUFFER });

        #[cfg(feature = "with-lvgl")]
        let mut main_ui = ui::new_screen(&lvgl_display, |screen| ui::MotionScreen::new(screen));
        #[cfg(feature = "with-lvgl")]
        lvgl_display.load_screen(&mut main_ui);
        //lvgl.run_tasks();

        Self {
            #[cfg(feature = "with-lvgl")]
            lvgl,
            #[cfg(feature = "with-lvgl")]
            lvgl_display,
            #[cfg(not(feature = "with-lvgl"))]
            raw_display,
            #[cfg(feature = "with-lvgl")]
            main_ui,
        }
    }

    #[cfg(feature = "with-lvgl")]
    pub fn ticks(&self) -> Ticks {
        self.lvgl.ticks()
    }

    #[inline]
    pub async fn update(&mut self) {
        #[cfg(feature = "with-lvgl")]
        {
            self.main_ui.context().as_mut().unwrap().refresh().await;
            self.lvgl.run_tasks();
        }
        #[cfg(not(feature = "with-lvgl"))]
        {
            self.main_ui.refresh(self.raw_display);
        }
    }
}

pub struct RawDisplay {
    device: RefCell<Ili9341<InterfaceType, hwi::peripheral::DISPLAY_PAR_RST_OUTPUT>>,
    cs: RefCell<hwi::peripheral::DISPLAY_PAR_CS_OUTPUT>,
}

impl RawDisplay {
    pub async fn new(device: DisplayDevice, event_bus: EventBusRef) -> Self {
        info!("Creating ILI dev");
        let mut delay = embassy_time::Delay;
        let mut cs = device.cs;
        cs.set_low();
        let mut lcd = Ili9341::new(
            device.interface,
            device.rst,
            &mut delay,
            Orientation::PortraitFlipped,
            DisplaySize240x320,
        )
        .unwrap();
        cs.set_high();

        Self {
            device: RefCell::new(lcd),
            cs: RefCell::new(cs),
        }
    }

    pub async fn init(&mut self) {
        info!("DISP: init START");
    }

    /*
    pub async fn init(&self) {

        info!("DISP: init START");

        let mut device = self.device.borrow_mut();

        device.rst.set_low();
        // RST signal must to be enabled for at least 10us
        Timer::after(Duration::from_micros(15)).await;
        device.rst.set_high();
        // Wait 5ms after reset before sending more commands
        Timer::after(Duration::from_millis(5)).await;

        device.cs.set_low();

        Self::command(&mut device, Command::SoftwareReset, &[]);
        // It will be necessary to wait 5msec before sending new command following software reset.
        // The display module loads all display supplier factory default values to the registers
        // during this 5msec. If Software Reset is applied during Sleep Out mode,
        // it will be necessary to wait 120msec before sending Sleep out command.
        // Software Reset Command cannot be sent during Sleep Out sequence.
        Timer::after(Duration::from_millis(120)).await;

        // Set orientation to Landscape
        Self::command(&mut device, Command::MemoryAccessControl, &[Mode::Portrait as u8]);

        // Set pixel format to 16 bits per pixel
        Self::command(&mut device, Command::PixelFormatSet, &[0b01010101u8]);

        Self::command(&mut device, Command::SleepModeOff, &[]);
        // Wait 5ms after Sleep Out before sending commands
        Timer::after(Duration::from_millis(5)).await;

        Self::command(&mut device, Command::DisplayOn, &[]);

        device.cs.set_high();
        info!("DISP: init DONE");
    }

    #[inline]
    fn command(device: &'_ mut RefMut<'_, DisplayDevice>, cmd: Command, args: &[u8])  {
        let mut x = once(cmd as u8);
        device.interface.send_commands(DataFormat::U8Iter(&mut x));
        device.interface.send_data(DataFormat::U8(&args));
    }

    #[inline]
    fn data16(device: &'_ mut RefMut<'_, DisplayDevice>, data: &[u16])  {
        device.interface.send_data(DataFormat::U16(&data));
    }

    #[inline]
    fn data16_it(device: &'_ mut RefMut<'_, DisplayDevice>, data: &'_ mut dyn Iterator<Item = u16>)  {
        device.interface.send_data(DataFormat::U16BEIter(data));
    }

    #[inline]
    fn data8(device: &'_ mut RefMut<'_, DisplayDevice>, data: &[u8])  {
        device.interface.send_data(DataFormat::U8(&data));
    }

    #[inline]
    fn set_window(device: &'_ mut RefMut<'_, DisplayDevice>, src: Point, dst: Point) {
        Self::command(
            device,
            Command::ColumnAddressSet,
            &[
                (src.x >> 8) as u8, (src.x & 0xff) as u8,
                (dst.x >> 8) as u8, (dst.x & 0xff) as u8,
            ],
        );
        Self::command(
            device,
            Command::PageAddressSet,
            &[
                (src.y >> 8) as u8, (src.y & 0xff) as u8,
                (dst.y >> 8) as u8, (dst.y & 0xff) as u8,
            ],
        );
    }
     */

    /// Draw a rectangle on the screen, represented by top-left corner (x0, y0)
    /// and bottom-right corner (x1, y1).
    ///
    /// The border is included.
    ///
    /// This method accepts an iterator of rgb565 pixel values.
    ///
    /// The iterator is useful to avoid wasting memory by holding a buffer for
    /// the whole screen when it is not necessary.
    #[inline]
    pub fn draw_raw_iter<I: IntoIterator<Item = u16>>(&self, src: Point, dst: Point, data: I) {
        info!(
            "draw_raw_iter called [{}, {}] [{}, {}]",
            src.x, src.y, dst.x, dst.y
        );
        let mut device = self.device.borrow_mut();
        self.cs.borrow_mut().set_low();
        device
            .draw_raw_iter(src.x as u16, src.y as u16, dst.x as u16, dst.y as u16, data)
            .unwrap();
        self.cs.borrow_mut().set_high();
    }
}

impl OriginDimensions for RawDisplay {
    #[inline]
    fn size(&self) -> Size {
        Size::new(WIDTH.into(), HEIGHT.into())
    }
}

impl DrawTarget for RawDisplay {
    type Color = PixelColor;
    type Error = core::convert::Infallible;

    #[inline]
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        info!("draw_iter called");

        let bbox = self.bounding_box();
        let mut device = self.device.borrow_mut();

        self.cs.borrow_mut().set_low();
        for Pixel(point, color) in pixels {
            if bbox.contains(point) {
                info!("draw_px");
                device
                    .draw_raw_iter(
                        point.x as u16,
                        point.y as u16,
                        point.x as u16,
                        point.y as u16,
                        once(RawU16::from(color).into_inner()),
                    )
                    .unwrap();
                /*
                Self::set_window(&mut device, point.clone(), point);
                Self::command(&mut device, Command::MemoryWrite, &[]);
                Self::data8(&mut device, RawU16::from(color).to_be_bytes().as_slice());

                 */
            }
        }
        self.cs.borrow_mut().set_high();
        Ok(())
    }

    #[inline]
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        info!("fill_contiguous called");
        let drawable_area = area.intersection(&self.bounding_box());
        match drawable_area.bottom_right() {
            None => {
                // Nothing to draw
            }
            Some(bottom_right) => {
                info!("fill_contiguous -> draw");
                self.draw_raw_iter(
                    drawable_area.top_left,
                    bottom_right,
                    area.points()
                        .zip(colors)
                        .filter(|(point, _)| drawable_area.contains(*point))
                        .map(|(_, color)| RawU16::from(color).into_inner()),
                )
            }
        }

        Ok(())
    }
}

#[allow(unused)]
#[derive(Clone, Copy)]
enum Command {
    SoftwareReset = 0x01,
    MemoryAccessControl = 0x36,
    PixelFormatSet = 0x3a,
    SleepModeOn = 0x10,
    SleepModeOff = 0x11,
    InvertOff = 0x20,
    InvertOn = 0x21,
    DisplayOff = 0x28,
    DisplayOn = 0x29,
    ColumnAddressSet = 0x2a,
    PageAddressSet = 0x2b,
    MemoryWrite = 0x2c,
    IdleModeOff = 0x38,
    IdleModeOn = 0x39,
    SetBrightness = 0x51,
    ContentAdaptiveBrightness = 0x55,
    NormalModeFrameRate = 0xb1,
    IdleModeFrameRate = 0xb2,
}

#[allow(unused)]
#[derive(Clone, Copy)]
enum Mode {
    Portrait = 0x40 | 0x08,
    Landscape = 0x20 | 0x08,
}
