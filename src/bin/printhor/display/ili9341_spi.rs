#[cfg(feature = "with-lvgl")]
use crate::display::ui;
use crate::display::ui::eg_ui::EmbeddedGraphicsUI;
#[cfg(feature = "with-lvgl")]
use crate::display::ui::MotionScreen;
/// Based on https://github.com/yuri91/ili9341-rs
use crate::hwa;
use crate::hwa::mem::TrackedStaticCell;
use crate::hwi;
use crate::hwi::{ControllerMutex, ControllerRef};
use core::iter::once;
use embassy_time::{Duration, Timer};
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::geometry::Size;
use embedded_graphics_core::geometry::{Dimensions, OriginDimensions, Point};
use embedded_graphics_core::pixelcolor::raw::{RawU16, ToBytes};
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::RawData;
use embedded_graphics_core::primitives::{PointsIter, Rectangle};
use embedded_graphics_core::Pixel;
#[cfg(feature = "with-lvgl")]
use lvgl::core::ObjExt;
#[cfg(feature = "with-lvgl")]
use lvgl::core::Ticks;
#[cfg(feature = "with-lvgl")]
use lvgl::core::{Display, Lvgl, Screen};
#[cfg(feature = "with-lvgl")]
use std::mem::MaybeUninit;

const WIDTH: u32 = 240;
const HEIGHT: u32 = 320;
#[cfg(feature = "with-lvgl")]
const LVGL_BUFFER_LEN: usize = (240 * 320) / 10;

pub type PixelColor = Rgb565;

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
    main_ui: EmbeddedGraphicsUI,
}

impl TFTDisplay {
    pub async fn new(
        display_dev: hwa::display::DisplayDevice,
        _event_bus: hwa::EventBusRef,
    ) -> Self {
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
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
        #[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static mut DRAW_BUFFER: [MaybeUninit<PixelColor>; LVGL_BUFFER_LEN] =
            [MaybeUninit::<PixelColor>::uninit(); LVGL_BUFFER_LEN];

        let mut raw_display = RawDisplay::new(display_dev).await;
        raw_display.init().await;
        let _ = raw_display.clear(PixelColor::new(0, 0, 0));

        #[cfg(feature = "with-lvgl")]
        let mut lvgl_display = Display::new(&lvgl, raw_display, unsafe { &mut DRAW_BUFFER });

        #[cfg(feature = "with-lvgl")]
        let mut main_ui = ui::new_screen(&lvgl_display, |screen| ui::MotionScreen::new(screen));
        #[cfg(feature = "with-lvgl")]
        lvgl_display.load_screen(&mut main_ui);

        let main_ui = EmbeddedGraphicsUI::new(_event_bus).await;
        //lvgl.run_tasks();

        Self {
            #[cfg(feature = "with-lvgl")]
            lvgl,
            #[cfg(feature = "with-lvgl")]
            lvgl_display,
            #[cfg(not(feature = "with-lvgl"))]
            raw_display,
            main_ui,
        }
    }

    #[cfg(feature = "with-lvgl")]
    pub fn ticks(&self) -> Ticks {
        self.lvgl.ticks()
    }

    pub async fn update(&mut self) {
        let _t0 = embassy_time::Instant::now();
        #[cfg(not(feature = "with-lvgl"))]
        {
            self.main_ui.refresh(&mut self.raw_display).await;
        }
        #[cfg(feature = "with-lvgl")]
        {
            self.raw_display.retain().await;
            self.main_ui.context().as_mut().unwrap().refresh().await;
            self.lvgl.run_tasks();
            self.raw_display.release().await;
        }
        //crate::info!("Updated in {} ms", _t0.elapsed().as_millis());
    }
}

pub struct Pins {
    pub rst: hwi::device::DISPLAY_SER_RST_OUTPUT,
    pub cs: hwi::device::DISPLAY_SER_CS_OUTPUT,
    pub dc: hwi::device::DISPLAY_SER_DC_OUTPUT,
}

pub struct RawDisplay {
    dev: hwi::SpiControllerRef,
    pins: ControllerRef<Pins>,
}

impl RawDisplay {
    pub async fn new(device: hwa::display::DisplayDevice) -> Self {
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
        #[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static DISPLAY_PINS: TrackedStaticCell<ControllerMutex<Pins>> = TrackedStaticCell::new();
        let pins = ControllerRef::new(DISPLAY_PINS.init(
            "DisplayController::Pins",
            ControllerMutex::new(Pins {
                rst: device.rst,
                cs: device.cs,
                dc: device.dc,
            }),
        ));
        Self {
            dev: device.interface,
            pins,
        }
    }

    pub async fn retain(&self) {
        //info!("spi retaining [D]");
        self.dev.retain().await;
        self.pins.retain().await;
        //info!("spi retained [D]");
    }

    pub async fn release(&self) {
        //info!("spi releasing [D]");
        self.pins.release().await;
        self.dev.release().await;
        //info!("spi released [D]");
    }

    pub async fn init(&self) {
        self.retain().await;

        // RST signal must be enabled for at least 10us
        Timer::after(Duration::from_micros(15)).await;
        let _ = self.pins.apply(|pins| {
            let _ = pins.rst.set_high();
            Ok(())
        });

        // Wait 5ms after reset before sending more commands
        Timer::after(Duration::from_millis(5)).await;
        //pins.cs.set_low();
        self.command(Command::SoftwareReset, &[]);
        // It will be necessary to wait 5msec before sending new command following software reset.
        // The display module loads all display supplier factory default values to the registers
        // during this 5msec. If Software Reset is applied during Sleep Out mode,
        // it will be necessary to wait 120msec before sending Sleep out command.
        // Software Reset Command cannot be sent during Sleep Out sequence.
        Timer::after(Duration::from_millis(120)).await;

        // Set orientation to Landscape
        self.command(Command::MemoryAccessControl, &[Mode::Portrait as u8]);

        // Set pixel format to 16 bits per pixel
        self.command(Command::PixelFormatSet, &[0b01010101u8]);

        self.command(Command::SleepModeOff, &[]);
        // Wait 5ms after Sleep Out before sending commands
        Timer::after(Duration::from_millis(5)).await;

        self.command(Command::DisplayOn, &[]);

        let _ = self.pins.apply(|pins| {
            pins.cs.set_high();
            Ok(())
        });
        self.release().await;
    }

    fn send_u8<'a, I>(&self, iter: I)
    where
        I: Iterator<Item = u8>,
    {
        let mut buf = [0; 32];
        let mut i = 0;

        let it = iter.into_iter();

        let _ = self.dev.apply(|spi| {
            for v in it {
                buf[i] = v;
                i += 1;

                if i == buf.len() {
                    let _ = spi.blocking_transfer_in_place(&mut buf);
                    i = 0;
                }
            }
            if i > 0 {
                let _ = spi.blocking_transfer_in_place(&mut buf[..i]);
            }
            Ok(())
        });
    }

    fn send_u16<'a, I>(&self, iter: I)
    where
        I: Iterator<Item = u16>,
    {
        let mut buf = [0; 32];
        let mut i = 0;
        let blen = buf.len();

        let _ = self.dev.apply(|spi| {
            for v in iter {
                buf[i] = v;
                i += 1;

                if i == blen {
                    let _ = spi.blocking_transfer_in_place(&mut buf);
                    i = 0;
                }
            }
            if i > 0 {
                let _ = spi.blocking_transfer_in_place(&mut buf[..i]);
            }
            Ok(())
        });
    }

    #[inline]
    fn command<'a>(&self, cmd: Command, args: &[u8]) {
        let _ = self.pins.apply(|pins| {
            pins.dc.set_low();
            Ok(())
        });
        let x = once(cmd as u8);
        self.send_u8(x);
        let _ = self.pins.apply(|pins| {
            pins.dc.set_high();
            Ok(())
        });
        let params = args.into_iter().cloned();
        self.send_u8(params);
        let _ = self.pins.apply(|pins| {
            pins.dc.set_low();
            Ok(())
        });
    }

    #[inline]
    fn data_u16<'a, I>(&self, data: I)
    where
        I: Iterator<Item = u16>,
    {
        let _ = self.pins.apply(|pins| {
            pins.dc.set_high();
            Ok(())
        });
        self.send_u16(data);
        let _ = self.pins.apply(|pins| {
            pins.dc.set_low();
            Ok(())
        });
    }

    #[inline]
    fn set_window(&self, src: Point, dst: Point) {
        self.command(
            Command::ColumnAddressSet,
            &[
                (src.x >> 8) as u8,
                (src.x & 0xff) as u8,
                (dst.x >> 8) as u8,
                (dst.x & 0xff) as u8,
            ],
        );
        self.command(
            Command::PageAddressSet,
            &[
                (src.y >> 8) as u8,
                (src.y & 0xff) as u8,
                (dst.y >> 8) as u8,
                (dst.y & 0xff) as u8,
            ],
        );
    }

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
        let _ = self.pins.apply(|pins| {
            pins.cs.set_low();
            Ok(())
        });
        self.set_window(src, dst);
        self.command(Command::MemoryWrite, &[]);
        self.data_u16(&mut data.into_iter());
        let _ = self.pins.apply(|pins| {
            pins.cs.set_high();
            Ok(())
        });
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
        let bbox = self.bounding_box();
        let _ = self.pins.apply(|pins| {
            pins.cs.set_low();
            Ok(())
        });

        for Pixel(point, color) in pixels {
            if bbox.contains(point) {
                self.set_window(point.clone(), point);
                self.command(Command::MemoryWrite, &[]);
                let _ = self.pins.apply(|pins| {
                    pins.dc.set_high();
                    Ok(())
                });
                let _ = self.dev.apply(|spi| {
                    let _ = spi.blocking_write(RawU16::from(color).to_be_bytes().as_slice());
                    Ok(())
                });

                let _ = self.pins.apply(|pins| {
                    pins.dc.set_low();
                    Ok(())
                });
            }
        }
        let _ = self.pins.apply(|pins| {
            pins.cs.set_high();
            Ok(())
        });
        Ok(())
    }

    #[inline]
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        #[cfg(feature = "native")]
        println!("fill_contiguous");
        let drawable_area = area.intersection(&self.bounding_box());
        match drawable_area.bottom_right() {
            None => {
                // Nothing to draw
            }
            Some(bottom_right) => self.draw_raw_iter(
                drawable_area.top_left,
                bottom_right,
                area.points()
                    .zip(colors)
                    .filter(|(point, _)| drawable_area.contains(*point))
                    .map(|(_, color)| RawU16::from(color).into_inner()),
            ),
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
