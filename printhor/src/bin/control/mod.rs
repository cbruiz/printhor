#[allow(unused)]
use crate::hwa;
use crate::math::Real;

#[cfg(feature = "native")]
use strum::Display;
use strum::{AsRefStr, VariantNames};
#[cfg(feature = "with-motion")]
pub mod motion;

mod processing;
pub use processing::*;

mod base;
pub mod task_control;
#[cfg(any(
    feature = "with-motion",
    feature = "with-hot-end",
    feature = "with-hot-bed"
))]
pub mod task_defer;
#[cfg(any(test, feature = "integration-test"))]
pub mod task_integration;
#[cfg(feature = "with-print-job")]
pub mod task_print_job;
#[cfg(feature = "with-motion")]
pub mod task_stepper;
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub mod task_temperature;

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct S {
    pub s: Option<Real>,
}

impl S {
    pub const fn new() -> Self {
        Self { s: None }
    }
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct N {
    pub n: Option<Real>,
}

impl N {
    pub const fn new() -> Self {
        Self { n: None }
    }
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct XYZE {
    pub x: Option<Real>,
    pub y: Option<Real>,
    pub z: Option<Real>,
    pub e: Option<Real>,
}

impl XYZE {
    pub const fn new() -> Self {
        Self {
            x: None,
            y: None,
            z: None,
            e: None,
        }
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for XYZE {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "X_Y_Z_E")
    }
}

#[derive(Clone)]
pub struct XYZF {
    pub x: Option<Real>,
    pub y: Option<Real>,
    pub z: Option<Real>,
    pub f: Option<Real>,
}

impl XYZF {
    pub const fn new() -> Self {
        Self {
            x: None,
            y: None,
            z: None,
            f: None,
        }
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for XYZF {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "X_Y_Z_F")
    }
}

impl core::fmt::Debug for XYZF {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(fmt, "X ")?;
        match &self.x {
            Some(v) => write!(fmt, "{:?}", v)?,
            None => write!(fmt, "NaN")?,
        }
        core::write!(fmt, "Y ")?;
        match &self.y {
            Some(v) => write!(fmt, "{:?}", v)?,
            None => write!(fmt, "NaN")?,
        }
        core::write!(fmt, "Z ")?;
        match &self.z {
            Some(v) => write!(fmt, "{:?}", v)?,
            None => write!(fmt, "NaN")?,
        }
        core::write!(fmt, "F ")?;
        match &self.f {
            Some(v) => write!(fmt, "{:?}", v)?,
            None => write!(fmt, "NaN")?,
        }
        Ok(())
    }
}

#[allow(dead_code)]
#[derive(Clone, Default, Debug)]
pub struct XYZEFS {
    pub x: Option<Real>,
    pub y: Option<Real>,
    pub z: Option<Real>,
    pub e: Option<Real>,
    pub f: Option<Real>,
    pub s: Option<Real>,
}

impl XYZEFS {
    pub const fn new() -> Self {
        Self {
            x: None,
            y: None,
            z: None,
            e: None,
            f: None,
            s: None,
        }
    }
    #[allow(unused)]
    pub fn with_x(&self, pos: i32) -> Self {
        Self {
            e: self.e,
            f: self.f,
            s: self.s,
            x: Some(Real::from_lit(pos as i64, 0)),
            y: self.y,
            z: self.z,
        }
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for XYZEFS {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "XYZEFS")
    }
}

#[derive(Debug)]
pub struct GCodeCmd {
    /// The gcode sequential number as coming from parser
    pub order_num: u32,
    /// The line number tagged in the gcode by NXX word (if any)
    pub line_tag: Option<u32>,
    /// The gcode variant.
    /// See [GCodeValue]
    pub value: GCodeValue,
}

impl GCodeCmd {
    pub const fn new(num: u32, line: Option<u32>, value: GCodeValue) -> Self {
        Self {
            order_num: num,
            line_tag: line,
            value,
        }
    }
}

impl core::fmt::Display for GCodeCmd {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "{{ gcode: {}, order_num: {}, line_tag: ",
            self.value.as_ref(),
            self.order_num
        )?;
        match &self.line_tag {
            None => {
                core::write!(f, "None }}")
            }
            Some(ln) => {
                core::write!(f, "{} }}", ln)
            }
        }
    }
}
/// The GCode Variants
//noinspection SpellCheckingInspection
#[derive(Clone, VariantNames, AsRefStr, Default, Debug)]
#[cfg_attr(feature = "native", derive(Display))]
pub enum GCodeValue {
    /// No Operation
    #[default]
    Nop,

    /// `GRBL` compat status
    #[cfg(feature = "grbl-compat")]
    Status,

    /// `GRBL` compat cmd
    #[cfg(feature = "grbl-compat")]
    GRBLCmd,

    /// List supported G-Codes
    G,

    /// Rapid move
    #[cfg(feature = "with-motion")]
    G0(XYZF),

    /// Linear move
    #[cfg(feature = "with-motion")]
    G1(XYZEFS),

    /// Dwell
    #[cfg(feature = "with-motion")]
    G4(S),

    /// Set coordinate system data
    #[cfg(feature = "with-motion")]
    G10,

    #[cfg(feature = "with-motion")]
    G11, // retraction

    #[cfg(feature = "with-motion")]
    G17,

    #[cfg(feature = "with-motion")]
    G18,

    #[cfg(feature = "with-motion")]
    G19, // CNC Plane selection

    #[cfg(feature = "with-motion")]
    G21, // Settings

    #[cfg(feature = "with-motion")]
    G22,

    #[cfg(feature = "with-motion")]
    G23, // Retraction

    #[cfg(feature = "with-motion")]
    /// Move to Origin (Home)
    G28(XYZE),

    /// Detailed Z-Probe
    #[cfg(feature = "with-motion")]
    G29,

    /// Set Z probe head offset
    #[cfg(feature = "with-motion")]
    #[strum(serialize = "G29.1")]
    G29_1,

    /// Set Z probe head offset calculated from tool head position
    #[cfg(feature = "with-motion")]
    #[strum(serialize = "G29.2")]
    G29_2,

    /// Single Z-Probe
    #[cfg(feature = "with-motion")]
    G30,

    /// Dock Sled
    #[cfg(feature = "with-probe")]
    G31,

    /// Undock Sled
    #[cfg(feature = "with-probe")]
    G32,

    #[strum(serialize = "G38.2")]
    G38_2,

    #[strum(serialize = "G38.3")]
    G38_3,

    #[strum(serialize = "G38.4")]
    G38_4,

    #[strum(serialize = "G38.5")]
    G38_5,

    G80,

    G81,

    #[cfg(feature = "with-probe")]
    G82, // Probing

    /// Set to Absolute Positioning
    #[cfg(feature = "with-motion")]
    G90,

    /// Set to Relative Positioning
    #[cfg(feature = "with-motion")]
    G91,

    /// Set position
    #[cfg(feature = "with-motion")]
    G92(XYZE),

    #[cfg(feature = "with-motion")]
    #[strum(serialize = "G92.1")]
    G92_1,

    #[cfg(feature = "with-motion")]
    #[strum(serialize = "G92.2")]
    G92_2, // Positioning

    #[cfg(feature = "with-motion")]
    G93,

    #[cfg(feature = "with-motion")]
    G94, // Feed rate

    /// List supported M-Codes
    M,

    M0,

    M1,

    M2, // Program control

    M3,

    M4,

    M5, // CNC/Laser

    M6,

    M7,

    M8,

    M9,

    M10,

    M11,

    M13,

    M16, // CNC

    M17,

    #[cfg(feature = "with-motion")]
    M18, // Stepper motors

    /// List SD
    #[cfg(feature = "with-sd-card")]
    M20(Option<alloc::string::String>),

    #[cfg(feature = "with-sd-card")]
    M21,

    #[cfg(feature = "with-sd-card")]
    M22,

    /// Select SD file
    #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
    M23(Option<alloc::string::String>),

    /// Start/resume SD print
    #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
    M24,

    /// Pause SD print
    #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
    M25,

    M26,

    M27,

    /// Program Stop
    M30,

    M31,

    M32,

    #[cfg(feature = "with-sd-card")]
    M33, // SD

    M37(S), // Simulation mode (Dry run mode)

    /// Set Print Progress
    M73,

    /// Soft reset
    M79,

    /// ATX Power ON
    #[cfg(feature = "with-ps-on")]
    M80,

    /// ATX Power OFF
    #[cfg(feature = "with-ps-on")]
    M81,

    /// Settings
    M83,

    /// Disable steppers
    #[cfg(feature = "with-motion")]
    M84,

    M92,

    /// Show memory usage
    M100,

    /// Set HotEnd Temperature
    #[cfg(feature = "with-hot-end")]
    M104(S),

    /// Get HotEnd and/or HotBed Temperature
    M105,

    /// Fan On
    #[cfg(feature = "with-fan-layer")]
    M106,

    /// Fan Off
    #[cfg(feature = "with-fan-layer")]
    M107,

    /// Wait for hotEnd temp
    #[cfg(feature = "with-hot-end")]
    M109(S),

    M110(N), // Settings

    /// Debug level
    M111,

    /// Full emergency stop
    M112,

    /// Get current position
    #[cfg(feature = "with-motion")]
    M114,

    /// Get Firmware Version and Capabilities
    M115,

    /// Wait
    M116,

    /// Display message
    M117,

    /// Echo message on host
    M118,

    /// Get Endstop Status
    M119,

    M120,

    #[cfg(feature = "with-motion")]
    M121, // Endstops get/set

    /// Set hot-bed temperature
    #[cfg(feature = "with-hot-bed")]
    M140(S),

    /// Wait for hotbed temperature
    #[cfg(feature = "with-hot-bed")]
    M190,

    M200,

    /// Print / Travel Move Limits
    #[cfg(feature = "with-motion")]
    M201,

    M202,

    /// Set Max Feedrate
    #[cfg(feature = "with-motion")]
    M203,

    M204,

    /// Set Advanced Settings
    M205,

    /// Set Home Offsets
    #[cfg(feature = "with-motion")]
    M206,

    M207,

    M208,

    M209,

    M210,

    M211,

    M212,

    M218, // Settings

    /// Set Feedrate percentage
    M220(S),

    /// Set Flow Percentage
    M221(S),

    #[cfg(feature = "with-motion")]
    M290, // Babystepping

    M302,

    M305,

    M350,

    M360, // Settings

    /// Wait for moves and finish
    #[cfg(feature = "with-motion")]
    M400,

    M401,

    #[cfg(feature = "with-probe")]
    M402, // Probing

    M404,

    M407, // Settings

    M410, // quick stop

    #[cfg(feature = "with-motion")]
    M422, // Probe point

    M450,

    M451,

    M452,

    M453, // Modes

    M500,

    M501,

    /// Restore Default Settings
    M502,

    M504,

    M505, // EEProm/State

    M510,

    M511,

    M512,

    M513, // Password and locking

    /// Abort SD printing
    #[cfg(feature = "with-sd-card")]
    M524,

    M555,

    M563,

    M851,

    /// Report the status of position encoder modules.
    #[cfg(feature = "with-motion")]
    #[strum(serialize = "M862.1")]
    M862_1,

    /// Perform an axis continuity test for position encoder modules.
    #[cfg(feature = "with-motion")]
    #[strum(serialize = "M862.2")]
    M862_2,

    /// Perform steps-per-mm calibration for position encoder modules.
    #[cfg(feature = "with-motion")]
    #[strum(serialize = "M862.3")]
    M862_3,

    /// Set Lineal Advance Factor
    #[cfg(feature = "with-motion")]
    M900,

    /// Set motor current
    #[cfg(feature = "with-motion")]
    M907,

    M929, // Logging
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for GCodeCmd {
    fn format(&self, fmt: defmt::Formatter) {
        let gcode_name: &str = self.value.as_ref();
        defmt::write!(fmt, "{}", gcode_name)
    }
}

pub use base::*;
