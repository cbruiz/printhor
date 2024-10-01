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
#[cfg(feature = "with-motion")]
pub mod task_defer;
#[cfg(any(test, feature = "integration-test"))]
pub mod task_integration;
#[cfg(feature = "with-printjob")]
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

#[derive(Clone, Debug)]
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

#[cfg(feature = "native")]
impl core::fmt::Display for XYZF {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "X {} Y {} Z {} F {}",
            self.x.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
            self.y.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
            self.z.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
            self.f.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
        )
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
    G0(XYZF),

    /// Linear move
    G1(XYZEFS),

    /// Dwell
    G4(S),
    /// Set coordinate system data
    G10,
    G11, // retraction
    G17,
    G18,
    G19, // CNC Plane selection
    G21, // Settings
    G22,
    G23, // Retraction

    /// Move to Origin (Home)
    G28(XYZE),
    /// Detailed Z-Probe
    G29,
    /// Set Z probe head offset
    #[strum(serialize = "G29.1")]
    G29_1,
    /// Set Z probe head offset calculated from tool head position
    #[strum(serialize = "G29.2")]
    G29_2,
    /// Single Z-Probe
    G30,
    /// Dock Sled
    G31,
    /// Undock Sled
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
    G82, // Probing
    /// Set to Absolute Positioning
    G90,
    /// Set to Relative Positioning
    G91,
    /// Set position
    G92(XYZE),
    #[strum(serialize = "G92.1")]
    G92_1,
    #[strum(serialize = "G92.2")]
    G92_2, // Positioning
    G93,
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
    M18, // Stepper motors
    /// List SD
    M20(Option<alloc::string::String>),
    M21,
    M22,
    /// Select SD file
    M23(Option<alloc::string::String>),
    /// Start/resume SD print
    M24,
    /// Pause SD print
    M25,
    M26,
    M27,
    /// Program Stop
    M30,
    M31,
    M32,
    M33,    // SD
    M37(S), // Simulation mode (Dry run mode)
    /// Set Print Progress
    M73,
    /// Soft reset
    M79,
    /// ATX Power ON
    M80,
    /// ATX Power OFF
    M81,
    /// Settings
    M83,
    /// Disable steppers
    M84,
    M92,
    /// Show memory usage
    M100,
    /// Set Hotend Temperature
    M104(S),
    /// Get Hotend and/or Hotbed Temperature
    M105,
    /// Fan On
    M106,
    /// Fan Off
    M107,
    /// Wait for hotend temp
    M109(S),
    M110(N), // Settings
    /// Debug level
    M111,
    /// Full emergency stop
    M112,
    /// Get current position
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
    M121, // Endstops get/set
    /// Set hotbed temperature
    M140(S),
    /// Wait for hotbed temperature
    M190,
    M200,
    /// Print / Travel Move Limits
    M201,
    M202,
    /// Set Max Feedrate
    M203,
    M204,
    /// Set Advanced Settings
    M205,
    /// Set Home Offsets
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
    M290, // Babystepping
    M302,
    M305,
    M350,
    M360, // Settings
    /// Wait for moves and finish
    M400,
    M401,
    M402, // Probing
    M404,
    M407, // Settings
    M410, // quick stop
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
    M524,
    M555,
    M563,
    M851,
    /// Report the status of position encoder modules.
    #[strum(serialize = "M862.1")]
    M862_1,
    /// Perform an axis continuity test for position encoder modules.
    #[strum(serialize = "M862.2")]
    M862_2,
    /// Perform steps-per-mm calibration for position encoder modules.
    #[strum(serialize = "M862.3")]
    M862_3,
    /// Set Lineal Advance Factor
    M900,
    /// Set motor current
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
