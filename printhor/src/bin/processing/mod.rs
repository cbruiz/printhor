//! GCode input parsing and processing
mod gcode_multiplexed_io;
mod gcode_parser;
mod gcode_processor;

pub use gcode_parser::{GCodeLineParser, GCodeLineParserError};
pub use gcode_processor::GCodeProcessor;

pub use gcode_multiplexed_io::GCodeMultiplexedInputStream;
use printhor_hwa_common::math::{Real, TVector};
use printhor_hwa_common::{CoordSel, EventStatus};
use strum::{AsRefStr, VariantNames};

#[cfg_attr(all(feature = "with-defmt", feature = "native"), derive(defmt::Format))]
#[cfg_attr(feature = "native", derive(Debug))]
#[allow(unused)]
pub enum CodeExecutionSuccess {
    /// Immediately executed
    OK,
    /// Immediately executed and reported
    CONSUMED,
    /// Queued but assumed it will be executed not too long, so practically same as OK
    QUEUED,
    /// Executed but it will take time to get a final response. EventStatus contains the needed flags to wait for
    DEFERRED(EventStatus),
}

#[cfg_attr(all(feature = "with-defmt", feature = "native"), derive(defmt::Format))]
#[derive(Debug)]
#[allow(unused)]
pub enum CodeExecutionFailure {
    /// Cannot perform because there is the same or something else running
    BUSY,
    /// Generic internal error
    ERR,
    /// Cannot perform because requires homing before
    HomingRequired,
    /// Cannot perform because requires homing before
    PowerRequired,
    /// Specific internal error: Numerical computation issue (division by 0, sqrt(x<0) or any other kind of ambiguity)
    NumericalError,
    /// The GCode is considered, but not yet implemented
    NotYetImplemented,
}

pub type CodeExecutionResult = Result<CodeExecutionSuccess, CodeExecutionFailure>;

#[derive(Clone, Debug)]
pub struct S {
    pub s: Option<Real>,
}

impl S {
    pub const fn new() -> Self {
        Self { s: None }
    }
}

#[derive(Clone, Debug)]
pub struct N {
    pub n: Option<Real>,
}

impl N {
    pub const fn new() -> Self {
        Self { n: None }
    }
}

#[derive(Clone, Debug)]
pub struct EXYZ {
    #[cfg(feature = "with-e-axis")]
    pub e: Option<Real>,
    //
    #[cfg(feature = "with-x-axis")]
    pub x: Option<Real>,
    #[cfg(feature = "with-y-axis")]
    pub y: Option<Real>,
    #[cfg(feature = "with-z-axis")]
    pub z: Option<Real>,
    //
    #[cfg(feature = "with-a-axis")]
    pub a: Option<Real>,
    #[cfg(feature = "with-b-axis")]
    pub b: Option<Real>,
    #[cfg(feature = "with-c-axis")]
    pub c: Option<Real>,
    //
    #[cfg(feature = "with-i-axis")]
    pub i: Option<Real>,
    #[cfg(feature = "with-j-axis")]
    pub j: Option<Real>,
    #[cfg(feature = "with-k-axis")]
    pub k: Option<Real>,
    //
    #[cfg(feature = "with-u-axis")]
    pub u: Option<Real>,
    #[cfg(feature = "with-v-axis")]
    pub v: Option<Real>,
    #[cfg(feature = "with-w-axis")]
    pub w: Option<Real>,
}

impl EXYZ {
    pub const fn new() -> Self {
        Self {
            #[cfg(feature = "with-e-axis")]
            e: None,
            //
            #[cfg(feature = "with-x-axis")]
            x: None,
            #[cfg(feature = "with-y-axis")]
            y: None,
            #[cfg(feature = "with-z-axis")]
            z: None,
            //
            #[cfg(feature = "with-a-axis")]
            a: None,
            #[cfg(feature = "with-b-axis")]
            b: None,
            #[cfg(feature = "with-c-axis")]
            c: None,
            //
            #[cfg(feature = "with-i-axis")]
            i: None,
            #[cfg(feature = "with-j-axis")]
            j: None,
            #[cfg(feature = "with-k-axis")]
            k: None,
            //
            #[cfg(feature = "with-u-axis")]
            u: None,
            #[cfg(feature = "with-v-axis")]
            v: None,
            #[cfg(feature = "with-w-axis")]
            w: None,
        }
    }
}

impl Into<TVector<Real>> for &EXYZ {
    fn into(self) -> TVector<Real> {
        self.clone().into()
    }
}

impl Into<TVector<Real>> for EXYZ {
    fn into(self) -> TVector<Real> {
        TVector::from_coords(
            #[cfg(feature = "with-e-axis")]
            self.e,
            #[cfg(feature = "with-x-axis")]
            self.x,
            #[cfg(feature = "with-y-axis")]
            self.y,
            #[cfg(feature = "with-z-axis")]
            self.z,
            //
            #[cfg(feature = "with-a-axis")]
            self.a,
            #[cfg(feature = "with-b-axis")]
            self.b,
            #[cfg(feature = "with-c-axis")]
            self.c,
            //
            #[cfg(feature = "with-i-axis")]
            self.i,
            #[cfg(feature = "with-j-axis")]
            self.j,
            #[cfg(feature = "with-k-axis")]
            self.k,
            //
            #[cfg(feature = "with-u-axis")]
            self.u,
            #[cfg(feature = "with-v-axis")]
            self.v,
            #[cfg(feature = "with-w-axis")]
            self.w,
        )
    }
}

impl From<TVector<Real>> for EXYZ {
    fn from(_v: TVector<Real>) -> Self {
        Self {
            #[cfg(feature = "with-e-axis")]
            e: _v.get_coord(CoordSel::E),
            #[cfg(feature = "with-x-axis")]
            x: _v.get_coord(CoordSel::X),
            #[cfg(feature = "with-y-axis")]
            y: _v.get_coord(CoordSel::Y),
            #[cfg(feature = "with-z-axis")]
            z: _v.get_coord(CoordSel::Z),
            //
            #[cfg(feature = "with-a-axis")]
            a: _v.get_coord(CoordSel::A),
            #[cfg(feature = "with-b-axis")]
            b: _v.get_coord(CoordSel::B),
            #[cfg(feature = "with-c-axis")]
            c: _v.get_coord(CoordSel::C),
            //
            #[cfg(feature = "with-i-axis")]
            i: _v.get_coord(CoordSel::I),
            #[cfg(feature = "with-j-axis")]
            j: _v.get_coord(CoordSel::J),
            #[cfg(feature = "with-k-axis")]
            k: _v.get_coord(CoordSel::K),
            //
            #[cfg(feature = "with-u-axis")]
            u: _v.get_coord(CoordSel::U),
            #[cfg(feature = "with-v-axis")]
            v: _v.get_coord(CoordSel::V),
            #[cfg(feature = "with-w-axis")]
            w: _v.get_coord(CoordSel::W),
        }
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for EXYZ {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "X_Y_Z_E")
    }
}

#[derive(Clone, Debug)]
pub struct FXYZ {
    pub f: Option<Real>,
    //
    #[cfg(feature = "with-x-axis")]
    pub x: Option<Real>,
    #[cfg(feature = "with-y-axis")]
    pub y: Option<Real>,
    #[cfg(feature = "with-z-axis")]
    pub z: Option<Real>,
    #[cfg(feature = "with-a-axis")]
    //
    pub a: Option<Real>,
    #[cfg(feature = "with-b-axis")]
    pub b: Option<Real>,
    #[cfg(feature = "with-c-axis")]
    pub c: Option<Real>,
    //
    #[cfg(feature = "with-i-axis")]
    pub i: Option<Real>,
    #[cfg(feature = "with-j-axis")]
    pub j: Option<Real>,
    #[cfg(feature = "with-k-axis")]
    pub k: Option<Real>,
    //
    #[cfg(feature = "with-u-axis")]
    pub u: Option<Real>,
    #[cfg(feature = "with-v-axis")]
    pub v: Option<Real>,
    #[cfg(feature = "with-w-axis")]
    pub w: Option<Real>,
}

impl FXYZ {
    pub const fn new() -> Self {
        Self {
            f: None,
            //
            #[cfg(feature = "with-x-axis")]
            x: None,
            #[cfg(feature = "with-y-axis")]
            y: None,
            #[cfg(feature = "with-z-axis")]
            z: None,
            //
            #[cfg(feature = "with-a-axis")]
            a: None,
            #[cfg(feature = "with-b-axis")]
            b: None,
            #[cfg(feature = "with-c-axis")]
            c: None,
            //
            #[cfg(feature = "with-i-axis")]
            i: None,
            #[cfg(feature = "with-j-axis")]
            j: None,
            #[cfg(feature = "with-k-axis")]
            k: None,
            //
            #[cfg(feature = "with-u-axis")]
            u: None,
            #[cfg(feature = "with-v-axis")]
            v: None,
            #[cfg(feature = "with-w-axis")]
            w: None,
        }
    }
}

impl Into<TVector<Real>> for &FXYZ {
    fn into(self) -> TVector<Real> {
        self.clone().into()
    }
}

impl Into<TVector<Real>> for FXYZ {
    fn into(self) -> TVector<Real> {
        TVector::from_coords(
            #[cfg(feature = "with-e-axis")]
            None,
            //
            #[cfg(feature = "with-x-axis")]
            self.x,
            #[cfg(feature = "with-y-axis")]
            self.y,
            #[cfg(feature = "with-z-axis")]
            self.z,
            //
            #[cfg(feature = "with-a-axis")]
            self.a,
            #[cfg(feature = "with-b-axis")]
            self.b,
            #[cfg(feature = "with-c-axis")]
            self.c,
            //
            #[cfg(feature = "with-i-axis")]
            self.i,
            #[cfg(feature = "with-j-axis")]
            self.j,
            #[cfg(feature = "with-k-axis")]
            self.k,
            //
            #[cfg(feature = "with-u-axis")]
            self.u,
            #[cfg(feature = "with-v-axis")]
            self.v,
            #[cfg(feature = "with-w-axis")]
            self.w,
        )
    }
}

impl From<TVector<Real>> for FXYZ {
    fn from(_v: TVector<Real>) -> Self {
        Self {
            f: None,
            #[cfg(feature = "with-x-axis")]
            x: _v.get_coord(CoordSel::X),
            #[cfg(feature = "with-y-axis")]
            y: _v.get_coord(CoordSel::Y),
            #[cfg(feature = "with-z-axis")]
            z: _v.get_coord(CoordSel::Z),
            //
            #[cfg(feature = "with-a-axis")]
            a: _v.get_coord(CoordSel::A),
            #[cfg(feature = "with-b-axis")]
            b: _v.get_coord(CoordSel::B),
            #[cfg(feature = "with-c-axis")]
            c: _v.get_coord(CoordSel::C),
            //
            #[cfg(feature = "with-i-axis")]
            i: _v.get_coord(CoordSel::I),
            #[cfg(feature = "with-j-axis")]
            j: _v.get_coord(CoordSel::J),
            #[cfg(feature = "with-k-axis")]
            k: _v.get_coord(CoordSel::K),
            //
            #[cfg(feature = "with-u-axis")]
            u: _v.get_coord(CoordSel::U),
            #[cfg(feature = "with-v-axis")]
            v: _v.get_coord(CoordSel::V),
            #[cfg(feature = "with-w-axis")]
            w: _v.get_coord(CoordSel::W),
        }
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for FXYZ {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "X_Y_Z_F")
    }
}

#[derive(Clone, Default, Debug)]
pub struct EFSXYZ {
    pub f: Option<Real>,
    pub s: Option<Real>,
    #[cfg(feature = "with-e-axis")]
    pub e: Option<Real>,
    //
    #[cfg(feature = "with-x-axis")]
    pub x: Option<Real>,
    #[cfg(feature = "with-y-axis")]
    pub y: Option<Real>,
    #[cfg(feature = "with-z-axis")]
    pub z: Option<Real>,
    //
    #[cfg(feature = "with-a-axis")]
    pub a: Option<Real>,
    #[cfg(feature = "with-b-axis")]
    pub b: Option<Real>,
    #[cfg(feature = "with-c-axis")]
    pub c: Option<Real>,
    //
    #[cfg(feature = "with-i-axis")]
    pub i: Option<Real>,
    #[cfg(feature = "with-j-axis")]
    pub j: Option<Real>,
    #[cfg(feature = "with-k-axis")]
    pub k: Option<Real>,
    //
    #[cfg(feature = "with-u-axis")]
    pub u: Option<Real>,
    #[cfg(feature = "with-v-axis")]
    pub v: Option<Real>,
    #[cfg(feature = "with-w-axis")]
    pub w: Option<Real>,
}

impl EFSXYZ {
    pub const fn new() -> Self {
        Self {
            #[cfg(feature = "with-e-axis")]
            e: None,
            f: None,
            s: None,
            //
            #[cfg(feature = "with-x-axis")]
            x: None,
            #[cfg(feature = "with-y-axis")]
            y: None,
            #[cfg(feature = "with-z-axis")]
            z: None,
            //
            #[cfg(feature = "with-a-axis")]
            a: None,
            #[cfg(feature = "with-b-axis")]
            b: None,
            #[cfg(feature = "with-c-axis")]
            c: None,
            //
            #[cfg(feature = "with-i-axis")]
            i: None,
            #[cfg(feature = "with-j-axis")]
            j: None,
            #[cfg(feature = "with-k-axis")]
            k: None,
            //
            #[cfg(feature = "with-u-axis")]
            u: None,
            #[cfg(feature = "with-v-axis")]
            v: None,
            #[cfg(feature = "with-w-axis")]
            w: None,
        }
    }

    pub fn as_vector(&self) -> TVector<Real> {
        TVector::from_coords(
            #[cfg(feature = "with-e-axis")]
            self.e,
            //
            #[cfg(feature = "with-x-axis")]
            self.x,
            #[cfg(feature = "with-y-axis")]
            self.y,
            #[cfg(feature = "with-z-axis")]
            self.z,
            //
            #[cfg(feature = "with-a-axis")]
            self.a,
            #[cfg(feature = "with-b-axis")]
            self.b,
            #[cfg(feature = "with-c-axis")]
            self.c,
            //
            #[cfg(feature = "with-i-axis")]
            self.i,
            #[cfg(feature = "with-j-axis")]
            self.j,
            #[cfg(feature = "with-k-axis")]
            self.k,
            //
            #[cfg(feature = "with-u-axis")]
            self.u,
            #[cfg(feature = "with-v-axis")]
            self.v,
            #[cfg(feature = "with-w-axis")]
            self.w,
        )
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for EFSXYZ {
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
    G0(FXYZ),

    /// Linear move
    #[cfg(feature = "with-motion")]
    G1(EFSXYZ),

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
    G28(EXYZ),

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
    G92(EXYZ),

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

    /// Program End
    #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
    M2,

    /// Spindle On, Clockwise
    ///
    /// In Laser Mode: Laser on
    M3,

    /// Spindle On, Counter-Clockwise
    ///
    /// In Laser Mode: Laser on
    M4,

    /// In CNC Mode: Spindle Off
    ///
    /// In Laser Mode: Laser off
    M5,

    /// Tool change
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
    M117(Option<alloc::string::String>),

    /// Echo message on host
    M118(Option<alloc::string::String>),

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

    /// Report Current Settings
    M503(S),

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
