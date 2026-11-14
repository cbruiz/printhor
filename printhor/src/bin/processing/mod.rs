//! GCode input parsing and processing
mod gcode_multiplexed_io;
mod gcode_parser;
mod gcode_processor;
#[cfg(feature = "with-eeprom")]
pub mod settings_store;

pub use gcode_parser::{GCodeLineParser, GCodeLineParserError};
pub use gcode_processor::GCodeProcessor;
#[cfg(feature = "with-fan-extra-1")]
pub use gcode_processor::FAN_EXTRA1_MANUAL_POWER;

pub use gcode_multiplexed_io::GCodeMultiplexedInputStream;
use printhor_hwa_common::math::{Real, TVector};
use printhor_hwa_common::{CoordSel, EventStatus};
use strum::{AsRefStr, VariantNames};

#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
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

#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
#[derive(PartialEq, Debug)]
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
#[cfg_attr(test, derive(PartialEq))]
pub struct S {
    pub s: Option<Real>,
}

impl S {
    pub const fn new() -> Self {
        Self { s: None }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(test, derive(PartialEq))]
pub struct N {
    pub n: Option<Real>,
}

impl N {
    pub const fn new() -> Self {
        Self { n: None }
    }
}

/// Declares a simple GCode argument holder made of optional [Real] words.
macro_rules! gcode_args {
    ($(#[$meta:meta])* $name:ident { $($field:ident),+ $(,)? }) => {
        $(#[$meta])*
        #[derive(Clone, Debug)]
        #[cfg_attr(test, derive(PartialEq))]
        pub struct $name {
            $(pub $field: Option<Real>,)+
        }

        impl $name {
            pub const fn new() -> Self {
                Self { $($field: None,)+ }
            }
        }
    };
}

gcode_args!(
    /// `S` and `P` words. Used by G4 (S=seconds, P=milliseconds), M106 (S=power, P=fan index)
    /// and M302 (S=min temp, P=allow cold extrusion)
    SP { s, p }
);

gcode_args!(
    /// `S` and `R` words. Used by M109/M190 (S=wait while heating, R=wait heating and cooling)
    SR { s, r }
);

gcode_args!(
    /// PID constants (M301 hot-end, M304 hot-bed)
    PID { p, i, d }
);

gcode_args!(
    /// M303 PID autotune: `E`=heater index (-1 = bed), `S`=target temperature, `C`=cycles
    ESC { e, s, c }
);

gcode_args!(
    /// M204 acceleration: `S`=generic, `P`=print, `T`=travel (all in world units/s²)
    SPT { s, p, t }
);

gcode_args!(
    /// M207/M208 firmware retraction: `S`=length, `F`=feed rate (world units/min), `Z`=z-hop
    SFZ { s, f, z }
);

/// Arc move (G2/G3) in the XY plane.
///
/// `I` and `J` are the center offsets relative to the start point. `R` is the radius (alternative form).
#[cfg(feature = "with-arcs")]
#[derive(Clone, Debug)]
#[cfg_attr(test, derive(PartialEq))]
pub struct ARC {
    pub coords: EFSXYZ,
    pub i: Option<Real>,
    pub j: Option<Real>,
    pub r: Option<Real>,
}

#[cfg(feature = "with-arcs")]
impl ARC {
    pub const fn new() -> Self {
        Self {
            coords: EFSXYZ::new(),
            i: None,
            j: None,
            r: None,
        }
    }
}

/// Cubic Bézier spline move (G5) in the XY plane.
///
/// `I`/`J`: offset of the first control point relative to the start point. `P`/`Q`: offset of the
/// second control point relative to the end point.
#[cfg(feature = "with-beziers")]
#[derive(Clone, Debug)]
#[cfg_attr(test, derive(PartialEq))]
pub struct BEZIER {
    pub coords: EFSXYZ,
    pub i: Option<Real>,
    pub j: Option<Real>,
    pub p: Option<Real>,
    pub q: Option<Real>,
}

#[cfg(feature = "with-beziers")]
impl BEZIER {
    pub const fn new() -> Self {
        Self {
            coords: EFSXYZ::new(),
            i: None,
            j: None,
            p: None,
            q: None,
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(test, derive(PartialEq))]
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
#[cfg_attr(test, derive(PartialEq))]
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
#[cfg_attr(test, derive(PartialEq))]
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
#[cfg_attr(test, derive(PartialEq))]
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
#[cfg_attr(test, derive(PartialEq))]
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

    /// Clockwise arc move
    #[cfg(feature = "with-arcs")]
    G2(ARC),

    /// Counter-clockwise arc move
    #[cfg(feature = "with-arcs")]
    G3(ARC),

    /// Cubic Bézier spline move
    #[cfg(feature = "with-beziers")]
    G5(BEZIER),

    /// Dwell (S=seconds, P=milliseconds)
    #[cfg(feature = "with-motion")]
    G4(SP),

    /// Firmware retract (see M207)
    #[cfg(feature = "with-motion")]
    G10,

    /// Firmware recover (see M208)
    #[cfg(feature = "with-motion")]
    G11,

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

    /// Set extruder to absolute mode
    #[cfg(feature = "with-motion")]
    M82,

    /// Set extruder to relative mode
    #[cfg(feature = "with-motion")]
    M83,

    /// Disable steppers
    #[cfg(feature = "with-motion")]
    M84,

    /// Set axis steps per world unit
    #[cfg(feature = "with-motion")]
    M92(EXYZ),

    /// Show memory usage
    M100,

    /// Set HotEnd Temperature
    #[cfg(feature = "with-hot-end")]
    M104(S),

    /// Get HotEnd and/or HotBed Temperature
    M105,

    /// Fan On (S=power [0-255])
    #[cfg(feature = "with-fan-layer")]
    M106(SP),

    /// Fan Off
    #[cfg(feature = "with-fan-layer")]
    M107,

    /// Wait for hotEnd temp
    #[cfg(feature = "with-hot-end")]
    M109(SR),

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

    /// Set servo/probe angle (P=servo index, only 0 supported, S=angle in degrees).
    /// With a BLTouch: S10 deploy, S90 stow, S120 self-test, S160 alarm release
    #[cfg(feature = "with-probe")]
    M280(SP),

    /// Deploy probe
    #[cfg(feature = "with-probe")]
    M401,

    /// Stow probe
    #[cfg(feature = "with-probe")]
    M402,

    M120,

    #[cfg(feature = "with-motion")]
    M121, // Endstops get/set

    /// Set hot-bed temperature
    #[cfg(feature = "with-hot-bed")]
    M140(S),

    /// Wait for hotbed temperature
    #[cfg(feature = "with-hot-bed")]
    M190(SR),

    M200,

    /// Set max acceleration per axis (world units/s²)
    #[cfg(feature = "with-motion")]
    M201(EXYZ),

    M202,

    /// Set max feed rate per axis (world units/s)
    #[cfg(feature = "with-motion")]
    M203(EXYZ),

    /// Set starting acceleration (world units/s²)
    #[cfg(feature = "with-motion")]
    M204(SPT),

    /// Set Advanced Settings (jerk). Accepted but ignored: Marlin semantics (instant speed change in
    /// world units/s) differ from the jerk (world units/s³) used by the S-curve motion profile
    M205,

    /// Set Home Offsets: shifts the coordinate system so that, after homing, the logical
    /// position is the homing point plus the offset (`logical = native + offset`, as Marlin)
    #[cfg(feature = "with-motion")]
    M206(EXYZ),

    /// Set firmware retraction (S=length, F=feed rate, Z=z-hop)
    #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
    M207(SFZ),

    /// Set firmware recover (S=extra length, F=feed rate)
    #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
    M208(SFZ),

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

    /// Set hot-end PID constants
    #[cfg(feature = "with-hot-end")]
    M301(PID),

    /// Cold extrusion (S=min temperature, P1=allow cold extrusion)
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    M302(SP),

    /// PID autotune
    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    M303(ESC),

    /// Set hot-bed PID constants
    #[cfg(feature = "with-hot-bed")]
    M304(PID),

    M305,

    M350,

    M360, // Settings

    /// Wait for moves and finish
    #[cfg(feature = "with-motion")]
    M400,

    M404,

    M407, // Settings

    /// Quick stop: discard the queued moves
    #[cfg(feature = "with-motion")]
    M410,

    #[cfg(feature = "with-motion")]
    M422, // Probe point

    M450,

    M451,

    M452,

    M453, // Modes

    /// Store the settings in the EEPROM
    #[cfg(feature = "with-eeprom")]
    M500,

    /// Restore the settings from the EEPROM
    #[cfg(feature = "with-eeprom")]
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

    /// Clear the alarm state after a fault (thermal or emergency stop)
    M999,
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for GCodeCmd {
    fn format(&self, fmt: defmt::Formatter) {
        let gcode_name: &str = self.value.as_ref();
        defmt::write!(fmt, "{}", gcode_name)
    }
}
