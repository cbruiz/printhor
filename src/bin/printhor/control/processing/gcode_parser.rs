use crate::control::{GCodeCmd, GCodeValue, N, S, XYZF, XYZE, XYZEFS};
use crate::helpers;
use crate::hwa;

#[cfg_attr(not(feature = "with-defmt"), derive(Debug))]
pub enum GCodeLineParserError {
    /// There was an error parsing
    ParseError(u32),
    /// The Gcode was not implemented
    GCodeNotImplemented(u32, alloc::string::String),
    /// EOF Reading from parser
    EOF,
    /// Unexpected fatal error
    #[allow(unused)]
    FatalError,
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for GCodeLineParserError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            GCodeLineParserError::ParseError(ln) => {
                defmt::write!(fmt, "ParseError(line={})", ln);
            }
            GCodeLineParserError::GCodeNotImplemented(ln, string) => {
                defmt::write!(
                    fmt,
                    "GCodeNotImplemented(line={}, gcode={})",
                    ln,
                    string.as_str()
                );
            }
            GCodeLineParserError::FatalError => {
                defmt::write!(fmt, "FatalError");
            }
            GCodeLineParserError::EOF => {
                defmt::write!(fmt, "EOF");
            }
        }
    }
}

pub struct RawGCodeSpec {
    code: char,
    spec: Option<i32>,
    sub: Option<i32>,
}

impl RawGCodeSpec {
    pub fn from(code: char, spec: Option<(i32, u8)>) -> Self {
        match spec {
            None => {
                Self {
                    code,
                    spec: None,
                    sub: None,
                }
            }
            Some((_num, _scale)) => {
                if _scale == 0 {
                    Self {
                        code,
                        spec: Some(_num),
                        sub: None,
                    }
                }
                else {
                    let sc = 10_i32.pow(_scale as u32);
                    let sp = _num / sc;
                    let ss = _num % sc;
                    Self {
                        code,
                        spec: Some(sp),
                        sub: Some(ss)
                    }
                }
            }
        }
    }
}
impl core::fmt::Debug for RawGCodeSpec {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(f, "{}", self.code.to_uppercase())?;
        match &self.spec {
            Some(_spec) => {
                core::write!(f, "{}", _spec)?;
                match &self.sub {
                    Some(_sub) => {
                        core::write!(f, ".{}", _sub)
                    }
                    _ => Ok(())
                }
            }
            _ => Ok(())
        }

    }
}

// dyn trait could reduce code size a lot with the penalty of the indirection
pub struct GCodeLineParser<STREAM>
where
    STREAM: async_gcode::ByteStream<Item = Result<u8, async_gcode::Error>>,
{
    raw_parser: async_gcode::Parser<STREAM, async_gcode::Error>,
    gcode_line: Option<u32>,
}

impl<STREAM> GCodeLineParser<STREAM>
where
    STREAM: async_gcode::ByteStream<Item = Result<u8, async_gcode::Error>>,
{
    pub fn new(stream: STREAM) -> Self {
        Self {
            raw_parser: async_gcode::Parser::new(stream),
            gcode_line: None,
        }
    }

    #[allow(unused)]
    pub fn gcode_line(&self) -> Option<u32> {
        self.gcode_line
    }

    pub async fn next_gcode(&mut self) -> Result<GCodeCmd, GCodeLineParserError> {

        // The GcodeCmd being constructed.
        // * Initially set to none
        // * Reset back to None when built, updated and returned
        let mut current_gcode: Option<GCodeCmd> = None;
        // Same as previous but unparsed
        let mut raw_gcode_spec: Option<RawGCodeSpec> = None;

        let mut line_num = None;
        let mut skip_gcode = false;


        loop {
            match self.raw_parser.next().await {
                None => {
                    hwa::trace!("EOF reading from stream");
                    return Err(GCodeLineParserError::EOF);
                }
                Some(parser_gcode) => {
                    match parser_gcode {
                        Ok(g) => {
                            match g {
                                // FIXME Undo this hacky temporary solution in async-gcode.
                                // A sub-protocol approach is preferred
                                #[cfg(feature = "grbl-compat")]
                                async_gcode::GCode::StatusCommand => {
                                    return Ok(GCodeCmd::new(
                                        self.raw_parser.get_current_line(),
                                        line_num,
                                        GCodeValue::Status
                                    ))
                                }
                                async_gcode::GCode::BlockDelete => {
                                    skip_gcode = true;
                                    //crate::debug!("BlockDelete");
                                }
                                async_gcode::GCode::LineNumber(n) => {
                                    line_num = Some(n);
                                }
                                async_gcode::GCode::Word(ch, fv) => {
                                    if skip_gcode {
                                        continue;
                                    }
                                    let frx = match &fv {
                                        async_gcode::RealValue::Literal(
                                            async_gcode::Literal::RealNumber(f),
                                        ) => Some((f.integer_part(), f.scale())),
                                        _ => None,
                                    };
                                    if let Some(current_gcode) = &mut current_gcode {
                                        // We already are building a GCodeCmd, so we update its fields
                                        update_current(current_gcode, ch, frx, fv)
                                    }
                                    else {
                                        hwa::debug!("Initializing: {} {:?}", ch, frx);
                                        raw_gcode_spec.replace(RawGCodeSpec::from(ch, frx));
                                        // We need to start building the current GCodeCmd that is being parsed
                                        match init_current(ch, frx) {
                                            None => {
                                                // Unable to init: The GCodeValue is unexpected
                                                // Hence, we will skip every [async_gcode::GCode::Word] until [async_gcode::GCode::Execute]
                                                skip_gcode = true;
                                            }
                                            Some(gcode_value) => {
                                                current_gcode.replace(
                                                    GCodeCmd::new(
                                                        0, // Will update later at [async_gcode::GCode::Execute]
                                                        line_num,
                                                        gcode_value)
                                                );
                                            }
                                        }
                                    }
                                }
                                async_gcode::GCode::Execute => {
                                    // Reset skip_gcode status
                                    skip_gcode = false;
                                    self.gcode_line = line_num;
                                    match current_gcode.take() {
                                        None => {
                                            match raw_gcode_spec.take() {
                                                None => {
                                                    hwa::warn!("Ignoring empty line");
                                                    continue
                                                }, // Empty line. Just ignore
                                                Some(rgs) => {
                                                    return Err(GCodeLineParserError::GCodeNotImplemented(
                                                        self.raw_parser.get_current_line(),
                                                        alloc::format!("{:?}", rgs),
                                                    ));
                                                }
                                            }
                                        }
                                        Some(cgv) => {
                                            return Ok(cgv)
                                        },
                                    }
                                }
                                _ => {
                                    return Err(GCodeLineParserError::GCodeNotImplemented(
                                        self.raw_parser.get_current_line(),
                                        alloc::string::String::from("N/A"),
                                    ))
                                }
                            }
                        }
                        Err(error) => {
                            match error {
                                async_gcode::Error::UnexpectedByte(b) => {
                                    hwa::warn!("Unexpected byte: {} ({})", b, char::from(b));
                                }
                                async_gcode::Error::NumberOverflow => {
                                    hwa::warn!("Number overflow");
                                }
                                async_gcode::Error::BadNumberFormat => {
                                    hwa::warn!("Bad number format");
                                }
                                _e => {
                                    #[cfg(feature = "native")]
                                    hwa::error!("Parse error {:?}", _e);
                                    hwa::error!("Parse error");
                                }
                            }
                            return Err(GCodeLineParserError::ParseError(self.raw_parser.get_current_line()));
                        }
                    }
                }
            }
        }
    }

    pub fn reset(&mut self) {
        hwa::warn!("AsyncGcodeParser reset");
        self.raw_parser.reset();
    }

    pub fn reset_current_line(&mut self) {
        hwa::warn!("AsyncGcodeParser reset_current_line");
        self.raw_parser.update_current_line(0);
    }

    pub fn get_state(&self) -> async_gcode::AsyncParserState {
        self.raw_parser.get_state()
    }

    pub fn get_line(&self) -> u32 {
        self.raw_parser.get_current_line()
    }


    #[allow(unused)]
    pub async fn close(&mut self) {
        self.raw_parser.reset();
    }
}

/// This trait is just a hack to give backward compatibility with unpatched async-gcode 0.3.0
/// Normally, won't be used as it is patched in cargo.toml
/// Purpose of this trait and impl is just to be able to publish printhor in crates.io
#[allow(unused)]
trait FixedAdaptor {
    fn integer_part(&self) -> i32;
    fn scale(&self) -> u8;
}

impl FixedAdaptor for f64 {
    fn integer_part(&self) -> i32 {
        panic!("Please, use patched async-gcode instead")
    }

    fn scale(&self) -> u8 {
        panic!("Please, use patched async-gcode instead")
    }
}

/// Initialize and EMPTY GCodeValue variant from ch, frx spec coming from parser
fn init_current(ch: char, frx: Option<(i32, u8)>) -> Option<GCodeValue> {
    match (ch, frx) {
        #[cfg(feature = "grbl-compat")]
        ('$', None) => Some(GCodeValue::GRBLCmd),
        ('g', None) => Some(GCodeValue::G),
        ('g', Some((0, 0))) => Some(GCodeValue::G0(XYZF::new())),
        ('g', Some((1, 0))) => Some(GCodeValue::G1(XYZEFS::new())),
        ('g', Some((4, 0))) => Some(GCodeValue::G4),
        ('g', Some((10, 0))) => Some(GCodeValue::G10),
        ('g', Some((17, 0))) => Some(GCodeValue::G17),
        ('g', Some((21, 0))) => Some(GCodeValue::G21),
        ('g', Some((28, 0))) => Some(GCodeValue::G28(XYZE::new())),
        ('g', Some((29, 0))) => Some(GCodeValue::G29),
        ('g', Some((31, 0))) => Some(GCodeValue::G31),
        ('g', Some((32, 0))) => Some(GCodeValue::G32),
        ('g', Some((80, 0))) => Some(GCodeValue::G80),
        ('g', Some((90, 0))) => Some(GCodeValue::G90),
        ('g', Some((91, 0))) => Some(GCodeValue::G91),
        ('g', Some((92, 0))) => Some(GCodeValue::G92(XYZE::new())),
        ('g', Some((94, 0))) => Some(GCodeValue::G94),
        ('g', Some((291, 1))) => Some(GCodeValue::G29_1),
        ('g', Some((292, 1))) => Some(GCodeValue::G29_2),
        ('m', None) => Some(GCodeValue::M),
        ('m', Some((3, 0))) => Some(GCodeValue::M3),
        ('m', Some((5, 0))) => Some(GCodeValue::M5),
        ('m', Some((20, 0))) => Some(GCodeValue::M20(None)),
        ('m', Some((23, 0))) => Some(GCodeValue::M23(None)),
        ('m', Some((24, 0))) => Some(GCodeValue::M24),
        ('m', Some((25, 0))) => Some(GCodeValue::M25),
        ('m', Some((73, 0))) => Some(GCodeValue::M73),
        ('m', Some((79, 0))) => Some(GCodeValue::M79),
        ('m', Some((80, 0))) => Some(GCodeValue::M80),
        ('m', Some((81, 0))) => Some(GCodeValue::M81),
        ('m', Some((83, 0))) => Some(GCodeValue::M83),
        ('m', Some((84, 0))) => Some(GCodeValue::M84),
        ('m', Some((100, 0))) => Some(GCodeValue::M100),
        ('m', Some((104, 0))) => Some(GCodeValue::M104(S::new())),
        ('m', Some((105, 0))) => Some(GCodeValue::M105),
        ('m', Some((106, 0))) => Some(GCodeValue::M106),
        ('m', Some((107, 0))) => Some(GCodeValue::M107),
        ('m', Some((109, 0))) => Some(GCodeValue::M109(S::new())),
        ('m', Some((110, 0))) => Some(GCodeValue::M110(N::new())),
        ('m', Some((114, 0))) => Some(GCodeValue::M114),
        ('m', Some((115, 0))) => Some(GCodeValue::M115),
        ('m', Some((117, 0))) => Some(GCodeValue::M117),
        ('m', Some((119, 0))) => Some(GCodeValue::M119),
        ('m', Some((140, 0))) => Some(GCodeValue::M140(S::new())),
        ('m', Some((190, 0))) => Some(GCodeValue::M190),
        ('m', Some((201, 0))) => Some(GCodeValue::M201),
        ('m', Some((203, 0))) => Some(GCodeValue::M203),
        ('m', Some((204, 0))) => Some(GCodeValue::M204),
        ('m', Some((205, 0))) => Some(GCodeValue::M205),
        ('m', Some((206, 0))) => Some(GCodeValue::M206),
        ('m', Some((220, 0))) => Some(GCodeValue::M220(S::new())),
        ('m', Some((221, 0))) => Some(GCodeValue::M221(S::new())),
        ('m', Some((502, 0))) => Some(GCodeValue::M502),
        ('m', Some((8621, 1))) => Some(GCodeValue::M862_1),
        ('m', Some((8623, 1))) => Some(GCodeValue::M862_3),
        ('m', Some((900, 0))) => Some(GCodeValue::M900),
        ('m', Some((907, 0))) => Some(GCodeValue::M907),
        _ => {
            None
        }
    }
}

fn update_current(gcode_cmd: &mut GCodeCmd, ch: char, frx: Option<(i32, u8)>, fv: async_gcode::RealValue) {
    match &mut gcode_cmd.value {
        #[cfg(feature = "grbl-compat")]
        GCodeValue::Status => match (ch, frx) {
            ('I', Some(val)) => {
                hwa::warn!("TODO!!");
            }
            _ => {}
        },
        GCodeValue::G0(coord) => match (ch, frx) {
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            ('f', Some(val)) => {
                coord.f.replace(helpers::to_fixed(val));
            }
            _ => {}
        },

        GCodeValue::G1(coord) => match (ch, frx) {
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            ('e', Some(val)) => {
                coord.e.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        GCodeValue::G28(coord) => match (ch, frx) {
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        GCodeValue::G92(coord) => match (ch, frx) {
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            ('e', Some(val)) => {
                coord.e.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        GCodeValue::M20(path) => {
            if ch == 'f' {
                if let async_gcode::RealValue::Literal(
                    async_gcode::Literal::String(mstr),
                ) = fv {
                    path.replace(mstr);
                }
            }
        }
        GCodeValue::M23(file) => {
            if ch == 'f' {
                if let async_gcode::RealValue::Literal(
                    async_gcode::Literal::String(mstr),
                ) = fv
                {
                    file.replace(mstr);
                }
            }
        }
        GCodeValue::M104(coord)
        | GCodeValue::M109(coord)
        | GCodeValue::M140(coord)
        | GCodeValue::M220(coord)  => match (ch, frx) {
            ('s', Some(val)) => {
                coord.s.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        GCodeValue::M110(coord) => match (ch, frx) {
            ('n', Some(val)) => {
                coord.n.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        _ => {}
    }
}