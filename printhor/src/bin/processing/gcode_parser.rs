use crate::helpers;
use crate::hwa;
#[allow(unused)]
use crate::processing::EFSXYZ;
#[allow(unused)]
use crate::processing::EXYZ;
#[allow(unused)]
use crate::processing::FXYZ;
#[allow(unused)]
use crate::processing::GCodeCmd;
#[allow(unused)]
use crate::processing::GCodeValue;
#[allow(unused)]
use crate::processing::N;
#[allow(unused)]
use crate::processing::S;
use hwa::CommChannel;

#[derive(Debug)]
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

#[cfg(feature = "debug-gcode")]
struct FormatableGCode<'a>(&'a async_gcode::GCode);

#[cfg(feature = "debug-gcode")]
impl core::fmt::Debug for FormatableGCode<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            async_gcode::GCode::StatusCommand => {
                f.write_str("StatusCommand")?;
            }
            async_gcode::GCode::BlockDelete => {
                f.write_str("BlockDelete")?;
            }
            async_gcode::GCode::LineNumber(_ln) => {
                core::write!(f, "LineNumber<{:?}>", _ln)?;
            }
            async_gcode::GCode::Word(_w, _v) => {
                core::write!(f, "Word[{}", _w.to_uppercase())?;
                match _v {
                    async_gcode::RealValue::Literal(_l) => {
                        match _l {
                            async_gcode::Literal::RealNumber(_r) => {
                                let div = 10_i32.pow(_r.scale().into());
                                let w0 = _r.integer_part() / div;
                                let w1 = _r.integer_part() % div;
                                core::write!(
                                    f,
                                    "{}.{:0width$}",
                                    w0,
                                    w1,
                                    width = _r.scale() as usize
                                )?;
                            }
                            async_gcode::Literal::String(_s) => {
                                core::write!(f, " {}", _s)?;
                            }
                        }
                        core::write!(f, "]")?;
                    }
                    _ => {}
                }
            }
            async_gcode::GCode::Text(_t) => match _t {
                async_gcode::RealValue::Literal(async_gcode::Literal::String(_s)) => {
                    core::write!(f, "String[{}]", _s)?;
                }
                _ => {}
            },
            async_gcode::GCode::Execute => {
                core::write!(f, "Execute")?;
            }
            #[allow(unreachable_patterns)]
            _ => {}
        }
        Ok(())
    }
}
#[cfg(all(feature = "with-defmt", feature = "debug-gcode"))]
impl defmt::Format for FormatableGCode<'_> {
    fn format(&self, f: defmt::Formatter) {
        match self.0 {
            async_gcode::GCode::StatusCommand => {
                defmt::write!(f, "StatusCommand");
            }
            async_gcode::GCode::BlockDelete => {
                defmt::write!(f, "BlockDelete");
            }
            async_gcode::GCode::LineNumber(_ln) => {
                defmt::write!(f, "LineNumber<{:?}>", _ln);
            }
            async_gcode::GCode::Word(_w, _v) => {
                use alloc::string::ToString;
                defmt::write!(f, "Word[{:?}", _w.to_uppercase().to_string().as_str());
                match _v {
                    async_gcode::RealValue::Literal(_l) => {
                        match _l {
                            async_gcode::Literal::RealNumber(_r) => {
                                defmt::write!(
                                    f,
                                    " val: {}, scale: {}",
                                    _r.integer_part(),
                                    _r.scale()
                                );
                            }
                            async_gcode::Literal::String(_s) => {
                                defmt::write!(f, " {}", _s.as_str());
                            }
                        }
                        defmt::write!(f, "]");
                    }
                    _ => {}
                }
            }
            async_gcode::GCode::Text(_t) => match _t {
                async_gcode::RealValue::Literal(async_gcode::Literal::String(_s)) => {
                    defmt::write!(f, "String[{}]", _s.as_str());
                }
                _ => {}
            },
            async_gcode::GCode::Execute => {
                defmt::write!(f, "Execute");
            }
            #[allow(unreachable_patterns)]
            _ => {}
        }
    }
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

/// Represents the raw specification of a G-code command.
///
/// G-code is a language used to control CNC machines, 3D printers, and other similar equipment.
/// A `RawGCodeSpec` captures the initial character of the command (e.g., 'G', 'M') and any numerical
/// specifics associated with it, potentially including a sub-value used for more granular control.
///
/// # Fields
/// - `code`: The main character of the G-code command.
/// - `spec`: An optional numerical part that follows the main character. For example, 'G1' would have
///   '1' as its spec.
/// - `sub`: An optional sub-value that provides additional specificity. For example, 'G1.1' would have
///   '1' as its spec and '1' as its sub.
///
/// # Example
///
/// ```
/// let gcode = RawTGCodeSpec::from('G', Some((1, 0)));
/// assert_eq!(format!("{:?}", gcode), "G1");
/// ```
pub struct RawGCodeSpec {
    code: char,
    spec: Option<i32>,
    sub: Option<i32>,
}

impl RawGCodeSpec {
    pub fn from(code: char, spec: Option<(i32, u8)>) -> Self {
        match spec {
            None => Self {
                code,
                spec: None,
                sub: None,
            },
            Some((_num, _scale)) => {
                if _scale == 0 {
                    Self {
                        code,
                        spec: Some(_num),
                        sub: None,
                    }
                } else {
                    let sc = 10_i32.pow(_scale as u32);
                    let sp = _num / sc;
                    let ss = _num % sc;
                    Self {
                        code,
                        spec: Some(sp),
                        sub: Some(ss),
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
                    _ => Ok(()),
                }
            }
            _ => Ok(()),
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

    pub fn gcode_line(&self) -> Option<u32> {
        self.gcode_line
    }

    pub async fn next_gcode(
        &mut self,
        _channel: CommChannel,
    ) -> Result<GCodeCmd, GCodeLineParserError> {
        // The GcodeCmd being constructed.
        // * Initially set to none
        // * Reset back to None when built, updated and returned
        let mut current_gcode: Option<GCodeCmd> = None;
        // Same as previous but unparsed
        let mut raw_gcode_spec: Option<RawGCodeSpec> = None;

        let mut tagged_line_num = None;
        let mut skip_gcode = false;

        loop {
            match self.raw_parser.next().await {
                None => {
                    hwa::trace!("EOF reading from stream");
                    self.gcode_line = tagged_line_num;
                    match current_gcode.take() {
                        None => {
                            match raw_gcode_spec.take() {
                                None => {
                                    return Err(GCodeLineParserError::EOF);
                                } // Empty line. Just ignore
                                Some(rgs) => {
                                    return Err(GCodeLineParserError::GCodeNotImplemented(
                                        self.raw_parser.get_current_line(),
                                        alloc::format!("{:?}", rgs),
                                    ));
                                }
                            }
                        }
                        Some(mut cgv) => {
                            cgv.order_num = self.raw_parser.get_current_line();
                            return Ok(cgv);
                        }
                    }
                }
                Some(parser_gcode) => {
                    match parser_gcode {
                        Ok(g) => {
                            #[cfg(feature = "debug-gcode")]
                            hwa::info!(
                                "[debug-gcode] channel: {:?} gcode: {:?}",
                                _channel,
                                FormatableGCode(&g)
                            );
                            match g {
                                // FIXME Undo this hacky temporary solution in async-gcode.
                                // A sub-protocol approach is preferred
                                #[cfg(feature = "grbl-compat")]
                                async_gcode::GCode::StatusCommand => {
                                    return Ok(GCodeCmd::new(
                                        self.raw_parser.get_current_line(),
                                        tagged_line_num,
                                        GCodeValue::Status,
                                    ));
                                }
                                async_gcode::GCode::BlockDelete => {
                                    skip_gcode = true;
                                    //crate::debug!("BlockDelete");
                                }
                                async_gcode::GCode::LineNumber(n) => {
                                    tagged_line_num = n;
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
                                    } else {
                                        raw_gcode_spec.replace(RawGCodeSpec::from(ch, frx));

                                        let has_text_argument = match (ch, frx) {
                                            ('m', Some((23, 0)))
                                            | ('m', Some((117, 0)))
                                            | ('m', Some((118, 0))) => true,
                                            _ => false,
                                        };
                                        // We need to start building the current GCodeCmd that is being parsed
                                        match init_current(ch, frx) {
                                            None => {
                                                // Unable to init: The GCodeValue is unexpected
                                                // Hence, we will skip every [async_gcode::GCode::Word] until [async_gcode::GCode::Execute]
                                                skip_gcode = true;
                                            }
                                            Some(gcode_value) => {
                                                current_gcode.replace(GCodeCmd::new(
                                                    0, // Will update later at [async_gcode::GCode::Execute]
                                                    tagged_line_num,
                                                    gcode_value,
                                                ));
                                                if has_text_argument {
                                                    self.raw_parser.switch_to_text();
                                                }
                                            }
                                        }
                                    }
                                }
                                async_gcode::GCode::Text(value) => {
                                    if let Some(current_gcode) = &mut current_gcode {
                                        // We already are building a GCodeCmd, so we update its fields
                                        update_current(current_gcode, 'T', None, value)
                                    }
                                }
                                async_gcode::GCode::Execute => {
                                    // Reset skip_gcode status
                                    skip_gcode = false;
                                    self.gcode_line = tagged_line_num;
                                    match current_gcode.take() {
                                        None => {
                                            match raw_gcode_spec.take() {
                                                None => {
                                                    #[cfg(feature = "trace-commands")]
                                                    hwa::warn!("Ignoring empty line");
                                                    continue;
                                                } // Empty line. Just ignore
                                                Some(rgs) => {
                                                    return Err(
                                                        GCodeLineParserError::GCodeNotImplemented(
                                                            self.raw_parser.get_current_line(),
                                                            alloc::format!("{:?}", rgs),
                                                        ),
                                                    );
                                                }
                                            }
                                        }
                                        Some(mut cgv) => {
                                            cgv.order_num = self.raw_parser.get_current_line();
                                            return Ok(cgv);
                                        }
                                    }
                                }
                                _ => {
                                    return Err(GCodeLineParserError::GCodeNotImplemented(
                                        self.raw_parser.get_current_line(),
                                        alloc::format!("N/A"),
                                    ));
                                }
                            }
                        }
                        Err(error) => {
                            match error {
                                async_gcode::Error::UnexpectedByte(_b) => {
                                    hwa::warn!("Unexpected byte: {} ({})", _b, char::from(_b));
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
                            return Err(GCodeLineParserError::ParseError(
                                self.raw_parser.get_current_line(),
                            ));
                        }
                    }
                }
            }
        }
    }

    pub async fn reset(&mut self) {
        hwa::debug!("AsyncGcodeParser reset");
        self.raw_parser.reset().await;
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

    pub async fn close(&mut self) {
        self.raw_parser.reset().await;
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
        #[cfg(feature = "with-motion")]
        ('g', Some((0, 0))) => Some(GCodeValue::G0(FXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('g', Some((1, 0))) => Some(GCodeValue::G1(EFSXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('g', Some((4, 0))) => Some(GCodeValue::G4(S::new())),
        #[cfg(feature = "with-motion")]
        ('g', Some((10, 0))) => Some(GCodeValue::G10),
        #[cfg(feature = "with-motion")]
        ('g', Some((17, 0))) => Some(GCodeValue::G17),
        #[cfg(feature = "with-motion")]
        ('g', Some((21, 0))) => Some(GCodeValue::G21),
        #[cfg(feature = "with-motion")]
        ('g', Some((28, 0))) => Some(GCodeValue::G28(EXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('g', Some((29, 0))) => Some(GCodeValue::G29),
        #[cfg(feature = "with-motion")]
        ('g', Some((291, 1))) => Some(GCodeValue::G29_1),
        #[cfg(feature = "with-motion")]
        ('g', Some((292, 1))) => Some(GCodeValue::G29_2),
        #[cfg(feature = "with-probe")]
        ('g', Some((31, 0))) => Some(GCodeValue::G31),
        #[cfg(feature = "with-probe")]
        ('g', Some((32, 0))) => Some(GCodeValue::G32),
        ('g', Some((80, 0))) => Some(GCodeValue::G80),
        #[cfg(feature = "with-motion")]
        ('g', Some((90, 0))) => Some(GCodeValue::G90),
        #[cfg(feature = "with-motion")]
        ('g', Some((91, 0))) => Some(GCodeValue::G91),
        #[cfg(feature = "with-motion")]
        ('g', Some((92, 0))) => Some(GCodeValue::G92(EXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('g', Some((94, 0))) => Some(GCodeValue::G94),
        ('m', None) => Some(GCodeValue::M),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((2, 0))) => Some(GCodeValue::M2),
        ('m', Some((3, 0))) => Some(GCodeValue::M3),
        ('m', Some((4, 0))) => Some(GCodeValue::M4),
        ('m', Some((5, 0))) => Some(GCodeValue::M5),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((20, 0))) => Some(GCodeValue::M20(None)),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((21, 0))) => Some(GCodeValue::M21),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((23, 0))) => Some(GCodeValue::M23(None)),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((24, 0))) => Some(GCodeValue::M24),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((25, 0))) => Some(GCodeValue::M25),
        ('m', Some((37, 0))) => Some(GCodeValue::M37(S::new())),
        ('m', Some((73, 0))) => Some(GCodeValue::M73),
        ('m', Some((79, 0))) => Some(GCodeValue::M79),
        #[cfg(feature = "with-ps-on")]
        ('m', Some((80, 0))) => Some(GCodeValue::M80),
        #[cfg(feature = "with-ps-on")]
        ('m', Some((81, 0))) => Some(GCodeValue::M81),
        ('m', Some((83, 0))) => Some(GCodeValue::M83),
        #[cfg(feature = "with-motion")]
        ('m', Some((84, 0))) => Some(GCodeValue::M84),
        ('m', Some((100, 0))) => Some(GCodeValue::M100),
        #[cfg(feature = "with-hot-end")]
        ('m', Some((104, 0))) => Some(GCodeValue::M104(S::new())),
        ('m', Some((105, 0))) => Some(GCodeValue::M105),
        #[cfg(feature = "with-fan-layer")]
        ('m', Some((106, 0))) => Some(GCodeValue::M106),
        #[cfg(feature = "with-fan-layer")]
        ('m', Some((107, 0))) => Some(GCodeValue::M107),
        #[cfg(feature = "with-hot-end")]
        ('m', Some((109, 0))) => Some(GCodeValue::M109(S::new())),
        ('m', Some((110, 0))) => Some(GCodeValue::M110(N::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((114, 0))) => Some(GCodeValue::M114),
        ('m', Some((115, 0))) => Some(GCodeValue::M115),
        ('m', Some((117, 0))) => Some(GCodeValue::M117(None)),
        ('m', Some((118, 0))) => Some(GCodeValue::M118(None)),
        ('m', Some((119, 0))) => Some(GCodeValue::M119),
        #[cfg(feature = "with-hot-bed")]
        ('m', Some((140, 0))) => Some(GCodeValue::M140(S::new())),
        #[cfg(feature = "with-hot-bed")]
        ('m', Some((190, 0))) => Some(GCodeValue::M190),
        #[cfg(feature = "with-motion")]
        ('m', Some((201, 0))) => Some(GCodeValue::M201),
        #[cfg(feature = "with-motion")]
        ('m', Some((203, 0))) => Some(GCodeValue::M203),
        ('m', Some((204, 0))) => Some(GCodeValue::M204),
        ('m', Some((205, 0))) => Some(GCodeValue::M205),
        #[cfg(feature = "with-motion")]
        ('m', Some((206, 0))) => Some(GCodeValue::M206),
        ('m', Some((220, 0))) => Some(GCodeValue::M220(S::new())),
        ('m', Some((221, 0))) => Some(GCodeValue::M221(S::new())),
        ('m', Some((502, 0))) => Some(GCodeValue::M502),
        ('m', Some((503, 0))) => Some(GCodeValue::M503(S::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((8621, 1))) => Some(GCodeValue::M862_1),
        #[cfg(feature = "with-motion")]
        ('m', Some((8623, 1))) => Some(GCodeValue::M862_3),
        #[cfg(feature = "with-motion")]
        ('m', Some((900, 0))) => Some(GCodeValue::M900),
        #[cfg(feature = "with-motion")]
        ('m', Some((907, 0))) => Some(GCodeValue::M907),
        _ => None,
    }
}

fn update_current(
    gcode_cmd: &mut GCodeCmd,
    ch: char,
    frx: Option<(i32, u8)>,
    fv: async_gcode::RealValue,
) {
    match &mut gcode_cmd.value {
        #[cfg(feature = "grbl-compat")]
        GCodeValue::Status => match (ch, frx) {
            ('I', Some(_val)) => {
                hwa::warn!("TODO!!");
            }
            _ => {}
        },
        #[cfg(feature = "with-motion")]
        GCodeValue::G0(coord) => match (ch, frx) {
            ('f', Some(val)) => {
                coord.f.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-x-axis")]
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-y-axis")]
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-z-axis")]
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-a-axis")]
            ('a', Some(val)) => {
                coord.a.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-b-axis")]
            ('b', Some(val)) => {
                coord.b.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-c-axis")]
            ('c', Some(val)) => {
                coord.c.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-i-axis")]
            ('i', Some(val)) => {
                coord.i.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-j-axis")]
            ('j', Some(val)) => {
                coord.j.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-k-axis")]
            ('k', Some(val)) => {
                coord.k.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-u-axis")]
            ('u', Some(val)) => {
                coord.u.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-v-axis")]
            ('v', Some(val)) => {
                coord.v.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-w-axis")]
            ('w', Some(val)) => {
                coord.w.replace(helpers::to_fixed(val));
            }
            _ => {}
        },

        #[cfg(feature = "with-motion")]
        GCodeValue::G1(coord) => match (ch, frx) {
            #[cfg(feature = "with-e-axis")]
            ('e', Some(val)) => {
                coord.e.replace(helpers::to_fixed(val));
            }
            ('f', Some(val)) => {
                coord.f.replace(helpers::to_fixed(val));
            }
            ('s', Some(val)) => {
                coord.s.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-x-axis")]
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-y-axis")]
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-z-axis")]
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-a-axis")]
            ('a', Some(val)) => {
                coord.a.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-b-axis")]
            ('b', Some(val)) => {
                coord.b.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-c-axis")]
            ('c', Some(val)) => {
                coord.c.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-i-axis")]
            ('i', Some(val)) => {
                coord.i.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-j-axis")]
            ('j', Some(val)) => {
                coord.j.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-k-axis")]
            ('k', Some(val)) => {
                coord.k.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-u-axis")]
            ('u', Some(val)) => {
                coord.u.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-v-axis")]
            ('v', Some(val)) => {
                coord.v.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-w-axis")]
            ('w', Some(val)) => {
                coord.w.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        #[cfg(feature = "with-motion")]
        GCodeValue::G4(param) => match (ch, frx) {
            ('s', Some(val)) => {
                param.s.replace(helpers::to_fixed(val).abs());
            }
            _ => {}
        },
        #[cfg(feature = "with-motion")]
        GCodeValue::G28(coord) => match (ch, frx) {
            #[cfg(feature = "with-e-axis")]
            ('e', Some(val)) => {
                coord.e.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-x-axis")]
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-y-axis")]
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-z-axis")]
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-a-axis")]
            ('a', Some(val)) => {
                coord.a.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-b-axis")]
            ('b', Some(val)) => {
                coord.b.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-c-axis")]
            ('c', Some(val)) => {
                coord.c.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-i-axis")]
            ('i', Some(val)) => {
                coord.i.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-j-axis")]
            ('j', Some(val)) => {
                coord.j.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-k-axis")]
            ('k', Some(val)) => {
                coord.k.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-u-axis")]
            ('u', Some(val)) => {
                coord.u.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-v-axis")]
            ('v', Some(val)) => {
                coord.v.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-w-axis")]
            ('w', Some(val)) => {
                coord.w.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        #[cfg(feature = "with-motion")]
        GCodeValue::G92(coord) => match (ch, frx) {
            #[cfg(feature = "with-e-axis")]
            ('e', Some(val)) => {
                coord.e.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-x-axis")]
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-y-axis")]
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-z-axis")]
            ('z', Some(val)) => {
                coord.z.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-a-axis")]
            ('a', Some(val)) => {
                coord.a.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-b-axis")]
            ('b', Some(val)) => {
                coord.b.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-c-axis")]
            ('c', Some(val)) => {
                coord.c.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-i-axis")]
            ('i', Some(val)) => {
                coord.i.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-j-axis")]
            ('j', Some(val)) => {
                coord.j.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-k-axis")]
            ('k', Some(val)) => {
                coord.k.replace(helpers::to_fixed(val));
            }
            //
            #[cfg(feature = "with-u-axis")]
            ('u', Some(val)) => {
                coord.u.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-v-axis")]
            ('v', Some(val)) => {
                coord.v.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-w-axis")]
            ('w', Some(val)) => {
                coord.w.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        #[cfg(feature = "with-sd-card")]
        GCodeValue::M20(path) => {
            if ch == 'f' {
                if let async_gcode::RealValue::Literal(async_gcode::Literal::String(mstr)) = fv {
                    path.replace(mstr);
                }
            }
        }
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        GCodeValue::M23(file) => {
            if let async_gcode::RealValue::Literal(async_gcode::Literal::String(mstr)) = fv {
                file.replace(mstr);
            }
        }
        GCodeValue::M37(coord) => match (ch, frx) {
            ('s', Some(val)) => {
                coord.s.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        #[cfg(feature = "with-hot-end")]
        GCodeValue::M104(coord) => match (ch, frx) {
            ('s', Some(val)) => {
                coord.s.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        #[cfg(feature = "with-hot-end")]
        GCodeValue::M109(coord) => match (ch, frx) {
            ('s', Some(val)) => {
                coord.s.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        GCodeValue::M117(msg) | GCodeValue::M118(msg) => {
            if let async_gcode::RealValue::Literal(async_gcode::Literal::String(text)) = fv {
                msg.replace(text);
            }
        }
        #[cfg(feature = "with-hot-bed")]
        GCodeValue::M140(coord) => match (ch, frx) {
            ('s', Some(val)) => {
                coord.s.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        GCodeValue::M220(coord) => match (ch, frx) {
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
        GCodeValue::M503(param) => match (ch, frx) {
            ('s', Some(val)) => {
                param.s.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        _ => {}
    }
}

#[cfg(feature = "native")]
#[cfg(test)]
mod tests {
    use super::*;
    use async_gcode::{AsyncParserState, ByteStream};
    use std::collections::VecDeque;

    struct BufferStream {
        buff: VecDeque<u8>,
    }

    impl BufferStream {
        const fn new(buff: VecDeque<u8>) -> Self {
            Self { buff }
        }
    }

    impl async_gcode::ByteStream for BufferStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            match self.buff.pop_front() {
                None => None,
                Some(_b) => Some(Ok(_b)),
            }
        }

        async fn recovery_check(&mut self) {}
    }

    async fn check_m32_benchy_result(parser: &mut GCodeLineParser<BufferStream>) {
        match parser.next_gcode(CommChannel::Internal).await {
            Ok(_cmd) => match _cmd.value {
                GCodeValue::M23(Some(_path)) => {
                    assert!(_path.eq("BENCHY.G"), "Got the proper string param")
                }
                _ => assert!(false, "Unexpected code value"),
            },
            Err(_e) => {
                assert!(false, "Got an error");
            }
        }
    }

    async fn check_display_m117_result(parser: &mut GCodeLineParser<BufferStream>) {
        match parser.next_gcode(CommChannel::Internal).await {
            Ok(_cmd) => match _cmd.value {
                GCodeValue::M117(Some(_msg)) => {
                    assert!(_msg.eq("Hello World!"), "Got the proper string param")
                }
                _ => assert!(false, "Unexpected code value"),
            },
            Err(_e) => {
                assert!(false, "Got an error");
            }
        }
    }
    async fn check_display_m118_result(parser: &mut GCodeLineParser<BufferStream>) {
        match parser.next_gcode(CommChannel::Internal).await {
            Ok(_cmd) => match _cmd.value {
                GCodeValue::M118(Some(_msg)) => {
                    assert!(_msg.eq("Hello World!"), "Got the proper string param")
                }
                _ => assert!(false, "Unexpected code value"),
            },
            Err(_e) => {
                assert!(false, "Got an error");
            }
        }
    }
    #[futures_test::test]
    async fn test_m23_with_newline() {
        let stream = BufferStream::new("M23 BENCHY.G\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        check_m32_benchy_result(&mut parser).await;
    }

    #[futures_test::test]
    async fn test_m23_with_eof_condition() {
        let stream = BufferStream::new("M23 BENCHY.G".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        check_m32_benchy_result(&mut parser).await;
    }

    #[futures_test::test]
    async fn test_m23_with_trailin_comments() {
        let stream = BufferStream::new("M23 BENCHY.G".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        check_m32_benchy_result(&mut parser).await;
    }

    #[futures_test::test]
    async fn test_m117() {
        let stream = BufferStream::new("M117 Hello World!".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        check_display_m117_result(&mut parser).await;
    }

    #[futures_test::test]
    async fn test_m118() {
        let stream = BufferStream::new("M118 Hello World!".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        check_display_m118_result(&mut parser).await;
    }

    #[futures_test::test]
    async fn test_gs() {
        // TODO: Check EOF condition
        let stream = BufferStream::new(
            "G0 F0 E0 X0 Y0 Z0 A0 B0 C0 I0 J0 K0 U0 V0 W0\n"
                .as_bytes()
                .to_vec()
                .into(),
        );
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G0 OK");

        let stream = BufferStream::new(
            "G1 F0 E0 X0 Y0 Z0 A0 B0 C0 I0 J0 K0 U0 V0 W0\n"
                .as_bytes()
                .to_vec()
                .into(),
        );
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G1 OK");

        let stream = BufferStream::new(
            "G92 F0 E0 X0 Y0 Z0 A0 B0 C0 I0 J0 K0 U0 V0 W0\n"
                .as_bytes()
                .to_vec()
                .into(),
        );
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G1 OK");

        let stream = BufferStream::new("G4\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G4 OK");

        let stream = BufferStream::new("G4 S10\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G4 S10 OK");

        let stream = BufferStream::new("N10 G28 F0 E0 X0 Y0 Z0\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G28 OK");

        parser.reset_current_line();

        let stream = BufferStream::new("N10 G29 F0 E0 X0 Y0 Z0\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G29 OK");

        let stream = BufferStream::new("N11 G29.1 F0 E0 X0 Y0 Z0\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G29.1 OK");

        let stream = BufferStream::new("G29.2 F0 E0 X0 Y0 Z0\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G29.2 OK");

        let stream = BufferStream::new("G31\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("G31 OK");
    }

    #[futures_test::test]
    async fn test_machinery() {
        let mut stream = BufferStream::new("".as_bytes().to_vec().into());
        assert_eq!(stream.next().await, None);
        stream.recovery_check().await;
        assert_eq!(stream.next().await, None);

        let spec = RawGCodeSpec::from('Z', None);

        let _str = format!("{:?}", spec);

        let stream = BufferStream::new("".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect_err("None");
        assert_eq!(parser.get_state(), AsyncParserState::Start(true));
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect_err("EOF");
        assert_eq!(parser.get_state(), AsyncParserState::Start(true));
        let _ = parser.close().await;

        let stream = BufferStream::new("/X0\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect_err("None");

        let stream = BufferStream::new("&0\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect_err("None");

        let stream = BufferStream::new("V0\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect_err("None");
    }

    #[futures_test::test]
    async fn test_ms() {
        let stream = BufferStream::new("M20 F\"X.g\"\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M20 OK");

        let stream = BufferStream::new("M37\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M37 OK");

        let stream = BufferStream::new("M37 S1\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M37 S1 OK");

        #[cfg(feature = "with-hot-bed")]
        {
            let stream = BufferStream::new("M140\n".as_bytes().to_vec().into());
            let mut parser = GCodeLineParser::new(stream);
            let _ = parser.gcode_line();
            parser
                .next_gcode(CommChannel::Internal)
                .await
                .expect("M140 OK");
        }

        #[cfg(feature = "with-hot-end")]
        {
            let stream = BufferStream::new("M104\n".as_bytes().to_vec().into());
            let mut parser = GCodeLineParser::new(stream);
            let _ = parser.gcode_line();
            parser
                .next_gcode(CommChannel::Internal)
                .await
                .expect("M104 OK");

            let stream = BufferStream::new("M109\n".as_bytes().to_vec().into());
            let mut parser = GCodeLineParser::new(stream);
            let _ = parser.gcode_line();
            parser
                .next_gcode(CommChannel::Internal)
                .await
                .expect("M109 OK");
        }

        let stream = BufferStream::new("M220\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M220 OK");

        let stream = BufferStream::new("M220 S1\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M220 S1 OK");

        let stream = BufferStream::new("M110\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M110 OK");

        parser.reset_current_line();
        parser.reset().await;

        let stream = BufferStream::new("M110 N1\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M110 N1 OK");

        let stream = BufferStream::new("M204\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M204 OK");

        let stream = BufferStream::new("M205\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M205 OK");

        let stream = BufferStream::new("M862.1\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M862.1 OK");

        let stream = BufferStream::new("M862.3\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M862.3 OK");

        let stream = BufferStream::new("M900\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M900 OK");

        let stream = BufferStream::new("M907\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        parser
            .next_gcode(CommChannel::Internal)
            .await
            .expect("M907 OK");
    }
}
