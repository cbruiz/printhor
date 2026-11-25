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
use crate::processing::PR;
#[allow(unused)]
use crate::processing::S;
#[cfg(feature = "with-motion")]
use crate::processing::SJ;
#[allow(unused)]
use crate::processing::{DSLT, ESC, PID, SBR, SFZ, SP, SPT, SR, ST};
#[cfg(feature = "with-arcs")]
use crate::processing::ARC;
#[cfg(feature = "with-beziers")]
use crate::processing::BEZIER;
#[cfg(feature = "with-pressure-advance")]
use crate::processing::KT;
#[cfg(feature = "with-input-shaping")]
use crate::processing::DFTXY;
use hwa::CommChannel;

// dyn trait could reduce code size a lot with the penalty of the indirection
pub struct GCodeLineParser<STREAM>
where
    STREAM: async_gcode::ByteStream<Item = Result<u8, async_gcode::Error>>,
{
    /// The parser
    raw_parser: async_gcode::Parser<STREAM, async_gcode::Error>,
    /// The current line in the parser state
    current_line: u32,
    /// The tagged line (NXX) from gcode commands, when provided
    tagged_line_num: Option<u32>,
    /// The last accepted line number (host protocol). Set by `M110` or by every numbered line
    /// accepted in sequence
    last_line_num: Option<u32>,
    /// Whether numbered lines must arrive in sequence (`N<last+1>`). Only for host channels.
    check_line_numbers: bool,
}

impl<STREAM> GCodeLineParser<STREAM>
where
    STREAM: async_gcode::ByteStream<Item = Result<u8, async_gcode::Error>>,
{
    pub fn new(stream: STREAM) -> Self {
        Self {
            raw_parser: async_gcode::Parser::new(stream),
            current_line: 0,
            tagged_line_num: None,
            last_line_num: None,
            check_line_numbers: false,
        }
    }

    /// Enables the host protocol line number check: once a numbered line (or `M110`) has been
    /// accepted, every numbered line must be the next one. Otherwise, the line is rejected with
    /// [GCodeLineParserError::LineNumberMismatch] so that the host resends it.
    pub fn with_line_number_check(mut self) -> Self {
        self.check_line_numbers = true;
        self
    }

    /// The last accepted line number (or the last tagged one when the check is disabled)
    pub fn gcode_line(&self) -> Option<u32> {
        self.last_line_num.or(self.tagged_line_num)
    }

    pub async fn next_gcode(
        &mut self,
        channel: CommChannel,
    ) -> Result<GCodeCmd, GCodeLineParserError> {
        let result = self.next_gcode_unchecked(channel).await;
        if let Ok(cmd) = &result {
            // M110 [N<line>] sets the current line number (Marlin semantics)
            if let GCodeValue::M110(arg) = &cmd.value {
                let n = arg
                    .n
                    .and_then(|v| v.to_i32())
                    // Negative numbers wrap around: `M110 N-1` expects N0 next (as the
                    // `N-1 M110` prefix form)
                    .map(|v| v as u32)
                    .or(cmd.line_tag);
                if n.is_some() {
                    self.last_line_num = n;
                }
            } else if let Some(n) = cmd.line_tag {
                match self.last_line_num {
                    Some(last) if self.check_line_numbers && n != last.wrapping_add(1) => {
                        return Err(GCodeLineParserError::LineNumberMismatch(last));
                    }
                    _ => self.last_line_num = Some(n),
                }
            }
        }
        result
    }

    async fn next_gcode_unchecked(
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
            self.current_line = self.raw_parser.get_current_line();
            match self.raw_parser.next().await {
                None => {
                    hwa::trace!("EOF reading from stream");
                    self.tagged_line_num = tagged_line_num;
                    match current_gcode.take() {
                        None => {
                            match raw_gcode_spec.take() {
                                None => {
                                    return Err(GCodeLineParserError::EOF);
                                } // Empty line. Just ignore
                                Some(rgs) => {
                                    return Err(GCodeLineParserError::GCodeNotImplemented(
                                        self.raw_parser.get_current_line(),
                                        rgs.name(),
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
                                            | ('m', Some((28, 0)))
                                            | ('m', Some((33, 0)))
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
                                                    self.current_line,
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
                                    self.tagged_line_num = tagged_line_num;
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
                                                            self.current_line,
                                                            rgs.name(),
                                                        ),
                                                    );
                                                }
                                            }
                                        }
                                        Some(mut cgv) => {
                                            cgv.order_num = self.current_line;
                                            return Ok(cgv);
                                        }
                                    }
                                }
                                _ => {
                                    return Err(GCodeLineParserError::GCodeNotImplemented(
                                        self.current_line,
                                        GCodeName::from("N/A"),
                                    ));
                                }
                            }
                        }
                        // Not logged here: the consumer of the line reports it (once)
                        Err(error) => {
                            return Err(GCodeLineParserError::from_parser_error(
                                self.current_line,
                                error,
                            ));
                        }
                    }
                }
            }
        }
    }

    /// Reads the next line without executing it (a line to be written to a file, see M28) into
    /// `buf`, and returns the range of its content: without the line number (`N<n>`) and the
    /// checksum (`*<cs>`), which are checked as for any other line (a wrong checksum is a
    /// [GCodeLineParserError::BadChecksum], an unexpected number a
    /// [GCodeLineParserError::LineNumberMismatch]: the host resends the line). Surrounding spaces
    /// are trimmed: an empty range is an empty line. A line longer than `buf` is a
    /// [GCodeLineParserError::ParseError].
    pub async fn next_raw_line(
        &mut self,
        buf: &mut [u8],
    ) -> Result<core::ops::Range<usize>, GCodeLineParserError> {
        self.current_line = self.raw_parser.get_current_line();
        let len = match self.raw_parser.next_raw_line(buf).await {
            None => return Err(GCodeLineParserError::EOF),
            Some(Err(error)) => {
                return Err(GCodeLineParserError::from_parser_error(self.current_line, error));
            }
            Some(Ok((_, true))) => {
                return Err(GCodeLineParserError::ParseError(self.current_line, ParseFailure::TooLong));
            }
            Some(Ok((len, false))) => len,
        };
        let line = &buf[..len];
        let trim = |r: core::ops::Range<usize>| {
            let (mut a, mut b) = (r.start, r.end);
            while a < b && line[a] == b' ' {
                a += 1;
            }
            while b > a && line[b - 1] == b' ' {
                b -= 1;
            }
            a..b
        };
        let mut content = trim(0..len);
        // Checksum: the XOR of every byte before the '*'
        if let Some(star) = line[content.clone()].iter().rposition(|b| *b == b'*') {
            let star = content.start + star;
            let digits = &line[star + 1..content.end];
            let expected = (!digits.is_empty() && digits.iter().all(u8::is_ascii_digit))
                .then(|| digits.iter().fold(0u32, |v, d| v * 10 + u32::from(d - b'0')));
            let sum = line[..star].iter().fold(0u8, |s, b| s ^ b);
            // As the G-code parser: a checksum that is not a number, or does not fit in a byte, is
            // a parse error; otherwise it is a mismatch
            let received = match expected {
                None => {
                    return Err(GCodeLineParserError::ParseError(
                        self.current_line,
                        ParseFailure::BadNumberFormat,
                    ));
                }
                Some(n) => u8::try_from(n).map_err(|_| {
                    GCodeLineParserError::ParseError(self.current_line, ParseFailure::NumberOverflow)
                })?,
            };
            if received != sum {
                return Err(GCodeLineParserError::BadChecksum(self.current_line, sum, received));
            }
            content = trim(content.start..star);
        }
        // Line number
        if line[content.clone()].first().is_some_and(|b| b.eq_ignore_ascii_case(&b'n')) {
            let digits_end = line[content.start + 1..content.end]
                .iter()
                .position(|b| !b.is_ascii_digit())
                .map_or(content.end, |p| content.start + 1 + p);
            if digits_end > content.start + 1 {
                let n = line[content.start + 1..digits_end]
                    .iter()
                    .fold(0u32, |v, d| v.wrapping_mul(10).wrapping_add(u32::from(d - b'0')));
                match self.last_line_num {
                    Some(last) if self.check_line_numbers && n != last.wrapping_add(1) => {
                        return Err(GCodeLineParserError::LineNumberMismatch(last));
                    }
                    _ => self.last_line_num = Some(n),
                }
                self.tagged_line_num = Some(n);
                content = trim(digits_end..content.end);
            }
        }
        Ok(content)
    }

    pub async fn reset(&mut self) {
        hwa::trace!("AsyncGcodeParser reset");
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

    /// Consumes the parser and returns its input stream, so that the stream can be closed (an SD
    /// card file keeps its entries open until its stream is closed).
    pub fn into_stream(self) -> STREAM {
        self.raw_parser.into_inner()
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
        #[cfg(feature = "with-arcs")]
        ('g', Some((2, 0))) => Some(GCodeValue::G2(ARC::new())),
        #[cfg(feature = "with-arcs")]
        ('g', Some((3, 0))) => Some(GCodeValue::G3(ARC::new())),
        #[cfg(feature = "with-beziers")]
        ('g', Some((5, 0))) => Some(GCodeValue::G5(BEZIER::new())),
        #[cfg(feature = "with-motion")]
        ('g', Some((4, 0))) => Some(GCodeValue::G4(SP::new())),
        #[cfg(feature = "with-motion")]
        ('g', Some((10, 0))) => Some(GCodeValue::G10),
        #[cfg(feature = "with-motion")]
        ('g', Some((11, 0))) => Some(GCodeValue::G11),
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
        #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
        ('g', Some((30, 0))) => Some(GCodeValue::G30(FXYZ::new())),
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
        ('m', Some((3, 0))) => Some(GCodeValue::M3(S::new())),
        ('m', Some((4, 0))) => Some(GCodeValue::M4(S::new())),
        ('m', Some((5, 0))) => Some(GCodeValue::M5),
        ('m', Some((7, 0))) => Some(GCodeValue::M7),
        ('m', Some((8, 0))) => Some(GCodeValue::M8),
        ('m', Some((9, 0))) => Some(GCodeValue::M9),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((20, 0))) => Some(GCodeValue::M20(None, false)),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((21, 0))) => Some(GCodeValue::M21),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((33, 0))) => Some(GCodeValue::M33(None)),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((22, 0))) => Some(GCodeValue::M22),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((23, 0))) => Some(GCodeValue::M23(None)),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((28, 0))) => Some(GCodeValue::M28(None)),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((29, 0))) => Some(GCodeValue::M29),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((24, 0))) => Some(GCodeValue::M24),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((25, 0))) => Some(GCodeValue::M25),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        ('m', Some((27, 0))) => Some(GCodeValue::M27(S::new())),
        #[cfg(feature = "with-sd-card")]
        ('m', Some((524, 0))) => Some(GCodeValue::M524),
        ('m', Some((37, 0))) => Some(GCodeValue::M37(S::new())),
        ('m', Some((73, 0))) => Some(GCodeValue::M73(PR::new())),
        ('m', Some((79, 0))) => Some(GCodeValue::M79),
        #[cfg(feature = "with-ps-on")]
        ('m', Some((80, 0))) => Some(GCodeValue::M80),
        #[cfg(feature = "with-ps-on")]
        ('m', Some((81, 0))) => Some(GCodeValue::M81),
        #[cfg(feature = "with-motion")]
        ('m', Some((18, 0))) => Some(GCodeValue::M84),
        #[cfg(feature = "with-motion")]
        ('m', Some((82, 0))) => Some(GCodeValue::M82),
        #[cfg(feature = "with-motion")]
        ('m', Some((83, 0))) => Some(GCodeValue::M83),
        #[cfg(feature = "with-motion")]
        ('m', Some((84, 0))) => Some(GCodeValue::M84),
        #[cfg(feature = "with-motion")]
        ('m', Some((92, 0))) => Some(GCodeValue::M92(EXYZ::new())),
        ('m', Some((100, 0))) => Some(GCodeValue::M100),
        #[cfg(feature = "with-hot-end")]
        ('m', Some((104, 0))) => Some(GCodeValue::M104(S::new())),
        ('m', Some((105, 0))) => Some(GCodeValue::M105),
        #[cfg(any(feature = "with-fan-layer", feature = "with-fan-extra-1"))]
        ('m', Some((106, 0))) => Some(GCodeValue::M106(SP::new())),
        #[cfg(any(feature = "with-fan-layer", feature = "with-fan-extra-1"))]
        ('m', Some((107, 0))) => Some(GCodeValue::M107),
        #[cfg(feature = "with-hot-end")]
        ('m', Some((109, 0))) => Some(GCodeValue::M109(SR::new())),
        ('m', Some((110, 0))) => Some(GCodeValue::M110(N::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((114, 0))) => Some(GCodeValue::M114),
        ('m', Some((112, 0))) => Some(GCodeValue::M112),
        ('m', Some((115, 0))) => Some(GCodeValue::M115),
        ('m', Some((117, 0))) => Some(GCodeValue::M117(None)),
        ('m', Some((118, 0))) => Some(GCodeValue::M118(None)),
        ('m', Some((119, 0))) => Some(GCodeValue::M119),
        #[cfg(feature = "with-probe")]
        ('m', Some((280, 0))) => Some(GCodeValue::M280(SP::new())),
        #[cfg(feature = "with-probe")]
        ('m', Some((401, 0))) => Some(GCodeValue::M401),
        #[cfg(feature = "with-probe")]
        ('m', Some((402, 0))) => Some(GCodeValue::M402),
        #[cfg(feature = "with-hot-bed")]
        ('m', Some((140, 0))) => Some(GCodeValue::M140(S::new())),
        #[cfg(feature = "with-hot-bed")]
        ('m', Some((190, 0))) => Some(GCodeValue::M190(SR::new())),
        #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
        ('m', Some((200, 0))) => Some(GCodeValue::M200(DSLT::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((201, 0))) => Some(GCodeValue::M201(EXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((203, 0))) => Some(GCodeValue::M203(EXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((204, 0))) => Some(GCodeValue::M204(SPT::new())),
        ('m', Some((205, 0))) => Some(GCodeValue::M205(EXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((206, 0))) => Some(GCodeValue::M206(EXYZ::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((851, 0))) => Some(GCodeValue::M851(EXYZ::new())),
        #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
        ('m', Some((207, 0))) => Some(GCodeValue::M207(SFZ::new())),
        #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
        ('m', Some((208, 0))) => Some(GCodeValue::M208(SFZ::new())),
        ('m', Some((211, 0))) => Some(GCodeValue::M211(S::new())),
        ('m', Some((220, 0))) => Some(GCodeValue::M220(SBR::new())),
        ('m', Some((221, 0))) => Some(GCodeValue::M221(ST::new())),
        #[cfg(feature = "with-hot-end")]
        ('m', Some((301, 0))) => Some(GCodeValue::M301(PID::new())),
        #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
        ('m', Some((302, 0))) => Some(GCodeValue::M302(SP::new())),
        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        ('m', Some((303, 0))) => Some(GCodeValue::M303(ESC::new())),
        #[cfg(feature = "with-hot-bed")]
        ('m', Some((304, 0))) => Some(GCodeValue::M304(PID::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((400, 0))) => Some(GCodeValue::M400),
        #[cfg(feature = "with-motion")]
        ('m', Some((410, 0))) => Some(GCodeValue::M410),
        #[cfg(feature = "with-eeprom")]
        ('m', Some((500, 0))) => Some(GCodeValue::M500),
        #[cfg(feature = "with-eeprom")]
        ('m', Some((501, 0))) => Some(GCodeValue::M501),
        ('m', Some((502, 0))) => Some(GCodeValue::M502),
        ('m', Some((503, 0))) => Some(GCodeValue::M503(S::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((800, 0))) => Some(GCodeValue::M800(SJ::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((801, 0))) => Some(GCodeValue::M801),
        #[cfg(feature = "with-motion")]
        ('m', Some((8621, 1))) => Some(GCodeValue::M862_1),
        #[cfg(feature = "with-motion")]
        ('m', Some((8623, 1))) => Some(GCodeValue::M862_3),
        #[cfg(all(feature = "with-motion", not(feature = "with-pressure-advance")))]
        ('m', Some((900, 0))) => Some(GCodeValue::M900),
        #[cfg(all(feature = "with-motion", feature = "with-pressure-advance"))]
        ('m', Some((900, 0))) => Some(GCodeValue::M900(KT::new())),
        #[cfg(all(feature = "with-motion", feature = "with-input-shaping"))]
        ('m', Some((593, 0))) => Some(GCodeValue::M593(DFTXY::new())),
        #[cfg(feature = "with-motion")]
        ('m', Some((907, 0))) => Some(GCodeValue::M907),
        ('m', Some((999, 0))) => Some(GCodeValue::M999),
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
        #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
        #[allow(unused_variables)] // Without X/Y axes: every word is ignored
        GCodeValue::G30(coord) => match (ch, frx) {
            #[cfg(feature = "with-x-axis")]
            ('x', Some(val)) => {
                coord.x.replace(helpers::to_fixed(val));
            }
            #[cfg(feature = "with-y-axis")]
            ('y', Some(val)) => {
                coord.y.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
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
        GCodeValue::G1(coord) => update_efsxyz(coord, ch, frx),
        #[cfg(feature = "with-arcs")]
        GCodeValue::G2(arc) | GCodeValue::G3(arc) => match ch {
            'i' => set_word(&mut arc.i, 'i', ch, frx),
            'j' => set_word(&mut arc.j, 'j', ch, frx),
            'r' => set_word(&mut arc.r, 'r', ch, frx),
            _ => update_efsxyz(&mut arc.coords, ch, frx),
        },
        #[cfg(feature = "with-beziers")]
        GCodeValue::G5(bz) => match ch {
            'i' => set_word(&mut bz.i, 'i', ch, frx),
            'j' => set_word(&mut bz.j, 'j', ch, frx),
            'p' => set_word(&mut bz.p, 'p', ch, frx),
            'q' => set_word(&mut bz.q, 'q', ch, frx),
            _ => update_efsxyz(&mut bz.coords, ch, frx),
        },
        #[cfg(feature = "with-motion")]
        GCodeValue::G4(param) => match (ch, frx) {
            ('s', Some(val)) => {
                param.s.replace(helpers::to_fixed(val).abs());
            }
            ('p', Some(val)) => {
                param.p.replace(helpers::to_fixed(val).abs());
            }
            _ => {}
        },
        #[cfg(feature = "with-motion")]
        GCodeValue::M92(coord)
        | GCodeValue::M201(coord)
        | GCodeValue::M203(coord)
        | GCodeValue::M205(coord)
        | GCodeValue::M206(coord)
        | GCodeValue::M851(coord) => {
            update_exyz(coord, ch, frx)
        }
        #[cfg(feature = "with-motion")]
        GCodeValue::M204(p) => {
            set_word(&mut p.s, 's', ch, frx);
            set_word(&mut p.p, 'p', ch, frx);
            set_word(&mut p.t, 't', ch, frx);
        }
        #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
        GCodeValue::M200(p) => {
            set_word(&mut p.d, 'd', ch, frx);
            set_word(&mut p.s, 's', ch, frx);
            set_word(&mut p.l, 'l', ch, frx);
            set_word(&mut p.t, 't', ch, frx);
        }
        #[cfg(all(feature = "with-motion", feature = "with-pressure-advance"))]
        GCodeValue::M900(p) => {
            set_word(&mut p.k, 'k', ch, frx);
            set_word(&mut p.t, 't', ch, frx);
        }
        #[cfg(all(feature = "with-motion", feature = "with-input-shaping"))]
        GCodeValue::M593(p) => {
            set_word(&mut p.d, 'd', ch, frx);
            set_word(&mut p.f, 'f', ch, frx);
            set_word(&mut p.t, 't', ch, frx);
            // The axes are flags (`M593 X F40`), also with a value (ignored, as Marlin)
            match ch {
                'x' => p.x = Some(hwa::math::ONE),
                'y' => p.y = Some(hwa::math::ONE),
                _ => {}
            }
        }
        #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
        GCodeValue::M207(p) | GCodeValue::M208(p) => {
            set_word(&mut p.s, 's', ch, frx);
            set_word(&mut p.f, 'f', ch, frx);
            set_word(&mut p.z, 'z', ch, frx);
        }
        #[cfg(feature = "with-probe")]
        GCodeValue::M280(p) => {
            set_word(&mut p.s, 's', ch, frx);
            set_word(&mut p.p, 'p', ch, frx);
        }
        #[cfg(any(feature = "with-fan-layer", feature = "with-fan-extra-1"))]
        GCodeValue::M106(p) => {
            set_word(&mut p.s, 's', ch, frx);
            set_word(&mut p.p, 'p', ch, frx);
        }
        GCodeValue::M3(p) | GCodeValue::M4(p) => set_word(&mut p.s, 's', ch, frx),
        #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
        GCodeValue::M302(p) => {
            set_word(&mut p.s, 's', ch, frx);
            set_word(&mut p.p, 'p', ch, frx);
        }
        #[cfg(feature = "with-hot-end")]
        GCodeValue::M301(p) => {
            set_word(&mut p.p, 'p', ch, frx);
            set_word(&mut p.i, 'i', ch, frx);
            set_word(&mut p.d, 'd', ch, frx);
        }
        #[cfg(feature = "with-hot-bed")]
        GCodeValue::M304(p) => {
            set_word(&mut p.p, 'p', ch, frx);
            set_word(&mut p.i, 'i', ch, frx);
            set_word(&mut p.d, 'd', ch, frx);
        }
        GCodeValue::M73(p) => {
            set_word(&mut p.p, 'p', ch, frx);
            set_word(&mut p.r, 'r', ch, frx);
        }
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        GCodeValue::M27(p) => set_word(&mut p.s, 's', ch, frx),
        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        GCodeValue::M303(p) => {
            set_word(&mut p.e, 'e', ch, frx);
            set_word(&mut p.s, 's', ch, frx);
            set_word(&mut p.c, 'c', ch, frx);
            // A flag: `U` alone is `U1`
            if ch == 'u' {
                p.u.replace(frx.map_or(hwa::math::ONE, helpers::to_fixed));
            }
        }
        // The axes to home: their words are flags (`G28 X Y`), also with a value (ignored, as Marlin)
        #[cfg(feature = "with-motion")]
        #[allow(unused_variables)] // No axes enabled: every word is ignored
        GCodeValue::G28(coord) => match (ch, frx) {
            #[cfg(feature = "with-e-axis")]
            ('e', val) => {
                coord.e.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            //
            #[cfg(feature = "with-x-axis")]
            ('x', val) => {
                coord.x.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-y-axis")]
            ('y', val) => {
                coord.y.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-z-axis")]
            ('z', val) => {
                coord.z.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            //
            #[cfg(feature = "with-a-axis")]
            ('a', val) => {
                coord.a.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-b-axis")]
            ('b', val) => {
                coord.b.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-c-axis")]
            ('c', val) => {
                coord.c.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-i-axis")]
            ('i', val) => {
                coord.i.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-j-axis")]
            ('j', val) => {
                coord.j.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-k-axis")]
            ('k', val) => {
                coord.k.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-u-axis")]
            ('u', val) => {
                coord.u.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-v-axis")]
            ('v', val) => {
                coord.v.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            #[cfg(feature = "with-w-axis")]
            ('w', val) => {
                coord.w.replace(val.map_or(hwa::math::ZERO, helpers::to_fixed));
            }
            _ => {}
        },
        #[cfg(feature = "with-motion")]
        #[allow(unused_variables)] // No axes enabled: every word is ignored
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
        GCodeValue::M20(path, long_names) => {
            if ch == 'f' {
                if let async_gcode::RealValue::Literal(async_gcode::Literal::String(mstr)) = fv {
                    path.replace(mstr);
                }
            } else if ch == 'l' {
                // A flag: `L` alone (Marlin's `M20 L`)
                *long_names = frx.is_none_or(|v| v.0 != 0);
            }
        }
        #[cfg(feature = "with-sd-card")]
        GCodeValue::M33(path) => {
            if let async_gcode::RealValue::Literal(async_gcode::Literal::String(mstr)) = fv {
                path.replace(mstr);
            }
        }
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        GCodeValue::M23(file) => {
            if let async_gcode::RealValue::Literal(async_gcode::Literal::String(mstr)) = fv {
                file.replace(mstr);
            }
        }
        #[cfg(feature = "with-sd-card")]
        GCodeValue::M28(file) => {
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
        GCodeValue::M109(coord) => {
            set_word(&mut coord.s, 's', ch, frx);
            set_word(&mut coord.r, 'r', ch, frx);
        }
        #[cfg(feature = "with-hot-bed")]
        GCodeValue::M190(coord) => {
            set_word(&mut coord.s, 's', ch, frx);
            set_word(&mut coord.r, 'r', ch, frx);
        }
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
        GCodeValue::M211(coord) => match (ch, frx) {
            ('s', Some(val)) => {
                coord.s.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        GCodeValue::M220(args) => match (ch, frx) {
            ('s', Some(val)) => {
                args.s.replace(helpers::to_fixed(val));
            }
            // Flags: `B`/`R` alone (no value) are the usual form
            ('b', _) => {
                args.b.replace(hwa::math::ONE);
            }
            ('r', _) => {
                args.r.replace(hwa::math::ONE);
            }
            _ => {}
        },
        GCodeValue::M221(args) => match (ch, frx) {
            ('s', Some(val)) => {
                args.s.replace(helpers::to_fixed(val));
            }
            ('t', Some(val)) => {
                args.t.replace(helpers::to_fixed(val));
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
        #[cfg(feature = "with-motion")]
        GCodeValue::M800(param) => match (ch, frx) {
            ('s', Some(val)) => {
                param.s.replace(helpers::to_fixed(val));
            }
            ('j', Some(val)) => {
                param.j.replace(helpers::to_fixed(val));
            }
            _ => {}
        },
        _ => {}
    }
}

/// Updates the words of a linear move (shared by G1 and G2/G3)
#[cfg(feature = "with-motion")]
fn update_efsxyz(coord: &mut EFSXYZ, ch: char, frx: Option<(i32, u8)>) {
    match (ch, frx) {
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
    }
}

/// Updates the per-axis words of an [EXYZ] argument
#[allow(unused)]
fn update_exyz(coord: &mut EXYZ, ch: char, frx: Option<(i32, u8)>) {
    match (ch, frx) {
        #[cfg(feature = "with-e-axis")]
        ('e', Some(val)) => {
            coord.e.replace(helpers::to_fixed(val));
        }
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
    }
}

/// Assigns `val` to `slot` when `ch` matches `letter`
#[allow(unused)]
#[inline]
fn set_word(slot: &mut Option<hwa::math::Real>, letter: char, ch: char, frx: Option<(i32, u8)>) {
    if let (true, Some(val)) = (ch == letter, frx) {
        slot.replace(helpers::to_fixed(val));
    }
}

/// Name of an unsupported G-code (e.g. `M9999`), for the error reports
pub type GCodeName = super::LineBuf<24>;

/// Why a line could not be parsed (reported by the consumer of the line, with its line number)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParseFailure {
    /// A byte that is not valid where it was found (e.g. a second `.` in `X1.2.3`)
    UnexpectedByte(u8),
    /// A number too big to be represented
    NumberOverflow,
    /// A malformed number (e.g. without digits)
    BadNumberFormat,
    /// A text or a line longer than it can be
    TooLong,
    /// A text that is not valid UTF-8
    InvalidUtf8,
    /// The input stream failed, or another reason
    Other,
}

impl core::fmt::Display for ParseFailure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ParseFailure::UnexpectedByte(b) if b.is_ascii_graphic() => {
                write!(f, "unexpected '{}'", char::from(*b))
            }
            ParseFailure::UnexpectedByte(b) => write!(f, "unexpected byte {}", b),
            ParseFailure::NumberOverflow => f.write_str("number overflow"),
            ParseFailure::BadNumberFormat => f.write_str("bad number format"),
            ParseFailure::TooLong => f.write_str("too long"),
            ParseFailure::InvalidUtf8 => f.write_str("invalid UTF-8"),
            ParseFailure::Other => f.write_str("unreadable"),
        }
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for ParseFailure {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ParseFailure::UnexpectedByte(b) => defmt::write!(fmt, "unexpected byte {}", b),
            ParseFailure::NumberOverflow => defmt::write!(fmt, "number overflow"),
            ParseFailure::BadNumberFormat => defmt::write!(fmt, "bad number format"),
            ParseFailure::TooLong => defmt::write!(fmt, "too long"),
            ParseFailure::InvalidUtf8 => defmt::write!(fmt, "invalid UTF-8"),
            ParseFailure::Other => defmt::write!(fmt, "unreadable"),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(test, derive(PartialEq))]
pub enum GCodeLineParserError {
    /// The line could not be parsed (line, why)
    ParseError(u32, ParseFailure),
    /// The checksum of the line (host protocol) is wrong (line, checksum computed from the line,
    /// checksum received). The host has to resend it
    BadChecksum(u32, u8, u8),
    /// The Gcode was not implemented. Holds its name (no heap: a G/M code fits in 24 bytes)
    GCodeNotImplemented(u32, GCodeName),
    /// EOF Reading from parser
    EOF,
    /// Unexpected fatal error
    #[allow(unused)]
    FatalError,
    /// A numbered line is not the expected one (host protocol). Holds the last accepted line
    LineNumberMismatch(u32),
}

impl GCodeLineParserError {
    /// The error of the line `line` the G-code parser reported
    fn from_parser_error(line: u32, error: async_gcode::Error) -> Self {
        let failure = match error {
            async_gcode::Error::BadChecksum { computed, received } => {
                return GCodeLineParserError::BadChecksum(line, computed, received);
            }
            async_gcode::Error::UnexpectedByte(b) => ParseFailure::UnexpectedByte(b),
            async_gcode::Error::NumberOverflow => ParseFailure::NumberOverflow,
            async_gcode::Error::BadNumberFormat => ParseFailure::BadNumberFormat,
            async_gcode::Error::TextTooLong => ParseFailure::TooLong,
            async_gcode::Error::InvalidUTF8String => ParseFailure::InvalidUtf8,
            #[allow(unreachable_patterns)]
            _ => ParseFailure::Other,
        };
        GCodeLineParserError::ParseError(line, failure)
    }
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
            GCodeLineParserError::ParseError(ln, failure) => {
                defmt::write!(fmt, "ParseError(line={}, {})", ln, failure);
            }
            GCodeLineParserError::BadChecksum(ln, computed, received) => {
                defmt::write!(fmt, "BadChecksum(line={}, computed={}, received={})", ln, computed, received);
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
            GCodeLineParserError::LineNumberMismatch(ln) => {
                defmt::write!(fmt, "LineNumberMismatch(last={})", ln);
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

    /// The name of the code, upper case (e.g. `G1`, `M862.2`).
    ///
    /// Written without `core::fmt` nor `char::to_uppercase` (whose Unicode table alone takes ~2 KB
    /// of flash): the letter of a word is always ASCII (the parser only accepts `a`..=`z`).
    pub fn name(&self) -> GCodeName {
        let mut name = GCodeName::new();
        let mut letter = [0u8; 4];
        name.push_str(self.code.to_ascii_uppercase().encode_utf8(&mut letter));
        if let Some(spec) = self.spec {
            name.push_i32(spec);
            if let Some(sub) = self.sub {
                name.push_str(".");
                name.push_i32(sub);
            }
        }
        name
    }
}

impl core::fmt::Debug for RawGCodeSpec {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.name().as_str())
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

/// Fallback for a non patched async-gcode (plain `f64` numbers): fixed point with
/// [F64_ADAPTOR_SCALE] decimals. Never panics: out of range values saturate (`as` cast)
const F64_ADAPTOR_SCALE: u8 = 4;
/// 10 ^ [F64_ADAPTOR_SCALE] (no `powi` in `no_std`)
const F64_ADAPTOR_FACTOR: f64 = 10_000.0;

impl FixedAdaptor for f64 {
    #[inline(always)]
    fn integer_part(&self) -> i32 {
        let scaled = *self * F64_ADAPTOR_FACTOR;
        (if scaled < 0.0 { scaled - 0.5 } else { scaled + 0.5 }) as i32
    }

    #[inline(always)]
    fn scale(&self) -> u8 {
        F64_ADAPTOR_SCALE
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

    /// Chunked stream with the shape of `SDCardStream`: bytes come from a 512-byte buffer that
    /// a non-inlined call refills.
    #[cfg(feature = "perf-bench")]
    struct ChunkStream {
        data: Vec<u8>,
        pos: usize,
        buf: [u8; 512],
        len: usize,
        idx: usize,
    }

    #[cfg(feature = "perf-bench")]
    impl ChunkStream {
        #[inline(never)]
        fn refill(&mut self) -> usize {
            let n = (self.data.len() - self.pos).min(self.buf.len());
            self.buf[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
            self.pos += n;
            self.len = n;
            self.idx = 0;
            n
        }
    }

    #[cfg(feature = "perf-bench")]
    impl async_gcode::ByteStream for ChunkStream {
        type Item = Result<u8, async_gcode::Error>;

        async fn next(&mut self) -> Option<Self::Item> {
            if self.idx >= self.len && self.refill() == 0 {
                return None;
            }
            let b = self.buf[self.idx];
            self.idx += 1;
            Some(Ok(b))
        }

        async fn recovery_check(&mut self) {}
    }

    /// Host benchmark of the whole G-code line path (`async-gcode` plus [GCodeLineParser]).
    /// Compiled only with the `perf-bench` feature. Point `PRINTHOR_BENCH_GCODE` to a G-code file
    /// (for instance `benchy.g` from `data/sdcard.img`) and run:
    ///
    /// ```text
    /// PRINTHOR_BENCH_GCODE=benchy.g cargo test --release --features integration-test,perf-bench --bin printhor perf_bench_gcode_line_parser -- --nocapture
    /// ```
    #[cfg(feature = "perf-bench")]
    #[futures_test::test]
    async fn perf_bench_gcode_line_parser() {
        let Ok(path) = std::env::var("PRINTHOR_BENCH_GCODE") else {
            println!("PERF gcode_line_parser skipped: PRINTHOR_BENCH_GCODE is not set");
            return;
        };
        let data = std::fs::read(&path).expect("readable G-code file");
        let mut times = Vec::new();
        let (mut commands, mut errors) = (0u32, 0u32);
        for _ in 0..7 {
            let stream = ChunkStream {
                data: data.clone(),
                pos: 0,
                buf: [0; 512],
                len: 0,
                idx: 0,
            };
            let mut parser = GCodeLineParser::new(stream);
            (commands, errors) = (0, 0);
            let t0 = std::time::Instant::now();
            loop {
                match parser.next_gcode(CommChannel::Internal).await {
                    Ok(cmd) => {
                        commands += 1;
                        std::hint::black_box(cmd);
                    }
                    Err(GCodeLineParserError::EOF) => break,
                    Err(_) => errors += 1,
                }
            }
            times.push(t0.elapsed().as_secs_f64());
        }
        times.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = times[times.len() / 2];
        println!(
            "PERF gcode_line_parser: {} commands, {} errors, {} bytes, median {:.2} ms ({:.2} ns/byte, {:.1} ns/command)",
            commands,
            errors,
            data.len(),
            median * 1e3,
            median * 1e9 / data.len() as f64,
            median * 1e9 / commands as f64
        );
    }

    #[cfg(feature = "with-beziers")]
    #[futures_test::test]
    async fn test_g5() {
        let stream = BufferStream::new("G5 I1 J2 P-3 Q4 X10 Y20 E0.5 F600\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let cmd = parser.next_gcode(CommChannel::Internal).await.expect("G5 OK");
        match cmd.value {
            GCodeValue::G5(bz) => {
                let r = |v: f32| Some(hwa::math::Real::from_f32(v));
                assert_eq!((bz.i, bz.j, bz.p, bz.q), (r(1.0), r(2.0), r(-3.0), r(4.0)));
                assert_eq!((bz.coords.x, bz.coords.y), (r(10.0), r(20.0)));
                assert_eq!(bz.coords.f, r(600.0));
            }
            other => panic!("unexpected {:?}", other),
        }
    }

    #[futures_test::test]
    async fn test_m23_with_newline() {
        let stream = BufferStream::new("M23 BENCHY.G\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(
                0,
                None,
                GCodeValue::M23(Some("BENCHY.G".into()))
            ))
        )
    }

    #[futures_test::test]
    async fn test_m23_with_eof_condition() {
        let stream = BufferStream::new("M23 BENCHY.G".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(
                0,
                None,
                GCodeValue::M23(Some("BENCHY.G".into()))
            ))
        )
    }

    #[futures_test::test]
    async fn test_m23_with_trailin_comments() {
        let stream = BufferStream::new("M23 BENCHY.G".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(
                0,
                None,
                GCodeValue::M23(Some("BENCHY.G".into()))
            ))
        )
    }

    #[futures_test::test]
    async fn test_m117() {
        let stream = BufferStream::new("M117 Hello World!".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(
                0,
                None,
                GCodeValue::M117(Some("Hello World!".into()))
            ))
        )
    }

    #[cfg(feature = "with-motion")]
    #[futures_test::test]
    async fn test_m800() {
        let stream = BufferStream::new("M800 S0 J50000\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let mut expected = SJ::new();
        expected.s = Some(hwa::math::ZERO);
        expected.j = Some(hwa::math::Real::from_lit(50000, 0));
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(0, None, GCodeValue::M800(expected)))
        )
    }

    #[futures_test::test]
    async fn test_m118() {
        let stream = BufferStream::new("M118 Hello World!".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(
                0,
                None,
                GCodeValue::M118(Some("Hello World!".into()))
            ))
        )
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

    /// [RawGCodeSpec::name] writes what its former `core::fmt` implementation wrote
    /// (`to_uppercase` of the letter, then `{}` of the numbers), for every word letter
    #[test]
    fn test_raw_gcode_spec_name() {
        let former = |code: char, spec: Option<(i32, u8)>| -> String {
            let mut s = format!("{}", code.to_uppercase());
            if let Some((num, scale)) = spec {
                if scale == 0 {
                    s.push_str(&format!("{}", num));
                } else {
                    let sc = 10_i32.pow(scale as u32);
                    s.push_str(&format!("{}.{}", num / sc, num % sc));
                }
            }
            s
        };
        let specs: [Option<(i32, u8)>; 10] = [
            None,
            Some((0, 0)),
            Some((1, 0)),
            Some((862_2, 1)),
            Some((38_02, 2)),
            Some((29_10, 2)),
            Some((-5, 0)),
            Some((-15, 1)),
            Some((i32::MAX, 0)),
            Some((i32::MIN, 0)),
        ];
        for code in 'a'..='z' {
            for spec in specs {
                let expected = former(code, spec);
                let raw = RawGCodeSpec::from(code, spec);
                assert_eq!(raw.name().as_str(), expected, "{:?} {:?}", code, spec);
                assert_eq!(format!("{:?}", raw), expected);
            }
        }
        assert_eq!(RawGCodeSpec::from('m', Some((862_2, 1))).name().as_str(), "M862.2");
        // The longest name fits (a [GCodeName] truncates)
        assert_eq!(
            RawGCodeSpec::from('g', Some((i32::MIN, 0))).name().as_str(),
            "G-2147483648"
        );
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

    /// The words of the G-codes added for Marlin parity: flags without a value and text paths
    #[futures_test::test]
    async fn flags_and_paths_of_homing_volumetric_autotune_and_sd() {
        async fn parse(line: &str) -> GCodeValue {
            let stream = BufferStream::new(line.as_bytes().to_vec().into());
            let mut parser = GCodeLineParser::new(stream);
            parser.next_gcode(CommChannel::Internal).await.expect(line).value
        }
        let one = Some(hwa::math::ONE);
        let zero = Some(hwa::math::ZERO);
        // G28: the axes are flags, with or without a value
        #[cfg(all(feature = "with-motion", feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis"))]
        {
            match parse("G28 X Y\n").await {
                GCodeValue::G28(axes) => assert!(axes.x == zero && axes.y == zero && axes.z.is_none()),
                other => panic!("{:?}", other),
            }
            match parse("G28 Z5\n").await {
                GCodeValue::G28(axes) => assert!(axes.x.is_none() && axes.z.is_some()),
                other => panic!("{:?}", other),
            }
        }
        #[cfg(all(feature = "with-motion", feature = "with-input-shaping"))]
        {
            match parse("M593 X F37.5 D0.1 T2\n").await {
                GCodeValue::M593(p) => assert!(
                    p.x == one && p.y.is_none() && p.f == Some(hwa::math::Real::new(375, 1))
                        && p.d == Some(hwa::math::Real::new(1, 1)) && p.t == Some(hwa::math::TWO)
                ),
                other => panic!("{:?}", other),
            }
            match parse("M593 Y0 X\n").await {
                GCodeValue::M593(p) => assert!(p.x == one && p.y == one && p.f.is_none()),
                other => panic!("{:?}", other),
            }
        }
        #[cfg(all(feature = "with-motion", feature = "with-pressure-advance"))]
        {
            match parse("M900 K0.045 T0\n").await {
                GCodeValue::M900(p) => assert!(p.k == Some(hwa::math::Real::new(45, 3)) && p.t == zero),
                other => panic!("{:?}", other),
            }
            match parse("M900\n").await {
                GCodeValue::M900(p) => assert!(p.k.is_none() && p.t.is_none()),
                other => panic!("{:?}", other),
            }
        }
        #[cfg(all(feature = "with-motion", feature = "with-e-axis"))]
        match parse("M200 D2.85 S1 L5 T0\n").await {
            GCodeValue::M200(p) => {
                assert_eq!(p.d, Some(hwa::math::Real::new(285, 2)));
                assert!(p.s == one && p.l == Some(hwa::math::FIVE) && p.t == zero);
            }
            other => panic!("{:?}", other),
        }
        #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
        {
            match parse("M303 E-1 S60 C8 U\n").await {
                GCodeValue::M303(p) => assert!(p.u == one && p.c == Some(hwa::math::EIGHT)),
                other => panic!("{:?}", other),
            }
            match parse("M303 U0\n").await {
                GCodeValue::M303(p) => assert!(p.u == zero),
                other => panic!("{:?}", other),
            }
        }
        #[cfg(feature = "with-sd-card")]
        {
            assert_eq!(parse("M20 L\n").await, GCodeValue::M20(None, true));
            assert_eq!(parse("M20 L0\n").await, GCodeValue::M20(None, false));
            assert_eq!(parse("M20\n").await, GCodeValue::M20(None, false));
            assert_eq!(
                parse("M33 /Long Folder Name/AREALL~1.GCO\n").await,
                GCodeValue::M33(Some("/Long Folder Name/AREALL~1.GCO".into()))
            );
        }
        let _ = (one, zero);
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

    #[futures_test::test]
    async fn test_line_number_sequence() {
        let stream = BufferStream::new(
            "N7 M114*32\nN9 M114*46\nN8 M114*47\nN10 M110 N3*79\nN4 M105*35\n".as_bytes().to_vec().into(),
        );
        let mut parser = GCodeLineParser::new(stream).with_line_number_check();
        let r = parser.next_gcode(CommChannel::Internal).await;
        assert!(r.is_ok(), "N7: {:?}", r.err().map(|e| alloc::format!("{:?}", e)));
        assert_eq!(parser.gcode_line(), Some(7));
        assert!(matches!(
            parser.next_gcode(CommChannel::Internal).await,
            Err(GCodeLineParserError::LineNumberMismatch(7))
        ));
        assert!(parser.next_gcode(CommChannel::Internal).await.is_ok());
        assert!(parser.next_gcode(CommChannel::Internal).await.is_ok());
        assert_eq!(parser.gcode_line(), Some(3));
        assert!(parser.next_gcode(CommChannel::Internal).await.is_ok());
        assert_eq!(parser.gcode_line(), Some(4));
    }

    /// Pronterface starts every print with `N-1 M110`: the next line expected is N0, whatever the
    /// former sequence was
    #[futures_test::test]
    async fn test_line_number_reset_to_minus_one() {
        let stream = BufferStream::new(
            "N5 M105*34\nN-1 M110*15\nN0 M80*27\nN1 G21*27\nM110 N-1\nN0 M80*27\n"
                .as_bytes()
                .to_vec()
                .into(),
        );
        let mut parser = GCodeLineParser::new(stream).with_line_number_check();
        for expected in [Some(5), Some(u32::MAX), Some(0), Some(1), Some(u32::MAX), Some(0)] {
            let r = parser.next_gcode(CommChannel::Internal).await;
            assert!(r.is_ok(), "{:?}: {:?}", expected, r.err().map(|e| alloc::format!("{:?}", e)));
            assert_eq!(parser.gcode_line(), expected);
        }
    }

    #[futures_test::test]
    async fn test_g_variants() {
        let stream = BufferStream::new("G\nG0\nG1\nG2\nG4\nG10\nG17\nG21\nG28\nG29\nG29.1\nG29.2\nG31\nG32\nG80\nG90\nG91\nG92\nG94\n".as_bytes().to_vec().into());

        /*
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
            #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
            ('m', Some((27, 0))) => Some(GCodeValue::M27(S::new())),
            ('m', Some((37, 0))) => Some(GCodeValue::M37(S::new())),
            ('m', Some((73, 0))) => Some(GCodeValue::M73(PR::new())),
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
            ('m', Some((220, 0))) => Some(GCodeValue::M220(SBR::new())),
            ('m', Some((221, 0))) => Some(GCodeValue::M221(ST::new())),
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
        */

        let mut parser = GCodeLineParser::new(stream);

        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(0, None, GCodeValue::G))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(1, None, GCodeValue::G0(FXYZ::new())))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(2, None, GCodeValue::G1(EFSXYZ::new())))
        );
        #[cfg(feature = "with-arcs")]
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(3, None, GCodeValue::G2(ARC::new())))
        );
        // Without arc support G2 is not a known G-code
        #[cfg(not(feature = "with-arcs"))]
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Err(GCodeLineParserError::GCodeNotImplemented(3, "G2".into()))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(4, None, GCodeValue::G4(SP::new())))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(5, None, GCodeValue::G10))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(6, None, GCodeValue::G17))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(7, None, GCodeValue::G21))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(8, None, GCodeValue::G28(EXYZ::new())))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(9, None, GCodeValue::G29))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(10, None, GCodeValue::G29_1))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(11, None, GCodeValue::G29_2))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(12, None, GCodeValue::G31))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(13, None, GCodeValue::G32))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(14, None, GCodeValue::G80))
        );

        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(15, None, GCodeValue::G90))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(16, None, GCodeValue::G91))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(17, None, GCodeValue::G92(EXYZ::new())))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(18, None, GCodeValue::G94))
        );
    }

    #[futures_test::test]
    async fn test_m_variants() {
        let stream = BufferStream::new("M\nM2\nM3\nM4\nM5\nM20\nM21\nM23\nM23\nM24\nM25\nM37\nM73\nM79\nM80\nM81\nM83\nM84\nM100\nM104\nM105\nM106\nM107\nM109\nM110\nM114\nM115\nM117\nM118\nM119\nM140\nM190\nM201\nM203\nM204\nM205\nM206\nM220\nM221\nM502\nM503\nM862.1\nM862.3\nM900\nM907\n".as_bytes().to_vec().into());

        let mut parser = GCodeLineParser::new(stream);

        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(0, None, GCodeValue::M))
        );
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Ok(GCodeCmd::new(1, None, GCodeValue::M2))
        );

        // WIP
    }

    #[futures_test::test]
    async fn test_boundary_cases() {
        let stream = BufferStream::new("X00\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let _ = parser.gcode_line();
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Err(GCodeLineParserError::GCodeNotImplemented(0, "X0".into()))
        );

        let stream = BufferStream::new("G.-1\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Err(GCodeLineParserError::ParseError(0, ParseFailure::UnexpectedByte(b'-')))
        );

        let stream = BufferStream::new("G999999999999999999999999999\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Err(GCodeLineParserError::ParseError(0, ParseFailure::NumberOverflow))
        );

        let stream = BufferStream::new("G1 X1.2.3\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Err(GCodeLineParserError::ParseError(0, ParseFailure::UnexpectedByte(b'.')))
        );
    }

    /// A wrong checksum is told apart from the other parse errors (the host resends the line),
    /// with the checksum computed from the line
    #[futures_test::test]
    async fn test_bad_checksum() {
        let stream = BufferStream::new("N14 M105*1\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        // XOR of "N14 M105"
        assert_eq!(
            parser.next_gcode(CommChannel::Internal).await,
            Err(GCodeLineParserError::BadChecksum(0, 18, 1))
        );
    }

    /// The lines stored without being executed (M28) check their checksum too: a mismatch (with
    /// both values), a checksum that is not a byte, and a valid one (stripped from the content)
    #[futures_test::test]
    async fn test_raw_line_checksum() {
        let mut buf = [0u8; 64];
        for (line, expected) in [
            ("N14 M105*1\n", Err(GCodeLineParserError::BadChecksum(0, 18, 1))),
            ("N14 M105*x\n", Err(GCodeLineParserError::ParseError(0, ParseFailure::BadNumberFormat))),
            ("N14 M105*300\n", Err(GCodeLineParserError::ParseError(0, ParseFailure::NumberOverflow))),
        ] {
            let stream = BufferStream::new(line.as_bytes().to_vec().into());
            let mut parser = GCodeLineParser::new(stream);
            assert_eq!(parser.next_raw_line(&mut buf).await, expected, "{:?}", line);
        }
        let stream = BufferStream::new("N14 M105*18\n".as_bytes().to_vec().into());
        let mut parser = GCodeLineParser::new(stream);
        let range = parser.next_raw_line(&mut buf).await.expect("valid checksum");
        assert_eq!(&buf[range], b"M105");
    }

    #[test]
    fn parse_failures_are_described() {
        use alloc::string::ToString;
        assert_eq!(ParseFailure::UnexpectedByte(b'.').to_string(), "unexpected '.'");
        assert_eq!(ParseFailure::UnexpectedByte(0).to_string(), "unexpected byte 0");
        assert_eq!(ParseFailure::NumberOverflow.to_string(), "number overflow");
        assert_eq!(ParseFailure::BadNumberFormat.to_string(), "bad number format");
        assert_eq!(ParseFailure::TooLong.to_string(), "too long");
        assert_eq!(ParseFailure::InvalidUtf8.to_string(), "invalid UTF-8");
        assert_eq!(ParseFailure::Other.to_string(), "unreadable");
    }
}
