use crate::hwa;
use crate::control::{GCode, N, S, XYZ, XYZEFS, XYZE};
use crate::helpers;
use futures::Stream;

#[cfg_attr(not(feature="with-defmt"), derive(Debug))]
pub enum GCodeLineParserError {
    ParseError(u32),
    GCodeNotImplemented(u32, alloc::string::String),
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
                defmt::write!(fmt, "GCodeNotImplemented(line={}, gcode={})", ln, string.as_str());
            }
            GCodeLineParserError::FatalError => {
                defmt::write!(fmt, "FatalError");
            }
        }

    }
}

// dyn trait could reduce code size a lot with the penalty of the indirection
pub struct GCodeLineParser<STREAM>
    where STREAM: Stream<Item = Result<u8, async_gcode::Error>> + Unpin
{
    raw_parser: async_gcode::Parser<STREAM, async_gcode::Error>,
    current_line: u32,
}

#[allow(unused)]
impl<STREAM> GCodeLineParser<STREAM>
    where STREAM: Stream<Item = Result<u8, async_gcode::Error>> + Unpin
{
    pub fn new(stream: STREAM) -> Self {
        Self {
            raw_parser: async_gcode::Parser::new(stream),
            current_line: 0,
        }
    }

    #[allow(unused)]
    pub fn current_line(&self) -> u32 {
        self.current_line
    }

    pub async fn next_gcode(&mut self) -> Result<Option<GCode>, GCodeLineParserError> {

        let mut current_gcode_value = None;
        let mut current_line_number = None;
        let mut skip_gcode = false;
        let mut raw_gcode_spec: Option<(char, Option<(i32, u8)>)> = None;

        // TODO
        //self.raw_parser.get_stream();

        loop {
            match self.raw_parser.next().await {
                None => {
                    hwa::trace!("EOF reading from stream");
                    return Ok(None);
                }
                Some(result) => {
                    match result {
                        Ok(g) => {
                            match g {
                                // FIXME Undo this hacky temporary solution in async-gcode.
                                // A sub-protocol approach is preferred
                                #[cfg(feature = "grbl-compat")]
                                async_gcode::GCode::StatusCommand => {
                                    return Ok(Some(GCode::STATUS))
                                }
                                async_gcode::GCode::BlockDelete => {
                                    skip_gcode = true;
                                    //crate::debug!("BlockDelete");
                                }
                                async_gcode::GCode::LineNumber(n) => {
                                    //println!("got LN");
                                    current_line_number = Some(n);
                                }
                                async_gcode::GCode::Word(ch, fv) => {
                                    if !skip_gcode {
                                        let frx = match fv {
                                            async_gcode::RealValue::Literal(async_gcode::Literal::RealNumber(f)) => {
                                                Some((f.integer_part(), f.scale()))
                                            },
                                            _ => {
                                                None
                                            },
                                        };
                                        match &mut current_gcode_value {
                                            None => {
                                                raw_gcode_spec.replace((ch.to_ascii_uppercase(), frx));
                                                hwa::debug!("GC: {} {:?}", ch, frx);

                                                current_gcode_value = match (ch, frx) {
                                                    #[cfg(feature = "grbl-compat")]
                                                    ('$', None) => {
                                                        Some(GCode::GRBLCMD)
                                                    },
                                                    ('g', None) => {
                                                        Some(GCode::G)
                                                    },
                                                    ('g', Some((0, 0))) => {
                                                        Some(GCode::G0(XYZ {
                                                            ln: current_line_number.clone(),
                                                            f: None,
                                                            x: None,
                                                            y: None,
                                                            z: None,
                                                        }))
                                                    }
                                                    ('g', Some((1, 0))) => {
                                                        Some(GCode::G1(XYZEFS {
                                                            ln: current_line_number.clone(),
                                                            e: None,
                                                            f: None,
                                                            s: None,
                                                            x: None,
                                                            y: None,
                                                            z: None,
                                                        }))
                                                    }
                                                    ('g', Some((4, 0))) => {
                                                        Some(GCode::G4)
                                                    }
                                                    ('g', Some((10, 0))) => {
                                                        Some(GCode::G10)
                                                    }
                                                    ('g', Some((17, 0))) => {
                                                        Some(GCode::G17)
                                                    }
                                                    ('g', Some((21, 0))) => {
                                                        Some(GCode::G21)
                                                    }
                                                    ('g', Some((28, 0))) => {
                                                        Some(GCode::G28(XYZE {
                                                            ln: current_line_number.clone(),
                                                            x: None,
                                                            y: None,
                                                            z: None,
                                                            e: None,
                                                        }))
                                                    }
                                                    ('g', Some((29, 0))) => {
                                                        Some(GCode::G29)
                                                    }
                                                    ('g', Some((31, 0))) => {
                                                        Some(GCode::G31)
                                                    }
                                                    ('g', Some((32, 0))) => {
                                                        Some(GCode::G32)
                                                    }
                                                    ('g', Some((80, 0))) => {
                                                        Some(GCode::G80)
                                                    }
                                                    ('g', Some((90, 0))) => {
                                                        Some(GCode::G90)
                                                    }
                                                    ('g', Some((91, 0))) => {
                                                        Some(GCode::G91)
                                                    }
                                                    ('g', Some((92, 0))) => {
                                                        Some(GCode::G92(XYZE {
                                                            ln: current_line_number.clone(),
                                                            x: None,
                                                            y: None,
                                                            z: None,
                                                            e: None,
                                                        }))
                                                    }
                                                    ('g', Some((94, 0))) => {
                                                        Some(GCode::G94)
                                                    }
                                                    ('g', Some((291, 1))) => {
                                                        Some(GCode::G29_1)
                                                    }
                                                    ('g', Some((292, 1))) => {
                                                        Some(GCode::G29_2)
                                                    }
                                                    ('m', None) => {
                                                        Some(GCode::M)
                                                    }
                                                    ('m', Some((3, 0))) => {
                                                        Some(GCode::M3)
                                                    }
                                                    ('m', Some((5, 0))) => {
                                                        Some(GCode::M5)
                                                    }
                                                    ('m', Some((20, 0))) => {
                                                        Some(GCode::M20(None))
                                                    }
                                                    ('m', Some((23, 0))) => {
                                                        Some(GCode::M23(None))
                                                    }
                                                    ('m', Some((24, 0))) => {
                                                        Some(GCode::M24)
                                                    }
                                                    ('m', Some((25, 0))) => {
                                                        Some(GCode::M25)
                                                    }
                                                    ('m', Some((73, 0))) => {
                                                        Some(GCode::M73)
                                                    }
                                                    ('m', Some((79, 0))) => {
                                                        Some(GCode::M79)
                                                    }
                                                    ('m', Some((80, 0))) => {
                                                        Some(GCode::M80)
                                                    }
                                                    ('m', Some((81, 0))) => {
                                                        Some(GCode::M81)
                                                    }
                                                    ('m', Some((83, 0))) => {
                                                        Some(GCode::M83)
                                                    }
                                                    ('m', Some((84, 0))) => {
                                                        Some(GCode::M84)
                                                    }
                                                    ('m', Some((100, 0))) => {
                                                        Some(GCode::M100)
                                                    }
                                                    ('m', Some((104, 0))) => {
                                                        Some(GCode::M104(S{ln: None, s: None}))
                                                    }
                                                    ('m', Some((105, 0))) => {
                                                        Some(GCode::M105)
                                                    }
                                                    ('m', Some((106, 0))) => {
                                                        Some(GCode::M106)
                                                    }
                                                    ('m', Some((107, 0))) => {
                                                        Some(GCode::M107)
                                                    }
                                                    ('m', Some((109, 0))) => {
                                                        Some(GCode::M109(
                                                            S {
                                                                ln: current_line_number.clone(),
                                                                s: None
                                                            })
                                                        )
                                                    }
                                                    ('m', Some((110, 0))) => {
                                                        Some(GCode::M110(N{ln: None, n: None}))
                                                    }
                                                    ('m', Some((114, 0))) => {
                                                        Some(GCode::M114)
                                                    }
                                                    ('m', Some((115, 0))) => {
                                                        Some(GCode::M115)
                                                    }
                                                    ('m', Some((117, 0))) => {
                                                        Some(GCode::M117)
                                                    }
                                                    ('m', Some((119, 0))) => {
                                                        Some(GCode::M119)
                                                    }
                                                    ('m', Some((140, 0))) => {
                                                        Some(GCode::M140(S{ln: None, s: None}))
                                                    }
                                                    ('m', Some((190, 0))) => {
                                                        Some(GCode::M190)
                                                    }
                                                    ('m', Some((201, 0))) => {
                                                        Some(GCode::M201)
                                                    }
                                                    ('m', Some((203, 0))) => {
                                                        Some(GCode::M203)
                                                    }
                                                    ('m', Some((204, 0))) => {
                                                        Some(GCode::M204)
                                                    }
                                                    ('m', Some((205, 0))) => {
                                                        Some(GCode::M205)
                                                    }
                                                    ('m', Some((206, 0))) => {
                                                        Some(GCode::M206)
                                                    }
                                                    ('m', Some((220, 0))) => {
                                                        Some(GCode::M220(S{ ln: None, s: None }))
                                                    }
                                                    ('m', Some((221, 0))) => {
                                                        Some(GCode::M221(S{ ln: None, s: None }))
                                                    }
                                                    ('m', Some((502, 0))) => {
                                                        Some(GCode::M502)
                                                    }
                                                    ('m', Some((8621, 1))) => {
                                                        Some(GCode::M862_1)
                                                    }
                                                    ('m', Some((8623, 1))) => {
                                                        Some(GCode::M862_3)
                                                    }
                                                    ('m', Some((900, 0))) => {
                                                        Some(GCode::M900)
                                                    }
                                                    ('m', Some((907, 0))) => {
                                                        Some(GCode::M907)
                                                    }
                                                    _ => {
                                                        skip_gcode = true;
                                                        None
                                                    }
                                                };
                                            }
                                            Some(current_gcode) => {
                                                match current_gcode {
                                                    #[cfg(feature = "grbl-compat")]
                                                    GCode::STATUS => {
                                                        match (ch, frx) {
                                                            ('I', Some(val)) => {
                                                                hwa::warn!("TODO!!");
                                                            },
                                                            _ => {}
                                                        }
                                                    }
                                                    GCode::G0(coord) => {
                                                        match (ch, frx) {
                                                            ('x', Some(val)) => {
                                                                coord.x.replace(helpers::to_fixed(val));
                                                            },
                                                            ('y', Some(val)) => {
                                                                coord.y.replace(helpers::to_fixed(val));
                                                            },
                                                            ('z', Some(val)) => {
                                                                coord.z.replace(helpers::to_fixed(val));
                                                            },
                                                            ('f', Some(val)) => {
                                                                coord.f.replace(helpers::to_fixed(val));
                                                            },
                                                            _ => {}
                                                        }
                                                    }

                                                    GCode::G1(coord) => {
                                                        match (ch, frx) {
                                                            ('x', Some(val)) => {
                                                                coord.x.replace(helpers::to_fixed(val));
                                                            },
                                                            ('y', Some(val)) => {
                                                                coord.y.replace(helpers::to_fixed(val));
                                                            },
                                                            ('z', Some(val)) => {
                                                                coord.z.replace(helpers::to_fixed(val));
                                                            },
                                                            ('e', Some(val)) => {
                                                                coord.e.replace(helpers::to_fixed(val));
                                                            }
                                                            _ => {}
                                                        }
                                                    }
                                                    GCode::G28(coord) => {
                                                        match (ch, frx) {
                                                            ('x', Some(val)) => {
                                                                coord.x.replace(helpers::to_fixed(val));
                                                            },
                                                            ('y', Some(val)) => {
                                                                coord.y.replace(helpers::to_fixed(val));
                                                            },
                                                            ('z', Some(val)) => {
                                                                coord.z.replace(helpers::to_fixed(val));
                                                            },
                                                            _ => {}
                                                        }
                                                    }
                                                    GCode::G92(coord) => {
                                                        match (ch, frx) {
                                                            ('x', Some(val)) => {
                                                                coord.x.replace(helpers::to_fixed(val));
                                                            },
                                                            ('y', Some(val)) => {
                                                                coord.y.replace(helpers::to_fixed(val));
                                                            },
                                                            ('z', Some(val)) => {
                                                                coord.z.replace(helpers::to_fixed(val));
                                                            },
                                                            ('e', Some(val)) => {
                                                                coord.e.replace(helpers::to_fixed(val));
                                                            },
                                                            _ => {}
                                                        }
                                                    }
                                                    GCode::M20(path) => {
                                                        if ch == 'f' {
                                                            if let async_gcode::RealValue::Literal(async_gcode::Literal::String(mstr)) = fv {
                                                                path.replace(mstr);
                                                            }
                                                        }
                                                    }
                                                    GCode::M23(file) => {
                                                        if ch == 'f' {
                                                            if let async_gcode::RealValue::Literal(async_gcode::Literal::String(mstr)) = fv {
                                                                file.replace(mstr);
                                                            }
                                                        }
                                                    }
                                                    GCode::M104(coord) | GCode::M109(coord) | GCode::M140(coord)  | GCode::M220(coord) => {
                                                        match (ch, frx) {
                                                            ('s', Some(val)) => {
                                                                coord.s.replace(helpers::to_fixed(val));
                                                            },
                                                            _ => {}
                                                        }
                                                    }
                                                    GCode::M110(coord)  => {
                                                        match (ch, frx) {
                                                            ('n', Some(val)) => {
                                                                coord.n.replace(helpers::to_fixed(val));
                                                            },
                                                            _ => {}
                                                        }
                                                    }
                                                    _ => {}
                                                }
                                            }
                                        }
                                    } else {
                                        //crate::debug!("I'm skipping words")
                                    }
                                }
                                async_gcode::GCode::Execute => {
                                    self.current_line += 1;
                                    match current_gcode_value {
                                        None => {
                                            match raw_gcode_spec {
                                                None => continue, // Empty line. Just ignore
                                                Some(rgs) => {
                                                    // Nice to report all this kind of things, but avoidable to save code size
                                                    let num_part = match rgs.1 {
                                                        None => {alloc::string::String::new()}
                                                        Some(frv) => {
                                                            let dv = 10i32.pow(frv.1.into());
                                                            let p1 = frv.0.abs() / dv;
                                                            let p2 = frv.0.abs() - (p1 * dv);
                                                            alloc::format!("{}{}.{}", if frv.0 < 0 {"-"} else {""}, p1, p2)
                                                        }
                                                    };
                                                    let s = alloc::format!("{}{}", rgs.0, num_part);
                                                    return Err(GCodeLineParserError::GCodeNotImplemented(self.current_line, s));
                                                }
                                            }
                                        },
                                        Some(cgv) => return Ok(Some(cgv)),
                                    }
                                }
                                _ => {
                                    return Err(GCodeLineParserError::GCodeNotImplemented(self.current_line, alloc::string::String::from("N/A")))
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
                            return Err(GCodeLineParserError::ParseError(self.current_line))
                        }
                    }
                }
            }
        }
    }

    #[allow(unused)]
    pub async fn close(&mut self) {
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
