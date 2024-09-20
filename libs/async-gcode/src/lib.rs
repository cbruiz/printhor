//! This crate implements a GCode (RS-274) parser.
//!
//! The default dialect is taken from NIST's [RS274/NGC interpreter version 3].
//! Some expensive part of that dialect such as parameters and expressions are gated behind feature
//! – respectively `parse-parameters` and `parse-expressions` – in order to avoid dependency on
//! dynamic allocation.
//!
//! Some extension to this dialect such as checksum, trailing comments, optional values are also
//! supported and gated behind features.
//!
//! ## 🔩 Example
//!
//! ```
//! use futures::stream;
//! use futures_executor::block_on;
//! use async_gcode::{Parser, Error};
//! let input = r"
//!    G21 H21. I21.098
//!    J-21 K-21. L-21.098
//!    M+21 N+21. P+21.098
//!    Q.098 R-.098 S+.098
//!    t - 21 . 33 "
//!    // mapping to `Result<u8, Error>` to render error conversion transparent.
//!    .bytes().map(Result::<_, Error>::Ok);
//!
//! block_on(async {
//!     let mut parser = Parser::new(stream::iter(input));
//!
//!     loop {
//!         if let Some(res) = parser.next().await {
//!             println!("{:?}", res);
//!         } else {
//!             break;
//!         }
//!     }
//! });
//! ```
//!
//! ## Error management
//!
//! On parsing error the `Parser` can no longer trust its input and enters an error recovery state.
//! No `GCode` or `Error` will be emitted until a new line character (`\n`) is received. Then a
//! `GCode::Execute` is emitted and the parser restarts in a reliable known state.
//!
//! ## ⚙ Features
//! - `std` : Enabled by default. Allows for the use of dynamic allocation.
//! - `parse-comments` : enables the parser to return `GCode::Comment(String)`; requires an allocator.
//! - `parse-trailing-comment`: allows line to end with a `; comment`.
//! - `parse-checksum` : Enables the use of xorsum.
//! - `parse-parameters` : Enables the use of `#` parameters ; requires an allocator.
//!   If `string-value` is enabled then parameters may use string index.
//!   If `optional-value` is enabled then parameters value may be omitted but **NOT** the indices.
//! - `parse-expressions` : Enables parsing infix expressions ; requires an allocator.
//! - `optional-value` : Allows to omit in `RealValue` in word and parameter value positions.
//!   Parameter indices cannot be omitted nor can be literals in expressions.
//! - `string-value` : Allows `RealValue` to be a string. Any character preceded with `\` will be
//!   used as is (useful for `"`, `)` or new line).
//!
//! ## ⚠ Warning
//!
//! Dev-dependencies currently leak features to dependencies.
//! This crate requires rust-nightly to build with no_std until `-Z features=dev_dep` makes it to
//! stable.
//!
//! ## Note for future development
//! It might be interesting to have a look at ISO 6983 and/or ISO 14649.
//!
//! During development fixed arithmetics was considered to be made available as an alternative to
//! the floating point arithmetics especially for small target. After investigation, it does not
//! seem to be a significant gain for the precision required by the gcode standard.
//!
//! Ideally all arithmetics triggered by a theoritically valid input should be caught and not
//! trigger a panic. For an excessive  number of digit in a number may exceed the capacity of the
//! variable used internally.
//!
//! [RS274/NGC interpreter version 3]: https://www.nist.gov/publications/nist-rs274ngc-interpreter-version-3?pub_id=823374
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(all(
    not(feature = "std"),
    any(
        feature = "parse-expressions",
        feature = "parse-parameters",
        feature = "parse-comments",
        feature = "string-value"
    )
))]
extern crate alloc;

#[cfg(all(not(feature = "std"), feature = "parse-comments"))]
use alloc::string::String;

#[macro_use]
mod utils;

mod stream;
mod types;

mod parser;

pub use parser::Parser;
pub use parser::AsyncParserState;
pub use types::Literal;
pub use types::RealValue;

pub use types::DecimalRepr;

#[cfg(any(feature = "parse-expressions", feature = "parse-parameters"))]
pub use types::expressions::Expression;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    /// Error no the gcode syntax
    UnexpectedByte(u8),

    /// The parsed number exceeded the expected range.
    NumberOverflow,

    /// Incompatible number conversions.
    InvalidNumberConversion,

    /// Format error during number parsing. Typically a dot without digits (at least one is
    /// required).
    BadNumberFormat,

    #[cfg(any(feature = "parse-comments", feature = "string-value"))]
    /// The string or comment received contained an invalid UTF-8 character sequence.
    InvalidUTF8String,

    #[cfg(feature = "parse-checksum")]
    /// Checksum verification failed. The error contains the computed value.
    BadChecksum(u8),

    #[cfg(feature = "parse-expressions")]
    /// The expressions received was invalid.
    InvalidExpression,
}

#[derive(Debug, PartialEq, Clone)]
pub enum GCode {
    StatusCommand,
    BlockDelete,
    LineNumber(u32),
    #[cfg(feature = "parse-comments")]
    Comment(String),
    Word(char, RealValue),
    #[cfg(feature = "parse-parameters")]
    /// When `optional-value` is enabled, the index cannot be `RealValue::None`.
    ParameterSet(RealValue, RealValue),
    Execute,
}
