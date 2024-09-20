//! The ebnf representatio following [https://bottlecaps.de/rr/ui]'s syntax
//! ```ebnf
//! line       ::= '/'? ( [Nn] [0-9]+ )?
//!                ( [a-zA-Z] real_value? | '#' real_value '=' real_value? | '(' [^)] ')' )*
//!                ( '*' [0-9]+ /* 0 to 255 */ )?
//!                ( ';' [^\n]* )? '\n'
//! real_value ::= '#'* ( real_number
//!                     | '"' [^"] '"'
//!                     | 'atan' expression '/' expression
//!                     | ( 'abs' | 'acos' | 'asin' | 'cos' | 'exp' | 'fix' | 'fup' | 'ln' | 'round' | 'sin' | 'sqrt' | 'tan' ) expression )
//! expression ::= '[' real_value ( ( '**' | '/' | 'mod' | '*' | 'and' | 'xor' | '-' | 'or' | '+' ) real_value )* ']'
//! real_number ::= ( '+' | '-' )? ( [0-9]+ ( '.' [0-9]* )? | '.' [0-9]+ )
//! ```
//!
#[cfg(all(not(feature = "std"), feature = "parse-comments"))]
use alloc::string::String;
#[cfg(all(not(feature = "std"), any(feature = "parse-comments")))]
use alloc::vec::Vec;

mod values;

#[cfg(feature = "parse-expressions")]
mod expressions;

#[cfg(test)]
mod test;

use futures::{Stream, StreamExt};

use crate::{
    stream::{MyTryStreamExt, PushBackable},
    types::{Comment, ParseResult},
    utils::skip_whitespaces,
    Error, GCode,
};

use values::parse_number;

#[cfg(not(feature = "parse-expressions"))]
use values::parse_real_value;

#[cfg(feature = "parse-expressions")]
use expressions::parse_real_value;

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum AsyncParserState {
    Start(bool),
    LineNumberOrSegment,
    Segment,
    ErrorRecovery,
    #[cfg(all(feature = "parse-trailing-comment", feature = "parse-checksum"))]
    EoLOrTrailingComment,
    #[cfg(any(feature = "parse-trailing-comment", feature = "parse-checksum"))]
    EndOfLine,
}

#[cfg(all(feature = "parse-trailing-comment", not(feature = "parse-comments")))]
async fn parse_eol_comment<S, E>(input: &mut S) -> Option<Result<Comment, E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin + PushBackable<Item = u8>,
{
    loop {
        let b = match input.next().await? {
            Ok(o) => o,
            Err(e) => return Some(Err(e)),
        };
        match b {
            b'\r' | b'\n' => {
                input.push_back(b);
                break Some(Ok(()));
            }
            _ => {}
        }
    }
}

#[cfg(all(feature = "parse-trailing-comment", feature = "parse-comments"))]
async fn parse_eol_comment<S, E>(input: &mut S) -> Option<ParseResult<Comment, E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin + PushBackable<Item = u8>,
{
    let mut v = Vec::new();
    loop {
        let b = try_result!(input.next());
        match b {
            b'\r' | b'\n' => {
                input.push_back(b);
                break Some(match String::from_utf8(v) {
                    Ok(s) => ParseResult::Ok(s),
                    Err(_) => Error::InvalidUTF8String.into(),
                });
            }
            b => v.push(b),
        }
    }
}

#[cfg(not(feature = "parse-comments"))]
async fn parse_inline_comment<S, E>(input: &mut S) -> Option<ParseResult<Comment, E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin + PushBackable<Item = u8>,
{
    loop {
        match try_result!(input.next()) {
            b'\\' => {
                try_result!(input.next());
            }
            b'(' => break Some(Error::UnexpectedByte(b'(').into()),
            b')' => break Some(ParseResult::Ok(())),
            _ => {}
        }
    }
}

#[cfg(feature = "parse-comments")]
async fn parse_inline_comment<S, E>(input: &mut S) -> Option<ParseResult<Comment, E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin + PushBackable<Item = u8>,
{
    let mut v = Vec::new();
    loop {
        let b = try_result!(input.next());
        match b {
            b'\\' => {
                v.push(try_result!(input.next()));
            }
            b'(' => break Some(Error::UnexpectedByte(b'(').into()),
            b')' => {
                break Some(match String::from_utf8(v) {
                    Ok(s) => ParseResult::Ok(s),
                    Err(_) => Error::InvalidUTF8String.into(),
                })
            }
            b => v.push(b),
        }
    }
}

// use a different struct to compute checksum
#[cfg(not(feature = "parse-checksum"))]
use crate::stream::pushback::PushBack;
#[cfg(feature = "parse-checksum")]
type PushBack<T> = crate::stream::xorsum_pushback::XorSumPushBack<T>;

async fn parse_eol<S, E>(
    state: &mut AsyncParserState,
    input: &mut PushBack<S>,
) -> Option<ParseResult<GCode, E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin,
{
    Some(loop {
        let b = try_result!(input.next());
        match b {
            b'\r' | b'\n' => {
                *state = AsyncParserState::Start(true);
                #[cfg(feature = "parse-checksum")]
                {
                    input.reset_sum(0);
                }
                break ParseResult::Ok(GCode::Execute);
            }
            b' ' => {}
            b => break Error::UnexpectedByte(b).into(),
        }
    })
}

macro_rules! try_await {
    ($input:expr) => {
        match $input.await? {
            ParseResult::Ok(ok) => ok,
            ParseResult::Parsing(err) => break Err(err.into()),
            ParseResult::Input(err) => break Err(err.into()),
        }
    };
}

macro_rules! try_await_result {
    ($input:expr) => {
        match $input.await? {
            Ok(ok) => ok,
            Err(err) => break Err(err.into()),
        }
    };
}

pub struct Parser<S, E>
where
    S: Stream<Item = Result<u8, E>> + Unpin,
{
    input: PushBack<S>,
    state: AsyncParserState,
}

impl<S, E> Parser<S, E>
where
    S: Stream<Item = Result<u8, E>> + Unpin,
    E: From<Error>,
{
    pub fn new(input: S) -> Self {
        Self {
            #[cfg(feature = "parse-checksum")]
            input: input.xor_summed_push_backable(0),
            #[cfg(not(feature = "parse-checksum"))]
            input: input.push_backable(),
            state: AsyncParserState::Start(true),
        }
    }
    pub fn reset(&mut self)  {
        #[cfg(feature = "parse-checksum")]
        self.input.reset_sum(0);
        self.state = AsyncParserState::Start(true);
    }
    pub fn get_state(&self) -> AsyncParserState {
        self.state
    }
    pub async fn next(&mut self) -> Option<Result<GCode, E>> {
        let res = loop {
            let b = match self.input.next().await? {
                Ok(b) => b,
                Err(err) => return Some(Err(err)),
            };

            // println!("{:?}: {:?}", self.state, char::from(b));
            match self.state {
                AsyncParserState::Start(ref mut first_byte) => match b {
                    b'?' => {
                        break Ok(GCode::StatusCommand);
                    }
                    b'\n' => {
                        self.input.push_back(b);
                        break Ok(try_await!(parse_eol(&mut self.state, &mut self.input)));
                    }
                    b'/' if *first_byte => {
                        self.state = AsyncParserState::LineNumberOrSegment;
                        break Ok(GCode::BlockDelete);
                    }
                    b' ' => {
                        *first_byte = false;
                    }
                    _ => {
                        self.input.push_back(b);
                        self.state = AsyncParserState::LineNumberOrSegment
                    }
                },
                AsyncParserState::LineNumberOrSegment => match b.to_ascii_lowercase() {
                    b'n' => {
                        try_await_result!(skip_whitespaces(&mut self.input));
                        let (n, ord) = try_await_result!(parse_number(&mut self.input));
                        break if ord == 0 {
                            let b = try_await_result!(self.input.next());
                            Err(Error::UnexpectedByte(b).into())
                        } else if ord > 5 {
                            Err(Error::NumberOverflow.into())
                        } else {
                            self.state = AsyncParserState::Segment;
                            Ok(GCode::LineNumber(n))
                        };
                    }
                    _ => {
                        self.input.push_back(b);
                        self.state = AsyncParserState::Segment;
                    }
                },
                AsyncParserState::Segment => match b.to_ascii_lowercase() {
                    b' ' => {}
                    letter @ b'a'..=b'z' => {
                        try_await_result!(skip_whitespaces(&mut self.input));
                        let rv = try_await!(parse_real_value(&mut self.input));
                        // println!("word({:?}, {:?})", letter as char, rv);
                        break Ok(GCode::Word(letter.into(), rv));
                    }
                    // FIXME
                    letter @ b'$' => {
                        try_await_result!(skip_whitespaces(&mut self.input));
                        let rv = try_await!(parse_real_value(&mut self.input));
                        // println!("word({:?}, {:?})", letter as char, rv);
                        break Ok(GCode::Word(letter.into(), rv));
                    }
                    b'\r' | b'\n' => {
                        self.input.push_back(b);
                        break Ok(try_await!(parse_eol(&mut self.state, &mut self.input)));
                    }
                    // param support feature
                    #[cfg(feature = "parse-parameters")]
                    b'#' => {
                        try_await_result!(skip_whitespaces(&mut self.input));
                        #[allow(clippy::match_single_binding)]
                        let param_id = match try_await!(parse_real_value(&mut self.input)) {
                            #[cfg(feature = "optional-value")]
                            crate::RealValue::None => {
                                let b = try_await_result!(self.input.next());
                                break Err(Error::UnexpectedByte(b).into());
                            }
                            id => id,
                        };
                        // println!("param_id: {:?}", param_id);
                        try_await_result!(skip_whitespaces(&mut self.input));
                        let b = try_await_result!(self.input.next());
                        if b'=' != b {
                            break Err(Error::UnexpectedByte(b).into());
                        }

                        try_await_result!(skip_whitespaces(&mut self.input));
                        let value = try_await!(parse_real_value(&mut self.input));
                        // println!("param_id: {:?}", value);

                        break Ok(GCode::ParameterSet(param_id, value));
                    }
                    // checksum support feature
                    #[cfg(feature = "parse-checksum")]
                    b'*' => {
                        let sum = self.input.sum() ^ b'*';
                        try_await_result!(skip_whitespaces(&mut self.input));
                        let (n, _) = try_await_result!(parse_number(&mut self.input));
                        // println!("{} {}", sum, n);
                        if n >= 256 {
                            break Err(Error::NumberOverflow.into());
                        } else if (n as u8) != sum {
                            break Err(Error::BadChecksum(sum).into());
                        } else {
                            try_await_result!(skip_whitespaces(&mut self.input));
                            #[cfg(not(feature = "parse-trailing-comment"))]
                            {
                                self.state = AsyncParserState::EndOfLine;
                            }
                            #[cfg(feature = "parse-trailing-comment")]
                            {
                                self.state = AsyncParserState::EoLOrTrailingComment;
                            }
                        }
                    }
                    // comment support features
                    #[cfg(not(feature = "parse-comments"))]
                    b'(' => {
                        try_await!(parse_inline_comment(&mut self.input));
                    }
                    #[cfg(feature = "parse-comments")]
                    b'(' => {
                        break Ok(GCode::Comment(try_await!(parse_inline_comment(
                            &mut self.input
                        ))));
                    }
                    #[cfg(all(
                        feature = "parse-trailing-comment",
                        not(feature = "parse-comments")
                    ))]
                    b';' => {
                        try_await_result!(parse_eol_comment(&mut self.input));
                        self.state = AsyncParserState::EndOfLine;
                    }
                    #[cfg(all(feature = "parse-trailing-comment", feature = "parse-comments"))]
                    b';' => {
                        let s = try_await!(parse_eol_comment(&mut self.input));
                        self.state = AsyncParserState::EndOfLine;
                        break Ok(GCode::Comment(s));
                    }
                    _ => break Err(Error::UnexpectedByte(b).into()),
                },
                #[cfg(all(
                    feature = "parse-trailing-comment",
                    not(feature = "parse-comments"),
                    feature = "parse-checksum"
                ))]
                AsyncParserState::EoLOrTrailingComment => match b {
                    b';' => try_await_result!(parse_eol_comment(&mut self.input)),
                    _ => {
                        self.input.push_back(b);
                        break Ok(try_await!(parse_eol(&mut self.state, &mut self.input)));
                    }
                },
                #[cfg(all(
                    feature = "parse-trailing-comment",
                    feature = "parse-comments",
                    feature = "parse-checksum"
                ))]
                AsyncParserState::EoLOrTrailingComment => match b {
                    b';' => {
                        let s = try_await!(parse_eol_comment(&mut self.input));
                        self.state = AsyncParserState::EndOfLine;
                        break Ok(GCode::Comment(s));
                    }
                    _ => {
                        self.input.push_back(b);
                        break Ok(try_await!(parse_eol(&mut self.state, &mut self.input)));
                    }
                },
                #[cfg(any(feature = "parse-trailing-comment", feature = "parse-checksum"))]
                AsyncParserState::EndOfLine => {
                    self.input.push_back(b);
                    break Ok(try_await!(parse_eol(&mut self.state, &mut self.input)));
                }
                AsyncParserState::ErrorRecovery => match b {
                    b'\r' | b'\n' => {
                        self.input.push_back(b);
                        break Ok(try_await!(parse_eol(&mut self.state, &mut self.input)));
                    }
                    _ => {}
                },
            }
        };
        // eprintln!("{}:{} {:?}", file!(), line!(), res);
        if res.is_err() {
            self.state = AsyncParserState::ErrorRecovery;
        }
        Some(res)
    }
}
