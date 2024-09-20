use futures::stream;

use super::{Error, GCode, Parser, StreamExt};

#[cfg(feature = "optional-value")]
use crate::types::RealValue;

use crate::DecimalRepr;

#[cfg(feature = "parse-checksum")]
mod parse_checksum;
#[cfg(feature = "parse-expressions")]
mod parse_expressions;
#[cfg(feature = "parse-parameters")]
mod parse_parameters;
#[cfg(feature = "parse-trailing-comment")]
mod parse_trailing_comment;

fn block_on<T: Iterator<Item = u8>>(it: T) -> Vec<Result<GCode, Error>> {
    let mut parser = Parser::new(stream::iter(it).map(Result::<_, Error>::Ok));

    futures_executor::block_on(
        stream::unfold(
            &mut parser,
            |p| async move { p.next().await.map(|w| (w, p)) },
        )
        .collect(),
    )
}
#[cfg(not(feature = "parse-comments"))]
fn to_gcode_comment(_msg: &str) -> [Result<GCode, Error>; 0] {
    []
}
#[cfg(feature = "parse-comments")]
fn to_gcode_comment(msg: &str) -> [Result<GCode, Error>; 1] {
    [Ok(GCode::Comment(msg.to_string()))]
}

#[test]
fn empty_lines_are_not_ignored() {
    assert_eq!(
        block_on("\n\n".bytes()),
        &[Ok(GCode::Execute), Ok(GCode::Execute)]
    );
}

#[test]
fn block_delete_emited_immediately() {
    assert_eq!(block_on("/".bytes()), &[Ok(GCode::BlockDelete)]);
}

#[test]
fn spaces_are_not_allowed_before_block_delete() {
    assert_eq!(block_on(" /".bytes()), &[Err(Error::UnexpectedByte(b'/'))]);
}

#[test]
fn error_in_underlying_stream_are_passed_through_and_parser_recovers_on_execute() {
    #[derive(Debug, Copy, Clone, PartialEq)]
    enum TestError {
        SomeError,
        ParseError(Error),
    }
    impl From<Error> for TestError {
        fn from(from: Error) -> Self {
            TestError::ParseError(from)
        }
    }

    let string = b"g12 h12 b32\n";
    let input = string[0..6]
        .iter()
        .copied()
        .map(Result::Ok)
        .chain([Err(TestError::SomeError)].iter().copied())
        .chain(string[6..].iter().copied().map(Result::Ok));

    let mut parser = Parser::new(stream::iter(input));

    assert_eq!(
        futures_executor::block_on(
            stream::unfold(
                &mut parser,
                |p| async move { p.next().await.map(|w| (w, p)) },
            )
            .collect::<Vec<_>>()
        ),
        [
            Ok(GCode::Word('g', (12).into())),
            Err(TestError::SomeError),
            Ok(GCode::Execute)
        ]
    )
}

#[test]
fn line_number() {
    let input = "n23\n".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::LineNumber(23)), Ok(GCode::Execute)]
    );

    let input = "N0023\n".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::LineNumber(23)), Ok(GCode::Execute)]
    );

    let input = " N 0023 \n".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::LineNumber(23)), Ok(GCode::Execute)]
    );
}

#[test]
fn line_number_with_more_than_5_digits_are_not_ok() {
    let input = "N000009\n".bytes();
    assert_eq!(
        block_on(input),
        &[Err(Error::NumberOverflow), Ok(GCode::Execute)]
    )
}

#[test]
fn line_number_can_only_be_intergers() {
    let input = "N000.09\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::LineNumber(0)),
            Err(Error::UnexpectedByte(b'.')),
            Ok(GCode::Execute)
        ]
    );
    let input = "N009.\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::LineNumber(9)),
            Err(Error::UnexpectedByte(b'.')),
            Ok(GCode::Execute)
        ]
    );
    let input = "N.009\n".bytes();
    assert_eq!(
        block_on(input),
        &[Err(Error::UnexpectedByte(b'.')), Ok(GCode::Execute)]
    )
}

#[test]
fn line_number_must_have_a_value() {
    let input = "N\n".bytes();
    assert_eq!(block_on(input), &[Err(Error::UnexpectedByte(b'\n'))]);
}

#[test]
fn comments_may_contain_utf8_characters() {
    let msg = "accélération = δv/δt";
    let input = format!("({})", msg).into_bytes().into_iter();

    assert_eq!(block_on(input), to_gcode_comment(msg));
}

#[test]
fn open_parenthesis_are_not_allowed_in_comments() {
    let msg = "accélération = (δv/δt";
    let input = format!("({})", msg).into_bytes().into_iter();
    assert_eq!(block_on(input), [Err(Error::UnexpectedByte(b'('))]);
}

#[test]
fn comments_may_contain_escaped_open_parenthesis() {
    let msg = "accélération = (δv/δt)";
    let input = "(accélération = \\(δv/δt\\))".bytes();
    assert_eq!(block_on(input), to_gcode_comment(msg));
}

#[test]
fn word_with_number() {
    let input = r"
        G21 H21. I21.098
        J-21 K-21. L-21.098
        M+21 N+21. P+21.098
        Q.098 R-.098 S+.098
        t - 21 . 33 "
        .bytes();

    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Execute),
            Ok(GCode::Word('g', (21.0).try_into().unwrap())),
            Ok(GCode::Word('h', (21.0).try_into().unwrap())),
            Ok(GCode::Word('i', (21.098).try_into().unwrap())),
            Ok(GCode::Execute),
            Ok(GCode::Word('j', (-21.0).try_into().unwrap())),
            Ok(GCode::Word('k', (-21.0).try_into().unwrap())),
            Ok(GCode::Word('l', (-21.098).try_into().unwrap())),
            Ok(GCode::Execute),
            Ok(GCode::Word('m', (21.0).try_into().unwrap())),
            Ok(GCode::Word('n', (21.0).try_into().unwrap())),
            Ok(GCode::Word('p', (21.098).try_into().unwrap())),
            Ok(GCode::Execute),
            Ok(GCode::Word('q', (0.098).try_into().unwrap())),
            Ok(GCode::Word('r', (-0.098).try_into().unwrap())),
            Ok(GCode::Word('s', (0.098).try_into().unwrap())),
            Ok(GCode::Execute),
            // avoid precision loss with f64 conversion in this case
            Ok(GCode::Word('t', DecimalRepr::new(-2133, 2).try_into().unwrap())),
        ]
    );
}

#[test]
#[cfg(all(feature = "optional-value", not(feature = "parse-expressions")))]
fn word_may_not_have_a_value() {
    let input = "G75 Z T48 S P.3\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word('g', (75.0).try_into().unwrap())),
            Ok(GCode::Word('z', RealValue::None)),
            Ok(GCode::Word('t', (48.0).try_into().unwrap())),
            Ok(GCode::Word('s', RealValue::None)),
            Ok(GCode::Word('p', (0.3).try_into().unwrap())),
            Ok(GCode::Execute)
        ]
    );
}

#[test]
#[cfg(all(feature = "optional-value", feature = "parse-expressions"))]
fn word_may_not_have_a_value_but_ambiguous_sequence_will_error() {
    // T following a value less Z will be read as the start of «Tan» and the expression will err
    // on the unexpected '4' that follows.
    let input = "G75 Z T48 S P.3\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word('g', (75.0).try_into().unwrap())),
            Ok(GCode::Word('z', RealValue::None)),
            Err(Error::UnexpectedByte(b'4')),
            Ok(GCode::Execute)
        ]
    );
}

#[test]
#[cfg(feature = "string-value")]
fn value_may_be_strings() {
    let input = r#"G "Hello\"World\"" "#.bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word('g', "Hello\"World\"".to_string().into()))]
    )
}
