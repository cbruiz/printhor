#[cfg(feature = "parse-trailing-comment")]
use super::to_gcode_comment;
use super::{block_on, Error, GCode};

#[test]
fn parse_checksum_from_start_to_checksum() {
    let input = "G0*119\n".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word('g', (0.).into())), Ok(GCode::Execute)]
    );

    let input = "G0*118\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word('g', (0.).into())),
            Err(Error::BadChecksum(119)),
            Ok(GCode::Execute)
        ]
    );

    let input = "G0* 119 \n".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word('g', (0.).into())), Ok(GCode::Execute)]
    );
}
#[test]
fn checksum_resets_between_lines() {
    let input = r"
G0*119
 G1*86
"
    .bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Execute),
            Ok(GCode::Word('g', (0.).into())),
            Ok(GCode::Execute),
            Ok(GCode::Word('g', (1.0).into())),
            Ok(GCode::Execute),
        ]
    )
}

#[test]
#[cfg(feature = "parse-trailing-comment")]
fn trailing_comment_are_not_covered_by_checksum() {
    let msg = " trailing comments may contain (or have?) parenthesis";
    let input = format!("G0* 119 ;{}\n", msg);
    let input = input.bytes();

    let mut expected_output = vec![Ok(GCode::Word('g', (0.).into()))];
    expected_output.extend(to_gcode_comment(msg).iter().cloned());
    expected_output.push(Ok(GCode::Execute));
    assert_eq!(block_on(input), expected_output)
}
