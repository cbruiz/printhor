use super::{block_on, to_gcode_comment, GCode};

#[test]
fn a_semicolon_is_followed_by_a_comment_until_the_end_of_the_line() {
    let msg = "Acceleration is defined as : δv/δt";
    let input = format!(";{}\n", msg);
    let input = input.bytes();
    let mut expected_output = to_gcode_comment(msg).to_vec();
    expected_output.push(Ok(GCode::Execute));
    assert_eq!(block_on(input), expected_output);

    let input = format!("g32 ;{}\n", msg);
    let input = input.bytes();
    let mut expected_output = vec![Ok(GCode::Word('g', (32.0).into()))];
    expected_output.extend_from_slice(&to_gcode_comment(msg));
    expected_output.push(Ok(GCode::Execute));
    assert_eq!(block_on(input), expected_output);
}
