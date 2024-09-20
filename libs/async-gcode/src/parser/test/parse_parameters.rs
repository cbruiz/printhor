use crate::{
    types::{
        expressions::{Expression, Operator},
        Literal,
    },
    GCode,
};

#[cfg(feature = "optional-value")]
use crate::{types::RealValue, Error};

use super::block_on;

#[test]
fn parse_param_get() {
    let input = "G#-21.098\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from(-21.098).into(),
                    Operator::GetParameter.into()
                ])
                .into()
            )),
            Ok(GCode::Execute)
        ]
    );
}

#[test]
fn parse_nested_parameter_get() {
    let input = "G # # - 21 . 098 \n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from(-21.098).into(),
                    Operator::GetParameter.into(),
                    Operator::GetParameter.into()
                ])
                .into()
            )),
            Ok(GCode::Execute)
        ]
    )
}

#[test]
fn parse_param_set() {
    let input = "# 23.4 = -75.8 ".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::ParameterSet((23.4).into(), (-75.8).into())),]
    );
}
#[test]
fn parse_param_set_with_nested_param_get() {
    let input = "# # 23.4 = # # -75.8 ".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::ParameterSet(
            Expression(vec![
                Literal::from(23.4).into(),
                Operator::GetParameter.into()
            ])
            .into(),
            Expression(vec![
                Literal::from(-75.8).into(),
                Operator::GetParameter.into(),
                Operator::GetParameter.into()
            ])
            .into()
        )),]
    )
}

#[test]
#[cfg(feature = "optional-value")]
fn values_are_always_required_for_parameters_id() {
    let input = "G#\n".bytes();
    assert_eq!(block_on(input), &[Err(Error::UnexpectedByte(b'\n'))]);

    let input = "#=".bytes();
    assert_eq!(block_on(input), &[Err(Error::UnexpectedByte(b'='))]);
}

#[test]
#[cfg(feature = "optional-value")]
fn parameter_set_may_have_no_value() {
    let input = "#12= g\n#34=\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::ParameterSet(
                Literal::from(12.).into(),
                RealValue::None
            )),
            Ok(GCode::Word('g', RealValue::None)),
            Ok(GCode::Execute),
            Ok(GCode::ParameterSet(
                Literal::from(34.).into(),
                RealValue::None
            )),
            Ok(GCode::Execute),
        ]
    );
}

#[test]
#[cfg(feature = "string-value")]
fn parse_param_with_string_index() {
    let input = r#"#"hello"="world" g#"hello""#.bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::ParameterSet(
                Literal::from("hello".to_owned()).into(),
                Literal::from("world".to_owned()).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from("hello".to_owned()).into(),
                    Operator::GetParameter.into()
                ])
                .into()
            )),
        ]
    );
}
