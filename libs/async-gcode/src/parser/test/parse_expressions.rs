use super::{block_on, GCode};
use crate::types::{
    expressions::{Expression, Operator},
    Literal,
};

#[test]
fn parse_sngle_real_value_expressions() {
    let input = "G[3]\n".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word('g', (3).into())), Ok(GCode::Execute)]
    );
}

#[test]
fn parse_unary_as_first_expressions() {
    let input = r"
G Cos [ 90 ] G Sin [ 90 ] G Tan [ 90 ]
G ACos [ .5 ] g ASin[ - . 5 ] g aTan [ 2.3 ] / [ 28 ]
g abs [ -.3 ] g exp [ 32 ] g fix [ 5.8 ] g fup [ 4.3 ]
g ln [ 2.718281828 ] g round [ 4.8 ] g sqrt [ 4 ]"
        .bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Execute),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(90).into(), Operator::Cos.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(90).into(), Operator::Sin.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(90).into(), Operator::Tan.into()]).into()
            )),
            Ok(GCode::Execute),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(0.5).into(), Operator::ACos.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(-0.5).into(), Operator::ASin.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from(2.3).into(),
                    Literal::from(28).into(),
                    Operator::ATan.into()
                ])
                .into()
            )),
            Ok(GCode::Execute),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(-0.3).into(), Operator::Abs.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(32).into(), Operator::Exp.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(5.8).into(), Operator::Fix.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(4.3).into(), Operator::Fup.into()]).into()
            )),
            Ok(GCode::Execute),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(2.718281828).into(), Operator::Ln.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(4.8).into(), Operator::Round.into()]).into()
            )),
            Ok(GCode::Word(
                'g',
                Expression(vec![Literal::from(4).into(), Operator::Sqrt.into()]).into()
            )),
        ]
    );
}

#[test]
fn parse_binary_operator_with_appropriate_precedence() {
    let input = "g [ 2 - 9 * [7 * 5] ** 2 * 4 + 8 / 4 ]".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word(
            'g',
            Expression(vec![
                Literal::from(2).into(),
                Literal::from(9).into(),
                Literal::from(7).into(),
                Literal::from(5).into(),
                Operator::Multiply.into(),
                Literal::from(2).into(),
                Operator::Power.into(),
                Operator::Multiply.into(),
                Literal::from(4).into(),
                Operator::Multiply.into(),
                Operator::Substract.into(),
                Literal::from(8).into(),
                Literal::from(4).into(),
                Operator::Divide.into(),
                Operator::Add.into(),
            ])
            .into()
        ))]
    )
}

#[test]
fn parse_addition_of_negative_numbers() {
    let input = "g [ -2 + -9 ]".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word(
            'g',
            Expression(vec![
                Literal::from(-2).into(),
                Literal::from(-9).into(),
                Operator::Add.into(),
            ])
            .into()
        ))]
    )
}

#[test]
fn unary_have_higher_precedence_on_power_operator() {
    let input = "g [ cos [ 90 ] ** 2 ]".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word(
            'g',
            Expression(vec![
                Literal::from(90).into(),
                Operator::Cos.into(),
                Literal::from(2).into(),
                Operator::Power.into()
            ])
            .into()
        ))]
    )
}

#[test]
fn operator_name_are_case_insensitive() {
    let input = "g [ Cos [ 90 ] ** 2 + sIn [ 90 ] ]".bytes();
    assert_eq!(
        block_on(input),
        &[Ok(GCode::Word(
            'g',
            Expression(vec![
                Literal::from(90).into(),
                Operator::Cos.into(),
                Literal::from(2).into(),
                Operator::Power.into(),
                Literal::from(90).into(),
                Operator::Sin.into(),
                Operator::Add.into(),
            ])
            .into()
        ))]
    )
}

#[test]
#[cfg(feature = "parse-parameters")]
fn parse_expressions_with_parameter_get() {
    let input = "G#[32+25]\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from(32).into(),
                    Literal::from(25).into(),
                    Operator::Add.into(),
                    Operator::GetParameter.into(),
                ])
                .into()
            )),
            Ok(GCode::Execute)
        ]
    );
    let input = "G # [ + 32 + - 25 + # - 8.32 ] \n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from(32).into(),
                    Literal::from(-25).into(),
                    Operator::Add.into(),
                    Literal::from(-8.32).into(),
                    Operator::GetParameter.into(),
                    Operator::Add.into(),
                    Operator::GetParameter.into(),
                ])
                .into()
            )),
            Ok(GCode::Execute)
        ]
    );
    let input = "G#Cos[90]\n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from(90).into(),
                    Operator::Cos.into(),
                    Operator::GetParameter.into(),
                ])
                .into()
            )),
            Ok(GCode::Execute)
        ]
    );
    let input = "G # atan [ 180 / 2 ] / [ 75 + 2 ] \n".bytes();
    assert_eq!(
        block_on(input),
        &[
            Ok(GCode::Word(
                'g',
                Expression(vec![
                    Literal::from(180).into(),
                    Literal::from(2).into(),
                    Operator::Divide.into(),
                    Literal::from(75).into(),
                    Literal::from(2).into(),
                    Operator::Add.into(),
                    Operator::ATan.into(),
                    Operator::GetParameter.into(),
                ])
                .into()
            )),
            Ok(GCode::Execute)
        ]
    );
}
