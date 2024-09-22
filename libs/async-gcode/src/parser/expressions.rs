use either::Either;
use futures::stream::{Stream, StreamExt};

#[cfg(all(
    not(feature = "std"),
    any(feature = "parse-parameters", feature = "parse-expressions")
))]
use alloc::vec::Vec;
#[cfg(feature = "std")]
use std::vec::Vec;

use crate::{
    stream::PushBackable,
    types::{
        expressions::{Associativity, ExprItem, Expression, OpType, Operator},
        Literal, ParseResult, RealValue,
    },
    utils::skip_whitespaces,
    Error,
};

#[derive(PartialEq, Debug, Clone)]
enum Token {
    OpenBracket, // stores what to expect on expr exit. BinOpOrEnd or ATanDiv
    CloseBracket,
    Literal(Literal),
    Operator(Operator),
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum Expect {
    UnaryOrLiteralOrExpr,
    Expr,
    ATanDiv,
    BinOpOrCloseBracket,
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum Stacked {
    Operator(Operator),
    OpenBracket,
    ATan,
}

macro_rules! match_operator {
    (@ $input:expr, $e:block) => {{
        $e
    }};
    (@ $input:expr, $p:expr) => {{
        Token::Operator($p)
    }};
    (@ $input:expr, $c:literal => $op:path) => {{
        let mut zipped = $input.zip(futures::stream::iter($c.iter()));
        for _ in 0..$c.len() {
            match zipped.next().await? {
                (Ok(input_byte), &expected_byte) => if input_byte.to_ascii_lowercase() != expected_byte {
                    return Some(ParseResult::Parsing(Error::UnexpectedByte(input_byte)))
                }
                (Err(e), _) => return Some(ParseResult::Input(e))
            }
        }

        Token::Operator($op)
    }};
    (@ $input:expr, $( $(#[$($m:tt)*])* $p:pat => { $($rest:tt)* } )*) => {{
        let b = match $input.next().await? {
            Ok(b) => b,
            Err(e) => return Some(ParseResult::Input(e)),
        };
        match b.to_ascii_lowercase() {
            $(
                $p => {
                    match_operator!(@ $input, $($rest)*)
                }
             )*
            #[allow(unreachable_patterns)]
            _ => return Some(ParseResult::Parsing(Error::UnexpectedByte(b))),
        }
    }};
    ($input:expr, $b:expr, $($(#[$($m:tt)*])* $p:pat => { $($rest:tt)* })* ) => {{
        match $b.to_ascii_lowercase() {
            $(
                $(#[$($m)*])*
                $p => {
                    match_operator!(@ $input, $($rest)*)
                }
             )*
            #[allow(unreachable_patterns)]
            b => return Some(ParseResult::Parsing(Error::UnexpectedByte(b))),
        }
    }}
}

async fn tokenize<S, E>(input: &mut S, expect: Expect) -> Option<ParseResult<Token, E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin + PushBackable<Item = u8>,
{
    let b = match input.next().await? {
        Ok(b) => b,
        Err(e) => return Some(ParseResult::Input(e)),
    };
    // println!("{:?}", b as char);
    let token = match expect {
        Expect::UnaryOrLiteralOrExpr => {
            match_operator! { input, b,
                #[cfg(feature = "parse-parameters")]
                b'#' => { Operator::GetParameter }
                b'[' => {{ Token::OpenBracket }}
                b'a' => {
                    b'b' => { b"s" => Operator::Abs }
                    b'c' => { b"os" => Operator::ACos }
                    b's' => { b"in" => Operator::ASin }
                    b't' => { b"an" => Operator::ATan }
                }
                b'c' => { b"os" => Operator::Cos }
                b'e' => { b"xp" => Operator::Exp }
                b'f' => {
                    b'i' => { b"x" => Operator::Fix }
                    b'u' => { b"p" => Operator::Fup }
                }
                b'l' => { b"n" => Operator::Ln }
                b'r' => { b"ound" => Operator::Round }
                b's' => {
                    b'i' => { b"n" => Operator::Sin }
                    b'q' => { b"rt" => Operator::Sqrt }
                }
                b't' => { b"an" => Operator::Tan }
                _ => {{
                    input.push_back(b);
                    let lit = try_parse!(super::values::parse_literal(input));
                            Token::Literal(lit)

                }}
            }
        }
        Expect::BinOpOrCloseBracket => {
            match_operator! { input, b,
                b'+' => { Operator::Add }
                b'-' => { Operator::Substract }
                b'*' => {
                    b'*' => { Operator::Power }
                    b => {{
                        input.push_back(b);
                        Token::Operator(Operator::Multiply)
                    }}
                }
                b'/' => { Operator::Divide }
                b'a' => { b"nd" => Operator::And }
                b'o' => { b"r" => Operator::Or }
                b'x' => { b"or" => Operator::Xor }
                b'm' => { b"od" => Operator::Modulus }
                b']' => {{ Token::CloseBracket }}
                #[cfg(not(feature = "parse-parameters"))]
                _ => {{ return Some(ParseResult::Parsing(Error::UnexpectedByte(b))) }}
            }
        }
        Expect::Expr => match b {
            b'[' => Token::OpenBracket,
            _ => return Some(ParseResult::Parsing(Error::UnexpectedByte(b))),
        },
        Expect::ATanDiv => match b {
            b'/' => Token::Operator(Operator::Divide),
            _ => return Some(ParseResult::Parsing(Error::UnexpectedByte(b))),
        },
    };
    Some(ParseResult::Ok(token))
}

pub(crate) async fn parse_real_value<S, E>(input: &mut S) -> Option<ParseResult<RealValue, E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin + PushBackable<Item = u8>,
{
    /*
    Begin
       initially push some special character say # into the stack
       for each character tok from infix expression, do
          literal => add tok to postfix expression
          openBracket => push ( into stack
          closeBracket =>
            while stack is not empty and stack top ≠ (,
              do pop and add item from stack to postfix expression
            done

            if stack.is_empty => error !

            pop ( also from the stack
          operator(op) =>
            while !stack.is_empty AND
                  precedence(op) < precedence(stack.top) OR
                  (precedence(op) == precedence(stack.top) AND !op.is_right_associative) do
              pop and add into postfix expression
            done

            stack.push(op)
       done

       while the stack contains some remaining token, do
          pop and add to the postfix expression
       done
       return postfix
    End
    */

    let mut expects = Expect::UnaryOrLiteralOrExpr;
    let mut stack: Vec<Stacked> = Vec::new();
    let mut postfix: Vec<ExprItem> = Vec::new();
    let mut depth = 0;

    loop {
        try_result!(skip_whitespaces(input));
        //println!("{:?}: {:?} {:?}", expects, postfix, stack);

        // lexical analysis
        let token = match tokenize(input, expects).await? {
            ParseResult::Ok(tok) => tok,
            #[cfg(feature = "optional-value")]
            ParseResult::Parsing(Error::UnexpectedByte(b))
                if postfix.is_empty() && stack.is_empty() =>
            {
                //println!("err: {:?}", b as char);
                input.push_back(b);
                return Some(ParseResult::Ok(RealValue::None));
            }
            ParseResult::Parsing(e) => return Some(ParseResult::Parsing(e)),
            ParseResult::Input(e) => return Some(ParseResult::Input(e)),
        };
        //println!("token: {:?}", token);

        // grammar analysis
        match token {
            Token::OpenBracket => {
                depth += 1;
                expects = Expect::UnaryOrLiteralOrExpr;
                stack.push(Stacked::OpenBracket)
            }
            Token::Literal(l) => {
                postfix.push(l.into());
                if depth != 0 {
                    expects = Expect::BinOpOrCloseBracket;
                } else {
                    break;
                }
            }
            Token::CloseBracket => {
                loop {
                    match stack.pop() {
                        Some(Stacked::Operator(op)) => postfix.push(op.into()),
                        Some(Stacked::OpenBracket) => break,
                        Some(_) | None => unreachable!(),
                    };
                }
                // println!("CloseBracket: {:?} {:?}", postfix, stack);
                // TODO: use checked arithmetic to prevent panic if depth == 0
                depth -= 1;

                if stack
                    .last()
                    .map(|&stacked| stacked == Stacked::ATan)
                    .unwrap_or(false)
                {
                    //TODO: this loop on expecting an ATan, let's use a stack specific Type and
                    //keep track there wether we already got our atandiv or not
                    expects = Expect::ATanDiv;
                } else if depth != 0 {
                    expects = Expect::BinOpOrCloseBracket;
                } else {
                    break;
                }
            }
            Token::Operator(Operator::Divide) if expects == Expect::ATanDiv => {
                stack.pop(); // this is known to be Stacked::ATan
                stack.push(Stacked::Operator(Operator::ATan));
                expects = Expect::Expr
            }
            Token::Operator(op) => {
                /*
                 * while !stack.is_empty AND
                 *       precedence(op) < precedence(stack.top) OR
                 *       (precedence(op) == precedence(stack.top) AND !op.is_right_associative) do
                 *   pop and add into postfix expression
                 * done
                 */
                while stack
                    .last()
                    .map(|tok| match tok {
                        Stacked::Operator(stacked_op) => {
                            op.precedence() < stacked_op.precedence()
                                || (op.precedence() == stacked_op.precedence()
                                    && op.associativity() == Associativity::Left)
                        }
                        Stacked::OpenBracket => false,
                        _ => unreachable!(),
                    })
                    .unwrap_or(false)
                {
                    postfix.push(
                        match stack.pop().unwrap() {
                            Stacked::Operator(op) => op,
                            _ => unreachable!(),
                        }
                        .into(),
                    )
                }

                match op {
                    #[cfg(feature = "parse-parameters")]
                    Operator::GetParameter => {
                        stack.push(Stacked::Operator(Operator::GetParameter));
                        expects = Expect::UnaryOrLiteralOrExpr;
                    }
                    Operator::ATan => {
                        stack.push(Stacked::ATan);
                        expects = Expect::Expr;
                    }
                    _ => {
                        stack.push(Stacked::Operator(op));

                        if op.op_type() == OpType::Unary {
                            expects = Expect::Expr
                        } else {
                            expects = Expect::UnaryOrLiteralOrExpr
                        }
                    }
                }
            }
        }
    }

    /*
      while the stack contains some remaining characters, do
         pop and add to the postfix expression
      done
      return postfix
    */
    while let Some(stacked) = stack.pop() {
        match stacked {
            Stacked::Operator(op) => postfix.push(op.into()),
            Stacked::OpenBracket => return Some(ParseResult::Parsing(Error::InvalidExpression)),
            _ => unreachable!(),
        }
    }

    debug_assert!(stack.is_empty());

    // println!("end: {:?} {:?}", postfix, stack);
    Some(ParseResult::Ok(if postfix.len() == 1 {
        if let Some(Either::Right(rv)) = postfix.pop() {
            rv.into()
        } else {
            unreachable!()
        }
    } else {
        Expression(postfix).into()
    }))
}
