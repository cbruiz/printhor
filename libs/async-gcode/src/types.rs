#[cfg(all(
    not(feature = "std"),
    any(feature = "parse-comments", feature = "string-value")
))]
use alloc::string::String;
#[cfg(feature = "float-f64")]
use core::f64;

#[derive(Debug)]
pub(crate) enum ParseResult<G, E> {
    Input(E),
    Parsing(crate::Error),
    Ok(G),
}
impl<O, E> From<core::result::Result<O, E>> for ParseResult<O, E> {
    fn from(f: core::result::Result<O, E>) -> Self {
        match f {
            Ok(o) => ParseResult::Ok(o),
            Err(e) => ParseResult::Input(e),
        }
    }
}

impl<O, E> From<crate::Error> for ParseResult<O, E> {
    fn from(f: crate::Error) -> Self {
        ParseResult::Parsing(f)
    }
}

#[cfg(not(feature = "parse-comments"))]
pub type Comment = ();
#[cfg(feature = "parse-comments")]
pub type Comment = String;

#[derive(Debug, Clone, Copy)]
pub struct DecimalRepr {
    integer: i32,
    scale: u8,
}

impl PartialEq for DecimalRepr {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        if self.scale == other.scale {
            self.integer == other.integer
        }
        else if self.scale > other.scale {
            self.integer == (other.integer * 10i32.pow((self.scale - other.scale) as u32))
        }
        else {
            other.integer == (self.integer * 10i32.pow((other.scale - self.scale) as u32))
        }
    }
}

impl Default for DecimalRepr {
    fn default() -> Self {
        Self {
            integer: 0i32,
            scale: 0u8,
        }
    }
}

impl DecimalRepr {
    pub const fn new(integer: i32, scale: u8) -> DecimalRepr {
        DecimalRepr {
            integer,
            scale,
        }
    }
    #[cfg(feature = "float-f64")]
    /// Convert from f64 with a maximum of 8 decimal positions
    pub fn from_f64(number: f64) -> DecimalRepr {
        let integer =  (number * 100000000.0f64).trunc() as i32;
        DecimalRepr {
            integer,
            scale: 8,
        }
    }
    #[cfg(feature = "float-f64")]
    pub fn to_f64(&self ) -> f64 {
        self.integer as f64 / (10u64.pow(self.scale as u32) as f64)
    }

    #[inline]
    pub fn integer_part(&self) -> i32 {
        self.integer
    }

    #[inline]
    pub fn scale(&self) -> u8 {
        self.scale
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum Literal {
    RealNumber(DecimalRepr),
    #[cfg(feature = "string-value")]
    String(String),
}
impl Literal {
    pub fn as_decimal(&self) -> Option<DecimalRepr> {
        match self {
            Literal::RealNumber(rn) => Some(*rn),
            #[cfg(feature = "string-value")]
            _ => None,
        }
    }
    #[cfg(feature = "float-f64")]
    pub fn as_real_number(&self) -> Option<f64> {
        match self {
            Literal::RealNumber(rn) => Some(rn.to_f64()),
            #[cfg(feature = "string-value")]
            _ => None,
        }
    }
    #[cfg(feature = "string-value")]
    pub fn as_string(&self) -> Option<&str> {
        match self {
            Literal::String(string) => Some(string),
            _ => None,
        }
    }
}

impl From<DecimalRepr> for Literal {
    fn from(from: DecimalRepr) -> Self {
        Self::RealNumber(from)
    }
}

impl From<i32> for Literal {
    fn from(from: i32) -> Self {
        Self::RealNumber(DecimalRepr::new(from, 0))
    }
}
impl TryFrom<u32> for Literal {

    type Error = crate::Error;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(Self::RealNumber(
            DecimalRepr::new(
                i32::try_from(value)
                    .map_err(|_| crate::Error::InvalidNumberConversion)?, 0
            )
        ))
    }
}
#[cfg(feature = "float-f64")]
impl TryFrom<f64> for Literal {
    type Error = crate::Error;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        // As parser constraints: 5 decimals max
        let scaled_value = value * 100000.0;
        if scaled_value > f64::from(i32::MAX) {
            Err(crate::Error::InvalidNumberConversion)
        } else if scaled_value < f64::from(i32::MIN) {
            Err(crate::Error::InvalidNumberConversion)
        } else {
            Ok(Self::RealNumber(
                DecimalRepr::new(scaled_value as i32, 5)
            ))
        }
    }
}

#[cfg(feature = "string-value")]
impl From<String> for Literal {
    fn from(from: String) -> Self {
        Self::String(from)
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum RealValue {
    Literal(Literal),
    #[cfg(any(feature = "parse-parameters", feature = "parse-expressions"))]
    Expression(expressions::Expression),
    #[cfg(feature = "optional-value")]
    None,
}
impl Default for RealValue {
    fn default() -> Self {
        DecimalRepr::new(0, 0).into()
    }
}
impl<T: Into<Literal>> From<T> for RealValue {
    fn from(from: T) -> Self {
        RealValue::Literal(from.into())
    }
}

#[cfg(feature = "float-f64")]
impl TryFrom<f64> for RealValue {
    type Error = crate::Error;

    fn try_from(from: f64) -> Result<Self, Self::Error> {
        Ok(RealValue::Literal(from.try_into()?))
    }
}

#[cfg(any(feature = "parse-parameters", feature = "parse-expressions"))]
pub(crate) mod expressions {
    use super::{Literal, RealValue};
    use crate::Error;
    use either::Either;

    #[cfg(not(feature = "std"))]
    use alloc::vec::Vec;

    pub(crate) type ExprItem = Either<Operator, Literal>;
    pub(crate) type ExprInner = Vec<ExprItem>;

    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    pub enum OpType {
        Unary,
        Binary,
    }
    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    pub enum Associativity {
        Left,
        #[cfg(feature = "parse-parameters")]
        Right,
    }
    #[derive(Debug, Clone, Copy, PartialOrd, PartialEq, Eq)]
    pub enum Precedence {
        Group1,
        Group2,
        Group3,
        #[cfg(feature = "parse-parameters")]
        Group4,
        Group5,
    }

    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    pub enum Operator {
        // Binary operators
        Add,
        Substract,
        Multiply,
        Divide,
        Power,

        And,
        Or,
        Xor,

        Modulus,

        // Unary operators
        Cos,
        Sin,
        Tan,
        ACos,
        ASin,
        ATan, // Atan is kind of binary

        Abs,
        Exp,
        Fix,
        Fup,
        Ln,
        Round,
        Sqrt,

        #[cfg(feature = "parse-parameters")]
        GetParameter,
    }
    impl Operator {
        pub fn op_type(&self) -> OpType {
            match self {
                Self::Add
                | Self::Substract
                | Self::Multiply
                | Self::Divide
                | Self::Modulus
                | Self::Power
                | Self::And
                | Self::Or
                | Self::Xor
                | Self::ATan => OpType::Binary,
                Self::Cos
                | Self::Sin
                | Self::Tan
                | Self::ACos
                | Self::ASin
                | Self::Abs
                | Self::Exp
                | Self::Fix
                | Self::Fup
                | Self::Ln
                | Self::Round
                | Self::Sqrt => OpType::Unary,
                #[cfg(feature = "parse-parameters")]
                Self::GetParameter => OpType::Unary,
            }
        }

        pub fn associativity(&self) -> Associativity {
            #[allow(clippy::match_single_binding)]
            match self {
                #[cfg(feature = "parse-parameters")]
                Self::GetParameter => Associativity::Right,
                _ => Associativity::Left,
            }
        }

        pub fn precedence(&self) -> Precedence {
            match *self {
                Self::Add | Self::Substract | Self::And | Self::Or | Self::Xor => {
                    Precedence::Group1
                }
                Self::Multiply | Self::Divide | Self::Modulus => Precedence::Group2,
                Self::Power => Precedence::Group3,
                #[cfg(feature = "parse-parameters")]
                Self::GetParameter => Precedence::Group4,
                Self::Cos
                | Self::Sin
                | Self::Tan
                | Self::ACos
                | Self::ASin
                | Self::ATan
                | Self::Abs
                | Self::Exp
                | Self::Fix
                | Self::Fup
                | Self::Ln
                | Self::Round
                | Self::Sqrt => Precedence::Group5,
            }
        }
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct Expression(pub(crate) ExprInner);
    impl Expression {
        /// Evaluates the expressions to a single literal. Math error may occur during the
        /// expression's resolution (e.g. division by 0).
        ///
        /// When `parse-parameters` is enabled, this method takes a closure as an argument.
        /// This closure is used to resolve parameters get.
        pub fn evaluate(
            &self,
            #[cfg(feature = "parse-parameters")] _cbk: &mut dyn FnMut(Literal) -> Literal,
        ) -> Result<Literal, Error> {
            todo!()
        }
    }

    impl From<Operator> for Either<Operator, Literal> {
        fn from(from: Operator) -> Self {
            Self::Left(from)
        }
    }
    impl From<Literal> for Either<Operator, Literal> {
        fn from(from: Literal) -> Self {
            Self::Right(from)
        }
    }

    impl From<Expression> for Either<Literal, Expression> {
        fn from(from: Expression) -> Self {
            Self::Right(from)
        }
    }
    impl From<Literal> for Either<Literal, Expression> {
        fn from(from: Literal) -> Self {
            Self::Left(from)
        }
    }

    impl From<Expression> for RealValue {
        fn from(from: Expression) -> RealValue {
            RealValue::Expression(from)
        }
    }

    #[cfg(test)]
    mod test {
        // test plan for expressions:
        // TBD
    }
}
