use crate::stream::PushBackable;
use futures::{Stream, StreamExt};

#[doc(hidden)]
#[macro_export]
macro_rules! try_parse {
    ($input:expr) => {
        match $input.await? {
            ParseResult::Ok(b) => b,
            ParseResult::Input(e) => return Some(ParseResult::Input(e)),
            ParseResult::Parsing(e) => return Some(ParseResult::Parsing(e)),
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! try_result {
    ($input:expr) => {
        match $input.await? {
            Ok(b) => b,
            Err(e) => return Some(ParseResult::Input(e)),
        }
    };
}

pub(crate) async fn skip_whitespaces<S, E>(input: &mut S) -> Option<Result<(), E>>
where
    S: Stream<Item = Result<u8, E>> + Unpin + PushBackable<Item = u8>,
{
    loop {
        let b = match input.next().await? {
            Ok(b) => b,
            Err(e) => return Some(Err(e)),
        };
        if b != b' ' {
            input.push_back(b);
            break;
        }
    }
    Some(Ok(()))
}

#[cfg(test)]
mod test {
    use super::{skip_whitespaces, stream, StreamExt};

    #[cfg(not(feature = "parse-checksum"))]
    use crate::stream::pushback::PushBack;
    #[cfg(feature = "parse-checksum")]
    use crate::stream::xorsum_pushback::XorSumPushBack;

    #[test]
    fn skips_white_spaces_and_pushes_back_the_first_non_space_byte() {
        let mut data = PushBack::new(stream::iter(
            b"      d"
                .iter()
                .copied()
                .map(Result::<_, core::convert::Infallible>::Ok),
        ));
        futures_executor::block_on(async move {
            skip_whitespaces(&mut data).await;
            assert_eq!(Some(Ok(b'd')), data.next().await);
        });
    }
}
