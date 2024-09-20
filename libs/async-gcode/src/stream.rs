use futures::TryStream;

pub(crate) trait MyTryStreamExt: TryStream {
    #[cfg(not(feature = "parse-checksum"))]
    fn push_backable(self) -> pushback::PushBack<Self>
    where
        Self: Sized,
    {
        pushback::PushBack::new(self)
    }

    #[cfg(feature = "parse-checksum")]
    fn xor_summed_push_backable(
        self,
        initial_sum: Self::Ok,
    ) -> xorsum_pushback::XorSumPushBack<Self>
    where
        Self: Sized,
        Self::Ok: Copy + core::ops::BitXorAssign,
    {
        xorsum_pushback::XorSumPushBack::new(self, initial_sum)
    }
}
impl<T: ?Sized> MyTryStreamExt for T where T: TryStream {}

pub(crate) trait PushBackable {
    type Item;
    fn push_back(&mut self, v: Self::Item) -> Option<Self::Item>;
}

#[cfg(not(feature = "parse-checksum"))]
pub(crate) mod pushback {
    use futures::{Stream, TryStream};
    use pin_project_lite::pin_project;

    use core::pin::Pin;
    use core::task::{Context, Poll};

    use super::PushBackable;

    pin_project! {
        pub(crate) struct PushBack<S: TryStream> {
            #[pin]
            stream: S,
            val: Option<S::Ok>,
        }
    }

    impl<S: TryStream> PushBack<S> {
        pub fn new(stream: S) -> Self {
            Self { stream, val: None }
        }
    }
    impl<S> PushBackable for PushBack<S>
    where
        S: TryStream,
    {
        type Item = S::Ok;
        fn push_back(&mut self, v: S::Ok) -> Option<S::Ok> {
            self.val.replace(v)
        }
    }

    impl<S> Stream for PushBack<S>
    where
        S: TryStream,
    {
        type Item = Result<S::Ok, S::Error>;
        fn poll_next(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
            let this = self.project();
            if let Some(v) = this.val.take() {
                Poll::Ready(Some(Ok(v)))
            } else {
                this.stream.try_poll_next(ctx)
            }
        }
    }

    #[cfg(test)]
    mod test {
        use super::{PushBack, PushBackable};
        use futures::stream::{self, StreamExt};

        #[test]
        fn the_stream_works() {
            let data = [1, 2, 4, 8, 16, 32, 64, 128]
                .iter()
                .copied()
                .map(Result::<_, core::convert::Infallible>::Ok)
                .collect::<Vec<_>>();
            let mut strm = PushBack::new(stream::iter(data.iter().copied()));
            assert_eq!(
                futures_executor::block_on((&mut strm).collect::<Vec<_>>()),
                data
            );
        }

        #[test]
        fn pushbacked_value_come_out_first() {
            let data = [1, 2, 4, 8, 16, 32, 64, 128]
                .iter()
                .copied()
                .map(Result::<_, core::convert::Infallible>::Ok)
                .collect::<Vec<_>>();
            let mut strm = PushBack::new(stream::iter(data.iter().copied()));
            assert_eq!(
                futures_executor::block_on((&mut strm).take(4).collect::<Vec<_>>()),
                &data[..=3]
            );

            strm.push_back(0xCC);

            assert_eq!(
                futures_executor::block_on((&mut strm).take(4).collect::<Vec<_>>()),
                [0xCC, 16, 32, 64]
                    .iter()
                    .copied()
                    .map(Result::<_, core::convert::Infallible>::Ok)
                    .collect::<Vec<_>>()
            );
        }
    }
}

#[cfg(feature = "parse-checksum")]
pub(crate) mod xorsum_pushback {
    use futures::{Stream, TryStream};
    use pin_project_lite::pin_project;

    use core::pin::Pin;
    use core::task::{Context, Poll};

    use super::PushBackable;

    pin_project! {
        pub(crate) struct XorSumPushBack<S: TryStream> {
            #[pin]
            stream: S,
            head: Option<S::Ok>,
            sum: S::Ok
        }
    }

    impl<S> XorSumPushBack<S>
    where
        S: TryStream,
        S::Ok: core::ops::BitXorAssign + Copy,
    {
        pub fn new(stream: S, initial_sum: S::Ok) -> Self {
            Self {
                stream,
                head: None,
                sum: initial_sum,
            }
        }

        pub fn reset_sum(&mut self, initial_sum: S::Ok) {
            self.sum = initial_sum;
        }

        pub fn sum(&self) -> S::Ok {
            self.sum
        }
    }

    impl<S> PushBackable for XorSumPushBack<S>
    where
        S: TryStream,
        S::Ok: core::ops::BitXorAssign + Copy,
    {
        type Item = S::Ok;
        fn push_back(&mut self, head: S::Ok) -> Option<S::Ok> {
            self.sum ^= head;
            self.head.replace(head)
        }
    }

    impl<S> Stream for XorSumPushBack<S>
    where
        S: TryStream,
        S::Ok: core::ops::BitXorAssign + Copy,
    {
        type Item = Result<S::Ok, S::Error>;

        fn poll_next(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
            let this = self.project();
            let item = if let Some(item) = this.head.take() {
                item
            } else {
                match this.stream.try_poll_next(ctx) {
                    Poll::Ready(Some(Ok(item))) => item,
                    other => return other,
                }
            };
            *this.sum ^= item;
            Poll::Ready(Some(Ok(item)))
        }
    }

    #[cfg(test)]
    mod test {
        use super::{PushBackable, XorSumPushBack};
        use futures::stream::{self, StreamExt};

        #[test]
        fn the_stream_works_and_the_xorsum_is_computed() {
            let data = [1, 2, 4, 8, 16, 32, 64, 128]
                .iter()
                .copied()
                .map(Result::<_, core::convert::Infallible>::Ok)
                .collect::<Vec<_>>();
            let mut strm = XorSumPushBack::new(stream::iter(data.iter().copied()), 0);
            assert_eq!(
                futures_executor::block_on((&mut strm).collect::<Vec<_>>()),
                data
            );
            assert_eq!(strm.sum(), 0xFF);
        }

        #[test]
        fn the_xorsum_can_be_reset() {
            let data = [1, 2, 4, 8, 16, 32, 64, 128]
                .iter()
                .copied()
                .map(Result::<_, core::convert::Infallible>::Ok)
                .collect::<Vec<_>>();
            let mut strm = XorSumPushBack::new(stream::iter(data.iter().copied()), 0);
            assert_eq!(
                futures_executor::block_on((&mut strm).take(4).collect::<Vec<_>>()),
                &data[..=3]
            );
            assert_eq!(strm.sum(), 0x0F);

            strm.reset_sum(0x30);

            assert_eq!(
                futures_executor::block_on((&mut strm).collect::<Vec<_>>()),
                &data[4..]
            );
            assert_eq!(strm.sum(), 0xC0);
        }

        #[test]
        fn pushing_back_updates_the_xorsum() {
            let data = [1, 2, 4, 8, 16, 32, 64, 128]
                .iter()
                .copied()
                .map(Result::<_, core::convert::Infallible>::Ok)
                .collect::<Vec<_>>();
            let mut strm = XorSumPushBack::new(stream::iter(data.iter().copied()), 0);
            assert_eq!(
                futures_executor::block_on((&mut strm).take(4).collect::<Vec<_>>()),
                &data[..=3]
            );
            assert_eq!(strm.sum(), 0x0F);

            strm.push_back(0xCC);

            assert_eq!(strm.sum(), 0xC3);
            assert_eq!(
                futures_executor::block_on((&mut strm).take(4).collect::<Vec<_>>()),
                [0xCC, 16, 32, 64]
                    .iter()
                    .copied()
                    .map(Result::<_, core::convert::Infallible>::Ok)
                    .collect::<Vec<_>>()
            );
        }
    }
}
