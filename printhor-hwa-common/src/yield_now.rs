//! Cooperative yield point for the async executors.

/// Yields once to the executor: the calling task is re-scheduled right away, so the other ready
/// tasks get the chance to run.
///
/// Intended for blocking chunks of work that the hardware/HAL can not await (e.g. ADC conversions
/// without DMA support): splitting them with `yield_now().await` bounds how long the CPU is held,
/// keeping latency-sensitive tasks (like the homing step train) on time.
pub fn yield_now() -> impl core::future::Future<Output = ()> {
    YieldNow { yielded: false }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct YieldNow {
    yielded: bool,
}

impl core::future::Future for YieldNow {
    type Output = ();

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<()> {
        if self.yielded {
            core::task::Poll::Ready(())
        } else {
            self.yielded = true;
            cx.waker().wake_by_ref();
            core::task::Poll::Pending
        }
    }
}

#[cfg(test)]
mod tests {
    #[futures_test::test]
    async fn yields_once_then_completes() {
        super::yield_now().await;
    }
}
