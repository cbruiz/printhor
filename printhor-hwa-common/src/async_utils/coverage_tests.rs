//! Tests for the serial TX wrapper.
//!
//! `embassy_time` timers need the Embassy executor's timer queue unless a generic queue is
//! enabled, so the scenarios run on an Embassy std executor.
use super::*;
use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_time::{Duration, Instant};
use std::boxed::Box;
use std::sync::Arc;
use std::vec::Vec;

#[embassy_executor::task(pool_size = 4)]
async fn scenario_task(scenario: Pin<Box<dyn Future<Output = ()>>>, done: Arc<AtomicBool>) {
    scenario.await;
    done.store(true, Ordering::Release);
}

/// Runs `scenario` to completion on a dedicated Embassy executor (panics propagate).
fn block_on(scenario: impl Future<Output = ()> + 'static) {
    let done = Arc::new(AtomicBool::new(false));
    let finished = done.clone();
    let executor = Box::leak(Box::new(embassy_executor::Executor::new()));
    executor.run_until(
        move |spawner| spawner.spawn(scenario_task(Box::pin(scenario), done).unwrap()),
        move || finished.load(Ordering::Acquire),
    );
}

/// Serial device that stores what it is sent, or fails every write.
struct Device {
    sent: Vec<u8>,
    failing: bool,
}

impl embedded_io_async::ErrorType for Device {
    type Error = embedded_io_async::ErrorKind;
}

impl embedded_io_async::Write for Device {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if self.failing {
            Err(embedded_io_async::ErrorKind::BrokenPipe)
        } else {
            self.sent.extend_from_slice(buf);
            Ok(buf.len())
        }
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // The wrapper never flushes the device: it waits for the estimated transmission time
        unreachable!("SerialTxWrapper does not flush the device")
    }
}

fn device(failing: bool) -> Device {
    Device {
        sent: Vec::new(),
        failing,
    }
}

#[test]
fn write_sends_everything_and_flush_waits_for_the_pending_words() {
    block_on(async {
        let mut w = SerialTxWrapper::new(device(false), 9600);
        // 9600 baud, 10 bits per word: 1041.67 µs, rounded up (never shorter than the real time)
        assert_eq!(w._ticks_by_word, Duration::from_micros(1042).as_ticks());

        let t0 = Instant::now();
        assert_eq!(w.wrapped_write(b"G28\n").await, Ok(4));
        assert_eq!(w.peri.sent, b"G28\n");
        assert!(t0.elapsed() >= Duration::from_millis(1), "settle delay after a write");
        assert_eq!(w.last_write_len, 4);

        // Flush waits for the written words (plus one) and resets the pending length
        let t1 = Instant::now();
        w.wrapped_flush().await;
        assert!(t1.elapsed() >= Duration::from_micros(1042 * 5));
        assert_eq!(w.last_write_len, 0);

        // Nothing pending: flushing again does not wait for anything
        w.wrapped_flush().await;
        assert_eq!(w.last_write_len, 0);

        // Empty writes are accepted
        assert_eq!(w.wrapped_write(b"").await, Ok(0));
        assert_eq!(w.peri.sent, b"G28\n");
    });
}

/// The wrapper logs the error of the underlying device and then reports success anyway.
#[test]
fn write_errors_are_reported_to_the_caller() {
    block_on(async {
        let mut w = SerialTxWrapper::new(device(true), 115200);
        assert_eq!(
            w.wrapped_write(b"M115\n").await,
            Err(embedded_io_async::ErrorKind::BrokenPipe)
        );
    });
}

/// The flush delay is derived from `(1_000_000 / baud_rate) * 10` µs: the integer division
/// truncates the bit time, so the wait is shorter than the real transmission time (115200 baud:
/// 80 µs instead of 86.8 µs per word) and becomes zero above 1 Mbaud.
#[test]
fn flush_delay_covers_the_real_word_time() {
    for baud in [9600u32, 115200, 250000, 921600, 2_000_000] {
        let w = SerialTxWrapper::new(device(false), baud);
        let word_ns = 10_000_000_000u64 / baud as u64;
        let waited_ns = Duration::from_ticks(w._ticks_by_word).as_micros() * 1000;
        assert!(waited_ns >= word_ns, "{} baud: waits {} ns per word, needs {} ns", baud, waited_ns, word_ns);
    }
}
