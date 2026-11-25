//! Tests for the bit-banging half-duplex UART, driven by mock pins.
//!
//! `embassy_time` timers need the Embassy executor's timer queue (hwa-common does not enable a
//! generic queue), so the asynchronous scenarios run on an Embassy std executor.
use super::*;
use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use hwa::uart::SerialError;
use std::boxed::Box;
use std::collections::VecDeque;
use std::sync::Arc;
use std::vec::Vec;

#[embassy_executor::task(pool_size = 8)]
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

#[derive(Debug, PartialEq, Clone, Copy)]
enum Mode {
    Unset,
    Output,
    Input,
}

/// Pin that records every level written to the line (TX side).
struct RecordingPin {
    mode: Mode,
    levels: Vec<bool>,
}

impl RecordingPin {
    fn new() -> Self {
        Self {
            mode: Mode::Unset,
            levels: Vec::new(),
        }
    }
}

impl IOPin for RecordingPin {
    fn set_output(&mut self) {
        self.mode = Mode::Output;
    }
    fn set_input(&mut self) {
        self.mode = Mode::Input;
    }
    fn is_high(&mut self) -> bool {
        self.levels.last().copied().unwrap_or(true)
    }
    fn set_high(&mut self) {
        self.levels.push(true);
    }
    fn set_low(&mut self) {
        self.levels.push(false);
    }
    fn set_open_drain(&mut self) {}
}

/// Pin whose sampled levels (RX side) are played back from a script; idle (high) afterwards.
struct ScriptedPin {
    script: VecDeque<bool>,
    samples: usize,
}

impl ScriptedPin {
    fn new(script: &[bool]) -> Self {
        Self {
            script: script.iter().copied().collect(),
            samples: 0,
        }
    }

    /// Two idle samples, the start bit, the 8 data bits (LSB first) and the given stop level.
    fn frame(byte: u8, stop: bool) -> Self {
        let mut levels = std::vec![true, true, false];
        levels.extend((0..8).map(|bit| (byte >> bit) & 1 == 1));
        levels.push(stop);
        Self::new(&levels)
    }
}

impl IOPin for ScriptedPin {
    fn set_output(&mut self) {}
    fn set_input(&mut self) {}
    fn is_high(&mut self) -> bool {
        self.samples += 1;
        self.script.pop_front().unwrap_or(true)
    }
    fn set_high(&mut self) {}
    fn set_low(&mut self) {}
    fn set_open_drain(&mut self) {}
}

/// Expected line levels for a byte: start bit, data bits LSB first, stop bit.
fn frame_levels(byte: u8) -> Vec<bool> {
    let mut levels = std::vec![false];
    levels.extend((0..8).map(|bit| (byte >> bit) & 1 == 1));
    levels.push(true);
    levels
}

#[test]
fn construction_idles_the_line_high_and_sets_timings() {
    let serial = HalfDuplexSerial::new(RecordingPin::new(), 9600);
    assert_eq!(serial.rxtx.mode, Mode::Output);
    assert_eq!(serial.rxtx.levels, [true]);
    assert_eq!(serial.bit_period, Duration::from_hz(9600));
    assert_eq!(serial.word_transfer_duration(), serial.bit_period * 10);
    assert_eq!(serial.timeout_ms, None);
}

#[test]
fn mode_switching_and_timeout_configuration() {
    block_on(async {
        let mut serial = HalfDuplexSerial::new(RecordingPin::new(), 115200);
        serial.set_read_mode().await;
        assert_eq!(serial.rxtx.mode, Mode::Input);
        serial.set_write_mode().await;
        assert_eq!(serial.rxtx.mode, Mode::Output);
        // Back to write mode: the line is driven to the idle (high) level again
        assert_eq!(serial.rxtx.levels, [true, true]);

        serial.set_timeout(Some(Duration::from_millis(5)));
        assert_eq!(serial.timeout_ms, Some(Duration::from_millis(5)));
        serial.set_timeout(None);
        assert_eq!(serial.timeout_ms, None);
    });
}

#[test]
fn write_emits_start_data_lsb_first_and_stop_bits() {
    block_on(async {
        let mut serial = HalfDuplexSerial::new(RecordingPin::new(), 115200);
        for byte in [0xA5u8, 0x00, 0xFF, b'?'] {
            serial.rxtx.levels.clear();
            let t0 = embassy_time::Instant::now();
            assert_eq!(serial.write(byte).await, Ok(()));
            assert_eq!(serial.rxtx.levels, frame_levels(byte), "byte {:#04x}", byte);
            // One ticker period per bit (start + 8 data + stop)
            assert!(t0.elapsed() >= serial.bit_period * 9, "byte {:#04x} sent too fast", byte);
        }
    });
}

#[test]
fn read_decodes_a_frame_sampled_lsb_first() {
    block_on(async {
        for byte in [0xA5u8, 0x00, 0xFF, 0x3C] {
            let mut serial = HalfDuplexSerial::new(ScriptedPin::frame(byte, true), 115200);
            assert_eq!(serial.read().await, Ok(byte));
            // idle, idle, start, 8 data bits, stop
            assert_eq!(serial.rxtx.samples, 12);
        }
    });
}

#[test]
fn read_reports_a_missing_stop_bit_as_framing_error() {
    block_on(async {
        let mut serial = HalfDuplexSerial::new(ScriptedPin::frame(0x81, false), 115200);
        assert_eq!(serial.read().await, Err(SerialError::Framing));
    });
}

#[test]
fn read_times_out_on_an_idle_line() {
    block_on(async {
        let mut serial = HalfDuplexSerial::new(ScriptedPin::new(&[]), 9600);
        let timeout = Duration::from_millis(2);
        serial.set_timeout(Some(timeout));
        let t0 = embassy_time::Instant::now();
        assert_eq!(serial.read().await, Err(SerialError::Timeout));
        assert!(t0.elapsed() >= timeout);
        assert!(serial.rxtx.samples > 1, "the line is polled while waiting");
    });
}
