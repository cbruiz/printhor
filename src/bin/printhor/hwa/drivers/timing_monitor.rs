use std::fs::File;
use std::io::Write;
use bitflags::bitflags;
use embassy_time::Instant;
use crate::hwa;
use crate::control::task_stepper_isr::now;

bitflags! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PinState: u16 {
        const USCLK    = 0b1000000000000000;
        const X_ENA    = 0b0000000000000001;
        const X_DIR    = 0b0000000000000010;
        const X_STEP   = 0b0000000000000100;
        const Y_ENA    = 0b0000000000001000;
        const Y_DIR    = 0b0000000000010000;
        const Y_STEP   = 0b0000000000100000;
        const Z_ENA    = 0b0000000001000000;
        const Z_DIR    = 0b0000000010000000;
        const Z_STEP   = 0b0000000100000000;
        const E_ENA    = 0b0000001000000000;
        const E_DIR    = 0b0000010000000000;
        const E_STEP   = 0b0000100000000000;
        const INIT     = PinState::X_ENA.bits() | PinState::Y_ENA.bits() | PinState::Z_ENA.bits() | PinState::E_ENA.bits();
    }
}


/// A monitor structure to debug signal timings
pub(crate) struct TimingsMonitor {
    move_id: usize,
    ref_time: Instant,
    #[cfg(feature = "no-real-time")]
    real_time: Instant,
    current_state: PinState,
    timings: Vec<(u64, PinState)>,
}

impl TimingsMonitor {
    pub(crate) fn new() -> Self {
        Self {
            ref_time: Instant::from_ticks(0),
            #[cfg(feature = "no-real-time")]
            real_time: Instant::from_ticks(0),
            move_id: 0,
            current_state: PinState::INIT,
            timings: Vec::with_capacity(20_000),
        }
    }

    pub(crate) fn reset(&mut self, ref_time: Instant, _real_time: Instant) {
        self.ref_time = ref_time;
        #[cfg(feature = "no-real-time")]
        {
            self.real_time = _real_time;
        }
        self.move_id += 1;
        self.current_state = PinState::INIT;
        self.timings.clear();
    }

    #[cfg(all(feature = "native", feature="no-real-time"))]
    pub(crate) fn set_clock(&mut self, real_time: Instant) {
        self.real_time = real_time;
    }

    pub(crate) fn update(&mut self, pin: PinState, state: bool) {
        self.current_state.set(pin, state);
        #[cfg(feature = "no-real-time")]
        let real_time = self.real_time;
        #[cfg(not(feature = "no-real-time"))]
        let real_time = now();
        let k1 = (real_time - self.ref_time).as_micros();
        self.timings.push((k1, self.current_state));
    }

    pub(crate) fn swap(&mut self, pin: PinState) {
        self.current_state.toggle(pin);
        #[cfg(feature = "no-real-time")]
        let real_time = self.real_time;
        #[cfg(not(feature = "no-real-time"))]
        let real_time = now();
        let k1 = (real_time - self.ref_time).as_micros();
        self.timings.push((k1, self.current_state));
    }

    pub(crate) fn commit(&mut self) {
        hwa::info!("commit {}", self.move_id);
        hwa::info!("Plotting {} events", self.timings.len());
        let mut file = File::create(format!("data/timing-{}.puml", self.move_id)).unwrap();
        const PRE: &'static str = r###"@startuml
clock "RefClock" as C0 with period 10000
concise "uSegment" as US
binary "X_ENA" as XE
binary "Y_ENA" as YE
binary "Z_ENA" as ZE
binary "E_ENA" as EE
binary "X_DIR" as XD
binary "Y_DIR" as YD
binary "Z_DIR" as ZD
binary "E_DIR" as ED
binary "X_STEP" as XS
binary "Y_STEP" as YS
binary "Z_STEP" as ZS
binary "E_STEP" as ES
scale 1000 as 50 pixels

"###;
        const POS: &'static str = r###"@enduml"###;
        file.write(PRE.as_bytes()).unwrap();
        file.write(format!(r###"@{}
US is uSeg
XE is 1
YE is 1
ZE is 1
EE is 1
XD is 0
YD is 0
ZD is 0
ED is 0
XS is 0
YS is 0
ZS is 0
ES is 0
"###, 0).as_bytes()).unwrap();

        let mut state = PinState::INIT;
        let iter = self.timings.iter();

        for t in iter {
            file.write(format!("@{}\n", t.0).as_bytes()).unwrap();
            write_state(&mut file, &mut state, &t.1, PinState::X_ENA, "XE", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::X_DIR, "XD", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::X_STEP, "XS", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::Y_ENA, "YE", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::Y_DIR, "YD", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::Y_STEP, "YS", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::Z_ENA, "ZE", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::Z_DIR, "ZD", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::Z_STEP, "ZS", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::E_ENA, "EE", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::E_DIR, "ED", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::E_STEP, "ES", "0", "1");
            write_state(&mut file, &mut state, &t.1, PinState::USCLK, "US", "uSeg", "uSeg");
        }
        file.write(POS.as_bytes()).unwrap();
        file.sync_all().unwrap();
    }

}

fn write_state(file: &mut File, state: &mut PinState, new_state: &PinState, pin: PinState, pin_name: &str, pin_low_value: &str, pin_high_value: &str) {
    let pin_set = new_state.contains(pin);
    if pin_set != state.contains(pin) {
        state.toggle(pin);
        file.write(format!("{} is {}\n", pin_name, if pin_set {pin_high_value} else {pin_low_value}).as_bytes()).unwrap();
    }
}