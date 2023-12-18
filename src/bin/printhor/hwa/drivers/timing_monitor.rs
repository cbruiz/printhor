use std::collections::HashMap;
use itertools::Itertools;
use std::fs::File;
use std::io::Write;
use bitflags::bitflags;
use embassy_time::Instant;
use crate::hwa;

bitflags! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PinState: u32 {
        const USCLK    = 0b10000000;
        const X_ENA    = 0b00000001;
        const X_DIR    = 0b00000010;
        const X_STEP   = 0b00000100;
        const Y_ENA    = 0b00001000;
        const Y_DIR    = 0b00010000;
        const Y_STEP   = 0b00100000;
        const Z_ENA    = 0b00001000;
        const Z_DIR    = 0b00010000;
        const Z_STEP   = 0b00100000;
        const E_ENA    = 0b00001000;
        const E_DIR    = 0b00010000;
        const E_STEP   = 0b00100000;
        const INIT     = PinState::X_ENA.bits() | PinState::Y_ENA.bits() | PinState::Z_ENA.bits() | PinState::E_ENA.bits();
    }
}


/// A monitor structure to debug signal timings
pub(crate) struct TimingsMonitor {
    move_id: usize,
    ref_time: Instant,
    current_state: PinState,
    timings: HashMap<u64, PinState>,
}

impl TimingsMonitor {
    pub(crate) fn new() -> Self {
        Self {
            ref_time: Instant::now(),
            move_id: 0,
            current_state: PinState::INIT,
            timings: HashMap::new(),
        }
    }

    pub(crate) fn reset(&mut self, ref_time: embassy_time::Instant) {
        self.ref_time = ref_time;
        self.move_id += 1;
        self.current_state = PinState::INIT;
        self.timings.clear();
    }

    pub(crate) fn update(&mut self, pin: PinState, state: bool) {
        self.current_state.set(pin, state);
        let k = (embassy_time::Instant::now() - self.ref_time).as_micros() /100;
        if let Some(val) = self.timings.get_mut(&k) {
            val.set(pin, state);
        }
        else {
            self.timings.insert(k, self.current_state);
        }
    }

    pub(crate) fn swap(&mut self, pin: PinState) {
        self.current_state.toggle(pin);
        let k = (embassy_time::Instant::now() - self.ref_time).as_micros() /100;
        if let Some(val) = self.timings.get_mut(&k) {
            val.toggle(pin);
        }
        else {
            self.timings.insert(k, self.current_state);
        }
    }

    pub(crate) fn commit(&mut self) {
        hwa::info!("commit {}", self.move_id);
        hwa::info!("Plotting {} events", self.timings.len());
        let mut file = File::create(format!("data/timing-{}.puml", self.move_id)).unwrap();
        const PRE: &'static str = r###"@startuml
clock "RefClock" as C0 with period 100
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
scale 100 as 40 pixels

"###;
        const POS: &'static str = r###"@enduml"###;
        file.write(PRE.as_bytes()).unwrap();
        file.write(format!(r###"@{}
US is {{-}}
XE is 0
YE is 0
ZE is 0
EE is 0
XD is 0
YD is 0
ZD is 0
ED is 0
XS is 0
YS is 0
ZS is 0
ES is 0

"###, 0).as_bytes()).unwrap();

        let mut usegment_set = false;

        for t in self.timings.iter().sorted_by_key(|x| x.0) {
            file.write(format!(
                "@{}\
\nXE is {}\nXD is {}\nXS is {}\
\nYE is {}\nYD is {}\nYS is {}\
\nZE is {}\nZD is {}\nZS is {}\
\nEE is {}\nED is {}\nES is {}\n",
                t.0,
                if t.1.contains(PinState::X_ENA) {"1"} else {"0"},
                if t.1.contains(PinState::X_DIR) {"1"} else {"0"},
                if t.1.contains(PinState::X_STEP) {"1"} else {"0"},
                if t.1.contains(PinState::Y_ENA) {"1"} else {"0"},
                if t.1.contains(PinState::Y_DIR) {"1"} else {"0"},
                if t.1.contains(PinState::Y_STEP) {"1"} else {"0"},
                if t.1.contains(PinState::Z_ENA) {"1"} else {"0"},
                if t.1.contains(PinState::Z_DIR) {"1"} else {"0"},
                if t.1.contains(PinState::Z_STEP) {"1"} else {"0"},
                if t.1.contains(PinState::E_ENA) {"1"} else {"0"},
                if t.1.contains(PinState::E_DIR) {"1"} else {"0"},
                if t.1.contains(PinState::E_STEP) {"1"} else {"0"},

            ).as_bytes()).unwrap();
            if t.1.contains(PinState::USCLK) != usegment_set {
                file.write(format!("US is {}", if usegment_set {"E"} else {"O"}).as_bytes()).unwrap();
                usegment_set = !usegment_set;
            }
            file.write(b"\n").unwrap();
        }
        file.write(POS.as_bytes()).unwrap();
        file.sync_all().unwrap();
    }

}