//! A TMC2209 model at datagram level: the drivers of the simulator.
//!
//! The UART is not simulated bit by bit (a bit-banged line between two tasks was slow and could
//! not answer reads): each write is decoded at once, and a read request queues the reply, so the
//! firmware sees a driver answering in no time. No task nor timer runs: the model only works when
//! the firmware talks to it.
//!
//! The registers the firmware uses are modelled (GCONF, CHOPCONF, GSTAT, IFCNT, DRV_STATUS; any
//! other readable register answers 0), with their power-on state. Faults are injected through
//! [crate::test_hooks] to exercise the health check of the firmware:
//! - reset: the driver goes back to its power-on state (GSTAT.reset set), as after a supply dip.
//! - fault: over-temperature (DRV_STATUS.ot); latched in GSTAT.drv_err until cleared (written 1)
//!   while the fault is no longer active.
//! - mute: the driver does not answer (nor take writes), as when disconnected or not powered.
use printhor_hwa_common as hwa;
use hwa::CoordSel;
use hwa::uart::SerialError;
use core::sync::atomic::{AtomicU8, Ordering};

/// Number of driver slots (X, Y, Z, E)
const NUM_DRIVERS: usize = 4;

/// What an idle line looks like to a read: the time without an answer before it gives up
const IDLE_LINE_MS: u64 = 2;

/// Pending reset of the driver (one-shot)
const INJECT_RESET: u8 = 0b001;
/// Over-temperature fault active
const INJECT_FAULT: u8 = 0b010;
/// The driver does not answer
const INJECT_MUTE: u8 = 0b100;

/// The injected conditions of each driver slot (see the module doc)
static INJECTED: [AtomicU8; NUM_DRIVERS] = [const { AtomicU8::new(0) }; NUM_DRIVERS];

/// Times the reset flag (GSTAT.reset) of each driver slot has been cleared: the firmware
/// configured the driver after a reset (power-on included)
static RESETS_CLEARED: [core::sync::atomic::AtomicU32; NUM_DRIVERS] =
    [const { core::sync::atomic::AtomicU32::new(0) }; NUM_DRIVERS];

/// Times the driver of `axis` has been configured after a reset (its GSTAT.reset cleared)
pub fn trinamic_resets_cleared(axis: CoordSel) -> u32 {
    slot_of(axis).map_or(0, |slot| RESETS_CLEARED[slot].load(Ordering::Acquire))
}

/// The driver slot of an axis
fn slot_of(axis: CoordSel) -> Option<usize> {
    #[cfg(feature = "with-x-axis")]
    if axis == CoordSel::X {
        return Some(0);
    }
    #[cfg(feature = "with-y-axis")]
    if axis == CoordSel::Y {
        return Some(1);
    }
    #[cfg(feature = "with-z-axis")]
    if axis == CoordSel::Z {
        return Some(2);
    }
    #[cfg(feature = "with-e-axis")]
    if axis == CoordSel::E {
        return Some(3);
    }
    None
}

/// Resets the driver of `axis` (see the module doc). Unknown axes are ignored.
pub fn trinamic_inject_reset(axis: CoordSel) {
    if let Some(slot) = slot_of(axis) {
        INJECTED[slot].fetch_or(INJECT_RESET, Ordering::AcqRel);
    }
}

/// Sets (or clears) an over-temperature fault on the driver of `axis`
pub fn trinamic_set_fault(axis: CoordSel, active: bool) {
    set_injected(axis, INJECT_FAULT, active);
}

/// Makes the driver of `axis` stop (or resume) answering
pub fn trinamic_set_mute(axis: CoordSel, active: bool) {
    set_injected(axis, INJECT_MUTE, active);
}

/// Clears every injected condition (the drivers are healthy again)
pub fn trinamic_clear_injections() {
    for flags in INJECTED.iter() {
        flags.store(0, Ordering::Release);
    }
}

fn set_injected(axis: CoordSel, flag: u8, active: bool) {
    if let Some(slot) = slot_of(axis) {
        if active {
            INJECTED[slot].fetch_or(flag, Ordering::AcqRel);
        } else {
            INJECTED[slot].fetch_and(!flag, Ordering::AcqRel);
        }
    }
}

/// GSTAT bits
const GSTAT_RESET: u32 = 0b001;
const GSTAT_DRV_ERR: u32 = 0b010;
const GSTAT_MASK: u32 = 0b111;
/// DRV_STATUS.ot
const DRV_STATUS_OT: u32 = 0b10;

/// The registers of a TMC2209
#[derive(Clone, Copy)]
struct MockedTmc2209 {
    gconf: u32,
    chopconf: u32,
    gstat: u32,
    /// Counter of the successful writes (8 bits, wraps)
    ifcnt: u8,
}

impl MockedTmc2209 {
    /// The power-on state: reset flagged
    fn power_on() -> Self {
        Self {
            gconf: tmc2209::reg::GCONF::default().into(),
            chopconf: tmc2209::reg::CHOPCONF::default().into(),
            gstat: GSTAT_RESET,
            ifcnt: 0,
        }
    }

    /// Applies the injected conditions. Returns whether the driver answers
    fn apply_injected(&mut self, slot: usize) -> bool {
        let injected = INJECTED[slot].fetch_and(!INJECT_RESET, Ordering::AcqRel);
        if injected & INJECT_RESET != 0 {
            *self = Self::power_on();
        }
        if injected & INJECT_FAULT != 0 {
            self.gstat |= GSTAT_DRV_ERR;
        }
        injected & INJECT_MUTE == 0
    }

    fn write(&mut self, slot: usize, address: u8, value: u32) {
        match tmc2209::reg::Address::try_from(address) {
            Ok(tmc2209::reg::Address::GCONF) => self.gconf = value,
            Ok(tmc2209::reg::Address::CHOPCONF) => self.chopconf = value,
            Ok(tmc2209::reg::Address::GSTAT) => {
                // Write 1 to clear. drv_err stays while the fault is active
                let mut clear = value & GSTAT_MASK;
                if INJECTED[slot].load(Ordering::Acquire) & INJECT_FAULT != 0 {
                    clear &= !GSTAT_DRV_ERR;
                }
                if self.gstat & clear & GSTAT_RESET != 0 {
                    RESETS_CLEARED[slot].fetch_add(1, Ordering::AcqRel);
                }
                self.gstat &= !clear;
            }
            // Accepted, not modelled
            Ok(_) => {}
            Err(_) => {
                hwa::warn!("[MockedTmc2209] slot {}: write to unknown register {}", slot, address);
                return;
            }
        }
        self.ifcnt = self.ifcnt.wrapping_add(1);
    }

    fn read(&self, slot: usize, address: u8) -> u32 {
        match tmc2209::reg::Address::try_from(address) {
            Ok(tmc2209::reg::Address::GCONF) => self.gconf,
            Ok(tmc2209::reg::Address::CHOPCONF) => self.chopconf,
            Ok(tmc2209::reg::Address::GSTAT) => self.gstat,
            Ok(tmc2209::reg::Address::IFCNT) => self.ifcnt as u32,
            Ok(tmc2209::reg::Address::DRV_STATUS) => {
                if INJECTED[slot].load(Ordering::Acquire) & INJECT_FAULT != 0 {
                    DRV_STATUS_OT
                } else {
                    0
                }
            }
            _ => 0,
        }
    }
}

/// The UART of the simulator: one TMC2209 per axis, on its own (single-wire) line, as the
/// MKS Robin Nano. It implements [hwa::traits::TrinamicUartTrait].
pub struct MockedTrinamicUart {
    selected: Option<usize>,
    drivers: [MockedTmc2209; NUM_DRIVERS],
    /// The reply to the last read request, still unread
    reply: [u8; tmc2209::ReadResponse::LEN_BYTES],
    reply_len: usize,
}

impl MockedTrinamicUart {
    pub fn new() -> Self {
        Self {
            selected: None,
            drivers: [MockedTmc2209::power_on(); NUM_DRIVERS],
            reply: [0; tmc2209::ReadResponse::LEN_BYTES],
            reply_len: 0,
        }
    }

    /// Decodes a datagram written to the selected driver
    fn receive(&mut self, slot: usize, bytes: &[u8]) {
        let driver = &mut self.drivers[slot];
        if !driver.apply_injected(slot) {
            return;
        }
        let len = bytes.len();
        if (len != tmc2209::WriteRequest::LEN_BYTES && len != tmc2209::ReadRequest::LEN_BYTES)
            || bytes[0] != tmc2209::SYNC_AND_RESERVED
            || bytes[len - 1] != tmc2209::crc(&bytes[..len - 1])
        {
            hwa::warn!("[MockedTmc2209] slot {}: bad datagram {:?}", slot, bytes);
            return;
        }
        let address = bytes[2] & 0x7F;
        if len == tmc2209::WriteRequest::LEN_BYTES && bytes[2] & 0x80 != 0 {
            let value = u32::from_be_bytes([bytes[3], bytes[4], bytes[5], bytes[6]]);
            hwa::trace!("[MockedTmc2209] slot {}: write [{}] = {:#x}", slot, address, value);
            driver.write(slot, address, value);
        } else if len == tmc2209::ReadRequest::LEN_BYTES {
            let value = driver.read(slot, address).to_be_bytes();
            hwa::trace!("[MockedTmc2209] slot {}: read [{}]", slot, address);
            // Reply datagram: sync, master address, register, data, crc
            self.reply = [
                tmc2209::SYNC_AND_RESERVED,
                0xFF,
                address,
                value[0],
                value[1],
                value[2],
                value[3],
                0,
            ];
            self.reply[7] = tmc2209::crc(&self.reply[..7]);
            self.reply_len = self.reply.len();
        }
    }
}

impl hwa::traits::TrinamicUartTrait for MockedTrinamicUart {
    fn select_stepper_of_axis(&mut self, axis: CoordSel) -> Result<(), ()> {
        self.reply_len = 0;
        if axis.is_empty() {
            self.selected = None;
            return Ok(());
        }
        match slot_of(axis) {
            Some(slot) => {
                self.selected = Some(slot);
                Ok(())
            }
            None => {
                hwa::error!("Invalid stepper channel selection: {:?}", axis);
                self.selected = None;
                Err(())
            }
        }
    }

    fn get_tmc_address(&self, axis: CoordSel) -> Result<u8, ()> {
        match slot_of(axis) {
            // One line per driver: every driver has the address 0
            Some(_) => Ok(0),
            None => {
                hwa::error!("Invalid stepper channel address requested: {:?}", axis);
                Err(())
            }
        }
    }

    async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, SerialError> {
        if self.selected.is_none() {
            return Err(SerialError::Framing);
        }
        if self.reply_len == 0 {
            // Nothing to read: the line stays idle (never returns without awaiting, so a caller
            // retrying until a deadline does not starve the executor)
            embassy_time::Timer::after_millis(IDLE_LINE_MS).await;
            return Err(SerialError::Timeout);
        }
        let n = self.reply_len.min(buffer.len());
        buffer[..n].copy_from_slice(&self.reply[..n]);
        self.reply.copy_within(n..self.reply_len, 0);
        self.reply_len -= n;
        Ok(n)
    }

    async fn write(&mut self, buffer: &[u8]) -> Result<(), SerialError> {
        match self.selected {
            Some(slot) => {
                self.receive(slot, buffer);
                Ok(())
            }
            None => Err(SerialError::Framing),
        }
    }

    async fn flush(&mut self) -> Result<(), SerialError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hwa::traits::TrinamicUartTrait;

    // The injected conditions are global: the tests using them run one at a time
    static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

    async fn read<R: tmc2209::ReadableRegister>(uart: &mut MockedTrinamicUart) -> Option<R> {
        uart.write(tmc2209::read_request::<R>(0).bytes()).await.ok()?;
        let mut buff = [0u8; 16];
        let n = uart.read_until_idle(&mut buff).await.ok()?;
        let mut reader = tmc2209::Reader::default();
        match reader.read_response(&buff[..n]) {
            (_, Some(response)) => response.register::<R>().ok(),
            _ => None,
        }
    }

    async fn write<R: tmc2209::WritableRegister>(uart: &mut MockedTrinamicUart, reg: R) {
        uart.write(tmc2209::WriteRequest::new(0, reg).bytes()).await.unwrap();
    }

    #[futures_test::test]
    async fn configuration_is_read_back() {
        let _guard = SERIAL.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        trinamic_clear_injections();
        let mut uart = MockedTrinamicUart::new();
        assert_eq!(uart.write(&[0u8; 4]).await, Err(SerialError::Framing));
        assert_eq!(uart.read_until_idle(&mut [0u8; 8]).await, Err(SerialError::Framing));
        assert_eq!(uart.select_stepper_of_axis(CoordSel::UNSET), Err(()));
        assert_eq!(uart.get_tmc_address(CoordSel::empty()), Err(()));
        assert_eq!(uart.select_stepper_of_axis(CoordSel::X), Ok(()));
        assert_eq!(uart.get_tmc_address(CoordSel::X), Ok(0));

        // Power-on: reset flagged
        let gstat = read::<tmc2209::reg::GSTAT>(&mut uart).await.unwrap();
        assert!(gstat.reset());

        let mut chopconf = tmc2209::reg::CHOPCONF::default();
        chopconf.set_mres(5u32.into());
        chopconf.set_dedge(true);
        write(&mut uart, chopconf).await;
        write(&mut uart, tmc2209::reg::GSTAT::from(0b111)).await;
        assert_eq!(read::<tmc2209::reg::CHOPCONF>(&mut uart).await.unwrap(), chopconf);
        assert!(!read::<tmc2209::reg::GSTAT>(&mut uart).await.unwrap().reset());
        assert_eq!(u32::from(read::<tmc2209::reg::IFCNT>(&mut uart).await.unwrap()), 2);

        // Nothing more to read: idle line
        assert_eq!(uart.read_until_idle(&mut [0u8; 8]).await, Err(SerialError::Timeout));
        // A bad datagram is ignored
        uart.write(&[1, 2, 3, 4]).await.unwrap();
        assert_eq!(uart.read_until_idle(&mut [0u8; 8]).await, Err(SerialError::Timeout));
        assert_eq!(uart.flush().await, Ok(()));
        assert_eq!(uart.select_stepper_of_axis(CoordSel::empty()), Ok(()));
    }

    #[futures_test::test]
    async fn injected_conditions() {
        let _guard = SERIAL.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        trinamic_clear_injections();
        let mut uart = MockedTrinamicUart::new();
        uart.select_stepper_of_axis(CoordSel::X).unwrap();
        write(&mut uart, tmc2209::reg::GSTAT::from(0b111)).await;

        // Reset: back to the power-on state
        trinamic_inject_reset(CoordSel::X);
        assert!(read::<tmc2209::reg::GSTAT>(&mut uart).await.unwrap().reset());
        write(&mut uart, tmc2209::reg::GSTAT::from(0b111)).await;

        // Fault: latched while active
        trinamic_set_fault(CoordSel::X, true);
        assert!(read::<tmc2209::reg::GSTAT>(&mut uart).await.unwrap().drv_err());
        assert!(read::<tmc2209::reg::DRV_STATUS>(&mut uart).await.unwrap().ot());
        write(&mut uart, tmc2209::reg::GSTAT::from(0b111)).await;
        assert!(read::<tmc2209::reg::GSTAT>(&mut uart).await.unwrap().drv_err());
        trinamic_set_fault(CoordSel::X, false);
        write(&mut uart, tmc2209::reg::GSTAT::from(0b111)).await;
        assert!(!read::<tmc2209::reg::GSTAT>(&mut uart).await.unwrap().drv_err());

        // Mute: no answer
        trinamic_set_mute(CoordSel::X, true);
        assert!(read::<tmc2209::reg::GSTAT>(&mut uart).await.is_none());
        trinamic_set_mute(CoordSel::X, false);
        assert!(read::<tmc2209::reg::GSTAT>(&mut uart).await.is_some());
        trinamic_clear_injections();
    }
}

#[cfg_attr(feature="with-trinamic", test)]
fn trinamic_proto() {
    #[allow(unused)]
    use crate as hwa;

    use tmc2209::reg::State;

    let buff1: [u8; 8] = [5, 0, 128, 0, 0, 0, 197, 41];
    let buff2: [u8; 8] = [5, 0, 236, 37, 0, 0, 83, 139];

    fn parse(buff: [u8; 8]) {
        let crc = tmc2209::crc(&buff[0..7]);
        assert_eq!(crc, buff[7], "CRC matches");
        let dt = u32::from_be_bytes([buff[3], buff[4], buff[5], buff[6]]);
        let rq = State::from_addr_and_data(
            tmc2209::reg::Address::try_from(buff[2] & 0b01111111).unwrap(), dt );
        match rq {
            State::GCONF(_) => {

                let mut reg_ref = tmc2209::reg::GCONF::default();
                reg_ref.set_shaft(false);
                reg_ref.set_pdn_disable(true);
                reg_ref.set_en_spread_cycle(true);
                reg_ref.set_mstep_reg_select(true);
                reg_ref.set_multistep_filt(false);
                let reg_val = reg_ref.0;

                let gconf = tmc2209::reg::GCONF::try_from(rq).unwrap();
                assert_eq!(gconf.shaft(), gconf.shaft());
                assert_eq!(gconf.pdn_disable(), gconf.pdn_disable());
                assert_eq!(gconf.en_spread_cycle(), gconf.en_spread_cycle());
                assert_eq!(gconf.mstep_reg_select(), gconf.mstep_reg_select());
                assert_eq!(gconf.multistep_filt(), gconf.multistep_filt());

                let gconf_val = gconf.0;
                assert_eq!(gconf_val, reg_val);

            }
            State::CHOPCONF(_x) => {
                let mut reg_ref = tmc2209::reg::CHOPCONF::default();
                reg_ref.set_intpol(false);
                reg_ref.set_dedge(true);
                reg_ref.set_mres(5u32.into());
                let reg_val = reg_ref.0;
                let chopconf = tmc2209::reg::CHOPCONF::try_from(rq).unwrap();
                assert_eq!(reg_ref.intpol(), chopconf.intpol());
                let chopconf_val = chopconf.0;
                assert_eq!(reg_val, chopconf_val);
            }
            _ => {}
        }
    }
    parse(buff1);
    parse(buff2);
}
