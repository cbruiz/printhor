//! Settings persistence (M500/M501) in the board non-volatile memory (see
//! [hwa::traits::AsyncEeprom]).
//!
//! The settings are stored as a compact binary image:
//!
//! | Bytes | Content                                                             |
//! |-------|---------------------------------------------------------------------|
//! | 4     | Magic (`PTHR`)                                                      |
//! | 1     | Format version                                                      |
//! | 1     | Layout flags (which optional sections are present)                  |
//! | 1     | Number of axis                                                      |
//! | 1     | Reserved (0)                                                        |
//! | 2     | Payload length (LE)                                                 |
//! | 2     | CRC-16/CCITT of the payload (LE)                                    |
//! | N     | Payload: a sequence of `f32` (LE). `NaN` means "not set"            |
//!
//! The layout depends on the enabled features (axis, heaters...). An image written by a firmware
//! with a different layout is rejected (and the defaults are kept), so a firmware upgrade never
//! applies misplaced values.
//!
//! The settings live in the usual (lock-free/atomic or short critical section) state of each
//! controller: the EEPROM is only accessed on `M500`, `M501` and at boot, so persistence adds no
//! latency to the motion or the thermal control.
use crate::{hwa, processing};
#[allow(unused)]
use hwa::math::{CoordSel, Real, TVector};
use hwa::traits::{AsyncEeprom, EepromError};
use hwa::{Contract, HwiContract};

const MAGIC: [u8; 4] = *b"PTHR";
const VERSION: u8 = 1;
const HEADER_LEN: usize = 12;
/// Exact payload length of this firmware (enabled features). The image buffers are sized with
/// it: no heap (the smaller boards have a tiny one) and no length to trust from the memory.
const PAYLOAD_LEN: usize = {
    #[allow(unused_mut)]
    let mut len = 0usize;
    #[cfg(feature = "with-motion")]
    {
        // steps per unit, max speed, max accel, max jerk, home offset; print and travel accel
        len += 5 * CoordSel::num_axis() * 4 + 2 * 4;
        // retraction: length, feed rate, z hop, recover extra, recover feed rate
        #[cfg(feature = "with-e-axis")]
        {
            len += 5 * 4;
        }
        // probe offset (M851)
        #[cfg(feature = "with-probe")]
        {
            len += CoordSel::num_axis() * 4;
        }
    }
    #[cfg(feature = "with-hot-end")]
    {
        len += 3 * 4;
    }
    #[cfg(feature = "with-hot-bed")]
    {
        len += 3 * 4;
    }
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    {
        len += 2 * 4;
    }
    len
};

#[allow(unused)]
const LAYOUT_MOTION: u8 = 1 << 0;
#[allow(unused)]
const LAYOUT_E_AXIS: u8 = 1 << 1;
#[allow(unused)]
const LAYOUT_HOT_END: u8 = 1 << 2;
#[allow(unused)]
const LAYOUT_HOT_BED: u8 = 1 << 3;
#[allow(unused)]
const LAYOUT_PROBE: u8 = 1 << 4;

const fn layout_flags() -> u8 {
    #[allow(unused_mut)]
    let mut flags = 0u8;
    #[cfg(feature = "with-motion")]
    {
        flags |= LAYOUT_MOTION;
    }
    #[cfg(feature = "with-e-axis")]
    {
        flags |= LAYOUT_E_AXIS;
    }
    #[cfg(feature = "with-hot-end")]
    {
        flags |= LAYOUT_HOT_END;
    }
    #[cfg(feature = "with-hot-bed")]
    {
        flags |= LAYOUT_HOT_BED;
    }
    #[cfg(all(feature = "with-motion", feature = "with-probe"))]
    {
        flags |= LAYOUT_PROBE;
    }
    flags
}

/// Why the settings could not be stored or retrieved
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SettingsError {
    /// The memory does not hold a settings image (blank or foreign content)
    Empty,
    /// The image was written by a firmware with a different version or settings layout
    LayoutMismatch,
    /// The image is corrupted
    CrcMismatch,
    /// The image does not fit in the memory
    TooBig,
    /// The memory failed
    Device(EepromError),
}

impl SettingsError {
    pub fn description(&self) -> &'static str {
        match self {
            SettingsError::Empty => "No stored settings",
            SettingsError::LayoutMismatch => "Stored settings version/layout mismatch",
            SettingsError::CrcMismatch => "Stored settings CRC mismatch",
            SettingsError::TooBig => "Settings do not fit in the EEPROM",
            SettingsError::Device(EepromError::OutOfBounds) => "EEPROM access out of bounds",
            SettingsError::Device(EepromError::Bus) => "EEPROM bus error",
            SettingsError::Device(EepromError::Timeout) => "EEPROM write timeout",
        }
    }
}

/// Outcome of a successful store/retrieve: (image size in bytes, CRC)
pub type SettingsInfo = (usize, u16);

/// CRC-16/CCITT-FALSE
fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for b in data {
        crc ^= (*b as u16) << 8;
        for _ in 0..8 {
            crc = match crc & 0x8000 != 0 {
                true => (crc << 1) ^ 0x1021,
                false => crc << 1,
            };
        }
    }
    crc
}

/// Writes the payload into a fixed buffer. Writes beyond its end are dropped (they can not happen:
/// [PAYLOAD_LEN] is the exact size, checked by the tests)
struct Writer {
    buf: [u8; PAYLOAD_LEN],
    len: usize,
}

#[allow(unused)]
impl Writer {
    fn f32(&mut self, v: f32) {
        if let Some(dst) = self.buf.get_mut(self.len..self.len + 4) {
            dst.copy_from_slice(&v.to_le_bytes());
        }
        self.len += 4;
    }
    fn real(&mut self, v: Option<Real>) {
        self.f32(v.map_or(f32::NAN, |x| x.to_f64() as f32));
    }
    fn vector(&mut self, v: &TVector<Real>) {
        v.foreach(|_, value| self.real(*value));
    }
}

struct Reader<'a> {
    buf: &'a [u8],
}

#[allow(unused)]
impl<'a> Reader<'a> {
    fn f32(&mut self) -> f32 {
        match self.buf.split_first_chunk::<4>() {
            Some((bytes, rest)) => {
                self.buf = rest;
                f32::from_le_bytes(*bytes)
            }
            None => f32::NAN,
        }
    }
    fn opt_f32(&mut self) -> Option<f32> {
        let v = self.f32();
        (!v.is_nan()).then_some(v)
    }
    fn real(&mut self) -> Option<Real> {
        self.opt_f32().map(Real::from_f32)
    }
    fn vector(&mut self) -> TVector<Real> {
        let mut v = TVector::new();
        TVector::<Real>::new().foreach(|coord, _| {
            v.set_coord(coord, self.real());
        });
        v
    }
}

/// The settings snapshot. Decoded completely before being applied: a corrupted image never
/// leaves the machine with half of the settings changed.
#[derive(Default)]
struct Settings {
    #[cfg(feature = "with-motion")]
    motion: Option<MotionSettings>,
    #[cfg(feature = "with-hot-end")]
    hot_end_pid: (Option<f32>, Option<f32>, Option<f32>),
    #[cfg(feature = "with-hot-bed")]
    hot_bed_pid: (Option<f32>, Option<f32>, Option<f32>),
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    cold_extrusion: (Option<f32>, Option<f32>),
}

#[cfg(feature = "with-motion")]
struct MotionSettings {
    steps_per_unit: TVector<Real>,
    max_speed: TVector<Real>,
    max_accel: TVector<Real>,
    max_jerk: TVector<Real>,
    print_accel: Option<Real>,
    travel_accel: Option<Real>,
    home_offset: TVector<Real>,
    #[cfg(feature = "with-probe")]
    probe_offset: TVector<Real>,
    #[cfg(feature = "with-e-axis")]
    retraction: [Option<Real>; 5],
}

/// Encodes the current settings. Returns the payload and the length written (`PAYLOAD_LEN`)
async fn encode(processor: &processing::GCodeProcessor) -> ([u8; PAYLOAD_LEN], usize) {
    let _ = processor;
    let mut w = Writer {
        buf: [0u8; PAYLOAD_LEN],
        len: 0,
    };
    #[cfg(feature = "with-motion")]
    {
        let cfg = processor.motion_planner.motion_config();
        w.vector(&cfg.get_steps_per_space_unit_as_vector());
        w.vector(&cfg.get_max_speed());
        w.vector(&cfg.get_max_accel());
        w.vector(&cfg.get_max_jerk());
        w.real(cfg.get_print_accel());
        w.real(cfg.get_travel_accel());
        w.vector(&cfg.get_home_offset());
        #[cfg(feature = "with-probe")]
        w.vector(&cfg.get_probe_offset());
        #[cfg(feature = "with-e-axis")]
        {
            let r = cfg.get_retraction();
            for v in [r.length, r.feed_rate, r.z_hop, r.recover_extra, r.recover_feed_rate] {
                w.real(Some(v));
            }
        }
    }
    #[cfg(feature = "with-hot-end")]
    {
        let (kp, ki, kd) = processor.hot_end.lock().await.get_pid();
        w.f32(kp);
        w.f32(ki);
        w.f32(kd);
    }
    #[cfg(feature = "with-hot-bed")]
    {
        let (kp, ki, kd) = processor.hot_bed.lock().await.get_pid();
        w.f32(kp);
        w.f32(ki);
        w.f32(kd);
    }
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    {
        let (min_temp, allowed) = super::gcode_processor::cold_extrusion_settings();
        w.f32(min_temp);
        w.f32(allowed as u8 as f32);
    }
    (w.buf, w.len)
}

fn decode(payload: &[u8]) -> Settings {
    #[allow(unused)]
    let mut r = Reader { buf: payload };
    #[allow(unused_mut)]
    let mut s = Settings::default();
    #[cfg(feature = "with-motion")]
    {
        s.motion = Some(MotionSettings {
            steps_per_unit: r.vector(),
            max_speed: r.vector(),
            max_accel: r.vector(),
            max_jerk: r.vector(),
            print_accel: r.real(),
            travel_accel: r.real(),
            home_offset: r.vector(),
            #[cfg(feature = "with-probe")]
            probe_offset: r.vector(),
            #[cfg(feature = "with-e-axis")]
            retraction: [r.real(), r.real(), r.real(), r.real(), r.real()],
        });
    }
    #[cfg(feature = "with-hot-end")]
    {
        s.hot_end_pid = (r.opt_f32(), r.opt_f32(), r.opt_f32());
    }
    #[cfg(feature = "with-hot-bed")]
    {
        s.hot_bed_pid = (r.opt_f32(), r.opt_f32(), r.opt_f32());
    }
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    {
        s.cold_extrusion = (r.opt_f32(), r.opt_f32());
    }
    s
}

async fn apply(processor: &processing::GCodeProcessor, s: Settings) {
    let _ = processor;
    #[cfg(feature = "with-motion")]
    if let Some(m) = s.motion {
        let positive = |v: &TVector<Real>| v.map_values(|_, x| x.is_defined_positive().then_some(x));
        let cfg = processor.motion_planner.motion_config();
        cfg.update_steps_per_space_unit(&m.steps_per_unit);
        cfg.update_max_speed(&positive(&m.max_speed));
        cfg.update_max_accel(&positive(&m.max_accel));
        let mut jerk = cfg.get_max_jerk();
        jerk.assign_if_set(CoordSel::all_axis(), &positive(&m.max_jerk));
        cfg.set_max_jerk(jerk);
        cfg.set_print_accel(m.print_accel.filter(|x| x.is_defined_positive()));
        cfg.set_travel_accel(m.travel_accel.filter(|x| x.is_defined_positive()));
        cfg.update_home_offset(&m.home_offset);
        #[cfg(feature = "with-probe")]
        cfg.update_probe_offset(&m.probe_offset);
        #[cfg(feature = "with-e-axis")]
        {
            let mut r = cfg.get_retraction();
            let [length, feed_rate, z_hop, recover_extra, recover_feed_rate] = m.retraction;
            r.length = length.unwrap_or(r.length);
            r.feed_rate = feed_rate.filter(|x| x.is_defined_positive()).unwrap_or(r.feed_rate);
            r.z_hop = z_hop.unwrap_or(r.z_hop);
            r.recover_extra = recover_extra.unwrap_or(r.recover_extra);
            r.recover_feed_rate = recover_feed_rate
                .filter(|x| x.is_defined_positive())
                .unwrap_or(r.recover_feed_rate);
            cfg.set_retraction(r);
        }
    }
    #[cfg(feature = "with-hot-end")]
    {
        let (kp, ki, kd) = s.hot_end_pid;
        processor.hot_end.lock().await.set_pid(kp, ki, kd);
    }
    #[cfg(feature = "with-hot-bed")]
    {
        let (kp, ki, kd) = s.hot_bed_pid;
        processor.hot_bed.lock().await.set_pid(kp, ki, kd);
    }
    #[cfg(all(feature = "with-hot-end", feature = "with-e-axis"))]
    {
        let (min_temp, allowed) = s.cold_extrusion;
        super::gcode_processor::set_cold_extrusion_settings(min_temp, allowed.map(|v| v != 0.0));
    }
}

/// Stores the current settings (M500). Unchanged EEPROM pages are not rewritten.
pub async fn store(processor: &processing::GCodeProcessor) -> Result<SettingsInfo, SettingsError> {
    let (payload, len) = encode(processor).await;
    // Can not happen ([PAYLOAD_LEN] is computed from the same features): never store a bad image
    if len != PAYLOAD_LEN {
        hwa::error!("[settings] Encoded {} bytes, expected {}", len, PAYLOAD_LEN);
        return Err(SettingsError::LayoutMismatch);
    }
    // Only the image stays alive across the EEPROM write (it is part of the future state)
    let (image, crc) = {
        let crc = crc16(&payload);
        let mut image = [0u8; HEADER_LEN + PAYLOAD_LEN];
        image[0..4].copy_from_slice(&MAGIC);
        image[4..8].copy_from_slice(&[VERSION, layout_flags(), CoordSel::num_axis() as u8, 0]);
        image[8..10].copy_from_slice(&(PAYLOAD_LEN as u16).to_le_bytes());
        image[10..12].copy_from_slice(&crc.to_le_bytes());
        image[HEADER_LEN..].copy_from_slice(&payload);
        (image, crc)
    };

    let offset = <Contract as HwiContract>::EEPROM_SETTINGS_OFFSET;
    let mut eeprom = processor.eeprom.lock().await;
    if offset + image.len() > eeprom.capacity() {
        return Err(SettingsError::TooBig);
    }
    eeprom
        .write(offset, &image)
        .await
        .map_err(SettingsError::Device)?;
    Ok((image.len(), crc))
}

/// Retrieves and applies the stored settings (M501 and boot). On error, nothing is changed.
pub async fn retrieve(
    processor: &processing::GCodeProcessor,
) -> Result<SettingsInfo, SettingsError> {
    let offset = <Contract as HwiContract>::EEPROM_SETTINGS_OFFSET;
    let mut payload = [0u8; PAYLOAD_LEN];
    {
        let mut eeprom = processor.eeprom.lock().await;
        let mut header = [0u8; HEADER_LEN];
        eeprom
            .read(offset, &mut header)
            .await
            .map_err(SettingsError::Device)?;
        if header[0..4] != MAGIC {
            return Err(SettingsError::Empty);
        }
        // The same layout flags but a different length means a different (incompatible) set of
        // settings: rejected before reading anything else
        let len = u16::from_le_bytes([header[8], header[9]]) as usize;
        if header[4] != VERSION
            || header[5] != layout_flags()
            || header[6] as usize != CoordSel::num_axis()
            || len != PAYLOAD_LEN
        {
            return Err(SettingsError::LayoutMismatch);
        }
        if offset + HEADER_LEN + PAYLOAD_LEN > eeprom.capacity() {
            return Err(SettingsError::TooBig);
        }
        let crc = u16::from_le_bytes([header[10], header[11]]);
        eeprom
            .read(offset + HEADER_LEN, &mut payload)
            .await
            .map_err(SettingsError::Device)?;
        if crc16(&payload) != crc {
            return Err(SettingsError::CrcMismatch);
        }
    }
    let crc = crc16(&payload);
    apply(processor, decode(&payload)).await;
    Ok((HEADER_LEN + PAYLOAD_LEN, crc))
}

#[cfg(test)]
mod tests {
    #[test]
    fn crc16_ccitt_false() {
        assert_eq!(super::crc16(b"123456789"), 0x29B1);
    }
}
