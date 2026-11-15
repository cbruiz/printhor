use crate::math;

use crate as hwa;
use micromath::F32Ext;
use num_traits::ToPrimitive;
use num_traits::Zero;
use num_traits::float::FloatCore;


/// Soft-float operation counters (feature `op-count`): a host measuring aid. On a Cortex-M0+
/// without FPU (SKR Mini E3) every float operation is a library call, so their (weighted) count
/// estimates the cost of an algorithm there without flashing it. Zero cost without the feature.
#[cfg(feature = "op-count")]
pub mod op_count {
    use core::sync::atomic::{AtomicU64, Ordering};
    pub const ADD: usize = 0;
    pub const MUL: usize = 1;
    pub const DIV: usize = 2;
    pub const CMP: usize = 3;
    pub const SQRT: usize = 4;
    pub const CBRT: usize = 5;
    pub const CONV: usize = 6;
    pub const MINMAX: usize = 7;
    pub const MISC: usize = 8;
    pub const NAMES: [&str; 9] = ["add", "mul", "div", "cmp", "sqrt", "cbrt", "conv", "minmax", "misc"];
    /// µs per operation on the SKR Mini E3 (64 MHz Cortex-M0+, `M801`, loop overhead removed).
    /// `sqrt` without the comparisons it counts itself
    pub const COST_US: [f64; 9] = [3.07, 3.06, 4.25, 1.0, 16.5, 144.0, 3.0, 1.5, 3.0];
    static COUNTS: [AtomicU64; 9] = [const { AtomicU64::new(0) }; 9];
    #[inline(always)]
    pub fn hit(k: usize) {
        COUNTS[k].fetch_add(1, Ordering::Relaxed);
    }
    pub fn reset() {
        for c in COUNTS.iter() {
            c.store(0, Ordering::Relaxed);
        }
    }
    pub fn get() -> [u64; 9] {
        core::array::from_fn(|i| COUNTS[i].load(Ordering::Relaxed))
    }
    /// Estimated µs on the SKR Mini E3
    pub fn cost_us(c: &[u64; 9]) -> f64 {
        c.iter().zip(COST_US.iter()).map(|(n, w)| *n as f64 * w).sum()
    }
}

#[cfg(feature = "op-count")]
macro_rules! hit {
    ($k:ident) => {
        op_count::hit(op_count::$k)
    };
}
#[cfg(not(feature = "op-count"))]
macro_rules! hit {
    ($k:ident) => {};
}

#[derive(Copy, Clone, Default)]
pub struct Real(pub f32);
pub type RealImpl = f32;

impl Real {
    pub fn new(num: i64, scale: u32) -> Self {
        Self::from_lit(num, scale)
    }

    pub const fn from_inner(num: f32) -> Self {
        Self(num)
    }

    /// Powers of ten that are exact in `f32` (`10^k = 2^k·5^k`, `5^10 < 2^24`): the same values
    /// `powi` produces, without its loop.
    const POW10: [f32; 11] = [1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10];

    /// `num / 10^scale`. Hot path (G-code literals, loop counters, µs → s): `scale == 0` is a
    /// plain conversion and small scales use an exact table, bit-identical to `powi`.
    pub fn from_lit(num: i64, scale: u32) -> Self {
        hit!(CONV);
        if scale == 0 {
            return Self(num as f32);
        }
        let s = match Self::POW10.get(scale as usize) {
            Some(p) => *p,
            None => FloatCore::powi(10.0f32, scale as i32),
        };
        Self((num as f32) / s)
    }

    pub const fn from_f32(num: f32) -> Self {
        Self(num)
    }

    pub fn inner(&self) -> f32 {
        self.0
    }

    pub fn abs(self) -> Self {
        Self(FloatCore::abs(self.0))
    }

    pub fn powi(self, x: i32) -> Self {
        Self(FloatCore::powi(self.0, x))
    }

    pub fn recip(self) -> Self {
        hit!(DIV);
        Self(FloatCore::recip(self.0))
    }

    pub const fn zero() -> Self {
        Self(0.0f32)
    }

    pub fn is_zero(&self) -> bool {
        hit!(CMP);
        f32::is_zero(&self.0)
    }

    pub const fn epsilon() -> Self {
        crate::math::EPSILON
    }

    pub fn is_negligible(&self) -> bool {
        hit!(CMP);
        FloatCore::abs(self.0) < <f32 as FloatCore>::epsilon()
    }

    pub fn is_defined_positive(&self) -> bool {
        hit!(CMP);
        self.0 > f32::zero()
    }

    pub fn is_positive(&self) -> bool {
        !self.0.is_sign_negative()
    }

    pub const fn one() -> Self {
        Self(1.0f32)
    }

    pub fn round(&self) -> Self {
        hit!(MISC);
        Real(FloatCore::round(self.0))
    }

    pub fn round_dp(&self, decimals: u32) -> Self {
        let s = FloatCore::powi(10.0f32, decimals as i32);
        Self(FloatCore::round(self.0 * s) / s)
    }
    /// Round to decimal digits

    pub fn rdp(&self, digits: u32) -> Self {
        self.round_dp(digits)
    }

    pub fn ceil(&self) -> Self {
        hit!(MISC);
        Real(FloatCore::ceil(self.0))
    }

    pub fn floor(&self) -> Self {
        hit!(MISC);
        Real(FloatCore::floor(self.0))
    }

    pub fn to_f64(&self) -> f64 {
        hit!(CONV);
        self.0 as f64
    }

    pub fn to_i32(&self) -> Option<i32> {
        hit!(CONV);
        self.0.to_i32()
    }

    pub fn to_i64(&self) -> Option<i64> {
        hit!(CONV);
        self.0.to_i64()
    }

    /// Truncated integer part. Never panics: saturates out of range and NaN is 0 (as `as` does)
    pub fn int(&self) -> i64 {
        hit!(CONV);
        self.0 as i64
    }

    /// Radians to degrees
    pub fn r2d(&self) -> Real {
        (*self) * hwa::make_real!(180.0) / hwa::math::PI
    }

    /// Degrees to radians
    /// 360 -> 2pi
    /// x ->
    pub fn d2r(&self) -> Real {
        (*self) * hwa::math::PI / hwa::make_real!(180.0)
    }

    pub fn sqrt(self) -> Option<Self> {
        hit!(SQRT);
        if self < math::ZERO {
            None
        } else if self == math::ZERO {
            Some(math::ZERO)
        } else {
            // y ≈ 1/sqrt(x) (fast inverse square root + 3 Newton iterations).
            // sqrt(x) = x·y avoids the costly division of the former `1/y`, and a final
            // division-free Markstein/Heron correction `r += ½·y·(x - r²)` brings the
            // result to ≤ 1 ulp (the uncorrected estimate drifts a few ulps).
            // Subnormals defeat the exponent-based seed: scale them by an even power of two
            // (exact) and undo it on the result (sqrt(x·4^k) = sqrt(x)·2^k)
            let (x, unscale) = if self.0 < f32::MIN_POSITIVE {
                (self.0 * 16777216.0f32, 1.0 / 4096.0f32)
            } else {
                (self.0, 1.0)
            };
            // Without FPU (ARM soft-float ABI: Cortex-M0+), each float operation is a library
            // call (~4 µs on a 64 MHz M0+): the ~20 of the approximation below cost ~85 µs. The
            // integer bit-by-bit square root costs ~5 µs there and is correctly rounded.
            #[cfg(all(target_arch = "arm", target_abi = "eabi"))]
            let r = {
                let _ = Self::quake_isqrt;
                Real::from_inner(Self::sqrt_bitwise(x) * unscale)
            };
            // With FPU (ARM hard-float ABI: Cortex-M4F), the IEEE (correctly rounded) instruction
            #[cfg(all(target_arch = "arm", target_abi = "eabihf"))]
            let r = {
                let _ = Self::quake_isqrt;
                let root: f32;
                // SAFETY: a register-only floating point instruction
                unsafe {
                    core::arch::asm!(
                        "vsqrt.f32 {0}, {1}",
                        out(sreg) root,
                        in(sreg) x,
                        options(pure, nomem, nostack)
                    );
                }
                Real::from_inner(root * unscale)
            };
            #[cfg(not(target_arch = "arm"))]
            let r = {
                let y = Self::quake_isqrt(x);
                let r0 = x * y;
                Real::from_inner((r0 + 0.5f32 * y * (x - r0 * r0)) * unscale)
            };
            if self < math::EPSILON {
                hwa::trace!("Very small sqrt calculation: sqrt({:?}) = {:?} ", self.0, r);
            }
            Some(r)
        }
    }

    /// Correctly rounded square root of a positive normal `x` with integer operations only
    /// (fdlibm `e_sqrtf.c`, round to nearest): the fast path on targets without FPU.
    pub fn sqrt_bitwise(x: f32) -> f32 {
        if !(x > 0.0) || !x.is_finite() {
            return x;
        }
        let mut ix = x.to_bits() as i32;
        let mut m = ix >> 23;
        if m == 0 {
            // Subnormal: normalize
            let mut i = 0;
            while (ix & 0x0080_0000) == 0 {
                ix <<= 1;
                i += 1;
            }
            m -= i - 1;
        }
        m -= 127;
        ix = (ix & 0x007f_ffff) | 0x0080_0000;
        if (m & 1) != 0 {
            ix += ix;
        }
        m >>= 1;
        ix += ix;
        let mut q: i32 = 0;
        let mut s: i32 = 0;
        let mut r: i32 = 0x0100_0000;
        while r != 0 {
            let t = s + r;
            if t <= ix {
                s = t + r;
                ix -= t;
                q += r;
            }
            ix += ix;
            r >>= 1;
        }
        if ix != 0 {
            q += q & 1;
        }
        let bits = (q >> 1) + 0x3f00_0000 + (m << 23);
        f32::from_bits(bits as u32)
    }

    /// The famous inverse square root approximation of 'ID Software'
    /// Hard-coded with 3 iterations
    fn quake_isqrt(number: f32) -> f32 {
        let xhalf = number * 0.5f32;
        let mut y =
            f32::from_bits(0x5F375A86_i32.wrapping_sub(number.to_bits() as i32 >> 1) as u32);
        y = y * (1.5f32 - xhalf * y * y);
        y = y * (1.5f32 - xhalf * y * y);
        y = y * (1.5f32 - xhalf * y * y);
        y
    }

    // cbrt implementation from rust libm
    // Warning: This method deviates too much for large values
    pub fn libm_cbrt(x: f32) -> f32 {
        const B1: u32 = 709958130; /* B1 = (127-127.0/3-0.03306235651)*2**23 */
        const B2: u32 = 642849266; /* B2 = (127-127.0/3-24/3-0.03306235651)*2**23 */

        let x1p24 = f32::from_bits(0x4b800000); // 0x1p24f === 2 ^ 24

        let mut ui: u32 = x.to_bits();
        let mut hx: u32 = ui & 0x7fffffff;

        if hx >= 0x7f800000 {
            /* cbrt(NaN,INF) is itself */
            return x + x;
        }

        /* rough cbrt to 5 bits */
        if hx < 0x00800000 {
            /* zero or subnormal? */
            if hx == 0 {
                return x; /* cbrt(+-0) is itself */
            }
            ui = (x * x1p24).to_bits();
            hx = ui & 0x7fffffff;
            hx = hx / 3 + B2;
        } else {
            hx = hx / 3 + B1;
        }
        ui &= 0x80000000;
        ui |= hx;

        /*
         * First step Newton iteration
         */
        // The Newton steps are performed in f64 (as libm's cbrtf does): in f32, `t³` and `2x + t³`
        // under/overflow for very small/large magnitudes
        let xd = x as f64;
        let mut t = f32::from_bits(ui) as f64;
        let mut r = t * t * t;
        t = t * (xd + xd + r) / (xd + r + r);

        /*
         * Second step Newton iteration
         */
        r = t * t * t;
        t = t * (xd + xd + r) / (xd + r + r);

        t as f32
    }

    /// Computes the cube root of the argument.
    pub fn cbrt(self) -> Self {
        hit!(CBRT);
        let r = Real::from_inner(Real::libm_cbrt(self.0));
        //let r = Real::from_inner(self.0.cbrt())
        //let r = Real::from_inner(micromath::F32::powf(self.0.into(), micromath::F32::from(1.0f32/3.0f32)).into());
        //hwa::info!("cbrt calculation: cbrt({:?}) = {}", self, r.0);
        r
    }

    pub fn sin(self) -> Self {
        Real(self.0.sin())
    }

    pub fn cos(self) -> Self {
        Real(self.0.cos())
    }

    pub fn tan(self) -> Self {
        Real(self.0.tan())
    }

    /// Computes the four quadrant arctangent of self (y) and other (x) in radians.
    pub fn atan2(self, other: Real) -> Self {
        Real(self.0.atan2(other.0))
    }

    /// Computes the arccosine of a number. Return value is in radians in the range [0, pi] or NaN if the number is outside the range [-1, 1].
    pub fn acos(self) -> Self {
        Real(self.0.acos())
    }

    pub fn ln(self) -> Self {
        Real(self.0.ln())
    }

    pub fn exp(self) -> Self {
        Real(self.0.exp())
    }

    pub fn sign(self) -> Self {
        let s = F32Ext::signum(self.0);
        if s.is_zero() { Real::one() } else { Real(s) }
    }

    pub fn vmin(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
        let mut m: Option<Real> = None;
        if let Some(x) = r1 {
            m = Some(x);
        }
        if let Some(y) = r2 {
            if let Some(mr) = &m {
                if mr.gt(&y) {
                    m = Some(y)
                }
            }
        }
        m
    }

    pub fn vmax(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
        let mut m: Option<Real> = None;
        if let Some(x) = r1 {
            m = Some(x);
        }
        if let Some(y) = r2 {
            if let Some(mr) = &m {
                if mr.lt(&y) {
                    m = Some(y)
                }
            }
        }
        m
    }
}

impl core::ops::Add for Real {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        hit!(ADD);
        Real(self.0.add(rhs.0))
    }
}

impl core::ops::AddAssign for Real {
    fn add_assign(&mut self, rhs: Self) {
        hit!(ADD);
        self.0.add_assign(rhs.0)
    }
}

impl core::ops::Sub for Real {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        hit!(ADD);
        Real(self.0.sub(rhs.0))
    }
}

impl core::ops::SubAssign for Real {
    fn sub_assign(&mut self, rhs: Self) {
        hit!(ADD);
        self.0.sub_assign(rhs.0)
    }
}

impl core::ops::Mul for Real {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        hit!(MUL);
        Real(self.0.mul(rhs.0))
    }
}

impl core::ops::MulAssign for Real {
    fn mul_assign(&mut self, rhs: Self) {
        hit!(MUL);
        self.0.mul_assign(rhs.0)
    }
}

impl core::ops::Div for Real {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        hit!(DIV);
        Real(self.0.div(rhs.0))
    }
}

impl core::ops::DivAssign for Real {
    fn div_assign(&mut self, rhs: Self) {
        hit!(DIV);
        self.0.div_assign(rhs.0)
    }
}

impl PartialOrd<Self> for Real {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        hit!(CMP);
        self.0.partial_cmp(&other.0)
    }

    fn lt(&self, other: &Self) -> bool {
        hit!(CMP);
        self.0.lt(&other.0)
    }

    fn le(&self, other: &Self) -> bool {
        hit!(CMP);
        self.0.le(&other.0)
    }

    fn gt(&self, other: &Self) -> bool {
        hit!(CMP);
        self.0.gt(&other.0)
    }

    fn ge(&self, other: &Self) -> bool {
        hit!(CMP);
        self.0.ge(&other.0)
    }
}

impl PartialEq<Self> for Real {
    fn eq(&self, other: &Self) -> bool {
        hit!(CMP);
        self.0.eq(&other.0)
    }
}

impl Eq for Real {}

impl Ord for Real {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        hit!(CMP);
        self.0.total_cmp(&other.0)
    }

    fn max(self, other: Self) -> Self
    where
        Self: Sized,
    {
        hit!(MINMAX);
        Real(self.0.max(other.0))
    }

    fn min(self, other: Self) -> Self
    where
        Self: Sized,
    {
        hit!(MINMAX);
        Real(self.0.min(other.0))
    }

    fn clamp(self, min: Self, max: Self) -> Self
    where
        Self: Sized,
        Self: PartialOrd,
    {
        hit!(MINMAX);
        hit!(MINMAX);
        // Not f32::clamp: its (formatted) assert links core's float formatting
        Real(self.0.max(min.0).min(max.0))
    }
}

impl core::ops::Neg for Real {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Real(self.0.neg())
    }
}

impl core::fmt::Debug for Real {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut buffer = [0u8; 24];
        f.write_str(format_f32(self.0, &mut buffer))
    }
}

/// Writes `x` as ryu does (`80.0`, `0.1`, `-1.5`, `1e-7`, `NaN`, `inf`), without its code and
/// tables (~4 KB of flash): the shortest decimal that reads back as `x`, found with exact integer
/// arithmetic on the mantissa.
///
/// Identical to ryu for every value in [1e-4, 1e13) and for every decimal of up to 7 significant
/// digits (any value typed in a G-code) from 1e-6. Below 1e-4 it may write fewer digits (12
/// decimals at most) and the scientific form (below 1e-6 or from 1e13) is approximate (7 digits).
fn format_f32(x: f32, buf: &mut [u8; 24]) -> &str {
    let mut w = Writer { buf: &mut buf[..], len: 0 };
    write_f32(x, &mut w);
    let len = w.len;
    core::str::from_utf8(&buf[..len]).unwrap_or("")
}

fn write_f32(x: f32, w: &mut Writer<'_>) {
    let bits = x.to_bits();
    if x.is_nan() {
        w.push(b"NaN");
        return;
    }
    if bits >> 31 != 0 {
        w.push(b"-");
    }
    let (exp_bits, frac) = (((bits >> 23) & 0xFF) as i32, (bits & 0x007F_FFFF) as u64);
    if exp_bits == 0xFF {
        w.push(b"inf");
        return;
    }
    if exp_bits == 0 && frac == 0 {
        w.push(b"0.0");
        return;
    }
    // |x| = m·2^e2, m < 2^24
    let (m, e2) = if exp_bits == 0 { (frac, -149) } else { (frac | 0x0080_0000, exp_bits - 150) };
    let abs = f32::from_bits(bits & 0x7FFF_FFFF);
    if abs < 1e-6 || abs >= 1e13 {
        // Scientific (ryu does it out of this range). The 7 digits are approximate here
        let (mut y, mut exp10) = (abs, 0i32);
        while y >= 10.0 {
            y /= 10.0;
            exp10 += 1;
        }
        while y < 1.0 {
            y *= 10.0;
            exp10 -= 1;
        }
        let mut n = (y * 1e6 + 0.5) as u64;
        if n >= 10_000_000 {
            n /= 10;
            exp10 += 1;
        }
        w.decimal(n, 6, false);
        w.push(b"e");
        if exp10 < 0 {
            w.push(b"-");
        }
        w.decimal(exp10.unsigned_abs() as u64, 0, false);
        return;
    }
    // n = round(|x|·10^p) with 7 significant digits (fewer when the integer part is longer)
    let scaled = |p: u32| -> u64 {
        let v = m * 10u64.pow(p);
        if e2 >= 0 {
            v << e2
        } else {
            let s = (-e2) as u32;
            if s >= 64 { 0 } else { (v + (1u64 << (s - 1))) >> s }
        }
    };
    // Digits of the integer part (truncated)
    let int_part = if e2 >= 0 { m << e2 } else if e2 > -64 { m >> -e2 } else { 0 };
    let int_digits = digits(int_part);
    if e2 >= 0 {
        // An integer (|x| >= 2^23): its shortest form that reads back as `x` (within half an
        // ulp; a quarter below a power of two), padded with zeros
        let half_ulp = (1u64 << e2) / if m == 0x0080_0000 { 4 } else { 2 };
        let mut j = int_digits - 1;
        loop {
            // NonZero: the divisions have no panic path
            let unit = core::num::NonZeroU64::new(10u64.pow(j)).unwrap_or(core::num::NonZeroU64::MIN);
            let c = (int_part + unit.get() / 2) / unit;
            let diff = (c * unit.get()).abs_diff(int_part);
            if j == 0 || diff < half_ulp || (diff == half_ulp && m & 1 == 0) {
                w.decimal(c, 0, false);
                for _ in 0..j {
                    w.push(b"0");
                }
                w.push(b".0");
                return;
            }
            j -= 1;
        }
    }
    // n = round(|x|·10^p) with the 9 significant digits that tell any two f32 apart. |x| < 1:
    // as many more decimals as leading zeros, 12 at most (m·10^12 < 2^64), so 7 digits at 1e-6
    let mut p = 9 - int_digits;
    let mut n = scaled(p);
    while int_digits == 0 && n < 100_000_000 && p < 12 {
        p += 1;
        n = scaled(p);
    }
    if e2 < 0 {
        // The shortest decimal (then the nearest) that reads back as `x`: within half an ulp of
        // it (a quarter below a power of two). In units of 2^e2/10^p: x is `xs`, an ulp `10^p`
        let (s, xs) = ((-e2) as u32, m * 10u64.pow(p));
        let half_ulp = 10u64.pow(p) / if m == 0x0080_0000 { 4 } else { 2 };
        let floor_n = if s < 64 { xs >> s } else { 0 };
        for j in (1..=p.min(8)).rev() {
            // NonZero: the divisions have no panic path
            let unit = core::num::NonZeroU64::new(10u64.pow(j)).unwrap_or(core::num::NonZeroU64::MIN);
            let dist = |c: u64| {
                let v = c * unit.get();
                v.checked_shl(s).filter(|w| w >> s == v).map(|w| w.abs_diff(xs))
            };
            // The two candidates around x: floor(x / unit) and the next one (ties to even, as ryu)
            let lo = floor_n / unit;
            let (c, diff) = match (dist(lo), dist(lo + 1)) {
                (Some(d_lo), Some(d_hi)) if d_hi < d_lo || (d_hi == d_lo && lo & 1 == 1) => (lo + 1, d_hi),
                (Some(d_lo), _) => (lo, d_lo),
                (None, _) => continue,
            };
            if diff < half_ulp || (diff == half_ulp && m & 1 == 0) {
                w.decimal(c, p - j, true);
                return;
            }
        }
    }
    w.decimal(n, p, true);
}

fn digits(mut v: u64) -> u32 {
    let mut d = 0;
    while v > 0 {
        v /= 10;
        d += 1;
    }
    d
}

struct Writer<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Writer<'_> {
    fn push(&mut self, s: &[u8]) {
        for &b in s {
            if let Some(slot) = self.buf.get_mut(self.len) {
                *slot = b;
                self.len += 1;
            }
        }
    }

    /// `n / 10^p`, then the decimals of `n % 10^p` without trailing zeros. With `dot0`, a zero
    /// fraction is written as `.0` (else nothing)
    fn decimal(&mut self, n: u64, p: u32, dot0: bool) {
        // Written backwards (from the last digit), without indexing (no panic paths)
        let mut tmp = [0u8; 21];
        let mut i = tmp.len();
        let mut put = |b: u8| {
            if let Some(slot) = i.checked_sub(1).and_then(|k| tmp.get_mut(k)) {
                *slot = b;
                i -= 1;
            }
        };
        let mut v = n;
        let mut pos = 0;
        let mut significant = false;
        while v > 0 || pos <= p {
            let d = (v % 10) as u8;
            v /= 10;
            // The trailing zeros of the fraction are dropped
            if pos >= p || significant || d != 0 {
                significant |= pos < p;
                put(b'0' + d);
            }
            pos += 1;
            if pos == p && significant {
                put(b'.');
            }
        }
        self.push(tmp.get(i..).unwrap_or(&[]));
        if !significant && dot0 {
            self.push(b".0");
        }
    }
}

#[cfg(test)]
mod format_tests {
    use super::format_f32;

    fn fmt(x: f32) -> std::string::String {
        let mut b = [0u8; 24];
        std::string::String::from(format_f32(x, &mut b))
    }

    #[test]
    fn as_ryu() {
        for (x, s) in [
            (0.0, "0.0"), (-0.0, "-0.0"), (80.0, "80.0"), (0.1, "0.1"), (-1.5, "-1.5"),
            (21.73, "21.73"), (0.001, "0.001"), (1e-5, "0.00001"), (123456.7, "123456.7"),
            (9999999.0, "9999999.0"), (16777216.0, "16777216.0"), (1e12, "1000000000000.0"),
            (1.0 / 3.0, "0.33333334"), (0.294921875, "0.29492188"), (0.345703125, "0.34570312"),
            (1e-7, "1e-7"), (1.5e20, "1.5e20"), (f32::NAN, "NaN"), (f32::INFINITY, "inf"),
            (f32::NEG_INFINITY, "-inf"),
        ] {
            assert_eq!(fmt(x), s, "{:e}", x);
        }
    }

    /// In [1e-4, 1e13): reads back as `x` and it is as short as std's Debug (which also writes
    /// the shortest decimal, but breaks exact ties up where ryu breaks them to even)
    #[test]
    fn shortest_round_trip() {
        let digits = |s: &str| s.bytes().filter(|b| b.is_ascii_digit()).count();
        let mut bits = 1e-4f32.to_bits();
        while bits < 1e13f32.to_bits() {
            let x = f32::from_bits(bits);
            let (got, std_fmt) = (fmt(x), std::format!("{:?}", x));
            assert_eq!(got.parse::<f32>(), Ok(x), "{}", got);
            assert_eq!(digits(&got), digits(&std_fmt), "{} {}", got, std_fmt);
            assert_eq!(fmt(-x), std::format!("-{}", got));
            bits += 9973;
        }
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for Real {
    fn format(&self, _f: defmt::Formatter) {
        defmt::write!(_f, "{}", self.0);
    }
}
