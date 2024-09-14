use crate::math::{Real, HALF, ONE, PI, TWO};
use core::ops::Div;
use core::ops::Neg;
#[allow(unused)]
pub trait Interpolator {
    fn interpolate(&self, tp: &Real) -> Real;
}

pub struct LinearInterpolator {
    pub(crate) x0: Real,
    pub(crate) bias: Real,
    pub(crate) slope: Real,
}

pub struct CosineInterpolator {
    pub(crate) x0: Real,
    pub(crate) heigth: Real,
    pub(crate) width: Real,
    pub(crate) bias: Real,
    #[allow(unused)]
    pub(crate) slope: Real,
}

pub struct ExpInterpolator {
    pub(crate) x0: Real,
    pub(crate) heigth: Real,
    pub(crate) width: Real,
    pub(crate) bias: Real,
    #[allow(unused)]
    pub(crate) slope: Real,
}

pub struct UniformInterpolator {
    pub(crate) value: Real,
}

impl Interpolator for UniformInterpolator {
    #[inline]
    fn interpolate(&self, _tp: &Real) -> Real {
        self.value
    }
}

impl Interpolator for LinearInterpolator {
    #[inline]
    fn interpolate(&self, tp: &Real) -> Real {
        self.bias + ((*tp - self.x0) * self.slope)
    }
}

impl Interpolator for CosineInterpolator {
    #[inline]
    fn interpolate(&self, tp: &Real) -> Real {
        self.bias + (self.heigth * (HALF - ((*tp - self.x0).div(self.width) * PI).cos().div(TWO)))
    }
}

impl Interpolator for ExpInterpolator {
    #[inline]
    fn interpolate(&self, tp: &Real) -> Real {
        let v = (Real::from_f32(12.0) * (*tp - self.x0) / self.width) - Real::from_f32(6.0);
        self.bias + self.heigth * (ONE / (ONE + v.neg().exp()))
    }
}
