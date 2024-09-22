cfg_if::cfg_if! {
    if #[cfg(feature="float-point-f32-impl")] {
        //use core::f32;
        use crate::math::Real;
        pub const ZERO: Real = Real::from_f32(0.0);
        pub const HALF: Real = Real::from_f32(0.5);
        pub const ONE_AND_HALF: Real = Real::from_f32(1.5);
        pub const ONE: Real = Real::from_f32(1.0);
        pub const TWO: Real = Real::from_f32(2.0);
        pub const THREE: Real = Real::from_f32(3.0);
        pub const FOUR: Real = Real::from_f32(4.0);
        pub const SIX: Real = Real::from_f32(6.0);
        pub const SIXTH: Real = Real::from_f32(1.0 / 6.0);

        /// A very small value used to determine the difference between two floating point numbers.
        /// This constant is useful for comparisons to handle floating point negligible differences.
        pub const EPSILON: Real = Real::from_f32(f32::EPSILON);

        #[allow(unused)]
        pub const ONE_HUNDRED: Real = Real::from_f32(100.0);
        #[allow(unused)]
        pub const ONE_THOUSAND: Real = Real::from_f32(1000.0);
        #[allow(unused)]
        pub const ONE_MILLION: Real = Real::from_f32(1000000.0);
        #[allow(unused)]
        pub const PI: Real = Real::from_f32(core::f32::consts::PI);
    }
}
