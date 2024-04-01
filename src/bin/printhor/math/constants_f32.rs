cfg_if::cfg_if! {
    if #[cfg(feature="float-point-f32-impl")] {
        use core::f32;
        use crate::math::Real;
        pub const ZERO: Real = Real::from_f32(0.0f32);
        pub const HALF: Real = Real::from_f32(0.5f32);
        pub const ONE_AND_HALF: Real = Real::from_f32(1.5f32);
        pub const ONE: Real = Real::from_f32(1.032);
        pub const TWO: Real = Real::from_f32(2.032);
        pub const THREE: Real = Real::from_f32(3.032);
        pub const FOUR: Real = Real::from_f32(4.032);
        pub const SIX: Real = Real::from_f32(6.032);
        pub const SIXTH: Real = Real::from_f32(1.0f32 / 6.0f32);
        pub const EPSILON: Real = Real::from_f32(f32::EPSILON);

        #[allow(unused)]
        pub const ONE_HUNDRED: Real = Real::from_f32(100.0f32);
        #[allow(unused)]
        pub const ONE_THOUSAND: Real = Real::from_f32(1000.0);
        #[allow(unused)]
        pub const ONE_MILLION: Real = Real::from_f32(1000000.0);
        #[allow(unused)]
        pub const PI: Real = Real::from_f32(f32::consts::PI);
    }
}

