// Math module
#[path = "../../src/bin/printhor/math/mod.rs"]
mod core_math;
pub mod math {
    pub use super::core_math::*;
}
pub use math::Real;

// TGeo Module
#[path = "../../src/bin/printhor/tgeo.rs"]
mod core_tgeo;
pub mod tgeo {
    pub use super::core_tgeo::*;
}

#[path = "../../src/bin/printhor/helpers/mod.rs"]
mod helpers;
#[path = "../../src/bin/printhor/machine.rs"]
mod machine;
#[path = "../../src/bin/printhor/control/motion_planning/mod.rs"]
mod core_motion_planing;

#[path = "../../src/bin/printhor/hwa/controllers/motion/motion_segment.rs"]
mod core_motion_segment;

pub mod control {

    pub use super::core_control_base::*;

    pub mod motion_planning {
        pub use crate::prelude::core_motion_planing::Constraints;
        pub use crate::prelude::core_motion_planing::Boundaries;
        pub use crate::prelude::core_motion_planing::SCurveMotionProfile;
        pub use crate::prelude::core_motion_planing::Cache;
        pub use crate::prelude::core_motion_planing::MotionProfile;
    }
}

#[path = "../../src/bin/printhor/control/base.rs"]
mod core_control_base;

pub mod hwa {
    pub use log::*;
    pub use printhor_hwa_common::*;

    pub const MACHINE_TYPE: &str = "Simulator/debugger";
    pub const MACHINE_BOARD: &str = "PC";
    pub const MACHINE_PROCESSOR: &str = std::env::consts::ARCH;

    pub mod controllers {
        pub mod motion {
            pub use crate::prelude::core_motion_planing::*;
            pub use crate::prelude::core_motion_segment::*;
        }
    }
}

pub use std::alloc;