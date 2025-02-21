use crate as hwa;
use hwa::CoordSel;
use hwa::kinematics::WorldToSpaceTransformer;
use hwa::math::{Real, TVector};

pub struct CoreXYTransformer;
impl WorldToSpaceTransformer for CoreXYTransformer {
    fn project_to_space(&self, _world_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))] {
                let mut space_pos = _world_pos.clone();
                let _x = _world_pos.get_coord(CoordSel::X);
                let _y = _world_pos.get_coord(CoordSel::Y);
                if let Some(x) = _x {
                    if let Some(y) = _y {
                        space_pos.set_coord(CoordSel::X, Some(x + y));
                        space_pos.set_coord(CoordSel::Y, Some(x - y));
                        return Ok(space_pos)
                    }
                }
            }
            else {
                compile_error!("CoreXY requires with-x-axis and with-y-axis");
            }
        }
        Err(())
    }

    fn project_to_world(&self, _space_coordinate: &TVector<Real>) -> Result<TVector<Real>, ()> {
        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))] {
                let mut world_pos = _space_coordinate.clone();
                let _x = _space_coordinate.get_coord(CoordSel::X);
                let _y = _space_coordinate.get_coord(CoordSel::Y);
                if let Some(x) = _x {
                    if let Some(y) = _y {
                        world_pos.set_coord(CoordSel::X, Some((x + y) / hwa::math::TWO));
                        world_pos.set_coord(CoordSel::Y, Some((x - y) / hwa::math::TWO));
                        return Ok(world_pos)
                    }
                }
            }
            else {
                compile_error!("CoreXY requires with-x-axis and with-y-axis");
            }
        }
        Err(())
    }
}

pub use CoreXYTransformer as DefaultTransformer;
