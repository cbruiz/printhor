//! Cartessian kinematics: Identity transform, basically.
use crate::kinematics::WorldToSpaceTransformer;

pub struct Identity;
impl WorldToSpaceTransformer for Identity {}

pub use Identity as DefaultTransformer;
