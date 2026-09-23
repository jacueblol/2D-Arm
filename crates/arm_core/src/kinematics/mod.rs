pub mod fk;
pub mod ik;

pub use fk::forward_kinematics;
pub use ik::{ElbowConfig, IkSolution, IkSolutions, solve, solve_3d};
