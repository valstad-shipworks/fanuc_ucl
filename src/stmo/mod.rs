mod driver;
pub mod proto;
mod types;
// mod driver2;

pub use self::{
    driver::StreamMotionDriver,
    types::{AxisMotionConstraint, JointMovementLimit, JointMovementLimits, StreamMotionError},
};

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register_child_module(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        let child_module = PyModule::new(parent_module.py(), "stmo")?;
        proto::py::register(&child_module)?;
        driver::py::register(&child_module)?;

        parent_module.add_submodule(&child_module)
    }
}
