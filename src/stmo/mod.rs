#[warn(missing_docs)]
mod buffer;
mod driver;
pub mod proto;
pub(crate) mod stmo_handle;
#[cfg(test)]
mod test;
mod tx_errqueue;
mod types;

pub use self::{
    driver::{StmoControlLoop, StreamMotionDriver},
    stmo_handle::StmoHandle,
    types::{
        AxisMotionConstraint, JointMovementLimit, JointMovementLimits, StmoStats, StreamMotionError,
    },
};

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register_child_module(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        let child_module = PyModule::new(parent_module.py(), "stmo")?;
        proto::py::register(&child_module)?;
        driver::py::register(&child_module)?;
        stmo_handle::py::register(&child_module)?;

        parent_module.add_submodule(&child_module)
    }
}
