#[warn(missing_docs)]
mod buffer;
mod driver;
#[cfg(test)]
mod fuzz_test;
pub mod proto;
pub(crate) mod stmo_handle;
#[cfg(all(test, snare))]
mod test;
mod types;

pub use self::{
    driver::{StmoControlLoop, StreamMotionDriver},
    stmo_handle::StmoHandle,
    types::{
        AxisMotionConstraint, JointMovementLimit, JointMovementLimits, StmoStats, StmoStatsHandle,
        StreamMotionError,
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
