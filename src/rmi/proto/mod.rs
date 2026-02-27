pub mod commands;
pub mod communication;
pub mod instructions;
pub mod member_structs;

#[cfg(not(feature = "py"))]
pub use commands::*;
#[cfg(not(feature = "py"))]
pub use communication::*;
#[cfg(not(feature = "py"))]
pub use instructions::*;
#[cfg(not(feature = "py"))]
pub use member_structs::*;

#[macro_export]
#[doc(hidden)]
macro_rules! zst_filler {
    ($name:ident) => {
        #[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
        #[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default)]
        pub struct $name;

        #[cfg_mixin::cfg_mixin(feature = "py")]
        #[cfg_attr(feature = "py", pyo3::pymethods)]
        impl $name {
            #[on(new)]
            pub fn new() -> Self {
                Self {}
            }
        }
    };
}
