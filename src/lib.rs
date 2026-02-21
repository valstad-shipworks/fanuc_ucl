#![doc = include_str!("../README.md")]
// #![deny(clippy::all, clippy::pedantic, clippy::nursery)]
// #![deny(
//     missing_copy_implementations,
//     single_use_lifetimes,
//     variant_size_differences,
//     arithmetic_overflow,
//     missing_debug_implementations,
//     trivial_casts,
//     trivial_numeric_casts,
//     unused_import_braces,
//     unused_lifetimes,
//     unused_unsafe,
//     useless_ptr_null_checks,
//     while_true,
//     unused_features,
//     absolute_paths_not_starting_with_crate,
//     unused_allocation,
//     unreachable_code,
//     unused_comparisons,
//     unused_parens,
//     asm_sub_register,
//     break_with_label_and_loop,
//     bindings_with_variant_name,
//     anonymous_parameters,
//     clippy::unwrap_used,
//     clippy::panicking_unwrap,
//     missing_abi,
//     clippy::missing_safety_doc,
//     clippy::missing_asserts_for_indexing,
//     clippy::missing_assert_message,
//     clippy::possible_missing_comma,
//     deprecated
// )]
// #![allow(clippy::module_name_repetitions, clippy::option_if_let_else)]
// #![cfg_attr(
//     not(test),
//     forbid(
//         clippy::panic,
//         clippy::todo,
//         clippy::unimplemented,
//         clippy::expect_used
//     )
// )]
// #![cfg_attr(not(test), warn(missing_docs))]

#[cfg(feature = "hmi")]
pub mod hmi;
#[cfg(feature = "hspo")]
pub mod hspo;
#[cfg(feature = "joint_conv")]
pub mod joints;
#[cfg(feature = "rmi")]
pub mod rmi;
#[cfg(feature = "stmo")]
pub mod stmo;

#[cfg(any(feature = "stmo", feature = "hspo", feature = "rmi", feature = "hmi"))]
mod thread_util;
#[cfg(any(feature = "stmo", feature = "hspo", feature = "rmi", feature = "hmi"))]
pub use thread_util::ThreadConfig;

#[cfg(feature = "py")]
pub mod py {
    use super::{hmi, hspo, joints, rmi, stmo, thread_util};
    use pyo3::prelude::*;
    use std::sync::{Mutex, OnceLock};
    use tracing::level_filters::LevelFilter;
    use tracing_subscriber::{
        EnvFilter, fmt, layer::SubscriberExt, registry::Registry, reload, util::SubscriberInitExt,
    };

    type LoggingHandle = reload::Handle<EnvFilter, Registry>;

    static LOGGING_INIT_LOCK: OnceLock<Mutex<Option<LoggingHandle>>> = OnceLock::new();

    fn logging_init_lock() -> &'static Mutex<Option<LoggingHandle>> {
        LOGGING_INIT_LOCK.get_or_init(|| Mutex::new(None))
    }

    #[pyclass(from_py_object)]
    #[derive(Debug, Clone, Copy)]
    pub enum LoggingLevel {
        Err,
        Warn,
        Info,
        Debug,
        Trace,
    }

    #[pyo3::pyfunction(signature = (level))]
    pub fn config_logging(level: LoggingLevel) -> PyResult<()> {
        let filter = match level {
            LoggingLevel::Err => LevelFilter::ERROR,
            LoggingLevel::Warn => LevelFilter::WARN,
            LoggingLevel::Info => LevelFilter::INFO,
            LoggingLevel::Debug => LevelFilter::DEBUG,
            LoggingLevel::Trace => LevelFilter::TRACE,
        };
        let env_filter = EnvFilter::builder()
            .with_default_directive(filter.into())
            .from_env_lossy();

        if let Some(handle) = logging_init_lock()
            .lock()
            .map_err(|_| {
                PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                    "fanuc logging init lock poisoned",
                )
            })?
            .as_ref()
        {
            let _ = handle.reload(env_filter);
            return Ok(());
        }

        let (reload_layer, reload_handle) = reload::Layer::new(env_filter);
        let tracing_result = tracing_subscriber::registry()
            .with(reload_layer)
            .with(fmt::layer().with_thread_ids(true).with_thread_names(true))
            .try_init();

        if tracing_result.is_ok() {
            *logging_init_lock().lock().map_err(|_| {
                PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                    "fanuc logging init lock poisoned",
                )
            })? = Some(reload_handle);
        } else {
            return Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                "fanuc logging initialization failed",
            ));
        }

        Ok(())
    }

    #[pyo3::pymodule(name = "_fanuc_core")]
    fn py_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
        hmi::py::register_child_module(m)?;
        stmo::py::register_child_module(m)?;
        hspo::py::register_child_module(m)?;
        rmi::py::register_child_module(m)?;
        m.add_class::<thread_util::ThreadConfig>()?;
        m.add_class::<joints::JointTemplate>()?;
        m.add_class::<joints::JointFormat>()?;
        m.add_class::<joints::JointType>()?;
        m.add_class::<LoggingLevel>()?;
        m.add_function(pyo3::wrap_pyfunction!(config_logging, m)?)?;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResponseNotFulfilled;
impl std::fmt::Display for ResponseNotFulfilled {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Response has not been fulfilled yet")
    }
}
impl std::error::Error for ResponseNotFulfilled {}

/// A trait representing a handle to an asynchronous response that can be awaited or queried for its status in synchronous contexts.
/// This is used for operations that may complete in the future, allowing the caller to check if the response is ready,
/// retrieve the result, or wait for completion with an optional timeout.
pub trait ResponseHandle:
    Future<Output = Result<Self::Ret, Self::Error>> + Send + Sync + 'static
{
    type Ret;
    type Error: std::error::Error;
    /// Checks if the response has been fulfilled and is ready to be retrieved.
    fn is_set(&self) -> bool;
    /// Attempts to retrieve the result of the response if it has been fulfilled, returning an error if it is not yet ready.
    /// If the response is fulfilled but contains an error, that error will be returned instead.
    /// If the response is not fulfilled at all, an error will be returned.
    fn get(&self) -> Result<Self::Ret, Self::Error>;
    /// Retrieves the timestamp of when the response was fulfilled, if available.
    /// This is can either be when the response was fulfilled or received over the network, depending on the implementation.
    fn timestamp(&self) -> Option<std::time::SystemTime>;
    /// Waits for the response to be fulfilled, with an optional timeout.
    /// If the timeout is reached before fulfillment an error is returned.
    fn wait_timeout(&self, timeout: std::time::Duration) -> Result<Self::Ret, Self::Error>;
    /// Waits indefinitely for the response to be fulfilled and retrieves the result.
    fn wait(&self) -> Result<Self::Ret, Self::Error>;
}

#[cfg(feature = "stmo")]
#[macro_export]
#[doc(hidden)]
macro_rules! bincode_enum {
    ($enm:ty : $int:ty) => {
        impl bincode::Encode for $enm {
            fn encode<E: bincode::enc::Encoder>(
                &self,
                encoder: &mut E,
            ) -> Result<(), bincode::error::EncodeError> {
                (*self as $int).encode(encoder)
            }
        }

        impl<Ctx> bincode::Decode<Ctx> for $enm {
            fn decode<D: bincode::de::Decoder<Context = Ctx>>(
                decoder: &mut D,
            ) -> Result<Self, bincode::error::DecodeError> {
                let variant_int = <$int as bincode::Decode<D::Context>>::decode(decoder)?;
                match <$enm>::try_from(variant_int) {
                    Ok(v) => Ok(v),
                    _ => Err(bincode::error::DecodeError::UnexpectedVariant {
                        found: variant_int as u32,
                        type_name: stringify!($enm),
                        allowed: &bincode::error::AllowedEnumVariants::Allowed(&[]),
                    }),
                }
            }
        }

        impl<'de, Ctx> bincode::de::BorrowDecode<'de, Ctx> for $enm {
            fn borrow_decode<D: bincode::de::Decoder<Context = Ctx>>(
                decoder: &mut D,
            ) -> Result<Self, bincode::error::DecodeError> {
                <Self as bincode::Decode<Ctx>>::decode(decoder)
            }
        }
    };
    (with_default $enm:ty : $int:ty) => {
        impl bincode::Encode for $enm {
            fn encode<E: bincode::enc::Encoder>(
                &self,
                encoder: &mut E,
            ) -> Result<(), bincode::error::EncodeError> {
                (*self as $int).encode(encoder)
            }
        }

        impl<Ctx> bincode::Decode<Ctx> for $enm {
            fn decode<D: bincode::de::Decoder<Context = Ctx>>(
                decoder: &mut D,
            ) -> Result<Self, bincode::error::DecodeError> {
                let variant_int = <$int as bincode::Decode<D::Context>>::decode(decoder)?;
                Ok(<$enm>::try_from(variant_int).unwrap_or_default())
            }
        }

        impl<Ctx> bincode::de::BorrowDecode<'_, Ctx> for $enm {
            fn borrow_decode<D: bincode::de::Decoder<Context = Ctx>>(
                decoder: &mut D,
            ) -> Result<Self, bincode::error::DecodeError> {
                <Self as bincode::Decode<Ctx>>::decode(decoder)
            }
        }
    };
}
