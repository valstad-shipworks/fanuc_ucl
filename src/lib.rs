#![cfg_attr(not(doctest), doc = include_str!("../README.md"))]
// #![deny(clippy::all, clippy::pedantic, clippy::nursery)]
// #![deny(
//     missing_copy_implementations,
//     single_use_lifetimes,
//     variant_size_differences,
//     trivial_casts,
//     trivial_numeric_casts,
//     unused_import_braces,
//     unused_lifetimes,
//     useless_ptr_null_checks,
//     while_true,
//     unused_features,
//     absolute_paths_not_starting_with_crate,
//     unused_allocation,
//     unused_comparisons,
//     unused_parens,
//     asm_sub_register,
//     break_with_label_and_loop,
//     bindings_with_variant_name,
//     anonymous_parameters,
//     missing_abi,
//     clippy::missing_assert_message,
//     clippy::possible_missing_comma,
//     deprecated
// )]
#![deny(
    arithmetic_overflow,
    missing_debug_implementations,
    unused_unsafe,
    unreachable_code,
    clippy::panicking_unwrap,
    clippy::missing_safety_doc
)]
#![allow(clippy::module_name_repetitions, clippy::option_if_let_else)]
#![cfg_attr(
    not(test),
    forbid(
        clippy::panic,
        clippy::todo,
        clippy::unimplemented,
        clippy::expect_used,
        clippy::unwrap_used
    )
)]
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

    /// Logging level surfaced to Python as an enum (no string parsing on the
    /// caller side). Maps 1:1 to [`log::LevelFilter`].
    #[pyo3::pyclass(eq, eq_int, from_py_object)]
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum LogLevel {
        Off,
        Error,
        Warn,
        Info,
        Debug,
        Trace,
    }

    impl From<LogLevel> for log::LevelFilter {
        fn from(l: LogLevel) -> Self {
            match l {
                LogLevel::Off => log::LevelFilter::Off,
                LogLevel::Error => log::LevelFilter::Error,
                LogLevel::Warn => log::LevelFilter::Warn,
                LogLevel::Info => log::LevelFilter::Info,
                LogLevel::Debug => log::LevelFilter::Debug,
                LogLevel::Trace => log::LevelFilter::Trace,
            }
        }
    }

    /// Parse the simple form of `RUST_LOG` (just a level name) so we can use
    /// it as a baseline. Per-module directives are not supported here — the
    /// expected use of `set_log_level` is "give me everything at level X",
    /// not selective per-target filtering.
    fn parse_rust_log_baseline() -> log::LevelFilter {
        use log::LevelFilter::*;
        let raw = std::env::var("RUST_LOG").ok();
        match raw
            .as_deref()
            .map(str::trim)
            .map(str::to_ascii_lowercase)
            .as_deref()
        {
            Some("off") => Off,
            Some("error") => Error,
            Some("warn") | Some("warning") => Warn,
            Some("info") => Info,
            Some("debug") => Debug,
            Some("trace") => Trace,
            _ => Warn,
        }
    }

    /// Set the runtime log level for the fanuc_ucl Rust core. The effective
    /// level is `max(current, requested)` — i.e. only ever raises verbosity,
    /// never lowers it below what `RUST_LOG` already established. Returns the
    /// effective level after the call.
    #[pyo3::pyfunction]
    fn set_log_level(level: LogLevel) -> LogLevel {
        let requested: log::LevelFilter = level.into();
        let current = log::max_level();
        let effective = if requested > current {
            requested
        } else {
            current
        };
        log::set_max_level(effective);
        match effective {
            log::LevelFilter::Off => LogLevel::Off,
            log::LevelFilter::Error => LogLevel::Error,
            log::LevelFilter::Warn => LogLevel::Warn,
            log::LevelFilter::Info => LogLevel::Info,
            log::LevelFilter::Debug => LogLevel::Debug,
            log::LevelFilter::Trace => LogLevel::Trace,
        }
    }

    #[pyo3::pymodule(name = "_fanuc_core")]
    fn py_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
        // Configure env_logger to pass every record through (no internal
        // filter) so `log::max_level()` is the single runtime gate. Otherwise
        // env_logger's parsed filter would clamp verbosity below whatever
        // `set_log_level` later asks for.
        let env_baseline = parse_rust_log_baseline();
        let _ = env_logger::Builder::new()
            .filter_level(log::LevelFilter::Trace)
            .try_init();
        log::set_max_level(env_baseline);

        log::trace!("Initializing Python module fanuc_ucl._fanuc_core");

        hmi::py::register_child_module(m)?;
        stmo::py::register_child_module(m)?;
        hspo::py::register_child_module(m)?;
        rmi::py::register_child_module(m)?;
        m.add_class::<thread_util::ThreadConfig>()?;
        m.add_class::<joints::JointTemplate>()?;
        m.add_class::<joints::JointFormat>()?;
        m.add_class::<joints::JointType>()?;
        m.add_class::<LogLevel>()?;
        m.add_function(pyo3::wrap_pyfunction!(set_log_level, m)?)?;
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

mod sealed {
    pub trait Sealed {}
}

/// A trait representing a handle to an asynchronous response that can be awaited or queried for its status in synchronous contexts.
/// This is used for operations that may complete in the future, allowing the caller to check if the response is ready,
/// retrieve the result, or wait for completion with an optional timeout.
///
/// This trait is sealed and cannot be implemented outside this crate.
pub trait ResponseHandle:
    sealed::Sealed + Future<Output = Result<Self::Ret, Self::Error>> + Send + Sync + 'static
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
