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

pub trait ResponseHandle:
    Future<Output = Result<Self::Ret, Self::Error>> + Send + Sync + 'static
{
    type Ret;
    type Error: std::error::Error;
    fn is_set(&self) -> bool;
    fn get(&self) -> Result<Self::Ret, Self::Error>;
    fn timestamp(&self) -> Option<std::time::SystemTime>;
    fn wait_timeout(&self, timeout: std::time::Duration) -> Result<Self::Ret, Self::Error>;
    fn wait(&self) -> Result<Self::Ret, Self::Error>;
}

#[cfg(any(feature = "stmo"))]
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

        impl<'de, Ctx> bincode::de::BorrowDecode<'de, Ctx> for $enm {
            fn borrow_decode<D: bincode::de::Decoder<Context = Ctx>>(
                decoder: &mut D,
            ) -> Result<Self, bincode::error::DecodeError> {
                <Self as bincode::Decode<Ctx>>::decode(decoder)
            }
        }
    };
}
