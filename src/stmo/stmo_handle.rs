use std::{
    future::Future,
    sync::{Arc, OnceLock},
    time::{Duration, SystemTime},
};

use event_listener::{Event, Listener};
use inherent::inherent;

use crate::{ResponseHandle, ResponseNotFulfilled, stmo::types::StreamMotionError};

#[cfg_attr(feature = "py", pyo3::pyclass(str))]
#[derive(Debug, Clone)]
pub struct StmoHandle {
    resp: Arc<(OnceLock<SystemTime>, Event)>,
}

impl StmoHandle {
    pub(crate) fn new() -> Self {
        Self {
            resp: Arc::new((OnceLock::new(), Event::new())),
        }
    }

    pub(crate) fn set(&self) {
        let _ = self.resp.0.set(SystemTime::now());
        self.resp.1.notify(usize::MAX);
    }
}

#[inherent]
impl ResponseHandle for StmoHandle {
    type Ret = ();
    type Error = StreamMotionError;

    pub fn is_set(&self) -> bool {
        self.resp.0.get().is_some()
    }

    pub fn get(&self) -> Result<(), StreamMotionError> {
        if self.is_set() {
            Ok(())
        } else {
            Err(StreamMotionError::ResponseNotFulfilled(
                ResponseNotFulfilled,
            ))
        }
    }

    pub fn timestamp(&self) -> Option<SystemTime> {
        self.resp.0.get().copied()
    }

    pub fn wait_timeout(&self, timeout: Duration) -> Result<(), StreamMotionError> {
        if self.is_set() {
            return Ok(());
        }
        let listener = self.resp.1.listen();
        if listener.wait_timeout(timeout).is_some() {
            self.get()
        } else {
            Err(StreamMotionError::Timeout)
        }
    }

    pub fn wait(&self) -> Result<(), StreamMotionError> {
        self.wait_timeout(Duration::MAX)
    }
}

impl Future for StmoHandle {
    type Output = Result<(), StreamMotionError>;

    fn poll(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        if self.is_set() {
            std::task::Poll::Ready(Ok(()))
        } else {
            let listener = self.resp.1.listen();
            let mut pinned = std::pin::pin!(listener);
            pinned.as_mut().poll(cx).map(|_| self.get())
        }
    }
}

impl std::fmt::Display for StmoHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "StmoHandle(fulfilled={})", self.is_set())
    }
}

#[cfg(feature = "py")]
#[pyo3::pymethods]
impl StmoHandle {
    pub fn is_set(&self) -> bool {
        self.resp.0.get().is_some()
    }

    #[pyo3(name = "get")]
    pub fn py_get(&self) -> pyo3::PyResult<()> {
        ResponseHandle::get(self).map_err(Into::into)
    }

    #[pyo3(signature = (timeout_secs = 10.0))]
    pub fn wait_timeout(&self, timeout_secs: f64) -> pyo3::PyResult<()> {
        let timeout = Duration::from_secs_f64(timeout_secs);
        ResponseHandle::wait_timeout(self, timeout).map_err(Into::into)
    }

    #[pyo3(name = "wait")]
    pub fn py_wait(&self) -> pyo3::PyResult<()> {
        ResponseHandle::wait(self).map_err(Into::into)
    }

    #[pyo3(name = "timestamp")]
    pub fn py_timestamp(&self) -> Option<f64> {
        ResponseHandle::timestamp(self).map(|t| {
            t.duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs_f64()
        })
    }
}

#[cfg(feature = "py")]
pub(crate) mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<StmoHandle>()?;
        Ok(())
    }
}
