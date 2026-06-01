use std::{
    sync::{Arc, OnceLock},
    time::{Duration, SystemTime},
};

use event_listener::{Event, Listener};
use inherent::inherent;

use crate::{
    ResponseHandle, ResponseNotFulfilled,
    hmi::{
        HmiError,
        proto::{
            ports::{DataPort, ReadableDataPort},
            wire::Message,
        },
    },
};

/// Convenience alias for `Result<T, HmiError>`.
pub type HmiResult<T> = Result<T, HmiError>;

#[derive(Debug)]
enum ResponseOrError {
    Response(Message, SystemTime),
    Error(HmiError, SystemTime),
}

pub(super) fn caster_singular<T: ReadableDataPort>(
    msg: Message,
    target: u16,
    _count: u16,
) -> HmiResult<T::ValueType> {
    let payload = msg.payload();
    if payload.len() < T::expected_size(target, 1) {
        log::error!(
            "Malformed response: expected at least {} bytes, got {} bytes",
            T::expected_size(target, 1),
            payload.len()
        );
        return Err(HmiError::MalformedResponse);
    }
    Ok(T::unpack_single(target, payload))
}

pub(super) fn caster_array<T: ReadableDataPort>(
    msg: Message,
    target: u16,
    count: u16,
) -> HmiResult<Box<[T::ValueType]>> {
    let payload = msg.payload();
    if payload.len() < T::expected_array_size(target, count) {
        log::error!(
            "Malformed response: expected at least {} bytes, got {} bytes",
            T::expected_array_size(target, count),
            payload.len()
        );
        return Err(HmiError::MalformedResponse);
    }
    Ok(T::unpack_array(target, payload, count))
}

pub(super) fn caster_null<T: DataPort>(_: Message, _: u16, _: u16) -> HmiResult<()> {
    Ok(())
}

/// An untyped handle to a pending HMI response.
///
/// This is the low-level handle returned by the runner; it can be polled, waited on,
/// or wrapped in a typed [`HmiHandle`] to decode the response payload.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct HmiHandleGeneric {
    resp: Arc<(OnceLock<ResponseOrError>, Event)>,
}
impl HmiHandleGeneric {
    pub(super) fn new() -> Self {
        Self {
            resp: Arc::new((OnceLock::new(), Event::new())),
        }
    }

    pub(super) fn set_generic(&self, value: Message) -> HmiResult<()> {
        let now = SystemTime::now();
        let _ = self.resp.0.set(ResponseOrError::Response(value, now));
        self.resp.1.notify(usize::MAX);
        Ok(())
    }

    pub(super) fn set_error(&self, error: HmiError) -> HmiResult<()> {
        let _ = self
            .resp
            .0
            .set(ResponseOrError::Error(error, SystemTime::now()));
        self.resp.1.notify(usize::MAX);
        Ok(())
    }
}

#[inherent]
impl ResponseHandle for HmiHandleGeneric {
    type Ret = Message;
    type Error = HmiError;

    /// Returns `true` if the response (or an error) has been received.
    pub fn is_set(&self) -> bool {
        self.resp.0.get().is_some()
    }

    /// Returns the response message if available, or an error if the request failed or the response has not yet arrived.
    pub fn get(&self) -> Result<Message, HmiError> {
        match self.resp.0.get() {
            Some(ResponseOrError::Response(v, _)) => Ok(v.clone()),
            Some(ResponseOrError::Error(e, _)) => Err(e.clone()),
            _ => Err(HmiError::ResponseNotFulfilled(ResponseNotFulfilled)),
        }
    }

    /// Returns the timestamp at which the response was received, or `None` if it has not arrived yet.
    pub fn timestamp(&self) -> Option<SystemTime> {
        if !self.is_set() {
            return None;
        }
        self.resp.0.get().map(|r| match r {
            ResponseOrError::Response(_, t) => *t,
            ResponseOrError::Error(_, t) => *t,
        })
    }

    /// Blocks until the response arrives or the timeout elapses, returning [`HmiError::Timeout`] on expiry.
    pub fn wait_timeout(&self, timeout: Duration) -> Result<Message, HmiError> {
        if self.is_set() {
            return self.get();
        }
        let listener = self.resp.1.listen();
        if !timeout.is_zero() && listener.wait_timeout(timeout).is_some() {
            self.get()
        } else {
            Err(HmiError::Timeout)
        }
    }

    /// Blocks indefinitely until the response arrives.
    pub fn wait(&self) -> Result<Message, HmiError> {
        self.wait_timeout(Duration::MAX)
    }
}

impl Future for HmiHandleGeneric {
    type Output = HmiResult<Message>;

    fn poll(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        if self.is_set() {
            std::task::Poll::Ready(self.get())
        } else {
            let listener = self.resp.1.listen();
            let mut pinned = std::pin::pin!(listener);
            pinned.as_mut().poll(cx).map(|_| self.get())
        }
    }
}

/// A typed handle to a pending HMI response that decodes the raw message into `T`.
///
/// Implements [`Future`] for async usage and provides blocking [`wait`](Self::wait) / [`wait_timeout`](Self::wait_timeout) methods.
#[derive(Debug, Clone)]
pub struct HmiHandle<T: Send + Sync + 'static> {
    generic: HmiHandleGeneric,
    pub(super) target: u16,
    pub(super) count: u16,
    caster: fn(Message, u16, u16) -> HmiResult<T>,
    _marker: std::marker::PhantomData<T>,
}
impl<T: Send + Sync + 'static> HmiHandle<T> {
    pub fn new_from_generic(
        generic: &HmiHandleGeneric,
        target: u16,
        count: u16,
        caster: fn(Message, u16, u16) -> HmiResult<T>,
    ) -> Self {
        Self {
            generic: generic.clone(),
            target,
            count,
            caster,
            _marker: std::marker::PhantomData,
        }
    }

    /// Returns a clone of the underlying untyped [`HmiHandleGeneric`].
    pub fn generic(&self) -> HmiHandleGeneric {
        self.generic.clone()
    }
}

#[inherent]
impl<T: Send + Sync + 'static> ResponseHandle for HmiHandle<T> {
    type Ret = T;
    type Error = HmiError;

    pub fn is_set(&self) -> bool {
        self.generic.is_set()
    }

    pub fn get(&self) -> Result<T, HmiError> {
        self.generic
            .get()
            .and_then(|v| (self.caster)(v, self.target, self.count))
    }

    pub fn timestamp(&self) -> Option<SystemTime> {
        self.generic.timestamp()
    }

    pub fn wait_timeout(&self, timeout: Duration) -> Result<T, HmiError> {
        self.generic
            .wait_timeout(timeout)
            .and_then(|v| (self.caster)(v, self.target, self.count))
    }

    pub fn wait(&self) -> Result<T, HmiError> {
        self.wait_timeout(Duration::MAX)
    }
}

impl<T: Send + Sync + 'static> Future for HmiHandle<T> {
    type Output = HmiResult<T>;

    fn poll(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        if self.is_set() {
            std::task::Poll::Ready(self.get())
        } else {
            let listener = self.generic.resp.1.listen();
            let mut pinned = std::pin::pin!(listener);
            pinned.as_mut().poll(cx).map(|_| self.get())
        }
    }
}

#[cfg(feature = "py")]
pub(super) mod py {

    use std::time::Duration;

    use pyo3::{IntoPyObjectExt, prelude::*, pyclass, pymethods};

    use crate::hmi::{
        hmi_handle::{HmiHandleGeneric, caster_array, caster_singular},
        proto::{ports::ReadableDataPort, wire::Message},
    };

    pub fn py_caster_singular<'a, T: ReadableDataPort>(
        py: Python<'a>,
        msg: Message,
        target: u16,
        count: u16,
    ) -> PyResult<Bound<'a, PyAny>> {
        let v: T::ValueType = caster_singular::<T>(msg, target, count)?;
        v.into_bound_py_any(py)
    }

    pub fn py_caster_array<'a, T: ReadableDataPort>(
        py: Python<'a>,
        msg: Message,
        target: u16,
        count: u16,
    ) -> PyResult<Bound<'a, PyAny>> {
        let list = pyo3::types::PyList::empty(py);
        let values: Box<[T::ValueType]> = caster_array::<T>(msg, target, count)?;
        for v in values {
            let obj = v.clone().into_bound_py_any(py)?;
            list.append(obj)?;
        }
        list.into_bound_py_any(py)
    }

    #[allow(clippy::extra_unused_type_parameters)]
    pub fn py_caster_null<'a, T>(
        py: Python<'a>,
        _: Message,
        _: u16,
        _: u16,
    ) -> PyResult<Bound<'a, PyAny>> {
        pyo3::types::PyNone::get(py).into_bound_py_any(py)
    }

    #[derive(Debug)]
    #[pyclass(name = "HmiHandle", generic)]
    pub struct PyHmiHandleGeneric {
        pub inner: HmiHandleGeneric,
        pub target: u16,
        pub count: u16,
        pub caster: for<'a> fn(Python<'a>, Message, u16, u16) -> PyResult<Bound<'a, PyAny>>,
    }

    #[pymethods]
    impl PyHmiHandleGeneric {
        pub fn is_set(&self) -> bool {
            self.inner.is_set()
        }

        pub fn get<'a>(&self, py: Python<'a>) -> PyResult<Bound<'a, PyAny>> {
            self.inner
                .get()
                .map_err(Into::into)
                .and_then(|v| (self.caster)(py, v, self.target, self.count))
        }

        pub fn wait_timeout<'a>(
            &self,
            py: Python<'a>,
            timeout_secs: f64,
        ) -> PyResult<Bound<'a, PyAny>> {
            let timeout = Duration::from_secs_f64(timeout_secs);
            self.inner
                .wait_timeout(timeout)
                .map_err(Into::into)
                .and_then(|v| (self.caster)(py, v, self.target, self.count))
        }

        pub fn wait<'a>(&self, py: Python<'a>) -> PyResult<Bound<'a, PyAny>> {
            self.wait_timeout(py, 100000.0)
        }
    }

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<PyHmiHandleGeneric>()?;
        Ok(())
    }
}
