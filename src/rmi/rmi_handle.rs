use std::{
    collections::VecDeque,
    sync::{Arc, OnceLock},
    time::{Duration, Instant, SystemTime},
};

use atomic_waker::AtomicWaker;
use event_listener::{Event, Listener};
use inherent::inherent;

use crate::{
    ResponseHandle, ResponseNotFulfilled,
    rmi::{
        ReceivablePacket, ResponsePacket,
        errors::{PacketMismatchError, RmiError, RmiProtocolError, RmiResult},
    },
};

#[derive(Debug)]
enum ResponseOrError {
    Response(ResponsePacket, SystemTime),
    Error(RmiError, SystemTime),
    Skipped,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct RmiHandleGeneric {
    packet_name: &'static str,
    seq_id: u32,
    // `.1` Event wakes blocking `wait_timeout` waiters; `.2` AtomicWaker wakes
    // the async `poll` waiter. Both are signalled after `.0` is set.
    resp: Arc<(OnceLock<ResponseOrError>, Event, AtomicWaker)>,
}
impl RmiHandleGeneric {
    const ERROR_NAMES: [&'static str; 2] = ["FRC_SystemFault", "FRC_Terminate"];

    pub(super) fn new(packet_name: &'static str, seq_id: u32) -> Self {
        Self {
            packet_name,
            seq_id,
            resp: Arc::new((OnceLock::new(), Event::new(), AtomicWaker::new())),
        }
    }

    pub(super) fn set_generic(&self, value: ResponsePacket) -> RmiResult<()> {
        let mut outcome = Ok(());
        // let seq_id = if let ResponsePacket::Instruction(ref inst) = value {
        //     inst.sequence_id()
        // } else {
        //     0
        // };
        let now = SystemTime::now();
        if Self::ERROR_NAMES.contains(&value.packet_name()) {
            tracing::error!(
                packet = value.packet_name(),
                "RMI received system fault/terminate"
            );
            let _ = self.resp.0.set(ResponseOrError::Error(
                RmiError::SystemFaultOrTerminate,
                now,
            ));
        } else if value.error_id() != 0 {
            let ec = RmiProtocolError::try_from(value.error_id()).unwrap_or(Default::default());
            tracing::error!(packet = self.packet_name, error = %ec, "RMI response error");
            let _ = self
                .resp
                .0
                .set(ResponseOrError::Error(RmiError::FanucErrorCode(ec), now));
        } else if value.packet_name() != self.packet_name {
            let _ = self.resp.0.set(ResponseOrError::Skipped);
            self.resp.1.notify(usize::MAX);
            outcome = Err(RmiError::PacketMismatch(PacketMismatchError));
        } else {
            let _ = self.resp.0.set(ResponseOrError::Response(value, now));
        }
        self.resp.1.notify(usize::MAX);
        self.resp.2.wake();
        outcome
    }

    pub(super) fn set_error(&self, error: RmiError) -> RmiResult<()> {
        let _ = self
            .resp
            .0
            .set(ResponseOrError::Error(error, SystemTime::now()));
        self.resp.1.notify(usize::MAX);
        self.resp.2.wake();
        Ok(())
    }
}

impl crate::sealed::Sealed for RmiHandleGeneric {}

#[inherent]
impl ResponseHandle for RmiHandleGeneric {
    type Ret = ResponsePacket;
    type Error = RmiError;

    pub fn is_set(&self) -> bool {
        self.resp.0.get().is_some()
    }

    pub fn get(&self) -> RmiResult<ResponsePacket> {
        match self.resp.0.get() {
            Some(ResponseOrError::Response(v, _)) => Ok(v.clone()),
            Some(ResponseOrError::Error(e, _)) => Err(e.clone()),
            _ => Err(RmiError::ResponseNotFulfilled(ResponseNotFulfilled)),
        }
    }

    pub fn timestamp(&self) -> Option<SystemTime> {
        self.resp.0.get().and_then(|r| match r {
            ResponseOrError::Response(_, t) => Some(*t),
            ResponseOrError::Error(_, t) => Some(*t),
            ResponseOrError::Skipped => None,
        })
    }

    pub fn wait_timeout(&self, timeout: Duration) -> RmiResult<ResponsePacket> {
        // Register the listener before checking state so we can't miss a
        // notify() that fires between the check and listen() (event_listener
        // only wakes listeners registered at notify time).
        let listener = self.resp.1.listen();
        if self.is_set() {
            return self.get();
        }
        if listener.wait_timeout(timeout).is_some() || self.is_set() {
            self.get()
        } else {
            Err(RmiError::Timeout)
        }
    }

    pub fn wait(&self) -> RmiResult<ResponsePacket> {
        self.wait_timeout(Duration::MAX)
    }
}

impl Future for RmiHandleGeneric {
    type Output = RmiResult<ResponsePacket>;

    fn poll(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        if self.is_set() {
            return std::task::Poll::Ready(self.get());
        }
        self.resp.2.register(cx.waker());
        // Re-check after registering: a response set between the check above and
        // the register would otherwise wake a waker we hadn't stored yet.
        if self.is_set() {
            std::task::Poll::Ready(self.get())
        } else {
            std::task::Poll::Pending
        }
    }
}

#[derive(Debug, Clone)]
pub struct RmiHandle<T: ReceivablePacket> {
    generic: RmiHandleGeneric,
    _marker: std::marker::PhantomData<T>,
}
impl<T: ReceivablePacket> RmiHandle<T> {
    pub fn new_from_generic(generic: &RmiHandleGeneric) -> Self {
        Self {
            generic: generic.clone(),
            _marker: std::marker::PhantomData,
        }
    }

    pub fn generic(&self) -> RmiHandleGeneric {
        self.generic.clone()
    }
}

impl<T: ReceivablePacket> crate::sealed::Sealed for RmiHandle<T> {}

#[inherent]
impl<T: ReceivablePacket> ResponseHandle for RmiHandle<T> {
    type Ret = T;
    type Error = RmiError;

    pub fn is_set(&self) -> bool {
        self.generic.is_set()
    }

    pub fn get(&self) -> RmiResult<T> {
        self.generic
            .get()
            .and_then(|v| T::try_from(v.clone()).map_err(Into::into))
    }

    pub fn timestamp(&self) -> Option<SystemTime> {
        self.generic.timestamp()
    }

    pub fn wait_timeout(&self, timeout: Duration) -> RmiResult<T> {
        self.generic
            .wait_timeout(timeout)
            .and_then(|v| T::try_from(v.clone()).map_err(Into::into))
    }

    pub fn wait(&self) -> RmiResult<T> {
        self.wait_timeout(Duration::MAX)
    }
}

impl<T: ReceivablePacket> Future for RmiHandle<T> {
    type Output = RmiResult<T>;

    fn poll(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        let generic_pin = std::pin::pin!(self.generic.clone());
        match generic_pin.poll(cx) {
            std::task::Poll::Ready(res) => {
                std::task::Poll::Ready(res.and_then(|v| T::try_from(v.clone()).map_err(Into::into)))
            }
            std::task::Poll::Pending => std::task::Poll::Pending,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RmiQueueGeneric {
    queue: VecDeque<RmiHandleGeneric>,
}

impl RmiQueueGeneric {
    pub fn push(&mut self, handle: RmiHandleGeneric) {
        self.queue.push_back(handle);
    }

    pub fn handles(&self) -> impl Iterator<Item = &RmiHandleGeneric> {
        self.queue.iter()
    }

    pub fn responses(&self) -> impl Iterator<Item = ResponsePacket> {
        self.queue
            .iter()
            .filter(|h| h.is_set())
            .filter_map(|h| h.get().ok())
    }

    pub fn wait_all_timeout(&self, timeout: Duration) -> RmiResult<Vec<ResponsePacket>> {
        let mut outcome = Vec::with_capacity(self.queue.len());
        let deadline = Instant::now().checked_add(timeout);
        for handle in &self.queue {
            let remaining = match deadline {
                Some(end) => end.saturating_duration_since(Instant::now()),
                None => Duration::MAX,
            };
            outcome.push(handle.wait_timeout(remaining)?);
        }
        Ok(outcome)
    }

    pub fn wait_all(&self) -> RmiResult<Vec<ResponsePacket>> {
        self.wait_all_timeout(Duration::MAX)
    }

    pub fn wait_next_timeout(&self, timeout: Duration) -> RmiResult<ResponsePacket> {
        let deadline = Instant::now().checked_add(timeout);
        for handle in &self.queue {
            if !handle.is_set() {
                let remaining = match deadline {
                    Some(end) => end.saturating_duration_since(Instant::now()),
                    None => Duration::MAX,
                };
                return handle.wait_timeout(remaining);
            }
        }
        Err(RmiError::ResponseNotFulfilled(ResponseNotFulfilled))
    }

    pub fn wait_next(&self) -> RmiResult<ResponsePacket> {
        self.wait_next_timeout(Duration::MAX)
    }

    pub fn all_set(&self) -> bool {
        self.queue.iter().all(|h| h.is_set())
    }

    pub fn prune(&mut self) {
        self.queue.retain(|h| !h.is_set());
    }

    #[cfg(feature = "async")]
    pub async fn wait_all_async(&self) -> RmiResult<Vec<ResponsePacket>> {
        let mut outcome = Vec::with_capacity(self.queue.len());
        for handle in &self.queue {
            outcome.push(handle.clone().await?);
        }
        Ok(outcome)
    }

    pub fn clear(&mut self) {
        self.queue.clear();
    }
}

#[derive(Debug, Clone)]
pub struct RmiQueue<T: ReceivablePacket> {
    generic: RmiQueueGeneric,
    _marker: std::marker::PhantomData<T>,
}

impl<T: ReceivablePacket> RmiQueue<T> {
    pub fn push(&mut self, handle: RmiHandle<T>) {
        self.generic.push(handle.generic());
    }

    pub fn handles(&self) -> impl Iterator<Item = RmiHandle<T>> {
        self.generic
            .handles()
            .map(|h| RmiHandle::new_from_generic(h))
    }

    pub fn responses(&self) -> impl Iterator<Item = T> {
        self.generic.responses().filter_map(|v| T::try_from(v).ok())
    }

    pub fn wait_all_timeout(&self, timeout: Duration) -> RmiResult<Vec<T>> {
        self.generic.wait_all_timeout(timeout).and_then(|vec| {
            vec.into_iter()
                .map(|v| T::try_from(v).map_err(Into::into))
                .collect()
        })
    }

    pub fn wait_all(&self) -> RmiResult<Vec<T>> {
        self.wait_all_timeout(Duration::MAX)
    }

    pub fn wait_next_timeout(&self, timeout: Duration) -> RmiResult<T> {
        self.generic
            .wait_next_timeout(timeout)
            .and_then(|v| T::try_from(v).map_err(Into::into))
    }

    pub fn wait_next(&self) -> RmiResult<T> {
        self.wait_next_timeout(Duration::MAX)
    }

    pub fn all_set(&self) -> bool {
        self.generic.all_set()
    }

    pub fn prune(&mut self) {
        self.generic.prune();
    }

    pub fn clear(&mut self) {
        self.generic.clear();
    }
}

#[cfg(feature = "py")]
pub(super) mod py {

    use std::time::Duration;

    use pyo3::{prelude::*, pyclass, pymethods, types::PyType};

    use crate::rmi::{ResponsePacket, errors::RmiError};

    #[pyclass(name = "RmiHandle", generic)]
    pub struct PyRmiHandleGeneric {
        pub inner: super::RmiHandleGeneric,
        pub pytype: Py<PyType>,
    }

    #[pymethods]
    impl PyRmiHandleGeneric {
        pub fn is_set(&self) -> bool {
            self.inner.is_set()
        }

        pub fn get(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
            self.inner
                .get()
                .map_err(Into::into)
                .and_then(|v| self.pytype.call_method1(py, "from_response_packet", (v,)))
        }

        pub fn wait_timeout(&self, py: Python<'_>, timeout_secs: f64) -> PyResult<Py<PyAny>> {
            let timeout = Duration::try_from_secs_f64(timeout_secs).unwrap_or(Duration::MAX);
            self.inner
                .wait_timeout(timeout)
                .map_err(Into::into)
                .and_then(|v| self.pytype.call_method1(py, "from_response_packet", (v,)))
        }

        pub fn wait(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
            self.wait_timeout(py, 100000.0)
        }

        pub fn timestamp(&self) -> Option<f64> {
            self.inner.timestamp().map(|t| {
                t.duration_since(std::time::SystemTime::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_secs_f64()
            })
        }
    }

    #[pyclass(name = "RmiHandleQueue", generic)]
    pub struct PyRmiHandleQueue {
        inner: super::RmiQueueGeneric,
        pytype: Option<Py<PyType>>,
    }

    #[pymethods]
    impl PyRmiHandleQueue {
        #[new]
        pub fn new() -> Self {
            Self {
                inner: super::RmiQueueGeneric {
                    queue: std::collections::VecDeque::new(),
                },
                pytype: None,
            }
        }

        pub fn push(&mut self, py: Python<'_>, handle: &PyRmiHandleGeneric) -> PyResult<()> {
            if let Some(ref t) = self.pytype {
                let current_type_name = t.getattr(py, "__name__")?.extract::<String>(py)?;
                let new_type_name = handle
                    .pytype
                    .getattr(py, "__name__")?
                    .extract::<String>(py)?;
                if current_type_name != new_type_name {
                    return Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(format!(
                        "Mismatched response handle types: expected {}, got {}",
                        current_type_name, new_type_name
                    )));
                }
            } else {
                self.pytype = Some(handle.pytype.clone_ref(py));
            }
            self.inner.push(handle.inner.clone());
            Ok(())
        }

        fn packet_into_py(&self, py: Python<'_>, packet: ResponsePacket) -> PyResult<Py<PyAny>> {
            self.pytype
                .as_ref()
                .ok_or_else(|| {
                    PyErr::new::<pyo3::exceptions::PyValueError, _>(
                        "Cannot convert packet to Python object without a known type",
                    )
                })?
                .call_method1(py, "from_response_packet", (packet,))
        }

        pub fn handles(&self, py: Python<'_>) -> PyResult<Vec<PyRmiHandleGeneric>> {
            if self.pytype.is_none() {
                return Ok(Vec::new());
            }
            self.inner
                .handles()
                .map(|h| {
                    Ok(PyRmiHandleGeneric {
                        inner: h.clone(),
                        pytype: self
                            .pytype
                            .as_ref()
                            .ok_or_else(|| {
                                PyErr::new::<pyo3::exceptions::PyValueError, _>(
                                    "Cannot convert packet to Python object without a known type",
                                )
                            })
                            .map(|t| t.clone_ref(py))?,
                    })
                })
                .collect::<PyResult<Vec<_>>>()
        }

        pub fn responses(&self, py: Python<'_>) -> Vec<Py<PyAny>> {
            self.inner
                .responses()
                .map(|r| self.packet_into_py(py, r))
                .filter_map(Result::ok)
                .collect()
        }

        pub fn wait_all_timeout(
            &self,
            py: Python<'_>,
            timeout_secs: f64,
        ) -> PyResult<Vec<Py<PyAny>>> {
            let timeout = Duration::try_from_secs_f64(timeout_secs).unwrap_or(Duration::MAX);
            self.inner
                .wait_all_timeout(timeout)
                .map_err(Into::into)
                .map(|vec| {
                    vec.into_iter()
                        .map(|r| self.packet_into_py(py, r))
                        .filter_map(Result::ok)
                        .collect()
                })
        }

        pub fn wait_all(&self, py: Python<'_>) -> PyResult<Vec<Py<PyAny>>> {
            self.wait_all_timeout(py, Duration::MAX.as_secs_f64())
        }

        pub fn wait_next_timeout(&self, py: Python<'_>, timeout_secs: f64) -> PyResult<Py<PyAny>> {
            let timeout = Duration::try_from_secs_f64(timeout_secs).unwrap_or(Duration::MAX);
            self.inner
                .wait_next_timeout(timeout)
                .map_err(Into::into)
                .and_then(|r| self.packet_into_py(py, r))
        }

        pub fn wait_next(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
            self.wait_next_timeout(py, Duration::MAX.as_secs_f64())
        }

        pub fn all_set(&self) -> bool {
            self.inner.all_set()
        }

        pub fn prune(&mut self) {
            self.inner.prune();
        }

        pub fn raise_errors(&self) -> PyResult<()> {
            match self.inner.wait_all_timeout(Duration::ZERO) {
                Ok(_) => Ok(()),
                Err(RmiError::Timeout) => Ok(()),
                Err(RmiError::ResponseNotFulfilled(_)) => Ok(()),
                Err(e) => Err(e.into()),
            }
        }

        pub fn clear(&mut self) {
            self.inner.clear();
        }
    }

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<PyRmiHandleGeneric>()?;
        parent_module.add_class::<PyRmiHandleQueue>()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::task::{Context, Poll, Wake, Waker};
    use std::time::Instant;

    struct ThreadWaker(std::thread::Thread);
    impl Wake for ThreadWaker {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }
        fn wake_by_ref(self: &Arc<Self>) {
            self.0.unpark();
        }
    }

    fn block_on<F: Future>(fut: F) -> F::Output {
        let mut fut = Box::pin(fut);
        let waker = Waker::from(Arc::new(ThreadWaker(std::thread::current())));
        let mut cx = Context::from_waker(&waker);
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
            std::thread::park();
        }
    }

    /// Awaiting a handle must wake when it is fulfilled *after* the first poll
    /// parks. Parking executor + watchdog so a lost wakeup fails slow, not
    /// forever.
    #[test]
    fn async_await_wakes_on_late_notify() {
        let handle = RmiHandleGeneric::new("test", 0);
        let fulfiller = handle.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            let _ = fulfiller.set_error(RmiError::Timeout);
        });

        let waiter = std::thread::current();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_secs(3));
            waiter.unpark();
        });

        let start = Instant::now();
        let result = block_on(handle);
        assert!(result.is_err());
        assert!(
            start.elapsed() < Duration::from_secs(1),
            "async poll did not wake on notify (lost-wakeup regression)"
        );
    }
}
