use std::{
    future::Future,
    sync::{Arc, OnceLock},
    time::{Duration, SystemTime},
};

use atomic_waker::AtomicWaker;
use event_listener::{Event, Listener};
use inherent::inherent;

use crate::{
    ResponseHandle, ResponseNotFulfilled, stmo::types::StreamMotionError, time_util::host_now,
};

#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone)]
pub struct StmoHandle {
    // `.1` Event wakes blocking `wait_timeout` waiters; `.2` AtomicWaker wakes
    // the async `poll` waiter. Both are signalled after `.0` is set.
    resp: Arc<(OnceLock<SystemTime>, Event, AtomicWaker)>,
}

impl StmoHandle {
    pub(crate) fn new() -> Self {
        Self {
            resp: Arc::new((OnceLock::new(), Event::new(), AtomicWaker::new())),
        }
    }

    pub(crate) fn set(&self) {
        let _ = self.resp.0.set(host_now());
        self.resp.1.notify(usize::MAX);
        self.resp.2.wake();
    }
}

impl crate::sealed::Sealed for StmoHandle {}

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
            return std::task::Poll::Ready(Ok(()));
        }
        self.resp.2.register(cx.waker());
        // Re-check after registering: a set() between the check above and the
        // register would otherwise wake a waker we hadn't stored yet.
        if self.is_set() {
            std::task::Poll::Ready(self.get())
        } else {
            std::task::Poll::Pending
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
    #[pyo3(name = "is_set")]
    pub fn py_is_set(&self) -> bool {
        self.resp.0.get().is_some()
    }

    #[pyo3(name = "get")]
    pub fn py_get(&self) -> pyo3::PyResult<()> {
        ResponseHandle::get(self).map_err(Into::into)
    }

    #[pyo3(name = "wait_timeout", signature = (timeout_secs = 10.0))]
    pub fn py_wait_timeout(&self, timeout_secs: f64) -> pyo3::PyResult<()> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
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

    /// Awaiting a handle must wake when it is set *after* the first poll parks.
    /// Parking executor + watchdog so a lost wakeup fails slow, not forever.
    #[test]
    fn async_await_wakes_on_late_notify() {
        // set() stamps host_now(), which under snare's shim resolves the
        // thread's clock slot — the test and the fulfiller must be in a
        // registered thread chain or snare panics and the wake is lost.
        snare::register_test();
        let handle = StmoHandle::new();
        let fulfiller = handle.clone();
        snare::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            fulfiller.set();
        });

        let waiter = std::thread::current();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_secs(3));
            waiter.unpark();
        });

        let start = Instant::now();
        let result = block_on(handle);
        assert!(result.is_ok());
        assert!(
            start.elapsed() < Duration::from_secs(1),
            "async poll did not wake on notify (lost-wakeup regression)"
        );
    }
}
