#![allow(dead_code)]

use parking_lot::Mutex;
use std::{
    error::Error,
    fmt::Debug,
    io,
    sync::{Arc, atomic::AtomicBool},
    task::Waker,
    thread::JoinHandle,
};

/// Wakers of every task polling a response handle or one of its clones.
///
/// A poller registers before it checks the state and returns `Pending` only if
/// the check still fails afterwards; the side that changes the state does so
/// first and then calls [`wake_all`](Self::wake_all), so no wake is lost.
#[derive(Default)]
pub(crate) struct WakerSet {
    wakers: Mutex<Vec<Waker>>,
}

impl WakerSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register(&self, waker: &Waker) {
        let mut wakers = self.wakers.lock();
        match wakers.iter_mut().find(|w| w.will_wake(waker)) {
            Some(w) => w.clone_from(waker),
            None => wakers.push(waker.clone()),
        }
    }

    pub fn wake_all(&self) {
        let wakers = std::mem::take(&mut *self.wakers.lock());
        for w in wakers {
            w.wake();
        }
    }
}

impl Debug for WakerSet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WakerSet")
            .field("waiting", &self.wakers.lock().len())
            .finish()
    }
}

#[derive(Debug, Clone)]
pub(crate) enum WakerVariant {
    #[allow(dead_code)]
    Std(Arc<std::task::Waker>),
    Mio(Arc<mio::Waker>),
}

#[derive(Debug)]
pub(crate) struct ThreadHandle {
    is_owner: bool,
    is_alive: Arc<AtomicBool>,
    should_die: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
    waker: Option<WakerVariant>,
}

impl ThreadHandle {
    pub fn new() -> Self {
        Self {
            is_owner: true,
            is_alive: Arc::new(AtomicBool::new(true)),
            should_die: Arc::new(AtomicBool::new(false)),
            handle: None,
            waker: None,
        }
    }

    pub fn set_handle(&mut self, handle: JoinHandle<()>) {
        self.handle = Some(handle);
    }

    #[allow(dead_code)]
    pub fn set_waker_std(&mut self, waker: Arc<std::task::Waker>) {
        self.waker = Some(WakerVariant::Std(waker));
    }

    pub fn set_waker_mio(&mut self, waker: Arc<mio::Waker>) {
        self.waker = Some(WakerVariant::Mio(waker));
    }

    pub fn wake(&self) -> io::Result<()> {
        if let Some(waker) = &self.waker {
            match waker {
                WakerVariant::Std(w) => {
                    w.wake_by_ref();
                    Ok(())
                }
                WakerVariant::Mio(w) => w.wake(),
            }
        } else {
            Ok(())
        }
    }

    pub fn is_alive(&self) -> bool {
        self.is_alive.load(std::sync::atomic::Ordering::Relaxed)
    }

    pub fn should_live(&self) -> bool {
        !self.should_die.load(std::sync::atomic::Ordering::Relaxed)
    }

    pub fn has_died(&self) {
        self.is_alive
            .store(false, std::sync::atomic::Ordering::Relaxed);
    }

    pub fn join(mut self) {
        self.stop_and_join();
    }

    fn stop_and_join(&mut self) {
        if !self.is_owner {
            return;
        }
        self.should_die
            .store(true, std::sync::atomic::Ordering::Relaxed);
        let _ = self.wake();
        if let Some(handle) = self.handle.take() {
            handle.thread().unpark();
            let _ = handle.join();
        }
    }

    pub fn to_pass_in(&self) -> Self {
        Self {
            is_owner: false,
            is_alive: self.is_alive.clone(),
            should_die: self.should_die.clone(),
            handle: None,
            waker: self.waker.clone(),
        }
    }
}

impl Drop for ThreadHandle {
    fn drop(&mut self) {
        self.stop_and_join();
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct NoCustomError;
impl std::fmt::Display for NoCustomError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "No custom error")
    }
}
impl Error for NoCustomError {}

#[derive(Debug)]
pub(crate) enum GeneralThreadError<
    T: Sized + Send + Sync + core::fmt::Debug + Error + 'static = NoCustomError,
> {
    Io(std::io::Error),
    FailedToCreatePoll,
    FailedSocketBinding,
    FailedSocketRegistry,
    FailedWakerCreation,
    FlumeSend,
    FlumeRecv(flume::RecvError),
    Custom(T),
}

impl<T: Sized + Send + Sync + core::fmt::Debug + Error + 'static> From<std::io::Error>
    for GeneralThreadError<T>
{
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

impl<T: Sized + Send + Sync + core::fmt::Debug + Error + 'static> From<flume::RecvError>
    for GeneralThreadError<T>
{
    fn from(value: flume::RecvError) -> Self {
        Self::FlumeRecv(value)
    }
}

impl<T: Sized + Send + Sync + core::fmt::Debug + Error + 'static, U> From<flume::SendError<U>>
    for GeneralThreadError<T>
{
    fn from(_value: flume::SendError<U>) -> Self {
        Self::FlumeSend
    }
}

impl<T: Sized + Send + Sync + core::fmt::Debug + Error + 'static> std::fmt::Display
    for GeneralThreadError<T>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "IO error: {}", e),
            Self::FailedToCreatePoll => write!(f, "Failed to create poll instance"),
            Self::FailedSocketBinding => write!(f, "Failed to bind socket"),
            Self::FailedSocketRegistry => write!(f, "Failed to register socket"),
            Self::FailedWakerCreation => write!(f, "Failed to create waker"),
            Self::FlumeSend => write!(f, "Flume send error"),
            Self::FlumeRecv(e) => write!(f, "Flume receive error: {}", e),
            Self::Custom(e) => write!(f, "Custom error: {:?}", e),
        }
    }
}

impl<T: Sized + Send + Sync + core::fmt::Debug + Error + 'static> Error for GeneralThreadError<T> {}

#[cfg(feature = "py")]
impl<T: Sized + Send + Sync + core::fmt::Debug + Error + 'static> From<GeneralThreadError<T>>
    for pyo3::PyErr
{
    fn from(value: GeneralThreadError<T>) -> Self {
        match value {
            GeneralThreadError::Io(e) => {
                pyo3::PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("IO error: {}", e))
            }
            other => pyo3::PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", other)),
        }
    }
}
