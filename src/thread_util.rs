#![allow(dead_code)]

use cfg_mixin::cfg_mixin;
#[cfg(target_family = "unix")]
use libc::{
    CPU_SET, CPU_ZERO, cpu_set_t, pthread_self, pthread_setaffinity_np, pthread_setschedparam,
    sched_get_priority_max, sched_get_priority_min, sched_param, setpriority,
};
use std::{
    io,
    sync::{Arc, atomic::AtomicBool},
    thread::JoinHandle,
};

/// Configuration for thread scheduling and CPU affinity.
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Debug, Clone, Copy)]
pub struct ThreadConfig {
    /// Thread priority. If less than 1, the thread will be scheduled with SCHED_OTHER and a nice value of -8.
    pub priority: i32,
    /// Optional CPU affinity. If set, the thread will be pinned to the specified CPU core.
    pub cpu_affinity: Option<usize>,
}

impl ThreadConfig {
    pub fn configure_this_thread(&self) -> io::Result<()> {
        configure_thread_scheduling(self.priority, self.cpu_affinity)
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl ThreadConfig {
    #[on(new)]
    #[on(pyo3(signature=(priority=0, cpu_affinity=None)))]
    pub fn new(priority: i32, cpu_affinity: Option<usize>) -> Self {
        Self {
            priority,
            cpu_affinity,
        }
    }
}

#[cfg(target_family = "unix")]
fn set_nice(nice: i32) -> io::Result<()> {
    let rc = unsafe { setpriority(libc::PRIO_PROCESS, 0, nice) };
    if rc == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

#[cfg(target_family = "unix")]
fn configure_thread_scheduling(prio: i32, cpu_affinity: Option<usize>) -> io::Result<()> {
    unsafe {
        if let Some(cpu) = cpu_affinity {
            let ncpus = libc::sysconf(libc::_SC_NPROCESSORS_CONF) as usize;
            if cpu >= ncpus {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("CPU {} out of range 0..{}", cpu, ncpus.saturating_sub(1)),
                ));
            }
            let mut set: cpu_set_t = std::mem::zeroed();
            CPU_ZERO(&mut set);
            CPU_SET(cpu, &mut set);
            let rc = pthread_setaffinity_np(pthread_self(), std::mem::size_of::<cpu_set_t>(), &set);
            if rc != 0 {
                return Err(io::Error::from_raw_os_error(rc));
            }
        }

        let rc = if prio < 1 {
            let param = sched_param { sched_priority: 0 };
            let r = pthread_setschedparam(pthread_self(), libc::SCHED_OTHER, &param);
            set_nice(-8)?;
            r
        } else {
            let min = sched_get_priority_min(libc::SCHED_FIFO);
            let max = sched_get_priority_max(libc::SCHED_FIFO);
            if prio < min || prio > max {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!(
                        "SCHED_FIFO priority {} out of range [{}..={}]",
                        prio, min, max
                    ),
                ));
            }
            let param = sched_param {
                sched_priority: prio,
            };
            pthread_setschedparam(pthread_self(), libc::SCHED_FIFO, &param)
        };

        if rc != 0 {
            return Err(io::Error::from_raw_os_error(rc));
        }
    }
    Ok(())
}

#[cfg(not(target_family = "unix"))]
fn configure_thread_scheduling(_prio: i32, _cpu_affinity: Option<usize>) -> io::Result<()> {
    Err(io::Error::new(
        io::ErrorKind::Other,
        "Thread scheduling configuration is only supported on Unix-like systems",
    ))
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
        if self.is_owner {
            self.should_die
                .store(true, std::sync::atomic::Ordering::Relaxed);
            let _ = self.wake();
            if let Some(handle) = self.handle.take() {
                handle.thread().unpark();
                let _ = handle.join();
            }
        }
    }
}
