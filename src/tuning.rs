use std::{fmt, io, net::SocketAddr};

use fast_talker::options::{Report, ReportSummary, Rules, SocketOption, ThreadOption};
#[cfg(any(feature = "rmi", feature = "hmi"))]
use fast_talker::rt::{Scheduler, ThreadPriority};
use fast_talker::sockets;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ThreadRole {
    #[cfg(feature = "stmo")]
    Cyclic,
    #[cfg(feature = "hspo")]
    Stream,
    #[cfg(any(feature = "rmi", feature = "hmi"))]
    Control,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SocketRole {
    #[cfg(feature = "stmo")]
    UdpCyclic,
    #[cfg(feature = "hspo")]
    UdpStreamRx,
    #[cfg(any(feature = "rmi", feature = "hmi"))]
    TcpControl,
}

/// An option a driver does not accept for the role it was given to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct RefusedOption {
    pub option: String,
    pub driver: &'static str,
}

impl fmt::Display for RefusedOption {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} does not accept {}", self.driver, self.option)
    }
}

pub(crate) fn thread_allowed(role: ThreadRole, o: &ThreadOption) -> bool {
    match role {
        #[cfg(feature = "stmo")]
        ThreadRole::Cyclic => matches!(
            o,
            ThreadOption::CpuAffinity(_)
                | ThreadOption::RtPriority(_)
                | ThreadOption::PrefaultStack(_)
                | ThreadOption::UnixScheduler(_)
                | ThreadOption::LinuxNice(_)
                | ThreadOption::WinPriority(_)
                | ThreadOption::WinDisablePowerThrottling
                | ThreadOption::WinMmcss(_)
                | ThreadOption::MacOsQos(_)
                | ThreadOption::MacOsTimeConstraint { .. }
        ),
        #[cfg(feature = "hspo")]
        ThreadRole::Stream => matches!(
            o,
            ThreadOption::CpuAffinity(_)
                | ThreadOption::RtPriority(_)
                | ThreadOption::PrefaultStack(_)
                | ThreadOption::UnixScheduler(_)
                | ThreadOption::LinuxNice(_)
                | ThreadOption::WinPriority(_)
                | ThreadOption::WinDisablePowerThrottling
                | ThreadOption::WinMmcss(_)
                | ThreadOption::MacOsQos(_)
        ),
        #[cfg(any(feature = "rmi", feature = "hmi"))]
        ThreadRole::Control => match o {
            ThreadOption::CpuAffinity(_)
            | ThreadOption::PrefaultStack(_)
            | ThreadOption::LinuxNice(_)
            | ThreadOption::WinDisablePowerThrottling
            | ThreadOption::MacOsQos(_) => true,
            ThreadOption::UnixScheduler(s) => {
                matches!(s, Scheduler::Other | Scheduler::Batch | Scheduler::Idle)
            }
            ThreadOption::WinPriority(p) => !matches!(p, ThreadPriority::TimeCritical),
            _ => false,
        },
    }
}

pub(crate) fn socket_allowed(role: SocketRole, o: &SocketOption) -> bool {
    match role {
        #[cfg(feature = "stmo")]
        SocketRole::UdpCyclic => matches!(
            o,
            SocketOption::RecvBuffer(_)
                | SocketOption::SendBuffer(_)
                | SocketOption::BindDevice(_)
                | SocketOption::DontFragment(_)
                | SocketOption::Dscp(_)
                | SocketOption::LinuxPriority(_)
                | SocketOption::LinuxBusyPoll(_)
                | SocketOption::LinuxPreferBusyPoll(_)
                | SocketOption::LinuxBusyPollBudget(_)
                | SocketOption::WinCpuAffinity(_)
        ),
        #[cfg(feature = "hspo")]
        SocketRole::UdpStreamRx => matches!(
            o,
            SocketOption::RecvBuffer(_)
                | SocketOption::BindDevice(_)
                | SocketOption::LinuxBusyPoll(_)
                | SocketOption::LinuxPreferBusyPoll(_)
                | SocketOption::LinuxBusyPollBudget(_)
                | SocketOption::WinCpuAffinity(_)
        ),
        #[cfg(any(feature = "rmi", feature = "hmi"))]
        SocketRole::TcpControl => matches!(
            o,
            SocketOption::BindDevice(_) | SocketOption::Dscp(_) | SocketOption::LinuxPriority(_)
        ),
    }
}

impl RefusedOption {
    fn new(driver: &'static str, kind: &str, option: &impl fmt::Debug) -> Self {
        Self {
            option: format!("{kind} ({option:?})"),
            driver,
        }
    }
}

pub(crate) fn check_thread(
    driver: &'static str,
    role: ThreadRole,
    options: &[ThreadOption],
) -> Result<(), RefusedOption> {
    let allow = |o: &ThreadOption| thread_allowed(role, o);
    match Rules::portable(&allow).first_rejected(options) {
        Some(o) => Err(RefusedOption::new(driver, o.kind_name(), o)),
        None => Ok(()),
    }
}

pub(crate) fn check_socket(
    driver: &'static str,
    role: SocketRole,
    options: &[SocketOption],
) -> Result<(), RefusedOption> {
    let allow = |o: &SocketOption| socket_allowed(role, o);
    match Rules::portable(&allow).first_rejected(options) {
        Some(o) => Err(RefusedOption::new(driver, o.kind_name(), o)),
        None => Ok(()),
    }
}

fn log_report<O: fmt::Debug>(driver: &str, what: &str, report: &Report<O>) {
    for a in &report.adjusted {
        tracing::warn!(
            option = ?a.option,
            effective = ?a.effective,
            reason = %a.reason,
            "{driver} {what} option adjusted"
        );
    }
    for s in &report.skipped {
        tracing::warn!(option = ?s.option, reason = %s.reason, "{driver} {what} option skipped");
    }
}

/// Applies `options` to the calling thread. The returned report holds guards
/// (MMCSS registration and the like) and must live as long as the thread.
pub(crate) fn apply_thread(
    driver: &'static str,
    role: ThreadRole,
    options: &[ThreadOption],
) -> io::Result<Report<ThreadOption>> {
    let allow = |o: &ThreadOption| thread_allowed(role, o);
    let report = ThreadOption::apply_all(options, &Rules::portable(&allow))?;
    log_report(driver, "thread", &report);
    Ok(report)
}

/// Binds a UDP socket to `addr` with `options` applied before the bind.
#[cfg(any(feature = "stmo", feature = "hspo"))]
pub(crate) fn bind_udp(
    driver: &'static str,
    role: SocketRole,
    addr: SocketAddr,
    options: &[SocketOption],
) -> io::Result<(std::net::UdpSocket, Report<SocketOption>)> {
    let allow = |o: &SocketOption| socket_allowed(role, o);
    let (socket, report) = sockets::bind_udp(addr, options, &Rules::portable(&allow))?;
    log_report(driver, "socket", &report);
    Ok((socket, report))
}

/// Connects a TCP stream to `addr` within `timeout`, with `options` applied
/// before the handshake.
#[cfg(any(feature = "rmi", feature = "hmi"))]
pub(crate) fn connect_tcp(
    driver: &'static str,
    role: SocketRole,
    addr: SocketAddr,
    timeout: std::time::Duration,
    options: &[SocketOption],
) -> io::Result<(std::net::TcpStream, Report<SocketOption>)> {
    let allow = |o: &SocketOption| socket_allowed(role, o);
    let (stream, report) =
        sockets::connect_tcp(addr, Some(timeout), options, &Rules::portable(&allow))?;
    log_report(driver, "socket", &report);
    Ok((stream, report))
}

/// What a connection's thread and socket options did, from the driver's
/// `tuning_report`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TuningReport {
    /// The I/O thread's options.
    pub thread: ReportSummary<ThreadOption>,
    /// The socket's options.
    pub socket: ReportSummary<SocketOption>,
}

#[cfg(feature = "py")]
mod py {
    use super::TuningReport;
    use pyo3::{IntoPyObject, prelude::*, types::PyDict};

    impl<'py> IntoPyObject<'py> for &TuningReport {
        type Target = PyDict;
        type Output = Bound<'py, PyDict>;
        type Error = PyErr;

        fn into_pyobject(self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
            let d = PyDict::new(py);
            d.set_item("thread", &self.thread)?;
            d.set_item("socket", &self.socket)?;
            Ok(d)
        }
    }
}

#[cfg(all(test, feature = "stmo", feature = "hspo", feature = "rmi"))]
mod tests {
    use super::*;

    #[test]
    fn cyclic_accepts_every_thread_option() {
        let time_constraint = ThreadOption::MacOsTimeConstraint {
            period_us: 8000,
            computation_us: 1000,
            constraint_us: 2000,
        };
        assert!(thread_allowed(ThreadRole::Cyclic, &time_constraint));
        assert!(!thread_allowed(ThreadRole::Stream, &time_constraint));
        assert!(thread_allowed(
            ThreadRole::Stream,
            &ThreadOption::RtPriority(80)
        ));
    }

    #[test]
    fn control_refuses_real_time_classes() {
        for refused in [
            ThreadOption::RtPriority(80),
            ThreadOption::UnixScheduler(Scheduler::Fifo(80)),
            ThreadOption::UnixScheduler(Scheduler::RoundRobin(10)),
            ThreadOption::WinPriority(ThreadPriority::TimeCritical),
            ThreadOption::WinMmcss("Pro Audio".into()),
        ] {
            assert!(
                !thread_allowed(ThreadRole::Control, &refused),
                "{refused:?}"
            );
        }
        for accepted in [
            ThreadOption::CpuAffinity(vec![1]),
            ThreadOption::LinuxNice(-5),
            ThreadOption::UnixScheduler(Scheduler::Batch),
            ThreadOption::WinPriority(ThreadPriority::Highest),
        ] {
            assert!(
                thread_allowed(ThreadRole::Control, &accepted),
                "{accepted:?}"
            );
        }
    }

    #[test]
    fn socket_roles() {
        assert!(socket_allowed(
            SocketRole::UdpCyclic,
            &SocketOption::SendBuffer(1 << 16)
        ));
        assert!(!socket_allowed(
            SocketRole::UdpStreamRx,
            &SocketOption::Dscp(46)
        ));
        assert!(socket_allowed(
            SocketRole::UdpStreamRx,
            &SocketOption::RecvBuffer(1 << 20)
        ));
        for role in [SocketRole::UdpCyclic, SocketRole::UdpStreamRx] {
            assert!(socket_allowed(role, &SocketOption::WinCpuAffinity(0)));
        }
        assert!(socket_allowed(
            SocketRole::TcpControl,
            &SocketOption::Dscp(46)
        ));
        assert!(socket_allowed(
            SocketRole::TcpControl,
            &SocketOption::BindDevice("eth0".into())
        ));
        assert!(!socket_allowed(
            SocketRole::TcpControl,
            &SocketOption::WinCpuAffinity(0)
        ));
        assert!(!socket_allowed(
            SocketRole::TcpControl,
            &SocketOption::RecvBuffer(1 << 20)
        ));
    }

    #[test]
    fn check_names_the_first_refused_option() {
        let err = check_thread(
            "rmi",
            ThreadRole::Control,
            &[ThreadOption::LinuxNice(0), ThreadOption::RtPriority(80)],
        )
        .unwrap_err();
        assert_eq!(err.driver, "rmi");
        assert_eq!(err.option, "rt_priority (RtPriority(80))");
        assert!(check_socket("hspo", SocketRole::UdpStreamRx, &[]).is_ok());
    }

    #[test]
    fn other_platform_options_are_skipped() {
        #[cfg(windows)]
        let foreign = ThreadOption::LinuxNice(0);
        #[cfg(not(windows))]
        let foreign = ThreadOption::WinDisablePowerThrottling;
        let report = apply_thread("stmo", ThreadRole::Cyclic, &[foreign]).unwrap();
        assert!(report.applied.is_empty());
        assert_eq!(report.skipped.len(), 1);
    }

    #[test]
    fn refused_option_fails_connect_before_any_io() {
        let config = crate::rmi::RmiDriverConfig::default_with_ip([192, 0, 2, 1]);
        let mut driver = crate::rmi::RmiDriver::new(config);
        let err = driver
            .connect(&[ThreadOption::RtPriority(80)], &[])
            .unwrap_err();
        assert!(matches!(
            err,
            crate::rmi::errors::RmiError::InvalidOption { driver: "rmi", .. }
        ));
    }
}
