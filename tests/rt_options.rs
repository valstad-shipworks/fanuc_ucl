//! The real-time option lists every driver takes at connect: an option its
//! role refuses fails before any socket exists, another platform's option is
//! skipped, accepted socket options land on the driver's own socket, and a
//! thread option the OS will not apply fails the connect instead of being lost.
#![cfg(all(
    snare,
    unix,
    any(
        all(
            target_os = "linux",
            target_env = "gnu",
            any(target_arch = "x86_64", target_arch = "aarch64")
        ),
        target_os = "macos",
        windows
    )
))]

mod common;

use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpListener};
use std::sync::atomic::AtomicU32;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use fanuc_ucl::hmi::{HmiDriver, HmiError};
use fanuc_ucl::hspo::{
    HspoBrokerError, HspoReceiver, broker_tuning_report, destroy_broker, initialize_broker,
};
use fanuc_ucl::rmi::errors::RmiError;
use fanuc_ucl::rmi::{RmiDriver, RmiDriverConfig};
use fanuc_ucl::stmo::{StreamMotionDriver, StreamMotionError};
use fanuc_ucl::{SocketOption, ThreadOption};
use fast_talker::rt::Scheduler;
use snare::prelude::*;
use snare::{SocketEntry, UnmodelledOption, socket_table, sockets_bound};

use common::{builder, hmi_server, rmi_server, run, sim};

const NIC: &str = "ft0";
const NIC_IP: Ipv4Addr = Ipv4Addr::new(10, 30, 0, 2);
const ROBOT: IpAddr = IpAddr::V4(Ipv4Addr::new(10, 30, 0, 1));

fn nic_sim() -> Sim {
    builder()
        .nic(
            NicSpec::new(NIC)
                .address(IpNet::new(IpAddr::V4(NIC_IP), 24))
                .station(ROBOT),
        )
        .build()
}

#[cfg(not(target_os = "macos"))]
fn unprivileged_sim() -> Sim {
    builder().privileges(snare::Privileges::none()).build()
}

/// A thread option every driver role accepts that an unprivileged process
/// cannot apply: Linux refuses lowering the nice value. macOS applies, clamps
/// or reports every option the roles accept, so it has none.
#[cfg(not(target_os = "macos"))]
fn unappliable() -> Vec<ThreadOption> {
    vec![ThreadOption::LinuxNice(-10)]
}

/// Accepted by every role, but written for Windows.
fn foreign_thread() -> ThreadOption {
    ThreadOption::WinDisablePowerThrottling
}

/// Accepted by the UDP roles, but written for Linux. Those roles accept no
/// socket option written only for macOS or Windows, so Linux has none.
fn foreign_udp_socket() -> Vec<SocketOption> {
    if cfg!(target_os = "linux") {
        vec![]
    } else {
        vec![SocketOption::LinuxBusyPoll(50)]
    }
}

/// `SO_RCVBUF`/`SO_SNDBUF` as getsockopt reports a requested size: Linux
/// doubles it for bookkeeping overhead.
fn reported_buffer(requested: usize) -> u32 {
    let n = requested as u32;
    if cfg!(target_os = "linux") { 2 * n } else { n }
}

fn open_sockets(kind: SocketKind, peer: SocketAddr) -> Vec<SocketEntry> {
    socket_table()
        .into_iter()
        .filter(|s| s.kind == kind && s.peer == Some(peer))
        .collect()
}

/// Options the sim does not model on `s`, but for fast-talker's
/// `getsockopt(SO_DOMAIN)` before it sets `IP_RECVERR` on Linux: the sim
/// refuses that read, and fast-talker takes the family from `getsockname`.
fn unmodelled(s: &SocketEntry) -> Vec<UnmodelledOption> {
    #[cfg(target_os = "linux")]
    let domain_probe = Some(UnmodelledOption::Get {
        level: libc::SOL_SOCKET,
        name: libc::SO_DOMAIN,
    });
    #[cfg(not(target_os = "linux"))]
    let domain_probe = None;
    s.unmodelled_options
        .iter()
        .copied()
        .filter(|o| Some(*o) != domain_probe)
        .collect()
}

fn stmo_socket() -> Vec<SocketEntry> {
    open_sockets(SocketKind::Udp, SocketAddr::new(ROBOT, 60015))
}

/// Lets threads that already failed finish dropping what they own.
#[cfg(not(target_os = "macos"))]
fn settle() {
    std::thread::sleep(Duration::from_millis(1));
}

#[test]
fn stmo_skips_another_platforms_options() {
    sim().run(|| {
        let mut driver = StreamMotionDriver::new(ROBOT, 5, false);
        driver
            .connect(&[foreign_thread()], &foreign_udp_socket())
            .unwrap();
        assert!(driver.is_connected());
        driver.disconnect();
    });
}

#[test]
fn stmo_reports_what_its_options_did() {
    sim().run(|| {
        let mut driver = StreamMotionDriver::new(ROBOT, 5, false);
        assert!(driver.tuning_report().is_none());
        let affinity = SocketOption::WinCpuAffinity(0);
        driver
            .connect(&[foreign_thread()], std::slice::from_ref(&affinity))
            .unwrap();
        let report = driver.tuning_report().unwrap();
        let skipped_thread: Vec<_> = report.thread.skipped.iter().map(|s| &s.option).collect();
        let skipped_socket: Vec<_> = report.socket.skipped.iter().map(|s| &s.option).collect();
        assert_eq!(skipped_thread, [&foreign_thread()]);
        if cfg!(windows) {
            assert_eq!(report.socket.applied, [affinity]);
        } else {
            assert_eq!(skipped_socket, [&affinity]);
        }
        driver.disconnect();
        assert!(driver.tuning_report().is_none());
    });
}

#[test]
fn stmo_socket_options_land_on_its_socket() {
    nic_sim().run(|| {
        let mut driver = StreamMotionDriver::new(ROBOT, 5, false);
        let mut options = vec![
            SocketOption::RecvBuffer(1 << 16),
            SocketOption::SendBuffer(1 << 15),
            SocketOption::BindDevice(NIC.into()),
            SocketOption::Dscp(46),
        ];
        if cfg!(target_os = "linux") {
            options.push(SocketOption::LinuxPriority(5));
        }
        driver.connect(&[], &options).unwrap();
        assert_eq!(
            driver.tuning_report().unwrap().socket.applied.len(),
            options.len()
        );
        let sockets = stmo_socket();
        assert_eq!(sockets.len(), 1, "{sockets:?}");
        let s = &sockets[0];
        assert_eq!(s.rcvbuf, reported_buffer(1 << 16));
        assert_eq!(s.sndbuf, reported_buffer(1 << 15));
        assert_eq!(s.bound_device.as_deref(), Some(NIC));
        assert!(unmodelled(s).is_empty(), "{:?}", s.unmodelled_options);
        driver.disconnect();
        assert!(stmo_socket().is_empty(), "disconnect left the socket open");
    });
}

#[test]
fn stmo_unknown_interface_fails_cleanly() {
    nic_sim().run(|| {
        let mut driver = StreamMotionDriver::new(ROBOT, 5, false);
        let err = driver
            .connect(&[], &[SocketOption::BindDevice("nope0".into())])
            .unwrap_err();
        assert!(matches!(err, StreamMotionError::Io(_)), "{err:?}");
        assert!(!driver.is_connected());
        assert!(
            stmo_socket().is_empty(),
            "the failed bind left a socket open"
        );
        driver.connect(&[], &[]).unwrap();
        assert!(driver.is_connected());
        driver.disconnect();
    });
}

#[cfg(not(target_os = "macos"))]
#[test]
fn stmo_thread_option_failure_fails_connect() {
    unprivileged_sim().run(|| {
        let mut driver = StreamMotionDriver::new(ROBOT, 5, false);
        let err = driver.connect(&unappliable(), &[]).unwrap_err();
        assert!(matches!(err, StreamMotionError::Io(_)), "{err:?}");
        assert!(!driver.is_connected());
        settle();
        assert!(
            stmo_socket().is_empty(),
            "the failed connect left a socket open"
        );
    });
}

/// The broker is process-global, so tests that start one take turns.
static BROKER: Mutex<()> = Mutex::new(());

/// Holds the broker for one test and destroys whatever it started, even when
/// the test fails, so the next one starts from nothing.
struct BrokerTurn(#[allow(dead_code)] MutexGuard<'static, ()>);

impl BrokerTurn {
    fn take() -> Self {
        let turn = BrokerTurn(BROKER.lock().unwrap_or_else(|e| e.into_inner()));
        destroy_broker(false);
        turn
    }
}

impl Drop for BrokerTurn {
    fn drop(&mut self) {
        destroy_broker(false);
    }
}

fn broker_addr() -> SocketAddr {
    SocketAddr::new(IpAddr::V4(NIC_IP), 60001)
}

fn broker_running() -> bool {
    HspoReceiver::try_new(ROBOT, 4, Duration::from_millis(16)).is_ok()
}

#[test]
fn hspo_refuses_options_before_binding() {
    let _turn = BrokerTurn::take();
    sim().run(|| {
        let time_constraint = ThreadOption::MacOsTimeConstraint {
            period_us: 4000,
            computation_us: 500,
            constraint_us: 1000,
        };
        let cases: [(Vec<ThreadOption>, Vec<SocketOption>); 5] = [
            (vec![time_constraint], vec![]),
            (vec![], vec![SocketOption::Dscp(46)]),
            (vec![], vec![SocketOption::SendBuffer(1 << 16)]),
            (vec![], vec![SocketOption::DontFragment(true)]),
            (vec![], vec![SocketOption::LinuxPriority(1)]),
        ];
        for (thread, socket) in cases {
            let err = initialize_broker(broker_addr(), &thread, &socket).unwrap_err();
            assert!(
                matches!(err, HspoBrokerError::InvalidOption { driver: "hspo", .. }),
                "{err:?}"
            );
            assert!(sockets_bound(broker_addr()).is_empty());
            assert!(!broker_running());
        }
    });
}

#[test]
fn hspo_skips_another_platforms_options() {
    let _turn = BrokerTurn::take();
    sim().run(|| {
        initialize_broker(
            broker_addr(),
            &[ThreadOption::WinMmcss("Pro Audio".into())],
            &foreign_udp_socket(),
        )
        .unwrap();
        assert!(broker_running());
        destroy_broker(true);
    });
}

#[test]
fn hspo_socket_options_land_on_its_socket() {
    let _turn = BrokerTurn::take();
    nic_sim().run(|| {
        let options = [
            SocketOption::RecvBuffer(1 << 17),
            SocketOption::BindDevice(NIC.into()),
        ];
        initialize_broker(broker_addr(), &[], &options).unwrap();
        assert_eq!(broker_tuning_report().unwrap().socket.applied, options);
        let sockets = sockets_bound(broker_addr());
        assert_eq!(sockets.len(), 1, "{sockets:?}");
        assert_eq!(sockets[0].rcvbuf, reported_buffer(1 << 17));
        assert_eq!(sockets[0].bound_device.as_deref(), Some(NIC));
        destroy_broker(true);
        assert!(broker_tuning_report().is_none());
        assert!(
            sockets_bound(broker_addr()).is_empty(),
            "destroying the broker left its socket open"
        );
    });
}

#[test]
fn hspo_unknown_interface_fails_cleanly() {
    let _turn = BrokerTurn::take();
    nic_sim().run(|| {
        let err = initialize_broker(
            broker_addr(),
            &[],
            &[SocketOption::BindDevice("nope0".into())],
        )
        .unwrap_err();
        assert!(matches!(err, HspoBrokerError::Io(_)), "{err:?}");
        assert!(!broker_running());
        assert!(sockets_bound(broker_addr()).is_empty());
        initialize_broker(broker_addr(), &[], &[]).expect("the port is still free");
        destroy_broker(true);
    });
}

#[cfg(not(target_os = "macos"))]
#[test]
fn hspo_thread_option_failure_fails_initialize() {
    let _turn = BrokerTurn::take();
    unprivileged_sim().run(|| {
        let err = initialize_broker(broker_addr(), &unappliable(), &[]).unwrap_err();
        assert!(matches!(err, HspoBrokerError::Io(_)), "{err:?}");
        assert!(!broker_running());
        assert!(sockets_bound(broker_addr()).is_empty());
        initialize_broker(broker_addr(), &[], &[]).expect("the port is still free");
        destroy_broker(true);
    });
}

fn rmi_config() -> RmiDriverConfig {
    RmiDriverConfig::default_with_ip(ROBOT)
}

/// Options the request/response roles refuse: real-time classes, and socket
/// options that defeat TCP autotuning or do nothing for this traffic.
fn refused_by_control() -> [(Vec<ThreadOption>, Vec<SocketOption>); 4] {
    [
        (vec![ThreadOption::RtPriority(80)], vec![]),
        (
            vec![ThreadOption::UnixScheduler(Scheduler::Fifo(10))],
            vec![],
        ),
        (vec![], vec![SocketOption::RecvBuffer(1 << 16)]),
        (vec![], vec![SocketOption::WinCpuAffinity(0)]),
    ]
}

#[test]
fn rmi_refuses_options_before_any_connection() {
    sim().run(|| {
        let _control = TcpListener::bind(SocketAddr::new(ROBOT, 16001)).unwrap();
        for (thread, socket) in refused_by_control() {
            let mut driver = RmiDriver::new(rmi_config());
            let err = driver.connect(&thread, &socket).unwrap_err();
            assert!(
                matches!(err, RmiError::InvalidOption { driver: "rmi", .. }),
                "{err:?}"
            );
            assert!(!driver.is_connected());
            let opened = socket_table()
                .into_iter()
                .chain(snare::closed_sockets())
                .filter(|s| s.kind == SocketKind::TcpStream)
                .count();
            assert_eq!(opened, 0, "a refused option still opened a connection");
        }
    });
}

#[test]
fn rmi_skips_another_platforms_options() {
    sim().run(|| {
        run(vec![rmi_server(ROBOT)], || {
            let mut driver = RmiDriver::new(rmi_config());
            driver.connect(&[foreign_thread()], &[]).unwrap();
            assert!(driver.is_connected());
            driver.disconnect().unwrap();
        });
    });
}

#[test]
fn rmi_socket_options_apply_to_both_connections() {
    sim().run(|| {
        run(vec![rmi_server(ROBOT)], || {
            let mut options = vec![SocketOption::Dscp(46)];
            if cfg!(target_os = "linux") {
                options.push(SocketOption::LinuxPriority(3));
            }
            let mut driver = RmiDriver::new(rmi_config());
            driver.connect(&[], &options).unwrap();
            assert_eq!(driver.tuning_report().unwrap().socket.applied, options);
            let handshake: Vec<_> = snare::closed_sockets()
                .into_iter()
                .filter(|s| s.peer == Some(SocketAddr::new(ROBOT, 16001)))
                .collect();
            let session = open_sockets(SocketKind::TcpStream, SocketAddr::new(ROBOT, 16002));
            assert_eq!(handshake.len(), 1, "the handshake connection is still open");
            assert_eq!(session.len(), 1, "{session:?}");
            for s in handshake.iter().chain(&session) {
                assert!(
                    s.unmodelled_options.is_empty(),
                    "{:?}",
                    s.unmodelled_options
                );
            }
            driver.disconnect().unwrap();
        });
    });
}

#[cfg(not(target_os = "macos"))]
#[test]
fn rmi_thread_option_failure_fails_connect() {
    unprivileged_sim().run(|| {
        run(vec![rmi_server(ROBOT)], || {
            let mut driver = RmiDriver::new(rmi_config());
            let err = driver.connect(&unappliable(), &[]).unwrap_err();
            assert!(matches!(err, RmiError::CommunicationError(_)), "{err:?}");
            assert!(!driver.is_connected());
            settle();
            let session = open_sockets(SocketKind::TcpStream, SocketAddr::new(ROBOT, 16002));
            assert!(
                session.is_empty(),
                "the failed connect left {session:?} open"
            );
        });
    });
}

#[test]
fn hmi_refuses_options_before_any_connection() {
    sim().run(|| {
        let _listener = TcpListener::bind(SocketAddr::new(ROBOT, 60008)).unwrap();
        for (thread, socket) in refused_by_control() {
            let mut driver = HmiDriver::new(ROBOT);
            let err = driver
                .connect(Some(Duration::from_secs(1)), &thread, &socket)
                .unwrap_err();
            assert!(
                matches!(err, HmiError::InvalidOption { driver: "hmi", .. }),
                "{err:?}"
            );
            assert!(!driver.is_connected());
            let opened = socket_table()
                .into_iter()
                .chain(snare::closed_sockets())
                .filter(|s| s.kind == SocketKind::TcpStream)
                .count();
            assert_eq!(opened, 0, "a refused option still opened a connection");
        }
    });
}

#[test]
fn hmi_accepts_its_options_and_skips_another_platforms() {
    nic_sim().run(|| {
        let requests = Arc::new(AtomicU32::new(0));
        run(vec![hmi_server(ROBOT, requests)], || {
            let mut socket = vec![SocketOption::Dscp(46), SocketOption::BindDevice(NIC.into())];
            if cfg!(target_os = "linux") {
                socket.push(SocketOption::LinuxPriority(3));
            }
            let mut driver = HmiDriver::new(ROBOT);
            driver
                .connect(Some(Duration::from_secs(1)), &[foreign_thread()], &socket)
                .unwrap();
            let report = driver.tuning_report().unwrap();
            assert_eq!(report.socket.applied, socket);
            assert_eq!(report.thread.skipped.len(), 1);
            let session = open_sockets(SocketKind::TcpStream, SocketAddr::new(ROBOT, 60008));
            assert_eq!(session.len(), 1, "{session:?}");
            assert_eq!(session[0].bound_device.as_deref(), Some(NIC));
            assert!(session[0].unmodelled_options.is_empty());
            driver.disconnect(false).unwrap();
        });
    });
}

#[cfg(not(target_os = "macos"))]
#[test]
fn hmi_thread_option_failure_fails_connect() {
    unprivileged_sim().run(|| {
        let requests = Arc::new(AtomicU32::new(0));
        let seen = requests.clone();
        run(vec![hmi_server(ROBOT, requests)], || {
            let mut driver = HmiDriver::new(ROBOT);
            let err = driver
                .connect(Some(Duration::from_secs(1)), &unappliable(), &[])
                .unwrap_err();
            assert!(matches!(err, HmiError::Io(_)), "{err:?}");
            assert!(!driver.is_connected());
            settle();
            let session = open_sockets(SocketKind::TcpStream, SocketAddr::new(ROBOT, 60008));
            assert!(
                session.is_empty(),
                "the failed connect left {session:?} open"
            );
        });
        assert_eq!(
            seen.load(std::sync::atomic::Ordering::SeqCst),
            0,
            "the handshake ran on a runner whose options failed"
        );
    });
}
