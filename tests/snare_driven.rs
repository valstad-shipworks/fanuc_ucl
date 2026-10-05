//! The drivers under a deterministic sim with exact virtual durations: the
//! orderings and cycle counts the controller protocol depends on, checked
//! against fake controllers and servers running beside the driver threads.
#![cfg(all(
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

use std::collections::BTreeMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use common::{hmi_server, rmi_server, run, sim};
use fanuc_ucl::hmi::HmiDriver;
use fanuc_ucl::joints::{JointFormat, JointTemplate};
use fanuc_ucl::rmi::{RmiDriver, RmiDriverConfig};
use fanuc_ucl::stmo::proto::{
    IoType, MotionCommandPacket, RobotStatusPacket, RxPackets, TxPackets,
    VersionNumberResponsePacket,
};
use fanuc_ucl::stmo::{StmoHandle, StreamMotionDriver};
use snare::sched::block_on;

const CYCLE: Duration = Duration::from_millis(8);
const STMO_PORT: u16 = 60015;
const PROTOCOL_VERSION: u32 = 2;
/// `READY_FOR_COMMANDS | COMMAND_RECEIVED`.
const STATUS_BITS: u8 = 0b0000_0011;

/// One command as the fake controller saw it.
#[derive(Debug, Clone, Copy)]
struct Seen {
    j1: f64,
    read_io: Option<(IoType, u16, u16)>,
}

#[derive(Debug, Default)]
struct CtlLog {
    faults: Vec<String>,
    statuses_sent: u32,
    /// Commands by sequence, in the order they arrived.
    commands: BTreeMap<u32, Vec<Seen>>,
    /// First sequence a command was received for.
    first_commanded: Option<u32>,
    /// Per stop packet, the highest sequence commanded since the last start.
    stopped_after: Vec<Option<u32>>,
}

#[derive(Clone)]
struct Ctl {
    log: Arc<Mutex<CtlLog>>,
    stop: Arc<AtomicBool>,
    mute: Arc<AtomicBool>,
}

impl Ctl {
    fn new() -> Self {
        Self {
            log: Arc::default(),
            stop: Arc::default(),
            mute: Arc::default(),
        }
    }

    fn log(&self) -> MutexGuard<'_, CtlLog> {
        self.log.lock().unwrap()
    }
}

/// A Stream Motion controller that announces a cycle every 8 ms of virtual
/// time, reading whatever arrived since the previous cycle first. Every cycle
/// after the first command must have been answered by exactly one command
/// carrying its sequence number.
fn fake_controller(socket: UdpSocket, ctl: Ctl, phase: Duration) {
    socket.set_nonblocking(true).unwrap();
    let mut peer = None;
    let mut started = false;
    let mut seq: u32 = 0;
    let mut buf = [0u8; 2048];
    let mut next = Instant::now() + CYCLE + phase;
    let send = |rx: RxPackets, to: SocketAddr| {
        let mut out = [0u8; 512];
        let n = rx.encode_into(PROTOCOL_VERSION, &mut out).unwrap();
        socket.send_to(&out[..n], to).unwrap();
    };
    while !ctl.stop.load(Ordering::SeqCst) {
        std::thread::sleep(next.saturating_duration_since(Instant::now()));
        next += CYCLE;
        let mut version_requested = false;
        loop {
            match socket.recv_from(&mut buf) {
                Ok((n, src)) => {
                    peer = Some(src);
                    match TxPackets::decode_from(&buf[..n]) {
                        Some(TxPackets::Start(_)) => {
                            started = true;
                            seq = 0;
                            let mut log = ctl.log();
                            log.commands.clear();
                            log.first_commanded = None;
                        }
                        Some(TxPackets::Stop(_)) => {
                            started = false;
                            let mut log = ctl.log();
                            let last = log.commands.keys().last().copied();
                            log.stopped_after.push(last);
                        }
                        Some(TxPackets::VersionNumberRequest(_)) => version_requested = true,
                        Some(TxPackets::MotionCommand(m)) => {
                            let mut log = ctl.log();
                            log.first_commanded.get_or_insert(m.seq());
                            log.commands.entry(m.seq()).or_default().push(Seen {
                                j1: m.commanded_position()[0],
                                read_io: m.read_io(),
                            });
                        }
                        _ => {}
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("controller socket: {e}"),
            }
        }
        let Some(to) = peer else { continue };
        if version_requested {
            send(
                RxPackets::VersionNumberResponse(VersionNumberResponsePacket {
                    version: PROTOCOL_VERSION,
                }),
                to,
            );
        }
        if !started || ctl.mute.load(Ordering::SeqCst) {
            continue;
        }
        {
            let mut log = ctl.log();
            if let Some(first) = log.first_commanded
                && seq >= first
            {
                let got = log.commands.get(&seq).map_or(0, Vec::len);
                if got != 1 {
                    log.faults
                        .push(format!("cycle {seq} answered by {got} commands"));
                }
            }
            log.statuses_sent += 1;
        }
        seq += 1;
        let status = RobotStatusPacket::new(seq, STATUS_BITS, seq, [0.0; 9]);
        send(RxPackets::RobotStatus(status), to);
    }
}

fn point(j1: f64) -> MotionCommandPacket {
    MotionCommandPacket::try_from_joints(
        JointFormat::FanucDeg,
        JointTemplate::SIX,
        [j1, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    .unwrap()
}

fn stmo_ip(last: u8) -> IpAddr {
    IpAddr::V4(Ipv4Addr::new(10, 7, 0, last))
}

fn connected(ip: IpAddr) -> StreamMotionDriver {
    let mut driver = StreamMotionDriver::new(ip, 5, false);
    driver.connect(&[], &[]).unwrap();
    driver.start(2.0).unwrap();
    driver
}

/// A small delay that differs per iteration, so the thread that enqueues a
/// change lands at a different point of the cycle each time.
fn jitter(i: u32) {
    let us = (i.wrapping_mul(2_654_435_761) >> 24) % 400;
    std::thread::sleep(Duration::from_micros(us as u64));
}

fn controller_participant(ip: IpAddr, ctl: &Ctl) -> Box<dyn FnOnce() + Send> {
    phased_controller(ip, ctl, Duration::ZERO)
}

fn phased_controller(ip: IpAddr, ctl: &Ctl, phase: Duration) -> Box<dyn FnOnce() + Send> {
    let socket = UdpSocket::bind(SocketAddr::new(ip, STMO_PORT)).unwrap();
    let ctl = ctl.clone();
    Box::new(move || fake_controller(socket, ctl, phase))
}

/// Streams `rows` setpoints the way a reactive caller does: each chunk is
/// queued only once the previous one's handle resolves, after a delay standing
/// in for the caller's own work.
fn stream_reactively(driver: &mut StreamMotionDriver, rows: usize, chunk: usize, base: f64) {
    let points: Vec<_> = (0..rows).map(|i| point(base + i as f64 * 0.01)).collect();
    let chunks: Vec<_> = points.chunks(chunk).map(<[_]>::to_vec).collect();
    let last = chunks.len() - 1;
    let mut pending: Option<StmoHandle> = None;
    for (i, c) in chunks.into_iter().enumerate() {
        if let Some(h) = pending.take() {
            h.wait().unwrap();
            jitter(i as u32);
        }
        pending = Some(driver.command_motion_with(c, i != last).unwrap());
    }
    if let Some(h) = pending {
        h.wait().unwrap();
    }
}

#[test]
fn start_takes_one_controller_cycle_of_virtual_time() {
    sim().run(|| {
        let ip = stmo_ip(1);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let spent = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = StreamMotionDriver::new(ip, 5, false);
            driver.connect(&[], &[]).unwrap();
            let t0 = Instant::now();
            driver.start(2.0).unwrap();
            let spent = t0.elapsed();
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            spent
        });
        assert!(spent >= CYCLE, "start returned after {spent:?}");
        assert!(
            spent < CYCLE + Duration::from_millis(1),
            "start took {spent:?}"
        );
    });
}

#[test]
fn a_chunked_stream_answers_every_cycle_exactly_once() {
    sim().run(|| {
        const ROWS: usize = 1500;
        const CHUNK: usize = 512;
        let ip = stmo_ip(2);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let stats = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            let rows: Vec<_> = (0..ROWS).map(|i| point(i as f64 * 0.01)).collect();
            let chunks: Vec<_> = rows.chunks(CHUNK).map(<[_]>::to_vec).collect();
            let last = chunks.len() - 1;
            let mut handles = Vec::new();
            for (i, chunk) in chunks.into_iter().enumerate() {
                handles.push(driver.command_motion_with(chunk, i != last).unwrap());
            }
            for h in &handles {
                h.wait_timeout(Duration::from_secs(60)).unwrap();
            }
            std::thread::sleep(CYCLE * 4);
            let stats = driver.stats();
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            stats
        });
        let log = ctl.log();
        assert!(log.faults.is_empty(), "controller faults: {:?}", log.faults);
        let real = log
            .commands
            .values()
            .flatten()
            .filter(|s| s.j1 != (ROWS - 1) as f64 * 0.01)
            .count();
        assert_eq!(real, ROWS - 1, "controller saw {real} distinct setpoints");
        assert_eq!(stats.missed_status_cycles, 0, "{stats}");
        assert_eq!(stats.catchup_commands, 0, "{stats}");
        assert_eq!(stats.send_retries, 0, "{stats}");
        assert_eq!(stats.lost_statuses, 0, "{stats}");
        assert_eq!(stats.mid_stream_fillers, 0, "{stats}");
        assert_eq!(stats.underruns, 0, "{stats}");
    });
}

#[test]
fn a_handle_wait_times_out_on_the_virtual_clock() {
    sim().run(|| {
        let ip = stmo_ip(3);
        let ctl = Ctl::new();
        let (stop, mute) = (ctl.stop.clone(), ctl.mute.clone());
        let (res, waited) = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            mute.store(true, Ordering::SeqCst);
            std::thread::sleep(CYCLE * 2);
            let handle = driver.command_motion(vec![point(1.0)]).unwrap();
            let t0 = Instant::now();
            let res = handle.wait_timeout(Duration::from_secs(2));
            let waited = t0.elapsed();
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            (res, waited)
        });
        assert!(res.is_err(), "an unanswerable batch was reported sent");
        assert!(
            waited >= Duration::from_secs(2)
                && waited < Duration::from_secs(2) + Duration::from_micros(1),
            "a 2 s wait returned after {waited:?}"
        );
    });
}

/// Iterations of the tests that race a change against the reply it must not
/// affect.
const RACES: u32 = 200;

/// The first sequence whose command carried `j1`.
fn first_seq_with(log: &CtlLog, j1: f64) -> Option<u32> {
    log.commands
        .iter()
        .find(|(_, seen)| seen.iter().any(|s| s.j1 == j1))
        .map(|(seq, _)| *seq)
}

#[test]
fn a_command_enqueued_while_reacting_to_a_status_answers_the_next_one() {
    sim().run(|| {
        let ip = stmo_ip(4);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let reacted = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            let mut reacted = Vec::new();
            for i in 0..RACES {
                let status = block_on(driver.next_status()).unwrap();
                jitter(i);
                driver
                    .command_motion(vec![point(100.0 + i as f64)])
                    .unwrap();
                reacted.push(status.seq);
            }
            std::thread::sleep(CYCLE * 3);
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            reacted
        });
        let log = ctl.log();
        assert!(log.faults.is_empty(), "controller faults: {:?}", log.faults);
        for (i, k) in reacted.iter().enumerate() {
            let answered = first_seq_with(&log, 100.0 + i as f64);
            assert_eq!(
                answered,
                Some(k + 1),
                "command {i}, enqueued while reacting to status {k}, answered {answered:?}"
            );
        }
    });
}

#[test]
fn a_hold_read_io_change_applies_from_the_next_status() {
    sim().run(|| {
        let ip = stmo_ip(5);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let reacted = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            driver
                .command_motion(vec![point(1.0)])
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            let mut reacted = Vec::new();
            for i in 0..RACES {
                let status = block_on(driver.next_status()).unwrap();
                jitter(i);
                driver.set_hold_read_io(Some((IoType::DI, 1000 + i as u16, 1)));
                reacted.push(status.seq);
            }
            std::thread::sleep(CYCLE * 3);
            driver.set_hold_read_io(None);
            std::thread::sleep(CYCLE * 3);
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            reacted
        });
        let log = ctl.log();
        assert!(log.faults.is_empty(), "controller faults: {:?}", log.faults);
        let read_io = |seq: u32| log.commands.get(&seq).and_then(|s| s[0].read_io);
        for (i, k) in reacted.iter().enumerate() {
            let want = Some((IoType::DI, 1000 + i as u16, 1));
            assert_ne!(read_io(*k), want, "change {i} applied to status {k} itself");
            assert_eq!(
                read_io(k + 1),
                want,
                "change {i} missing from status {}",
                k + 1
            );
        }
        let last = *log.commands.keys().last().unwrap();
        assert_eq!(
            read_io(last),
            None,
            "clearing the hold read_io did not stick"
        );
    });
}

#[test]
fn next_status_never_returns_a_status_from_before_the_call() {
    sim().run(|| {
        let ip = stmo_ip(6);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let (k, stored, next) = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            let k = block_on(driver.next_status()).unwrap().seq;
            std::thread::sleep(CYCLE * 5 + CYCLE / 2);
            // Queuing a batch drains the received statuses into the driver's
            // store, so the call below starts with older statuses on hand.
            driver.command_motion(vec![point(1.0)]).unwrap();
            let stored = driver.pull_states().len();
            std::thread::sleep(CYCLE / 4);
            driver.command_motion(vec![point(2.0)]).unwrap();
            let next = block_on(driver.next_status()).unwrap().seq;
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            (k, stored, next)
        });
        assert_eq!(stored, 5, "expected five statuses on hand after {k}");
        assert_eq!(next, k + 6);
    });
}

#[test]
fn hspo_wait_for_returns_the_first_packet_at_its_virtual_instant() {
    sim().run(|| {
        use fanuc_ucl::hspo::{HspoReceiver, destroy_broker, initialize_broker};
        let broker = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 7, 1, 1)), 60001);
        let robot = IpAddr::V4(Ipv4Addr::new(10, 7, 1, 2));
        let ready = Arc::new(AtomicBool::new(false));
        let emitter = {
            let ready = ready.clone();
            let socket = UdpSocket::bind(SocketAddr::new(robot, 60002)).unwrap();
            Box::new(move || {
                while !ready.load(Ordering::SeqCst) {
                    std::thread::sleep(Duration::from_micros(100));
                }
                std::thread::sleep(Duration::from_millis(5));
                socket.send_to(&joint_packet(1), broker).unwrap();
            }) as Box<dyn FnOnce() + Send>
        };
        let (got, waited) = run(vec![emitter], move || {
            initialize_broker(broker, &[], &[]).unwrap();
            let receiver = HspoReceiver::try_new(robot, 16, Duration::from_millis(20)).unwrap();
            let t0 = Instant::now();
            ready.store(true, Ordering::SeqCst);
            let got = receiver.joint.wait_for(Duration::from_secs(3));
            let waited = t0.elapsed();
            destroy_broker(true);
            (got.map(|p| p.clock), waited)
        });
        assert_eq!(got, Some(1));
        assert!(
            waited >= Duration::from_millis(5) && waited < Duration::from_millis(6),
            "packet sent at +5 ms arrived after {waited:?}"
        );
    });
}

/// An HSPO joint-angles datagram: big-endian fixed-width fields, type 4.
fn joint_packet(clock: u32) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&1u32.to_be_bytes());
    out.extend_from_slice(&0u32.to_be_bytes());
    out.extend_from_slice(&clock.to_be_bytes());
    out.extend_from_slice(&4u16.to_be_bytes());
    out.extend_from_slice(&1u16.to_be_bytes());
    for _ in 0..9 {
        out.extend_from_slice(&0f32.to_be_bytes());
    }
    out.extend_from_slice(&0u32.to_be_bytes());
    out.extend_from_slice(&0u32.to_be_bytes());
    out
}

#[test]
fn rmi_and_hmi_connect_and_disconnect_without_hanging() {
    sim().run(|| {
        let ip = IpAddr::V4(Ipv4Addr::new(10, 7, 2, 1));
        let hmi_requests = Arc::new(AtomicU32::new(0));
        let servers = vec![rmi_server(ip), hmi_server(ip, hmi_requests.clone())];
        run(servers, move || {
            let mut rmi = RmiDriver::new(RmiDriverConfig {
                address: ip,
                expected_major_version: 7,
                software_options: vec![],
                buffer_cnt: 8,
                timeout: Duration::from_secs(2),
            });
            rmi.connect(&[], &[]).unwrap();
            assert!(rmi.is_connected());
            let bye = rmi.disconnect().unwrap();
            assert!(!rmi.is_connected());
            drop(bye);

            let mut hmi = HmiDriver::new(ip);
            hmi.connect(Some(Duration::from_secs(2)), &[], &[]).unwrap();
            assert!(hmi.is_connected());
            hmi.disconnect(false).unwrap();
            assert!(!hmi.is_connected());
        });
        assert!(hmi_requests.load(Ordering::SeqCst) >= 3);
    });
}

#[test]
fn chunks_queued_when_the_previous_one_resolves_leave_no_gap() {
    sim().run(|| {
        let ip = stmo_ip(7);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let stats = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            stream_reactively(&mut driver, 400, 7, 0.0);
            std::thread::sleep(CYCLE * 2);
            let stats = driver.stats();
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            stats
        });
        let log = ctl.log();
        assert!(log.faults.is_empty(), "controller faults: {:?}", log.faults);
        assert_eq!(stats.mid_stream_fillers, 0, "{stats}");
        assert_eq!(stats.missed_status_cycles, 0, "{stats}");
        assert_eq!(stats.catchup_commands, 0, "{stats}");
        assert_eq!(stats.underruns, 0, "{stats}");
    });
}

#[test]
fn a_late_chunk_is_counted_as_a_mid_stream_filler() {
    sim().run(|| {
        let ip = stmo_ip(8);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let (before, after) = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            let stats = driver.stats_handle();
            let before = stats.snapshot();
            let h = driver
                .command_motion_with(vec![point(1.0), point(2.0)], true)
                .unwrap();
            h.wait().unwrap();
            std::thread::sleep(CYCLE * 3);
            driver.command_motion(vec![point(3.0)]).unwrap();
            std::thread::sleep(CYCLE * 3);
            let after = stats.snapshot();
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            (before, after)
        });
        assert_eq!(
            after.mid_stream_fillers - before.mid_stream_fillers,
            3,
            "{after}"
        );
        assert!(after.idle_holds > before.idle_holds, "{after}");
    });
}

#[test]
fn two_controllers_out_of_phase_stream_without_gaps() {
    sim().run(|| {
        let (ip_a, ip_b) = (stmo_ip(9), stmo_ip(10));
        let (ctl_a, ctl_b) = (Ctl::new(), Ctl::new());
        let stops = [ctl_a.stop.clone(), ctl_b.stop.clone()];
        let client = |ip: IpAddr, base: f64| {
            move || {
                let mut driver = connected(ip);
                stream_reactively(&mut driver, 300, 16, base);
                std::thread::sleep(CYCLE * 2);
                let stats = driver.stats();
                driver.disconnect();
                stats
            }
        };
        let setup = snare::sched::setup_scope("test-spawn");
        let controllers = [
            std::thread::spawn(phased_controller(ip_a, &ctl_a, Duration::ZERO)),
            std::thread::spawn(phased_controller(ip_b, &ctl_b, CYCLE / 2)),
        ];
        let a = std::thread::spawn(client(ip_a, 0.0));
        let b = std::thread::spawn(client(ip_b, 50.0));
        drop(setup);
        let (a, b) = (a.join().unwrap(), b.join().unwrap());
        for s in &stops {
            s.store(true, Ordering::SeqCst);
        }
        for h in controllers {
            h.join().unwrap();
        }
        for (log, stats) in [(ctl_a.log(), a), (ctl_b.log(), b)] {
            assert!(log.faults.is_empty(), "controller faults: {:?}", log.faults);
            assert_eq!(stats.mid_stream_fillers, 0, "{stats}");
            assert_eq!(stats.missed_status_cycles, 0, "{stats}");
            assert_eq!(stats.underruns, 0, "{stats}");
        }
    });
}

#[test]
fn a_control_loop_command_answers_the_status_after_the_one_it_reacts_to() {
    sim().run(|| {
        let ip = stmo_ip(11);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let reacted = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            driver
                .command_motion(vec![point(1.0)])
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            let mut reacted = Vec::new();
            {
                let mut session = driver.control_loop().unwrap();
                for i in 0..RACES / 2 {
                    let status = session.wait_for_status(CYCLE * 4).unwrap();
                    jitter(i);
                    session.send_command(point(200.0 + i as f64)).unwrap();
                    reacted.push(status.seq);
                }
            }
            std::thread::sleep(CYCLE * 3);
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            reacted
        });
        let log = ctl.log();
        let repeated: Vec<_> = log.faults.iter().filter(|f| !f.contains("by 0")).collect();
        assert!(repeated.is_empty(), "controller faults: {repeated:?}");
        for (i, k) in reacted.iter().enumerate() {
            let answered = first_seq_with(&log, 200.0 + i as f64);
            assert_eq!(
                answered,
                Some(k + 1),
                "command {i}, sent while reacting to status {k}, answered {answered:?}"
            );
        }
    });
}

#[test]
fn a_stop_made_while_reacting_to_a_status_replaces_the_next_command() {
    sim().run(|| {
        let ip = stmo_ip(12);
        let ctl = Ctl::new();
        let stop = ctl.stop.clone();
        let reacted = run(vec![controller_participant(ip, &ctl)], move || {
            let mut driver = connected(ip);
            let mut reacted = Vec::new();
            for i in 0..20 {
                if i > 0 {
                    driver.start(2.0).unwrap();
                }
                driver
                    .command_motion(vec![point(i as f64)])
                    .unwrap()
                    .wait_timeout(Duration::from_secs(1))
                    .unwrap();
                std::thread::sleep(CYCLE * 2);
                let k = block_on(driver.next_status()).unwrap().seq;
                jitter(i);
                driver.stop();
                std::thread::sleep(CYCLE * 3);
                reacted.push(k);
            }
            driver.disconnect();
            stop.store(true, Ordering::SeqCst);
            reacted
        });
        let log = ctl.log();
        assert!(log.faults.is_empty(), "controller faults: {:?}", log.faults);
        assert!(log.stopped_after.len() >= reacted.len());
        for (i, k) in reacted.iter().enumerate() {
            assert_eq!(
                log.stopped_after[i],
                Some(*k),
                "stop {i}, made while reacting to status {k}, came after {:?}",
                log.stopped_after[i]
            );
        }
    });
}
