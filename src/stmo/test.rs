//! Integration tests against an emulated Stream Motion controller.
//!
//! The emulator is a plain UDP socket inside the sim rather than a tester
//! handler, so it can hold the state a controller actually has: a command
//! queue with a drain threshold, and a cycle clock. It enforces the four rules
//! the hardware faults on — the queue may not overflow, it may not run dry
//! once the robot is moving, no sequence number may arrive twice, and every
//! announced cycle must be answered. Anything the driver does that would trip
//! an e-stop shows up here as a recorded fault.

use std::collections::{HashSet, VecDeque};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use snare::prelude::*;

use crate::joints::{JointFormat, JointTemplate};
use crate::stmo::buffer::CAPACITY;
use crate::stmo::proto::{
    MotionCommandPacket, RobotStatusPacket, RxPackets, TxPackets, VersionNumberResponsePacket,
};
use crate::stmo::{StmoStats, StreamMotionDriver};

/// Interpolation period the emulator runs at, matching real hardware.
const CYCLE: Duration = Duration::from_millis(8);
/// `READY_FOR_COMMANDS | COMMAND_RECEIVED`. The command-received bit is set
/// unconditionally, matching hardware, where it carries no information.
const STATUS_BITS: u8 = 0b0000_0011;
const STMO_PORT: u16 = 60015;
const PROTOCOL_VERSION: u32 = 2;

#[derive(Clone, Copy)]
struct ControllerCfg {
    /// How full the queue must get before the robot starts moving.
    drain_threshold: usize,
    /// First sequence whose status packet is deliberately not transmitted.
    drop_from: u32,
    /// How many consecutive statuses to drop from `drop_from`.
    drop_count: u32,
    /// Sequence of the first status after a start packet.
    first_seq: u32,
}

impl ControllerCfg {
    fn new(drain_threshold: usize) -> Self {
        Self {
            drain_threshold,
            drop_from: u32::MAX,
            drop_count: 0,
            first_seq: 1,
        }
    }

    fn dropping(mut self, from: u32, count: u32) -> Self {
        self.drop_from = from;
        self.drop_count = count;
        self
    }

    fn starting_at(mut self, first_seq: u32) -> Self {
        self.first_seq = first_seq;
        self
    }
}

/// What the emulator observed, read by the test once the run is over.
#[derive(Debug, Default)]
struct Report {
    faults: Vec<String>,
    commands_received: u32,
    statuses_sent: u32,
    statuses_dropped: u32,
    max_depth: usize,
    /// Sequences the controller received a command for.
    commanded_seqs: HashSet<u32>,
    /// Sequences whose status packet was deliberately suppressed.
    dropped_seqs: Vec<u32>,
    /// The driver's source address, so tests can apply link policy to it.
    driver_addr: Option<SocketAddr>,
    stops_received: u32,
    /// While set the controller stops cycling: it neither executes nor
    /// announces anything, as a controller held in a fault does.
    paused: bool,
}

struct Controller {
    socket: std::net::UdpSocket,
    cfg: ControllerCfg,
    report: Arc<Mutex<Report>>,
    peer: Option<SocketAddr>,
    started: bool,
    seq: u32,
    queue: VecDeque<u32>,
    draining: bool,
    seen: HashSet<u32>,
    faulted: bool,
}

impl Controller {
    fn fault(&mut self, why: String) {
        self.faulted = true;
        let mut r = self.report.lock().unwrap();
        // Only the first matters: a real controller e-stops and everything
        // after is a consequence.
        if r.faults.is_empty() {
            r.faults.push(why);
        }
    }

    fn send(&self, rx: RxPackets, to: SocketAddr) {
        let mut buf = [0u8; 512];
        let n = rx.encode_into(PROTOCOL_VERSION, &mut buf).unwrap();
        let _ = self.socket.send_to(&buf[..n], to);
    }

    fn reset_stream(&mut self) {
        self.queue.clear();
        self.draining = false;
        self.seen.clear();
        self.seq = self.cfg.first_seq.wrapping_sub(1);
    }

    fn handle(&mut self, data: &[u8], src: SocketAddr) {
        if self.peer != Some(src) {
            self.peer = Some(src);
            self.report.lock().unwrap().driver_addr = Some(src);
        }
        let Some(tx) = TxPackets::decode_from(data) else {
            return;
        };
        match tx {
            TxPackets::Start(_) => {
                self.started = true;
                self.reset_stream();
            }
            TxPackets::Stop(_) => {
                self.started = false;
                self.reset_stream();
                self.report.lock().unwrap().stops_received += 1;
            }
            TxPackets::VersionNumberRequest(_) => self.send(
                RxPackets::VersionNumberResponse(VersionNumberResponsePacket {
                    version: PROTOCOL_VERSION,
                }),
                src,
            ),
            TxPackets::MotionCommand(m) => {
                if !self.started || self.faulted {
                    return;
                }
                let seq = m.seq();
                if !self.seen.insert(seq) {
                    self.fault(format!("sequence {seq} received twice"));
                    return;
                }
                self.queue.push_back(seq);
                let depth = self.queue.len();
                {
                    let mut r = self.report.lock().unwrap();
                    r.commands_received += 1;
                    r.max_depth = r.max_depth.max(depth);
                    r.commanded_seqs.insert(seq);
                }
                if depth > CAPACITY as usize {
                    self.fault(format!("queue overflowed to {depth}"));
                }
                if !self.draining && depth >= self.cfg.drain_threshold {
                    self.draining = true;
                }
                if m.is_last_command() {
                    self.draining = false;
                    self.queue.clear();
                }
            }
            _ => {}
        }
    }

    /// One interpolation cycle: execute a queued command, then announce it.
    fn cycle(&mut self) {
        let Some(peer) = self.peer else { return };
        if !self.started || self.report.lock().unwrap().paused {
            return;
        }
        if self.draining && self.queue.pop_front().is_none() {
            // Real hardware e-stops here. The emulator records it and keeps
            // announcing cycles, so a test can still see how the driver
            // responds to the gap it caused.
            self.fault(format!("queue ran dry at cycle {}", self.seq));
        }

        self.seq = self.seq.wrapping_add(1);
        if self.seq.wrapping_sub(self.cfg.drop_from) < self.cfg.drop_count {
            let mut r = self.report.lock().unwrap();
            r.statuses_dropped += 1;
            r.dropped_seqs.push(self.seq);
            return;
        }
        self.report.lock().unwrap().statuses_sent += 1;
        let status = RobotStatusPacket::new(self.seq, STATUS_BITS, self.seq, [0.0; 9]);
        self.send(RxPackets::RobotStatus(status), peer);
    }
}

/// Runs the controller until `stop`: reads whatever arrives until the next
/// cycle is due, then runs that cycle.
fn run_controller(
    socket: UdpSocket,
    cfg: ControllerCfg,
    report: Arc<Mutex<Report>>,
    stop: Arc<AtomicBool>,
) {
    let mut c = Controller {
        socket,
        cfg,
        report,
        peer: None,
        started: false,
        seq: 0,
        queue: VecDeque::new(),
        draining: false,
        seen: HashSet::new(),
        faulted: false,
    };
    let mut buf = [0u8; 2048];
    let mut next_cycle = Instant::now() + CYCLE;

    while !stop.load(Ordering::Relaxed) {
        let now = Instant::now();
        if now >= next_cycle {
            next_cycle += CYCLE;
            c.cycle();
            continue;
        }
        c.socket.set_read_timeout(Some(next_cycle - now)).unwrap();
        if let Ok((n, src)) = c.socket.recv_from(&mut buf) {
            let data = buf[..n].to_vec();
            c.handle(&data, src);
        }
    }
}

/// Runs `client` beside the emulator in a deterministic sim: under snare
/// 2.0.0-alpha.1 a plain sim can skip a sleeper's deadline while other sims in
/// the process are polling, which here would show up as a skipped cycle.
fn run_stmo_test<F, R>(ip: Ipv4Addr, cfg: ControllerCfg, client: F) -> (Report, R)
where
    F: FnOnce(IpAddr, &Arc<Mutex<Report>>) -> R,
{
    let sim = Sim::builder()
        .deterministic()
        .strict_sockopts()
        .stuck_after(Duration::from_secs(30))
        .build();
    sim.run(|| {
        let addr = IpAddr::V4(ip);
        let report = Arc::new(Mutex::new(Report::default()));
        let stop = Arc::new(AtomicBool::new(false));

        let socket = UdpSocket::bind(SocketAddr::new(addr, STMO_PORT)).unwrap();
        let handle = {
            let (report, stop) = (report.clone(), stop.clone());
            std::thread::spawn(move || run_controller(socket, cfg, report, stop))
        };

        let result = client(addr, &report);

        stop.store(true, Ordering::Relaxed);
        handle.join().unwrap();
        let taken = std::mem::take(&mut *report.lock().unwrap());
        (taken, result)
    })
}

fn trajectory(points: usize) -> Vec<MotionCommandPacket> {
    (0..points)
        .map(|i| {
            let a = i as f64 * 0.01;
            MotionCommandPacket::try_from_joints(
                JointFormat::FanucDeg,
                JointTemplate::SIX,
                [a, a, a, a, a, a],
            )
            .unwrap()
        })
        .collect()
}

fn connected_driver(addr: IpAddr, buffer_size_before_drain: u8) -> StreamMotionDriver {
    let mut driver = StreamMotionDriver::new(addr, buffer_size_before_drain, false);
    driver.connect(&[], &[]).unwrap();
    driver.start(2.0).unwrap();
    driver
}

/// Waits for the emulator to announce `cycles` more cycles, draining received
/// statuses meanwhile so the consumer side is exercised too. Returns how many
/// statuses reached the consumer.
fn drain_for_cycles(
    driver: &mut StreamMotionDriver,
    report: &Arc<Mutex<Report>>,
    cycles: u32,
) -> usize {
    let target = report.lock().unwrap().statuses_sent + cycles;
    let deadline = Instant::now() + CYCLE * (2 * cycles + 10);
    let mut seen = 0;
    loop {
        seen += driver.pull_states().len();
        let sent = report.lock().unwrap().statuses_sent;
        if sent >= target {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "emulator stalled at {sent} of {target} cycles"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    seen
}

/// Streams a trajectory across `cycles` controller cycles.
fn stream_trajectory(
    addr: IpAddr,
    buffer_size_before_drain: u8,
    cycles: u32,
    report: &Arc<Mutex<Report>>,
) -> (StmoStats, usize) {
    let mut driver = connected_driver(addr, buffer_size_before_drain);
    driver
        .command_motion(trajectory(cycles as usize * 2))
        .unwrap();

    let seen = drain_for_cycles(&mut driver, report, cycles);

    let stats = driver.stats();
    driver.disconnect();
    (stats, seen)
}

#[test]
fn nominal_stream_never_faults_the_controller() {
    let (report, (stats, seen)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 1),
        ControllerCfg::new(5),
        |addr, report| stream_trajectory(addr, 5, 150, report),
    );

    assert!(
        report.faults.is_empty(),
        "controller faulted: {:?}",
        report.faults
    );
    assert!(
        report.commands_received > 50,
        "only {} commands reached the controller",
        report.commands_received
    );
    assert!(
        report.max_depth <= CAPACITY as usize,
        "queue reached {}",
        report.max_depth
    );
    assert_eq!(stats.send_failures, 0);
    assert_eq!(stats.underruns, 0);
    assert_eq!(stats.overflow_skips, 0);
    assert!(seen > 30, "consumer only saw {seen} statuses");
}

#[test]
fn a_shallow_drain_threshold_is_honoured() {
    let (report, (stats, _)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 2),
        ControllerCfg::new(2),
        |addr, report| stream_trajectory(addr, 2, 150, report),
    );

    assert!(
        report.faults.is_empty(),
        "controller faulted: {:?}",
        report.faults
    );
    // The robot starts moving after two commands rather than the default five,
    // and the driver keeps feeding it from that much shallower a buffer.
    assert!(
        report.max_depth >= 2,
        "queue never reached the threshold, peaking at {}",
        report.max_depth
    );
    // A catch-up burst can leave the queue settled above the threshold, which
    // is harmless, but never near the capacity it faults at.
    assert!(
        report.max_depth < CAPACITY as usize,
        "queue reached {} against a threshold of 2",
        report.max_depth
    );
    // The model must not claim more is queued than the controller ever held.
    assert!(
        stats.buffer_depth as usize <= report.max_depth,
        "driver modelled depth {} but the queue never exceeded {}",
        stats.buffer_depth,
        report.max_depth
    );
}

#[test]
fn lost_statuses_within_the_threshold_are_refilled() {
    let (report, (stats, _)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 3),
        ControllerCfg::new(5).dropping(20, 3),
        |addr, report| stream_trajectory(addr, 5, 80, report),
    );

    assert_eq!(report.statuses_dropped, 3);
    assert!(
        report.faults.is_empty(),
        "controller faulted after recoverable loss: {:?}",
        report.faults
    );
    assert_eq!(
        stats.lost_statuses, 3,
        "driver did not notice all three lost statuses"
    );
    // The sequences whose status never arrived were still commanded, which is
    // the whole point of the refill.
    let refilled: Vec<u32> = report
        .dropped_seqs
        .iter()
        .copied()
        .filter(|seq| report.commanded_seqs.contains(seq))
        .collect();
    assert_eq!(
        refilled, report.dropped_seqs,
        "lost sequences {:?} were not all refilled (got {refilled:?})",
        report.dropped_seqs
    );
    assert!(
        report.max_depth <= CAPACITY as usize,
        "queue reached {}",
        report.max_depth
    );
}

#[test]
fn loss_past_the_threshold_is_not_refilled() {
    let (report, (stats, _)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 4),
        ControllerCfg::new(5).dropping(20, 8),
        |addr, report| stream_trajectory(addr, 5, 80, report),
    );

    assert_eq!(report.statuses_dropped, 8);
    assert_eq!(
        stats.lost_statuses, 8,
        "driver did not notice all eight lost statuses"
    );
    // Past the drain threshold the queue has already emptied, so those
    // sequences must be left alone rather than burst into a faulted robot.
    let refilled: Vec<u32> = report
        .dropped_seqs
        .iter()
        .copied()
        .filter(|seq| report.commanded_seqs.contains(seq))
        .collect();
    assert!(
        refilled.is_empty(),
        "driver refilled {refilled:?} into a queue that had already run dry"
    );
    // Eight unanswered cycles against a five-deep prefill starves the robot,
    // but the driver must not compound it by repeating a sequence number or
    // overflowing what is left.
    assert!(
        report.faults.iter().all(|f| f.contains("ran dry")),
        "unexpected fault: {:?}",
        report.faults
    );
}

#[test]
fn a_blocked_transmit_is_retried_without_starving_consumers() {
    let (report, (stats, seen, during)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 5),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            driver.command_motion(trajectory(600)).unwrap();

            // Let the queue reach its steady depth before interfering.
            let before = drain_for_cycles(&mut driver, report, 30);
            let sut = report.lock().unwrap().driver_addr.unwrap();

            // Two cycles' worth of blocked transmit. The controller holds five,
            // so the retry has runway and must ride it out. Measured in the
            // emulator's cycles, so the block stays proportional to the runway
            // however slowly the host is running.
            block_transmit(sut, true);
            let during = drain_for_cycles(&mut driver, report, 2);
            block_transmit(sut, false);

            let after = drain_for_cycles(&mut driver, report, 60);
            let stats = driver.stats();
            driver.disconnect();
            (stats, before + during + after, during)
        },
    );

    assert!(
        report.faults.is_empty(),
        "controller faulted through a blocked transmit: {:?}",
        report.faults
    );
    assert!(
        stats.send_retries > 0,
        "transmit was never blocked; the test proved nothing"
    );
    // Statuses reaching the consumer across the blocked window prove the
    // receive path was pumped while the transmit side was stuck — end to end,
    // rather than through the retry loop's own bookkeeping.
    assert!(
        during > 0,
        "consumers were starved while the transmit was blocked"
    );
    assert!(seen > 20, "consumer only saw {seen} statuses");
}

/// Forces the driver's socket to refuse sends, so the retry path runs.
fn block_transmit(addr: SocketAddr, blocked: bool) {
    snare::set_udp_policy(addr, |p| {
        p.send_queue_depth = if blocked { Some(0) } else { None }
    });
}

fn controller_addr(addr: IpAddr) -> SocketAddr {
    SocketAddr::new(addr, STMO_PORT)
}

/// The driver's own socket, found by the controller it is connected to.
fn driver_socket(addr: IpAddr) -> snare::SocketEntry {
    snare::socket_table()
        .into_iter()
        .find(|s| s.kind == snare::SocketKind::Udp && s.peer == Some(controller_addr(addr)))
        .expect("the driver has no socket connected to the controller")
}

/// The address the driver's socket is bound at: link policy there shapes
/// only what arrives at the driver, the status stream.
fn status_path(addr: IpAddr) -> SocketAddr {
    driver_socket(addr)
        .local
        .expect("the driver's socket is unbound")
}

fn assert_no_faults(report: &Report) {
    assert!(
        report.faults.is_empty(),
        "controller faulted: {:?}",
        report.faults
    );
}

#[test]
fn random_status_loss_is_counted_exactly() {
    let (report, (stats, lost_on_wire)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 6),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            driver.command_motion(trajectory(600)).unwrap();
            drain_for_cycles(&mut driver, report, 10);
            let path = status_path(addr);
            snare::set_udp_policy(path, |p| p.loss_rate = 0.15);
            drain_for_cycles(&mut driver, report, 150);
            snare::set_udp_policy(path, |p| p.loss_rate = 0.0);
            drain_for_cycles(&mut driver, report, 5);
            let lost = driver_socket(addr).wire_lost;
            let stats = driver.stats();
            driver.disconnect();
            (stats, lost)
        },
    );

    assert!(lost_on_wire >= 10, "only {lost_on_wire} statuses were lost");
    assert_eq!(stats.lost_statuses, lost_on_wire, "{stats}");
    assert_no_faults(&report);
    assert_eq!(stats.underruns, 0, "{stats}");
    assert!(stats.catchup_commands > 0, "{stats}");
}

#[test]
fn duplicated_statuses_never_repeat_a_sequence() {
    let (report, stats) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 7),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            driver.command_motion(trajectory(400)).unwrap();
            drain_for_cycles(&mut driver, report, 10);
            // Each copy draws its own jitter, so a duplicate usually lands in
            // a later read than its original, but never after the next cycle.
            snare::set_udp_policy(status_path(addr), |p| {
                p.duplicate_rate = 0.5;
                p.jitter = Duration::from_millis(3);
            });
            drain_for_cycles(&mut driver, report, 120);
            let stats = driver.stats();
            driver.disconnect();
            stats
        },
    );

    assert_no_faults(&report);
    assert!(
        stats.stale_statuses > 0,
        "no duplicate reached the driver: {stats}"
    );
    assert_eq!(stats.lost_statuses, 0, "{stats}");
    assert_eq!(stats.missed_status_cycles, 0, "{stats}");
    assert_eq!(stats.underruns, 0, "{stats}");
}

#[test]
fn reordered_statuses_are_never_answered_twice() {
    let (report, stats) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 8),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            driver.command_motion(trajectory(400)).unwrap();
            drain_for_cycles(&mut driver, report, 10);
            // Up to 14 ms of jitter against an 8 ms cycle: statuses overtake
            // each other.
            snare::set_udp_policy(status_path(addr), |p| p.jitter = Duration::from_millis(14));
            drain_for_cycles(&mut driver, report, 150);
            let stats = driver.stats();
            driver.disconnect();
            stats
        },
    );

    assert_no_faults(&report);
    assert!(
        stats.stale_statuses > 0,
        "no status arrived out of order: {stats}"
    );
    assert!(stats.catchup_commands > 0, "{stats}");
    assert_eq!(stats.underruns, 0, "{stats}");
}

/// Streams with `latency` and `jitter` on both directions of the link.
fn stream_over_a_slow_link(
    ip: Ipv4Addr,
    latency: Duration,
    jitter: Duration,
) -> (Report, StmoStats) {
    run_stmo_test(ip, ControllerCfg::new(5), move |addr, report| {
        let mut driver = connected_driver(addr, 5);
        driver.command_motion(trajectory(500)).unwrap();
        drain_for_cycles(&mut driver, report, 10);
        for path in [status_path(addr), controller_addr(addr)] {
            snare::set_udp_policy(path, |p| {
                p.latency = latency;
                p.jitter = jitter;
            });
        }
        drain_for_cycles(&mut driver, report, 150);
        let stats = driver.stats();
        driver.disconnect();
        stats
    })
}

fn assert_every_cycle_answered_once(report: &Report, stats: &StmoStats) {
    assert_no_faults(report);
    assert_eq!(stats.lost_statuses, 0, "{stats}");
    assert_eq!(stats.missed_status_cycles, 0, "{stats}");
    assert_eq!(stats.catchup_commands, 0, "{stats}");
    assert_eq!(stats.stale_statuses, 0, "{stats}");
    assert_eq!(stats.underruns, 0, "{stats}");
}

#[test]
fn latency_and_jitter_inside_a_cycle_keep_every_cycle_answered() {
    let (report, stats) = stream_over_a_slow_link(
        Ipv4Addr::new(10, 0, 5, 9),
        Duration::from_millis(3),
        Duration::from_millis(1),
    );
    assert_every_cycle_answered_once(&report, &stats);
    assert!(
        (7_700..=8_300).contains(&stats.cycle_us),
        "measured a {} µs cycle through 1 ms of jitter",
        stats.cycle_us
    );
}

#[test]
fn a_round_trip_longer_than_a_cycle_is_absorbed_by_the_buffer() {
    let (report, stats) = stream_over_a_slow_link(
        Ipv4Addr::new(10, 0, 5, 10),
        Duration::from_millis(10),
        Duration::from_millis(2),
    );
    assert_every_cycle_answered_once(&report, &stats);
}

#[test]
fn a_stream_across_the_u32_sequence_wrap_never_faults() {
    let first = u32::MAX - 40;
    let (report, (stats, _)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 14),
        ControllerCfg::new(5)
            .starting_at(first)
            .dropping(u32::MAX - 1, 3),
        |addr, report| stream_trajectory(addr, 5, 120, report),
    );

    assert_no_faults(&report);
    assert_eq!(report.dropped_seqs, vec![u32::MAX - 1, u32::MAX, 0]);
    assert_eq!(stats.lost_statuses, 3, "{stats}");
    assert_eq!(stats.stale_statuses, 0, "{stats}");
    for seq in &report.dropped_seqs {
        assert!(
            report.commanded_seqs.contains(seq),
            "sequence {seq} lost across the wrap was not refilled"
        );
    }
    assert!(
        report.commanded_seqs.iter().any(|&s| s > first) && report.commanded_seqs.contains(&5),
        "the stream did not run through the wrap: {:?}",
        report.commanded_seqs
    );
}

#[test]
fn a_gap_longer_than_sixteen_cycles_is_still_counted() {
    let (report, (stats, _)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 11),
        ControllerCfg::new(5).dropping(20, 20),
        |addr, report| stream_trajectory(addr, 5, 80, report),
    );

    assert_eq!(report.statuses_dropped, 20);
    assert!(
        report.faults.iter().any(|f| f.contains("ran dry")),
        "twenty unanswered cycles should have starved the controller"
    );
    assert_eq!(stats.lost_statuses, 20, "{stats}");
    assert!(
        stats.underruns > 0,
        "the controller ran dry but the driver counted no underrun: {stats}"
    );
}

#[test]
fn a_controller_that_goes_silent_holds_the_stream_until_it_returns() {
    let (report, (silence, waited, done, stats)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 12),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            let batch = driver.command_motion(trajectory(40)).unwrap();
            drain_for_cycles(&mut driver, report, 10);

            report.lock().unwrap().paused = true;
            let t0 = Instant::now();
            let silence = driver
                .recv_status_timeout(Duration::from_millis(50))
                .unwrap();
            let waited = t0.elapsed();
            assert!(!batch.is_set(), "a batch completed with nobody answering");
            assert!(driver.is_connected());
            assert!(!driver.has_connection_errored());

            report.lock().unwrap().paused = false;
            let done = batch.wait_timeout(Duration::from_secs(2)).is_ok();
            let stats = driver.stats();
            driver.disconnect();
            (silence, waited, done, stats)
        },
    );

    assert!(
        silence.is_none(),
        "a status arrived from a silent controller"
    );
    assert!(
        waited >= Duration::from_millis(50) && waited < Duration::from_millis(51),
        "a 50 ms status wait returned after {waited:?}"
    );
    assert!(done, "the stream did not resume once the controller did");
    assert_no_faults(&report);
    assert_eq!(stats.lost_statuses, 0, "{stats}");
    assert_eq!(stats.missed_status_cycles, 0, "{stats}");
    assert_eq!(stats.underruns, 0, "{stats}");
}

/// The error an ICMP port unreachable leaves on a connected UDP socket.
#[cfg(unix)]
const PORT_UNREACHABLE: i32 = libc::ECONNREFUSED;
/// `WSAECONNRESET`.
#[cfg(windows)]
const PORT_UNREACHABLE: i32 = 10054;

#[test]
fn an_icmp_port_unreachable_mid_stream_is_survived() {
    let (report, (landed, cleared, stats, errored, seen)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 13),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            driver.command_motion(trajectory(400)).unwrap();
            drain_for_cycles(&mut driver, report, 20);
            let socket = driver_socket(addr);
            snare::inject_icmp_port_unreachable(socket.local.unwrap(), controller_addr(addr));
            let landed = snare::socket_entry(socket.id).unwrap().pending_error;
            let seen = drain_for_cycles(&mut driver, report, 60);
            let cleared = snare::socket_entry(socket.id).unwrap().pending_error;
            let stats = driver.stats();
            let errored = driver.has_connection_errored();
            driver.disconnect();
            (landed, cleared, stats, errored, seen)
        },
    );

    assert_eq!(
        landed,
        Some(PORT_UNREACHABLE),
        "the ICMP error never reached the socket"
    );
    assert_eq!(cleared, None, "the driver never consumed the error");
    assert!(!errored, "one unreachable marked the connection errored");
    assert!(
        seen > 30,
        "only {seen} statuses reached the consumer afterwards"
    );
    assert_no_faults(&report);
    assert!(stats.send_failures <= 1, "{stats}");
    #[cfg(target_os = "linux")]
    assert_eq!(
        stats.tx_errors, 1,
        "IP_RECVERR did not report the unreachable: {stats}"
    );
}

#[test]
fn a_transmit_blocked_past_the_runway_marks_the_connection_errored() {
    let (_, (before, stats, errored)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 14),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            driver.command_motion(trajectory(600)).unwrap();
            drain_for_cycles(&mut driver, report, 20);
            let before = driver.has_connection_errored();
            let sut = report.lock().unwrap().driver_addr.unwrap();
            block_transmit(sut, true);
            drain_for_cycles(&mut driver, report, 120);
            block_transmit(sut, false);
            let stats = driver.stats();
            let errored = driver.has_connection_errored();
            driver.disconnect();
            (before, stats, errored)
        },
    );

    assert!(!before);
    assert!(
        stats.send_failures >= 4,
        "a second of blocked transmit failed only {} sends",
        stats.send_failures
    );
    assert!(
        errored,
        "sends failed for a second without the connection erroring"
    );
}

#[test]
fn reconnecting_starts_a_fresh_stream_on_a_fresh_socket() {
    let (report, (first_closed, stops_after_first, second_done)) = run_stmo_test(
        Ipv4Addr::new(10, 0, 5, 15),
        ControllerCfg::new(5),
        |addr, report| {
            let mut driver = connected_driver(addr, 5);
            driver
                .command_motion(trajectory(30))
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            let first = driver_socket(addr).id;
            driver.disconnect();
            assert!(!driver.is_connected());
            let first_closed = snare::socket_entry(first).unwrap().closed_at.is_some();
            let stops_after_first = report.lock().unwrap().stops_received;

            driver.connect(&[], &[]).unwrap();
            driver.start(2.0).unwrap();
            assert_ne!(driver_socket(addr).id, first);
            let second_done = driver
                .command_motion(trajectory(30))
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .is_ok();
            drain_for_cycles(&mut driver, report, 5);
            driver.disconnect();
            (first_closed, stops_after_first, second_done)
        },
    );

    assert!(first_closed, "disconnect left the first socket open");
    assert_eq!(stops_after_first, 1, "disconnect did not stop the stream");
    assert!(second_done, "the second connection never streamed");
    assert_eq!(report.stops_received, 2);
    assert_no_faults(&report);
}
