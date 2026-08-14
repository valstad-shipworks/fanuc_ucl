//! Integration tests against an emulated Stream Motion controller.
//!
//! The emulator is a real in-process socket on snare's virtual network rather
//! than a tester-framework handler, so it can hold the state a controller
//! actually has: a command queue with a drain threshold, and a cycle clock.
//! It enforces the four rules the hardware faults on — the queue may not
//! overflow, it may not run dry once the robot is moving, no sequence number
//! may arrive twice, and every announced cycle must be answered. Anything the
//! driver does that would trip an e-stop shows up here as a recorded fault.

use std::collections::{HashSet, VecDeque};
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

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
}

impl ControllerCfg {
    fn new(drain_threshold: usize) -> Self {
        Self {
            drain_threshold,
            drop_from: u32::MAX,
            drop_count: 0,
        }
    }

    fn dropping(mut self, from: u32, count: u32) -> Self {
        self.drop_from = from;
        self.drop_count = count;
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
}

struct Controller {
    socket: snare::net::UdpSocket,
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
        self.seq = 0;
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
        if !self.started {
            return;
        }
        if self.draining && self.queue.pop_front().is_none() {
            // Real hardware e-stops here. The emulator records it and keeps
            // announcing cycles, so a test can still see how the driver
            // responds to the gap it caused.
            self.fault(format!("queue ran dry at cycle {}", self.seq));
        }

        self.seq += 1;
        if self.seq >= self.cfg.drop_from && self.seq < self.cfg.drop_from + self.cfg.drop_count {
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

fn run_controller(
    ip: IpAddr,
    cfg: ControllerCfg,
    report: Arc<Mutex<Report>>,
    ready: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
) {
    let socket = snare::net::UdpSocket::bind(SocketAddr::new(ip, STMO_PORT)).unwrap();
    socket.set_nonblocking(true).unwrap();
    ready.store(true, Ordering::SeqCst);

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
        loop {
            match c.socket.recv_from(&mut buf) {
                Ok((n, src)) if n > 0 => {
                    let data = buf[..n].to_vec();
                    c.handle(&data, src);
                }
                Ok(_) => break,
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        if Instant::now() >= next_cycle {
            next_cycle = Instant::now() + CYCLE;
            c.cycle();
        }
        std::thread::sleep(Duration::from_micros(200));
    }
}

fn run_stmo_test<F, R>(ip: Ipv4Addr, cfg: ControllerCfg, client: F) -> (Report, R)
where
    F: FnOnce(IpAddr, &Arc<Mutex<Report>>) -> R,
{
    snare::register_test();
    let addr = IpAddr::V4(ip);
    snare::add_ip_addr(addr);

    let report = Arc::new(Mutex::new(Report::default()));
    let ready = Arc::new(AtomicBool::new(false));
    let stop = Arc::new(AtomicBool::new(false));

    let handle = {
        let (report, ready, stop) = (report.clone(), ready.clone(), stop.clone());
        snare::thread::spawn(move || run_controller(addr, cfg, report, ready, stop))
    };
    while !ready.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(1));
    }

    let result = client(addr, &report);

    stop.store(true, Ordering::Relaxed);
    handle.join().unwrap();
    let taken = std::mem::take(&mut *report.lock().unwrap());
    (taken, result)
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
    driver.connect(None).unwrap();
    driver.start(2.0).unwrap();
    driver
}

/// Generous ceiling on how long a test waits for the emulator to work through
/// its cycles. Only reached if something has genuinely stalled — a loaded CI
/// runner is slower per cycle but still makes progress.
const CYCLE_WAIT_TIMEOUT: Duration = Duration::from_secs(30);

/// Waits for the emulator to announce `cycles` more cycles, draining received
/// statuses meanwhile so the consumer side is exercised too. Returns how many
/// statuses reached the consumer.
///
/// Progress is measured in controller cycles rather than wall time: a loaded
/// host completes far fewer per second, and every assertion here is about what
/// happened over a number of cycles, not over an interval.
fn drain_for_cycles(
    driver: &mut StreamMotionDriver,
    report: &Arc<Mutex<Report>>,
    cycles: u32,
) -> usize {
    let target = report.lock().unwrap().statuses_sent + cycles;
    let deadline = Instant::now() + CYCLE_WAIT_TIMEOUT;
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

    // A two-deep buffer is only two cycles of runway, so a host stall that long
    // starves the robot no matter what the driver does — real hardware would
    // fault too. Only the faults the driver is responsible for are asserted on.
    let driver_faults: Vec<&String> = report
        .faults
        .iter()
        .filter(|f| !f.contains("ran dry"))
        .collect();
    assert!(
        driver_faults.is_empty(),
        "controller faulted on the driver's account: {driver_faults:?}"
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
