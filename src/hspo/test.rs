#![cfg(unix)]

use snare::{Bytes, Sim, TesterAction, run_testers, udp_tester};

use super::*;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};

fn encode_packet<T: bincode::Encode>(packet: &T) -> Vec<u8> {
    let config = bincode::config::standard()
        .with_fixed_int_encoding()
        .with_big_endian();
    bincode::encode_to_vec(packet, config).unwrap()
}

fn make_tcp_position_packet(clock: u32) -> TcpCartesianPositionPacket {
    TcpCartesianPositionPacket {
        version: 1,
        index: 0,
        clock,
        typ: 1,
        motion_group: 1,
        x: 100.0,
        y: 200.0,
        z: 300.0,
        yaw: 10.0,
        pitch: 20.0,
        roll: 30.0,
        status: 0,
        io: 0,
    }
}

fn make_joint_angles_packet(clock: u32) -> JointAnglesPacket {
    JointAnglesPacket {
        version: 1,
        index: 0,
        clock,
        typ: 4,
        motion_group: 1,
        joints: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0],
        status: 0,
        io: 0,
    }
}

fn make_variables_packet(clock: u32) -> VariablesPacket {
    VariablesPacket {
        version: 1,
        index: 0,
        clock,
        typ: 16,
        data: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
    }
}

#[test]
fn test_packet_type_from_bytes() {
    let tcp_bytes = encode_packet(&make_tcp_position_packet(1000));
    assert_eq!(
        PacketType::from_bytes(&tcp_bytes, 12),
        PacketType::TcpCartesianPosition
    );

    let joint_bytes = encode_packet(&make_joint_angles_packet(1000));
    assert_eq!(
        PacketType::from_bytes(&joint_bytes, 12),
        PacketType::JointAngles
    );

    let var_bytes = encode_packet(&make_variables_packet(1000));
    assert_eq!(
        PacketType::from_bytes(&var_bytes, 12),
        PacketType::Variables
    );

    // Unknown type value (typ = 99)
    let mut unknown_bytes = tcp_bytes.clone();
    unknown_bytes[12] = 0;
    unknown_bytes[13] = 99;
    assert_eq!(
        PacketType::from_bytes(&unknown_bytes, 12),
        PacketType::Unknown
    );

    // Too short
    assert_eq!(PacketType::from_bytes(&[0; 13], 12), PacketType::Unknown);
}

#[test]
fn test_packet_encode_decode_roundtrip() {
    let config = bincode::config::standard()
        .with_fixed_int_encoding()
        .with_big_endian();

    let tcp_pkt = make_tcp_position_packet(42);
    let bytes = encode_packet(&tcp_pkt);
    let (decoded, _): (TcpCartesianPositionPacket, _) =
        bincode::decode_from_slice(&bytes, config).unwrap();
    assert_eq!(decoded, tcp_pkt);

    let joint_pkt = make_joint_angles_packet(42);
    let bytes = encode_packet(&joint_pkt);
    let (decoded, _): (JointAnglesPacket, _) = bincode::decode_from_slice(&bytes, config).unwrap();
    assert_eq!(decoded, joint_pkt);

    let var_pkt = make_variables_packet(42);
    let bytes = encode_packet(&var_pkt);
    let (decoded, _): (VariablesPacket, _) = bincode::decode_from_slice(&bytes, config).unwrap();
    assert_eq!(decoded, var_pkt);
}

#[test]
fn test_stream_clock_index_gate() {
    let sc = StreamClock::default();
    assert_eq!(sc.accept(0, 1000, 0), Some(1000));
    assert_eq!(sc.accept(1, 1008, 8), Some(1008));

    // Lower index than the newest seen: disregarded.
    assert_eq!(sc.accept(0, 5000, 16), None);

    // A controller that leaves the index fixed keeps delivering (equal is not lower).
    assert_eq!(sc.accept(1, 1016, 24), Some(1016));
    assert_eq!(sc.accept(2, 1024, 32), Some(1024));
}

#[test]
fn test_stream_clock_wrap() {
    let sc = StreamClock::default();
    let cycle = u32::MAX as u64 + 1;
    let pre = u32::MAX - 5;

    assert_eq!(sc.accept(0, pre, 0), Some(pre as u64));
    // Strictly-newer packet whose clock stepped backward across the boundary: a wrap.
    // 8µs of receive time separates them, so the counter ran `cycle - pre + 4`.
    assert_eq!(sc.accept(1, 4, 8), Some(pre as u64 + 8));
    assert_eq!(sc.accept(2, 12, 16), Some(pre as u64 + 16));
    // The cycle it measured is the full range, since that is what the times say.
    assert_eq!(sc.accept(3, 20, 24), Some(pre as u64 + 24));
    assert!(
        pre as u64 + 24 > cycle - 100,
        "still inside the first cycle"
    );
}

#[test]
fn test_stream_clock_wrap_fixed_index() {
    let sc = StreamClock::default();
    let pre = u32::MAX - 5;

    // Index never advances, so the boundary check alone must catch the wrap.
    assert_eq!(sc.accept(0, pre, 0), Some(pre as u64));
    assert_eq!(sc.accept(0, 4, 8), Some(pre as u64 + 8));
}

#[test]
fn test_stream_clock_wrap_at_a_short_cycle() {
    // The R-30iB counter cycles at ~1.29e8µs, nowhere near the field's range.
    // Assuming 2^32 here inserted a 4166s jump into every packet still buffered
    // from before the wrap, which pinned a sweep's corners onto one pose.
    let sc = StreamClock::default();
    let cycle = 128_850_307u64;
    let pre = (cycle - 341) as u32;
    let step = 4_000u64;
    let base_sys = 1_787_083_917_000_000u64;

    assert_eq!(sc.accept(0, pre, base_sys), Some(pre as u64));
    // 4ms later the counter has rolled to 3659; absolute time must advance by 4ms.
    let post = (pre as u64 + step - cycle) as u32;
    assert_eq!(post, 3_659);
    assert_eq!(
        sc.accept(1, post, base_sys + step),
        Some(pre as u64 + step),
        "the wrap advances the clock by the elapsed time, not by 2^32"
    );
    assert_eq!(
        sc.accept(2, post + step as u32, base_sys + 2 * step),
        Some(pre as u64 + 2 * step)
    );

    // The pre-wrap packet, resolved after the wrap, keeps its own receive time.
    let epoch = |micros: u64| SystemTime::UNIX_EPOCH + Duration::from_micros(micros);
    assert_eq!(sc.system_time_of(0, pre), Some(epoch(base_sys)));
    assert_eq!(sc.system_time_of(1, post), Some(epoch(base_sys + step)));
}

#[test]
fn test_stream_clock_wrap_across_a_stall_of_several_cycles() {
    // Nothing arrives for long enough that the counter rolls over three times.
    // Only one backward step is visible, but the receive times still say how
    // much time actually passed, so both packets keep their own stamps.
    let sc = StreamClock::default();
    let cycle = 128_850_307u64;
    let stall = 2 * cycle + (cycle - 200);
    let epoch = |micros: u64| SystemTime::UNIX_EPOCH + Duration::from_micros(micros);

    assert_eq!(sc.accept(0, 500, 0), Some(500));
    assert_eq!(sc.accept(1, 300, stall), Some(stall + 500));
    assert_eq!(sc.system_time_of(0, 500), Some(epoch(0)));
    assert_eq!(sc.system_time_of(1, 300), Some(epoch(stall)));

    // The skipped cycles must not be learned as the cycle length: a later
    // single-boundary wrap is the shorter measurement and wins.
    let next = stall + cycle - 100;
    assert_eq!(sc.accept(2, 200, next), Some(next + 500));
    assert_eq!(sc.system_time_of(2, 200), Some(epoch(next)));
}

#[test]
fn test_stream_clock_reorder_dropped() {
    let sc = StreamClock::default();
    assert_eq!(sc.accept(10, 1000, 0), Some(1000));
    assert_eq!(sc.accept(12, 1016, 16), Some(1016));

    // The 1008 sample (index 11) arrives after index 12: dropped, so no spurious
    // ~4.29e9 wrap from the backward clock step.
    assert_eq!(sc.accept(11, 1008, 24), None);
    assert_eq!(sc.accept(13, 1024, 32), Some(1024));
}

#[test]
fn test_channel_recv_async() {
    use std::task::{Context, Poll, Wake, Waker};

    fn block_on<F: Future>(fut: F) -> F::Output {
        struct ThreadWaker(std::thread::Thread);
        impl Wake for ThreadWaker {
            fn wake(self: Arc<Self>) {
                self.0.unpark();
            }
        }
        let waker = Waker::from(Arc::new(ThreadWaker(std::thread::current())));
        let mut cx = Context::from_waker(&waker);
        let mut fut = std::pin::pin!(fut);
        loop {
            match fut.as_mut().poll(&mut cx) {
                Poll::Ready(v) => return v,
                Poll::Pending => std::thread::park(),
            }
        }
    }

    let (tx, rx) = bounded::<VariablesPacket>(4);
    let channel = HspoChannel::new(rx, Arc::new(StreamClock::default()));

    let sender = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(20));
        tx.send(make_variables_packet(42)).unwrap();
        // tx drops here, disconnecting the channel
    });

    assert_eq!(
        block_on(channel.recv_async()),
        Some(make_variables_packet(42))
    );
    assert_eq!(block_on(channel.recv_async()), None);
    sender.join().unwrap();
}

#[test]
fn test_stream_clock_system_time_of() {
    let sc = StreamClock::default();
    let epoch = |micros: u64| SystemTime::UNIX_EPOCH + Duration::from_micros(micros);

    // Nothing accepted yet: no offset to reconstruct from.
    assert_eq!(sc.system_time_of(0, 1000), None);

    // System time runs 5_000µs ahead of the controller clock.
    sc.accept(0, 1000, 6_000);
    sc.accept(1, 1008, 6_008);
    assert_eq!(sc.system_time_of(0, 1000), Some(epoch(6_000)));
    assert_eq!(sc.system_time_of(1, 1008), Some(epoch(6_008)));
}

#[test]
fn test_stream_clock_system_time_of_across_wrap() {
    let sc = StreamClock::default();
    let span = u32::MAX as u64 + 1;
    let offset = 5_000u64;
    let epoch = |micros: u64| SystemTime::UNIX_EPOCH + Duration::from_micros(micros);

    let pre_wrap_clock = u32::MAX - 5;
    sc.accept(0, pre_wrap_clock, pre_wrap_clock as u64 + offset);
    sc.accept(1, 4, span + 4 + offset);
    sc.accept(2, 12, span + 12 + offset);

    // A pre-wrap packet read after the wrap still resolves with wrap count 0.
    assert_eq!(
        sc.system_time_of(0, pre_wrap_clock),
        Some(epoch(pre_wrap_clock as u64 + offset))
    );
    // Post-wrap packets resolve with the folded wrap.
    assert_eq!(sc.system_time_of(1, 4), Some(epoch(span + 4 + offset)));
    assert_eq!(sc.system_time_of(2, 12), Some(epoch(span + 12 + offset)));
}

#[test]
fn test_stream_clock_batch_buffered_across_a_wrap() {
    // The production failure: a consumer drains the channel every 500ms, so at
    // any moment a batch of packets is buffered and gets resolved only after
    // the broker has already folded in a wrap. Every one of them has to come
    // back with its own receive time. Folding in 2^32 instead of the ~1.29e8
    // the counter really ran put the whole pre-wrap half of the batch 4166s
    // into the past, and the sweep reading them pinned its corners onto one
    // pose. Rates and values are the R-30iB's: 1333µs per index, 4000µs per
    // three, wrapping 128849966 -> 3659.
    let sc = StreamClock::default();
    let cycle = 128_850_307u64;
    let step = 1_333u64;
    let base_sys = 1_787_083_917_167_106u64;
    let start_clock = 128_849_966u64;
    let start_index = 33_905u32;

    // 500ms of packets before the wrap, then 200ms after it.
    let mut sent: Vec<(u32, u32, u64)> = Vec::new();
    for i in 0..525u64 {
        let index = start_index + i as u32;
        let clock = ((start_clock + i * step) % cycle) as u32;
        sent.push((index, clock, base_sys + i * step));
    }
    // The wrap is in there, and only once.
    let wraps = sent.windows(2).filter(|w| w[1].1 < w[0].1).count();
    assert_eq!(wraps, 1, "one rollover in the batch");

    for &(index, clock, sys) in &sent {
        assert!(
            sc.accept(index, clock, sys).is_some(),
            "index {index} gated"
        );
    }

    // Now resolve the whole batch, offset anchored on the newest packet — what
    // a consumer draining after the wrap actually sees.
    for &(index, clock, sys) in &sent {
        let at = sc
            .system_time_of(index, clock)
            .expect("a packet the broker accepted resolves");
        let micros = at
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64;
        assert_eq!(
            micros,
            sys,
            "packet {index} came back {} µs from its receive time",
            micros as i64 - sys as i64
        );
    }
}

#[test]
fn test_stream_clock_wrap_inside_a_drained_backlog() {
    // Without kernel rx timestamps — a non-unix host, snare's shim socket, or
    // Linux before its deferred static key turns generation on — every datagram
    // is stamped at user-space receive, so a drained backlog stamps a burst of
    // them within the same microsecond. Here the whole burst shares one stamp,
    // which is the limiting case: the receive times say no time passed and the
    // spacing has to come from the index instead.
    let sc = StreamClock::default();
    let cycle = 128_850_307u64;
    let step = 4_000u64;
    let sys = 1_787_083_917_000_000u64;
    let start = cycle - 3 * step;

    let mut absolute = Vec::new();
    for i in 0..6u64 {
        let clock = ((start + i * step) % cycle) as u32;
        absolute.push(sc.accept(i as u32, clock, sys).expect("accepted"));
    }
    for (i, w) in absolute.windows(2).enumerate() {
        assert_eq!(
            w[1] - w[0],
            step,
            "packet {i} -> {} lost its spacing across the wrap",
            i + 1
        );
    }
}

#[test]
fn test_stream_clock_recovers_from_a_restarted_stream() {
    // The controller restarts its stream and counts from zero again. A plain
    // high-water mark would reject every packet from here on and the stream
    // would never deliver again.
    let sc = StreamClock::default();
    let step = 8_000u64;
    let sys = 1_787_083_917_000_000u64;
    let epoch = |micros: u64| SystemTime::UNIX_EPOCH + Duration::from_micros(micros);

    for i in 0..4u64 {
        let at = sc.accept(
            40_000 + i as u32,
            (500_000 + i * step) as u32,
            sys + i * step,
        );
        assert!(at.is_some(), "pre-restart packet {i}");
    }
    let last_sys = sys + 3 * step;
    let resumed_at = last_sys + 250_000;

    // The first packets of the new stream still look stale and are dropped.
    for i in 0..(StreamClock::STALE_RUN_LIMIT - 1) {
        let dropped = sc.accept(i, 1_000 + i * 10, resumed_at + u64::from(i) * step);
        assert_eq!(
            dropped, None,
            "packet {i} of the restart is not yet a restart"
        );
    }

    // Past the limit it is read as a restart, and the stream delivers again.
    let i = StreamClock::STALE_RUN_LIMIT - 1;
    let at = resumed_at + u64::from(i) * step;
    let first = sc.accept(i, 1_000 + i * 10, at).expect("stream recovers");
    assert_eq!(
        sc.system_time_of(i, 1_000 + i * 10),
        Some(epoch(at)),
        "the resumed packet reports its own receive time"
    );

    // And it keeps running forward from there rather than jumping backward.
    let next = sc
        .accept(i + 1, 1_000 + (i + 1) * 10, at + step)
        .expect("accepted");
    assert!(
        next > first,
        "device time keeps advancing after the restart"
    );
    assert_eq!(
        sc.system_time_of(i + 1, 1_000 + (i + 1) * 10),
        Some(epoch(at + step))
    );
}

#[test]
fn test_stream_clock_fixed_index_resolves_buffered_packets() {
    // A controller that leaves `index` fixed records every base against the same
    // index, so buffered packets used to resolve against the newest base — putting
    // the pre-wrap ones a whole cycle into the future. Within one base the clock
    // only climbs, so it is what separates them.
    //
    // The cycle has to be the full range here: a fixed-index stream on a shorter
    // counter has neither wrap signal available, since the index never moves and
    // the boundary guard is calibrated to a range the counter never reaches.
    let sc = StreamClock::default();
    let cycle = u32::MAX as u64 + 1;
    let step = 4_000u64;
    let sys = 1_787_083_917_000_000u64;
    let epoch = |micros: u64| SystemTime::UNIX_EPOCH + Duration::from_micros(micros);
    let start = cycle - 2 * step;

    let mut sent = Vec::new();
    for i in 0..4u64 {
        let clock = ((start + i * step) % cycle) as u32;
        let at = sys + i * step;
        assert!(sc.accept(0, clock, at).is_some(), "packet {i} gated");
        sent.push((clock, at));
    }

    for (i, &(clock, at)) in sent.iter().enumerate() {
        assert_eq!(
            sc.system_time_of(0, clock),
            Some(epoch(at)),
            "packet {i} of a fixed-index stream"
        );
    }
}

/// The broker is process-global, so tests that start one take turns.
static BROKER: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Holds the broker for one test and destroys whatever it started, even when
/// the test fails, so the next one starts from nothing.
struct BrokerTurn(#[allow(dead_code)] std::sync::MutexGuard<'static, ()>);

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

const BROKER_ADDR: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1)), 60000);
const CONNECTION_TIMEOUT: Duration = Duration::from_millis(16);

/// How many (var, tcp, joint) triplets a robot sends before it stops, so the
/// expected packet count is exact.
const TARGET_PACKET_COUNT: usize = 21;

/// A robot at `addr` that sends `TARGET_PACKET_COUNT` triplets of (var, tcp,
/// joint) packets to the broker, one triplet every 2 ms, then stops.
fn robot(addr: SocketAddr) -> snare::Tester<Bytes, usize> {
    udp_tester::<Bytes>(addr)
        .with_state(0usize)
        .with_stateful_cyclic_action(Duration::from_millis(2), |sent| {
            let clock = *sent as u32;
            *sent += 1;
            TesterAction::Multiple(vec![
                TesterAction::SendTo(
                    BROKER_ADDR,
                    Bytes(encode_packet(&make_variables_packet(clock))),
                ),
                TesterAction::SendTo(
                    BROKER_ADDR,
                    Bytes(encode_packet(&make_tcp_position_packet(clock))),
                ),
                TesterAction::SendTo(
                    BROKER_ADDR,
                    Bytes(encode_packet(&make_joint_angles_packet(clock))),
                ),
            ])
        })
        .until_state(|sent| *sent >= TARGET_PACKET_COUNT)
}

/// Lets the broker thread take in everything already sent: virtual time
/// cannot pass this sleep while the broker still has datagrams to read.
fn settle() {
    std::thread::sleep(Duration::from_millis(1));
}

/// Deterministic: under snare 2.0.0-alpha.1 a plain sim can wake a joining
/// thread late, at the broker's next poll timeout, which would push the
/// connection checks past the timeout they measure.
#[test]
fn test_all() {
    let _turn = BrokerTurn::take();
    Sim::builder()
        .deterministic()
        .strict_sockopts()
        .stuck_after(Duration::from_secs(30))
        .build()
        .run(|| {
            assert!(
                HspoReceiver::try_new([0, 0, 0, 1], 128, CONNECTION_TIMEOUT).is_err(),
                "Receiver initialized before the broker was started."
            );

            initialize_broker(BROKER_ADDR, &[], &[]).expect("Failed to initialize broker.");

            assert!(
                HspoReceiver::try_new([0, 0, 0, 2], 128, CONNECTION_TIMEOUT).is_ok(),
                "Failed to initialize receiver after broker was started."
            );

            test_connection();
            test_drain();
            test_telemetry();
            test_connection_times_out_on_the_virtual_clock();

            destroy_broker(true);
        });
}

/// The broker's liveness sweep runs on the sim's clock: a robot that stops
/// sending stays connected within its timeout and is dropped once that much
/// virtual time has passed, with no real time spent waiting.
fn test_connection_times_out_on_the_virtual_clock() {
    let addr = SocketAddr::from(([10, 0, 0, 5], 60000));
    let receiver = HspoReceiver::try_new(addr.ip(), 128, CONNECTION_TIMEOUT)
        .expect("Failed to initialize receiver.");

    let tester = robot(addr);
    run_testers!(tester);
    settle();
    assert!(
        receiver.is_connected(),
        "Receiver did not receive any packets."
    );

    std::thread::sleep(CONNECTION_TIMEOUT / 2);
    assert!(
        receiver.is_connected(),
        "Connection expired inside its timeout."
    );

    std::thread::sleep(CONNECTION_TIMEOUT * 3);
    assert!(
        !receiver.is_connected(),
        "Connection survived three timeouts without a packet."
    );
}

fn test_telemetry() {
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct CountingSink(Arc<AtomicUsize>);
    impl crate::TelemetrySink<(), HspoRxPacket> for CountingSink {
        fn sent(&self, _tx: &(), _timestamp: SystemTime) {}
        fn received(&self, _rx: &HspoRxPacket, _timestamp: SystemTime) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }

    let addr = SocketAddr::from(([10, 0, 0, 4], 60000));
    let received = Arc::new(AtomicUsize::new(0));
    let receiver = HspoReceiver::try_new_with_telemetry(
        addr.ip(),
        128,
        CONNECTION_TIMEOUT,
        CountingSink(received.clone()),
    )
    .expect("Failed to initialize receiver.");

    let tester = robot(addr);
    run_testers!(tester);
    settle();

    assert!(
        receiver.is_connected(),
        "Receiver did not receive any packets."
    );
    assert_eq!(
        received.load(Ordering::Relaxed),
        3 * TARGET_PACKET_COUNT,
        "received hook did not fire once per packet"
    );
}

fn test_connection() {
    let addr = SocketAddr::from(([10, 0, 0, 2], 60000));
    let receiver = HspoReceiver::try_new(addr.ip(), 128, CONNECTION_TIMEOUT)
        .expect("Failed to initialize receiver.");

    let tester = robot(addr);
    run_testers!(tester);
    settle();

    assert!(
        receiver.is_connected(),
        "Receiver did not receive any packets."
    );
}

fn test_drain() {
    let addr = SocketAddr::from(([10, 0, 0, 3], 60000));
    let receiver = HspoReceiver::try_new(addr.ip(), 128, CONNECTION_TIMEOUT)
        .expect("Failed to initialize receiver.");

    let tester = robot(addr);
    run_testers!(tester);
    settle();

    assert_eq!(
        receiver.joint.recv_all().len(),
        TARGET_PACKET_COUNT,
        "Receiver did not receive expected joint packet count."
    );
    assert!(
        receiver.joint.recv_all().is_empty(),
        "Receiver did not drain joint packets."
    );
    assert_eq!(
        receiver.tcp.recv_all().len(),
        TARGET_PACKET_COUNT,
        "Receiver did not receive expected TCP packet count."
    );
    assert!(
        receiver.tcp.recv_all().is_empty(),
        "Receiver did not drain TCP packets."
    );
    assert_eq!(
        receiver.var.recv_all().len(),
        TARGET_PACKET_COUNT,
        "Receiver did not receive expected variables packet count."
    );
    assert!(
        receiver.var.recv_all().is_empty(),
        "Receiver did not drain variables packets."
    );
}

fn hspo_sim() -> Sim {
    Sim::builder()
        .deterministic()
        .strict_sockopts()
        .stuck_after(Duration::from_secs(30))
        .build()
}

fn joint_datagram(index: u32, clock: u32) -> Vec<u8> {
    let mut packet = make_joint_angles_packet(clock);
    packet.index = index;
    encode_packet(&packet)
}

/// Kernel receive stamps of the joint packets one receiver's broker decoded,
/// by packet index.
#[derive(Clone, Default)]
struct JointStamps(Arc<parking_lot::Mutex<Vec<(u32, SystemTime)>>>);

impl crate::TelemetrySink<(), HspoRxPacket> for JointStamps {
    fn sent(&self, _tx: &(), _timestamp: SystemTime) {}
    fn received(&self, rx: &HspoRxPacket, timestamp: SystemTime) {
        if let HspoRxPacket::JointAngles(p) = rx {
            self.0.lock().push((p.index, timestamp));
        }
    }
}

impl JointStamps {
    fn of(&self, index: u32) -> SystemTime {
        self.0
            .lock()
            .iter()
            .find(|(i, _)| *i == index)
            .map(|(_, t)| *t)
            .unwrap_or_else(|| panic!("packet {index} was never stamped"))
    }
}

/// `a` and `b` within `slack` of each other, either way.
fn close(a: SystemTime, b: SystemTime, slack: Duration) -> bool {
    a.duration_since(b)
        .or_else(|e| Ok::<_, ()>(e.duration()))
        .unwrap()
        <= slack
}

/// The R-30iB's controller clock cycle, in µs.
const R30IB_CYCLE: u64 = 128_850_307;

/// Sends `count` joint packets from `socket` to the broker every `period`,
/// stamping each with a controller clock that runs at 1 µs per µs of virtual
/// time from `clock0`, wrapping at `cycle`. Returns each packet's send time.
fn emit_joints(
    socket: &std::net::UdpSocket,
    count: u32,
    period: Duration,
    clock0: u64,
    cycle: u64,
) -> Vec<SystemTime> {
    let t0 = SystemTime::now();
    (0..count)
        .map(|index| {
            let at = SystemTime::now();
            let elapsed = at.duration_since(t0).unwrap().as_micros() as u64;
            let clock = ((clock0 + elapsed) % cycle) as u32;
            socket
                .send_to(&joint_datagram(index, clock), BROKER_ADDR)
                .unwrap();
            std::thread::sleep(period);
            at
        })
        .collect()
}

#[test]
fn rx_timestamps_are_the_virtual_arrival_time() {
    let _turn = BrokerTurn::take();
    hspo_sim().run(|| {
        initialize_broker(BROKER_ADDR, &[], &[]).unwrap();
        let robot = SocketAddr::from(([10, 0, 0, 20], 60000));
        let stamps = JointStamps::default();
        let receiver = HspoReceiver::try_new_with_telemetry(
            robot.ip(),
            256,
            Duration::from_millis(100),
            stamps.clone(),
        )
        .unwrap();
        let latency = Duration::from_millis(3);
        snare::set_udp_policy(BROKER_ADDR, |p| p.latency = latency);
        let socket = std::net::UdpSocket::bind(robot).unwrap();
        let sent = emit_joints(&socket, 50, Duration::from_millis(2), 1_000, R30IB_CYCLE);
        std::thread::sleep(Duration::from_millis(5));

        let packets = receiver.joint.recv_all();
        assert_eq!(packets.len(), 50);
        for p in &packets {
            let expected = sent[p.index as usize] + latency;
            let stamp = stamps.of(p.index);
            assert!(
                close(stamp, expected, Duration::from_micros(3)),
                "packet {} sent at {:?} stamped {:?}, {latency:?} of latency",
                p.index,
                sent[p.index as usize],
                stamp
            );
            let at = receiver.joint.received_at(p).unwrap();
            assert!(
                close(at, stamp, Duration::from_micros(3)),
                "packet {} received_at {at:?}, stamped {stamp:?}",
                p.index
            );
        }
        destroy_broker(true);
    });
}

#[test]
fn received_at_survives_a_controller_clock_wrap_on_the_wire() {
    let _turn = BrokerTurn::take();
    hspo_sim().run(|| {
        initialize_broker(BROKER_ADDR, &[], &[]).unwrap();
        let robot = SocketAddr::from(([10, 0, 0, 21], 60000));
        let stamps = JointStamps::default();
        let receiver = HspoReceiver::try_new_with_telemetry(
            robot.ip(),
            256,
            Duration::from_millis(100),
            stamps.clone(),
        )
        .unwrap();
        let socket = std::net::UdpSocket::bind(robot).unwrap();
        // The clock wraps 100 ms in; nothing is read until 240 ms of packets
        // from both sides of it are buffered.
        emit_joints(
            &socket,
            60,
            Duration::from_millis(4),
            R30IB_CYCLE - 100_000,
            R30IB_CYCLE,
        );
        std::thread::sleep(Duration::from_millis(1));

        let packets = receiver.joint.recv_all();
        assert_eq!(packets.len(), 60);
        let wraps = packets
            .windows(2)
            .filter(|w| w[1].clock < w[0].clock)
            .count();
        assert_eq!(wraps, 1, "the stream did not wrap exactly once");
        for p in &packets {
            let at = receiver.joint.received_at(p).unwrap();
            let stamp = stamps.of(p.index);
            assert!(
                close(at, stamp, Duration::from_micros(3)),
                "packet {} (clock {}) resolved to {at:?}, received at {stamp:?}",
                p.index,
                p.clock
            );
        }
        destroy_broker(true);
    });
}

#[test]
fn a_robot_that_stops_sending_drops_out_alone() {
    let _turn = BrokerTurn::take();
    hspo_sim().run(|| {
        initialize_broker(BROKER_ADDR, &[], &[]).unwrap();
        let (ip_a, ip_b) = ([10, 0, 0, 22], [10, 0, 0, 23]);
        let a = HspoReceiver::try_new(ip_a, 256, CONNECTION_TIMEOUT).unwrap();
        let b = HspoReceiver::try_new(ip_b, 256, CONNECTION_TIMEOUT).unwrap();
        let sock_a = std::net::UdpSocket::bind(SocketAddr::from((ip_a, 60000))).unwrap();
        let sock_b = std::net::UdpSocket::bind(SocketAddr::from((ip_b, 60000))).unwrap();

        // Both robots every 2 ms; b goes quiet after packet 19 and comes back
        // at 40.
        let mut seen = Vec::new();
        for i in 0..60u32 {
            sock_a
                .send_to(&joint_datagram(i, i * 2000), BROKER_ADDR)
                .unwrap();
            if !(20..40).contains(&i) {
                sock_b
                    .send_to(&joint_datagram(i, i * 2000), BROKER_ADDR)
                    .unwrap();
            }
            std::thread::sleep(Duration::from_millis(2));
            seen.push((i, a.is_connected(), b.is_connected()));
        }

        for &(i, a_up, b_up) in &seen {
            assert!(a_up, "robot a dropped out at {i}");
            // b's last packet before the gap went out at 19; the timeout is 16 ms.
            let quiet_for = i.saturating_sub(19) * 2;
            match i {
                0..=19 => assert!(b_up, "robot b down at {i} while sending"),
                20..=39 if quiet_for < 16 => {
                    assert!(b_up, "robot b down {quiet_for} ms into its timeout")
                }
                20..=39 if quiet_for > 18 => assert!(
                    !b_up,
                    "robot b still up {quiet_for} ms after its last packet"
                ),
                40.. => assert!(b_up, "robot b did not come back at {i}"),
                _ => {}
            }
        }
        assert_eq!(a.joint.recv_all().len(), 60);
        assert_eq!(b.joint.recv_all().len(), 40);
        destroy_broker(true);
    });
}

#[test]
fn malformed_and_foreign_datagrams_are_ignored() {
    let _turn = BrokerTurn::take();
    hspo_sim().run(|| {
        initialize_broker(BROKER_ADDR, &[], &[]).unwrap();
        let robot = SocketAddr::from(([10, 0, 0, 24], 60000));
        let stamps = JointStamps::default();
        let receiver = HspoReceiver::try_new_with_telemetry(
            robot.ip(),
            16,
            Duration::from_millis(100),
            stamps.clone(),
        )
        .unwrap();
        let socket = std::net::UdpSocket::bind(robot).unwrap();
        let stranger =
            std::net::UdpSocket::bind(SocketAddr::from(([10, 0, 0, 99], 60000))).unwrap();

        let valid = joint_datagram(3, 3000);
        let mut unknown_type = valid.clone();
        unknown_type[12..14].copy_from_slice(&99u16.to_be_bytes());
        let mut tcp_typed_joint = valid.clone();
        tcp_typed_joint[12..14].copy_from_slice(&1u16.to_be_bytes());
        tcp_typed_joint.truncate(40);
        for junk in [
            &[][..],
            &valid[..20],
            &valid[..valid.len() - 1],
            &unknown_type[..],
            &tcp_typed_joint[..],
            &[0xff; 7][..],
        ] {
            socket.send_to(junk, BROKER_ADDR).unwrap();
        }
        stranger
            .send_to(&joint_datagram(1, 1000), BROKER_ADDR)
            .unwrap();
        std::thread::sleep(Duration::from_millis(1));
        assert!(receiver.joint.recv_all().is_empty());
        assert!(receiver.tcp.recv_all().is_empty());
        assert!(receiver.var.recv_all().is_empty());
        assert!(
            stamps.0.lock().is_empty(),
            "a malformed datagram reached telemetry"
        );

        socket.send_to(&valid, BROKER_ADDR).unwrap();
        std::thread::sleep(Duration::from_millis(1));
        let got = receiver.joint.recv_all();
        assert_eq!(got.len(), 1, "the broker stopped after junk");
        assert_eq!(got[0].index, 3);
        assert!(!has_broker_errored());
        destroy_broker(true);
    });
}

#[test]
fn reordered_packets_never_reach_a_channel_out_of_order() {
    let _turn = BrokerTurn::take();
    hspo_sim().run(|| {
        initialize_broker(BROKER_ADDR, &[], &[]).unwrap();
        let robot = SocketAddr::from(([10, 0, 0, 25], 60000));
        let receiver = HspoReceiver::try_new(robot.ip(), 256, Duration::from_millis(100)).unwrap();
        snare::set_udp_policy(BROKER_ADDR, |p| p.jitter = Duration::from_millis(6));
        let socket = std::net::UdpSocket::bind(robot).unwrap();
        emit_joints(&socket, 100, Duration::from_millis(2), 0, R30IB_CYCLE);
        std::thread::sleep(Duration::from_millis(10));

        let indices: Vec<u32> = receiver.joint.recv_all().iter().map(|p| p.index).collect();
        assert!(
            indices.windows(2).all(|w| w[0] < w[1]),
            "a channel delivered out of order: {indices:?}"
        );
        assert!(
            indices.len() < 100,
            "nothing was reordered, so nothing was tested"
        );
        assert!(
            indices.len() > 50,
            "only {} of 100 packets survived",
            indices.len()
        );
        assert_eq!(indices.last(), Some(&99));
        destroy_broker(true);
    });
}

#[test]
fn a_full_buffer_keeps_the_newest_packets() {
    let _turn = BrokerTurn::take();
    hspo_sim().run(|| {
        initialize_broker(BROKER_ADDR, &[], &[]).unwrap();
        let robot = SocketAddr::from(([10, 0, 0, 26], 60000));
        let receiver = HspoReceiver::try_new(robot.ip(), 4, Duration::from_millis(100)).unwrap();
        let socket = std::net::UdpSocket::bind(robot).unwrap();
        emit_joints(&socket, 10, Duration::from_millis(1), 0, R30IB_CYCLE);
        let indices: Vec<u32> = receiver.joint.recv_all().iter().map(|p| p.index).collect();
        assert_eq!(indices, vec![6, 7, 8, 9]);
        destroy_broker(true);
    });
}

#[test]
fn destroying_the_broker_frees_its_port_and_ends_its_receivers() {
    let _turn = BrokerTurn::take();
    hspo_sim().run(|| {
        let robot = SocketAddr::from(([10, 0, 0, 27], 60000));
        initialize_broker(BROKER_ADDR, &[], &[]).unwrap();
        let old = HspoReceiver::try_new(robot.ip(), 16, Duration::from_millis(100)).unwrap();
        destroy_broker(true);
        assert!(
            snare::sockets_bound(BROKER_ADDR).is_empty(),
            "the destroyed broker's socket is still open"
        );
        let t0 = Instant::now();
        assert!(old.joint.wait_for(Duration::from_secs(1)).is_none());
        assert!(
            t0.elapsed() < Duration::from_millis(1),
            "a receiver of a destroyed broker waited {:?} instead of ending",
            t0.elapsed()
        );

        initialize_broker(BROKER_ADDR, &[], &[]).expect("the port was not freed");
        let new = HspoReceiver::try_new(robot.ip(), 16, Duration::from_millis(100)).unwrap();
        let socket = std::net::UdpSocket::bind(robot).unwrap();
        emit_joints(&socket, 3, Duration::from_millis(1), 0, R30IB_CYCLE);
        assert_eq!(new.joint.recv_all().len(), 3);
        assert!(old.joint.recv_all().is_empty());
        destroy_broker(true);
    });
}
