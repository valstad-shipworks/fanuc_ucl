use snare::{TesterAction, connect_tester, run_testers};

use super::*;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};

#[derive(Debug, Clone)]
struct RawPacket(Vec<u8>);
impl snare::Packetable for RawPacket {
    const CAN_BE_FLATTENED: bool = false;
    const SOCKET_TYPE: snare::SocketType = snare::SocketType::Udp;

    fn encode(&self) -> Vec<u8> {
        self.0.clone()
    }

    fn decode(data: &[u8]) -> Option<(Self, usize)> {
        if data.is_empty() {
            None
        } else {
            Some((Self(data.to_vec()), data.len()))
        }
    }
}

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

// ---- Pure unit tests (no broker needed) ----

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

#[cfg(feature = "async")]
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

const BROKER_ADDR: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1)), 60000);

/// Stateful sender that emits at most `TARGET_PACKET_COUNT` triplets of
/// (var, tcp, joint) packets and then stops, so the expected packet count
/// is deterministic regardless of CI timing.
#[derive(Default)]
struct CountedSender {
    sent: usize,
}

const TARGET_PACKET_COUNT: usize = 21;

fn counted_packet_sender(state: &mut CountedSender) -> Option<TesterAction<RawPacket>> {
    if state.sent >= TARGET_PACKET_COUNT {
        return None;
    }
    let clock = state.sent as u32;
    state.sent += 1;
    Some(TesterAction::Multiple(vec![
        TesterAction::Send(
            BROKER_ADDR,
            RawPacket(encode_packet(&make_variables_packet(clock))),
        ),
        TesterAction::Send(
            BROKER_ADDR,
            RawPacket(encode_packet(&make_tcp_position_packet(clock))),
        ),
        TesterAction::Send(
            BROKER_ADDR,
            RawPacket(encode_packet(&make_joint_angles_packet(clock))),
        ),
    ]))
}

#[test]
fn test_all() {
    snare::register_test();

    snare::add_ip_addr(BROKER_ADDR.ip());

    if HspoReceiver::try_new([0, 0, 0, 1], 128, Duration::from_millis(16)).is_ok() {
        panic!("Failed to initialize receiver after broker was started.");
    }

    initialize_broker(BROKER_ADDR, None).expect("Failed to initialize broker.");

    if HspoReceiver::try_new([0, 0, 0, 2], 128, Duration::from_millis(16)).is_err() {
        panic!("Failed to initialize receiver after broker was started.");
    }

    test_connection();
    test_drain();
    test_telemetry();
    test_virtual_clock_connection_timeout();

    destroy_broker(false);
}

/// Proves the broker's liveness sweep runs on snare's virtual clock: a frozen
/// clock keeps the connection alive through real-time waits far past the
/// connection timeout, and advancing the clock expires it without any real
/// packet gap. Sleeps are `std::thread::sleep` on purpose — the broker's poll
/// cadence is real time, only its `snare::time` reads are virtual.
fn test_virtual_clock_connection_timeout() {
    let addr = SocketAddr::from(([10, 0, 0, 5], 60000));
    snare::add_ip_addr(addr.ip());

    let receiver = HspoReceiver::try_new(addr.ip(), 128, Duration::from_millis(16))
        .expect("Failed to initialize receiver.");

    snare::pause_time();

    let mut tester = connect_tester::<RawPacket>(addr)
        .with_stateful_cyclic_action::<CountedSender>(
            Duration::from_millis(2),
            counted_packet_sender,
        )
        .until_stateful_condition::<CountedSender>(|state| state.sent >= TARGET_PACKET_COUNT);

    run_testers!(tester);

    assert!(
        receiver.is_connected(),
        "Receiver did not receive any packets."
    );

    std::thread::sleep(Duration::from_millis(100));
    assert!(
        receiver.is_connected(),
        "Connection expired while the virtual clock was paused."
    );

    snare::advance_time(Duration::from_millis(50));
    std::thread::sleep(Duration::from_millis(100));
    assert!(
        !receiver.is_connected(),
        "Connection survived a virtual-clock jump past the connection timeout."
    );

    snare::resume_time();
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
    snare::add_ip_addr(addr.ip());

    let received = Arc::new(AtomicUsize::new(0));
    let receiver = HspoReceiver::try_new_with_telemetry(
        addr.ip(),
        128,
        Duration::from_millis(16),
        CountingSink(received.clone()),
    )
    .expect("Failed to initialize receiver.");

    let mut tester = connect_tester::<RawPacket>(addr)
        .with_stateful_cyclic_action::<CountedSender>(
            Duration::from_millis(2),
            counted_packet_sender,
        )
        .until_stateful_condition::<CountedSender>(|state| state.sent >= TARGET_PACKET_COUNT);

    run_testers!(tester);

    assert!(
        receiver.is_connected(),
        "Receiver did not receive any packets."
    );
    assert!(
        received.load(Ordering::Relaxed) > 0,
        "received hook never fired"
    );
}

fn test_connection() {
    let addr = SocketAddr::from(([10, 0, 0, 2], 60000));
    snare::add_ip_addr(addr.ip());

    let receiver = HspoReceiver::try_new(addr.ip(), 128, Duration::from_millis(16))
        .expect("Failed to initialize receiver.");

    let mut tester = connect_tester::<RawPacket>(addr)
        .with_stateful_cyclic_action::<CountedSender>(
            Duration::from_millis(2),
            counted_packet_sender,
        )
        .until_stateful_condition::<CountedSender>(|state| state.sent >= TARGET_PACKET_COUNT);

    run_testers!(tester);

    assert!(
        receiver.is_connected(),
        "Receiver did not receive any packets."
    );
}

fn test_drain() {
    let addr = SocketAddr::from(([10, 0, 0, 3], 60000));
    snare::add_ip_addr(addr.ip());

    let receiver = HspoReceiver::try_new(addr.ip(), 128, Duration::from_millis(16))
        .expect("Failed to initialize receiver.");

    // Send exactly TARGET_PACKET_COUNT triplets, terminated by counter not
    // wall-clock — the previous wall-clock-bounded version flaked on slow
    // CI runners that didn't fire all 21 cycles within the 40ms window.
    let mut tester = connect_tester::<RawPacket>(addr)
        .with_stateful_cyclic_action::<CountedSender>(
            Duration::from_millis(2),
            counted_packet_sender,
        )
        .until_stateful_condition::<CountedSender>(|state| state.sent >= TARGET_PACKET_COUNT);

    run_testers!(tester);

    assert_eq!(
        drain_expected(&receiver.joint, TARGET_PACKET_COUNT).len(),
        TARGET_PACKET_COUNT,
        "Receiver did not receive expected joint packet count."
    );
    assert!(
        receiver.joint.recv_all().is_empty(),
        "Receiver did not drain joint packets."
    );
    assert_eq!(
        drain_expected(&receiver.tcp, TARGET_PACKET_COUNT).len(),
        TARGET_PACKET_COUNT,
        "Receiver did not receive expected TCP packet count."
    );
    assert!(
        receiver.tcp.recv_all().is_empty(),
        "Receiver did not drain TCP packets."
    );
    assert_eq!(
        drain_expected(&receiver.var, TARGET_PACKET_COUNT).len(),
        TARGET_PACKET_COUNT,
        "Receiver did not receive expected variables packet count."
    );
    assert!(
        receiver.var.recv_all().is_empty(),
        "Receiver did not drain variables packets."
    );
}

/// Drains `channel` until `expected` packets have arrived or a real-time
/// watchdog expires, instead of a fixed settle-sleep after `run_testers!`.
/// Real clock and real sleep on purpose: the broker delivers on its own
/// real-time poll cadence.
fn drain_expected<T>(channel: &HspoChannel<T>, expected: usize) -> Vec<T> {
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    let mut out = channel.recv_all();
    while out.len() < expected && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
        out.append(&mut channel.recv_all());
    }
    out
}
