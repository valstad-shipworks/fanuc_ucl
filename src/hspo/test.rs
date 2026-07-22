use snare::{TesterAction, TimerState, connect_tester, run_testers};

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
    let span = u32::MAX as u64 + 1;

    assert_eq!(sc.accept(0, u32::MAX - 5, 0), Some((u32::MAX - 5) as u64));
    // Strictly-newer packet whose clock stepped backward across the boundary: a wrap.
    assert_eq!(sc.accept(1, 4, 8), Some(span + 4));
    assert_eq!(sc.accept(2, 12, 16), Some(span + 12));
}

#[test]
fn test_stream_clock_wrap_fixed_index() {
    let sc = StreamClock::default();
    let span = u32::MAX as u64 + 1;

    // Index never advances, so the boundary check alone must catch the wrap.
    assert_eq!(sc.accept(0, u32::MAX - 5, 0), Some((u32::MAX - 5) as u64));
    assert_eq!(sc.accept(0, 4, 8), Some(span + 4));
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

const BROKER_ADDR: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1)), 60000);

fn hspo_packet_sender(timer: &mut TimerState) -> Option<TesterAction<RawPacket>> {
    let time = timer.poll_elapsed().as_millis() as u32;
    Some(TesterAction::Multiple(vec![
        TesterAction::Send(
            BROKER_ADDR,
            RawPacket(encode_packet(&make_variables_packet(time))),
        ),
        TesterAction::Send(
            BROKER_ADDR,
            RawPacket(encode_packet(&make_tcp_position_packet(time))),
        ),
        TesterAction::Send(
            BROKER_ADDR,
            RawPacket(encode_packet(&make_joint_angles_packet(time))),
        ),
    ]))
}

/// Stateful sender that emits at most `TARGET_PACKET_COUNT` triplets of
/// (var, tcp, joint) packets and then stops. Used by `test_drain` so the
/// expected packet count is deterministic regardless of CI timing.
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

    destroy_broker(false);
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
        .with_stateful_cyclic_action::<TimerState>(Duration::from_millis(2), hspo_packet_sender)
        .until_stateful_condition::<TimerState>(|state| {
            state.poll_elapsed() >= Duration::from_millis(40)
        });

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
        .with_stateful_cyclic_action::<TimerState>(Duration::from_millis(2), hspo_packet_sender)
        .until_stateful_condition::<TimerState>(|state| {
            state.poll_elapsed() >= Duration::from_millis(40)
        });

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

    thread::sleep(Duration::from_millis(50));

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
