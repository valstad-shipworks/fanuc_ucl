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
fn test_clock_spec() {
    let clock = ClockSpec::default();
    assert_eq!(clock.read().0, 0);

    clock.write(1000, 100);
    assert_eq!(clock.read(), (1000, 100));

    clock.write(500, 200);
    assert_eq!(clock.read(), (1500, 200));
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

    destroy_broker(false);
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

    let mut tester = connect_tester::<RawPacket>(addr)
        .with_stateful_cyclic_action::<TimerState>(Duration::from_millis(2), hspo_packet_sender)
        .until_stateful_condition::<TimerState>(|state| {
            state.poll_elapsed() >= Duration::from_millis(40)
        });

    run_testers!(tester);

    thread::sleep(Duration::from_millis(50));

    assert_eq!(
        receiver.recv_all_joint_packets().len(),
        21,
        "Receiver did not receive any joint packets."
    );
    assert!(
        receiver.recv_all_joint_packets().is_empty(),
        "Receiver did not drain joint packets."
    );
    assert_eq!(
        receiver.recv_all_tcp_packets().len(),
        21,
        "Receiver did not receive any TCP packets."
    );
    assert!(
        receiver.recv_all_tcp_packets().is_empty(),
        "Receiver did not drain TCP packets."
    );
    assert_eq!(
        receiver.recv_all_var_packets().len(),
        21,
        "Receiver did not receive any variables packets."
    );
    assert!(
        receiver.recv_all_var_packets().is_empty(),
        "Receiver did not drain variables packets."
    );
}
