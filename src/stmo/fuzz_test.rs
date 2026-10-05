//! Property tests for the Stream Motion datagram codecs.
//!
//! Every property runs on a fixed seed, so a failure reproduces on every run;
//! proptest prints the minimal failing input.

use proptest::collection::vec;
use proptest::prelude::*;
use proptest::test_runner::RngSeed;

use crate::joints::{JointFormat, JointTemplate};
use crate::stmo::proto::{
    IoType, MotionCommandPacket, PoseData, RobotStatusPacket, RxPackets,
    ThresholdTableResponsePacket, TxPackets, VersionNumberResponsePacket,
};

fn config(cases: u32) -> ProptestConfig {
    ProptestConfig {
        cases,
        rng_seed: RngSeed::Fixed(0x5354_4d4f),
        failure_persistence: None,
        ..ProptestConfig::default()
    }
}

const ROBOT_STATUS: u32 = 0;
const THRESHOLD_TABLE: u32 = 3;
const COMMAND_POSITION: u32 = 4;
const VERSION_NUMBER: u32 = 6;

fn encode_rx(rx: &RxPackets, version: u32) -> Vec<u8> {
    let mut buf = [0u8; 512];
    let n = rx.encode_into(version, &mut buf).unwrap();
    buf[..n].to_vec()
}

fn encode_tx(tx: TxPackets, version: u32) -> Vec<u8> {
    let mut buf = [0u8; 512];
    let n = tx.encode_into(version, &mut buf).unwrap();
    buf[..n].to_vec()
}

fn header_version(bytes: &[u8]) -> u32 {
    u32::from_be_bytes(bytes[4..8].try_into().unwrap())
}

fn type_of(rx: &RxPackets) -> u32 {
    match rx {
        RxPackets::RobotStatus(_) => ROBOT_STATUS,
        RxPackets::ThresholdTableResponse(_) => THRESHOLD_TABLE,
        RxPackets::CommandPositionResponse(_) => COMMAND_POSITION,
        RxPackets::VersionNumberResponse(_) => VERSION_NUMBER,
    }
}

fn pose() -> impl Strategy<Value = PoseData> {
    any::<[f32; 9]>()
        .prop_map(|v| PoseData::new(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]))
}

fn robot_status() -> impl Strategy<Value = RobotStatusPacket> {
    (
        any::<(u32, u8, u32)>(),
        any::<(u8, u16, u16, u16)>(),
        pose(),
        any::<[f32; 9]>(),
        any::<[f32; 9]>(),
    )
        .prop_map(|((seq, status, stamp), io, pose, joints, current)| {
            let mut p = RobotStatusPacket::new(seq, status, stamp, joints);
            p.set_read_io_reply(io.0, io.1, io.2, io.3);
            p.pose = pose;
            p.motor_current = current;
            p
        })
}

fn threshold_table() -> impl Strategy<Value = ThresholdTableResponsePacket> {
    (any::<[u32; 4]>(), any::<[f32; 20]>(), any::<[f32; 20]>()).prop_map(|(h, no, max)| {
        ThresholdTableResponsePacket {
            axis_number: h[0],
            limit_type: h[1],
            vmax: h[2],
            inter_check_time: h[3],
            no_payload: no,
            max_payload: max,
        }
    })
}

/// A command position reply has no public constructor; its 76-byte body is
/// any bit pattern.
fn command_position_bytes(version: u32) -> impl Strategy<Value = Vec<u8>> {
    vec(any::<u8>(), 76).prop_map(move |body| {
        let mut bytes = COMMAND_POSITION.to_be_bytes().to_vec();
        bytes.extend_from_slice(&version.to_be_bytes());
        bytes.extend_from_slice(&body);
        bytes
    })
}

fn rx_datagram() -> impl Strategy<Value = Vec<u8>> {
    any::<u32>().prop_flat_map(|version| {
        prop_oneof![
            robot_status().prop_map(move |p| encode_rx(&RxPackets::RobotStatus(p), version)),
            threshold_table()
                .prop_map(move |p| encode_rx(&RxPackets::ThresholdTableResponse(p), version)),
            command_position_bytes(version),
            any::<u32>().prop_map(|v| encode_rx(
                &RxPackets::VersionNumberResponse(VersionNumberResponsePacket { version: v }),
                0
            )),
        ]
    })
}

fn io_type() -> impl Strategy<Value = IoType> {
    prop::sample::select(vec![
        IoType::None,
        IoType::DI,
        IoType::DO,
        IoType::RI,
        IoType::RO,
        IoType::SI,
        IoType::SO,
        IoType::WI,
        IoType::WO,
        IoType::UI,
        IoType::UO,
        IoType::WSI,
        IoType::WSO,
        IoType::F,
        IoType::M,
    ])
}

fn motion_command() -> impl Strategy<Value = MotionCommandPacket> {
    (
        prop_oneof![
            any::<[f64; 6]>().prop_map(|j| MotionCommandPacket::try_from_joints(
                JointFormat::FanucDeg,
                JointTemplate::SIX,
                j
            )
            .unwrap()),
            pose().prop_map(|p| MotionCommandPacket::from_pose(p).unwrap()),
        ],
        any::<(u32, bool)>(),
        (io_type(), any::<(u16, u16)>()),
        (io_type(), any::<(u16, u16, u16)>()),
    )
        .prop_map(
            |(mut cmd, (seq, last), (rt, (ri, rm)), (wt, (wi, wm, wv)))| {
                cmd.seq = seq;
                cmd.set_last_command(last);
                cmd.set_read_io(rt, ri, rm);
                cmd.set_write_io(wt, wi, wm, wv);
                cmd
            },
        )
}

proptest! {
    #![proptest_config(config(1024))]

    /// Arbitrary datagrams never panic, and the bytes of whatever is accepted
    /// read back losslessly: re-encoding the packet reproduces the front of
    /// the datagram.
    #[test]
    fn arbitrary_rx_datagrams_decode_losslessly_or_not_at_all(bytes in vec(any::<u8>(), 0..300)) {
        if let Some(rx) = RxPackets::decode_from(&bytes) {
            let again = encode_rx(&rx, header_version(&bytes));
            prop_assert_eq!(&bytes[..again.len()], &again[..]);
        }
    }

    #[test]
    fn typed_rx_datagrams_decode_losslessly_or_not_at_all(
        typ in prop::sample::select(vec![ROBOT_STATUS, THRESHOLD_TABLE, COMMAND_POSITION, VERSION_NUMBER]),
        mut bytes in vec(any::<u8>(), 8..200),
    ) {
        bytes[..4].copy_from_slice(&typ.to_be_bytes());
        if let Some(rx) = RxPackets::decode_from(&bytes) {
            prop_assert_eq!(type_of(&rx), typ);
            let again = encode_rx(&rx, header_version(&bytes));
            prop_assert_eq!(&bytes[..again.len()], &again[..]);
        }
    }

    /// The controller-side decoder never panics on arbitrary input.
    #[test]
    fn arbitrary_tx_datagrams_never_panic(
        typ in 0u32..8,
        mut bytes in vec(any::<u8>(), 0..200),
    ) {
        if bytes.len() >= 4 {
            bytes[..4].copy_from_slice(&typ.to_be_bytes());
        }
        let _ = TxPackets::decode_from(&bytes);
    }
}

proptest! {
    #![proptest_config(config(256))]

    #[test]
    fn rx_packets_roundtrip(bytes in rx_datagram()) {
        let rx = RxPackets::decode_from(&bytes).expect("a valid datagram decodes");
        prop_assert_eq!(encode_rx(&rx, header_version(&bytes)), bytes);
    }

    #[test]
    fn a_truncated_rx_packet_is_rejected(bytes in rx_datagram(), cut in any::<prop::sample::Index>()) {
        let typ = u32::from_be_bytes(bytes[..4].try_into().unwrap());
        let shortest = if typ == VERSION_NUMBER { 8 } else { bytes.len() };
        let cut = cut.index(shortest);
        prop_assert!(RxPackets::decode_from(&bytes[..cut]).is_none());
    }

    /// The decoder sees how long each packet type should be. A datagram that
    /// is longer is not that packet, and must not be read as one.
    #[test]
    fn an_extended_rx_packet_is_rejected(bytes in rx_datagram(), tail in vec(any::<u8>(), 1..64)) {
        let mut long = bytes;
        long.extend_from_slice(&tail);
        prop_assert!(RxPackets::decode_from(&long).is_none());
    }

    /// A flipped bit in the body has no checksum to catch it, so it decodes
    /// faithfully: same packet type, every other field intact.
    #[test]
    fn a_flipped_body_bit_is_read_faithfully(bytes in rx_datagram(), bit in any::<prop::sample::Index>()) {
        let mut flipped = bytes.clone();
        let bit = 32 + bit.index((bytes.len() - 4) * 8);
        flipped[bit / 8] ^= 1 << (bit % 8);
        let rx = RxPackets::decode_from(&flipped).expect("a body flip still decodes");
        prop_assert_eq!(
            u32::from_be_bytes(flipped[..4].try_into().unwrap()),
            type_of(&rx)
        );
        prop_assert_eq!(encode_rx(&rx, header_version(&flipped)), flipped);
    }

    /// A flipped bit in the packet type field must not turn one packet into
    /// another. A status flipped 0 -> 4 or a command position flipped 4 -> 6
    /// is a different type of the wrong length.
    #[test]
    fn a_flipped_type_bit_never_changes_the_packet_type(bytes in rx_datagram(), bit in 0usize..32) {
        let typ = u32::from_be_bytes(bytes[..4].try_into().unwrap());
        let mut flipped = bytes;
        flipped[bit / 8] ^= 1 << (bit % 8);
        if let Some(rx) = RxPackets::decode_from(&flipped) {
            prop_assert_eq!(type_of(&rx), typ, "type flipped by bit {}", bit);
        }
    }

    /// Every field the driver sends reaches the controller side unchanged,
    /// positions narrowed to f32 exactly when the protocol version or the
    /// Cartesian format call for the single-precision packet.
    #[test]
    fn motion_commands_roundtrip(cmd in motion_command(), version in 0u32..4) {
        let single = version < 2 || !cmd.joint_format();
        let bytes = encode_tx(TxPackets::MotionCommand(cmd), version);
        let Some(TxPackets::MotionCommand(got)) = TxPackets::decode_from(&bytes) else {
            return Err(TestCaseError::fail("motion command did not decode"));
        };
        prop_assert_eq!(got.seq(), cmd.seq());
        prop_assert_eq!(got.is_last_command(), cmd.is_last_command());
        prop_assert_eq!(got.joint_format(), cmd.joint_format());
        prop_assert_eq!(got.read_io(), cmd.read_io());
        prop_assert_eq!(got.write_io(), cmd.write_io());
        let expected = cmd
            .commanded_position()
            .map(|v| if single { v as f32 as f64 } else { v });
        prop_assert_eq!(
            got.commanded_position().map(f64::to_bits),
            expected.map(f64::to_bits)
        );
    }

    #[test]
    fn a_truncated_motion_command_is_rejected(
        cmd in motion_command(),
        version in 0u32..4,
        cut in any::<prop::sample::Index>(),
    ) {
        let bytes = encode_tx(TxPackets::MotionCommand(cmd), version);
        let cut = 8 + cut.index(bytes.len() - 8);
        prop_assert!(TxPackets::decode_from(&bytes[..cut]).is_none());
    }
}
