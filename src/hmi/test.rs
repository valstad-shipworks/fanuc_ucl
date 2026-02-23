use snare::{TesterAction, TimerState, connect_tester, run_testers};

use super::*;
use proto::ports::*;
use proto::wire::*;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;

// --- Packetable impl for SNPX over TCP ---

#[derive(Debug, Clone)]
struct SnpxPacket(Vec<u8>);

impl snare::Packetable for SnpxPacket {
    const CAN_BE_FLATTENED: bool = false;
    const SOCKET_TYPE: snare::SocketType = snare::SocketType::Tcp;

    fn encode(&self) -> Vec<u8> {
        self.0.clone()
    }

    fn decode(data: &[u8]) -> Option<(Self, usize)> {
        if data.len() < 42 {
            return None;
        }
        let (header, _) = bincode::decode_from_slice::<Header, _>(&data[..42], BINCODE_CFG).ok()?;
        let total = 56 + header.payload_len() as usize;
        if data.len() < total {
            return None;
        }
        Some((SnpxPacket(data[..total].to_vec()), total))
    }
}

// --- Simulated robot state ---

struct RobotState {
    /// SegmentSelector::OutputBit — DI, RI, UI, SI, WI, WSI (bit-packed)
    output_bits: Vec<u8>,
    /// SegmentSelector::InputBit — DO, RO, UO, SO, WO, WSO (bit-packed)
    input_bits: Vec<u8>,
    /// SegmentSelector::AnalogOutput — GI, AI (2 bytes per value)
    analog_output: Vec<u8>,
    /// SegmentSelector::AnalogInput — GO, AO (2 bytes per value)
    analog_input: Vec<u8>,
    /// SegmentSelector::Registers — R[n] (2 bytes per register)
    registers: Vec<u8>,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            output_bits: vec![0u8; 2048],
            input_bits: vec![0u8; 2048],
            analog_output: vec![0u8; 4096],
            analog_input: vec![0u8; 4096],
            registers: vec![0u8; 4096],
        }
    }
}

impl RobotState {
    fn segment_bytes(&self, seg: SegmentSelector) -> &[u8] {
        match seg {
            SegmentSelector::OutputBit => &self.output_bits,
            SegmentSelector::InputBit => &self.input_bits,
            SegmentSelector::AnalogOutput => &self.analog_output,
            SegmentSelector::AnalogInput => &self.analog_input,
            SegmentSelector::Registers => &self.registers,
            _ => &[],
        }
    }

    fn segment_bytes_mut(&mut self, seg: SegmentSelector) -> Option<&mut Vec<u8>> {
        match seg {
            SegmentSelector::OutputBit => Some(&mut self.output_bits),
            SegmentSelector::InputBit => Some(&mut self.input_bits),
            SegmentSelector::AnalogOutput => Some(&mut self.analog_output),
            SegmentSelector::AnalogInput => Some(&mut self.analog_input),
            SegmentSelector::Registers => Some(&mut self.registers),
            _ => None,
        }
    }

    fn read_bytes(&self, seg: SegmentSelector, index: u16, byte_count: usize) -> Vec<u8> {
        let data = self.segment_bytes(seg);
        let start = index as usize;
        let end = start + byte_count;
        if end <= data.len() {
            data[start..end].to_vec()
        } else {
            vec![0u8; byte_count]
        }
    }

    fn write_bytes(&mut self, seg: SegmentSelector, index: u16, payload: &[u8]) {
        if let Some(data) = self.segment_bytes_mut(seg) {
            let start = index as usize;
            let end = start + payload.len();
            if end <= data.len() {
                data[start..end].copy_from_slice(payload);
            }
        }
    }
}

/// Convert protocol-level target_index to a byte offset into the segment.
fn target_to_byte_offset(seg: SegmentSelector, target_index: u16) -> usize {
    match seg {
        // Bit segments: target_index is a bit index
        SegmentSelector::OutputBit | SegmentSelector::InputBit => (target_index / 8) as usize,
        // Word segments: target_index is a word/register index (2 bytes each)
        SegmentSelector::AnalogInput
        | SegmentSelector::AnalogOutput
        | SegmentSelector::Registers => (target_index as usize) * 2,
        // Byte segments: target_index is already a byte offset
        _ => target_index as usize,
    }
}

/// Map segment type + target_size to actual byte count for read responses.
fn response_byte_count(seg: SegmentSelector, target_size: u16) -> usize {
    match seg {
        SegmentSelector::OutputBit | SegmentSelector::InputBit => {
            (target_size as usize).div_ceil(8)
        }
        SegmentSelector::AnalogInput
        | SegmentSelector::AnalogOutput
        | SegmentSelector::Registers => target_size as usize * 2,
        SegmentSelector::GlobalByte => target_size as usize,
        _ => target_size as usize,
    }
}

fn zero_plc_status() -> PlcStatus {
    let bytes = [0u8; 6];
    bincode::decode_from_slice::<PlcStatus, _>(&bytes, BINCODE_CFG)
        .unwrap()
        .0
}

fn encode_msg(msg: Message) -> SnpxPacket {
    SnpxPacket(bincode::encode_to_vec(msg, BINCODE_CFG).unwrap())
}

fn ack_resp(seq: u8) -> SnpxPacket {
    encode_msg(Message::new_test_resp(seq, [0u8; 6], zero_plc_status()))
}

// --- SNPX request handler ---

fn handle_snpx_request(
    state: &mut RobotState,
    packet: SnpxPacket,
    src: SocketAddr,
) -> TesterAction<SnpxPacket> {
    let (msg, _) = bincode::decode_from_slice::<Message, _>(&packet.0, BINCODE_CFG).unwrap();
    let seq = msg.seq();

    match msg.body {
        Body::Req {
            service_request,
            segment,
            target_index,
            target_size,
            payload,
            ..
        } => match (service_request, segment) {
            // INIT handshake
            (ServiceRequestCode::PLCStatus, SegmentSelector::Init) => {
                TesterAction::Send(src, encode_msg(Message::INIT_ACK))
            }
            // MAGIC handshake
            (ServiceRequestCode::Magic, SegmentSelector::Magic) => {
                TesterAction::Send(src, encode_msg(Message::MAGIC))
            }
            // Read
            (ServiceRequestCode::ReadSysMemory, seg) => {
                let byte_offset = target_to_byte_offset(seg, target_index);
                let byte_count = response_byte_count(seg, target_size);
                let data = state.read_bytes(seg, byte_offset as u16, byte_count);
                if data.len() <= 6 {
                    let mut resp_payload = [0u8; 6];
                    resp_payload[..data.len()].copy_from_slice(&data);
                    TesterAction::Send(
                        src,
                        encode_msg(Message::new_test_resp(seq, resp_payload, zero_plc_status())),
                    )
                } else {
                    TesterAction::Send(
                        src,
                        encode_msg(Message::new_test_ext_resp(seq, service_request, seg, data)),
                    )
                }
            }
            // Write (inline payload)
            (ServiceRequestCode::WriteSysMemory, seg) => {
                let byte_offset = target_to_byte_offset(seg, target_index);
                let byte_count = response_byte_count(seg, target_size);
                state.write_bytes(seg, byte_offset as u16, &payload[..byte_count.min(6)]);
                TesterAction::Send(src, ack_resp(seq))
            }
            // Default ack
            _ => TesterAction::Send(src, ack_resp(seq)),
        },
        Body::ExtReq {
            service_request,
            segment,
            target_index,
            payload,
            ..
        } => {
            if service_request == ServiceRequestCode::WriteSysMemory {
                let byte_offset = target_to_byte_offset(segment, target_index);
                state.write_bytes(segment, byte_offset as u16, &payload);
            }
            TesterAction::Send(src, ack_resp(seq))
        }
        _ => TesterAction::Send(src, ack_resp(seq)),
    }
}

// --- Helper to run a test with a client thread and snare tester ---

fn noop_state_setup(_: &mut RobotState) {}

fn run_hmi_test<F>(addr: SocketAddr, setup_state: fn(&mut RobotState), client_fn: F)
where
    F: FnOnce(SocketAddr) + Send + 'static,
{
    snare::register_test();
    snare::add_ip_addr(addr.ip());

    let mut tester = connect_tester::<SnpxPacket>(addr)
        .with_state::<RobotState>(setup_state)
        .then_stateful_action::<RobotState>(handle_snpx_request)
        .until_stateful_condition::<TimerState>(|t| t.poll_elapsed() >= Duration::from_secs(5));

    let thread_id = std::thread::current().id();
    let client_handle = std::thread::spawn(move || {
        snare::register_thread_child_of(thread_id);
        client_fn(addr);
    });

    run_testers!(tester);
    client_handle.join().unwrap();
}

#[test]
fn test_connect_disconnect() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 20)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver
                .connect(Some(Duration::from_secs(10)), None)
                .expect("Failed to connect");
            assert!(driver.is_connected(), "Driver should be connected");
            driver.disconnect(true).expect("Failed to disconnect");
        },
    );
}

#[test]
fn test_write_read_digital_output() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 21)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Write DO[1] = true
            driver
                .write::<DigitalOutput>(1, true)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();

            // Read DO[1] back
            let val = driver
                .read::<DigitalOutput>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert!(val, "DO[1] should be true after write");

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_write_read_register() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 22)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Write R[1] = 42
            driver
                .write::<Register>(1, 42i16)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();

            // Read R[1]
            let val = driver
                .read::<Register>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert_eq!(val, 42i16, "R[1] should be 42 after write");

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_write_read_register_array() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 23)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Write R[1..5] = [10, 20, 30, 40, 50]
            driver
                .write_array::<Register>(1, &[10i16, 20, 30, 40, 50])
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();

            // Read R[1..5]
            let vals = driver
                .read_array::<Register>(1, 5)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert_eq!(&*vals, &[10i16, 20, 30, 40, 50], "R[1..5] mismatch");

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_read_digital_input() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 24)), 60008),
        |state: &mut RobotState| {
            // DI reads from OutputBit segment (zero-indexed).
            // Use byte-aligned indices (multiples of 8) to avoid alignment edge cases.
            // DI[0] = bit 0 of byte 0, DI[8] = bit 0 of byte 1.
            state.output_bits[0] = 0b00000001; // DI[0] = true
        },
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            let val = driver
                .read::<DigitalInput>(0)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert!(val, "DI[0] should be true (pre-populated)");

            let val = driver
                .read::<DigitalInput>(8)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert!(!val, "DI[8] should be false");

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_write_command() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 25)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Send CLRALM command
            driver
                .clear_alarms()
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_write_read_group_output() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 26)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Write GO[1] = 100
            driver
                .write::<GroupOutput>(1, 100i16)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();

            // Read GO[1] back
            let val = driver
                .read::<GroupOutput>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert_eq!(val, 100i16, "GO[1] should be 100 after write");

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_read_group_input() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 27)), 60008),
        |state: &mut RobotState| {
            // GI reads from AnalogOutput segment, offset=0, zero-indexed.
            // GI[0] = first 2 bytes of analog_output (i16 LE).
            state.analog_output[0..2].copy_from_slice(&77i16.to_le_bytes());
        },
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            let val = driver
                .read::<GroupInput>(0)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert_eq!(val, 77i16, "GI[0] should be 77 (pre-populated)");

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_write_read_robot_output() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 28)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Write RO[1] = true (offset 5000, one-indexed)
            driver
                .write::<RobotOutput>(1, true)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();

            // Read RO[1] back
            let val = driver
                .read::<RobotOutput>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert!(val, "RO[1] should be true after write");

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_not_connected_error() {
    snare::register_test();
    let driver = HmiDriver::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 29)));
    assert!(
        !driver.is_connected(),
        "Driver should not be connected initially"
    );
    assert!(
        driver.read::<Register>(1).is_err(),
        "Read should fail when not connected"
    );
    assert!(
        driver.write::<Register>(1, 42i16).is_err(),
        "Write should fail when not connected"
    );
}

#[test]
fn test_multiple_sequential_register_writes() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 30)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Write several registers individually
            for i in 1..=5 {
                driver
                    .write::<Register>(i, (i as i16) * 100)
                    .unwrap()
                    .wait_timeout(Duration::from_secs(1))
                    .unwrap();
            }

            // Read each one back
            for i in 1..=5 {
                let val = driver
                    .read::<Register>(i)
                    .unwrap()
                    .wait_timeout(Duration::from_secs(1))
                    .unwrap();
                assert_eq!(val, (i as i16) * 100, "R[{}] mismatch", i);
            }

            driver.disconnect(true).ok();
        },
    );
}

#[test]
fn test_write_read_analog_output() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 31)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver.connect(Some(Duration::from_secs(2)), None).unwrap();

            // Write AO[1] = -500 (offset 1000, one-indexed)
            driver
                .write::<AnalogOutput>(1, -500i16)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();

            // Read AO[1] back
            let val = driver
                .read::<AnalogOutput>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert_eq!(val, -500i16, "AO[1] should be -500 after write");

            driver.disconnect(true).ok();
        },
    );
}
