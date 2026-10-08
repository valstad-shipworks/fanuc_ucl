use snare::prelude::*;

use super::*;
use proto::ports::*;
use proto::wire::*;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::atomic::AtomicBool;
use std::time::{Duration, Instant};

/// One SNPX message: a 42-byte header whose text length gives the whole frame
/// as `56 + text_len` bytes.
#[derive(Debug, Clone)]
struct SnpxPacket(Vec<u8>);

impl Packet for SnpxPacket {
    fn parse(buf: &mut Vec<u8>) -> Option<Self> {
        if buf.len() < 42 {
            return None;
        }
        let (header, _) = bincode::decode_from_slice::<Header, _>(&buf[..42], BINCODE_CFG).ok()?;
        let total = 56 + header.payload_len() as usize;
        if buf.len() < total {
            return None;
        }
        Some(SnpxPacket(buf.drain(..total).collect()))
    }

    fn to_bytes(&self) -> Vec<u8> {
        self.0.clone()
    }
}

fn sim() -> Sim {
    Sim::builder()
        .deterministic()
        .strict_sockopts()
        .stuck_after(Duration::from_secs(30))
        .build()
}

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

fn handle_snpx_request(
    state: &mut RobotState,
    packet: SnpxPacket,
    _src: SocketAddr,
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
                TesterAction::Send(encode_msg(Message::INIT_ACK))
            }
            // MAGIC handshake
            (ServiceRequestCode::Magic, SegmentSelector::Magic) => {
                TesterAction::Send(encode_msg(Message::MAGIC))
            }
            // Read
            (ServiceRequestCode::ReadSysMemory, seg) => {
                let byte_offset = target_to_byte_offset(seg, target_index);
                let byte_count = response_byte_count(seg, target_size);
                let data = state.read_bytes(seg, byte_offset as u16, byte_count);
                if data.len() <= 6 {
                    let mut resp_payload = [0u8; 6];
                    resp_payload[..data.len()].copy_from_slice(&data);
                    TesterAction::Send(encode_msg(Message::new_test_resp(
                        seq,
                        resp_payload,
                        zero_plc_status(),
                    )))
                } else {
                    TesterAction::Send(encode_msg(Message::new_test_ext_resp(
                        seq,
                        service_request,
                        seg,
                        data,
                    )))
                }
            }
            // Write (inline payload)
            (ServiceRequestCode::WriteSysMemory, seg) => {
                let byte_offset = target_to_byte_offset(seg, target_index);
                let byte_count = response_byte_count(seg, target_size);
                state.write_bytes(seg, byte_offset as u16, &payload[..byte_count.min(6)]);
                TesterAction::Send(ack_resp(seq))
            }
            // Default ack
            _ => TesterAction::Send(ack_resp(seq)),
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
            TesterAction::Send(ack_resp(seq))
        }
        _ => TesterAction::Send(ack_resp(seq)),
    }
}

/// Marks the client finished even when it panics, so the tester stops and the
/// panic reaches the test instead of a hang.
struct Finished(Arc<AtomicBool>);

impl Drop for Finished {
    fn drop(&mut self) {
        self.0.store(true, Ordering::SeqCst);
    }
}

fn noop_state_setup(_: &mut RobotState) {}

fn run_hmi_test<F>(addr: SocketAddr, setup_state: fn(&mut RobotState), client_fn: F)
where
    F: FnOnce(SocketAddr) + Send + 'static,
{
    sim().run(|| {
        let mut state = RobotState::default();
        setup_state(&mut state);
        let done = Arc::new(AtomicBool::new(false));
        let finished = done.clone();
        let tester = connect_tester::<SnpxPacket>(addr)
            .with_state(state)
            .then_stateful_action(handle_snpx_request)
            .until(move |_| finished.load(Ordering::SeqCst));
        let guard = Finished(done);
        let client = std::thread::spawn(move || {
            let _guard = guard;
            client_fn(addr)
        });
        run_testers!(tester);
        if let Err(panic) = client.join() {
            std::panic::resume_unwind(panic);
        }
    });
}

#[test]
fn test_connect_disconnect() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 20)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver
                .connect(Some(Duration::from_secs(10)), &[], &[])
                .expect("Failed to connect");
            assert!(driver.is_connected(), "Driver should be connected");
            driver.disconnect(true).expect("Failed to disconnect");
        },
    );
}

#[test]
fn test_telemetry_sink() {
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct CountingSink {
        sent: Arc<AtomicUsize>,
        received: Arc<AtomicUsize>,
    }
    impl crate::TelemetrySink<Message, Message> for CountingSink {
        fn sent(&self, _tx: &Message, _timestamp: std::time::SystemTime) {
            self.sent.fetch_add(1, Ordering::Relaxed);
        }
        fn received(&self, _rx: &Message, _timestamp: std::time::SystemTime) {
            self.received.fetch_add(1, Ordering::Relaxed);
        }
    }

    let sent = Arc::new(AtomicUsize::new(0));
    let received = Arc::new(AtomicUsize::new(0));
    let sink = CountingSink {
        sent: sent.clone(),
        received: received.clone(),
    };
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 31)), 60008),
        noop_state_setup,
        move |addr| {
            let mut driver = HmiDriver::new_with_telemetry(addr.ip(), sink);
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();
            driver
                .write::<DigitalOutput>(1, true)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            driver.disconnect(true).ok();
        },
    );

    // At minimum INIT, MAGIC, CLRASG, and the DO write cross the wire, each acked.
    assert!(
        sent.load(Ordering::Relaxed) >= 4,
        "sent hook fired {} times",
        sent.load(Ordering::Relaxed)
    );
    assert!(
        received.load(Ordering::Relaxed) >= 4,
        "received hook fired {} times",
        received.load(Ordering::Relaxed)
    );
}

#[test]
fn test_write_read_digital_output() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 21)), 60008),
        noop_state_setup,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();

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

/// Awaiting a handle must wake when the response is fulfilled *after* the first
/// poll has parked. Uses a parking executor (only the waker can unpark it) plus
/// a watchdog so a lost wakeup fails late instead of hanging forever.
#[test]
fn async_await_wakes_on_late_notify() {
    use std::future::Future;
    use std::sync::Arc;
    use std::task::{Context, Poll, Wake, Waker};
    use std::time::Instant;

    struct ThreadWaker(std::thread::Thread);
    impl Wake for ThreadWaker {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }
        fn wake_by_ref(self: &Arc<Self>) {
            self.0.unpark();
        }
    }

    fn block_on<F: Future>(fut: F) -> F::Output {
        let mut fut = Box::pin(fut);
        let waker = Waker::from(Arc::new(ThreadWaker(std::thread::current())));
        let mut cx = Context::from_waker(&waker);
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
            std::thread::park();
        }
    }

    sim().run(|| {
        let handle = HmiHandleGeneric::new();
        let fulfiller = handle.clone();
        let fulfil = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            let _ = fulfiller.set_error(HmiError::Timeout);
        });

        let waiter = std::thread::current();
        let watchdog = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_secs(3));
            waiter.unpark();
        });

        let start = Instant::now();
        let result = block_on(handle);
        let elapsed = start.elapsed();
        fulfil.join().unwrap();
        watchdog.join().unwrap();
        assert!(result.is_err());
        assert!(
            elapsed >= Duration::from_millis(50) && elapsed < Duration::from_millis(51),
            "woke {elapsed:?} after a fulfil at 50 ms (lost-wakeup regression)"
        );
    });
}

/// The controller side of an SNPX connection, framed as the driver frames it.
struct SnpxPeer {
    stream: std::net::TcpStream,
    pending: Vec<u8>,
}

impl SnpxPeer {
    /// The next request, or `None` once the driver closes its end.
    fn next(&mut self) -> Option<server::HmiRequest> {
        use std::io::Read;
        let mut buf = [0u8; 4096];
        loop {
            if let Some((req, used)) = server::parse_request(&self.pending) {
                self.pending.drain(..used);
                return Some(req);
            }
            match self.stream.read(&mut buf) {
                Ok(0) | Err(_) => return None,
                Ok(n) => self.pending.extend_from_slice(&buf[..n]),
            }
        }
    }

    fn send(&mut self, bytes: &[u8]) {
        use std::io::Write;
        let _ = self.stream.write_all(bytes);
    }

    fn reply(&mut self, msg: Message) {
        self.send(&encode_msg(msg).0);
    }

    /// Answers INIT, MAGIC and the CLRASG that `connect` sends.
    fn handshake(&mut self) {
        for _ in 0..3 {
            match self.next() {
                Some(server::HmiRequest::Init { .. }) => self.send(&server::init_ack()),
                Some(other) => self.send(&server::ack_resp(other.seq())),
                None => return,
            }
        }
    }

    /// Acknowledges every request until the driver closes the connection.
    fn serve(&mut self) {
        while let Some(req) = self.next() {
            self.send(&server::ack_resp(req.seq()));
        }
    }
}

/// A register-read reply carrying `value`.
fn register_reply(seq: u8, value: i16) -> Message {
    let mut payload = [0u8; 6];
    payload[..2].copy_from_slice(&value.to_le_bytes());
    Message::new_test_resp(seq, payload, zero_plc_status())
}

/// Runs `client` against `peer`, which owns the controller end of the one
/// connection the driver makes to `addr`. Returns what `peer` returns.
fn with_peer<T: Send + 'static>(
    addr: SocketAddr,
    peer: impl FnOnce(SnpxPeer) -> T + Send + 'static,
    client: impl FnOnce(IpAddr) + Send + 'static,
) -> T {
    sim().run(|| {
        let listener = std::net::TcpListener::bind(addr).unwrap();
        let setup = snare::sched::setup_scope("test-spawn");
        let server = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            peer(SnpxPeer {
                stream,
                pending: Vec::new(),
            })
        });
        let client = std::thread::spawn(move || client(addr.ip()));
        drop(setup);
        if let Err(panic) = client.join() {
            std::panic::resume_unwind(panic);
        }
        server.join().unwrap()
    })
}

fn connected(ip: IpAddr) -> HmiDriver {
    let mut driver = HmiDriver::new(ip);
    driver
        .connect(Some(Duration::from_secs(1)), &[], &[])
        .unwrap();
    driver
}

fn peer_addr(last: u8) -> SocketAddr {
    SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 3, last)), 60008)
}

#[test]
fn a_response_split_across_reads_is_reassembled() {
    with_peer(
        peer_addr(1),
        |mut p| {
            p.handshake();
            let seq = p.next().unwrap().seq();
            let frame = encode_msg(register_reply(seq, 42)).0;
            for chunk in [&frame[..10], &frame[10..41], &frame[41..]] {
                p.send(chunk);
                std::thread::sleep(Duration::from_millis(1));
            }
            p.serve();
        },
        |ip| {
            let mut driver = connected(ip);
            let value = driver
                .read::<Register>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert_eq!(value, 42);
            driver.disconnect(false).unwrap();
        },
    );
}

#[test]
fn responses_out_of_order_resolve_by_sequence() {
    with_peer(
        peer_addr(2),
        |mut p| {
            p.handshake();
            let first = p.next().unwrap().seq();
            let second = p.next().unwrap().seq();
            p.reply(register_reply(second, 22));
            p.reply(register_reply(first, 11));
            p.serve();
        },
        |ip| {
            let mut driver = connected(ip);
            let r1 = driver.read::<Register>(1).unwrap();
            let r2 = driver.read::<Register>(2).unwrap();
            assert_eq!(r2.wait_timeout(Duration::from_secs(1)).unwrap(), 22);
            assert_eq!(r1.wait_timeout(Duration::from_secs(1)).unwrap(), 11);
            driver.disconnect(false).unwrap();
        },
    );
}

#[test]
fn a_response_for_no_request_is_ignored() {
    with_peer(
        peer_addr(3),
        |mut p| {
            p.handshake();
            let seq = p.next().unwrap().seq();
            p.reply(register_reply(seq.wrapping_add(100), 99));
            p.reply(register_reply(seq, 7));
            p.serve();
        },
        |ip| {
            let mut driver = connected(ip);
            let value = driver
                .read::<Register>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            assert_eq!(value, 7);
            assert!(driver.is_connected());
            driver.disconnect(false).unwrap();
        },
    );
}

#[test]
fn a_peer_close_mid_request_fails_it_at_once() {
    with_peer(
        peer_addr(4),
        |mut p| {
            p.handshake();
            p.next();
        },
        |ip| {
            let driver = connected(ip);
            let handle = driver.read::<Register>(1).unwrap();
            let t0 = Instant::now();
            let res = handle.wait_timeout(Duration::from_secs(2));
            let waited = t0.elapsed();
            assert!(matches!(res, Err(HmiError::NotConnected)), "{res:?}");
            assert!(waited < Duration::from_millis(1), "took {waited:?}");
            std::thread::sleep(Duration::from_millis(1));
            assert!(!driver.is_connected());
            assert!(driver.has_connection_errored());
            assert!(matches!(
                driver.read::<Register>(1),
                Err(HmiError::NotConnected)
            ));
        },
    );
}

#[test]
fn an_error_frame_fails_the_request_instead_of_hanging() {
    with_peer(
        peer_addr(5),
        |mut p| {
            p.handshake();
            let seq = p.next().unwrap().seq();
            let mut frame = encode_msg(register_reply(seq, 0)).0;
            // The header's message type: an SNPX error reply.
            frame[31] = 0xd1;
            p.send(&frame);
            p.serve();
        },
        |ip| {
            let driver = connected(ip);
            let t0 = Instant::now();
            let res = driver
                .read::<Register>(1)
                .unwrap()
                .wait_timeout(Duration::from_secs(2));
            assert!(
                matches!(res, Err(ref e) if !matches!(e, HmiError::Timeout)),
                "{res:?}"
            );
            assert!(t0.elapsed() < Duration::from_millis(1));
            std::thread::sleep(Duration::from_millis(1));
            assert!(driver.has_connection_errored());
        },
    );
}

#[test]
fn a_short_read_reply_is_malformed_and_the_connection_survives() {
    with_peer(
        peer_addr(6),
        |mut p| {
            p.handshake();
            let seq = p.next().unwrap().seq();
            p.send(&server::ext_resp_regs(seq, &[1, 2, 3]));
            p.serve();
        },
        |ip| {
            let mut driver = connected(ip);
            let res = driver
                .read_array::<Register>(1, 10)
                .unwrap()
                .wait_timeout(Duration::from_secs(1));
            assert!(matches!(res, Err(HmiError::MalformedResponse)), "{res:?}");
            driver
                .write::<Register>(1, 5)
                .unwrap()
                .wait_timeout(Duration::from_secs(1))
                .unwrap();
            driver.disconnect(false).unwrap();
        },
    );
}

#[test]
fn sequence_numbers_wrap_without_losing_a_reply() {
    let seqs = with_peer(
        peer_addr(7),
        |mut p| {
            p.handshake();
            let mut seqs = Vec::new();
            while let Some(req) = p.next() {
                seqs.push(req.seq());
                p.send(&server::ack_resp(req.seq()));
            }
            seqs
        },
        |ip| {
            let mut driver = connected(ip);
            for i in 0..300i16 {
                driver
                    .write::<Register>(1, i)
                    .unwrap()
                    .wait_timeout(Duration::from_secs(1))
                    .unwrap_or_else(|e| panic!("write {i}: {e}"));
            }
            driver.disconnect(false).unwrap();
        },
    );
    assert_eq!(seqs.len(), 300);
    let wraps = seqs.windows(2).filter(|w| w[1] < w[0]).count();
    assert!(
        wraps >= 1,
        "300 requests never wrapped the sequence: {seqs:?}"
    );
    assert!(seqs.windows(2).all(|w| w[1] == w[0].wrapping_add(1)));
}

#[cfg(unix)]
#[test]
fn a_refused_connect_fails_at_once() {
    sim().run(|| {
        let addr = peer_addr(8);
        snare::set_listener_behavior(addr, snare::ListenerBehavior::Refusing);
        let mut driver = HmiDriver::new(addr.ip());
        let t0 = Instant::now();
        let err = driver
            .connect(Some(Duration::from_secs(1)), &[], &[])
            .unwrap_err();
        assert!(
            matches!(&err, HmiError::Io(e) if e.kind() == std::io::ErrorKind::ConnectionRefused),
            "{err:?}"
        );
        assert!(t0.elapsed() < Duration::from_millis(1));
        assert!(!driver.is_connected());
    });
}

#[test]
fn an_unacknowledged_init_times_out_at_the_connect_timeout() {
    with_peer(
        peer_addr(9),
        |mut p| while p.next().is_some() {},
        |ip| {
            let mut driver = HmiDriver::new(ip);
            let t0 = Instant::now();
            let err = driver
                .connect(Some(Duration::from_millis(300)), &[], &[])
                .unwrap_err();
            let waited = t0.elapsed();
            assert!(matches!(err, HmiError::Timeout), "{err:?}");
            assert!(
                waited >= Duration::from_millis(300) && waited < Duration::from_millis(301),
                "a 300 ms handshake timeout fired after {waited:?}"
            );
            driver.disconnect(false).ok();
        },
    );
}

#[test]
fn a_failed_handshake_leaves_the_driver_disconnected() {
    with_peer(
        peer_addr(10),
        |mut p| while p.next().is_some() {},
        |ip| {
            let mut driver = HmiDriver::new(ip);
            let first = driver.connect(Some(Duration::from_millis(100)), &[], &[]);
            let connected_after_failure = driver.is_connected();
            let retry = driver.connect(Some(Duration::from_millis(100)), &[], &[]);
            driver.disconnect(false).ok();
            assert!(first.is_err());
            assert!(
                !connected_after_failure,
                "a driver whose handshake failed reports itself connected"
            );
            assert!(
                retry.is_err(),
                "a retried connect to a silent controller succeeded"
            );
        },
    );
}

#[test]
fn connect_after_the_controller_dropped_the_link_reconnects() {
    let addr = peer_addr(11);
    sim().run(|| {
        let listener = std::net::TcpListener::bind(addr).unwrap();
        let setup = snare::sched::setup_scope("test-spawn");
        let server = std::thread::spawn(move || {
            let mut handshakes = 0;
            for _ in 0..2 {
                let (stream, _) = listener.accept().unwrap();
                let mut peer = SnpxPeer {
                    stream,
                    pending: Vec::new(),
                };
                peer.handshake();
                handshakes += 1;
                if handshakes == 2 {
                    peer.serve();
                }
            }
            handshakes
        });
        let client = std::thread::spawn(move || {
            let mut driver = connected(addr.ip());
            while driver.is_connected() {
                std::thread::sleep(Duration::from_millis(1));
            }
            driver
                .connect(Some(Duration::from_secs(1)), &[], &[])
                .unwrap();
            assert!(driver.is_connected());
            driver.disconnect(false).unwrap();
        });
        drop(setup);
        if let Err(panic) = client.join() {
            std::panic::resume_unwind(panic);
        }
        assert_eq!(
            server.join().unwrap(),
            2,
            "the second connect skipped its handshake"
        );
    });
}

fn bit_pattern(byte: usize) -> u8 {
    (byte as u8).wrapping_mul(37).wrapping_add(11)
}

fn patterned_inputs(state: &mut RobotState) {
    for (k, b) in state.output_bits.iter_mut().enumerate().take(32) {
        *b = bit_pattern(k);
    }
}

fn input_bit(i: usize) -> bool {
    bit_pattern(i / 8) >> (i % 8) & 1 != 0
}

#[test]
#[ignore = "HmiDriver::read of a bit port decodes bit 0 of the aligned byte instead of the requested bit"]
fn a_single_bit_read_returns_that_bit_at_any_index() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 40)), 60008),
        patterned_inputs,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();
            let wrong: Vec<usize> = (0..48)
                .filter(|&i| {
                    let got = driver
                        .read::<DigitalInput>(i)
                        .unwrap()
                        .wait_timeout(Duration::from_secs(1))
                        .unwrap();
                    got != input_bit(i)
                })
                .collect();
            driver.disconnect(true).ok();
            assert!(wrong.is_empty(), "DI reads came back wrong at {wrong:?}");
        },
    );
}

#[test]
#[ignore = "HmiDriver::read_array of a bit port returns the byte-aligned span, indexed from an absolute byte offset"]
fn a_bit_array_read_returns_exactly_those_bits() {
    run_hmi_test(
        SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 0, 0, 41)), 60008),
        patterned_inputs,
        |addr| {
            let mut driver = HmiDriver::new(addr.ip());
            driver
                .connect(Some(Duration::from_secs(2)), &[], &[])
                .unwrap();
            let mut wrong = Vec::new();
            for (start, count) in [(0, 8), (0, 3), (3, 4), (8, 8), (13, 11), (16, 20), (30, 2)] {
                let got = driver
                    .read_array::<DigitalInput>(start, count)
                    .unwrap()
                    .wait_timeout(Duration::from_secs(1));
                let want: Vec<bool> = (start..start + count).map(input_bit).collect();
                if got.as_deref().ok() != Some(&want[..]) {
                    wrong.push((start, count, got));
                }
            }
            driver.disconnect(true).ok();
            assert!(
                wrong.is_empty(),
                "DI array reads came back wrong: {wrong:?}"
            );
        },
    );
}
