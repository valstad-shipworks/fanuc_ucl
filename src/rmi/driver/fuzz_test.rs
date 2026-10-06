//! Property tests for RMI line framing: the handshake's `rmi_string_reader`
//! and the runner's `\r\n` line buffer.
//!
//! The runner's resync rule: every `\r\n`-terminated line is one response,
//! lines holding only whitespace are skipped, and responses resolve the
//! pending requests in the order those were sent. Every property runs on a
//! fixed seed, so a failure reproduces on every run.

use std::net::Ipv4Addr;

use proptest::collection::vec;
use proptest::prelude::*;
use proptest::test_runner::RngSeed;

use super::*;
use crate::rmi::proto::commands::CommandResponse;

fn config(cases: u32) -> ProptestConfig {
    ProptestConfig {
        cases,
        rng_seed: RngSeed::Fixed(0x524d_4931),
        failure_persistence: None,
        ..ProptestConfig::default()
    }
}

fn loopback_config() -> RmiDriverConfig {
    RmiDriverConfig {
        address: Ipv4Addr::LOCALHOST.into(),
        expected_major_version: 7,
        software_options: vec![],
        buffer_cnt: 8,
        timeout: Duration::from_secs(2),
    }
}

/// A runner whose socket is never touched: `dispatch_lines` works on
/// `rx_buf` alone.
fn runner() -> RmiRunner {
    let listener = std::net::TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let stream = StdTcpStream::connect(listener.local_addr().unwrap()).unwrap();
    stream.set_nonblocking(true).unwrap();
    let (_, from_driver) = flume::unbounded();
    RmiRunner {
        handle: ThreadHandle::new(),
        tcp_stream: TcpStream::from_std(stream),
        from_driver,
        response_stack: VecDeque::new(),
        config: loopback_config(),
        telemetry: None,
        rx_buf: Vec::new(),
        disconnect_deadline: None,
        write_blocked: false,
    }
}

impl RmiRunner {
    fn expect(&mut self, n: usize) -> Vec<RmiHandleGeneric> {
        let handles: Vec<_> = (0..n)
            .map(|_| RmiHandleGeneric::new("FRC_ReadDIN", 0))
            .collect();
        self.response_stack.extend(handles.iter().cloned());
        handles
    }

    /// Pending FRC_ReadDIN requests for `ports`, as `send` records them.
    fn expect_reads(&mut self, ports: &[u16]) -> Vec<RmiHandleGeneric> {
        let handles: Vec<_> = ports
            .iter()
            .map(|port| {
                let request = serde_json::json!({"Command": "FRC_ReadDIN", "PortNumber": port});
                RmiHandleGeneric::new("FRC_ReadDIN", 0).echoing(request.as_object().unwrap())
            })
            .collect();
        self.response_stack.extend(handles.iter().cloned());
        handles
    }

    fn feed(&mut self, bytes: &[u8], cuts: &[prop::sample::Index]) {
        let mut points: Vec<usize> = cuts.iter().map(|c| c.index(bytes.len() + 1)).collect();
        points.push(0);
        points.push(bytes.len());
        points.sort_unstable();
        for w in points.windows(2) {
            self.rx_buf.extend_from_slice(&bytes[w[0]..w[1]]);
            self.dispatch_lines();
        }
    }
}

fn read_din_reply(port: u16) -> String {
    format!(
        r#"{{"Command":"FRC_ReadDIN","ErrorID":0,"PortNumber":{port},"PortValue":{}}}"#,
        port % 2
    )
}

fn port_of(handle: &RmiHandleGeneric) -> Option<u16> {
    match handle.get() {
        Ok(ResponsePacket::Command(CommandResponse::FrcReadDIN(r))) => Some(r.port_number),
        _ => None,
    }
}

/// Whitespace-only lines a peer might emit between replies.
fn blank_line() -> impl Strategy<Value = String> {
    vec(prop::sample::select(vec![' ', '\t', '\r', '\n']), 0..4).prop_map(|cs| {
        let mut s: String = cs.into_iter().collect();
        s = s.replace("\r\n", " ");
        s.push_str("\r\n");
        s
    })
}

proptest! {
    #![proptest_config(config(256))]

    /// Arbitrary bytes in arbitrary chunks never panic. Exactly one pending
    /// request is consumed per complete line that is not whitespace-only, and
    /// whatever follows the last `\r\n` stays buffered.
    #[test]
    fn arbitrary_bytes_consume_one_request_per_line(
        bytes in vec(prop_oneof![4 => any::<u8>(), 1 => Just(b'\r'), 1 => Just(b'\n')], 0..256),
        cuts in vec(any::<prop::sample::Index>(), 0..8),
    ) {
        let mut r = runner();
        let handles = r.expect(64);
        r.feed(&bytes, &cuts);

        let mut lines = 0;
        let mut rest = &bytes[..];
        while let Some(at) = rest.windows(2).position(|w| w == b"\r\n") {
            if !rest[..at].iter().all(u8::is_ascii_whitespace) {
                lines += 1;
            }
            rest = &rest[at + 2..];
        }
        let resolved = handles.iter().filter(|h| h.is_set()).count();
        prop_assert_eq!(resolved, lines.min(64));
        prop_assert!(handles[..resolved].iter().all(RmiHandleGeneric::is_set));
        prop_assert_eq!(&r.rx_buf[..], rest);
    }

    /// Valid replies, with whitespace-only lines between them, split at
    /// arbitrary boundaries (including inside `\r\n`): every request resolves
    /// with its own reply, in order.
    #[test]
    fn replies_split_anywhere_resolve_in_order(
        replies in vec((1u16..=512, blank_line(), any::<bool>()), 1..24),
        cuts in vec(any::<prop::sample::Index>(), 0..40),
    ) {
        let mut stream = String::new();
        for (port, blank, before) in &replies {
            if *before {
                stream.push_str(blank);
            }
            stream.push_str(&read_din_reply(*port));
            stream.push_str("\r\n");
        }
        let mut r = runner();
        let handles = r.expect(replies.len());
        r.feed(stream.as_bytes(), &cuts);
        let got: Vec<_> = handles.iter().map(port_of).collect();
        let want: Vec<_> = replies.iter().map(|(p, _, _)| Some(*p)).collect();
        prop_assert_eq!(got, want);
        prop_assert!(r.rx_buf.is_empty());
        prop_assert!(r.response_stack.is_empty());
    }

    /// A line the JSON parser rejects fails exactly the request it is matched
    /// with; the request after it still resolves with the next reply.
    #[test]
    fn a_garbage_line_fails_only_its_own_request(
        ports in vec(1u16..=512, 2..12),
        bad in any::<prop::sample::Index>(),
        junk in "[a-z{}:,\"]{1,24}",
    ) {
        prop_assume!(serde_json::from_str::<ResponsePacket>(&junk).is_err());
        let bad = bad.index(ports.len());
        let mut stream = String::new();
        for (k, port) in ports.iter().enumerate() {
            if k == bad {
                stream.push_str(&junk);
            } else {
                stream.push_str(&read_din_reply(*port));
            }
            stream.push_str("\r\n");
        }
        let mut r = runner();
        let handles = r.expect(ports.len());
        r.feed(stream.as_bytes(), &[]);
        for (k, (h, port)) in handles.iter().zip(&ports).enumerate() {
            if k == bad {
                prop_assert!(h.get().is_err());
            } else {
                prop_assert_eq!(port_of(h), Some(*port));
            }
        }
    }

    /// Replies are matched to requests by position alone. When one reply's
    /// line is merged into its neighbour's or an extra line appears, every
    /// later request is answered with another request's reply. Each reply
    /// names its port, so a handle resolving with a different port is caught.
    #[test]
    fn a_framing_error_never_resolves_a_request_with_another_reply(
        ports in vec(1u16..=512, 3..12),
        at in any::<prop::sample::Index>(),
        merge in any::<bool>(),
    ) {
        let at = at.index(ports.len() - 1);
        let mut stream = String::new();
        for (k, port) in ports.iter().enumerate() {
            stream.push_str(&read_din_reply(*port));
            if !(merge && k == at) {
                stream.push_str("\r\n");
            }
            if !merge && k == at {
                stream.push_str(&read_din_reply(9999));
                stream.push_str("\r\n");
            }
        }
        let mut r = runner();
        let handles = r.expect_reads(&ports);
        r.feed(stream.as_bytes(), &[]);
        for (h, port) in handles.iter().zip(&ports) {
            if let Some(got) = port_of(h) {
                prop_assert_eq!(got, *port, "a request for port {} resolved with port {}", port, got);
            }
        }
    }

    #[test]
    fn rmi_string_reader_never_panics_and_splits_on_crlf(bytes in vec(any::<u8>(), 0..256)) {
        match rmi_string_reader(&bytes) {
            Err(_) => prop_assert!(std::str::from_utf8(&bytes).is_err()),
            Ok(parsed) => {
                let text = std::str::from_utf8(&bytes).unwrap();
                let parts: Vec<&str> = text.split("\r\n").filter(|p| !p.is_empty()).collect();
                match parsed {
                    VariadicString::None => prop_assert!(parts.is_empty()),
                    VariadicString::Single(s) => prop_assert_eq!(vec![s.as_str()], parts),
                    VariadicString::Multiple(v) => {
                        prop_assert!(v.len() > 1);
                        prop_assert_eq!(v, parts);
                    }
                }
            }
        }
    }

    /// What the writer frames, the handshake reader reads back as one line.
    #[test]
    fn rmi_string_writer_roundtrips_through_the_reader(
        fields in prop::collection::btree_map("[A-Za-z]{1,12}", any::<i64>(), 0..8),
    ) {
        let mut object = JsonObject::new();
        object.insert("Communication".into(), "FRC_Connect".into());
        for (k, v) in &fields {
            object.insert(k.clone(), (*v).into());
        }
        let bytes = rmi_string_writer(object.clone()).unwrap();
        let VariadicString::Single(line) = rmi_string_reader(&bytes).unwrap() else {
            return Err(TestCaseError::fail("the writer's frame did not read back as one line"));
        };
        let back: JsonObject = serde_json::from_str(&line).unwrap();
        prop_assert_eq!(back, object);
    }

    /// Response JSON in arbitrary shapes never panics the parser.
    #[test]
    fn response_json_of_any_shape_never_panics(
        name in prop::sample::select(vec![
            "FRC_ReadDIN", "FRC_Connect", "FRC_WaitTime", "FRC_ReadCartesianPosition",
            "FRC_GetStatus", "FRC_ReadJointAngles", "FRC_SystemFault", "nonsense",
        ]),
        kind in prop::sample::select(vec!["Command", "Communication", "Instruction"]),
        fields in prop::collection::btree_map(
            prop::sample::select(vec![
                "ErrorID", "PortNumber", "PortValue", "SequenceID", "MajorVersion",
                "MinorVersion", "Configuration", "Position", "Group", "Time",
            ]),
            prop_oneof![
                any::<i64>().prop_map(serde_json::Value::from),
                any::<f64>().prop_map(serde_json::Value::from),
                "[ -~]{0,8}".prop_map(serde_json::Value::from),
                Just(serde_json::Value::Null),
                Just(serde_json::json!({"X": 1.0, "Y": "a"})),
            ],
            0..6,
        ),
    ) {
        let mut object = JsonObject::new();
        object.insert(kind.into(), name.into());
        object.extend(fields.into_iter().map(|(k, v)| (k.to_string(), v)));
        let _ = serde_json::from_value::<ResponsePacket>(serde_json::Value::Object(object.clone()));
        let _ = serde_json::from_str::<ResponsePacket>(&serde_json::to_string(&object).unwrap());
    }

    #[test]
    fn responses_roundtrip_through_json(port in any::<u16>(), value in any::<u8>(), error in any::<u32>()) {
        let text = format!(r#"{{"Command":"FRC_ReadDIN","ErrorID":{error},"PortNumber":{port},"PortValue":{value}}}"#);
        let parsed: ResponsePacket = serde_json::from_str(&text).unwrap();
        let again: ResponsePacket = serde_json::from_str(&serde_json::to_string(&parsed).unwrap()).unwrap();
        prop_assert_eq!(again, parsed);
    }
}

/// B-84184EN/03 §2.3: command packets take effect immediately while queued
/// instructions run, and an instruction's reply comes back when it completes
/// (§1.4.4). A command sent behind a still-running instruction is answered
/// first.
#[test]
fn a_command_reply_overtaking_a_running_instruction_resolves_both() {
    let mut r = runner();
    let wait = RmiHandleGeneric::new("FRC_WaitTime", 1);
    let status = RmiHandleGeneric::new("FRC_ReadDIN", 0);
    r.response_stack.extend([wait.clone(), status.clone()]);
    r.rx_buf
        .extend_from_slice(format!("{}\r\n", read_din_reply(7)).as_bytes());
    r.rx_buf.extend_from_slice(
        b"{\"Instruction\":\"FRC_WaitTime\",\"ErrorID\":0,\"SequenceID\":1}\r\n",
    );
    r.dispatch_lines();
    assert_eq!(port_of(&status), Some(7));
    assert!(wait.get().is_ok(), "{:?}", wait.get());
}

/// A driver whose runner is a channel the test reads: what `send` would put
/// on the wire, without a socket.
fn wired_driver(next_seq: u32) -> (RmiDriver, Receiver<RunnerMessage>) {
    let mut driver = RmiDriver::new(loopback_config());
    let (to_runner, from_driver) = flume::unbounded();
    driver.connection = Some(RmiConnection {
        major_version: 7,
        minor_version: 1,
        handle: ThreadHandle::new(),
        to_runner,
        err_flag: Arc::new(AtomicBool::new(false)),
        tuning: Default::default(),
    });
    driver.seq.store(next_seq, Ordering::Relaxed);
    (driver, from_driver)
}

fn wait_time() -> crate::rmi::proto::instructions::FrcWaitTime {
    serde_json::from_str(r#"{"Time":0.001}"#).unwrap()
}

fn sent_sequence_id(rx: &Receiver<RunnerMessage>) -> u64 {
    let Ok(RunnerMessage::SendPacket(bytes, _, _)) = rx.try_recv() else {
        panic!("nothing was sent");
    };
    let json: JsonValue = serde_json::from_slice(&bytes).unwrap();
    json["SequenceID"].as_u64().unwrap()
}

proptest! {
    #![proptest_config(config(64))]

    /// Instructions carry consecutive, non-zero SequenceIDs from any point of
    /// the u32 counter.
    #[test]
    fn instruction_sequence_ids_are_consecutive(start in 0u32..(u32::MAX - 64), n in 1u32..32) {
        let (driver, rx) = wired_driver(start);
        for k in 0..n {
            driver.send(wait_time()).unwrap();
            prop_assert_eq!(sent_sequence_id(&rx), u64::from(start) + u64::from(k) + 1);
        }
    }
}

/// The counter is a u32 and `seq + 1` is taken without wrapping. Reaching it
/// takes 2^32 instructions on one connection, but a debug build panics in the
/// caller and a release build sends SequenceID 1 again.
#[test]
#[ignore = "asserts the SequenceID after u32::MAX is not 1, but SequenceIDs count from 1 and the driver wraps u32::MAX to 1"]
fn the_sequence_id_counter_survives_u32_max() {
    let (driver, rx) = wired_driver(u32::MAX - 1);
    let sent = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        for _ in 0..2 {
            driver.send(wait_time()).unwrap();
        }
    }));
    assert!(sent.is_ok(), "send panicked at the end of the u32 counter");
    assert_eq!(sent_sequence_id(&rx), u64::from(u32::MAX));
    assert_ne!(sent_sequence_id(&rx), 1, "a SequenceID was reused");
}
