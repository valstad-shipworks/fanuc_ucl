//! Property tests for SNPX framing and decoding: the runner's reassembly of
//! responses, [`server::parse_request`](crate::hmi::server::parse_request),
//! the message codec, and the ASG value decoders.
//!
//! SNPX frames are a 42-byte header whose text length gives the whole frame
//! as `56 + text_len` bytes; responses are matched to requests by the u8
//! sequence number. There is no delimiter, magic or checksum, so a framer has
//! nothing to resynchronise on: bytes that are not a frame either fail to
//! decode (the runner then ends the connection) or are read as one. Every
//! property runs on a fixed seed, so a failure reproduces on every run.

use std::net::Ipv4Addr;

use proptest::collection::vec;
use proptest::prelude::*;
use proptest::test_runner::RngSeed;

use super::*;
use crate::hmi::HmiDriver;
use crate::hmi::proto::asg::{
    AlarmData, CartesianData, FrameData, HmiWireable, JointData, PositionData, TimeData,
};
use crate::hmi::proto::ports::*;
use crate::hmi::proto::wire::{SegmentSelector, ServiceRequestCode};
use crate::hmi::server::{self, HmiRequest};

fn config(cases: u32) -> ProptestConfig {
    ProptestConfig {
        cases,
        rng_seed: RngSeed::Fixed(0x534e_5058),
        failure_persistence: None,
        ..ProptestConfig::default()
    }
}

/// A runner on a loopback connection whose far end is kept open, so writes
/// land in the kernel buffer and nothing is ever read back.
fn runner() -> (HmiRunner, std::net::TcpStream) {
    let listener = std::net::TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let stream = std::net::TcpStream::connect(listener.local_addr().unwrap()).unwrap();
    let (far, _) = listener.accept().unwrap();
    stream.set_nonblocking(true).unwrap();
    let (_, from_driver) = flume::unbounded();
    let runner = HmiRunner {
        handle: ThreadHandle::new(),
        tcp_stream: TcpStream::from_std(stream),
        from_driver,
        pending_responses: HashMap::new(),
        held: VecDeque::new(),
        read_buffer: Vec::new(),
        shutting_down: false,
        telemetry: None,
    };
    (runner, far)
}

fn encode(msg: &Message) -> Vec<u8> {
    bincode::encode_to_vec(msg, BINCODE_CFG).unwrap()
}

impl HmiRunner {
    fn expect(&mut self, seq: u8) -> HmiHandleGeneric {
        let handle = HmiHandleGeneric::new();
        self.pending_responses.insert(seq, handle.clone());
        handle
    }

    /// Feeds `bytes` split at `cuts`, stopping at the first decode error as
    /// the runner does.
    fn feed(&mut self, bytes: &[u8], cuts: &[prop::sample::Index]) -> HmiResult<()> {
        let mut points: Vec<usize> = cuts.iter().map(|c| c.index(bytes.len() + 1)).collect();
        points.push(0);
        points.push(bytes.len());
        points.sort_unstable();
        for w in points.windows(2) {
            self.read_buffer.extend_from_slice(&bytes[w[0]..w[1]]);
            self.process_buffer()?;
        }
        Ok(())
    }
}

fn response(seq: u8) -> impl Strategy<Value = Message> {
    prop_oneof![
        any::<[u8; 6]>().prop_map(move |p| Message::resp(seq, p)),
        vec(any::<u8>(), 0..300).prop_map(move |p| Message::ext_resp(
            seq,
            ServiceRequestCode::ReadSysMemory,
            SegmentSelector::Registers,
            p
        )),
    ]
}

/// Responses carrying distinct sequence numbers, as a burst of outstanding
/// requests would get.
fn responses(max: usize) -> impl Strategy<Value = Vec<Message>> {
    Just((0..=255u8).collect::<Vec<_>>())
        .prop_shuffle()
        .prop_flat_map(move |seqs| {
            (1..max).prop_flat_map(move |n| {
                seqs[..n]
                    .iter()
                    .map(|&s| response(s).boxed())
                    .collect::<Vec<_>>()
            })
        })
}

fn request() -> impl Strategy<Value = (Message, HmiRequest)> {
    prop_oneof![
        (any::<u8>(), 0u16..8000, 1u16..64).prop_map(|(seq, index, count)| (
            Message::new_read_req::<Register>(seq, index, count),
            HmiRequest::ReadRegs { seq, index, count }
        )),
        (any::<u8>(), 1u16..8000, vec(any::<i16>(), 1..64)).prop_map(|(seq, index, values)| (
            Message::new_write_req::<Register>(seq, index, &values),
            HmiRequest::WriteRegs {
                seq,
                index: index - 1,
                values
            }
        )),
        (any::<u8>(), 1u16..4000, vec(any::<bool>(), 1..80)).prop_map(|(seq, index, bits)| (
            Message::new_write_req::<DigitalOutput>(seq, index, &bits),
            HmiRequest::WriteBits {
                seq,
                segment: server::Segment::InputBit,
                index: index - 1,
                bits
            }
        )),
        (any::<u8>(), "[A-Z]{1,6}( [A-Z0-9$_.\\[\\]]{1,20}){0,4}").prop_map(|(seq, text)| (
            Message::new_write_req::<Command>(seq, 0, std::slice::from_ref(&text)),
            HmiRequest::Command { seq, text }
        )),
    ]
}

proptest! {
    #![proptest_config(config(256))]

    /// Valid responses concatenated into one stream and split at arbitrary
    /// boundaries: every pending request resolves with exactly its own
    /// message, and nothing is left over.
    #[test]
    fn responses_split_anywhere_resolve_their_own_requests(
        msgs in responses(40),
        cuts in vec(any::<prop::sample::Index>(), 0..40),
    ) {
        let (mut r, _far) = runner();
        let handles: Vec<_> = msgs.iter().map(|m| r.expect(m.seq())).collect();
        let stream: Vec<u8> = msgs.iter().flat_map(encode).collect();
        prop_assert!(r.feed(&stream, &cuts).is_ok());
        for (h, m) in handles.iter().zip(&msgs) {
            prop_assert_eq!(h.get().ok(), Some(m.clone()));
        }
        prop_assert!(r.read_buffer.is_empty());
        prop_assert!(r.pending_responses.is_empty());
    }

    /// Arbitrary bytes never panic the reassembler or allocate past what a
    /// 16-bit text length allows; anything left buffered is shorter than the
    /// frame its header announces.
    #[test]
    fn arbitrary_bytes_never_panic_the_reassembler(
        bytes in vec(any::<u8>(), 0..600),
        cuts in vec(any::<prop::sample::Index>(), 0..8),
        msg_type in prop::sample::select(vec![0x00u8, 0xc0, 0xd4, 0x80, 0x94, 0xd1, 0xff]),
    ) {
        let mut bytes = bytes;
        if bytes.len() > 31 {
            bytes[31] = msg_type;
        }
        let (mut r, _far) = runner();
        if r.feed(&bytes, &cuts).is_ok() && r.read_buffer.len() >= 42 {
            let (header, _) =
                bincode::decode_from_slice::<Header, _>(&r.read_buffer[..42], BINCODE_CFG).unwrap();
            prop_assert!(r.read_buffer.len() < 56 + header.payload_len() as usize);
        }
    }

    /// Frames before a burst of junk are delivered intact whatever the junk
    /// does to the framing after it.
    #[test]
    fn frames_before_junk_are_delivered_intact(
        msgs in responses(12),
        junk in vec(any::<u8>(), 1..80),
    ) {
        let (mut r, _far) = runner();
        let handles: Vec<_> = msgs.iter().map(|m| r.expect(m.seq())).collect();
        let mut stream: Vec<u8> = msgs.iter().flat_map(encode).collect();
        stream.extend_from_slice(&junk);
        let _ = r.feed(&stream, &[]);
        for (h, m) in handles.iter().zip(&msgs) {
            prop_assert_eq!(h.get().ok(), Some(m.clone()));
        }
    }

    #[test]
    fn messages_roundtrip_through_the_codec(m in (any::<u8>(), any::<bool>()).prop_flat_map(|(seq, req)| {
        if req {
            request().prop_map(|(m, _)| m).boxed()
        } else {
            response(seq).boxed()
        }
    })) {
        let bytes = encode(&m);
        prop_assert_eq!(bytes.len(), 56 + m.header.payload_len() as usize);
        let (back, used) = bincode::decode_from_slice::<Message, _>(&bytes, BINCODE_CFG).unwrap();
        prop_assert_eq!(used, bytes.len());
        prop_assert_eq!(back, m);
    }

    /// The device-side framer, on the driver's own requests concatenated and
    /// split anywhere: each request is classified with the fields it was
    /// built from, in order, consuming exactly its frame.
    #[test]
    fn parse_request_classifies_the_drivers_requests(
        reqs in vec(request(), 1..16),
        cuts in vec(any::<prop::sample::Index>(), 0..16),
    ) {
        let stream: Vec<u8> = reqs.iter().flat_map(|(m, _)| encode(m)).collect();
        let mut points: Vec<usize> = cuts.iter().map(|c| c.index(stream.len() + 1)).collect();
        points.push(stream.len());
        points.sort_unstable();
        let mut pending = Vec::new();
        let mut parsed = Vec::new();
        let mut from = 0;
        for to in points {
            pending.extend_from_slice(&stream[from..to]);
            from = to;
            while let Some((req, used)) = server::parse_request(&pending) {
                pending.drain(..used);
                parsed.push(req);
            }
        }
        let want: Vec<_> = reqs.into_iter().map(|(_, r)| r).collect();
        prop_assert_eq!(parsed, want);
        prop_assert!(pending.is_empty());
    }

    #[test]
    fn parse_request_never_panics(bytes in vec(any::<u8>(), 0..400), msg_type in any::<u8>()) {
        let mut bytes = bytes;
        if bytes.len() > 31 {
            bytes[31] = msg_type;
        }
        if let Some((_, used)) = server::parse_request(&bytes) {
            prop_assert!(used >= 56 && used <= bytes.len());
        }
    }

    #[test]
    fn asg_values_never_panic_on_arbitrary_bytes(
        bytes in vec(any::<u8>(), 0..220),
        offset in 0usize..240,
        view in 0usize..240,
    ) {
        let _ = f32::unpack(&bytes);
        let _ = i32::unpack(&bytes);
        let _ = i16::unpack_sysvar(&bytes);
        let _ = u16::unpack(&bytes);
        let _ = i8::unpack(&bytes);
        let _ = bool::unpack_sysvar(&bytes);
        let _ = String::unpack(&bytes);
        let _ = CartesianData::unpack(&bytes);
        let _ = JointData::unpack(&bytes);
        let _ = FrameData::unpack(&bytes);
        let _ = PositionData::unpack(&bytes);
        let _ = TimeData::unpack(&bytes);
        let _ = AlarmData::unpack(&bytes);
        let _ = PositionData::partial_unpack::<200>(&bytes, offset, view);
        let _ = CartesianData::partial_unpack::<64>(&bytes, offset, view);
    }

    /// Whatever an ASG decoder accepts, packing it again and decoding gives
    /// the same value: decoding never lands on a value it cannot represent.
    #[test]
    fn asg_values_decode_to_a_fixed_point(bytes in vec(any::<u8>(), 220)) {
        fn fixed_point<T: HmiWireable + PartialEq + std::fmt::Debug>(bytes: &[u8]) -> Result<(), TestCaseError> {
            if let Ok((v, used)) = T::unpack(bytes) {
                prop_assert!(used <= bytes.len());
                let mut again = vec![0u8; 256];
                let n = v.pack(&mut again);
                let (w, _) = T::unpack(&again[..n]).expect("a packed value decodes");
                let same = format!("{v:?}") == format!("{w:?}");
                prop_assert!(same, "{:?} -> {:?}", v, w);
            }
            Ok(())
        }
        fixed_point::<i32>(&bytes)?;
        fixed_point::<i16>(&bytes)?;
        fixed_point::<u16>(&bytes)?;
        fixed_point::<i8>(&bytes)?;
        fixed_point::<bool>(&bytes)?;
        fixed_point::<CartesianData>(&bytes)?;
        fixed_point::<JointData>(&bytes)?;
        fixed_point::<FrameData>(&bytes)?;
        fixed_point::<PositionData>(&bytes)?;
        fixed_point::<TimeData>(&bytes)?;
        fixed_point::<AlarmData>(&bytes)?;
    }

    /// Controller strings are NUL-padded 80-byte fields; any text that fits
    /// reads back as written.
    #[test]
    fn asg_strings_roundtrip(text in "[^\\x00]{0,20}".prop_filter("fits the field", |t| t.len() <= 80)) {
        let mut field = [0xffu8; 80];
        prop_assert_eq!(text.pack(&mut field), 80);
        prop_assert_eq!(String::unpack(&field).unwrap(), (text, 80));
    }

    /// Port values written by the driver read back from the reply payload a
    /// controller returns for that range.
    #[test]
    fn register_values_survive_pack_and_unpack(values in vec(any::<i16>(), 1..100)) {
        let packed = Register::pack_array(0, &values);
        prop_assert_eq!(packed.len(), Register::expected_size(0, values.len() as u16));
        prop_assert_eq!(&*Register::unpack_array(0, &packed, values.len() as u16), &values[..]);
    }

    /// Out-of-range indices are the caller's mistake and come back as an
    /// error, never a panic or a silently different address. A disconnected
    /// driver checks them before it notices it has no connection.
    #[test]
    fn out_of_range_indices_are_errors_not_panics(index in 50_000usize..=65_535, count in 1usize..=64) {
        let driver = HmiDriver::new(Ipv4Addr::new(10, 255, 255, 1));
        type Call<'a> = (&'static str, Box<dyn Fn() -> bool + 'a>);
        let calls: [Call; 4] = [
            ("read WireStickInput", Box::new(|| driver.read::<WireStickInput>(index).is_err())),
            (
                "read_array DigitalInput",
                Box::new(|| driver.read_array::<DigitalInput>(index, count).is_err()),
            ),
            ("write WeldOutput", Box::new(|| driver.write::<WeldOutput>(index, true).is_err())),
            ("read AnalogInput", Box::new(|| driver.read::<AnalogInput>(index).is_err())),
        ];
        for (name, call) in calls {
            let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(call));
            prop_assert!(matches!(ok, Ok(true)), "{} at {} panicked", name, index);
        }
    }
}

/// A 257th request in flight reuses the first one's u8 sequence number while
/// the first is still unanswered.
#[test]
fn a_reused_sequence_never_strands_or_misroutes_a_request() {
    let (mut r, _far) = runner();
    let mut queue = VecDeque::new();
    let handles: Vec<_> = (0..257u32)
        .map(|i| {
            let seq = i as u8;
            let message = Message::new_read_req::<Register>(seq, i as u16, 1);
            let handle = HmiHandleGeneric::new();
            queue.push_back(PendingWrite {
                buf: encode(&message),
                offset: 0,
                seq,
                handle: handle.clone(),
                message,
            });
            handle
        })
        .collect();
    r.write_from_queue(&mut queue).unwrap();
    assert!(queue.is_empty());

    let first = Message::resp(0, [1, 0, 0, 0, 0, 0]);
    r.read_buffer.extend_from_slice(&encode(&first));
    r.process_buffer().unwrap();
    assert_eq!(handles[0].get().ok(), Some(first), "the first request");
    assert!(
        !handles[256].is_set(),
        "the reply to the first resolved the 257th"
    );
}

/// Input libFuzzer found on the `hmi_asg_values` target: 36 bytes for
/// `JointData`, nine joints and no validity byte.
#[test]
fn asg_decoders_survive_the_fuzzer_crash_input() {
    let data: &[u8] = &[
        0x01, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x2b, 0xff,
        0xff, 0xff, 0xff, 0xff, 0x02, 0x00,
    ];
    let _ = f32::unpack(data);
    let _ = i16::unpack_sysvar(data);
    let _ = u16::unpack(data);
    let _ = bool::unpack_sysvar(data);
    let _ = String::unpack(data);
    let _ = CartesianData::unpack(data);
    let _ = JointData::unpack(data);
    let _ = FrameData::unpack(data);
    let _ = PositionData::unpack(data);
    let _ = TimeData::unpack(data);
    let _ = AlarmData::unpack(data);
    let _ = PositionData::partial_unpack::<200>(&data[2..], data[0] as usize, data[1] as usize);
}

/// Every length a reply can be cut to below a value's packed size is an error,
/// never a panic: `from_message` hands the reply payload straight to these.
#[test]
fn a_truncated_asg_reply_is_malformed_not_a_panic() {
    fn every_cut<T: HmiWireable>(name: &str) {
        let full = vec![0u8; T::PACKED_SIZE];
        for len in 0..T::PACKED_SIZE {
            let r = std::panic::catch_unwind(|| T::unpack(&full[..len]).is_err());
            assert!(
                matches!(r, Ok(true)),
                "{name} cut to {len} of {} bytes",
                T::PACKED_SIZE
            );
        }
    }
    every_cut::<CartesianData>("CartesianData");
    every_cut::<JointData>("JointData");
    every_cut::<FrameData>("FrameData");
    every_cut::<PositionData>("PositionData");
    every_cut::<TimeData>("TimeData");
    every_cut::<AlarmData>("AlarmData");
}
