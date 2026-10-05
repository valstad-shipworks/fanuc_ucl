//! Property tests for the HSPO datagram decoder and [`StreamClock`].
//!
//! Every property runs on a fixed seed, so a failure reproduces on every run;
//! proptest prints the minimal failing input.

use std::time::{Duration, SystemTime};

use proptest::collection::vec;
use proptest::prelude::*;
use proptest::test_runner::RngSeed;

use super::*;

fn config(cases: u32) -> ProptestConfig {
    ProptestConfig {
        cases,
        rng_seed: RngSeed::Fixed(0x4853_504f),
        failure_persistence: None,
        ..ProptestConfig::default()
    }
}

fn wire() -> impl bincode::config::Config {
    bincode::config::standard()
        .with_fixed_int_encoding()
        .with_big_endian()
}

/// The broker's receive path: dispatch on the type field at offset 12, then
/// decode that packet from the datagram.
fn decode_datagram(buf: &[u8]) -> Option<HspoRxPacket> {
    match PacketType::from_bytes(buf, 12) {
        PacketType::TcpCartesianPosition => bincode::decode_from_slice(buf, wire())
            .ok()
            .map(|(p, _)| HspoRxPacket::TcpCartesianPosition(p)),
        PacketType::JointAngles => bincode::decode_from_slice(buf, wire())
            .ok()
            .map(|(p, _)| HspoRxPacket::JointAngles(p)),
        PacketType::Variables => bincode::decode_from_slice(buf, wire())
            .ok()
            .map(|(p, _)| HspoRxPacket::Variables(p)),
        PacketType::Unknown => None,
    }
}

fn encode(packet: &HspoRxPacket) -> Vec<u8> {
    match packet {
        HspoRxPacket::TcpCartesianPosition(p) => bincode::encode_to_vec(p, wire()),
        HspoRxPacket::JointAngles(p) => bincode::encode_to_vec(p, wire()),
        HspoRxPacket::Variables(p) => bincode::encode_to_vec(p, wire()),
    }
    .unwrap()
}

fn tcp_packet() -> impl Strategy<Value = TcpCartesianPositionPacket> {
    (
        any::<[u32; 3]>(),
        any::<u16>(),
        any::<[f32; 6]>(),
        any::<[u32; 2]>(),
    )
        .prop_map(|(h, motion_group, v, s)| TcpCartesianPositionPacket {
            version: h[0],
            index: h[1],
            clock: h[2],
            typ: PacketType::TcpCartesianPosition as u16,
            motion_group,
            x: v[0],
            y: v[1],
            z: v[2],
            yaw: v[3],
            pitch: v[4],
            roll: v[5],
            status: s[0],
            io: s[1],
        })
}

fn joint_packet() -> impl Strategy<Value = JointAnglesPacket> {
    (
        any::<[u32; 3]>(),
        any::<u16>(),
        any::<[f32; 9]>(),
        any::<[u32; 2]>(),
    )
        .prop_map(|(h, motion_group, joints, s)| JointAnglesPacket {
            version: h[0],
            index: h[1],
            clock: h[2],
            typ: PacketType::JointAngles as u16,
            motion_group,
            joints,
            status: s[0],
            io: s[1],
        })
}

fn variables_packet() -> impl Strategy<Value = VariablesPacket> {
    (any::<[u32; 3]>(), any::<[f32; 10]>()).prop_map(|(h, data)| VariablesPacket {
        version: h[0],
        index: h[1],
        clock: h[2],
        typ: PacketType::Variables as u16,
        data,
    })
}

fn any_packet() -> impl Strategy<Value = HspoRxPacket> {
    prop_oneof![
        tcp_packet().prop_map(HspoRxPacket::TcpCartesianPosition),
        joint_packet().prop_map(HspoRxPacket::JointAngles),
        variables_packet().prop_map(HspoRxPacket::Variables),
    ]
}

proptest! {
    #![proptest_config(config(1024))]

    /// Arbitrary datagrams never panic, and whatever the broker accepts it
    /// decodes losslessly: re-encoding gives back the bytes it read.
    #[test]
    fn arbitrary_datagrams_decode_losslessly_or_not_at_all(
        bytes in vec(any::<u8>(), 0..128),
    ) {
        if let Some(p) = decode_datagram(&bytes) {
            let again = encode(&p);
            prop_assert_eq!(&bytes[..again.len()], &again[..]);
        }
    }

    /// Datagrams that carry a valid type field, with the rest random.
    #[test]
    fn typed_datagrams_decode_losslessly_or_not_at_all(
        typ in prop_oneof![Just(1u16), Just(4), Just(16)],
        mut bytes in vec(any::<u8>(), 14..96),
    ) {
        bytes[12..14].copy_from_slice(&typ.to_be_bytes());
        if let Some(p) = decode_datagram(&bytes) {
            let again = encode(&p);
            prop_assert_eq!(&bytes[..again.len()], &again[..]);
        }
    }
}

proptest! {
    #![proptest_config(config(256))]

    #[test]
    fn valid_packets_roundtrip(p in any_packet()) {
        let bytes = encode(&p);
        let decoded = decode_datagram(&bytes).expect("a valid packet decodes");
        prop_assert_eq!(encode(&decoded), bytes);
    }

    #[test]
    fn the_broker_wire_config_matches_the_joint_feedback_encoder(p in joint_packet()) {
        prop_assert_eq!(encode_joint_packet(&p), encode(&HspoRxPacket::JointAngles(p)));
    }

    #[test]
    fn a_truncated_packet_is_rejected(p in any_packet(), cut in any::<prop::sample::Index>()) {
        let bytes = encode(&p);
        let cut = cut.index(bytes.len());
        prop_assert!(decode_datagram(&bytes[..cut]).is_none());
    }

    /// HSPO carries no length field, so a datagram with bytes after the
    /// packet still decodes, to exactly the packet in front.
    #[test]
    fn an_extended_packet_decodes_to_its_prefix(
        p in any_packet(),
        tail in vec(any::<u8>(), 1..32),
    ) {
        let bytes = encode(&p);
        let mut long = bytes.clone();
        long.extend_from_slice(&tail);
        let decoded = decode_datagram(&long).expect("the prefix still decodes");
        prop_assert_eq!(encode(&decoded), bytes);
    }

    /// No checksum covers the payload, so a flipped bit outside the type field
    /// is a different but faithful reading of the bytes. One inside the type
    /// field is never another valid type (1, 4 and 16 are two bits apart), so
    /// the datagram is dropped instead of being read as another stream.
    #[test]
    fn a_single_flipped_bit_is_read_faithfully_or_dropped(
        p in any_packet(),
        bit in any::<prop::sample::Index>(),
    ) {
        let mut bytes = encode(&p);
        let bit = bit.index(bytes.len() * 8);
        bytes[bit / 8] ^= 1 << (bit % 8);
        match decode_datagram(&bytes) {
            None => prop_assert!((12..14).contains(&(bit / 8)), "bit {} dropped the packet", bit),
            Some(decoded) => {
                prop_assert!(!(12..14).contains(&(bit / 8)));
                prop_assert_eq!(encode(&decoded), bytes);
            }
        }
    }
}

const EPOCH_2026: u64 = 1_780_000_000_000_000;

fn micros_of(t: SystemTime) -> u64 {
    t.duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}

fn modulus() -> impl Strategy<Value = u64> {
    prop_oneof![
        Just(128_850_307u64),
        Just(1u64 << 32),
        (1u64 << 20)..=(1u64 << 32),
    ]
}

/// A controller clock counting 1µs per tick from `start` and wrapping at
/// `modulus`, sampled once every `period` µs. `gaps[k]` is how many indices
/// separate packet `k` from packet `k + 1`: everything past 1 is a datagram
/// that never arrived.
#[derive(Debug, Clone)]
struct Stream {
    modulus: u64,
    period: u64,
    start: u64,
    first_index: u32,
    gaps: Vec<u32>,
}

#[derive(Debug, Clone, Copy)]
struct Sent {
    index: u32,
    clock: u32,
    /// Controller time, unwrapped.
    time: u64,
}

impl Stream {
    fn packets(&self) -> Vec<Sent> {
        let mut index = self.first_index;
        let mut time = self.start;
        let mut out = vec![self.sent(index, time)];
        for &gap in &self.gaps {
            index += gap;
            time += u64::from(gap) * self.period;
            out.push(self.sent(index, time));
        }
        out
    }

    fn sent(&self, index: u32, time: u64) -> Sent {
        Sent {
            index,
            clock: (time % self.modulus) as u32,
            time,
        }
    }

    fn wraps_between(&self, a: &Sent, b: &Sent) -> u64 {
        b.time / self.modulus - a.time / self.modulus
    }
}

/// Gaps are mostly single steps, sometimes a loss burst of up to half a clock
/// cycle. The stream can start anywhere in the cycle, often just before a wrap.
fn stream(max_packets: usize) -> impl Strategy<Value = Stream> {
    stream_with(max_packets, true)
}

/// As [`stream`], losing at most three datagrams in a row.
fn steady_stream(max_packets: usize) -> impl Strategy<Value = Stream> {
    stream_with(max_packets, false)
}

fn stream_with(max_packets: usize, bursts: bool) -> impl Strategy<Value = Stream> {
    (modulus(), 250u64..=16_000).prop_flat_map(move |(modulus, period)| {
        let longest = if bursts {
            ((modulus / 2) / period).max(1) as u32
        } else {
            4
        };
        (
            Just(modulus),
            Just(period),
            prop_oneof![0..modulus, (modulus - 4 * period)..modulus],
            0u32..(1 << 31),
            vec(
                prop_oneof![3 => 1u32..=4, 1 => 1u32..=longest],
                1..max_packets,
            ),
        )
            .prop_map(|(modulus, period, start, first_index, gaps)| Stream {
                modulus,
                period,
                start,
                first_index,
                gaps,
            })
    })
}

proptest! {
    #![proptest_config(config(256))]

    /// Exact receive stamps at any modulus: every packet is accepted, the
    /// unwrapped clock advances by exactly the controller time that passed
    /// (no missed and no spurious wrap), and once the whole stream is in,
    /// every packet still resolves to its own receive time.
    #[test]
    fn stream_clock_unwraps_any_modulus_exactly(s in stream(160), epoch in EPOCH_2026..EPOCH_2026 + (1 << 40)) {
        let sent = s.packets();
        prop_assume!(s.wraps_between(&sent[0], sent.last().unwrap()) < StreamClock::WRAP_HISTORY as u64);
        let sc = StreamClock::default();
        let first = sc.accept(sent[0].index, sent[0].clock, epoch + sent[0].time).unwrap();
        for p in &sent[1..] {
            let absolute = sc.accept(p.index, p.clock, epoch + p.time);
            prop_assert_eq!(absolute, Some(first + p.time - sent[0].time), "packet {:?}", p);
        }
        for p in &sent {
            let at = sc.system_time_of(p.index, p.clock).map(micros_of);
            prop_assert_eq!(at, Some(epoch + p.time), "packet {:?}", p);
        }
    }

    /// Receive stamps late by up to `jitter`: a wrap can only be overstated by
    /// how late its packet was, so a resolved time is never further off than
    /// one jitter per wrap since it, plus the newest packet's own lateness.
    #[test]
    fn stream_clock_error_under_jitter_is_bounded_by_the_jitter(
        s in stream(120),
        jitter in 0u64..2_000,
        seed in any::<u64>(),
    ) {
        let sent = s.packets();
        prop_assume!(s.wraps_between(&sent[0], sent.last().unwrap()) < StreamClock::WRAP_HISTORY as u64);
        let late = |k: usize| {
            let x = seed.wrapping_add(k as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15);
            (x >> 32) % (jitter + 1)
        };
        let sc = StreamClock::default();
        for (k, p) in sent.iter().enumerate() {
            prop_assert!(sc.accept(p.index, p.clock, EPOCH_2026 + p.time + late(k)).is_some());
        }
        let newest = sent.last().unwrap();
        for (k, p) in sent.iter().enumerate() {
            let at = micros_of(sc.system_time_of(p.index, p.clock).unwrap()) as i64;
            let error = at - (EPOCH_2026 + p.time + late(k)) as i64;
            let bound = (jitter * (s.wraps_between(p, newest) + 1)) as i64;
            prop_assert!(
                (-bound..=jitter as i64).contains(&error),
                "packet {} resolved {} µs off (bound -{}..={})", k, error, bound, jitter
            );
        }
    }

    /// Datagrams delayed in flight by up to `latency` (which reorders them)
    /// and sometimes duplicated. Packets that arrive behind a newer one are
    /// gated out; everything accepted comes out in index order with its clock
    /// unwrapped to within one latency per wrap of the truth.
    #[test]
    fn stream_clock_gates_reordered_and_duplicated_datagrams(
        s in steady_stream(160),
        latency_periods in 0u64..=3,
        seed in any::<u64>(),
    ) {
        let sent = s.packets();
        let latency = latency_periods * s.period;
        let mix = |k: u64| (seed ^ k).wrapping_mul(0x9e37_79b9_7f4a_7c15) >> 32;
        let mut arrivals: Vec<(u64, usize)> = Vec::new();
        for (k, p) in sent.iter().enumerate() {
            arrivals.push((p.time + mix(2 * k as u64) % (latency + 1), k));
            if mix(2 * k as u64 + 1) % 8 == 0 {
                arrivals.push((p.time + mix(3 * k as u64) % (latency + 1), k));
            }
        }
        arrivals.sort();

        let sc = StreamClock::default();
        let first_time = sent[arrivals[0].1].time;
        let mut first = None;
        let mut newest_index = None;
        let mut last_absolute = 0;
        for &(arrival, k) in &arrivals {
            let p = sent[k];
            let accepted = sc.accept(p.index, p.clock, EPOCH_2026 + arrival);
            let behind = newest_index.is_some_and(|n| p.index < n);
            prop_assert_eq!(accepted.is_none(), behind, "packet {:?}", p);
            let Some(absolute) = accepted else { continue };
            newest_index = Some(p.index);
            let first = *first.get_or_insert(absolute);
            prop_assert!(absolute >= last_absolute, "accepted packets ran backwards at {:?}", p);
            last_absolute = absolute;
            let truth = p.time as i64 - first_time as i64;
            let wraps = (p.time / s.modulus).abs_diff(first_time / s.modulus);
            let bound = (latency * (wraps + 1)) as i64;
            let error = (absolute - first) as i64 - truth;
            prop_assert!(error.abs() <= bound, "packet {:?} unwrapped {} µs off", p, error);
        }
    }

    /// The controller restarts its stream (the index starts over, the clock
    /// keeps running) after an arbitrary pause. The first
    /// `STALE_RUN_LIMIT - 1` packets of the new stream are indistinguishable
    /// from stale ones and are dropped; from then on every packet resolves to
    /// its own receive time again, across any number of further wraps.
    #[test]
    fn stream_clock_recovers_from_a_restart_at_any_modulus(
        before in stream(40),
        pause in 0u64..(1 << 33),
        restart_index in 0u32..1000,
        after_gaps in vec(1u32..=4, 40..200),
    ) {
        let sent = before.packets();
        let last = *sent.last().unwrap();
        prop_assume!(restart_index < last.index);
        let resumed = Stream {
            start: last.time + pause,
            first_index: restart_index,
            gaps: after_gaps,
            ..before.clone()
        };
        let after = resumed.packets();
        prop_assume!(after.last().unwrap().index < last.index);

        let sc = StreamClock::default();
        for p in &sent {
            prop_assert!(sc.accept(p.index, p.clock, EPOCH_2026 + p.time).is_some());
        }
        for (k, p) in after.iter().enumerate() {
            let accepted = sc.accept(p.index, p.clock, EPOCH_2026 + p.time).is_some();
            prop_assert_eq!(accepted, k as u32 >= StreamClock::STALE_RUN_LIMIT - 1, "packet {} after the restart", k);
        }
        let delivered = &after[StreamClock::STALE_RUN_LIMIT as usize - 1..];
        prop_assume!(resumed.wraps_between(&delivered[0], delivered.last().unwrap()) < StreamClock::WRAP_HISTORY as u64);
        for p in delivered {
            let at = sc.system_time_of(p.index, p.clock).map(micros_of);
            prop_assert_eq!(at, Some(EPOCH_2026 + p.time), "packet {:?}", p);
        }
    }

    /// No sequence of header values, however garbled, panics the tracker.
    #[test]
    fn stream_clock_never_panics_on_arbitrary_headers(
        steps in vec((any::<u32>(), any::<u32>(), any::<u64>()), 1..64),
        probes in vec((any::<u32>(), any::<u32>()), 0..8),
    ) {
        let sc = StreamClock::default();
        for &(index, clock, sys) in &steps {
            let _ = sc.accept(index, clock, sys);
        }
        for &(index, clock) in &probes {
            let _ = sc.system_time_of(index, clock);
        }
    }

    /// Plausible header values: indices and clocks that move by bounded steps
    /// in either direction, as reordering, restarts and corruption produce.
    #[test]
    fn stream_clock_never_panics_on_plausible_headers(
        start in (any::<u32>(), any::<u32>(), EPOCH_2026..EPOCH_2026 + (1 << 40)),
        steps in vec((-64i64..=4096, -(1i64 << 31)..=(1i64 << 31), 0u64..(1 << 34)), 1..128),
    ) {
        let sc = StreamClock::default();
        let (mut index, mut clock, mut sys) = start;
        for &(di, dc, ds) in &steps {
            index = (index as i64 + di).rem_euclid(1 << 32) as u32;
            clock = (clock as i64 + dc).rem_euclid(1 << 32) as u32;
            sys += ds;
            let _ = sc.accept(index, clock, sys);
            let _ = sc.system_time_of(index, clock);
        }
    }
}

proptest! {
    #![proptest_config(config(64))]

    /// Nothing arrives for one or more whole clock cycles while the controller
    /// keeps counting `index`. The receive times say exactly how much time
    /// passed, so the unwrapped clock has to advance by all of it whether the
    /// counter came back above or below where it stopped.
    #[test]
    fn stream_clock_spans_a_stall_of_whole_cycles(
        before in steady_stream(20),
        cycles in 1u64..4,
        extra in any::<prop::sample::Index>(),
        after_gaps in vec(1u32..=4, 1..20),
    ) {
        let sent = before.packets();
        let last = *sent.last().unwrap();
        let stall_ticks = cycles * before.modulus + extra.index(before.modulus as usize) as u64;
        let stall_gap = (stall_ticks / before.period).max(1);
        prop_assume!(u64::from(last.index) + stall_gap + 1000 < u64::from(u32::MAX));
        let mut gaps = before.gaps.clone();
        gaps.push(stall_gap as u32);
        gaps.extend(after_gaps);
        let s = Stream { gaps, ..before };
        let all = s.packets();

        let sc = StreamClock::default();
        let first = sc.accept(all[0].index, all[0].clock, EPOCH_2026 + all[0].time).unwrap();
        for p in &all[1..] {
            let absolute = sc.accept(p.index, p.clock, EPOCH_2026 + p.time);
            prop_assert_eq!(absolute, Some(first + p.time - all[0].time), "packet {:?}", p);
        }
        for p in &all {
            let at = sc.system_time_of(p.index, p.clock).map(micros_of);
            prop_assert_eq!(at, Some(EPOCH_2026 + p.time), "packet {:?}", p);
        }
    }

    /// The packet index is a free-running u32 too. Counting past `u32::MAX`
    /// is the stream continuing, not restarting, so no packet may be lost.
    #[test]
    fn stream_clock_follows_the_index_past_u32_max(
        s in steady_stream(40),
        before_wrap in 1u32..20,
    ) {
        let sent = Stream { first_index: u32::MAX - before_wrap, ..s.clone() };
        let sc = StreamClock::default();
        let mut index = sent.first_index;
        let mut time = sent.start;
        for (k, &gap) in std::iter::once(&0).chain(sent.gaps.iter()).enumerate() {
            index = index.wrapping_add(gap);
            time += u64::from(gap) * sent.period;
            let clock = (time % sent.modulus) as u32;
            prop_assert!(sc.accept(index, clock, EPOCH_2026 + time).is_some(), "packet {} (index {}) dropped", k, index);
            prop_assert_eq!(sc.system_time_of(index, clock).map(micros_of), Some(EPOCH_2026 + time));
        }
    }

    /// One packet carries a corrupted clock (a single flipped bit) and is
    /// accepted. Every other packet in the stream, resolved after the stream
    /// has moved on, must still come back at its own receive time: the
    /// receive times contradict the corrupt value, and one bad datagram must
    /// not move the timestamps of the ones around it.
    #[test]
    fn stream_clock_contains_a_single_corrupted_clock(
        s in steady_stream(80),
        victim in any::<prop::sample::Index>(),
        bit in 0u32..32,
    ) {
        let sent = s.packets();
        prop_assume!(sent.len() > 2);
        prop_assume!(s.wraps_between(&sent[0], sent.last().unwrap()) < StreamClock::WRAP_HISTORY as u64);
        let victim = 1 + victim.index(sent.len() - 2);
        let sc = StreamClock::default();
        for (k, p) in sent.iter().enumerate() {
            let clock = if k == victim { p.clock ^ (1 << bit) } else { p.clock };
            sc.accept(p.index, clock, EPOCH_2026 + p.time);
        }
        for (k, p) in sent.iter().enumerate() {
            if k == victim {
                continue;
            }
            let at = micros_of(sc.system_time_of(p.index, p.clock).unwrap());
            let error = at.abs_diff(EPOCH_2026 + p.time);
            prop_assert!(
                error <= s.period,
                "packet {} of {} resolved {} µs off (corrupted {})", k, sent.len(), error, victim
            );
        }
    }

    /// A controller that leaves `index` fixed, on a counter shorter than
    /// three quarters of the field's range. With the index frozen only the
    /// boundary check can see the wrap, and its guard band is a quarter of the
    /// assumed 2^32 cycle until a wrap has been measured, which never happens.
    #[test]
    fn stream_clock_fixed_index_wraps_on_a_short_counter(
        modulus in (1u64 << 20)..(3u64 << 30),
        steps in 2usize..64,
    ) {
        let period = 4_000u64;
        let start = modulus - period;
        let sc = StreamClock::default();
        for k in 0..steps as u64 {
            let time = start + k * period;
            sc.accept(7, (time % modulus) as u32, EPOCH_2026 + time);
        }
        for k in 0..steps as u64 {
            let time = start + k * period;
            let at = sc.system_time_of(7, (time % modulus) as u32).map(micros_of);
            prop_assert_eq!(at, Some(EPOCH_2026 + time), "step {}", k);
        }
    }
}

#[test]
fn a_fixed_index_stream_at_the_full_range_wraps_on_the_boundary() {
    let sc = StreamClock::default();
    let modulus = 1u64 << 32;
    let start = modulus - 3 * 4_000;
    for k in 0..8u64 {
        let time = start + k * 4_000;
        let clock = (time % modulus) as u32;
        assert_eq!(sc.accept(0, clock, EPOCH_2026 + time), Some(time));
        assert_eq!(
            sc.system_time_of(0, clock),
            Some(SystemTime::UNIX_EPOCH + Duration::from_micros(EPOCH_2026 + time))
        );
    }
}

/// Two corrupted headers, each an index jump to `u32::MAX` right after a
/// clock jump to `u32::MAX`, with ordinary receive times between them. The
/// rate estimate times the index jump is folded into the base each time.
#[test]
fn stream_clock_survives_corrupted_index_and_clock_jumps() {
    let sc = StreamClock::default();
    let mut sys = EPOCH_2026;
    let mut send = |index: u32, clock: u32| {
        sys += 8_000;
        sc.accept(index, clock, sys);
    };
    for round in 0..2u32 {
        let first = round * 100;
        send(first, 0);
        send(first + 1, u32::MAX);
        send(u32::MAX, 0);
        for i in 0..StreamClock::STALE_RUN_LIMIT {
            send(first + 10 + i, 0);
        }
    }
    send(200, 1);
}
