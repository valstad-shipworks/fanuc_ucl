//! Property tests for the sequence arithmetic in [`ControllerBuffer`].
//!
//! The status sequence number is a u32 the controller increments once per
//! transmission and rolls from `0xFFFFFFFF` to `0`. Every property runs on a
//! fixed seed, so a failure reproduces on every run.

use std::collections::HashSet;
use std::time::{Duration, Instant};

use proptest::collection::vec;
use proptest::prelude::*;
use proptest::test_runner::RngSeed;

use super::*;

fn config(cases: u32) -> ProptestConfig {
    ProptestConfig {
        cases,
        rng_seed: RngSeed::Fixed(0x5345_5121),
        failure_persistence: None,
        ..ProptestConfig::default()
    }
}

#[derive(Debug, PartialEq, Eq)]
enum Answer {
    /// `saw_status` refused it.
    Refused,
    Stale,
    /// Sequences put on the wire for this status, catch-ups first.
    Commanded(Vec<u32>),
}

/// The driver's handling of one status, as `pump_rx` and `respond_inner` do
/// it with the controller taking commands.
struct Driver {
    buf: ControllerBuffer,
    origin: Instant,
    on_wire: HashSet<u32>,
    repeated: Vec<u32>,
}

impl Driver {
    fn new() -> Self {
        Self {
            buf: ControllerBuffer::new(CAPACITY),
            origin: Instant::now(),
            on_wire: HashSet::new(),
            repeated: Vec::new(),
        }
    }

    fn status(&mut self, seq: u32, cycle: u64) -> Answer {
        let at = self.origin + Duration::from_millis(8 * cycle);
        if !self.buf.saw_status(seq, at) {
            return Answer::Refused;
        }
        let outstanding = match self.buf.plan(seq) {
            SeqVerdict::Stale => return Answer::Stale,
            SeqVerdict::Resync => 0,
            SeqVerdict::Command { outstanding } => outstanding,
        };
        let burst = self.buf.burst_for(outstanding);
        let sent: Vec<u32> = (0..burst)
            .map(|i| seq.wrapping_sub(burst - i))
            .chain(std::iter::once(seq))
            .collect();
        for &s in &sent {
            self.buf.commanded(s);
            if !self.on_wire.insert(s) {
                self.repeated.push(s);
            }
        }
        Answer::Commanded(sent)
    }
}

fn start() -> impl Strategy<Value = u32> {
    prop_oneof![any::<u32>(), (u32::MAX - 64)..=u32::MAX]
}

proptest! {
    #![proptest_config(config(256))]

    /// Statuses with losses of up to 7 in a row, starting anywhere and
    /// crossing `u32::MAX` as often as the run allows. Every status is
    /// answered, owes exactly the sequences lost before it, and the lost
    /// count is exact: the wrap is never mistaken for a restart or a gap of
    /// four billion.
    #[test]
    fn statuses_are_answered_across_the_u32_wrap(first in start(), gaps in vec(1u32..=8, 1..600)) {
        let mut d = Driver::new();
        let mut seq = first;
        let mut cycle = 0u64;
        prop_assert_eq!(d.status(seq, cycle), Answer::Commanded(vec![seq]));
        let mut lost = 0u64;
        for &gap in &gaps {
            seq = seq.wrapping_add(gap);
            cycle += u64::from(gap);
            lost += u64::from(gap - 1);
            let Answer::Commanded(sent) = d.status(seq, cycle) else {
                return Err(TestCaseError::fail(format!("status {seq} not answered")));
            };
            let catch_up = sent.len() as u32 - 1;
            prop_assert!(catch_up < gap, "{} catch-ups for a gap of {}", catch_up, gap);
            let expected: Vec<u32> = (0..=catch_up).rev().map(|i| seq.wrapping_sub(i)).collect();
            prop_assert_eq!(sent, expected);
        }
        prop_assert_eq!(d.buf.lost_statuses(), lost);
        prop_assert_eq!(d.repeated, Vec::<u32>::new());
    }

    /// The catch-up burst fills exactly the sequences the controller was
    /// never given, wrapping below zero when the newest status is just past
    /// the rollover.
    #[test]
    fn catch_up_fills_the_skipped_sequences_across_the_wrap(
        first in (u32::MAX - 16)..=u32::MAX,
        warmup in 10u32..20,
        gap in 2u32..=5,
    ) {
        let mut d = Driver::new();
        let mut seq = first.wrapping_sub(warmup);
        for k in 0..warmup {
            prop_assert!(matches!(d.status(seq, u64::from(k)), Answer::Commanded(_)));
            seq = seq.wrapping_add(1);
        }
        let skipped: Vec<u32> = (0..gap - 1).map(|i| seq.wrapping_add(i)).collect();
        let newest = seq.wrapping_add(gap - 1);
        let mut expected = skipped.clone();
        expected.push(newest);
        prop_assert_eq!(d.status(newest, u64::from(warmup + gap)), Answer::Commanded(expected));
        prop_assert_eq!(d.repeated, Vec::<u32>::new());
    }

    /// Duplicated datagrams (bursts of up to 15 copies) and adjacent swaps,
    /// across the wrap. No sequence ever goes to the controller twice, and
    /// every status that is newer than all before it is answered.
    #[test]
    fn duplicates_and_reorders_never_repeat_a_sequence(
        first in start(),
        events in vec((0u8..8, 1u32..=15), 1..300),
    ) {
        let mut arrivals = vec![first];
        let mut seq = first;
        for &(kind, copies) in &events {
            seq = seq.wrapping_add(1);
            match kind {
                0 => arrivals.extend(std::iter::repeat_n(seq, copies as usize + 1)),
                1 => {
                    let next = seq.wrapping_add(1);
                    arrivals.extend([next, seq]);
                    seq = next;
                }
                _ => arrivals.push(seq),
            }
        }

        let mut d = Driver::new();
        let mut newest: Option<u32> = None;
        for (cycle, &s) in arrivals.iter().enumerate() {
            let ahead = newest.is_none_or(|n| {
                let delta = s.wrapping_sub(n);
                delta != 0 && delta <= u32::MAX / 2
            });
            let answer = d.status(s, cycle as u64);
            if ahead {
                prop_assert!(matches!(answer, Answer::Commanded(_)), "status {} answered {:?}", s, answer);
                newest = Some(s);
            }
        }
        prop_assert_eq!(d.repeated, Vec::<u32>::new());
    }

    /// A counter that rolled over at some modulus other than 2^32, which is
    /// not what the manual specifies but is what a controller doing so would
    /// cause. The rollover reads as a restart: fewer than
    /// `2 * RESYNC_STATUSES` statuses go unanswered while the buffer resyncs,
    /// every one after is answered, and no sequence goes out twice. Only a
    /// rollover within `FORWARD_WINDOW` of 2^32 is taken for forward motion,
    /// and counts the sequences it skipped as lost; losses among the statuses
    /// refused during a resync go uncounted.
    #[test]
    fn a_shorter_counter_resyncs_at_its_rollover(
        modulus in prop_oneof![(1u64 << 12)..(1u64 << 32), ((1u64 << 32) - 2048)..(1u64 << 32)],
        before in 1u32..64,
        gaps in vec(1u32..=3, 64..200),
    ) {
        let mut d = Driver::new();
        let mut value = modulus - u64::from(before);
        prop_assert!(matches!(d.status(value as u32, 0), Answer::Commanded(_)));
        let mut lost = 0u64;
        let mut wrapped = false;
        let mut resynced = false;
        let mut unanswered = 0;
        for (k, &gap) in gaps.iter().enumerate() {
            let next = (value + u64::from(gap)) % modulus;
            if next < value {
                wrapped = true;
            } else {
                lost += u64::from(gap - 1);
            }
            value = next;
            let answered = matches!(d.status(value as u32, k as u64 + 1), Answer::Commanded(_));
            if !wrapped {
                prop_assert!(answered);
            } else if resynced {
                prop_assert!(answered, "status {} after the resync went unanswered", value);
            } else if answered {
                resynced = true;
            } else {
                unanswered += 1;
            }
        }
        prop_assert!(wrapped && resynced);
        prop_assert!(unanswered < 2 * ControllerBuffer::RESYNC_STATUSES, "{} unanswered", unanswered);
        prop_assert_eq!(d.repeated, Vec::<u32>::new());
        let skipped = (1u64 << 32) - modulus + 2;
        let uncounted = 2 * u64::from(unanswered + 1);
        prop_assert!(
            (lost.saturating_sub(uncounted)..=lost + skipped).contains(&d.buf.lost_statuses()),
            "lost {} of {} (+{} skipped)", d.buf.lost_statuses(), lost, skipped
        );
    }
}

/// Sixteen copies of one status in a row: four refusals per resync in
/// `saw_status`, four stale plans per resync in `plan`. Nothing distinguishes
/// the copies from a restart, so the buffer resyncs onto the sequence it has
/// already sent.
#[test]
fn a_burst_of_identical_statuses_never_repeats_its_sequence() {
    let mut d = Driver::new();
    for seq in 1..=20u32 {
        d.status(seq, u64::from(seq));
    }
    for k in 0..16 {
        d.status(20, 21 + k);
    }
    assert_eq!(d.repeated, Vec::<u32>::new());
}
