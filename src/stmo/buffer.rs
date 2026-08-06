//! Model of the controller's command buffer.
//!
//! Stream Motion is not request/response at the packet level. The controller
//! queues the commands it receives and consumes one per interpolation cycle,
//! and it faults on all four ways that can go wrong: the queue overflowing, the
//! queue running dry while the robot is moving, a cycle going unanswered, and a
//! sequence number arriving twice. Mirroring the queue locally is what lets the
//! driver answer the questions those constraints raise — how many commands may
//! go out at once, how long a blocked send may be retried before the robot
//! starves, and which sequence numbers are still unused.

use std::time::{Duration, Instant};

/// Commands the controller holds before it faults on overflow.
pub const CAPACITY: u8 = 10;

/// Interpolation period assumed until enough statuses have arrived to measure
/// the real one.
const NOMINAL_CYCLE: Duration = Duration::from_millis(8);
/// Bounds on the measured cycle. A clock artefact or a burst of reordered
/// statuses must not turn into an unbounded retry window or a zero-length one.
const MIN_CYCLE: Duration = Duration::from_millis(1);
const MAX_CYCLE: Duration = Duration::from_millis(100);
/// Inverse weight of each new interval in the cycle estimate.
const CYCLE_ALPHA: u64 = 8;
/// Sequence deltas beyond this are treated as a restart or a reorder rather
/// than a run of lost cycles, and contribute no rate sample.
const MAX_MEASURABLE_DELTA: u32 = 16;

/// What to do with an incoming status, given what the controller already has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeqVerdict {
    /// Answer this status, first filling the `outstanding` sequences before it
    /// that the controller was never given.
    Command { outstanding: u32 },
    /// The controller's counter restarted; answer this status and take it as
    /// the new origin.
    Resync,
    /// Answering would repeat a sequence number the controller has consumed.
    Stale,
}

#[derive(Debug)]
pub struct ControllerBuffer {
    /// How full the controller's queue gets before it starts moving the robot.
    /// A controller-side setting the driver has to be told about; it decides
    /// both the steady-state depth and how much loss is recoverable.
    drain_threshold: u8,
    depth: u8,
    draining: bool,
    cycle: Duration,
    last_status: Option<(u32, Instant)>,
    last_commanded: Option<u32>,
    stale: u32,
    underruns: u64,
    lost_statuses: u64,
}

impl ControllerBuffer {
    /// How far ahead of the last answered sequence a status may be and still
    /// count as forward progress. Past this the controller restarted its
    /// counter or the datagram arrived out of order.
    const FORWARD_WINDOW: u32 = 1024;
    /// Statuses outside the forward window before the sequence is accepted as
    /// restarted. Sustained backwards motion is a restart; one or two packets
    /// is reordering, and answering those would repeat a sequence number.
    const RESYNC_STATUSES: u32 = 4;

    pub fn new(drain_threshold: u8) -> Self {
        Self {
            drain_threshold: drain_threshold.clamp(1, CAPACITY),
            depth: 0,
            draining: false,
            cycle: NOMINAL_CYCLE,
            last_status: None,
            last_commanded: None,
            stale: 0,
            underruns: 0,
            lost_statuses: 0,
        }
    }

    /// Folds a status into the model. Called for every status received,
    /// including those seen while a send is being retried — the drain clock
    /// runs whether or not the driver is in a position to answer.
    pub fn saw_status(&mut self, seq: u32, at: Instant) {
        let elapsed = match self.last_status {
            Some((prev_seq, prev_at)) => {
                let delta = seq.wrapping_sub(prev_seq);
                if (1..=MAX_MEASURABLE_DELTA).contains(&delta) {
                    let sample = at.saturating_duration_since(prev_at) / delta;
                    self.cycle = blend(self.cycle, sample.clamp(MIN_CYCLE, MAX_CYCLE));
                    // Every sequence between two received statuses is one the
                    // controller sent and we never saw.
                    self.lost_statuses += (delta - 1) as u64;
                    delta
                } else {
                    // A restart or a reorder: no usable interval, and no
                    // grounds to claim that many cycles were drained.
                    1
                }
            }
            None => 0,
        };
        self.last_status = Some((seq, at));

        if self.draining && elapsed > 0 {
            let drained = elapsed.min(u8::MAX as u32) as u8;
            if drained > self.depth {
                // The controller wanted more cycles' worth than it was holding.
                // Draining to exactly zero is survivable if this cycle refills
                // it; needing one more than it had is not.
                self.underruns += 1;
                self.depth = 0;
            } else {
                self.depth -= drained;
            }
        }
    }

    /// Decides whether this status may be answered, and how many sequences
    /// before it the controller is still owed.
    pub fn plan(&mut self, seq: u32) -> SeqVerdict {
        let Some(prev) = self.last_commanded else {
            self.stale = 0;
            return SeqVerdict::Command { outstanding: 0 };
        };
        let advance = seq.wrapping_sub(prev);
        if advance != 0 && advance <= Self::FORWARD_WINDOW {
            self.stale = 0;
            return SeqVerdict::Command {
                outstanding: advance - 1,
            };
        }
        self.stale += 1;
        if self.stale < Self::RESYNC_STATUSES {
            return SeqVerdict::Stale;
        }
        self.stale = 0;
        self.last_commanded = None;
        SeqVerdict::Resync
    }

    /// Records a command that reached the wire.
    pub fn commanded(&mut self, seq: u32) {
        self.depth = (self.depth + 1).min(CAPACITY);
        self.last_commanded = Some(seq);
        if !self.draining && self.depth >= self.drain_threshold {
            self.draining = true;
        }
    }

    /// Records that `seq` needs nothing further, either because the driver
    /// deliberately withheld a command or because the controller was not taking
    /// them that cycle. Unlike [`commanded`](Self::commanded) this adds no
    /// depth — nothing was queued on the far side.
    pub fn settled(&mut self, seq: u32) {
        self.last_commanded = Some(seq);
    }

    /// The controller stopped consuming: a stop packet, or a command carrying
    /// the last-data flag.
    pub fn stream_ended(&mut self) {
        self.depth = 0;
        self.draining = false;
        self.last_commanded = None;
        self.stale = 0;
    }

    /// Commands that may still be queued without faulting the controller.
    pub fn headroom(&self) -> u8 {
        CAPACITY - self.depth
    }

    /// How many of `outstanding` unanswered cycles to refill this cycle.
    ///
    /// Past the drain threshold the queue has already run dry and the robot has
    /// faulted, so refilling then only risks stacking an overflow on top. One
    /// slot is always held back for this cycle's own command.
    pub fn burst_for(&self, outstanding: u32) -> u32 {
        if outstanding == 0 || outstanding > self.drain_threshold as u32 {
            return 0;
        }
        outstanding.min(self.headroom().saturating_sub(1) as u32)
    }

    /// Time before the controller runs out of commands to execute.
    ///
    /// This is the whole retry budget for a blocked send: the queue drains one
    /// command per cycle, so `depth` cycles remain. While the controller is
    /// still filling it consumes nothing, and the answer is bounded by a full
    /// buffer rather than left unbounded.
    pub fn runway(&self) -> Duration {
        let depth = if self.draining { self.depth } else { CAPACITY };
        self.cycle * depth as u32
    }

    /// Measured interpolation period.
    pub fn cycle(&self) -> Duration {
        self.cycle
    }

    pub fn depth(&self) -> u8 {
        self.depth
    }

    #[cfg(test)]
    pub fn draining(&self) -> bool {
        self.draining
    }

    /// Times the queue is believed to have run dry mid-motion, each of which
    /// faults the controller.
    pub fn underruns(&self) -> u64 {
        self.underruns
    }

    /// Status packets the controller sent that never arrived, counted from
    /// gaps between consecutively *received* sequence numbers.
    ///
    /// Distinct from the cycles the driver did not answer one at a time: when
    /// the host is loaded, several statuses land in one read and only the
    /// newest is answered directly, which starves the controller just the same
    /// but is not packet loss.
    pub fn lost_statuses(&self) -> u64 {
        self.lost_statuses
    }
}

fn blend(current: Duration, sample: Duration) -> Duration {
    let c = current.as_nanos() as u64;
    let s = sample.as_nanos() as u64;
    Duration::from_nanos((c * (CYCLE_ALPHA - 1) + s) / CYCLE_ALPHA)
}

#[cfg(test)]
mod test {
    use super::*;

    fn at(ms: u64) -> Instant {
        // A fixed origin keeps the arithmetic readable; only differences matter.
        origin() + Duration::from_millis(ms)
    }

    fn origin() -> Instant {
        use std::sync::OnceLock;
        static ORIGIN: OnceLock<Instant> = OnceLock::new();
        *ORIGIN.get_or_init(Instant::now)
    }

    /// Drives `cycles` of nominal exchange: one status in, one command out.
    fn run_nominal(buf: &mut ControllerBuffer, cycles: u32, start_seq: u32) {
        for i in 0..cycles {
            let seq = start_seq + i;
            buf.saw_status(seq, at(8 * seq as u64));
            assert_eq!(buf.plan(seq), SeqVerdict::Command { outstanding: 0 });
            buf.commanded(seq);
        }
    }

    #[test]
    fn depth_climbs_to_threshold_then_holds() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 4, 1);
        assert_eq!(buf.depth(), 4);
        assert!(
            !buf.draining(),
            "under threshold the controller is still filling"
        );

        run_nominal(&mut buf, 1, 5);
        assert_eq!(buf.depth(), 5);
        assert!(buf.draining());

        // Once draining, one in and one out per cycle holds the depth.
        run_nominal(&mut buf, 20, 6);
        assert_eq!(buf.depth(), 5);
        assert_eq!(buf.underruns(), 0);
    }

    #[test]
    fn depth_never_exceeds_capacity() {
        let mut buf = ControllerBuffer::new(5);
        for seq in 1..=30 {
            buf.saw_status(seq, at(8 * seq as u64));
            // Command far more than the controller can take.
            for _ in 0..4 {
                buf.commanded(seq);
            }
            assert!(buf.depth() <= CAPACITY, "depth {} exceeded", buf.depth());
        }
    }

    #[test]
    fn unanswered_cycles_drain_the_buffer_and_underrun() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 1);
        assert_eq!(buf.depth(), 5);

        // Six cycles pass with nothing sent.
        buf.saw_status(11, at(88));
        assert_eq!(buf.depth(), 0);
        assert_eq!(buf.underruns(), 1);
    }

    #[test]
    fn outstanding_counts_cycles_never_commanded() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 1);
        // Statuses 6, 7 lost; 8 arrives.
        buf.saw_status(8, at(64));
        assert_eq!(buf.plan(8), SeqVerdict::Command { outstanding: 2 });
    }

    #[test]
    fn burst_is_capped_by_drain_threshold() {
        let mut buf = ControllerBuffer::new(4);
        run_nominal(&mut buf, 4, 1);
        // Within the threshold: refill.
        assert_eq!(buf.burst_for(3), 3);
        assert_eq!(buf.burst_for(4), 4);
        // Past it the buffer has already emptied and the robot has faulted.
        assert_eq!(buf.burst_for(5), 0);
        assert_eq!(buf.burst_for(0), 0);
    }

    #[test]
    fn burst_leaves_room_for_this_cycles_command() {
        let mut buf = ControllerBuffer::new(9);
        run_nominal(&mut buf, 9, 1);
        assert_eq!(buf.depth(), 9);
        // One slot of headroom, and it is reserved for the current sequence.
        assert_eq!(buf.headroom(), 1);
        assert_eq!(buf.burst_for(5), 0);
    }

    #[test]
    fn a_repeated_sequence_is_refused() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 1);
        assert_eq!(buf.plan(5), SeqVerdict::Stale);
        assert_eq!(buf.plan(3), SeqVerdict::Stale);
    }

    #[test]
    fn sustained_backwards_sequence_resyncs() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 100);
        // The controller restarted its counter.
        assert_eq!(buf.plan(1), SeqVerdict::Stale);
        assert_eq!(buf.plan(2), SeqVerdict::Stale);
        assert_eq!(buf.plan(3), SeqVerdict::Stale);
        assert_eq!(buf.plan(4), SeqVerdict::Resync);
        assert_eq!(buf.plan(5), SeqVerdict::Command { outstanding: 0 });
    }

    #[test]
    fn a_failed_send_leaves_its_sequence_outstanding() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 1);
        // Cycle 6 arrives and the send fails, so nothing is recorded.
        buf.saw_status(6, at(48));
        assert_eq!(buf.plan(6), SeqVerdict::Command { outstanding: 0 });
        // Cycle 7: sequence 6 was never given to the controller, so it is owed.
        buf.saw_status(7, at(56));
        assert_eq!(buf.plan(7), SeqVerdict::Command { outstanding: 1 });
    }

    #[test]
    fn a_withheld_cycle_is_not_owed() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 1);
        buf.saw_status(6, at(48));
        buf.settled(6);
        buf.saw_status(7, at(56));
        assert_eq!(buf.plan(7), SeqVerdict::Command { outstanding: 0 });
    }

    #[test]
    fn cycle_estimate_converges_on_the_observed_rate() {
        let mut buf = ControllerBuffer::new(5);
        for seq in 1..=60u32 {
            buf.saw_status(seq, at(4 * seq as u64));
        }
        let cycle = buf.cycle();
        assert!(
            cycle > Duration::from_micros(3_800) && cycle < Duration::from_micros(4_200),
            "cycle estimate {cycle:?} did not converge on 4ms"
        );
    }

    #[test]
    fn cycle_estimate_divides_by_the_sequence_gap() {
        let mut buf = ControllerBuffer::new(5);
        // Every other status is lost: 16ms apart, but still an 8ms cycle.
        for i in 1..=60u32 {
            let seq = i * 2;
            buf.saw_status(seq, at(8 * seq as u64));
        }
        let cycle = buf.cycle();
        assert!(
            cycle > Duration::from_micros(7_800) && cycle < Duration::from_micros(8_200),
            "cycle estimate {cycle:?} did not converge on 8ms"
        );
    }

    #[test]
    fn lost_statuses_counts_only_what_never_arrived() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 1);
        assert_eq!(buf.lost_statuses(), 0);

        // Statuses 6 and 7 never arrive.
        buf.saw_status(8, at(64));
        assert_eq!(buf.lost_statuses(), 2);

        // A run of received statuses adds nothing.
        for seq in 9..=20 {
            buf.saw_status(seq, at(8 * seq as u64));
        }
        assert_eq!(buf.lost_statuses(), 2);
    }

    #[test]
    fn runway_tracks_the_remaining_depth() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 5, 1);
        assert_eq!(buf.runway(), buf.cycle() * 5);

        // Sequences 6 through 9 go unanswered: four cycles drained.
        buf.saw_status(9, at(72));
        assert_eq!(buf.depth(), 1);
        assert_eq!(buf.runway(), buf.cycle());
    }

    #[test]
    fn runway_is_bounded_while_the_controller_is_still_filling() {
        let buf = ControllerBuffer::new(5);
        assert!(!buf.draining());
        assert_eq!(buf.runway(), buf.cycle() * CAPACITY as u32);
    }

    #[test]
    fn ending_the_stream_clears_the_model() {
        let mut buf = ControllerBuffer::new(5);
        run_nominal(&mut buf, 8, 1);
        buf.stream_ended();
        assert_eq!(buf.depth(), 0);
        assert!(!buf.draining());
        // A fresh stream starts over rather than reporting a huge backlog.
        buf.saw_status(1, at(0));
        assert_eq!(buf.plan(1), SeqVerdict::Command { outstanding: 0 });
    }

    #[test]
    fn threshold_is_clamped_to_a_usable_range() {
        let mut zero = ControllerBuffer::new(0);
        zero.saw_status(1, at(0));
        zero.commanded(1);
        assert!(
            zero.draining(),
            "a zero threshold must still start draining"
        );

        // Clamped to the real buffer, so nothing past it is ever refilled.
        let huge = ControllerBuffer::new(200);
        assert_eq!(huge.burst_for(CAPACITY as u32 + 1), 0);
        assert_eq!(huge.burst_for(CAPACITY as u32), CAPACITY as u32 - 1);
    }
}
