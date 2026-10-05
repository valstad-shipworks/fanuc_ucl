#[cfg(test)]
mod fuzz_test;
#[cfg(test)]
mod test;

use cfg_vis::{cfg_vis, cfg_vis_fields};
use fast_talker::{
    Config, Received, Source, Timestamped,
    options::{SocketOption, ThreadOption},
};
use parking_lot::Mutex;
use std::{
    collections::{HashMap, VecDeque},
    io,
    net::{IpAddr, SocketAddr},
    sync::{
        Arc, LazyLock,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant, SystemTime},
};

use crate::{
    joints::{JointFormat, JointTemplate},
    thread_util::GeneralThreadError,
    tuning::{self, OptionsReport, SocketRole, ThreadRole, TuningReport},
};
use bincode::{Decode, Encode};
use cfg_mixin::cfg_mixin;
use flume::{Receiver, Sender, TrySendError, bounded, unbounded};
use mio::{Events, Interest, Poll, Token, Waker, net::UdpSocket as MioUdpSocket};
use serde::Serialize;

const TOK_SOCKET: Token = Token(0);
const TOK_WAKER: Token = Token(1);

const DRIVER: &str = "hspo";

static HSPO_SERVER: LazyLock<Mutex<Option<HspoBroker>>> = LazyLock::new(|| Mutex::new(None));

/// Error returned when attempting to create an [`HspoReceiver`] before the HSPO broker has been initialized.
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HspoBrokerNotInitializedError;
impl std::fmt::Display for HspoBrokerNotInitializedError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "HSPO server not initialized. Please initialize the server before creating a driver."
        )
    }
}
impl std::error::Error for HspoBrokerNotInitializedError {}
#[cfg(feature = "py")]
impl From<HspoBrokerNotInitializedError> for pyo3::PyErr {
    fn from(err: HspoBrokerNotInitializedError) -> Self {
        pyo3::exceptions::PyRuntimeError::new_err(err.to_string())
    }
}

/// Error returned when the HSPO broker cannot be started.
#[derive(Debug, thiserror::Error)]
pub enum HspoBrokerError {
    /// An option the broker does not accept.
    #[error("{driver} does not accept option {option}")]
    InvalidOption {
        option: String,
        driver: &'static str,
    },
    /// Binding or setting up the socket, applying an option, or spawning the
    /// broker thread failed.
    #[error("HSPO broker setup failed: {0}")]
    Io(#[from] io::Error),
}

impl From<tuning::RefusedOption> for HspoBrokerError {
    fn from(r: tuning::RefusedOption) -> Self {
        HspoBrokerError::InvalidOption {
            option: r.option,
            driver: r.driver,
        }
    }
}

#[cfg(feature = "py")]
impl From<HspoBrokerError> for pyo3::PyErr {
    fn from(err: HspoBrokerError) -> Self {
        match err {
            HspoBrokerError::InvalidOption { .. } => {
                pyo3::exceptions::PyValueError::new_err(err.to_string())
            }
            HspoBrokerError::Io(_) => pyo3::exceptions::PyIOError::new_err(err.to_string()),
        }
    }
}

/// A packet from a FANUC controller containing the TCP (Tool Center Point) cartesian position.
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, Encode, Decode, PartialEq, Serialize)]
#[repr(C)]
pub struct TcpCartesianPositionPacket {
    #[on(pyo3(get))]
    pub version: u32,
    #[on(pyo3(get))]
    pub index: u32,
    #[on(pyo3(get))]
    pub clock: u32,
    #[serde(rename = "type")]
    pub typ: u16,
    #[on(pyo3(get))]
    pub motion_group: u16,
    #[on(pyo3(get))]
    pub x: f32,
    #[on(pyo3(get))]
    pub y: f32,
    #[on(pyo3(get))]
    pub z: f32,
    #[on(pyo3(get))]
    pub yaw: f32,
    #[on(pyo3(get))]
    pub pitch: f32,
    #[on(pyo3(get))]
    pub roll: f32,
    #[on(pyo3(get))]
    pub status: u32,
    #[on(pyo3(get))]
    pub io: u32,
}

impl std::fmt::Display for TcpCartesianPositionPacket {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "TcpCartesianPositionPacket {{ version: {}, index: {}, clock: {}, type: {}, motion_group: {}, x: {}, y: {}, z: {}, yaw: {}, pitch: {}, roll: {}, status: {}, io: {} }}",
            self.version,
            self.index,
            self.clock,
            self.typ,
            self.motion_group,
            self.x,
            self.y,
            self.z,
            self.yaw,
            self.pitch,
            self.roll,
            self.status,
            self.io
        )
    }
}

/// A packet from a FANUC controller containing joint angle values.
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
#[cfg_vis_fields]
pub struct JointAnglesPacket {
    #[on(pyo3(get))]
    pub version: u32,
    #[on(pyo3(get))]
    pub index: u32,
    #[on(pyo3(get))]
    pub clock: u32,
    #[serde(rename = "type")]
    pub typ: u16,
    #[on(pyo3(get))]
    pub motion_group: u16,
    #[cfg_vis(test, pub)]
    joints: [f32; 9],
    #[on(pyo3(get))]
    pub status: u32,
    #[on(pyo3(get))]
    pub io: u32,
}

#[cfg_attr(feature = "py", pyo3::pymethods)]
impl JointAnglesPacket {
    /// Returns the joint angles converted from the internal FANUC radian format to the specified format and template.
    pub fn joints(&self, format: JointFormat, template: JointTemplate) -> [f32; 9] {
        format.convert_from(JointFormat::FanucRad, &template, self.joints)
    }
}

/// Protocol version stamped on controller-emitted feedback packets. The broker
/// ignores it, so any value round-trips.
const FEEDBACK_VERSION: u32 = 1;

impl JointAnglesPacket {
    /// Build a controller-side joint feedback packet from an on-wire **FanucRad**
    /// arm-first body (`[J1..J6, J7=track_mm, 0, 0]`) — the exact representation
    /// [`joints`](Self::joints) reads. Use this when the pose is already in the
    /// FanucRad frame so no interaction re-compensation is applied.
    pub fn feedback_from_fanuc_rad(
        index: u32,
        clock: u32,
        motion_group: u16,
        fanuc_rad: [f32; 9],
        status: u32,
        io: u32,
    ) -> Self {
        Self {
            version: FEEDBACK_VERSION,
            index,
            clock,
            typ: PacketType::JointAngles as u16,
            motion_group,
            joints: fanuc_rad,
            status,
            io,
        }
    }

    /// Build a controller-side joint feedback packet from arm-first **AbsRad**
    /// (`[j1..j6, J7=track_mm, 0, 0]`), converting to the on-wire FanucRad body
    /// (re-applying the FANUC J2/J3 interaction) — the inverse of [`joints`](Self::joints).
    pub fn feedback_from_abs_rad(
        index: u32,
        clock: u32,
        motion_group: u16,
        template: JointTemplate,
        abs_rad: [f32; 9],
        status: u32,
        io: u32,
    ) -> Self {
        let fanuc_rad = JointFormat::FanucRad.convert_from(JointFormat::AbsRad, &template, abs_rad);
        Self::feedback_from_fanuc_rad(index, clock, motion_group, fanuc_rad, status, io)
    }
}

/// Encode a joint feedback packet to a datagram using the broker's wire config
/// (bincode standard, fixed-int, **big-endian**).
pub fn encode_joint_packet(packet: &JointAnglesPacket) -> Vec<u8> {
    let config = bincode::config::standard()
        .with_fixed_int_encoding()
        .with_big_endian();
    bincode::encode_to_vec(packet, config).unwrap_or_default()
}

impl std::fmt::Display for JointAnglesPacket {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "JointAnglesPacket {{ version: {}, index: {}, clock: {}, type: {}, motion_group: {}, joints: {:?}, status: {}, io: {} }}",
            self.version,
            self.index,
            self.clock,
            self.typ,
            self.motion_group,
            self.joints.iter().collect::<Vec<_>>(),
            self.status,
            self.io
        )
    }
}

/// A packet from a FANUC controller containing up to 10 user-configured variable values.
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct VariablesPacket {
    #[on(pyo3(get))]
    pub version: u32,
    #[on(pyo3(get))]
    pub index: u32,
    #[on(pyo3(get))]
    pub clock: u32,
    #[serde(rename = "type")]
    pub typ: u16,
    #[on(pyo3(get))]
    pub data: [f32; 10],
}

impl std::fmt::Display for VariablesPacket {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "VariablesPacket {{ version: {}, index: {}, clock: {}, type: {}, data: {:?} }}",
            self.version,
            self.index,
            self.clock,
            self.typ,
            self.data.iter().collect::<Vec<_>>()
        )
    }
}

/// Header fields shared by every HSPO packet type.
///
/// This trait is sealed and cannot be implemented outside this crate.
pub trait HspoPacket: crate::sealed::Sealed {
    /// Per-stream packet sequence index.
    fn index(&self) -> u32;
    /// Controller clock at send time, 1µs per unit. Free-running and wrapping,
    /// at a modulus that varies by controller and is not the field's full range.
    fn clock(&self) -> u32;
}

macro_rules! impl_hspo_packet {
    ($($pkt:ty),*) => {$(
        impl crate::sealed::Sealed for $pkt {}
        impl HspoPacket for $pkt {
            fn index(&self) -> u32 {
                self.index
            }
            fn clock(&self) -> u32 {
                self.clock
            }
        }
    )*};
}
impl_hspo_packet!(
    TcpCartesianPositionPacket,
    JointAnglesPacket,
    VariablesPacket
);

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(u16)]
#[cfg_vis(test, pub)]
enum PacketType {
    TcpCartesianPosition = 1,
    JointAngles = 4,
    Variables = 16,
    Unknown,
}

impl PacketType {
    #[cfg_vis(test, pub)]
    fn from_bytes(bytes: &[u8], offset: usize) -> Self {
        if bytes.len() < offset + 2 {
            return PacketType::Unknown;
        }
        match u16::from_be_bytes([bytes[offset], bytes[offset + 1]]) {
            1 => PacketType::TcpCartesianPosition,
            4 => PacketType::JointAngles,
            16 => PacketType::Variables,
            _ => PacketType::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_vis(test, pub)]
enum HspoStream {
    Tcp,
    Joint,
    Variables,
}

/// Per-stream clock tracker shared between the broker thread and a channel.
///
/// The broker folds each accepted packet's wrapping clock into a cumulative value,
/// recording at which packet index each wrap was first seen and the offset between
/// the cumulative clock and system time. [`system_time_of`](Self::system_time_of)
/// reconstructs the receive time of a buffered packet from that record.
#[derive(Debug, Default)]
#[cfg_vis(test, pub)]
struct StreamClock {
    state: Mutex<StreamClockState>,
}

/// A recently accepted packet that predictions are made from.
#[derive(Debug, Clone, Copy)]
struct ClockReference {
    index: u32,
    absolute: u64,
    sys_micros: u64,
    /// How far the packet's clock was from where its neighbours put it.
    residual: u64,
}

#[derive(Debug, Default)]
struct StreamClockState {
    last_index: Option<u32>,
    last_clock: u32,
    /// Micros contributed by the cycles already folded in, so that `base + clock`
    /// is the packet's absolute controller time.
    base: u64,
    /// Length of one clock cycle, as measured at wraps. Only used to check that a
    /// later wrap is plausible, never as the amount folded in.
    cycle: Option<u64>,
    /// The newest accepted packets, oldest first.
    references: VecDeque<ClockReference>,
    /// Clock micros per packet index, in 1/2^PERIOD_FRACTION_BITS µs, from recent
    /// steps that did not wrap.
    periods: VecDeque<u64>,
    /// Packets rejected in a row for carrying a stale index.
    stale_run: u32,
    /// Packets in a row whose clock disagreed with the stream.
    suspect_run: u32,
    /// `(first index seen on this base, its clock, base)`, newest last.
    wrap_points: VecDeque<(u32, u32, u64)>,
    /// System micros minus cumulative clock micros, from the newest accepted packet.
    offset_micros: Option<i128>,
    /// Where `offset_micros` was stamped.
    offset_source: Option<Source>,
}

impl StreamClock {
    /// Cycle assumed for a packet older than every recorded wrap point.
    const NOMINAL_CYCLE: u64 = u32::MAX as u64 + 1;
    const WRAP_HISTORY: usize = 32;
    /// Packets rejected in a row before the stream is taken to have restarted
    /// rather than reordered. Deep enough that no plausible datagram reordering
    /// reaches it, shallow enough that recovery costs a fraction of a second.
    const STALE_RUN_LIMIT: u32 = 16;
    /// Packets in a row that disagree with the stream before the stream itself is
    /// taken to have changed. One or two are corrupted datagrams; a run is not.
    const SUSPECT_RUN_LIMIT: u32 = 4;
    /// Index distances at or past this are behind, not ahead: the index is a
    /// free-running u32 that rolls over to 0.
    const INDEX_HALF_RANGE: u32 = 1 << 31;
    const REFERENCES: usize = 8;
    const PERIOD_SAMPLES: usize = 9;
    const PERIOD_FRACTION_BITS: u32 = 8;
    /// The shortest clock cycle taken seriously. A µs counter that wraps more
    /// than once a second is not a controller clock, and anything shorter would
    /// be indistinguishable from the tolerances below.
    const MIN_CYCLE: u64 = 1 << 20;
    /// How far a clock may sit from where the packet index puts it. The index
    /// and the clock both come from the controller's interpolation timer, so
    /// this only has to absorb a fractional period and rounding.
    const INDEX_TOLERANCE: u64 = 1_000;
    /// How far a clock may sit from where the receive times put it. Receive
    /// times carry network and scheduling latency.
    const SYSTEM_TOLERANCE: u64 = Self::MIN_CYCLE / 4;

    /// Gates a packet by index and folds its clock into the cumulative value,
    /// recording the offset against `sys_micros`.
    ///
    /// Returns `None` when `index` is behind the newest already seen on this
    /// stream — a reordered or stale datagram the caller must disregard. Otherwise
    /// returns the absolute cumulative clock `base + clock`.
    ///
    /// Each packet's absolute time is first *predicted* from the newest packets
    /// accepted since the last wrap: from the index distance times the measured
    /// period once two steps agree on one and the index moves, from the elapsed
    /// receive time otherwise (no less than the index puts it, if a period has
    /// been seen at all, since a drained backlog stamps a burst of packets within
    /// the same microsecond). The prediction is the median over those packets,
    /// so one bad packet among them cannot steer it.
    ///
    /// The packet's clock is then read against the prediction. Within tolerance,
    /// no wrap happened. Behind it by at least half the shortest plausible cycle,
    /// the counter wrapped one or more times: the prediction minus the clock is
    /// MEASURED and folded into the base, rather than assumed to be the
    /// counter's full range — controllers do not all count to 2^32 (the R-30iB
    /// cycles at roughly 1.29e8µs) — and it holds whether the clock came back
    /// below or above where it stopped. A wrap is only taken when it is
    /// consistent: the new clock is no further into its cycle than the time that
    /// passed, and the measured advance is a whole number of the cycles seen so
    /// far (or a shorter cycle, which means the earlier one spanned several).
    ///
    /// A packet whose clock fits none of that is still delivered, but nothing is
    /// learned from it: one corrupted clock must not move the stream. A run of
    /// [`SUSPECT_RUN_LIMIT`](Self::SUSPECT_RUN_LIMIT) of them means the stream
    /// itself changed, and it is re-anchored on the receive time.
    ///
    /// A high-water mark alone would strand the stream for good once a controller
    /// restarts its stream and counts from zero again, so a backward index that
    /// persists past [`STALE_RUN_LIMIT`](Self::STALE_RUN_LIMIT) is read as a restart
    /// instead: the clock re-anchors on the new packet and keeps running forward.
    ///
    /// The offset to system time is taken from the newest packet stamped by
    /// the kernel or the NIC, and from user-space stamps only until the first
    /// such packet: a user-space stamp includes however long the packet sat in
    /// the socket buffer.
    fn accept_from(&self, index: u32, clock: u32, sys_micros: u64, source: Source) -> Option<u64> {
        let mut state = self.state.lock();
        let (absolute, committed) = Self::fold_in(&mut state, index, clock, sys_micros)?;
        if committed && (source >= Source::Kernel || state.offset_source < Some(Source::Kernel)) {
            state.offset_micros = Some(sys_micros as i128 - absolute as i128);
            state.offset_source = Some(source);
        }
        Some(absolute)
    }

    /// [`accept_from`](Self::accept_from) with a kernel receive stamp.
    #[cfg(test)]
    pub fn accept(&self, index: u32, clock: u32, sys_micros: u64) -> Option<u64> {
        self.accept_from(index, clock, sys_micros, Source::Kernel)
    }

    /// The packet's absolute clock, and whether the stream learned from it.
    fn fold_in(
        state: &mut StreamClockState,
        index: u32,
        clock: u32,
        sys_micros: u64,
    ) -> Option<(u64, bool)> {
        let Some(last_index) = state.last_index else {
            let base = state.base;
            return Some((state.commit(index, clock, sys_micros, base, 0, false), true));
        };
        let distance = index.wrapping_sub(last_index);
        if distance >= Self::INDEX_HALF_RANGE || state.index_outruns_time(distance, sys_micros) {
            state.stale_run += 1;
            if state.stale_run < Self::STALE_RUN_LIMIT {
                return None;
            }
            // Indices start over, so the recorded points can no longer say
            // which base a buffered packet belongs to. The cycle survives:
            // it is a property of the controller, not of the stream.
            let base = state.resumed_base(clock, sys_micros);
            state.wrap_points.clear();
            state.forget_stream();
            return Some((state.commit(index, clock, sys_micros, base, 0, false), true));
        }
        state.stale_run = 0;

        let fold = match state.period().filter(|_| distance > 0) {
            Some(period) => state.fold(index, clock, sys_micros, Basis::Index(period)),
            // A period not yet agreed on keeps a drained backlog's shared receive
            // stamp from collapsing the spacing, unless it is the period that
            // makes the packet fit nowhere.
            None => state
                .fold(index, clock, sys_micros, Basis::Time(state.any_period()))
                .or_else(|| state.fold(index, clock, sys_micros, Basis::Time(None))),
        };
        if let Some(fold) = fold {
            state.suspect_run = 0;
            return Some((state.apply(index, clock, sys_micros, fold), true));
        }
        state.suspect_run += 1;
        if state.suspect_run < Self::SUSPECT_RUN_LIMIT {
            return Some((state.base.saturating_add(clock as u64), false));
        }
        state.suspect_run = 0;
        let fold = state.fold(index, clock, sys_micros, Basis::Time(None));
        state.forget_stream();
        let absolute = match fold {
            Some(fold) => state.apply(index, clock, sys_micros, fold),
            None => {
                let base = state.resumed_base(clock, sys_micros);
                state.commit(index, clock, sys_micros, base, 0, false)
            }
        };
        Some((absolute, true))
    }

    /// Reconstructs the system time at which the packet carrying this index and
    /// clock was received, or `None` before any packet has been accepted.
    ///
    /// The base is the one in effect at the packet's index per the recorded wrap
    /// points, so buffered packets resolve correctly even when read after the clock
    /// has wrapped again. Assumes the controller clock ticks at 1µs per unit.
    ///
    /// A controller that leaves `index` fixed records every base against the same
    /// index, so the clock is what separates them: within one base it only climbs,
    /// which bounds each base below by the clock first seen on it and bounds the
    /// newest one above by the newest clock accepted.
    #[cfg_vis(test, pub)]
    fn system_time_of(&self, index: u32, clock: u32) -> Option<SystemTime> {
        let state = self.state.lock();
        let offset = state.offset_micros?;
        let at_or_after =
            |first_index: u32| index.wrapping_sub(first_index) < Self::INDEX_HALF_RANGE;
        let base =
            state
                .wrap_points
                .back()
                .filter(|&&(first_index, first_clock, _)| {
                    at_or_after(first_index) && first_clock <= clock && clock <= state.last_clock
                })
                .or_else(|| {
                    state.wrap_points.iter().rev().skip(1).find(
                        |&&(first_index, first_clock, _)| {
                            at_or_after(first_index) && first_clock <= clock
                        },
                    )
                })
                .map(|&(_, _, b)| b)
                .or_else(|| {
                    let &(_, _, oldest) = state.wrap_points.front()?;
                    Some(oldest.saturating_sub(state.cycle.unwrap_or(Self::NOMINAL_CYCLE)))
                })?;
        let micros = base as i128 + clock as i128 + offset;
        let micros = u64::try_from(micros).ok()?;
        SystemTime::UNIX_EPOCH.checked_add(Duration::from_micros(micros))
    }
}

/// What a packet's absolute time is predicted from.
#[derive(Debug, Clone, Copy)]
enum Basis {
    /// The index distance times this period.
    Index(u64),
    /// The receive time elapsed, no less than the index distance times this
    /// period.
    Time(Option<u64>),
}

/// How a packet's clock reads against the stream: the base it belongs on, and
/// the cycle estimate that results.
#[derive(Debug, Clone, Copy)]
struct ClockFold {
    base: u64,
    cycle: Option<u64>,
    residual: u64,
    wrapped: bool,
}

impl StreamClockState {
    /// The per-index period, once two recent steps agree on it. A single
    /// sample could be a corrupted clock and is not enough to steer by.
    fn period(&self) -> Option<u64> {
        let median = Self::median_of(self.periods.iter().copied())?;
        let tolerance = (median / 64).max(1 << StreamClock::PERIOD_FRACTION_BITS);
        let agreeing = self
            .periods
            .iter()
            .filter(|&&p| p.abs_diff(median) <= tolerance)
            .count();
        (agreeing >= 2).then_some(median)
    }

    /// Any period seen at all, agreed on or not.
    fn any_period(&self) -> Option<u64> {
        Self::median_of(self.periods.iter().copied())
    }

    fn median_of(values: impl Iterator<Item = u64>) -> Option<u64> {
        let mut sorted: Vec<u64> = values.collect();
        if sorted.is_empty() {
            return None;
        }
        sorted.sort_unstable();
        Some(sorted[(sorted.len() - 1) / 2])
    }

    fn by_rate(period: u64, distance: u32) -> u64 {
        let micros = (period as u128 * distance as u128) >> StreamClock::PERIOD_FRACTION_BITS;
        u64::try_from(micros).unwrap_or(u64::MAX)
    }

    /// An index that claims far more packets than the receive times allow is a
    /// corrupted or restarted index, not the stream moving forward.
    fn index_outruns_time(&self, distance: u32, sys_micros: u64) -> bool {
        let (Some(period), Some(newest)) = (self.period(), self.references.back()) else {
            return false;
        };
        let elapsed = sys_micros.saturating_sub(newest.sys_micros);
        Self::by_rate(period, distance)
            > elapsed
                .saturating_mul(2)
                .saturating_add(StreamClock::MIN_CYCLE)
    }

    /// Where the references put a packet with this index and receive time, and
    /// how far from that its clock may sit.
    fn predict(&self, index: u32, sys_micros: u64, basis: Basis) -> Option<(u64, u64)> {
        let mut predictions: Vec<(u64, u64)> = self
            .references
            .iter()
            .map(|r| {
                let distance = index.wrapping_sub(r.index);
                let advance = match basis {
                    Basis::Index(period) => Self::by_rate(period, distance),
                    Basis::Time(floor) => {
                        let elapsed = sys_micros.saturating_sub(r.sys_micros);
                        elapsed.max(floor.map_or(0, |p| Self::by_rate(p, distance)))
                    }
                };
                (r.absolute.saturating_add(advance), r.residual)
            })
            .collect();
        let newest = self.references.back()?.absolute;
        let prediction = Self::robust_median(&mut predictions)?.max(newest);
        let advance = prediction - newest;
        let tolerance = match basis {
            Basis::Index(_) => StreamClock::INDEX_TOLERANCE.saturating_add(advance / 64),
            Basis::Time(_) => StreamClock::SYSTEM_TOLERANCE,
        };
        Some((prediction, tolerance))
    }

    /// Reads `clock` against the prediction: the same base, or a measured wrap
    /// onto a later one. `None` when it fits neither.
    fn fold(&self, index: u32, clock: u32, sys_micros: u64, basis: Basis) -> Option<ClockFold> {
        let (prediction, tolerance) = self.predict(index, sys_micros, basis)?;
        let here = self.base.checked_add(clock as u64)?;
        if here.abs_diff(prediction) <= tolerance {
            return Some(ClockFold {
                base: self.base,
                cycle: self.cycle,
                residual: here.abs_diff(prediction),
                wrapped: false,
            });
        }
        if here > prediction {
            return None;
        }
        let advance = prediction - here;
        let since_newest = prediction.saturating_sub(self.references.back()?.absolute);
        if advance < StreamClock::MIN_CYCLE / 2
            || clock as u64 > since_newest.saturating_add(tolerance)
        {
            return None;
        }
        let (cycle, residual) = match self.cycle {
            None => (advance, 0),
            Some(cycle) => {
                let cycles = advance.saturating_add(cycle / 2) / cycle;
                let whole = cycles.saturating_mul(cycle);
                if cycles >= 1 && advance.abs_diff(whole) <= tolerance.max(cycle / 16) {
                    let cycle = if cycles == 1 {
                        cycle.min(advance)
                    } else {
                        cycle
                    };
                    (cycle, advance.abs_diff(whole))
                } else if advance < cycle {
                    // The cycle measured before spanned several boundaries.
                    (advance, 0)
                } else {
                    return None;
                }
            }
        };
        Some(ClockFold {
            base: self.base.checked_add(advance)?,
            cycle: Some(cycle),
            residual,
            wrapped: true,
        })
    }

    /// Where the stream resumes when nothing it carries can be trusted: the
    /// newest packet's time plus the receive time since.
    fn resumed_base(&self, clock: u32, sys_micros: u64) -> u64 {
        let resumed = match self.references.back() {
            Some(newest) => newest
                .absolute
                .saturating_add(sys_micros.saturating_sub(newest.sys_micros)),
            None => self.base.saturating_add(self.last_clock as u64),
        };
        resumed.saturating_sub(clock as u64)
    }

    /// The median, with an even count's middle pair split by which value came
    /// from the reference the stream agreed with more closely when it arrived.
    fn robust_median<T: Ord + Copy>(values: &mut [(T, u64)]) -> Option<T> {
        values.sort_unstable();
        let n = values.len();
        if n == 0 {
            return None;
        }
        if n % 2 == 1 {
            return Some(values[n / 2].0);
        }
        let (lo, hi) = (values[n / 2 - 1], values[n / 2]);
        Some(if hi.1 < lo.1 { hi.0 } else { lo.0 })
    }

    fn forget_stream(&mut self) {
        self.references.clear();
        self.periods.clear();
    }

    fn apply(&mut self, index: u32, clock: u32, sys_micros: u64, fold: ClockFold) -> u64 {
        self.cycle = fold.cycle;
        self.commit(
            index,
            clock,
            sys_micros,
            fold.base,
            fold.residual,
            fold.wrapped,
        )
    }

    fn commit(
        &mut self,
        index: u32,
        clock: u32,
        sys_micros: u64,
        base: u64,
        residual: u64,
        wrapped: bool,
    ) -> u64 {
        let absolute = base.saturating_add(clock as u64);
        if let Some(newest) = self.references.back() {
            let distance = index.wrapping_sub(newest.index);
            if !wrapped && distance > 0 && absolute >= newest.absolute {
                let micros = (absolute - newest.absolute) as u128;
                let period = (micros << StreamClock::PERIOD_FRACTION_BITS) / distance as u128;
                self.periods
                    .push_back(u64::try_from(period).unwrap_or(u64::MAX));
                if self.periods.len() > StreamClock::PERIOD_SAMPLES {
                    self.periods.pop_front();
                }
            }
        }
        if base != self.base {
            self.references.clear();
        }
        self.base = base;
        if self.wrap_points.back().map(|&(_, _, b)| b) != Some(base) {
            self.wrap_points.push_back((index, clock, base));
            if self.wrap_points.len() > StreamClock::WRAP_HISTORY {
                self.wrap_points.pop_front();
            }
        }
        self.references.push_back(ClockReference {
            index,
            absolute,
            sys_micros,
            residual,
        });
        if self.references.len() > StreamClock::REFERENCES {
            self.references.pop_front();
        }
        self.last_index = Some(index);
        self.last_clock = clock;
        absolute
    }
}

/// A decoded HSPO datagram, as reported to a [`crate::TelemetrySink`].
///
/// HSPO is receive-only, so sinks use `()` as their TX type and `sent` never fires.
#[derive(Debug, Clone, Copy)]
pub enum HspoRxPacket {
    TcpCartesianPosition(TcpCartesianPositionPacket),
    JointAngles(JointAnglesPacket),
    Variables(VariablesPacket),
}

/// Shared sink observing one receiver's incoming HSPO packets.
pub type HspoTelemetry = Arc<dyn crate::TelemetrySink<(), HspoRxPacket>>;

#[derive(Debug)]
struct RobotSender {
    ip_of_interest: IpAddr,
    last_packet_time: Option<Instant>,
    connection_active: Arc<AtomicBool>,
    connection_timeout: Duration,
    tcp_tx: Sender<TcpCartesianPositionPacket>,
    joint_tx: Sender<JointAnglesPacket>,
    var_tx: Sender<VariablesPacket>,
    tcp_dropper: Receiver<TcpCartesianPositionPacket>,
    joint_dropper: Receiver<JointAnglesPacket>,
    var_dropper: Receiver<VariablesPacket>,
    tcp_clock: Arc<StreamClock>,
    joint_clock: Arc<StreamClock>,
    var_clock: Arc<StreamClock>,
    telemetry: Option<HspoTelemetry>,
}

impl RobotSender {
    /// Gates a freshly received packet by its per-stream index and folds its clock
    /// into the stream's shared wrap-corrected clock tracker. `sys_micros` is the
    /// receive time as micros since the Unix epoch, taken from `source`: the
    /// kernel rx timestamp when available, user-space receive time otherwise.
    ///
    /// Returns `false` if `index` is older than the newest already seen on `stream`,
    /// meaning the packet is reordered or stale and the caller must disregard it (not
    /// forward it to its channel). Each stream tracks its own highest index.
    fn accept_packet(
        &self,
        stream: HspoStream,
        index: u32,
        clock: u32,
        sys_micros: u64,
        source: Source,
    ) -> bool {
        let stream_clock = match stream {
            HspoStream::Tcp => &self.tcp_clock,
            HspoStream::Joint => &self.joint_clock,
            HspoStream::Variables => &self.var_clock,
        };
        stream_clock
            .accept_from(index, clock, sys_micros, source)
            .is_some()
    }
}

/// A channel for receiving HSPO packets of a specific type.
///
/// Wraps an internal bounded buffer and provides blocking, non-blocking, and drain operations.
#[derive(Debug)]
pub struct HspoChannel<T> {
    rx: Receiver<T>,
    clock: Arc<StreamClock>,
}

impl<T> Clone for HspoChannel<T> {
    fn clone(&self) -> Self {
        Self {
            rx: self.rx.clone(),
            clock: self.clock.clone(),
        }
    }
}

impl<T: HspoPacket> HspoChannel<T> {
    /// Returns the system time at which the broker received `packet`, reconstructed
    /// from the packet's index and controller clock using the stream's recorded
    /// wrap points and clock-to-system offset.
    ///
    /// Works for buffered packets read after the controller's 32-bit clock has
    /// wrapped again. Returns `None` if nothing has been received on this stream yet.
    pub fn received_at(&self, packet: &T) -> Option<SystemTime> {
        self.clock.system_time_of(packet.index(), packet.clock())
    }
}

impl<T> HspoChannel<T> {
    fn new(rx: Receiver<T>, clock: Arc<StreamClock>) -> Self {
        Self { rx, clock }
    }

    /// Blocks until a packet is received or the timeout elapses.
    pub fn wait_for(&self, timeout: Duration) -> Option<T> {
        self.rx.recv_timeout(timeout).ok()
    }

    /// Awaits the next packet. Resolves to `None` if the broker is destroyed.
    ///
    /// Buffered packets are yielded immediately; otherwise the future is woken
    /// when the broker thread delivers the next packet. Pair with your runtime's
    /// timeout combinator if a deadline is needed.
    pub async fn recv_async(&self) -> Option<T> {
        self.rx.recv_async().await.ok()
    }

    /// Returns the next buffered packet without blocking, or `None` if the buffer is empty.
    pub fn try_recv(&self) -> Option<T> {
        self.rx.try_recv().ok()
    }

    /// Drains and returns all buffered packets.
    pub fn recv_all(&self) -> Vec<T> {
        let mut packets = Vec::new();
        while let Ok(p) = self.rx.try_recv() {
            packets.push(p);
        }
        packets
    }

    /// Discards all buffered packets.
    pub fn clear(&self) {
        while self.rx.try_recv().is_ok() {}
    }

    pub(crate) fn clone_rx(&self) -> Receiver<T> {
        self.rx.clone()
    }
}

#[cfg(feature = "py")]
mod py_channel {
    use super::*;
    use pyo3::{prelude::*, pyclass, pymethods};

    #[derive(Debug)]
    enum InnerChannel {
        Tcp(HspoChannel<TcpCartesianPositionPacket>),
        Joint(HspoChannel<JointAnglesPacket>),
        Var(HspoChannel<VariablesPacket>),
    }

    /// A channel for receiving HSPO packets of a specific type.
    #[pyclass(name = "HspoChannel", generic)]
    #[derive(Debug)]
    pub struct PyHspoChannel {
        inner: InnerChannel,
    }

    impl PyHspoChannel {
        pub fn from_tcp(channel: &HspoChannel<TcpCartesianPositionPacket>) -> Self {
            Self {
                inner: InnerChannel::Tcp(channel.clone()),
            }
        }

        pub fn from_joint(channel: &HspoChannel<JointAnglesPacket>) -> Self {
            Self {
                inner: InnerChannel::Joint(channel.clone()),
            }
        }

        pub fn from_var(channel: &HspoChannel<VariablesPacket>) -> Self {
            Self {
                inner: InnerChannel::Var(channel.clone()),
            }
        }
    }

    macro_rules! dispatch_channel {
        ($self:expr, $method:ident $(, $arg:expr)*) => {
            match &$self.inner {
                InnerChannel::Tcp(ch) => ch.$method($($arg),*),
                InnerChannel::Joint(ch) => ch.$method($($arg),*),
                InnerChannel::Var(ch) => ch.$method($($arg),*),
            }
        };
    }

    #[pymethods]
    impl PyHspoChannel {
        /// Blocks until a packet is received or the timeout elapses.
        fn wait_for(&self, py: Python<'_>, timeout_secs: f64) -> Option<Py<PyAny>> {
            let timeout = Duration::from_secs_f64(timeout_secs);
            match &self.inner {
                InnerChannel::Tcp(ch) => py
                    .detach(|| ch.wait_for(timeout))
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
                InnerChannel::Joint(ch) => py
                    .detach(|| ch.wait_for(timeout))
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
                InnerChannel::Var(ch) => py
                    .detach(|| ch.wait_for(timeout))
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
            }
        }

        /// Returns the next buffered packet without blocking, or `None` if the buffer is empty.
        fn try_recv(&self, py: Python<'_>) -> Option<Py<PyAny>> {
            match &self.inner {
                InnerChannel::Tcp(ch) => ch
                    .try_recv()
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
                InnerChannel::Joint(ch) => ch
                    .try_recv()
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
                InnerChannel::Var(ch) => ch
                    .try_recv()
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
            }
        }

        /// Drains and returns all buffered packets.
        fn recv_all(&self, py: Python<'_>) -> Vec<Py<PyAny>> {
            match &self.inner {
                InnerChannel::Tcp(ch) => ch
                    .recv_all()
                    .into_iter()
                    .filter_map(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind()))
                    .collect(),
                InnerChannel::Joint(ch) => ch
                    .recv_all()
                    .into_iter()
                    .filter_map(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind()))
                    .collect(),
                InnerChannel::Var(ch) => ch
                    .recv_all()
                    .into_iter()
                    .filter_map(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind()))
                    .collect(),
            }
        }

        /// Discards all buffered packets.
        fn clear(&self) {
            dispatch_channel!(self, clear);
        }

        /// Returns the system time the broker received `packet` as seconds since
        /// the Unix epoch, or `None` if nothing has been received on this stream yet.
        fn received_at(&self, packet: &Bound<'_, PyAny>) -> PyResult<Option<f64>> {
            let received = match &self.inner {
                InnerChannel::Tcp(ch) => {
                    ch.received_at(&packet.extract::<TcpCartesianPositionPacket>()?)
                }
                InnerChannel::Joint(ch) => ch.received_at(&packet.extract::<JointAnglesPacket>()?),
                InnerChannel::Var(ch) => ch.received_at(&packet.extract::<VariablesPacket>()?),
            };
            Ok(received.map(|t| {
                t.duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap_or(Duration::ZERO)
                    .as_secs_f64()
            }))
        }
    }
}

/// Receives HSPO (High Speed Position Output) packets from a specific FANUC controller.
///
/// Created via [`initialize_broker`] followed by `try_new`. Packets are buffered internally
/// and can be consumed via the [`tcp`](Self::tcp), [`joint`](Self::joint), and [`var`](Self::var) channels.
#[cfg_attr(feature = "py", pyo3::pyclass)]
#[derive(Debug)]
pub struct HspoReceiver {
    connection_active: Arc<AtomicBool>,
    /// Channel for TCP cartesian position packets.
    pub tcp: HspoChannel<TcpCartesianPositionPacket>,
    /// Channel for joint angles packets.
    pub joint: HspoChannel<JointAnglesPacket>,
    /// Channel for variables packets.
    pub var: HspoChannel<VariablesPacket>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl HspoReceiver {
    #[cfg(on)]
    #[new]
    #[pyo3(signature=(ip_of_interest, packet_buffer_size=128, connection_timeout_secs=0.016))]
    pub fn new(
        ip_of_interest: pyo3::Bound<pyo3::PyAny>,
        packet_buffer_size: usize,
        connection_timeout_secs: f64,
    ) -> pyo3::PyResult<Self> {
        use pyo3::types::PyAnyMethods;
        let ip_of_interest: IpAddr = ip_of_interest.extract()?;
        let connection_timeout = Duration::from_secs_f64(connection_timeout_secs);
        if let Some(server) = HSPO_SERVER.lock().as_ref() {
            Ok(server.add_robot(ip_of_interest, packet_buffer_size, connection_timeout, None)?)
        } else {
            Err(pyo3::exceptions::PyRuntimeError::new_err(
                "HSPO server not initialized. Please initialize the server before creating a driver.",
            ))
        }
    }

    /// Creates a new receiver for the given robot IP address with the specified packet buffer size.
    ///
    /// The HSPO broker must be initialized with [`initialize_broker`] before calling this method.
    #[cfg(off)]
    pub fn try_new<T: Into<IpAddr>>(
        ip_of_interest: T,
        packet_buffer_size: usize,
        connection_timeout: Duration,
    ) -> Result<Self, HspoBrokerNotInitializedError> {
        Self::try_new_inner(
            ip_of_interest.into(),
            packet_buffer_size,
            connection_timeout,
            None,
        )
    }

    /// Like [`try_new`](Self::try_new), with a telemetry sink observing every
    /// decoded packet from this robot on the broker thread; it is moved into
    /// the broker on registration. Kernel receive timestamps are used when available.
    #[cfg(off)]
    pub fn try_new_with_telemetry<T: Into<IpAddr>, S: crate::TelemetrySink<(), HspoRxPacket>>(
        ip_of_interest: T,
        packet_buffer_size: usize,
        connection_timeout: Duration,
        telemetry: S,
    ) -> Result<Self, HspoBrokerNotInitializedError> {
        Self::try_new_inner(
            ip_of_interest.into(),
            packet_buffer_size,
            connection_timeout,
            Some(Arc::new(telemetry)),
        )
    }

    #[cfg(off)]
    fn try_new_inner(
        ip_of_interest: IpAddr,
        packet_buffer_size: usize,
        connection_timeout: Duration,
        telemetry: Option<HspoTelemetry>,
    ) -> Result<Self, HspoBrokerNotInitializedError> {
        if let Some(server) = HSPO_SERVER.lock().as_ref() {
            server.add_robot(
                ip_of_interest,
                packet_buffer_size,
                connection_timeout,
                telemetry,
            )
        } else {
            Err(HspoBrokerNotInitializedError)
        }
    }

    /// Returns `true` if a packet has been received from this robot within
    /// its connection timeout.
    pub fn is_connected(&self) -> bool {
        self.connection_active.load(Ordering::Relaxed)
    }

    /// Returns the TCP cartesian position channel.
    #[cfg(on)]
    #[getter]
    pub fn tcp(&self) -> py_channel::PyHspoChannel {
        py_channel::PyHspoChannel::from_tcp(&self.tcp)
    }

    /// Returns the joint angles channel.
    #[cfg(on)]
    #[getter]
    pub fn joint(&self) -> py_channel::PyHspoChannel {
        py_channel::PyHspoChannel::from_joint(&self.joint)
    }

    /// Returns the variables channel.
    #[cfg(on)]
    #[getter]
    pub fn var(&self) -> py_channel::PyHspoChannel {
        py_channel::PyHspoChannel::from_var(&self.var)
    }
}

struct HspoBroker {
    robot_appender: Sender<RobotSender>,
    tuning: TuningReport,
    waker: Arc<Waker>,
    err_flag: Arc<AtomicBool>,
    kill_switch: Arc<AtomicBool>,
    _thread_handle: std::thread::JoinHandle<()>,
}

/// The broker's socket and poller, set up on the caller so a bind failure
/// surfaces from [`initialize_broker`] and the waker exists before the thread.
struct BrokerIo {
    socket: Timestamped<MioUdpSocket>,
    tuning: OptionsReport<SocketOption>,
    poll: Poll,
    waker: Arc<Waker>,
}

impl BrokerIo {
    fn new(listen_on: SocketAddr, options: &[SocketOption]) -> Result<Self, HspoBrokerError> {
        let poll = Poll::new().map_err(|_| GeneralThreadError::FailedToCreatePoll)?;
        let (socket, report) =
            tuning::bind_udp(DRIVER, SocketRole::UdpStreamRx, listen_on, options)?;
        socket.set_nonblocking(true)?;
        let mut socket =
            Timestamped::with_config(MioUdpSocket::from_std(socket), Config::kernel_only());
        if socket.source() < Source::Kernel {
            tracing::debug!("HSPO kernel rx timestamps unavailable");
        }
        poll.registry()
            .register(&mut socket, TOK_SOCKET, Interest::READABLE)
            .map_err(|_| GeneralThreadError::FailedSocketRegistry)?;
        let waker = Arc::new(
            Waker::new(poll.registry(), TOK_WAKER)
                .map_err(|_| GeneralThreadError::FailedWakerCreation)?,
        );
        Ok(Self {
            socket,
            tuning: OptionsReport::from(&report),
            poll,
            waker,
        })
    }
}

impl From<GeneralThreadError> for HspoBrokerError {
    fn from(e: GeneralThreadError) -> Self {
        match e {
            GeneralThreadError::Io(e) => HspoBrokerError::Io(e),
            other => HspoBrokerError::Io(io::Error::other(other.to_string())),
        }
    }
}

fn broker_runtime(
    io: BrokerIo,
    thread: Vec<ThreadOption>,
    started: Sender<io::Result<OptionsReport<ThreadOption>>>,
    robot_receiver: Receiver<RobotSender>,
    thread_kill_switch: Arc<AtomicBool>,
) {
    let _tuning = match tuning::apply_thread(DRIVER, ThreadRole::Stream, &thread) {
        Ok(report) => {
            let _ = started.send(Ok(OptionsReport::from(&report)));
            report
        }
        Err(e) => {
            let _ = started.send(Err(e));
            return;
        }
    };

    let BrokerIo {
        socket, mut poll, ..
    } = io;
    let mut socket_drops = 0;
    let mut events = Events::with_capacity(256);

    let mut robot_senders: HashMap<IpAddr, Vec<RobotSender>> = HashMap::new();
    let mut shortest_timeout = Duration::from_millis(256);

    let mut buf = [0u8; 2048];
    let config = bincode::config::standard()
        .with_fixed_int_encoding()
        .with_big_endian();

    loop {
        let _ = poll.poll(&mut events, Some(shortest_timeout));

        if thread_kill_switch.load(Ordering::Relaxed) {
            break;
        }

        // Drain any new RobotSender registrations.
        while let Ok(rs) = robot_receiver.try_recv() {
            if rs.connection_timeout < shortest_timeout {
                shortest_timeout = rs.connection_timeout;
            }
            if let Some(sink) = &rs.telemetry {
                sink.warmup();
            }
            robot_senders.entry(rs.ip_of_interest).or_default().push(rs);
        }

        // Process all readable events.
        for ev in events.iter() {
            if ev.token() != TOK_SOCKET || !ev.is_readable() {
                continue;
            }

            // Read all pending datagrams.
            loop {
                match socket.recv_from(&mut buf) {
                    Ok(Received {
                        len: n,
                        from,
                        timestamp,
                        drops,
                        ..
                    }) => {
                        if let Some(drops) = drops {
                            if drops > socket_drops {
                                tracing::warn!(
                                    dropped = drops - socket_drops,
                                    total = drops,
                                    "HSPO socket dropped datagrams for lack of receive buffer"
                                );
                            }
                            socket_drops = drops;
                        }
                        if n == 0 {
                            continue;
                        }
                        let src_ip = from.ip();

                        // Fast path: if nobody cares about this IP, skip parsing.
                        let Some(listeners) = robot_senders.get_mut(&src_ip) else {
                            continue;
                        };

                        // Determine packet type. 'typ' is at offset 12 (u32,u32,u32 -> 12 bytes).
                        let pkt_type = PacketType::from_bytes(&buf[..n], 12);
                        let now = Instant::now();
                        let sys_time = timestamp.time;
                        let sys_micros: u64 = sys_time
                            .duration_since(SystemTime::UNIX_EPOCH)
                            .unwrap_or(Duration::ZERO)
                            .as_micros()
                            .try_into()
                            .unwrap_or(u64::MAX);

                        match pkt_type {
                            PacketType::TcpCartesianPosition => {
                                if let Ok((p, _n)) =
                                    bincode::decode_from_slice::<TcpCartesianPositionPacket, _>(
                                        &buf[..n],
                                        config,
                                    )
                                {
                                    for rs in listeners.iter_mut() {
                                        rs.last_packet_time = Some(now);
                                        rs.connection_active.store(true, Ordering::Relaxed);
                                        if let Some(sink) = &rs.telemetry {
                                            sink.received(
                                                &HspoRxPacket::TcpCartesianPosition(p),
                                                sys_time,
                                            );
                                        }
                                        if !rs.accept_packet(
                                            HspoStream::Tcp,
                                            p.index,
                                            p.clock,
                                            sys_micros,
                                            timestamp.source,
                                        ) {
                                            continue;
                                        }
                                        match rs.tcp_tx.try_send(p) {
                                            Ok(_) => {}
                                            Err(TrySendError::Full(fb_p)) => {
                                                let _ = rs.tcp_dropper.try_recv();
                                                let _ = rs.tcp_tx.try_send(fb_p);
                                            }
                                            Err(_) => {}
                                        }
                                    }
                                }
                            }
                            PacketType::JointAngles => {
                                if let Ok((p, _n)) =
                                    bincode::decode_from_slice::<JointAnglesPacket, _>(
                                        &buf[..n],
                                        config,
                                    )
                                {
                                    for rs in listeners.iter_mut() {
                                        rs.last_packet_time = Some(now);
                                        rs.connection_active.store(true, Ordering::Relaxed);
                                        if let Some(sink) = &rs.telemetry {
                                            sink.received(&HspoRxPacket::JointAngles(p), sys_time);
                                        }
                                        if !rs.accept_packet(
                                            HspoStream::Joint,
                                            p.index,
                                            p.clock,
                                            sys_micros,
                                            timestamp.source,
                                        ) {
                                            continue;
                                        }
                                        match rs.joint_tx.try_send(p) {
                                            Ok(_) => {}
                                            Err(TrySendError::Full(fb_p)) => {
                                                let _ = rs.joint_dropper.try_recv();
                                                let _ = rs.joint_tx.try_send(fb_p);
                                            }
                                            Err(_) => {}
                                        }
                                    }
                                }
                            }
                            PacketType::Variables => {
                                if let Ok((p, _n)) = bincode::decode_from_slice::<VariablesPacket, _>(
                                    &buf[..n],
                                    config,
                                ) {
                                    for rs in listeners.iter_mut() {
                                        rs.last_packet_time = Some(now);
                                        rs.connection_active.store(true, Ordering::Relaxed);
                                        if let Some(sink) = &rs.telemetry {
                                            sink.received(&HspoRxPacket::Variables(p), sys_time);
                                        }
                                        if !rs.accept_packet(
                                            HspoStream::Variables,
                                            p.index,
                                            p.clock,
                                            sys_micros,
                                            timestamp.source,
                                        ) {
                                            continue;
                                        }
                                        match rs.var_tx.try_send(p) {
                                            Ok(_) => {}
                                            Err(TrySendError::Full(fb_p)) => {
                                                let _ = rs.var_dropper.try_recv();
                                                let _ = rs.var_tx.try_send(fb_p);
                                            }
                                            Err(_) => {}
                                        }
                                    }
                                }
                            }
                            PacketType::Unknown => {
                                // Ignore unknown packet types.
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        // No more datagrams right now.
                        break;
                    }
                    Err(e) => {
                        tracing::error!(error = %e, "HSPO broker socket recv error");
                        break;
                    }
                }
            }
        }

        // Update connection-active flags based on last packet timestamp (10ms timeout).
        let now = Instant::now();
        for listeners in robot_senders.values_mut() {
            for rs in listeners.iter_mut() {
                let active = rs
                    .last_packet_time
                    .map(|t| now.duration_since(t) <= rs.connection_timeout)
                    .unwrap_or(false);
                rs.connection_active.store(active, Ordering::Relaxed);
                if !active {
                    // Optional: avoid monotonic growth of Some(…) when inactive.
                    rs.last_packet_time = None;
                }
            }
            // remove listeners that internally dropped all receivers
            listeners.retain(|rs| {
                !(rs.tcp_tx.is_disconnected()
                    && rs.joint_tx.is_disconnected()
                    && rs.var_tx.is_disconnected())
            });
        }
    }
}

impl HspoBroker {
    fn add_robot(
        &self,
        ip_of_interest: IpAddr,
        packet_buffer_size: usize,
        connection_timeout: Duration,
        telemetry: Option<HspoTelemetry>,
    ) -> Result<HspoReceiver, HspoBrokerNotInitializedError> {
        let (tcp_tx, tcp_rx) = bounded::<TcpCartesianPositionPacket>(packet_buffer_size);
        let (joint_tx, joint_rx) = bounded::<JointAnglesPacket>(packet_buffer_size);
        let (var_tx, var_rx) = bounded::<VariablesPacket>(packet_buffer_size);
        let connection_active = Arc::new(AtomicBool::new(false));

        let tcp = HspoChannel::new(tcp_rx, Arc::new(StreamClock::default()));
        let joint = HspoChannel::new(joint_rx, Arc::new(StreamClock::default()));
        let var = HspoChannel::new(var_rx, Arc::new(StreamClock::default()));

        let robot_sender = RobotSender {
            ip_of_interest,
            last_packet_time: None,
            connection_active: connection_active.clone(),
            connection_timeout,
            tcp_tx,
            joint_tx,
            var_tx,
            tcp_dropper: tcp.clone_rx(),
            joint_dropper: joint.clone_rx(),
            var_dropper: var.clone_rx(),
            tcp_clock: tcp.clock.clone(),
            joint_clock: joint.clock.clone(),
            var_clock: var.clock.clone(),
            telemetry,
        };

        tracing::info!(
            ip = %ip_of_interest,
            buffer_size = packet_buffer_size,
            "HSPO registering receiver"
        );
        self.robot_appender
            .send(robot_sender)
            .map_err(|_| HspoBrokerNotInitializedError)?;
        let _ = self.waker.wake();

        Ok(HspoReceiver {
            connection_active,
            tcp,
            joint,
            var,
        })
    }

    fn create(
        listen_on: SocketAddr,
        thread: &[ThreadOption],
        socket: &[SocketOption],
    ) -> Result<Self, HspoBrokerError> {
        tuning::check_thread(DRIVER, ThreadRole::Stream, thread)?;
        tuning::check_socket(DRIVER, SocketRole::UdpStreamRx, socket)?;
        let local_kill_switch = Arc::new(AtomicBool::new(false));
        let local_err_flag = Arc::new(AtomicBool::new(false));
        let (robot_appender, robot_receiver) = unbounded::<RobotSender>();
        let io = BrokerIo::new(listen_on, socket).inspect_err(|e| {
            tracing::error!(error = %e, "HSPO broker setup failed");
        })?;
        let waker = io.waker.clone();
        let socket_report = io.tuning.clone();
        let (started_tx, started_rx) = bounded(1);

        let thread_kill_switch = local_kill_switch.clone();
        let thread_options = thread.to_vec();
        let worker = thread::Builder::new()
            .name("hspo_server".to_string())
            .spawn(move || {
                broker_runtime(
                    io,
                    thread_options,
                    started_tx,
                    robot_receiver,
                    thread_kill_switch,
                );
            })?;
        let started = started_rx
            .recv()
            .unwrap_or_else(|_| Err(io::Error::other("HSPO broker thread exited during startup")));
        let thread_report = match started {
            Ok(report) => report,
            Err(e) => {
                let _ = worker.join();
                return Err(e.into());
            }
        };

        Ok(HspoBroker {
            robot_appender,
            tuning: TuningReport {
                thread: thread_report,
                socket: socket_report,
            },
            waker,
            kill_switch: local_kill_switch,
            err_flag: local_err_flag,
            _thread_handle: worker,
        })
    }
}

/// Initializes the global HSPO broker, binding a socket to `listen_on` and spawning a background listener thread.
///
/// This must be called before creating any [`HspoReceiver`]. Calling it again after initialization is a no-op.
///
/// `thread` is applied by the broker thread to itself before it receives
/// anything. That thread is a latency-sensitive receive loop with no period
/// of its own, so every [`ThreadOption`] is accepted except
/// `MacOsTimeConstraint`, which reserves a computation slice per period.
/// Process-wide settings (memory locking, `cpu_dma_latency`, Windows priority
/// class and timer resolution) are the application's to make with
/// [`ProcessOption::apply_all`](fast_talker::options::ProcessOption::apply_all).
///
/// `socket` is applied to the receive socket before it is bound:
/// `RecvBuffer`, `BindDevice`, `LinuxBusyPoll`, `LinuxPreferBusyPoll`,
/// `LinuxBusyPollBudget`, `WinCpuAffinity`. `SendBuffer`, `DontFragment`,
/// `Dscp` and `LinuxPriority` are refused: they only shape traffic, and this
/// socket sends none.
///
/// Options for another platform, or that this platform cannot do, are
/// skipped with a warning, as are options the platform applied with a
/// different value (a receive buffer capped by `net.core.rmem_max`, say);
/// [`broker_tuning_report`] lists them.
///
/// # Errors
/// [`HspoBrokerError::InvalidOption`] for an option the broker does not
/// accept; [`HspoBrokerError::Io`] if binding the socket, applying an
/// option, or spawning the thread fails.
#[cfg(not(feature = "py"))]
pub fn initialize_broker(
    listen_on: SocketAddr,
    thread: &[ThreadOption],
    socket: &[SocketOption],
) -> Result<(), HspoBrokerError> {
    let mut guard = HSPO_SERVER.lock();
    if guard.is_none() {
        tracing::info!(addr = %listen_on, "Initializing HSPO broker");
        let server = HspoBroker::create(listen_on, thread, socket)?;
        *guard = Some(server);
        tracing::info!("HSPO broker initialized");
    }
    Ok(())
}

/// Initializes the global HSPO broker, binding a socket to `listen_on` and spawning a background listener thread.
///
/// This must be called before creating any [`HspoReceiver`]. Calling it again after initialization is a no-op.
/// `thread` and `socket` are as for the Rust `initialize_broker`.
#[cfg(feature = "py")]
#[pyo3::pyfunction]
#[pyo3(signature=(listen_on, thread=None, socket=None))]
pub fn initialize_broker(
    listen_on: String,
    thread: Option<fast_talker::py::ThreadOptions>,
    socket: Option<fast_talker::py::SocketOptions>,
) -> pyo3::PyResult<()> {
    let listen_on: SocketAddr = listen_on.parse().map_err(|_| {
        pyo3::exceptions::PyValueError::new_err("Invalid SocketAddr format for listen_on")
    })?;
    let mut guard = HSPO_SERVER.lock();
    if guard.is_none() {
        tracing::info!(addr = %listen_on, "Initializing HSPO broker");
        let server = HspoBroker::create(
            listen_on,
            &thread.unwrap_or_default(),
            &socket.unwrap_or_default(),
        )?;
        *guard = Some(server);
        tracing::info!("HSPO broker initialized");
    }
    Ok(())
}

/// What the running broker's thread and socket options did, or `None` when
/// no broker is running.
pub fn broker_tuning_report() -> Option<TuningReport> {
    HSPO_SERVER.lock().as_ref().map(|b| b.tuning.clone())
}

/// What the running broker's thread and socket options did, as
/// `{"thread": report, "socket": report}`, or `None` when no broker is running.
#[cfg(feature = "py")]
#[pyo3::pyfunction]
#[pyo3(name = "broker_tuning_report")]
pub fn py_broker_tuning_report(py: pyo3::Python<'_>) -> pyo3::PyResult<pyo3::Py<pyo3::PyAny>> {
    use pyo3::IntoPyObjectExt;
    broker_tuning_report().as_ref().into_py_any(py)
}

/// Shuts down the global HSPO broker
///
/// If `wait_for_thread` is `true`, this will block until the broker thread has fully exited.
#[cfg(feature = "py")]
#[pyo3::pyfunction]
#[pyo3(name = "destroy_broker", signature=(wait_for_thread=true))]
pub fn py_destroy_broker(py: pyo3::Python<'_>, wait_for_thread: bool) {
    py.detach(|| destroy_broker(wait_for_thread));
}

/// Shuts down the global HSPO broker
///
/// If `wait_for_thread` is `true`, this will block until the broker thread has fully exited.
pub fn destroy_broker(wait_for_thread: bool) {
    let mut guard = HSPO_SERVER.lock();
    if let Some(broker) = guard.take() {
        tracing::info!("Destroying HSPO broker");
        broker.kill_switch.store(true, Ordering::Relaxed);
        let _ = broker.waker.wake();
        if wait_for_thread {
            match broker._thread_handle.join() {
                Ok(()) => tracing::info!("HSPO broker thread exited cleanly"),
                Err(e) => tracing::error!(error = ?e, "HSPO broker thread panicked"),
            }
        }
    }
}

/// Checks if the HSPO broker thread has encountered an error.
/// If this returns `true`, the broker is likely non-functional and should be destroyed and re-initialized.
#[cfg_attr(feature = "py", pyo3::pyfunction)]
pub fn has_broker_errored() -> bool {
    if let Some(broker) = HSPO_SERVER.lock().as_ref() {
        broker.err_flag.load(Ordering::Relaxed)
    } else {
        false
    }
}

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register_child_module(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        let child_module = PyModule::new(parent_module.py(), "hspo")?;
        child_module.add_class::<HspoReceiver>()?;
        child_module.add_class::<py_channel::PyHspoChannel>()?;
        child_module.add_function(wrap_pyfunction!(initialize_broker, &child_module)?)?;
        child_module.add_function(wrap_pyfunction!(py_destroy_broker, &child_module)?)?;
        child_module.add_function(wrap_pyfunction!(has_broker_errored, &child_module)?)?;
        child_module.add_function(wrap_pyfunction!(py_broker_tuning_report, &child_module)?)?;
        child_module.add_class::<TcpCartesianPositionPacket>()?;
        child_module.add_class::<JointAnglesPacket>()?;
        child_module.add_class::<VariablesPacket>()?;

        parent_module.add_submodule(&child_module)
    }
}
