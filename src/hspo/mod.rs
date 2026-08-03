mod rx_timestamp;
#[cfg(test)]
mod test;

use cfg_vis::{cfg_vis, cfg_vis_fields};
use parking_lot::Mutex;
use snare::thread;
use std::{
    collections::{HashMap, VecDeque},
    net::{IpAddr, SocketAddr},
    sync::{
        Arc, LazyLock,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, SystemTime},
};

use crate::{
    joints::{JointFormat, JointTemplate},
    thread_util::{GeneralThreadError, ThreadConfig},
    time_util::host_now,
};
use bincode::{Decode, Encode};
use cfg_mixin::cfg_mixin;
use flume::{Receiver, Sender, TrySendError, bounded, unbounded};
use serde::Serialize;
use snare::mio::{Events, Interest, Poll, Token, Waker, net::UdpSocket as MioUdpSocket};

const TOK_SOCKET: Token = Token(0);
const TOK_WAKER: Token = Token(1);

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
    /// Controller clock at send time (wrapping 32-bit, 1µs per unit).
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
/// The broker folds each accepted packet's wrapping 32-bit clock into a cumulative
/// value, recording at which packet index each wrap was first seen and the offset
/// between the cumulative clock and system time. [`system_time_of`](Self::system_time_of)
/// reconstructs the receive time of a buffered packet from that record.
#[derive(Debug, Default)]
#[cfg_vis(test, pub)]
struct StreamClock {
    state: Mutex<StreamClockState>,
}

#[derive(Debug, Default)]
struct StreamClockState {
    last_index: Option<u32>,
    last_clock: u32,
    wraps: u64,
    /// `(first packet index seen at this wrap count, wrap count)`, newest last.
    wrap_points: VecDeque<(u32, u64)>,
    /// System micros minus cumulative clock micros, from the newest accepted packet.
    offset_micros: Option<i64>,
}

impl StreamClock {
    const SPAN: u64 = u32::MAX as u64 + 1;
    const WRAP_HISTORY: usize = 32;

    /// Gates a packet by index and folds its clock into the cumulative value,
    /// recording wrap points and the clock-to-system offset as it goes.
    ///
    /// Returns `None` when `index` is older than the newest already seen on this
    /// stream — a reordered or stale datagram the caller must disregard. Otherwise
    /// returns the absolute cumulative clock `wraps * 2^32 + clock`.
    ///
    /// Because out-of-order packets are gated out here, the accepted samples are in
    /// order, so a backward clock step on a strictly-newer packet is an unambiguous
    /// 32-bit wrap. The boundary check (previous value near the top of the range, new
    /// value near the bottom) covers streams whose index does not advance between
    /// packets, e.g. duplicates or controllers that leave `index` fixed.
    #[cfg_vis(test, pub)]
    fn accept(&self, index: u32, clock: u32, sys_micros: u64) -> Option<u64> {
        const WRAP_GUARD: u32 = u32::MAX / 4;
        let mut state = self.state.lock();
        if let Some(last_index) = state.last_index {
            if index < last_index {
                return None;
            }
            let crossed_boundary = state.last_clock > u32::MAX - WRAP_GUARD && clock < WRAP_GUARD;
            if clock < state.last_clock && (index > last_index || crossed_boundary) {
                state.wraps += 1;
            }
        }
        if state.wrap_points.back().map(|&(_, w)| w) != Some(state.wraps) {
            let wraps = state.wraps;
            state.wrap_points.push_back((index, wraps));
            if state.wrap_points.len() > Self::WRAP_HISTORY {
                state.wrap_points.pop_front();
            }
        }
        state.last_index = Some(index);
        state.last_clock = clock;
        let absolute = state.wraps * Self::SPAN + clock as u64;
        state.offset_micros = Some(sys_micros as i64 - absolute as i64);
        Some(absolute)
    }

    /// Reconstructs the system time at which the packet carrying this index and
    /// clock was received, or `None` before any packet has been accepted.
    ///
    /// The wrap count is the one in effect at the packet's index per the recorded
    /// wrap points, so buffered packets resolve correctly even when read after the
    /// clock has wrapped again. Assumes the controller clock ticks at 1µs per unit.
    #[cfg_vis(test, pub)]
    fn system_time_of(&self, index: u32, clock: u32) -> Option<SystemTime> {
        let state = self.state.lock();
        let offset = state.offset_micros?;
        let wraps = state
            .wrap_points
            .iter()
            .rev()
            .find(|&&(first_index, _)| first_index <= index)
            .map(|&(_, w)| w)
            .or_else(|| state.wrap_points.front().map(|&(_, w)| w.saturating_sub(1)))?;
        let micros = (wraps * Self::SPAN + clock as u64) as i128 + offset as i128;
        u64::try_from(micros)
            .ok()
            .map(|m| SystemTime::UNIX_EPOCH + Duration::from_micros(m))
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
    last_packet_time: Option<snare::time::Instant>,
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
    /// receive time as micros since the Unix epoch — the kernel rx timestamp when
    /// available, user-space receive time otherwise.
    ///
    /// Returns `false` if `index` is older than the newest already seen on `stream`,
    /// meaning the packet is reordered or stale and the caller must disregard it (not
    /// forward it to its channel). Each stream tracks its own highest index.
    fn accept_packet(&self, stream: HspoStream, index: u32, clock: u32, sys_micros: u64) -> bool {
        let stream_clock = match stream {
            HspoStream::Tcp => &self.tcp_clock,
            HspoStream::Joint => &self.joint_clock,
            HspoStream::Variables => &self.var_clock,
        };
        stream_clock.accept(index, clock, sys_micros).is_some()
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
    #[cfg(feature = "async")]
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
                InnerChannel::Tcp(ch) => ch
                    .wait_for(timeout)
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
                InnerChannel::Joint(ch) => ch
                    .wait_for(timeout)
                    .and_then(|v| Bound::new(py, v).ok().map(|b| b.into_any().unbind())),
                InnerChannel::Var(ch) => ch
                    .wait_for(timeout)
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

    /// Returns `true` if a packet has been received from this robot recently.
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
    waker: Arc<Waker>,
    err_flag: Arc<AtomicBool>,
    kill_switch: Arc<AtomicBool>,
    _thread_handle: std::thread::JoinHandle<()>,
}

fn broker_runtime(
    listen_on: SocketAddr,
    thread_config: Option<ThreadConfig>,
    robot_receiver: Receiver<RobotSender>,
    thread_kill_switch: Arc<AtomicBool>,
    waker_tx: Sender<Arc<Waker>>,
) -> Result<(), GeneralThreadError> {
    if let Some(thread_config) = thread_config {
        thread_config.configure_this_thread_print_failure();
    }

    let mut poll = Poll::new().map_err(|_| GeneralThreadError::FailedToCreatePoll)?;
    let mut events = Events::with_capacity(256);
    let mut socket =
        MioUdpSocket::bind(listen_on).map_err(|_| GeneralThreadError::FailedSocketBinding)?;
    #[cfg(test)]
    {
        let _ = socket.set_nonblocking(true);
    }
    let kernel_ts = match rx_timestamp::enable_rx_timestamping(&socket) {
        Ok(()) => true,
        Err(e) => {
            tracing::debug!(error = %e, "HSPO kernel rx timestamps unavailable");
            false
        }
    };
    poll.registry()
        .register(&mut socket, TOK_SOCKET, Interest::READABLE)
        .map_err(|_| GeneralThreadError::FailedSocketRegistry)?;
    let waker = Arc::new(
        Waker::new(poll.registry(), TOK_WAKER)
            .map_err(|_| GeneralThreadError::FailedWakerCreation)?,
    );
    waker_tx.send(waker)?;

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
                let received = if kernel_ts {
                    rx_timestamp::recv_from_timestamped(&socket, &mut buf)
                } else {
                    socket.recv_from(&mut buf).map(|(n, addr)| (n, addr, None))
                };
                match received {
                    Ok((n, addr, rx_ts)) => {
                        if n == 0 {
                            continue;
                        }
                        let src_ip = addr.ip();

                        // Fast path: if nobody cares about this IP, skip parsing.
                        let Some(listeners) = robot_senders.get_mut(&src_ip) else {
                            continue;
                        };

                        // Determine packet type. 'typ' is at offset 12 (u32,u32,u32 -> 12 bytes).
                        let pkt_type = PacketType::from_bytes(&buf[..n], 12);
                        let now = snare::time::Instant::now();
                        let sys_time = rx_ts.unwrap_or_else(host_now);
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
        let now = snare::time::Instant::now();
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
    Ok(())
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
        thread_config: Option<ThreadConfig>,
    ) -> Result<Self, HspoBrokerNotInitializedError> {
        let local_kill_switch = Arc::new(AtomicBool::new(false));
        let local_err_flag = Arc::new(AtomicBool::new(false));
        let (robot_appender, robot_receiver) = unbounded::<RobotSender>();
        let (waker_tx, waker_rx) = bounded::<Arc<Waker>>(1);

        let thread_kill_switch = local_kill_switch.clone();
        let thread_err_flag = local_err_flag.clone();
        let _thread_handle = thread::Builder::new()
            .name("hspo_server".to_string())
            .spawn(move || {
                if let Err(e) = broker_runtime(
                    listen_on,
                    thread_config,
                    robot_receiver,
                    thread_kill_switch,
                    waker_tx,
                ) {
                    tracing::error!(error = %e, "HSPO broker thread exited with error");
                    thread_err_flag.store(true, Ordering::Relaxed);
                }
            })
            .map_err(|_| HspoBrokerNotInitializedError)?;

        let waker = waker_rx.recv().map_err(|_| HspoBrokerNotInitializedError)?;

        Ok(HspoBroker {
            robot_appender,
            waker,
            kill_switch: local_kill_switch,
            err_flag: local_err_flag,
            _thread_handle,
        })
    }
}

/// Initializes the global HSPO broker, binding a socket to `listen_on` and spawning a background listener thread.
///
/// This must be called before creating any [`HspoReceiver`]. Calling it again after initialization is a no-op.
#[cfg(not(feature = "py"))]
pub fn initialize_broker(
    listen_on: SocketAddr,
    thread_config: Option<ThreadConfig>,
) -> Result<(), HspoBrokerNotInitializedError> {
    let mut guard = HSPO_SERVER.lock();
    if guard.is_none() {
        tracing::info!(addr = %listen_on, "Initializing HSPO broker");
        let server = HspoBroker::create(listen_on, thread_config)?;
        *guard = Some(server);
        tracing::info!("HSPO broker initialized");
    }
    Ok(())
}

/// Initializes the global HSPO broker, binding a socket to `listen_on` and spawning a background listener thread.
///
/// This must be called before creating any [`HspoReceiver`]. Calling it again after initialization is a no-op.
#[cfg(feature = "py")]
#[pyo3::pyfunction]
#[pyo3(signature=(listen_on, thread_config=None))]
pub fn initialize_broker(
    listen_on: String,
    thread_config: Option<ThreadConfig>,
) -> pyo3::PyResult<()> {
    let listen_on: SocketAddr = listen_on.parse().map_err(|_| {
        pyo3::exceptions::PyValueError::new_err("Invalid SocketAddr format for listen_on")
    })?;
    let mut guard = HSPO_SERVER.lock();
    if guard.is_none() {
        tracing::info!(addr = %listen_on, "Initializing HSPO broker");
        let server = HspoBroker::create(listen_on, thread_config)?;
        *guard = Some(server);
        tracing::info!("HSPO broker initialized");
    }
    Ok(())
}

/// Shuts down the global HSPO broker
///
/// If `wait_for_thread` is `true`, this will block until the broker thread has fully exited.
#[cfg_attr(feature = "py", pyo3::pyfunction)]
#[cfg_attr(feature = "py", pyo3(signature=(wait_for_thread=true)))]
pub fn destroy_broker(wait_for_thread: bool) {
    let mut guard = HSPO_SERVER.lock();
    if let Some(broker) = guard.take() {
        tracing::info!("Destroying HSPO broker");
        broker.kill_switch.store(true, Ordering::Relaxed);
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
        child_module.add_function(wrap_pyfunction!(destroy_broker, &child_module)?)?;
        child_module.add_function(wrap_pyfunction!(has_broker_errored, &child_module)?)?;
        child_module.add_class::<TcpCartesianPositionPacket>()?;
        child_module.add_class::<JointAnglesPacket>()?;
        child_module.add_class::<VariablesPacket>()?;

        parent_module.add_submodule(&child_module)
    }
}
