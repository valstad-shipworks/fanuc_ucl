#[cfg(test)]
mod test;

use cfg_vis::{cfg_vis, cfg_vis_fields};
use parking_lot::Mutex;
use std::{
    collections::HashMap,
    net::{IpAddr, SocketAddr},
    sync::{
        Arc, LazyLock,
        atomic::{AtomicBool, AtomicU32, Ordering},
    },
    time::{Duration, SystemTime},
};
use snare::thread;

use crate::{
    joints::{JointFormat, JointTemplate},
    thread_util::{GeneralThreadError, ThreadConfig},
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

#[derive(Debug, Clone, Copy, PartialEq)]
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

#[derive(Debug)]
#[cfg_vis(test, pub)]
struct ClockSpec {
    mutex: Mutex<(u64, u64)>,
}

impl Default for ClockSpec {
    fn default() -> Self {
        ClockSpec {
            mutex: Mutex::new((0, 0)),
        }
    }
}

impl ClockSpec {
    #[cfg_vis(test, pub)]
    fn write(&self, hspo_add: u64, system: u64) {
        let mut guard = self.mutex.lock();
        guard.0 += hspo_add;
        guard.1 = system;
    }

    #[cfg_vis(test, pub)]
    fn read(&self) -> (u64, u64) {
        let guard = self.mutex.lock();
        (guard.0, guard.1)
    }
}

#[derive(Debug)]
struct RobotSender {
    ip_of_interest: IpAddr,
    last_packet_time: Option<std::time::Instant>,
    connection_active: Arc<AtomicBool>,
    connection_timeout: Duration,
    tcp_tx: Sender<TcpCartesianPositionPacket>,
    joint_tx: Sender<JointAnglesPacket>,
    var_tx: Sender<VariablesPacket>,
    tcp_dropper: Receiver<TcpCartesianPositionPacket>,
    joint_dropper: Receiver<JointAnglesPacket>,
    var_dropper: Receiver<VariablesPacket>,
    raw_clock: AtomicU32,
    tracked_clock: Arc<ClockSpec>,
}

impl RobotSender {
    fn update_clocks(&self, new_clock: u32) {
        let prev_clock = self.raw_clock.load(Ordering::Relaxed);
        let mut hspo_add = 0;
        if new_clock < prev_clock {
            let extra = (u32::MAX - prev_clock) as u64 + new_clock as u64 + 1;
            hspo_add = extra;
        } else if new_clock > prev_clock {
            let diff = (new_clock - prev_clock) as u64;
            hspo_add = diff;
        }
        self.raw_clock.store(new_clock, Ordering::Relaxed);
        let sys_micros = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap_or(Duration::ZERO)
            .as_micros();
        self.tracked_clock
            .write(hspo_add, sys_micros.try_into().unwrap_or(u64::MAX));
    }
}

/// A channel for receiving HSPO packets of a specific type.
///
/// Wraps an internal bounded buffer and provides blocking, non-blocking, and drain operations.
#[derive(Debug)]
pub struct HspoChannel<T> {
    rx: Receiver<T>,
}

impl<T> HspoChannel<T> {
    fn new(rx: Receiver<T>) -> Self {
        Self { rx }
    }

    /// Blocks until a packet is received or the timeout elapses.
    pub fn wait_for(&self, timeout: Duration) -> Option<T> {
        self.rx.recv_timeout(timeout).ok()
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
                inner: InnerChannel::Tcp(HspoChannel::new(channel.clone_rx())),
            }
        }

        pub fn from_joint(channel: &HspoChannel<JointAnglesPacket>) -> Self {
            Self {
                inner: InnerChannel::Joint(HspoChannel::new(channel.clone_rx())),
            }
        }

        pub fn from_var(channel: &HspoChannel<VariablesPacket>) -> Self {
            Self {
                inner: InnerChannel::Var(HspoChannel::new(channel.clone_rx())),
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
    }
}

/// Receives HSPO (High Speed Position Output) packets from a specific FANUC controller.
///
/// Created via [`initialize_broker`] followed by [`try_new`](Self::try_new). Packets are buffered internally
/// and can be consumed via the [`tcp`](Self::tcp), [`joint`](Self::joint), and [`var`](Self::var) channels.
#[cfg_attr(feature = "py", pyo3::pyclass)]
#[derive(Debug)]
pub struct HspoReceiver {
    connection_active: Arc<AtomicBool>,
    tracked_clock: Arc<ClockSpec>,
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
            Ok(server.add_robot(ip_of_interest, packet_buffer_size, connection_timeout)?)
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
        if let Some(server) = HSPO_SERVER.lock().as_ref() {
            server.add_robot(
                ip_of_interest.into(),
                packet_buffer_size,
                connection_timeout,
            )
        } else {
            Err(HspoBrokerNotInitializedError)
        }
    }

    /// Returns `true` if a packet has been received from this robot recently.
    pub fn is_connected(&self) -> bool {
        self.connection_active.load(Ordering::Relaxed)
    }

    /// Returns the cumulative HSPO clock in microseconds, accounting for wraps of the controller's 32-bit clock.
    pub fn clock_micros(&self) -> u64 {
        self.tracked_clock.read().0
    }

    /// Returns the cumulative HSPO clock in milliseconds.
    pub fn clock_ms(&self) -> f64 {
        self.clock_micros() as f64 / 1000.0
    }

    /// Returns a pair of `(hspo_clock_micros, system_time_micros)` for correlating controller time with system time.
    pub fn clock_pair_micros(&self) -> (u64, u64) {
        self.tracked_clock.read()
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
                    Ok((n, addr)) => {
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
                        let now = std::time::Instant::now();

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
                                        rs.update_clocks(p.clock);
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
                                        rs.update_clocks(p.clock);
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
                                        rs.update_clocks(p.clock);
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
                        log::error!("HSPO broker socket recv error: {}", e);
                        break;
                    }
                }
            }
        }

        // Update connection-active flags based on last packet timestamp (10ms timeout).
        let now = std::time::Instant::now();
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
    ) -> Result<HspoReceiver, HspoBrokerNotInitializedError> {
        let (tcp_tx, tcp_rx) = bounded::<TcpCartesianPositionPacket>(packet_buffer_size);
        let (joint_tx, joint_rx) = bounded::<JointAnglesPacket>(packet_buffer_size);
        let (var_tx, var_rx) = bounded::<VariablesPacket>(packet_buffer_size);
        let connection_active = Arc::new(AtomicBool::new(false));
        let tracked_clock = Arc::new(ClockSpec::default());

        let tcp = HspoChannel::new(tcp_rx);
        let joint = HspoChannel::new(joint_rx);
        let var = HspoChannel::new(var_rx);

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
            raw_clock: AtomicU32::new(0),
            tracked_clock: tracked_clock.clone(),
        };

        log::info!(
            "HSPO registering receiver for {} (buffer_size={})",
            ip_of_interest,
            packet_buffer_size
        );
        self.robot_appender
            .send(robot_sender)
            .map_err(|_| HspoBrokerNotInitializedError)?;
        let _ = self.waker.wake();

        Ok(HspoReceiver {
            connection_active,
            tracked_clock,
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
                    log::error!("HSPO broker thread exited with error: {}", e);
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
        log::info!("Initializing HSPO broker on {}", listen_on);
        let server = HspoBroker::create(listen_on, thread_config)?;
        *guard = Some(server);
        log::info!("HSPO broker initialized");
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
        log::info!("Initializing HSPO broker on {}", listen_on);
        let server = HspoBroker::create(listen_on, thread_config)?;
        *guard = Some(server);
        log::info!("HSPO broker initialized");
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
        log::info!("Destroying HSPO broker");
        broker.kill_switch.store(true, Ordering::Relaxed);
        if wait_for_thread {
            match broker._thread_handle.join() {
                Ok(()) => log::info!("HSPO broker thread exited cleanly"),
                Err(e) => log::error!("HSPO broker thread panicked: {:?}", e),
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
