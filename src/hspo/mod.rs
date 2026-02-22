use parking_lot::Mutex;
use std::{
    collections::HashMap,
    net::{IpAddr, SocketAddr},
    sync::{
        Arc, LazyLock,
        atomic::{AtomicBool, AtomicU32, Ordering},
    },
    thread,
    time::{Duration, SystemTime},
};

use crate::{
    joints::{JointFormat, JointTemplate},
    thread_util::{GeneralThreadError, ThreadConfig},
};
use bincode::{Decode, Encode};
use cfg_mixin::cfg_mixin;
use flume::{Receiver, Sender, TrySendError, bounded, unbounded};
use mio::net::UdpSocket as MioUdpSocket;
use mio::{Events, Interest, Poll, Token};
use serde::Serialize;

const TOK_SOCKET: Token = Token(0);

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
        format.convert_from(JointFormat::FanucRad, template, self.joints)
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
enum PacketType {
    TcpCartesianPosition = 1,
    JointAngles = 4,
    Variables = 16,
    Unknown,
}

impl PacketType {
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
    fn write(&self, hspo_add: u64, system: u64) {
        let mut guard = self.mutex.lock();
        guard.0 += hspo_add;
        guard.1 = system;
    }

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

/// Receives HSPO (High Speed Position Output) packets from a specific FANUC controller.
///
/// Created via [`initialize_broker`] followed by [`try_new`](Self::try_new). Packets are buffered internally
/// and can be consumed with blocking or non-blocking methods.
#[cfg_attr(feature = "py", pyo3::pyclass)]
#[derive(Debug)]
pub struct HspoReceiver {
    connection_active: Arc<AtomicBool>,
    tracked_clock: Arc<ClockSpec>,
    tcp_rx: Receiver<TcpCartesianPositionPacket>,
    joint_rx: Receiver<JointAnglesPacket>,
    var_rx: Receiver<VariablesPacket>,
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

    /// Blocks until a TCP cartesian position packet is received or the timeout elapses.
    #[cfg(off)]
    pub fn wait_for_tcp_packet(&self, timeout: Duration) -> Option<TcpCartesianPositionPacket> {
        self.tcp_rx.recv_timeout(timeout).ok()
    }

    /// Blocks until a TCP cartesian position packet is received or the timeout elapses.
    #[cfg(on)]
    pub fn wait_for_tcp_packet(&self, timeout_secs: f64) -> Option<TcpCartesianPositionPacket> {
        self.tcp_rx
            .recv_timeout(Duration::from_secs_f64(timeout_secs))
            .ok()
    }

    /// Blocks until a joint angles packet is received or the timeout elapses.
    #[cfg(off)]
    pub fn wait_for_joint_packet(&self, timeout: Duration) -> Option<JointAnglesPacket> {
        self.joint_rx.recv_timeout(timeout).ok()
    }

    /// Blocks until a joint angles packet is received or the timeout elapses.
    #[cfg(on)]
    pub fn wait_for_joint_packet(&self, timeout_secs: f64) -> Option<JointAnglesPacket> {
        self.joint_rx
            .recv_timeout(Duration::from_secs_f64(timeout_secs))
            .ok()
    }

    /// Blocks until a variables packet is received or the timeout elapses.
    #[cfg(off)]
    pub fn wait_for_var_packet(&self, timeout: Duration) -> Option<VariablesPacket> {
        self.var_rx.recv_timeout(timeout).ok()
    }

    /// Blocks until a variables packet is received or the timeout elapses.
    #[cfg(on)]
    pub fn wait_for_var_packet(&self, timeout_secs: f64) -> Option<VariablesPacket> {
        self.var_rx
            .recv_timeout(Duration::from_secs_f64(timeout_secs))
            .ok()
    }

    /// Returns the next buffered TCP cartesian position packet without blocking, or `None` if the buffer is empty.
    pub fn try_recv_tcp_packet(&self) -> Option<TcpCartesianPositionPacket> {
        self.tcp_rx.try_recv().ok()
    }

    /// Returns the next buffered joint angles packet without blocking, or `None` if the buffer is empty.
    pub fn try_recv_joint_packet(&self) -> Option<JointAnglesPacket> {
        self.joint_rx.try_recv().ok()
    }

    /// Returns the next buffered variables packet without blocking, or `None` if the buffer is empty.
    pub fn try_recv_var_packet(&self) -> Option<VariablesPacket> {
        self.var_rx.try_recv().ok()
    }

    /// Drains and returns all buffered TCP cartesian position packets.
    pub fn recv_all_tcp_packets(&self) -> Vec<TcpCartesianPositionPacket> {
        let mut packets = Vec::new();
        while let Ok(w) = self.tcp_rx.try_recv() {
            packets.push(w);
        }
        packets
    }

    /// Drains and returns all buffered joint angles packets.
    pub fn recv_all_joint_packets(&self) -> Vec<JointAnglesPacket> {
        let mut packets = Vec::new();
        while let Ok(w) = self.joint_rx.try_recv() {
            packets.push(w);
        }
        packets
    }

    /// Drains and returns all buffered variables packets.
    pub fn recv_all_var_packets(&self) -> Vec<VariablesPacket> {
        let mut packets = Vec::new();
        while let Ok(w) = self.var_rx.try_recv() {
            packets.push(w);
        }
        packets
    }

    /// Discards all buffered joint angles packets.
    pub fn clear_joint_packet_buffer(&self) {
        while self.joint_rx.try_recv().is_ok() {}
    }

    /// Discards all buffered TCP cartesian position packets.
    pub fn clear_tcp_packet_buffer(&self) {
        while self.tcp_rx.try_recv().is_ok() {}
    }

    /// Discards all buffered variables packets.
    pub fn clear_var_packet_buffer(&self) {
        while self.var_rx.try_recv().is_ok() {}
    }
}

struct HspoBroker {
    robot_appender: Sender<RobotSender>,
    err_flag: Arc<AtomicBool>,
    kill_switch: Arc<AtomicBool>,
    _thread_handle: std::thread::JoinHandle<()>,
}

fn broker_runtime(
    listen_on: SocketAddr,
    thread_config: Option<ThreadConfig>,
    robot_receiver: Receiver<RobotSender>,
    thread_kill_switch: Arc<AtomicBool>,
) -> Result<(), GeneralThreadError> {
    if let Some(thread_config) = thread_config {
        thread_config.configure_this_thread_print_failure();
    }

    let mut poll = Poll::new().map_err(|_| GeneralThreadError::FailedToCreatePoll)?;
    let mut events = Events::with_capacity(256);
    let mut socket =
        MioUdpSocket::bind(listen_on).map_err(|_| GeneralThreadError::FailedSocketBinding)?;
    poll.registry()
        .register(&mut socket, TOK_SOCKET, Interest::READABLE)
        .map_err(|_| GeneralThreadError::FailedSocketRegistry)?;

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
                    Err(_e) => {
                        // Socket error: continue loop and let poll wake us again.
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

        let robot_sender = RobotSender {
            ip_of_interest,
            last_packet_time: None,
            connection_active: connection_active.clone(),
            connection_timeout,
            tcp_tx,
            joint_tx,
            var_tx,
            tcp_dropper: tcp_rx.clone(),
            joint_dropper: joint_rx.clone(),
            var_dropper: var_rx.clone(),
            raw_clock: AtomicU32::new(0),
            tracked_clock: tracked_clock.clone(),
        };

        self.robot_appender
            .send(robot_sender)
            .map_err(|_| HspoBrokerNotInitializedError)?;

        Ok(HspoReceiver {
            connection_active,
            tracked_clock,
            tcp_rx,
            joint_rx,
            var_rx,
        })
    }

    fn create(
        listen_on: SocketAddr,
        thread_config: Option<ThreadConfig>,
    ) -> Result<Self, HspoBrokerNotInitializedError> {
        let local_kill_switch = Arc::new(AtomicBool::new(false));
        let local_err_flag = Arc::new(AtomicBool::new(false));
        let (robot_appender, robot_receiver) = unbounded::<RobotSender>();

        let thread_kill_switch = local_kill_switch.clone();
        let thread_err_flag = local_err_flag.clone();
        let _thread_handle = thread::Builder::new()
            .name("hspo_server".to_string())
            .spawn(move || {
                if let Err(e) =
                    broker_runtime(listen_on, thread_config, robot_receiver, thread_kill_switch)
                {
                    log::error!("HSPO broker thread exited with error: {}", e);
                    thread_err_flag.store(true, Ordering::Relaxed);
                }
            })
            .map_err(|_| HspoBrokerNotInitializedError)?;

        Ok(HspoBroker {
            robot_appender,
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
        let server = HspoBroker::create(listen_on, thread_config)?;
        *guard = Some(server);
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
        let server = HspoBroker::create(listen_on, thread_config)?;
        *guard = Some(server);
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
        broker.kill_switch.store(true, Ordering::Relaxed);
        if wait_for_thread {
            match broker._thread_handle.join() {
                Ok(()) => {}
                Err(e) => log::error!("HSPO broker thread was panicked: {:?}", e),
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
        child_module.add_function(wrap_pyfunction!(initialize_broker, &child_module)?)?;
        child_module.add_function(wrap_pyfunction!(destroy_broker, &child_module)?)?;
        child_module.add_function(wrap_pyfunction!(has_broker_errored, &child_module)?)?;
        child_module.add_class::<TcpCartesianPositionPacket>()?;
        child_module.add_class::<JointAnglesPacket>()?;
        child_module.add_class::<VariablesPacket>()?;

        parent_module.add_submodule(&child_module)
    }
}
