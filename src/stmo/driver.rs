use std::{
    collections::VecDeque,
    io,
    net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4},
    sync::Arc,
    time::{Duration, Instant},
};

use cfg_mixin::cfg_mixin;
use flume::{Receiver, Sender};

use crate::{
    joints::JointDataSizeError,
    stmo::{
        JointMovementLimit,
        proto::{
            CommandPositionRequestPacket, CommandPositionResponsePacket, MotionCommandPacket,
            RobotStatusPacket, RxPackets, StartPacket, StopPacket, ThresholdTableRequestPacket,
            TxPackets, VersionNumberRequestPacket,
        },
        types::{AxisMotionConstraint, JointMovementLimits, RxStorage, StreamMotionError},
    },
    thread_util::{ThreadConfig, ThreadHandle},
};

use mio::net::UdpSocket as MioUdpSocket;
use mio::{Events, Interest, Poll, Token, Waker};

#[cfg(feature = "py")]
use pyo3::prelude::*;

const TOK_SOCKET: Token = Token(0);
const TOK_WAKER: Token = Token(1);

#[derive(Debug, Clone)]
enum MaybeMany<T: Clone> {
    One(T),
    Many(Vec<T>),
}

enum ToThreadMessage {
    Start(StartPacket),
    Stop(StopPacket),
    ThresholdTableRequest(ThresholdTableRequestPacket),
    MotionCommandDouble(MaybeMany<MotionCommandPacket>),
}

#[derive(Debug)]
struct StreamMotionContext {
    socket: MioUdpSocket,
    from_driver: Receiver<ToThreadMessage>,
    to_driver: Sender<RxPackets>,
    protocol_version: u32,
    last_command_position_request_time: Instant,
    motion_command_queue: VecDeque<MaybeMany<MotionCommandPacket>>,
}

impl StreamMotionContext {
    const COMMAND_POSITION_RATE: Duration = Duration::from_millis(128);

    fn new(
        from_driver: Receiver<ToThreadMessage>,
        to_driver: Sender<RxPackets>,
        socket: MioUdpSocket,
    ) -> Self {
        Self {
            from_driver,
            to_driver,
            socket,
            protocol_version: 0,
            last_command_position_request_time: Instant::now() - Self::COMMAND_POSITION_RATE,
            motion_command_queue: VecDeque::new(),
        }
    }

    /**
     * Sends the packet or queues it if the socket is not ready.
     */
    fn send(
        &mut self,
        tx: TxPackets,
        buf: &mut [u8],
        timeout: Option<Duration>,
        version_override: Option<u32>,
    ) -> Option<String> {
        let n = tx.encode_into(version_override.unwrap_or(self.protocol_version), buf);
        match self.socket.send(&buf[..n]) {
            Ok(_) => None,
            Err(ref e)
                if e.kind() == io::ErrorKind::WouldBlock
                    || e.kind() == io::ErrorKind::Interrupted =>
            {
                if let Some(to) = timeout {
                    match self.retry_sending(tx, to, buf) {
                        Ok(()) => None,
                        Err(e) => Some(format!("Error sending packet with retry: {:?}", e)),
                    }
                } else {
                    Some("Socket busy, packet queued".into())
                }
            }
            Err(e) => Some(format!("Error sending packet: {:?}", e)),
        }
    }

    /**
     * Try to send a packet, retrying until timeout if the socket is not ready.
     */
    fn retry_sending(
        &mut self,
        tx: TxPackets,
        timeout: Duration,
        buf: &mut [u8],
    ) -> Result<(), StreamMotionError> {
        let start = Instant::now();
        let mut sent = false;
        let n = tx.encode_into(self.protocol_version, buf);
        let sleeper = spin_sleep::SpinSleeper::new(1_000_000);
        while !sent && start.elapsed() < timeout {
            match self.socket.send(&buf[..n]) {
                Ok(_) => sent = true,
                Err(ref e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::Interrupted =>
                {
                    sleeper.sleep(Duration::from_micros(500));
                }
                Err(e) => {
                    tracing::error!("Error sending packet: {:?}", e);
                    break;
                }
            }
        }
        if sent {
            Ok(())
        } else {
            Err(StreamMotionError::Timeout)
        }
    }

    fn next_motion_command(&mut self) -> Option<MotionCommandPacket> {
        let front_is_many = match self.motion_command_queue.front()? {
            MaybeMany::Many(_) => true,
            MaybeMany::One(_) => false,
        };
        if front_is_many {
            if let Some(MaybeMany::Many(vec)) = self.motion_command_queue.front_mut() {
                if !vec.is_empty() {
                    return vec.pop();
                } else {
                    self.motion_command_queue.pop_front();
                    return self.next_motion_command();
                }
            }
        } else {
            return self.motion_command_queue.pop_front().map(|m| match m {
                MaybeMany::One(c) => c,
                _ => unreachable!("Checked variant"),
            });
        }
        None
    }

    pub fn context_loop(mut self, thread_handle: ThreadHandle, mut poll: Poll) {
        let mut events = Events::with_capacity(64);
        let mut rx_buf = [0u8; 2048];
        let mut tx_buf = [0u8; 1024];
        let mut last_motion_was_last = false;
        // let mut status_cycle_count: u32 = 0;

        while thread_handle.should_live() {
            if let Err(e) = poll.poll(&mut events, None) {
                if e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                break;
            }

            for ev in &events {
                match ev.token() {
                    TOK_SOCKET => {
                        // drain UDP
                        loop {
                            match self.socket.recv(&mut rx_buf) {
                                Ok(n) if n > 0 => {
                                    if let Some(rx) = RxPackets::decode_from(&rx_buf[..n]) {
                                        let _ = self.to_driver.send(rx);
                                        if let RxPackets::VersionNumberResponse(vn) = &rx {
                                            self.protocol_version = vn.version;
                                            tracing::info!(
                                                "Detected Stream Motion protocol version {}",
                                                self.protocol_version
                                            );
                                        }
                                        // respond with the best matching motion command if applicable
                                        if let RxPackets::RobotStatus(state) = &rx {
                                            if !state.status_bits().ready_for_commands() {
                                                continue;
                                            }
                                            // status_cycle_count = status_cycle_count.wrapping_add(1);
                                            // if status_cycle_count < state.status_bits().packet_rate() as u32
                                            // {
                                            //     continue;
                                            // }
                                            // status_cycle_count = 0;
                                            if let Some(mut cmd) = self.next_motion_command() {
                                                cmd.seq = state.seq;
                                                last_motion_was_last = cmd.last_command;
                                                let _ = self.send(
                                                    TxPackets::MotionCommand(cmd),
                                                    &mut tx_buf,
                                                    Some(Duration::from_millis(6)),
                                                    None,
                                                );
                                            } else if state.status_bits().command_received() {
                                                if !last_motion_was_last {
                                                    tracing::warn!(
                                                        "Motion command queue empty, last command was not marked last. Sending hold command."
                                                    );
                                                }
                                                let mut cmd =
                                                    MotionCommandPacket::from_status(&state, false);
                                                cmd.seq = state.seq;
                                                let _ = self.send(
                                                    TxPackets::MotionCommand(cmd),
                                                    &mut tx_buf,
                                                    Some(Duration::from_millis(6)),
                                                    None,
                                                );
                                            }
                                        }
                                    } else {
                                        tracing::warn!(
                                            "Received unknown packet: ({}) {:02X?}",
                                            n,
                                            &rx_buf[..n]
                                        );
                                    }
                                }
                                Ok(_) => {
                                    tracing::warn!("Received empty packet");
                                    break;
                                }
                                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                                Err(ref e) if e.kind() == io::ErrorKind::Interrupted => continue,
                                Err(e) => {
                                    tracing::error!("Error receiving packet: {:?}", e);
                                    break;
                                }
                            }
                        }

                        let req =
                            TxPackets::CommandPositionRequest(CommandPositionRequestPacket {});
                        if self.last_command_position_request_time.elapsed()
                            >= Self::COMMAND_POSITION_RATE
                            && self.protocol_version != 0
                        {
                            let _ =
                                self.send(req, &mut tx_buf, Some(Duration::from_millis(2)), None);
                            self.last_command_position_request_time = Instant::now();
                        }
                    }

                    TOK_WAKER => {
                        // drain commands from driver
                        while let Ok(tx) = self.from_driver.try_recv() {
                            match tx {
                                ToThreadMessage::Start(pkt) => {
                                    let _ = self.send(
                                        TxPackets::Start(pkt),
                                        &mut tx_buf,
                                        Some(Duration::from_millis(24)),
                                        Some(3),
                                    );
                                    let _ = self.send(
                                        TxPackets::VersionNumberRequest(
                                            VersionNumberRequestPacket {},
                                        ),
                                        &mut tx_buf,
                                        Some(Duration::from_millis(24)),
                                        Some(3),
                                    );
                                }
                                ToThreadMessage::MotionCommandDouble(pkt) => {
                                    self.motion_command_queue.push_back(pkt);
                                }
                                ToThreadMessage::Stop(pkt) => {
                                    let _ = self.send(
                                        TxPackets::Stop(pkt),
                                        &mut tx_buf,
                                        Some(Duration::from_millis(24)),
                                        None,
                                    );
                                }
                                ToThreadMessage::ThresholdTableRequest(pkt) => {
                                    let _ = self.send(
                                        TxPackets::ThresholdTableRequest(pkt),
                                        &mut tx_buf,
                                        None,
                                        None,
                                    );
                                    tracing::info!("Sent ThresholdTableRequest");
                                }
                            }
                        }
                    }

                    _ => {}
                }
            }
        }

        // graceful shutdown if asked to stop while loop breaks
        if thread_handle.should_live() {
            if self.protocol_version == 0 {
                // never started, nothing to do
                tracing::info!("StreamMotionContext exiting (never started)");
                thread_handle.has_died();
                return;
            }
            let stop_res = self.retry_sending(
                TxPackets::Stop(StopPacket {}),
                Duration::from_millis(24),
                &mut tx_buf,
            );
            if let Err(e) = stop_res {
                tracing::error!("Error sending stop packet during shutdown: {e:?}");
            }
        }
        tracing::info!("StreamMotionContext exited");
        thread_handle.has_died();
    }
}

#[derive(Debug)]
struct StreamMotionConnection {
    thread_handle: ThreadHandle,
    to_thread: Sender<ToThreadMessage>,
    from_thread: Receiver<RxPackets>,
    is_started: bool,
}

#[cfg_attr(feature = "py", pyo3::pyclass(str))]
#[derive(Debug)]
pub struct StreamMotionDriver {
    remote_addr: IpAddr,
    connection: Option<StreamMotionConnection>,
    cached_movement_limits: Option<JointMovementLimits>,
    rx_storage: RxStorage,
}

impl StreamMotionDriver {
    #[inline]
    fn send_packet(&self, tx: ToThreadMessage) {
        if let Some(conn) = &self.connection {
            if let Err(e) = conn.to_thread.send(tx) {
                tracing::error!("Error sending packet to thread: {:?}", e);
            }
            let _ = conn.thread_handle.wake();
        }
    }
}

#[cfg(feature = "py")]
type DriverResult<T> = pyo3::PyResult<T>;
#[cfg(not(feature = "py"))]
type DriverResult<T> = Result<T, StreamMotionError>;

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl StreamMotionDriver {
    #[cfg(on)]
    #[on(pyo3(signature = (addr)))]
    #[on(new)]
    pub fn new(addr: Bound<PyAny>) -> DriverResult<Self> {
        let addr = addr.extract::<IpAddr>()?;

        Ok(Self {
            remote_addr: addr,
            connection: None,
            cached_movement_limits: None,
            rx_storage: RxStorage::new(),
        })
    }

    #[cfg(off)]
    pub fn new<T: Into<IpAddr>>(remote_addr: T) -> Self {
        let remote_addr = remote_addr.into();
        Self {
            remote_addr,
            connection: None,
            cached_movement_limits: None,
            rx_storage: RxStorage::new(),
        }
    }

    #[on(pyo3(signature = ()))]
    pub fn get_remote_addr(&self) -> String {
        self.remote_addr.to_string()
    }

    pub fn refresh(&mut self) {
        let connection = match &self.connection {
            Some(c) => c,
            None => return,
        };
        while let Ok(pkt) = connection.from_thread.try_recv() {
            match pkt {
                RxPackets::RobotStatus(state) => self.rx_storage.state.push_back(state),
                RxPackets::ThresholdTableResponse(threshold) => {
                    self.rx_storage.threshold_table.push_back(threshold)
                }
                RxPackets::CommandPositionResponse(cmd_pos) => {
                    self.rx_storage.command_position.push_back(cmd_pos)
                }
                _ => {}
            }
        }
        self.rx_storage.prune();
    }

    pub fn command_motion(&mut self, motion: MotionCommandPacket) -> DriverResult<()> {
        if let None = &self.connection {
            return Err(StreamMotionError::NotConnected).map_err(Into::into);
        }
        if !self.is_started() {
            return Err(StreamMotionError::NotStarted).map_err(Into::into);
        }
        self.send_packet(ToThreadMessage::MotionCommandDouble(MaybeMany::One(motion)));
        self.refresh();
        Ok(())
    }

    pub fn command_motion_batch(
        &mut self,
        mut motions: Vec<MotionCommandPacket>,
    ) -> DriverResult<()> {
        if motions.len() == 1 {
            return self.command_motion(motions.pop().unwrap());
        }
        if let None = &self.connection {
            return Err(StreamMotionError::NotConnected).map_err(Into::into);
        }
        if !self.is_started() {
            return Err(StreamMotionError::NotStarted).map_err(Into::into);
        }
        if motions.is_empty() {
            return Ok(());
        }
        motions.reverse();
        self.send_packet(ToThreadMessage::MotionCommandDouble(MaybeMany::Many(
            motions,
        )));
        self.refresh();
        Ok(())
    }

    pub fn stop(&mut self) {
        self.send_packet(ToThreadMessage::Stop(StopPacket {}));
        self.refresh();
    }

    #[on(pyo3(signature = (thread_config=None)))]
    pub fn connect(&mut self, thread_config: Option<ThreadConfig>) -> DriverResult<()> {
        if let Some(conn) = &self.connection
            && conn.thread_handle.is_alive()
        {
            return Ok(());
        }
        let port = openport::pick_unused_port(57000..60000).unwrap_or(60000);
        let local_addr = SocketAddrV4::new(Ipv4Addr::new(0, 0, 0, 0), port);
        let socket = std::net::UdpSocket::bind(local_addr).map_err(StreamMotionError::from)?;
        socket
            .connect(SocketAddr::new(self.remote_addr, 60015))
            .map_err(StreamMotionError::from)?;
        socket
            .set_nonblocking(true)
            .map_err(StreamMotionError::from)?;

        let (to_thread, from_driver) = flume::unbounded();
        let (to_driver, from_thread) = flume::unbounded();

        let mut thread_handle = ThreadHandle::new();
        let thread_handle_mv = thread_handle.to_pass_in();

        let (waker_tx, waker_rx) = flume::bounded(1);

        let thread = std::thread::Builder::new()
            .name("fanuc-stmo-runner".to_string())
            .spawn(move || {
                let mut thread_handle = thread_handle_mv;
                if let Some(cfg) = thread_config {
                    let _ = cfg.configure_this_thread();
                }

                let mut socket = MioUdpSocket::from_std(socket);

                let poll = Poll::new().expect("Poll creation failed");
                poll.registry()
                    .register(&mut socket, TOK_SOCKET, Interest::READABLE)
                    .expect("Register UDP socket failed");

                let waker = Arc::new(
                    Waker::new(poll.registry(), TOK_WAKER).expect("Waker creation failed"),
                );
                // send a clone to the driver so API calls can wake the poller
                let _ = waker_tx.send(waker.clone());
                thread_handle.set_waker_mio(waker);

                let context = StreamMotionContext::new(from_driver, to_driver, socket);
                context.context_loop(thread_handle, poll);
            })?;

        let thread_waker = waker_rx.recv().expect("Waker was not sent from thread");
        thread_handle.set_waker_mio(thread_waker);
        thread_handle.set_handle(thread);

        self.connection = Some(StreamMotionConnection {
            thread_handle,
            to_thread,
            from_thread,
            is_started: false,
        });

        Ok(())
    }

    #[on(pyo3(signature = (timeout_secs=2.0)))]
    pub fn start(&mut self, timeout_secs: f32) -> DriverResult<()> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        let start_time = Instant::now();
        let end_time = start_time + timeout;
        if let Some(conn) = &self.connection {
            self.send_packet(ToThreadMessage::Start(StartPacket {}));
            let mut started = false;
            while start_time.elapsed() < timeout {
                let remaining = end_time.saturating_duration_since(Instant::now());
                match conn.from_thread.recv_timeout(remaining) {
                    Ok(RxPackets::VersionNumberResponse(_)) => {
                        started = true;
                    }
                    _ => {}
                }
            }
            if !started {
                Err(StreamMotionError::Timeout)?;
            }
        } else {
            Err(StreamMotionError::NotConnected)?;
        };
        if let Some(conn) = &mut self.connection {
            conn.is_started = true;
        }
        Ok(())
    }

    pub fn disconnect(&mut self) {
        if let Some(conn) = self.connection.take() {
            let _ = conn.to_thread.send(ToThreadMessage::Stop(StopPacket {}));
            conn.thread_handle.join();
        }
        self.rx_storage.clear();
    }

    pub fn is_connected(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.thread_handle.is_alive()
        } else {
            false
        }
    }

    pub fn is_started(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.is_started
        } else {
            false
        }
    }

    #[on(pyo3(signature = (extra_axis=0)))]
    pub fn fetch_movement_limits(&mut self, extra_axis: u8) -> DriverResult<JointMovementLimits> {
        if !self.is_connected() {
            return Err(StreamMotionError::NotConnected).map_err(Into::into);
        }
        if !self.is_started() {
            return Err(StreamMotionError::NotStarted).map_err(Into::into);
        }
        if let Some(cached) = self.cached_movement_limits {
            return Ok(cached);
        }
        if extra_axis > 3 {
            return Err(StreamMotionError::JointDataTooSmall(JointDataSizeError(9)))
                .map_err(Into::into);
        }

        let axis_cnt = 6 + extra_axis as usize;

        let mut seen = vec![[false; 3]; axis_cnt];
        let mut limits = JointMovementLimits::default();

        let mut last_send = Instant::now()
            .checked_sub(Duration::from_millis(50))
            .unwrap_or_else(Instant::now);

        let all_filled = |seen: &Vec<[bool; 3]>| seen.iter().flatten().all(|&b| b);

        while !all_filled(&seen) && self.is_connected() {
            if last_send.elapsed() >= Duration::from_millis(48) {
                for joint_idx in 0..axis_cnt {
                    for deriv_idx in 0..3 {
                        if !seen[joint_idx][deriv_idx] {
                            let req = ThresholdTableRequestPacket::try_from((
                                joint_idx as u32 + 1,
                                deriv_idx as u32,
                            ));
                            if let Err(e) = req {
                                tracing::error!(
                                    "Invalid ThresholdTableRequestPacket parameters: {:?}",
                                    e
                                );
                                continue;
                            }
                            self.send_packet(ToThreadMessage::ThresholdTableRequest(req.unwrap()));
                            std::thread::sleep(Duration::from_millis(24));
                        }
                    }
                }
                last_send = Instant::now();
            }

            self.refresh();

            while let Some(pkt) = self.rx_storage.threshold_table.pop_front() {
                tracing::debug!(
                    "Received movement limit: axis {}, type {}, vmax {}",
                    pkt.axis_number,
                    pkt.limit_type,
                    pkt.vmax,
                );
                let axis = pkt.axis_number as usize - 1;
                let deriv = pkt.limit_type as usize;

                if axis < axis_cnt && deriv < 3 && !seen[axis][deriv] {
                    let entry = &mut limits.joints[axis];

                    // set vmax once (first response wins)
                    if limits.vmax == 0 {
                        limits.vmax = pkt.vmax;
                    }

                    let cons = AxisMotionConstraint {
                        no_payload: pkt.no_payload,
                        max_payload: pkt.max_payload,
                    };

                    if entry.is_none() {
                        *entry = Some(JointMovementLimit::default());
                    }
                    let entry = entry.as_mut().unwrap();

                    match deriv {
                        0 => entry.velocity = cons,
                        1 => entry.acceleration = cons,
                        2 => entry.jerk = cons,
                        _ => {}
                    }
                    seen[axis][deriv] = true;
                }
            }

            std::thread::sleep(Duration::from_millis(25));
        }

        if self.is_connected() && all_filled(&seen) {
            self.cached_movement_limits = Some(limits);
            Ok(limits)
        } else {
            Err(StreamMotionError::NotConnected).map_err(Into::into)
        }
    }

    pub fn pull_states(&mut self) -> Vec<RobotStatusPacket> {
        self.refresh();
        self.rx_storage.state.drain(..).collect()
    }

    pub fn pull_command_positions(&mut self) -> Vec<CommandPositionResponsePacket> {
        self.refresh();
        self.rx_storage.command_position.drain(..).collect()
    }

    #[on(pyo3(signature = (timeout_ms = 200)))]
    pub fn wait_for_command_position(
        &mut self,
        timeout_ms: u32,
    ) -> Option<CommandPositionResponsePacket> {
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(timeout_ms.into()) {
            self.refresh();
            if let Some(pkt) = self.rx_storage.command_position.pop_front() {
                return Some(pkt);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        None
    }
}

impl Drop for StreamMotionDriver {
    fn drop(&mut self) {
        self.disconnect();
    }
}

impl std::fmt::Display for StreamMotionDriver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opening = if cfg!(feature = "py") { "(" } else { "{" };
        let closing = if cfg!(feature = "py") { ")" } else { "}" };
        write!(
            f,
            "StreamMotionDriver{}remote_addr: {}, connected: {}{}",
            opening,
            self.remote_addr,
            self.is_connected(),
            closing
        )
    }
}

#[cfg(feature = "py")]
pub mod py {
    use crate::stmo::types::JointMovementLimit;

    use super::*;

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<AxisMotionConstraint>()?;
        parent_module.add_class::<JointMovementLimit>()?;
        parent_module.add_class::<JointMovementLimits>()?;
        parent_module.add_class::<StreamMotionDriver>()?;

        Ok(())
    }
}
