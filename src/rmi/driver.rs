//! TCP driver for the FANUC Remote Motion Interface (RMI).

use std::{
    collections::VecDeque,
    io::{Read, Write},
    net::{IpAddr, SocketAddr},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU32, Ordering},
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

use cfg_mixin::cfg_mixin;
use fast_talker::options::{SocketOption, ThreadOption};
use flume::{Receiver, Sender};
use mio::{Events, Interest, Poll, Token, Waker, net::TcpStream};
use serde::Deserialize;
use serde_json::{Map as JsonMap, Value as JsonValue};
use std::net::TcpStream as StdTcpStream;

use crate::{
    TelemetrySink,
    rmi::{
        FeatureGates, FeatureLockEntry, ResponsePacket, SendPacket, SendablePacket,
        SoftwareOptions,
        errors::{RmiError, RmiProtocolError, RmiResult},
        proto::{
            commands::{FrcAbort, FrcReset, FrcResetResponse},
            communication::{
                FrcConnectResponse, FrcDisconnectResponse, connect_json, disconnect_json,
            },
        },
        rmi_handle::*,
    },
    thread_util::ThreadHandle,
    tuning::{self, SocketRole, ThreadRole},
};

const DRIVER: &str = "rmi";

type JsonObject = JsonMap<String, JsonValue>;

/// Shared sink observing RMI traffic: outgoing [`SendPacket`], incoming [`ResponsePacket`].
pub type RmiTelemetry = Arc<dyn TelemetrySink<SendPacket, ResponsePacket>>;

#[derive(Debug, Clone)]
enum VariadicString {
    None,
    Single(String),
    Multiple(Vec<String>),
}

impl std::fmt::Display for VariadicString {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VariadicString::None => write!(f, "<empty>"),
            VariadicString::Single(s) => write!(f, "{}", s),
            VariadicString::Multiple(v) => write!(f, "{:?}", v),
        }
    }
}

fn rmi_string_reader(input: &[u8]) -> RmiResult<VariadicString> {
    let s = std::str::from_utf8(input)?;
    let parts: Vec<String> = s
        .split("\r\n")
        .filter(|part| !part.is_empty())
        .map(|part| part.to_string())
        .collect();
    if parts.is_empty() {
        Ok(VariadicString::None)
    } else if parts.len() == 1 {
        Ok(VariadicString::Single(parts[0].clone()))
    } else {
        Ok(VariadicString::Multiple(parts))
    }
}

/// Reads the controller's one-line reply on a blocking stream by `deadline`,
/// however many segments it arrives in.
fn read_line_by(tcp: &mut StdTcpStream, deadline: Instant) -> RmiResult<Vec<u8>> {
    const LIMIT: usize = 4096;
    let mut line = Vec::new();
    let mut buf = [0u8; 512];
    while !line.windows(2).any(|w| w == b"\r\n") {
        if line.len() > LIMIT {
            return Err(RmiError::Structure(format!(
                "no line end in the first {LIMIT} bytes of the reply"
            )));
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(std::io::Error::from(std::io::ErrorKind::TimedOut).into());
        }
        tcp.set_read_timeout(Some(remaining))?;
        match tcp.read(&mut buf) {
            Ok(0) => return Err(std::io::Error::from(std::io::ErrorKind::UnexpectedEof).into()),
            Ok(n) => line.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                return Err(std::io::Error::from(std::io::ErrorKind::TimedOut).into());
            }
            Err(e) => return Err(e.into()),
        }
    }
    Ok(line)
}

enum ReplyName<'a> {
    Instruction,
    Other(&'a str),
}

fn reply_name(reply: &JsonValue) -> Option<ReplyName<'_>> {
    let text = |key| reply.get(key).and_then(JsonValue::as_str);
    text("Instruction")
        .map(|_| ReplyName::Instruction)
        .or_else(|| {
            text("Command")
                .or_else(|| text("Communication"))
                .map(ReplyName::Other)
        })
}

fn sequence_id(reply: &JsonValue) -> Option<u64> {
    reply.get("SequenceID").and_then(JsonValue::as_u64)
}

fn rmi_string_writer(value: JsonObject) -> RmiResult<Vec<u8>> {
    let mut serialized = serde_json::to_string(&value)?;
    serialized.push_str("\r\n");
    Ok(serialized.into_bytes())
}

fn all_feature_gate(packet: &str) -> Vec<(&'static str, &'static [FeatureGates])> {
    let packet_specific = FeatureLockEntry::get(packet).field_gates.iter();
    let all = FeatureLockEntry::get("ALL").field_gates.iter();
    let packet_type = FeatureLockEntry::get("PACKET").field_gates.iter();
    packet_specific
        .chain(all)
        .chain(packet_type)
        .map(|(f, g)| (*f, *g))
        .collect()
}

#[inline]
fn validate_gates(
    value: &JsonObject,
    major_version: u8,
    options: &[SoftwareOptions],
) -> RmiResult<()> {
    let packet_name = if let Some(JsonValue::String(name)) = value.get("Communication") {
        name.as_str()
    } else if let Some(JsonValue::String(name)) = value.get("Command") {
        name.as_str()
    } else if let Some(JsonValue::String(name)) = value.get("Instruction") {
        name.as_str()
    } else {
        return Err(RmiError::Structure(
            "Packet does not contain a valid name field".to_string(),
        ));
    };
    let mut fields: Vec<&str> = value.keys().map(|k| k.as_str()).collect();
    fields.push(packet_name);
    for (field, gates) in all_feature_gate(packet_name) {
        if fields.contains(&field) {
            for gate in gates {
                gate.validate(field.to_string(), major_version, options, &fields)?;
            }
        }
    }

    Ok(())
}

struct PendingWrite {
    buf: Vec<u8>,
    offset: usize,
    handle: RmiHandleGeneric,
    is_disconnect: bool,
    packet: Option<SendPacket>,
}

struct RmiRunner {
    handle: ThreadHandle,
    tcp_stream: TcpStream,
    from_driver: Receiver<RunnerMessage>,
    response_stack: VecDeque<RmiHandleGeneric>,
    config: RmiDriverConfig,
    telemetry: Option<RmiTelemetry>,
    /// Bytes read past the last complete response.
    rx_buf: Vec<u8>,
    /// Set once FRC_Disconnect is on the wire: how long the runner keeps
    /// reading for its reply and any still in flight.
    disconnect_deadline: Option<Instant>,
}

impl RmiRunner {
    const TOK_SOCKET: Token = Token(0);
    const TOK_WAKER: Token = Token(1);
    /// How long a sent FRC_Disconnect waits for the controller's reply before
    /// the runner gives up and its handle resolves as disconnected.
    const DISCONNECT_REPLY_WAIT: Duration = Duration::from_millis(500);

    #[allow(clippy::too_many_arguments)]
    fn start(
        addr: SocketAddr,
        connect_timeout: Duration,
        handle: ThreadHandle,
        from_driver: Receiver<RunnerMessage>,
        config: RmiDriverConfig,
        thread: Vec<ThreadOption>,
        socket: &[SocketOption],
        telemetry: Option<RmiTelemetry>,
    ) -> RmiResult<(JoinHandle<()>, Arc<Waker>, Arc<AtomicBool>)> {
        let std_stream = StdTcpStream::connect_timeout(&addr, connect_timeout)?;
        std_stream.set_nonblocking(true)?;
        let mut tcp_stream = TcpStream::from_std(std_stream);
        tuning::apply_socket(DRIVER, SocketRole::TcpControl, &tcp_stream, socket)?;
        let poll = Poll::new()?;
        poll.registry().register(
            &mut tcp_stream,
            RmiRunner::TOK_SOCKET,
            Interest::READABLE.add(Interest::WRITABLE),
        )?;
        let waker = Arc::new(Waker::new(poll.registry(), RmiRunner::TOK_WAKER)?);
        let local_err_flag = Arc::new(AtomicBool::new(false));
        let thread_err_flag = local_err_flag.clone();
        let (started_tx, started_rx) = flume::bounded(1);
        let handle = std::thread::Builder::new()
            .name("fanuc-rmi-runner".to_string())
            .spawn(move || {
                rmi_runner_runtime(
                    handle,
                    tcp_stream,
                    poll,
                    from_driver,
                    config,
                    thread,
                    started_tx,
                    telemetry,
                    thread_err_flag,
                )
            })?;
        let started = started_rx
            .recv()
            .unwrap_or_else(|_| Err(std::io::Error::other("RMI runner exited during startup")));
        if let Err(e) = started {
            let _ = handle.join();
            return Err(e.into());
        }
        Ok((handle, waker, local_err_flag))
    }

    fn fill_queue(&mut self, message_queue: &mut VecDeque<PendingWrite>) -> bool {
        while let Ok(msg) = self.from_driver.try_recv() {
            if msg.is_priority() {
                // A request already partly on the wire has to be finished
                // before anything else can be framed after it.
                let at = usize::from(message_queue.front().is_some_and(|w| w.offset > 0));
                message_queue.insert(at, msg.into_pending_write());
            } else {
                message_queue.push_back(msg.into_pending_write());
            }
        }
        message_queue.front().is_some_and(|w| self.may_write(w))
            || self.response_stack.len() < self.config.buffer_cnt as usize
    }

    /// FRC_Disconnect is exempt from the in-flight limit: when every slot is
    /// held by a request the controller never answers, it is the only way left
    /// to tell the controller the session is over.
    fn may_write(&self, w: &PendingWrite) -> bool {
        w.is_disconnect
            || w.offset > 0
            || self.response_stack.len() < self.config.buffer_cnt as usize
    }

    fn read_stream(&mut self, buf: &mut [u8]) -> RmiResult<()> {
        loop {
            match self.tcp_stream.read(buf) {
                Ok(0) => {
                    tracing::error!("RMI TCP stream read returned 0 bytes, connection lost");
                    return Err(RmiError::Disconnected);
                }
                Ok(n) => {
                    tracing::trace!(len = n, "Read bytes from RMI stream");
                    self.rx_buf.extend_from_slice(&buf[..n]);
                    self.dispatch_lines();
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(e) => {
                    tracing::error!(error = %e, "RMI TCP stream read error");
                    return Err(RmiError::CommunicationError(e));
                }
            }
        }
    }

    /// Resolves a handle for every complete `\r\n`-terminated response
    /// received so far, leaving a trailing partial one buffered.
    fn dispatch_lines(&mut self) {
        let mut consumed = 0;
        while let Some(len) = self.rx_buf[consumed..]
            .windows(2)
            .position(|w| w == b"\r\n")
        {
            let line = self.rx_buf[consumed..consumed + len].to_vec();
            consumed += len + 2;
            if !line.iter().all(u8::is_ascii_whitespace) {
                self.dispatch_line(&line);
            }
        }
        self.rx_buf.drain(..consumed);
    }

    /// One line from the controller: normally one reply, but two replies
    /// whose `\r\n` was lost between them arrive as one line of two objects.
    fn dispatch_line(&mut self, line: &[u8]) {
        let text = match std::str::from_utf8(line) {
            Ok(text) => text,
            Err(e) => return self.resolve(None, Err(e.into()), line),
        };
        tracing::debug!(response = %text, "RMI Runner received response");
        let values: Vec<JsonValue> = serde_json::Deserializer::from_str(text)
            .into_iter()
            .collect::<Result<_, _>>()
            .unwrap_or_default();
        if values.len() > 1 && values.iter().all(|v| reply_name(v).is_some()) {
            for value in values {
                let parsed = ResponsePacket::deserialize(&value).map_err(RmiError::from);
                self.resolve(Some(&value), parsed, line);
            }
            return;
        }
        let parsed = serde_json::from_str::<ResponsePacket>(text).map_err(RmiError::from);
        self.resolve(values.first(), parsed, line);
    }

    /// Settles the request a reply answers.
    ///
    /// Instructions are answered when they complete, and commands at once
    /// even while instructions are still running (B-84184EN/03 §1.4.4,
    /// §2.3), so replies do not come back in the order the requests went
    /// out. An instruction's reply carries its SequenceID; a command's or
    /// communication packet's carries its name and repeats fields of the
    /// request, and goes to the oldest unanswered request that matches. A
    /// line that cannot be read at all goes to the oldest unanswered command,
    /// since only instructions can still be running.
    fn resolve(
        &mut self,
        reply: Option<&JsonValue>,
        parsed: RmiResult<ResponsePacket>,
        line: &[u8],
    ) {
        let stack = &self.response_stack;
        let oldest_command = || stack.iter().position(|h| !h.is_instruction());
        let instruction = |seq: Option<u64>| {
            seq.and_then(|seq| {
                stack
                    .iter()
                    .position(|h| h.is_instruction() && u64::from(h.sequence_id()) == seq)
            })
        };
        let at = match reply.and_then(|r| reply_name(r).map(|name| (r, name))) {
            None => oldest_command().or_else(|| (!stack.is_empty()).then_some(0)),
            Some((_, ReplyName::Instruction)) => instruction(reply.and_then(sequence_id)),
            Some((r, ReplyName::Other("FRC_SystemFault"))) => instruction(sequence_id(r))
                .or_else(|| stack.iter().position(RmiHandleGeneric::is_instruction))
                .or_else(|| (!stack.is_empty()).then_some(0)),
            Some((_, ReplyName::Other("FRC_Terminate"))) => {
                let packet = parsed.ok();
                for handle in self.response_stack.drain(..) {
                    match &packet {
                        Some(p) => drop(handle.set_generic(p.clone())),
                        None => drop(handle.set_error(RmiError::SystemFaultOrTerminate)),
                    }
                }
                return;
            }
            Some((r, ReplyName::Other(name))) => stack
                .iter()
                .position(|h| !h.is_instruction() && h.packet_name() == name && h.is_echoed_by(r)),
        };
        let Some(handle) = at.and_then(|i| self.response_stack.remove(i)) else {
            tracing::warn!(
                response = %String::from_utf8_lossy(line),
                "No response handle to match incoming response"
            );
            return;
        };
        match parsed {
            Ok(packet) => {
                if let Some(sink) = &self.telemetry {
                    sink.received(&packet, crate::time_util::host_now());
                }
                let _ = handle.set_generic(packet);
            }
            Err(e) => {
                let _ = handle.set_error(e);
            }
        }
    }

    fn write_from_queue(&mut self, q: &mut VecDeque<PendingWrite>) -> bool {
        if !q.is_empty() {
            tracing::trace!("Writing to RMI tcp stream");
        }
        while q.front().is_some_and(|w| self.may_write(w)) {
            let Some(front) = q.front_mut() else {
                break;
            };
            let mut write_cnt = 0;
            loop {
                match self.tcp_stream.write(&front.buf[front.offset..]) {
                    Ok(0) => {
                        tracing::error!("RMI TCP stream write returned 0, peer closed connection");
                        return true; // peer closed
                    }
                    Ok(n) => {
                        write_cnt += 1;
                        front.offset += n;
                        if front.offset == front.buf.len() {
                            let is_disc = front.is_disconnect;
                            let handle = front.handle.clone();
                            if let (Some(sink), Some(packet)) = (&self.telemetry, &front.packet) {
                                sink.sent(packet, crate::time_util::host_now());
                            }
                            // push response stack entry once the whole request is on the wire
                            self.response_stack.push_back(handle);
                            q.pop_front();
                            if is_disc {
                                self.disconnect_deadline =
                                    Some(Instant::now() + Self::DISCONNECT_REPLY_WAIT);
                                return true;
                            }
                            break; // move to next message in queue
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        return false; // wait for next WRITABLE
                    }
                    Err(e) => {
                        tracing::error!(error = %e, "RMI TCP stream write error");
                        let _ = front.handle.set_error(RmiError::CommunicationError(e));
                        q.pop_front();
                        break;
                    }
                }
            }
            tracing::trace!(writes = write_cnt, "Wrote packet to RMI stream");
        }
        false
    }

    /// Whether a sent FRC_Disconnect has been answered, along with everything
    /// before it, or has waited as long as it may.
    fn disconnect_settled(&self) -> bool {
        self.disconnect_deadline
            .is_some_and(|d| self.response_stack.is_empty() || Instant::now() >= d)
    }

    /// Serves the connection until told to stop or it fails, then fails every
    /// request that never made it onto the wire.
    fn run(&mut self, poll: Poll) -> RmiResult<()> {
        let mut message_queue: VecDeque<PendingWrite> = VecDeque::new();
        let result = self.serve(poll, &mut message_queue);
        for unsent in message_queue.drain(..) {
            let _ = unsent.handle.set_error(RmiError::Disconnected);
        }
        result
    }

    fn serve(
        &mut self,
        mut poll: Poll,
        message_queue: &mut VecDeque<PendingWrite>,
    ) -> RmiResult<()> {
        let mut events = Events::with_capacity(128);
        let mut read_buf = vec![0u8; 8192];
        let mut connection_established = false;
        let mut exit_by: Option<Instant> = None;

        loop {
            let timeout = match self.disconnect_deadline {
                Some(d) => Some(
                    d.saturating_duration_since(Instant::now())
                        .min(Duration::from_millis(8)),
                ),
                None if message_queue.is_empty() => Some(Duration::from_millis(8)),
                None => Some(Duration::from_millis(96)),
            };
            poll.poll(&mut events, timeout)
                .map_err(RmiError::CommunicationError)?;

            if !self.handle.should_live() {
                let exit_by =
                    *exit_by.get_or_insert_with(|| Instant::now() + Self::DISCONNECT_REPLY_WAIT);
                self.fill_queue(message_queue);
                if connection_established {
                    self.write_from_queue(message_queue);
                }
                // A queued FRC_Disconnect still has to reach the controller,
                // even when the connection is still being established: keep
                // serving the socket until it is written, within limits.
                let disconnect_unsent =
                    message_queue.iter().any(|w| w.is_disconnect) && Instant::now() < exit_by;
                let awaiting_reply =
                    self.disconnect_deadline.is_some() && !self.disconnect_settled();
                if !disconnect_unsent && !awaiting_reply {
                    self.tcp_stream.shutdown(std::net::Shutdown::Both)?;
                    tracing::info!("RMI Runner thread told to exit");
                    return Ok(());
                }
            }

            for event in events.iter() {
                if event.is_writable()
                    && event.token() == RmiRunner::TOK_SOCKET
                    && !connection_established
                {
                    self.tcp_stream
                        .set_nodelay(true)
                        .map_err(|_| std::io::Error::other("Failed to set TCP_NODELAY"))
                        .map_err(RmiError::from)?;
                    connection_established = true;
                }
                match event.token() {
                    RmiRunner::TOK_WAKER if !self.fill_queue(message_queue) => {
                        continue;
                    }
                    RmiRunner::TOK_SOCKET => {
                        match self.read_stream(&mut read_buf) {
                            // The controller may close its end once it has
                            // answered FRC_Disconnect.
                            Err(RmiError::Disconnected) if self.disconnect_deadline.is_some() => {
                                self.tcp_stream.shutdown(std::net::Shutdown::Both).ok();
                                tracing::info!("RMI Runner thread terminating");
                                return Ok(());
                            }
                            r => r?,
                        }
                        if !self.fill_queue(message_queue) {
                            continue;
                        }
                    }
                    _ => {}
                }

                if connection_established && self.write_from_queue(message_queue) {
                    tracing::info!("Disconnect sent, waiting for the reply");
                }
            }

            if self.disconnect_settled() || !self.handle.is_alive() {
                self.tcp_stream.shutdown(std::net::Shutdown::Both)?;
                tracing::info!("RMI Runner thread terminating");
                return Ok(());
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn rmi_runner_runtime(
    handle: ThreadHandle,
    tcp_stream: TcpStream,
    poll: Poll,
    from_driver: Receiver<RunnerMessage>,
    config: RmiDriverConfig,
    thread: Vec<ThreadOption>,
    started: Sender<std::io::Result<()>>,
    telemetry: Option<RmiTelemetry>,
    err_flag: Arc<AtomicBool>,
) {
    let _tuning = match tuning::apply_thread(DRIVER, ThreadRole::Control, &thread) {
        Ok(report) => {
            let _ = started.send(Ok(()));
            report
        }
        Err(e) => {
            let _ = started.send(Err(e));
            return;
        }
    };
    if let Some(sink) = &telemetry {
        sink.warmup();
    }
    let mut runner = RmiRunner {
        handle,
        tcp_stream,
        from_driver,
        response_stack: VecDeque::with_capacity(config.buffer_cnt as usize),
        config,
        telemetry,
        rx_buf: Vec::new(),
        disconnect_deadline: None,
    };
    if let Err(e) = runner.run(poll) {
        tracing::error!(error = %e, "RMI runner terminated with error");
        err_flag.store(true, Ordering::Relaxed);
    }
    runner.handle.has_died();
    while let Some(pending) = runner.response_stack.pop_front() {
        let _ = pending.set_error(RmiError::Disconnected);
    }
}

/// Connection settings for an [`RmiDriver`].
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Debug, Clone)]
pub struct RmiDriverConfig {
    /// IP address of the robot controller.
    pub address: IpAddr,
    /// Minimum RMI major version the controller must report at connect.
    #[on(pyo3(get, set))]
    pub expected_major_version: u8,
    /// Installed software options, used to validate feature-gated packet fields.
    #[on(pyo3(get, set))]
    pub software_options: Vec<SoftwareOptions>,
    /// Maximum number of requests in flight before sends are held back.
    #[on(pyo3(get, set))]
    pub buffer_cnt: u8,
    /// Read/write timeout for the connect handshake.
    pub timeout: Duration,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl RmiDriverConfig {
    /// Config with sensible defaults: major version 7, no software options, 8-deep buffer, 2 s timeout.
    #[cfg(off)]
    pub fn default_with_ip<T: Into<IpAddr>>(address: T) -> Self {
        Self {
            address: address.into(),
            expected_major_version: 7,
            software_options: vec![],
            buffer_cnt: 8,
            timeout: Duration::from_secs(2),
        }
    }

    /// Builds a config, validating the buffer size and timeout.
    ///
    /// # Errors
    /// Returns [`RmiError::Structure`] if `buffer_cnt` is 0 or `timeout_secs` is not positive.
    #[cfg(on)]
    #[on(new)]
    #[on(pyo3(signature = (address, expected_major_version=7, software_options=None, buffer_cnt=8, timeout_secs=2.0)))]
    pub fn new(
        address: IpAddr,
        expected_major_version: u8,
        software_options: Option<Vec<SoftwareOptions>>,
        buffer_cnt: u8,
        timeout_secs: f64,
    ) -> RmiResult<Self> {
        if buffer_cnt == 0 {
            return Err(RmiError::Structure(
                "buffer_cnt must be at least 1".to_string(),
            ));
        }
        if timeout_secs <= 0.0 {
            return Err(RmiError::Structure(
                "timeout_secs must be positive".to_string(),
            ));
        }
        Ok(Self {
            address,
            expected_major_version,
            software_options: software_options.unwrap_or_default(),
            buffer_cnt,
            timeout: Duration::from_secs_f64(timeout_secs),
        })
    }

    #[cfg(on)]
    #[on(setter)]
    fn set_address(&mut self, address: &str) -> RmiResult<()> {
        self.address = address
            .parse()
            .map_err(|e| RmiError::Structure(format!("Invalid IP address: {}", e)))?;
        Ok(())
    }

    #[cfg(on)]
    #[on(getter)]
    fn get_address(&self) -> String {
        self.address.to_string()
    }

    #[cfg(on)]
    #[on(setter)]
    fn set_timeout_secs(&mut self, timeout_secs: f64) -> RmiResult<()> {
        if timeout_secs <= 0.0 {
            return Err(RmiError::Structure("Timeout must be positive".to_string()));
        }
        self.timeout = Duration::from_secs_f64(timeout_secs);
        Ok(())
    }

    #[cfg(on)]
    #[on(getter)]
    fn get_timeout_secs(&self) -> f64 {
        self.timeout.as_secs_f64()
    }
}

#[derive(Debug, Clone)]
enum RunnerMessage {
    SendPacket(Vec<u8>, RmiHandleGeneric, Option<SendPacket>),
    Disconnect(Vec<u8>, RmiHandleGeneric, bool),
}

impl RunnerMessage {
    fn into_pending_write(self) -> PendingWrite {
        match self {
            RunnerMessage::SendPacket(data, handle, packet) => PendingWrite {
                buf: data,
                offset: 0,
                handle,
                is_disconnect: false,
                packet,
            },
            RunnerMessage::Disconnect(data, handle, _) => PendingWrite {
                buf: data,
                offset: 0,
                handle,
                is_disconnect: true,
                packet: None,
            },
        }
    }

    fn is_priority(&self) -> bool {
        match self {
            RunnerMessage::SendPacket(_, _, _) => false,
            RunnerMessage::Disconnect(_, _, p) => *p,
        }
    }
}

#[derive(Debug)]
struct RmiConnection {
    major_version: u8,
    minor_version: u8,
    handle: ThreadHandle,
    to_runner: Sender<RunnerMessage>,
    err_flag: Arc<AtomicBool>,
}

/// Client for the FANUC Remote Motion Interface: a JSON-over-TCP protocol for sending
/// motion instructions and commands to the controller. Sends return immediately with a
/// handle that resolves once the controller's response arrives on the I/O thread.
#[derive(Debug)]
pub struct RmiDriver {
    config: RmiDriverConfig,
    seq: AtomicU32,
    connection: Option<RmiConnection>,
    telemetry: Option<RmiTelemetry>,
}

impl RmiDriver {
    /// Creates an unconnected driver; call [`connect`](Self::connect) before sending.
    pub fn new(config: RmiDriverConfig) -> Self {
        Self {
            config,
            seq: AtomicU32::new(2),
            connection: None,
            telemetry: None,
        }
    }

    /// Like [`new`](Self::new), with a telemetry sink observing [`SendPacket`]s
    /// at wire completion and decoded [`ResponsePacket`]s, from every connection
    /// this driver makes. The FRC_Connect/FRC_Disconnect
    /// handshake frames are raw JSON, not `SendPacket`s, and are not reported.
    pub fn new_with_telemetry<S: TelemetrySink<SendPacket, ResponsePacket>>(
        config: RmiDriverConfig,
        telemetry: S,
    ) -> Self {
        let mut driver = Self::new(config);
        driver.telemetry = Some(Arc::new(telemetry));
        driver
    }

    /// The controller's reported `(major, minor)` RMI version, or `None` if not connected.
    pub fn version(&self) -> Option<(u8, u8)> {
        self.connection
            .as_ref()
            .map(|c| (c.major_version, c.minor_version))
    }

    /// Performs the FRC_Connect handshake on port 16001 and spawns the I/O thread
    /// on the negotiated port.
    ///
    /// `thread` is applied by the I/O thread to itself before it starts. That
    /// thread serves request/response traffic, so it accepts `CpuAffinity`,
    /// `PrefaultStack`, `LinuxNice`, `UnixScheduler` with `Other`, `Batch` or
    /// `Idle`, `WinPriority` below `TimeCritical`, `WinDisablePowerThrottling`
    /// and `MacOsQos`. Real-time classes (`RtPriority`, `UnixScheduler` with
    /// `Fifo` or `RoundRobin`, `WinPriority(TimeCritical)`, `WinMmcss`,
    /// `MacOsTimeConstraint`) are refused: on a thread that blocks on TCP
    /// round-trips they only risk starving the rest of the system.
    /// Process-wide settings (memory locking, `cpu_dma_latency`, Windows
    /// priority class and timer resolution) are the application's to make with
    /// [`ProcessOption::apply_all`](fast_talker::options::ProcessOption::apply_all).
    ///
    /// `socket` is applied to both TCP connections once they are made, so it
    /// accepts only `Dscp` and `LinuxPriority`. `BindDevice` would have to
    /// precede the connect, buffer sizes would turn off TCP autotuning, and
    /// busy polling, `DontFragment` and `WinCpuAffinity` do nothing useful for
    /// this traffic.
    ///
    /// Options for another platform, or that this platform cannot do, are
    /// skipped with a warning.
    ///
    /// # Errors
    /// Fails if already connected, with [`RmiError::InvalidOption`] for an
    /// option this driver does not accept, on TCP or serde errors during the
    /// handshake, if applying an option fails, if the controller rejects the
    /// connect with an error code, or if its major version is below
    /// [`RmiDriverConfig::expected_major_version`].
    pub fn connect(
        &mut self,
        thread: &[ThreadOption],
        socket: &[SocketOption],
    ) -> RmiResult<FrcConnectResponse> {
        tracing::info!(addr = %self.config.address, "Attempting to connect RmiDriver");
        if self.connection.is_some() {
            tracing::warn!(
                addr = %self.config.address,
                "RmiDriver::connect called but already connected"
            );
            return Err(RmiError::Structure("Driver already started".to_string()));
        }
        tuning::check_thread(DRIVER, ThreadRole::Control, thread)?;
        tuning::check_socket(DRIVER, SocketRole::TcpControl, socket)?;

        let (to_runner, from_driver) = flume::unbounded();

        let deadline = Instant::now() + self.config.timeout;
        let mut tcp = StdTcpStream::connect_timeout(
            &SocketAddr::new(self.config.address, 16001),
            self.config.timeout,
        )?;
        tuning::apply_socket(DRIVER, SocketRole::TcpControl, &tcp, socket)?;

        tcp.set_write_timeout(Some(self.config.timeout))?;
        tcp.set_nodelay(true)?;
        let connect_bytes = rmi_string_writer(connect_json())?;
        tcp.write_all(&connect_bytes)?;
        tcp.flush()?;
        let response_bytes = read_line_by(&mut tcp, deadline)?;
        let response_str = rmi_string_reader(&response_bytes)?;
        tracing::debug!(response = ?response_str, "Connect response");
        let response = match response_str {
            VariadicString::Single(s) => serde_json::from_str::<FrcConnectResponse>(&s)?,
            other => {
                return Err(RmiError::Structure(format!(
                    "Expected single response from FRC_Connect, got: {:?}",
                    other
                )));
            }
        };
        if response.error_id != 0 {
            let ec = RmiProtocolError::try_from(response.error_id).unwrap_or(Default::default());
            tracing::error!(error = %ec, "RMI connect rejected by robot");
            return Err(RmiError::FanucErrorCode(ec));
        }
        let major_version = response.major_version as u8;
        let minor_version = response.minor_version as u8;
        if major_version < self.config.expected_major_version {
            tracing::error!(
                robot_major = major_version,
                robot_minor = minor_version,
                expected_major = self.config.expected_major_version,
                "RMI version mismatch"
            );
            return Err(RmiError::Initialization(format!(
                "Robot major version {} is lower than expected {}",
                major_version, self.config.expected_major_version
            )));
        }

        let mut handle = ThreadHandle::new();
        let session_timeout = deadline.saturating_duration_since(Instant::now());
        if session_timeout.is_zero() {
            return Err(std::io::Error::from(std::io::ErrorKind::TimedOut).into());
        }
        let (join_handle, waker, err_flag) = RmiRunner::start(
            SocketAddr::new(self.config.address, response.port_number),
            session_timeout,
            handle.to_pass_in(),
            from_driver,
            self.config.clone(),
            thread.to_vec(),
            socket,
            self.telemetry.clone(),
        )?;
        handle.set_handle(join_handle);
        handle.set_waker_mio(waker);

        self.seq.store(0, Ordering::Relaxed);
        self.connection = Some(RmiConnection {
            major_version,
            minor_version,
            handle,
            to_runner,
            err_flag,
        });

        tracing::info!(addr = %self.config.address, "RmiDriver connected");

        Ok(response)
    }

    /// Sends FRC_Disconnect and joins the I/O thread, blocking until it exits.
    ///
    /// # Errors
    /// Returns [`RmiError::Disconnected`] if not connected, or a communication
    /// error if the disconnect packet cannot be queued to the I/O thread.
    pub fn disconnect(&mut self) -> RmiResult<RmiHandle<FrcDisconnectResponse>> {
        if let Some(conn) = self.connection.take() {
            tracing::info!(addr = %self.config.address, "RmiDriver disconnecting");
            let data = rmi_string_writer(disconnect_json())?;
            let resp_handle = RmiHandleGeneric::new("FRC_Disconnect", 0);
            let specific_handle = RmiHandle::new_from_generic(&resp_handle);
            conn.to_runner
                .send(RunnerMessage::Disconnect(data, resp_handle, true))
                .map_err(|e| RmiError::CommunicationError(std::io::Error::other(e)))?;
            let _ = conn.handle.wake();
            conn.handle.join();
            tracing::info!(addr = %self.config.address, "RmiDriver disconnected");
            Ok(specific_handle)
        } else {
            Err(RmiError::Disconnected)
        }
    }

    fn get_connection(&self) -> RmiResult<&RmiConnection> {
        let cnx = self.connection.as_ref().ok_or(RmiError::Disconnected)?;
        if cnx.handle.is_alive() {
            Ok(cnx)
        } else {
            tracing::error!(
                addr = %self.config.address,
                "RMI runner thread is dead, connection lost"
            );
            Err(RmiError::Disconnected)
        }
    }

    /// Whether the driver is connected and its I/O thread is still alive.
    pub fn is_connected(&self) -> bool {
        self.get_connection().is_ok()
    }

    /// Whether the I/O thread stopped on an error.
    pub fn has_connection_errored(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.err_flag.load(Ordering::Relaxed)
        } else {
            false
        }
    }

    /// Clears the connection state if the I/O thread has died.
    ///
    /// # Errors
    /// Returns [`RmiError::Disconnected`] if not connected or the I/O thread has exited.
    pub fn update_connection(&mut self) -> RmiResult<()> {
        if let Some(conn) = &self.connection {
            if !conn.handle.is_alive() {
                self.connection = None;
                return Err(RmiError::Disconnected);
            }
            Ok(())
        } else {
            Err(RmiError::Disconnected)
        }
    }

    /// The SequenceID for the next instruction. IDs count from 1 on each
    /// connection, and 0 is never one, so the counter steps from `u32::MAX`
    /// back to 1.
    fn next_sequence_id(&self) -> u32 {
        let step = |seq: u32| seq.checked_add(1).unwrap_or(1);
        let previous = self
            .seq
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |seq| Some(step(seq)))
            .unwrap_or_else(|seq| seq);
        step(previous)
    }

    /// Queues a type-erased packet for sending, assigning instructions a sequence ID,
    /// and returns a handle that resolves with the controller's response.
    ///
    /// # Errors
    /// Fails if not connected, if the packet fails to serialize, if a field is
    /// rejected by feature-gate validation (version or software option), or if
    /// handing the packet to the I/O thread fails.
    #[inline(never)]
    pub fn send_generic(&self, mut packet: SendPacket) -> RmiResult<RmiHandleGeneric> {
        let conn = self.get_connection()?;
        let seq_id = if let SendPacket::Instruction(inst) = &mut packet {
            let seq = self.next_sequence_id();
            inst.set_seq_id(seq);
            seq
        } else {
            0
        };
        let json_value = serde_json::to_value(&packet)?;
        let content = if let JsonValue::Object(map) = json_value {
            map
        } else {
            return Err(RmiError::Structure(
                "Packet did not serialize to a JSON object".to_string(),
            ));
        };
        let generic_handle = RmiHandleGeneric::new(packet.packet_name(), seq_id).echoing(&content);
        validate_gates(&content, conn.major_version, &self.config.software_options)?;

        tracing::debug!(
            packet = %serde_json::to_string_pretty(&content)?,
            "Sending packet to runner"
        );
        conn.to_runner
            .send(RunnerMessage::SendPacket(
                rmi_string_writer(content)?,
                generic_handle.clone(),
                Some(packet),
            ))
            .map_err(|e| RmiError::CommunicationError(std::io::Error::other(e)))?;
        conn.handle.wake()?;
        Ok(generic_handle)
    }

    /// Queues a packet for sending, returning a handle typed to its response counterpart.
    ///
    /// # Errors
    /// Same as [`send_generic`](Self::send_generic).
    #[inline]
    pub fn send<P: SendablePacket>(&self, packet: P) -> RmiResult<RmiHandle<P::Counterpart>> {
        // this is to reduce the extra code from monomorphization
        Ok(RmiHandle::new_from_generic(
            &self.send_generic(packet.into())?,
        ))
    }

    /// Resets, aborts, then resets again to clear faults and any queued motion,
    /// waiting on the first two responses before issuing the final reset.
    ///
    /// # Errors
    /// Same as [`send_generic`](Self::send_generic).
    pub fn send_full_reset(&self) -> RmiResult<RmiHandle<FrcResetResponse>> {
        let _ = self.send(FrcReset)?.wait();
        let _ = self.send(FrcAbort)?.wait();
        self.send(FrcReset)
    }
}

#[cfg(feature = "py")]
pub(super) mod py {
    use pyo3::{prelude::*, types::PyType};

    use crate::rmi::{
        proto::{commands::FrcReset, communication::FrcDisconnect},
        rmi_handle::py::PyRmiHandleGeneric,
    };

    use super::*;

    #[pyclass(name = "RmiDriver")]
    pub struct PyRmiDriver {
        inner: RmiDriver,
    }

    #[pymethods]
    impl PyRmiDriver {
        #[new]
        pub fn new(config: RmiDriverConfig) -> Self {
            Self {
                inner: RmiDriver::new(config),
            }
        }
        #[pyo3(signature = (thread = None, socket = None))]
        pub fn connect(
            &mut self,
            py: Python<'_>,
            thread: Option<fast_talker::py::ThreadOptions>,
            socket: Option<fast_talker::py::SocketOptions>,
        ) -> PyResult<FrcConnectResponse> {
            let (thread, socket) = (thread.unwrap_or_default(), socket.unwrap_or_default());
            py.detach(|| self.inner.connect(&thread, &socket))
                .map_err(Into::into)
        }
        pub fn disconnect(&mut self, py: Python<'_>) -> PyResult<PyRmiHandleGeneric> {
            py.detach(|| self.inner.disconnect())
                .map_err(Into::into)
                .map(|inner| PyRmiHandleGeneric {
                    inner: inner.generic(),
                    pytype: FrcDisconnect::counterpart_typeinfo(py).unbind(),
                })
        }
        pub fn is_connected(&self) -> bool {
            self.inner.is_connected()
        }
        pub fn has_connection_errored(&self) -> bool {
            self.inner.has_connection_errored()
        }
        pub fn version(&self) -> Option<(u8, u8)> {
            self.inner.version()
        }
        pub fn send(&self, packet: Bound<PyAny>) -> PyResult<PyRmiHandleGeneric> {
            // self.inner.send_generic(packet).map_err(Into::into).map(PyResponseHandleGeneric::from)
            // call `into_send_packet` method of packet
            let send_packet: SendPacket = packet.call_method0("into_send_packet")?.extract()?;
            // call counterpart_typeinfo static method of packet to get the type of the counterpart
            let pytype: Py<PyType> = packet
                .get_type()
                .call_method0("counterpart_typeinfo")?
                .extract()?;
            self.inner
                .send_generic(send_packet)
                .map_err(Into::into)
                .map(|inner| PyRmiHandleGeneric { inner, pytype })
        }
        pub fn send_full_reset(&self, py: Python<'_>) -> PyResult<PyRmiHandleGeneric> {
            py.detach(|| self.inner.send_full_reset())
                .map_err(Into::into)
                .map(|inner| PyRmiHandleGeneric {
                    inner: inner.generic(),
                    pytype: FrcReset::counterpart_typeinfo(py).unbind(),
                })
        }
    }

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<PyRmiDriver>()?;
        parent_module.add_class::<RmiDriverConfig>()?;
        Ok(())
    }
}

#[cfg(test)]
mod fuzz_test;
