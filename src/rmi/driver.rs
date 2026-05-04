use std::{
    collections::VecDeque,
    io::{Read, Write},
    net::{IpAddr, SocketAddr},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU32, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

use cfg_mixin::cfg_mixin;
use flume::{Receiver, Sender};
use serde_json::{Map as JsonMap, Value as JsonValue};
use snare::mio::{Events, Interest, Poll, Token, Waker, net::TcpStream};
use snare::net::TcpStream as StdTcpStream;

use crate::{
    rmi::{
        FeatureGates, FeatureLockEntry, SendPacket, SendablePacket, SoftwareOptions,
        errors::{RmiError, RmiProtocolError, RmiResult},
        proto::{
            commands::{FrcAbort, FrcReset, FrcResetResponse},
            communication::{
                FrcConnectResponse, FrcDisconnectResponse, connect_json, disconnect_json,
            },
        },
        rmi_handle::*,
    },
    thread_util::{GeneralThreadError, ThreadConfig, ThreadHandle},
};

type JsonObject = JsonMap<String, JsonValue>;

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

impl VariadicString {
    fn vec(self) -> Vec<String> {
        match self {
            VariadicString::None => vec![],
            VariadicString::Single(s) => vec![s],
            VariadicString::Multiple(v) => v,
        }
    }

    fn len(&self) -> usize {
        match self {
            VariadicString::None => 0,
            VariadicString::Single(_) => 1,
            VariadicString::Multiple(v) => v.len(),
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
}

struct RmiRunner {
    handle: ThreadHandle,
    tcp_stream: TcpStream,
    from_driver: Receiver<RunnerMessage>,
    response_stack: VecDeque<RmiHandleGeneric>,
    config: RmiDriverConfig,
}

impl RmiRunner {
    const TOK_SOCKET: Token = Token(0);
    const TOK_WAKER: Token = Token(1);

    fn start(
        addr: SocketAddr,
        handle: ThreadHandle,
        from_driver: Receiver<RunnerMessage>,
        config: RmiDriverConfig,
        thread_config: Option<ThreadConfig>,
    ) -> RmiResult<(JoinHandle<()>, Arc<Waker>, Arc<AtomicBool>)> {
        let tcp_stream = TcpStream::connect(addr)?;
        #[cfg(test)]
        tcp_stream.set_nonblocking(true)?;
        let (waker_tx, waker_rx) = flume::unbounded();
        let local_err_flag = Arc::new(AtomicBool::new(false));
        let thread_err_flag = local_err_flag.clone();
        let handle = snare::thread::Builder::new()
            .name("fanuc-rmi-runner".to_string())
            .spawn(move || {
                if let Err(e) = rmi_runner_runtime(
                    handle,
                    tcp_stream,
                    from_driver,
                    config,
                    thread_config,
                    waker_tx,
                ) {
                    log::error!("RMI runner thread setup failed: {:?}", e);
                    thread_err_flag.store(true, Ordering::Relaxed);
                }
            })?;
        let waker = waker_rx
            .recv()
            .map_err(|e| RmiError::CommunicationError(std::io::Error::other(e)))?;
        Ok((handle, waker, local_err_flag))
    }

    fn fill_queue(&mut self, message_queue: &mut VecDeque<PendingWrite>) -> bool {
        while let Ok(msg) = self.from_driver.try_recv() {
            if msg.is_priority() {
                message_queue.push_front(msg.into_pending_write());
            } else {
                message_queue.push_back(msg.into_pending_write());
            }
        }
        self.response_stack.len() < self.config.buffer_cnt as usize
    }

    fn read_stream(&mut self, buf: &mut [u8]) -> RmiResult<()> {
        loop {
            match self.tcp_stream.read(buf) {
                Ok(0) => {
                    log::error!("RMI TCP stream read returned 0 bytes, connection lost");
                    return Err(RmiError::Disconnected);
                }
                Ok(n) => {
                    log::trace!("Read {} bytes", n);
                    match rmi_string_reader(&buf[..n]) {
                        Ok(variadic) => {
                            log::trace!("Variadic string has {} parts", variadic.len());
                            for json_str in variadic.vec() {
                                if json_str.trim().is_empty() {
                                    continue;
                                }
                                log::debug!("RMI Runner received response:\n{}", json_str);
                                if let Some(resp_handle) = self.response_stack.front() {
                                    match serde_json::from_str(&json_str) {
                                        Ok(packet) => {
                                            let _ = resp_handle.set_generic(packet);
                                        }
                                        Err(e) => {
                                            let _ = resp_handle.set_error(e.into());
                                        }
                                    }
                                    self.response_stack.pop_front();
                                } else {
                                    log::warn!(
                                        "No response handle to match incoming response: {}",
                                        json_str
                                    );
                                    break;
                                }
                            }
                        }
                        Err(e) => {
                            log::warn!("Failed to read RMI response: {}", e);
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(e) => {
                    log::error!("RMI TCP stream read error: {}", e);
                    return Err(RmiError::CommunicationError(e));
                }
            }
        }
    }

    fn write_from_queue(&mut self, q: &mut VecDeque<PendingWrite>) -> bool {
        if self.response_stack.len() >= self.config.buffer_cnt as usize {
            return false;
        }
        if !q.is_empty() {
            log::trace!("Writing to RMI tcp stream");
        }
        while let Some(front) = q.front_mut() {
            let mut write_cnt = 0;
            loop {
                match self.tcp_stream.write(&front.buf[front.offset..]) {
                    Ok(0) => {
                        log::error!("RMI TCP stream write returned 0, peer closed connection");
                        return true; // peer closed
                    }
                    Ok(n) => {
                        write_cnt += 1;
                        front.offset += n;
                        if front.offset == front.buf.len() {
                            let is_disc = front.is_disconnect;
                            let handle = front.handle.clone();
                            // push response stack entry once the whole request is on the wire
                            self.response_stack.push_back(handle);
                            q.pop_front();
                            if is_disc {
                                self.handle.has_died();
                                return true;
                            }
                            break; // move to next message in queue
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        return false; // wait for next WRITABLE
                    }
                    Err(e) => {
                        log::error!("RMI TCP stream write error: {}", e);
                        let _ = front.handle.set_error(RmiError::CommunicationError(e));
                        q.pop_front();
                        break;
                    }
                }
            }
            log::trace!("Wrote {} times for one packet", write_cnt);
            if self.response_stack.len() >= self.config.buffer_cnt as usize {
                break;
            }
        }
        false
    }

    fn run(&mut self, mut poll: Poll) -> RmiResult<()> {
        let mut events = Events::with_capacity(128);
        let mut read_buf = vec![0u8; 8192];
        let mut message_queue: VecDeque<PendingWrite> = VecDeque::new();
        let mut connection_established = false;

        loop {
            let timeout = if message_queue.is_empty() {
                Duration::from_millis(8)
            } else {
                Duration::from_millis(96)
            };
            poll.poll(&mut events, Some(timeout))
                .map_err(RmiError::CommunicationError)?;

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
                    RmiRunner::TOK_WAKER => {
                        if !self.fill_queue(&mut message_queue) {
                            continue;
                        }
                    }
                    RmiRunner::TOK_SOCKET => {
                        self.read_stream(&mut read_buf)?;
                        if !self.fill_queue(&mut message_queue) {
                            continue;
                        }
                    }
                    _ => {}
                }

                if connection_established && self.write_from_queue(&mut message_queue) {
                    log::info!("Disconnect sent, terminating runner");
                    break;
                }
            }

            if !self.handle.is_alive() {
                self.tcp_stream.shutdown(std::net::Shutdown::Both)?;
                log::info!("RMI Runner thread terminating");
                return Ok(());
            }
        }
    }
}

fn rmi_runner_runtime(
    handle: ThreadHandle,
    mut tcp_stream: TcpStream,
    from_driver: Receiver<RunnerMessage>,
    config: RmiDriverConfig,
    thread_config: Option<ThreadConfig>,
    waker_tx: Sender<Arc<Waker>>,
) -> Result<(), GeneralThreadError> {
    if let Some(cfg) = thread_config {
        cfg.configure_this_thread_print_failure();
    }
    let poll = Poll::new().map_err(|_| GeneralThreadError::FailedToCreatePoll)?;
    poll.registry()
        .register(
            &mut tcp_stream,
            RmiRunner::TOK_SOCKET,
            Interest::READABLE.add(Interest::WRITABLE),
        )
        .map_err(|_| GeneralThreadError::FailedSocketRegistry)?;
    let waker = Arc::new(
        Waker::new(poll.registry(), RmiRunner::TOK_WAKER)
            .map_err(|_| GeneralThreadError::FailedWakerCreation)?,
    );
    waker_tx.send(waker.clone())?;
    let mut runner = RmiRunner {
        handle,
        tcp_stream,
        from_driver,
        response_stack: VecDeque::with_capacity(config.buffer_cnt as usize),
        config,
    };
    if let Err(e) = runner.run(poll) {
        log::error!("RMI runner terminated with error: {}", e);
    }
    Ok(())
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Debug, Clone)]
pub struct RmiDriverConfig {
    pub address: IpAddr,
    #[on(pyo3(get, set))]
    pub expected_major_version: u8,
    #[on(pyo3(get, set))]
    pub software_options: Vec<SoftwareOptions>,
    #[on(pyo3(get, set))]
    pub buffer_cnt: u8,
    pub timeout: Duration,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl RmiDriverConfig {
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
    SendPacket(Vec<u8>, RmiHandleGeneric),
    Disconnect(Vec<u8>, RmiHandleGeneric, bool),
}

impl RunnerMessage {
    fn into_pending_write(self) -> PendingWrite {
        match self {
            RunnerMessage::SendPacket(data, handle) => PendingWrite {
                buf: data,
                offset: 0,
                handle,
                is_disconnect: false,
            },
            RunnerMessage::Disconnect(data, handle, _) => PendingWrite {
                buf: data,
                offset: 0,
                handle,
                is_disconnect: true,
            },
        }
    }

    fn is_priority(&self) -> bool {
        match self {
            RunnerMessage::SendPacket(_, _) => false,
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

#[derive(Debug)]
pub struct RmiDriver {
    config: RmiDriverConfig,
    seq: AtomicU32,
    connection: Option<RmiConnection>,
}

impl RmiDriver {
    pub fn new(config: RmiDriverConfig) -> Self {
        Self {
            config,
            seq: AtomicU32::new(2),
            connection: None,
        }
    }

    pub fn version(&self) -> Option<(u8, u8)> {
        self.connection
            .as_ref()
            .map(|c| (c.major_version, c.minor_version))
    }

    pub fn connect(
        &mut self,
        thread_config: Option<ThreadConfig>,
    ) -> RmiResult<FrcConnectResponse> {
        log::info!("Attempting to connect RmiDriver to {}", self.config.address);
        if self.connection.is_some() {
            log::warn!(
                "RmiDriver::connect called but already connected to {}",
                self.config.address
            );
            return Err(RmiError::Structure("Driver already started".to_string()));
        }

        let (to_runner, from_driver) = flume::unbounded();

        let mut tcp = StdTcpStream::connect(SocketAddr::new(self.config.address, 16001))?;

        tcp.set_write_timeout(Some(self.config.timeout))?;
        tcp.set_read_timeout(Some(self.config.timeout))?;
        tcp.set_nodelay(true)?;
        let connect_bytes = rmi_string_writer(connect_json())?;
        tcp.write_all(&connect_bytes)?;
        tcp.flush()?;
        let response_bytes = {
            let mut buf = vec![0u8; 4096];
            let n = tcp.read(&mut buf)?;
            buf.truncate(n);
            buf
        };
        let response_str = rmi_string_reader(&response_bytes)?;
        log::debug!("Connect response: {}", response_str);
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
            log::error!("RMI connect rejected by robot: {}", ec);
            return Err(RmiError::FanucErrorCode(ec));
        }
        let major_version = response.major_version as u8;
        let minor_version = response.minor_version as u8;
        if major_version < self.config.expected_major_version {
            log::error!(
                "RMI version mismatch: robot v{}.{} < expected v{}",
                major_version,
                minor_version,
                self.config.expected_major_version
            );
            return Err(RmiError::Initialization(format!(
                "Robot major version {} is lower than expected {}",
                major_version, self.config.expected_major_version
            )));
        }

        let mut handle = ThreadHandle::new();
        let (join_handle, waker, err_flag) = RmiRunner::start(
            SocketAddr::new(self.config.address, response.port_number),
            handle.to_pass_in(),
            from_driver,
            self.config.clone(),
            thread_config,
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

        log::info!("RmiDriver connected to {}", self.config.address);

        Ok(response)
    }

    pub fn disconnect(&mut self) -> RmiResult<RmiHandle<FrcDisconnectResponse>> {
        if let Some(conn) = self.connection.take() {
            log::info!("RmiDriver disconnecting from {}", self.config.address);
            let data = rmi_string_writer(disconnect_json())?;
            let resp_handle = RmiHandleGeneric::new("FRC_Disconnect", 0);
            let specific_handle = RmiHandle::new_from_generic(&resp_handle);
            conn.to_runner
                .send(RunnerMessage::Disconnect(data, resp_handle, true))
                .map_err(|e| RmiError::CommunicationError(std::io::Error::other(e)))?;
            conn.handle.join();
            log::info!("RmiDriver disconnected from {}", self.config.address);
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
            log::error!(
                "RMI runner thread is dead, connection to {} lost",
                self.config.address
            );
            Err(RmiError::Disconnected)
        }
    }

    pub fn is_connected(&self) -> bool {
        self.get_connection().is_ok()
    }

    pub fn has_connection_errored(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.err_flag.load(Ordering::Relaxed)
        } else {
            false
        }
    }

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

    #[inline(never)]
    pub fn send_generic(&self, mut packet: SendPacket) -> RmiResult<RmiHandleGeneric> {
        let conn = self.get_connection()?;
        let seq_id = if let SendPacket::Instruction(inst) = &mut packet {
            let seq = self.seq.fetch_add(1, Ordering::Relaxed);
            inst.set_seq_id(seq + 1);
            seq
        } else {
            0
        };
        let generic_handle = RmiHandleGeneric::new(packet.packet_name(), seq_id);
        let json_value = serde_json::to_value(&packet)?;
        let content = if let JsonValue::Object(map) = json_value {
            map
        } else {
            return Err(RmiError::Structure(
                "Packet did not serialize to a JSON object".to_string(),
            ));
        };
        validate_gates(&content, conn.major_version, &self.config.software_options)?;

        log::debug!(
            "Sending packet to runner: {}",
            serde_json::to_string_pretty(&content)?
        );
        conn.to_runner
            .send(RunnerMessage::SendPacket(
                rmi_string_writer(content)?,
                generic_handle.clone(),
            ))
            .map_err(|e| RmiError::CommunicationError(std::io::Error::other(e)))?;
        conn.handle.wake()?;
        Ok(generic_handle)
    }

    #[inline]
    pub fn send<P: SendablePacket>(&self, packet: P) -> RmiResult<RmiHandle<P::Counterpart>> {
        // this is to reduce the extra code from monomorphization
        Ok(RmiHandle::new_from_generic(
            &self.send_generic(packet.into())?,
        ))
    }

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
        #[pyo3(signature = (thread_config=None))]
        pub fn connect(
            &mut self,
            thread_config: Option<ThreadConfig>,
        ) -> PyResult<FrcConnectResponse> {
            self.inner.connect(thread_config).map_err(Into::into)
        }
        pub fn disconnect(&mut self, py: Python<'_>) -> PyResult<PyRmiHandleGeneric> {
            self.inner
                .disconnect()
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
            self.inner
                .send_full_reset()
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
