use fast_talker::options::{SocketOption, ThreadOption};
use flume::{Receiver, Sender};
use std::collections::{HashMap, VecDeque};
use std::io::{ErrorKind, Read, Write};
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use mio::{Events, Interest, Poll, Token, Waker, net::TcpStream};

use crate::hmi::proto::wire::{Body, Header, Message};
use crate::hmi::{BINCODE_CFG, DRIVER, DriverResult, HmiError, HmiTelemetry};
use crate::thread_util::ThreadHandle;
use crate::tuning::{self, OptionsReport, SocketRole, ThreadRole, TuningReport};

use super::hmi_handle::{HmiHandleGeneric, HmiResult};

#[derive(Debug, Clone)]
pub(super) enum RunnerMessage {
    Send {
        seq: u8,
        data: Vec<u8>,
        handle: HmiHandleGeneric,
        message: Message,
    },
    Shutdown,
}

#[derive(Debug, Clone)]
pub(super) struct PendingWrite {
    buf: Vec<u8>,
    offset: usize,
    seq: u8,
    handle: HmiHandleGeneric,
    message: Message,
}

pub(super) struct HmiRunner {
    handle: ThreadHandle,
    tcp_stream: TcpStream,
    from_driver: Receiver<RunnerMessage>,
    pending_responses: HashMap<u8, HmiHandleGeneric>,
    /// Writes waiting for a sequence number to free up: every one of the 256 is
    /// still awaiting its response.
    held: VecDeque<PendingWrite>,
    read_buffer: Vec<u8>,
    shutting_down: bool,
    telemetry: Option<HmiTelemetry>,
}

/// A runner thread that applied its options and is serving the connection.
pub(super) struct StartedRunner {
    pub join: std::thread::JoinHandle<()>,
    pub waker: Arc<Waker>,
    pub err_flag: Arc<AtomicBool>,
    pub tuning: TuningReport,
}

impl HmiRunner {
    const TOK_SOCKET: Token = Token(0);
    const TOK_WAKER: Token = Token(1);

    #[allow(clippy::too_many_arguments)]
    pub(super) fn start(
        addr: SocketAddr,
        connect_timeout: Duration,
        handle: ThreadHandle,
        from_driver: Receiver<RunnerMessage>,
        thread: Vec<ThreadOption>,
        socket: &[SocketOption],
        telemetry: Option<HmiTelemetry>,
    ) -> DriverResult<StartedRunner> {
        let (std_stream, socket_report) = tuning::connect_tcp(
            DRIVER,
            SocketRole::TcpControl,
            addr,
            connect_timeout,
            socket,
        )
        .map_err(HmiError::from)?;
        std_stream.set_nonblocking(true)?;
        let mut tcp_stream = TcpStream::from_std(std_stream);
        tracing::trace!(addr = %addr, "HMI runner connected");
        let poll = Poll::new().map_err(HmiError::from)?;
        poll.registry()
            .register(
                &mut tcp_stream,
                HmiRunner::TOK_SOCKET,
                Interest::READABLE.add(Interest::WRITABLE),
            )
            .map_err(HmiError::from)?;
        let waker =
            Arc::new(Waker::new(poll.registry(), HmiRunner::TOK_WAKER).map_err(HmiError::from)?);
        let local_err_flag = Arc::new(AtomicBool::new(false));
        let thread_err_flag = local_err_flag.clone();
        let (started_tx, started_rx) = flume::bounded(1);
        let join_handle = std::thread::Builder::new()
            .name("fanuc-hmi-runner".to_string())
            .spawn(move || {
                hmi_runner_runtime(
                    handle,
                    tcp_stream,
                    poll,
                    from_driver,
                    thread,
                    started_tx,
                    telemetry,
                    thread_err_flag,
                )
            })
            .map_err(HmiError::from)?;
        let started = started_rx
            .recv()
            .unwrap_or_else(|_| Err(std::io::Error::other("HMI runner exited during startup")));
        let thread_report = match started {
            Ok(report) => report,
            Err(e) => {
                let _ = join_handle.join();
                return Err(HmiError::from(e).into());
            }
        };
        tracing::trace!("HMI runner started");
        let tuning = TuningReport {
            thread: thread_report,
            socket: OptionsReport::from(&socket_report),
        };
        Ok(StartedRunner {
            join: join_handle,
            waker,
            err_flag: local_err_flag,
            tuning,
        })
    }

    fn run(&mut self, mut poll: Poll, queue: &mut VecDeque<PendingWrite>) -> HmiResult<()> {
        let mut events = Events::with_capacity(64);
        let mut scratch = [0u8; 2048];
        let mut connection_established = false;
        let timeout = Some(Duration::from_millis(96));

        loop {
            self.drain_channel(queue)?;
            if self.shutting_down {
                break;
            }

            if (!queue.is_empty() || !self.held.is_empty()) && connection_established {
                self.write_from_queue(queue)?;
            }

            poll.poll(&mut events, timeout).map_err(HmiError::from)?;

            for event in events.iter() {
                if event.is_writable()
                    && event.token() == HmiRunner::TOK_SOCKET
                    && !connection_established
                {
                    self.tcp_stream
                        .set_nodelay(true)
                        .map_err(|_| std::io::Error::other("Failed to set TCP_NODELAY"))
                        .map_err(HmiError::from)?;
                    connection_established = true;
                }
                match event.token() {
                    HmiRunner::TOK_WAKER => {
                        self.drain_channel(queue)?;
                    }
                    HmiRunner::TOK_SOCKET => {
                        if event.is_readable() {
                            self.read_stream(&mut scratch)?;
                        }
                        self.drain_channel(queue)?;
                    }
                    _ => {}
                }
            }

            if (!queue.is_empty() || !self.held.is_empty()) && connection_established {
                self.write_from_queue(queue)?;
            }

            if self.shutting_down {
                break;
            }

            if !self.handle.should_live()
                && queue.is_empty()
                && self.held.is_empty()
                && self.pending_responses.is_empty()
            {
                break;
            }
        }
        Ok(())
    }

    fn drain_channel(&mut self, queue: &mut VecDeque<PendingWrite>) -> HmiResult<()> {
        while let Ok(msg) = self.from_driver.try_recv() {
            match msg {
                RunnerMessage::Send {
                    seq,
                    data,
                    handle,
                    message,
                } => {
                    queue.push_back(PendingWrite {
                        buf: data,
                        offset: 0,
                        seq,
                        handle,
                        message,
                    });
                }
                RunnerMessage::Shutdown => {
                    self.shutting_down = true;
                    break;
                }
            }
        }
        Ok(())
    }

    fn write_from_queue(&mut self, queue: &mut VecDeque<PendingWrite>) -> HmiResult<()> {
        tracing::trace!("Writing to HMI tcp stream");
        while let Some(write) = self.held.pop_back() {
            queue.push_front(write);
        }
        while let Some(front) = queue.front_mut() {
            // Responses are routed by sequence number alone, so a request may
            // not go out under one that is still awaiting its response.
            if front.offset == 0 && self.pending_responses.contains_key(&front.seq) {
                let free = (1..=u8::MAX)
                    .map(|k| front.seq.wrapping_add(k))
                    .find(|seq| !self.pending_responses.contains_key(seq));
                let Some(seq) = free else {
                    tracing::debug!(
                        held = queue.len(),
                        "All 256 HMI sequence numbers are awaiting responses; holding writes"
                    );
                    self.held.extend(queue.drain(..));
                    break;
                };
                front.message.set_seq(seq);
                front.buf = bincode::encode_to_vec(&front.message, BINCODE_CFG)?;
                front.seq = seq;
            }
            match self.tcp_stream.write(&front.buf[front.offset..]) {
                Ok(0) => {
                    tracing::error!("HMI TCP write returned 0, peer closed connection");
                    return Err(HmiError::NotConnected);
                }
                Ok(n) => {
                    front.offset += n;
                    if front.offset == front.buf.len() {
                        if let Some(sink) = &self.telemetry {
                            sink.sent(&front.message, crate::time_util::host_now());
                        }
                        self.pending_responses
                            .insert(front.seq, front.handle.clone());
                        queue.pop_front();
                    }
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => {
                    let handle = front.handle.clone();
                    queue.pop_front();
                    tracing::error!(error = %e, "HMI TCP write error");
                    let _ = handle.set_error(HmiError::Io(e));
                }
            }
        }
        Ok(())
    }

    fn read_stream(&mut self, scratch: &mut [u8]) -> HmiResult<()> {
        loop {
            match self.tcp_stream.read(scratch) {
                Ok(0) => {
                    tracing::error!("HMI TCP read returned 0, connection lost");
                    return Err(HmiError::NotConnected);
                }
                Ok(n) => {
                    self.read_buffer.extend_from_slice(&scratch[..n]);
                    self.process_buffer()?;
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return Ok(()),
                Err(e) => {
                    tracing::error!(error = %e, "HMI TCP read error");
                    return Err(HmiError::Io(e));
                }
            }
        }
    }

    fn process_buffer(&mut self) -> HmiResult<()> {
        loop {
            if self.read_buffer.len() < 42 {
                break;
            }
            let (header, _consumed) =
                bincode::decode_from_slice::<Header, _>(&self.read_buffer[..42], BINCODE_CFG)?;
            let total = 56 + header.payload_len() as usize;
            if self.read_buffer.len() < total {
                break;
            }
            let (msg, _consumed_fulll) =
                bincode::decode_from_slice::<Message, _>(&self.read_buffer[..total], BINCODE_CFG)?;
            self.read_buffer.drain(0..total);
            self.handle_message(msg);
        }
        Ok(())
    }

    fn handle_message(&mut self, msg: Message) {
        if let Some(sink) = &self.telemetry {
            sink.received(&msg, crate::time_util::host_now());
        }
        let seq = msg.seq();
        if let Some(handle) = self.pending_responses.remove(&seq) {
            let _ = handle.set_generic(msg);
        } else if matches!(msg.body, Body::Resp { .. } | Body::ExtResp { .. }) {
            tracing::error!(seq, "HMI runner received response with no awaiting handle");
        }
    }

    fn fail_all(&mut self, queue: &mut VecDeque<PendingWrite>, error: HmiError) {
        let pending_count = self.pending_responses.len() + self.held.len() + queue.len();
        if pending_count > 0 {
            tracing::warn!(
                pending = pending_count,
                error = %error,
                "HMI runner failing pending requests"
            );
        }
        for (_, handle) in self.pending_responses.drain() {
            let _ = handle.set_error(error.clone());
        }
        for pending in self.held.drain(..).chain(queue.drain(..)) {
            let _ = pending.handle.set_error(error.clone());
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn hmi_runner_runtime(
    handle: ThreadHandle,
    tcp_stream: TcpStream,
    poll: Poll,
    from_driver: Receiver<RunnerMessage>,
    thread: Vec<ThreadOption>,
    started: Sender<std::io::Result<OptionsReport<ThreadOption>>>,
    telemetry: Option<HmiTelemetry>,
    err_flag: Arc<AtomicBool>,
) {
    let _tuning = match tuning::apply_thread(DRIVER, ThreadRole::Control, &thread) {
        Ok(report) => {
            let _ = started.send(Ok(OptionsReport::from(&report)));
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
    let mut runner = HmiRunner {
        handle,
        tcp_stream,
        from_driver,
        pending_responses: HashMap::new(),
        held: VecDeque::new(),
        read_buffer: Vec::with_capacity(2048),
        shutting_down: false,
        telemetry,
    };
    let mut queue = VecDeque::new();
    if let Err(e) = runner.run(poll, &mut queue) {
        tracing::error!(error = %e, "HMI runner terminated with error");
        err_flag.store(true, Ordering::Relaxed);
    }
    runner.handle.has_died();
    runner.fail_all(&mut queue, HmiError::NotConnected);
    while let Ok(msg) = runner.from_driver.try_recv() {
        if let RunnerMessage::Send { handle, .. } = msg {
            let _ = handle.set_error(HmiError::NotConnected);
        }
    }
}

#[cfg(test)]
mod fuzz_test;
