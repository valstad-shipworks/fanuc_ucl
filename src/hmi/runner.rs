use flume::{Receiver, Sender};
use std::collections::{HashMap, VecDeque};
use std::io::{Error as IoError, ErrorKind, Read, Write};
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

#[cfg(not(test))]
use mio::{Events, Interest, Poll, Token, Waker, net::TcpStream};
#[cfg(test)]
use snare::mio::{Events, Interest, Poll, Token, Waker, net::TcpStream};

use crate::hmi::proto::wire::{Body, Header, Message};
use crate::hmi::{BINCODE_CFG, DriverResult, HmiError};
use crate::thread_util::{GeneralThreadError, ThreadConfig, ThreadHandle};

use super::hmi_handle::{HmiHandleGeneric, HmiResult};

#[derive(Debug, Clone)]
pub(super) enum RunnerMessage {
    Send {
        seq: u8,
        data: Vec<u8>,
        handle: HmiHandleGeneric,
    },
    Shutdown,
}

#[derive(Debug, Clone)]
pub(super) struct PendingWrite {
    buf: Vec<u8>,
    offset: usize,
    seq: u8,
    handle: HmiHandleGeneric,
}

pub(super) struct HmiRunner {
    handle: ThreadHandle,
    tcp_stream: TcpStream,
    from_driver: Receiver<RunnerMessage>,
    pending_responses: HashMap<u8, HmiHandleGeneric>,
    read_buffer: Vec<u8>,
    shutting_down: bool,
}

impl HmiRunner {
    const TOK_SOCKET: Token = Token(0);
    const TOK_WAKER: Token = Token(1);

    pub(super) fn start(
        addr: SocketAddr,
        handle: ThreadHandle,
        from_driver: Receiver<RunnerMessage>,
        thread_config: Option<ThreadConfig>,
    ) -> DriverResult<(std::thread::JoinHandle<()>, Arc<Waker>, Arc<AtomicBool>)> {
        let tcp_stream = TcpStream::connect(addr)?;
        #[cfg(test)]
        {
            tcp_stream.set_nonblocking(true)?;
        }
        log::trace!("HMI runner connected to {}", addr);
        let (waker_tx, waker_rx) = flume::bounded(1);
        let local_err_flag = Arc::new(AtomicBool::new(false));
        let thread_err_flag = local_err_flag.clone();
        let thread_id = std::thread::current().id();
        let join_handle = std::thread::Builder::new()
            .name("fanuc-hmi-runner".to_string())
            .spawn(move || {
                snare::register_thread_child_of(thread_id);
                if let Err(e) =
                    hmi_runner_runtime(handle, tcp_stream, from_driver, thread_config, waker_tx)
                {
                    log::error!("HMI runner thread setup failed: {:?}", e);
                    thread_err_flag.store(true, Ordering::Relaxed);
                }
            })?;
        let waker = waker_rx
            .recv()
            .map_err(|e| HmiError::Io(IoError::other(e)))?;
        log::trace!("HMI runner started");
        Ok((join_handle, waker, local_err_flag))
    }

    fn run(&mut self, mut poll: Poll) -> HmiResult<()> {
        let mut events = Events::with_capacity(64);
        let mut queue: VecDeque<PendingWrite> = VecDeque::new();
        let mut scratch = [0u8; 2048];
        let mut connection_established = false;

        loop {
            self.drain_channel(&mut queue)?;
            if self.shutting_down {
                break;
            }

            if !queue.is_empty() && connection_established {
                self.write_from_queue(&mut queue)?;
            }

            poll.poll(&mut events, Some(Duration::from_millis(96)))
                .map_err(HmiError::from)?;

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
                        self.drain_channel(&mut queue)?;
                    }
                    HmiRunner::TOK_SOCKET => {
                        if event.is_readable() {
                            self.read_stream(&mut scratch)?;
                        }
                        self.drain_channel(&mut queue)?;
                    }
                    _ => {}
                }
            }

            if !queue.is_empty() && connection_established {
                self.write_from_queue(&mut queue)?;
            }

            if self.shutting_down {
                break;
            }

            if !self.handle.should_live() && queue.is_empty() && self.pending_responses.is_empty() {
                break;
            }
        }

        self.fail_all(&mut queue, HmiError::NotConnected);
        self.handle.has_died();
        Ok(())
    }

    fn drain_channel(&mut self, queue: &mut VecDeque<PendingWrite>) -> HmiResult<()> {
        while let Ok(msg) = self.from_driver.try_recv() {
            match msg {
                RunnerMessage::Send { seq, data, handle } => {
                    queue.push_back(PendingWrite {
                        buf: data,
                        offset: 0,
                        seq,
                        handle,
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
        log::trace!("Writing to HMI tcp stream");
        while let Some(front) = queue.front_mut() {
            match self.tcp_stream.write(&front.buf[front.offset..]) {
                Ok(0) => {
                    log::error!("HMI TCP write returned 0, peer closed connection");
                    return Err(HmiError::NotConnected);
                }
                Ok(n) => {
                    front.offset += n;
                    if front.offset == front.buf.len() {
                        self.pending_responses
                            .insert(front.seq, front.handle.clone());
                        queue.pop_front();
                    }
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => {
                    let handle = front.handle.clone();
                    queue.pop_front();
                    log::error!("HMI TCP write error: {}", e);
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
                    log::error!("HMI TCP read returned 0, connection lost");
                    return Err(HmiError::NotConnected);
                }
                Ok(n) => {
                    self.read_buffer.extend_from_slice(&scratch[..n]);
                    self.process_buffer()?;
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return Ok(()),
                Err(e) => {
                    log::error!("HMI TCP read error: {}", e);
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
        let seq = msg.seq();
        if let Some(handle) = self.pending_responses.remove(&seq) {
            let _ = handle.set_generic(msg);
        } else if matches!(msg.body, Body::Resp { .. } | Body::ExtResp { .. }) {
            log::error!(
                "HMI runner received response with no awaiting handle: seq {}",
                seq
            );
        }
    }

    fn fail_all(&mut self, queue: &mut VecDeque<PendingWrite>, error: HmiError) {
        let pending_count = self.pending_responses.len() + queue.len();
        if pending_count > 0 {
            log::warn!(
                "HMI runner failing {} pending requests: {}",
                pending_count,
                error
            );
        }
        for (_, handle) in self.pending_responses.drain() {
            let _ = handle.set_error(error.clone());
        }
        while let Some(pending) = queue.pop_front() {
            let _ = pending.handle.set_error(error.clone());
        }
    }
}

fn hmi_runner_runtime(
    handle: ThreadHandle,
    mut tcp_stream: TcpStream,
    from_driver: Receiver<RunnerMessage>,
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
            HmiRunner::TOK_SOCKET,
            Interest::READABLE.add(Interest::WRITABLE),
        )
        .map_err(|_| GeneralThreadError::FailedSocketRegistry)?;
    let waker = Arc::new(
        Waker::new(poll.registry(), HmiRunner::TOK_WAKER)
            .map_err(|_| GeneralThreadError::FailedWakerCreation)?,
    );
    waker_tx.send(waker.clone())?;
    let mut runner = HmiRunner {
        handle,
        tcp_stream,
        from_driver,
        pending_responses: HashMap::new(),
        read_buffer: Vec::with_capacity(2048),
        shutting_down: false,
    };
    if let Err(e) = runner.run(poll) {
        log::error!("HMI runner terminated with error: {}", e);
    }
    Ok(())
}
