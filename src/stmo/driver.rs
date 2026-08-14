#![allow(clippy::unnecessary_map_on_constructor, clippy::useless_conversion)]

use std::{
    collections::VecDeque,
    io,
    net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use cfg_mixin::cfg_mixin;
use event_listener::{Event, Listener};
use flume::{Receiver, Sender};

use crate::{
    TelemetrySink,
    joints::JointDataSizeError,
    stmo::{
        JointMovementLimit,
        buffer::{ControllerBuffer, SeqVerdict},
        proto::{
            CommandPositionRequestPacket, CommandPositionResponsePacket, MotionCommandPacket,
            RobotStatusPacket, RxPackets, StartPacket, StopPacket, ThresholdTableRequestPacket,
            TxPackets, VersionNumberRequestPacket,
        },
        stmo_handle::StmoHandle,
        tx_errqueue::{TxError, drain_error_queue, enable_tx_error_reporting},
        types::{
            AxisMotionConstraint, JointMovementLimits, RxStorage, StmoCounters, StmoStats,
            StreamMotionError,
        },
    },
    thread_util::{GeneralThreadError, ThreadConfig, ThreadHandle},
    time_util::host_now,
};

use snare::mio::net::UdpSocket as MioUdpSocket;
use snare::mio::{Events, Interest, Poll, Token, Waker};

#[cfg(feature = "py")]
use pyo3::prelude::*;

const TOK_SOCKET: Token = Token(0);
const TOK_WAKER: Token = Token(1);

/// Shared sink observing STMO traffic: outgoing [`TxPackets`], incoming [`RxPackets`].
pub type StmoTelemetry = Arc<dyn TelemetrySink<TxPackets, RxPackets>>;

/// Whether a failed send left the datagram entirely inside this host.
///
/// The controller faults both on a command it never receives and on a sequence
/// number it receives twice, so a resend is only safe when the first attempt
/// provably put nothing on the wire. These are those errors, and all of them
/// clear on their own: the socket buffer or qdisc is momentarily full
/// (`EAGAIN`, `ENOBUFS`, `ENOMEM`), or a signal interrupted the call
/// (`EINTR`). Anything else — `ECONNREFUSED` from an ICMP reply, an
/// unreachable route, a bad argument — either means the peer already saw
/// something or that retrying cannot help.
fn is_transient_send_error(e: &io::Error) -> bool {
    if matches!(
        e.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
    ) {
        return true;
    }
    #[cfg(unix)]
    if let Some(raw) = e.raw_os_error() {
        return raw == libc::ENOBUFS || raw == libc::ENOMEM;
    }
    false
}

/// Scratch buffers for one I/O thread. Held outside the context so the receive
/// path can be pumped from inside a send retry without aliasing `self`.
struct IoBufs {
    rx: [u8; 2048],
    tx: [u8; 1024],
}

impl IoBufs {
    fn new() -> Self {
        Self {
            rx: [0; 2048],
            tx: [0; 1024],
        }
    }
}

#[derive(Debug, Clone)]
enum MaybeMany<T: Clone> {
    One(T),
    Many(Vec<T>),
}

enum ToThreadMessage {
    Start(StartPacket),
    Stop(StopPacket),
    ThresholdTableRequest(ThresholdTableRequestPacket),
    MotionCommandDouble(MaybeMany<MotionCommandPacket>, Option<StmoHandle>),
}

#[derive(Debug)]
struct StreamMotionContext {
    socket: MioUdpSocket,
    from_driver: Receiver<ToThreadMessage>,
    to_driver: Sender<RxPackets>,
    protocol_version: u32,
    send_last_command: bool,
    last_command_position_request_time: snare::time::Instant,
    motion_command_queue: VecDeque<(MaybeMany<MotionCommandPacket>, Option<StmoHandle>)>,
    itl: Arc<(Event, AtomicBool)>,
    telemetry: Option<StmoTelemetry>,
    counters: Arc<StmoCounters>,
    err_flag: Arc<AtomicBool>,
    consecutive_send_failures: u32,
    /// Descriptor to drain transmit errors from, owned by `socket`. `None`
    /// where the kernel cannot report them.
    tx_error_fd: Option<i32>,
    tx_errors: Vec<TxError>,
    buffer: ControllerBuffer,
    /// Newest status not yet answered. Only the newest is worth a command —
    /// answering an older one would command a cycle that has already passed —
    /// but every status reaches consumers through `to_driver` regardless.
    pending_status: Option<RobotStatusPacket>,
    /// Set while inside a send retry, so pumped statuses are attributed there.
    retrying: bool,
}

impl StreamMotionContext {
    const COMMAND_POSITION_RATE: Duration = Duration::from_millis(128);
    /// Failed sends in a row before the connection is declared errored.
    const SEND_FAILURE_LIMIT: u32 = 4;
    /// Gap between attempts while a send is blocked. Short enough to land
    /// several attempts and several receive pumps inside one cycle.
    const RETRY_INTERVAL: Duration = Duration::from_micros(250);
    /// Absolute ceiling on one retry, in case the controller model is wrong
    /// about how much runway is left.
    const MAX_RETRY: Duration = Duration::from_millis(250);
    /// Retry window for control packets, which the buffer model says nothing
    /// about.
    const CONTROL_SEND_TIMEOUT: Duration = Duration::from_millis(24);
    /// Responses per poll wakeup, so a peer streaming faster than we can answer
    /// cannot hold the loop.
    const MAX_RESPONSES_PER_WAKE: u32 = 32;

    #[allow(clippy::too_many_arguments)]
    fn new(
        from_driver: Receiver<ToThreadMessage>,
        to_driver: Sender<RxPackets>,
        socket: MioUdpSocket,
        itl: Arc<(Event, AtomicBool)>,
        send_last_command: bool,
        telemetry: Option<StmoTelemetry>,
        counters: Arc<StmoCounters>,
        err_flag: Arc<AtomicBool>,
        tx_error_fd: Option<i32>,
        buffer_size_before_drain: u8,
    ) -> Self {
        Self {
            from_driver,
            to_driver,
            socket,
            protocol_version: 0,
            // Backdated so the first request fires immediately. checked_sub
            // because the shimmed clock starts near its epoch, where plain
            // subtraction underflows.
            last_command_position_request_time: snare::time::Instant::now()
                .checked_sub(Self::COMMAND_POSITION_RATE)
                .unwrap_or_else(snare::time::Instant::now),
            motion_command_queue: VecDeque::new(),
            itl,
            send_last_command,
            telemetry,
            counters,
            err_flag,
            consecutive_send_failures: 0,
            tx_error_fd,
            tx_errors: Vec::with_capacity(8),
            buffer: ControllerBuffer::new(buffer_size_before_drain),
            pending_status: None,
            retrying: false,
        }
    }

    /// Offers `tx` to the socket once.
    fn send_once(&mut self, tx: &TxPackets, buf: &[u8]) -> Result<(), StreamMotionError> {
        match self.socket.send(buf) {
            Ok(written) if written == buf.len() => {
                if let Some(sink) = &self.telemetry {
                    sink.sent(tx, host_now());
                }
                Ok(())
            }
            Ok(written) => {
                self.counters.short_sends.fetch_add(1, Ordering::Relaxed);
                tracing::error!(
                    written,
                    expected = buf.len(),
                    "STMO UDP send truncated the packet"
                );
                Err(StreamMotionError::Io(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "short UDP send",
                )))
            }
            Err(e) => Err(StreamMotionError::from(e)),
        }
    }

    /// Sends `tx`, re-offering the same datagram while the socket refuses it.
    ///
    /// Only errors where the packet provably never left this host are retried,
    /// so the controller can never see a repeated sequence number. `budget`
    /// bounds the attempt; for motion commands the caller derives it from how
    /// long the controller's buffer can keep the robot moving without us.
    ///
    /// Between attempts the receive path is pumped. A controller whose status
    /// stream is fine while our transmit side is blocked must not have its
    /// telemetry starved, and the cycles that arrive meanwhile become the
    /// catch-up burst once this lands.
    fn send_retrying(
        &mut self,
        what: &'static str,
        tx: TxPackets,
        io: &mut IoBufs,
        budget: Duration,
        version_override: Option<u32>,
    ) -> bool {
        let version = version_override.unwrap_or(self.protocol_version);
        let n = match tx.encode_into(version, &mut io.tx) {
            Ok(n) => n,
            Err(e) => {
                tracing::error!(packet = what, error = %e, "STMO encode failed");
                return false;
            }
        };

        // Real clock: the socket unblocks in kernel time and the wait between
        // attempts is real, so a virtual deadline would spin forever under a
        // paused shim clock.
        let start = Instant::now();
        let sleeper = spin_sleep::SpinSleeper::new(1_000_000);
        let mut attempts: u32 = 0;

        let failure = loop {
            attempts += 1;
            match self.send_once(&tx, &io.tx[..n]) {
                Ok(()) => {
                    if attempts > 1 {
                        self.counters.send_retries.fetch_add(1, Ordering::Relaxed);
                        tracing::debug!(
                            packet = what,
                            attempts,
                            elapsed_us = start.elapsed().as_micros(),
                            "STMO send landed after retry"
                        );
                    }
                    self.consecutive_send_failures = 0;
                    return true;
                }
                Err(StreamMotionError::Io(ref e)) if is_transient_send_error(e) => {
                    if start.elapsed() >= budget.min(Self::MAX_RETRY) {
                        break io::Error::from(e.kind()).to_string();
                    }
                    // Pumping keeps consumers fed and drains the runway as
                    // cycles pass, so a command that is already late gives up
                    // sooner than a fresh one.
                    self.retrying = true;
                    self.pump_rx(io);
                    self.retrying = false;
                    sleeper.sleep(Self::RETRY_INTERVAL);
                }
                Err(e) => {
                    tracing::error!(packet = what, error = %e, "STMO UDP send error");
                    break e.to_string();
                }
            }
        };

        self.fail_send(what, &failure);
        false
    }

    fn fail_send(&mut self, what: &'static str, detail: &str) {
        self.consecutive_send_failures += 1;
        let total = self.counters.send_failures.fetch_add(1, Ordering::Relaxed) + 1;
        tracing::error!(
            packet = what,
            detail,
            consecutive = self.consecutive_send_failures,
            total,
            "STMO send failed"
        );
        self.drain_tx_errors();
        if self.consecutive_send_failures >= Self::SEND_FAILURE_LIMIT {
            self.err_flag.store(true, Ordering::SeqCst);
        }
    }

    /// Sends a queued motion command, returning whether the controller can be
    /// assumed to have it.
    ///
    /// A command that never made it onto the wire goes back to the head of the
    /// queue with its handle unfulfilled: its sequence number is still unused
    /// on the controller, so the next cycle can fill the gap rather than skip
    /// the trajectory point.
    fn send_motion(
        &mut self,
        what: &'static str,
        cmd: MotionCommandPacket,
        handle: Option<StmoHandle>,
        io: &mut IoBufs,
    ) -> bool {
        let budget = self.buffer.runway();
        if self.send_retrying(what, TxPackets::MotionCommand(cmd), io, budget, None) {
            self.buffer.commanded(cmd.seq);
            if cmd.last_command {
                self.buffer.stream_ended();
            }
            if let Some(h) = handle {
                h.set();
            }
            true
        } else {
            self.motion_command_queue
                .push_front((MaybeMany::One(cmd), handle));
            false
        }
    }

    /// Reports transmit errors the kernel would otherwise only have counted
    /// internally. Also clears the error queue, which the poller needs: while
    /// it holds an entry the socket stays permanently readable.
    fn drain_tx_errors(&mut self) {
        let Some(fd) = self.tx_error_fd else {
            return;
        };
        drain_error_queue(fd, &mut self.tx_errors);
        if self.tx_errors.is_empty() {
            return;
        }
        self.counters
            .tx_errors
            .fetch_add(self.tx_errors.len() as u64, Ordering::Relaxed);
        for err in self.tx_errors.drain(..) {
            tracing::error!(
                errno = err.errno,
                origin = err.origin_str(),
                "STMO packet dropped on the transmit path: {err}"
            );
        }
    }

    fn next_motion_command(&mut self) -> Option<(MotionCommandPacket, Option<StmoHandle>)> {
        loop {
            let should_pop_entry = match self.motion_command_queue.front()? {
                (MaybeMany::One(_), _) => true,
                (MaybeMany::Many(vec), _) => vec.len() <= 1,
            };

            if should_pop_entry {
                let (cmds, handle) = self.motion_command_queue.pop_front()?;
                match cmds {
                    MaybeMany::One(cmd) => return Some((cmd, handle)),
                    MaybeMany::Many(mut vec) => {
                        if let Some(cmd) = vec.pop() {
                            return Some((cmd, handle));
                        } else {
                            // empty batch — fulfill handle and try next entry
                            if let Some(h) = handle {
                                h.set();
                            }
                            continue;
                        }
                    }
                }
            } else {
                // Many with >1 element — pop one without consuming the handle yet
                if let Some((MaybeMany::Many(vec), _)) = self.motion_command_queue.front_mut() {
                    return vec.pop().map(|c| (c, None));
                }
                return None;
            }
        }
    }

    /// Drains the socket, forwarding every decoded packet to the driver and the
    /// telemetry sink and folding each status into the controller model.
    ///
    /// This is the only place packets are read, so it is safe to call from
    /// inside a send retry — consumers keep receiving while the transmit side
    /// is blocked. Only the newest status is parked for a reply.
    fn pump_rx(&mut self, io: &mut IoBufs) -> u32 {
        let mut statuses = 0;
        loop {
            match self.socket.recv(&mut io.rx) {
                Ok(n) if n > 0 => {
                    let Some(rx) = RxPackets::decode_from(&io.rx[..n]) else {
                        tracing::warn!(len = n, "Received unknown packet: {:02X?}", &io.rx[..n]);
                        continue;
                    };
                    if let Some(sink) = &self.telemetry {
                        sink.received(&rx, host_now());
                    }
                    tracing::trace!(packet = ?rx, "Received packet");
                    if let RxPackets::VersionNumberResponse(vn) = &rx {
                        self.protocol_version = vn.version;
                        tracing::info!(
                            version = self.protocol_version,
                            "Detected Stream Motion protocol version"
                        );
                    }
                    if let RxPackets::RobotStatus(state) = &rx {
                        statuses += 1;
                        self.buffer.saw_status(state.seq, Instant::now());
                        let newer = self
                            .pending_status
                            .is_none_or(|p| state.seq.wrapping_sub(p.seq) <= u32::MAX / 2);
                        if newer {
                            self.pending_status = Some(*state);
                        }
                        // Cycle-driven consumers wait on this; notifying as
                        // soon as the packet lands keeps them in step even when
                        // the reply is still being retried.
                        if self.itl.1.load(Ordering::SeqCst) {
                            self.itl.0.notify(1);
                        }
                    }
                    let _ = self.to_driver.send(rx);
                }
                Ok(_) => {
                    tracing::warn!("Received empty packet");
                    break;
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(ref e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    tracing::error!(error = %e, "Error receiving packet");
                    break;
                }
            }
        }
        if statuses > 0 {
            self.publish_gauges();
            if self.retrying {
                self.counters
                    .statuses_during_retry
                    .fetch_add(statuses as u64, Ordering::Relaxed);
            }
        }
        statuses
    }

    /// Mirrors the controller model into the counters consumers read.
    fn publish_gauges(&self) {
        self.counters
            .buffer_depth
            .store(self.buffer.depth() as u64, Ordering::Relaxed);
        self.counters
            .cycle_us
            .store(self.buffer.cycle().as_micros() as u64, Ordering::Relaxed);
        self.counters
            .underruns
            .store(self.buffer.underruns(), Ordering::Relaxed);
        self.counters
            .lost_statuses
            .store(self.buffer.lost_statuses(), Ordering::Relaxed);
    }

    /// Answers one status: fills any sequences the controller was never given,
    /// then commands the current cycle.
    fn respond(&mut self, state: &RobotStatusPacket, io: &mut IoBufs, prev: &mut PrevCommand) {
        self.respond_inner(state, io, prev);
        self.publish_gauges();
    }

    fn respond_inner(
        &mut self,
        state: &RobotStatusPacket,
        io: &mut IoBufs,
        prev: &mut PrevCommand,
    ) {
        let outstanding = match self.buffer.plan(state.seq) {
            SeqVerdict::Command { outstanding } => outstanding,
            SeqVerdict::Resync => {
                tracing::warn!(seq = state.seq, "STMO sequence restarted, resyncing");
                0
            }
            SeqVerdict::Stale => {
                self.counters.stale_statuses.fetch_add(1, Ordering::Relaxed);
                tracing::debug!(seq = state.seq, "Stale STMO status, not commanding");
                return;
            }
        };

        if !state.status_bits().ready_for_commands() {
            // Nothing is owed for a cycle the controller was not taking
            // commands in.
            self.buffer.settled(state.seq);
            return;
        }
        if state.status_bits().packet_rate() as u32 != 0 {
            tracing::debug!(
                rate = state.status_bits().packet_rate(),
                "Robot status packet rate"
            );
        }

        if outstanding > 0 {
            self.counters
                .missed_status_cycles
                .fetch_add(outstanding as u64, Ordering::Relaxed);
        }

        // Under a control loop the caller owns the cadence; bursting would
        // desync their status/command pairing.
        let burst = if self.itl.1.load(Ordering::SeqCst) {
            0
        } else {
            self.buffer.burst_for(outstanding)
        };
        if burst > 0 {
            tracing::warn!(
                seq = state.seq,
                outstanding,
                burst,
                depth = self.buffer.depth(),
                "STMO cycles unanswered, refilling controller buffer"
            );
        }
        for i in 0..burst {
            let Some((mut cmd, handle)) = self.next_motion_command() else {
                break;
            };
            // The missed sequences were never given to the controller, so
            // filling them is a resend rather than a repeat.
            cmd.seq = state.seq.wrapping_sub(burst - i);
            prev.record(cmd, true);
            self.counters
                .catchup_commands
                .fetch_add(1, Ordering::Relaxed);
            if !self.send_motion("motion_command_catchup", cmd, handle, io) {
                break;
            }
        }

        if self.buffer.headroom() == 0 {
            // One more would overflow the controller and fault it. It drains a
            // command this cycle, so the slot reopens on the next.
            self.counters.overflow_skips.fetch_add(1, Ordering::Relaxed);
            tracing::warn!(seq = state.seq, "STMO controller buffer full, withholding");
            self.buffer.settled(state.seq);
            return;
        }

        if let Some((mut cmd, handle)) = self.next_motion_command() {
            cmd.seq = state.seq;
            if prev.fillers > 0 {
                tracing::debug!(
                    seq = state.seq,
                    filler_cycles = prev.fillers,
                    starved_us = prev.fillers as u128 * self.buffer.cycle().as_micros(),
                    "STMO queue refilled"
                );
            }
            prev.record(cmd, true);
            if cmd.last_command {
                tracing::trace!("Last motion command sent");
            }
            self.send_motion("motion_command", cmd, handle, io);
        } else if state.status_bits().command_received() && !self.itl.1.load(Ordering::SeqCst) {
            if let Some(held) = prev.packet {
                let mut cmd = MotionCommandPacket::filler(state, &held, self.send_last_command);
                cmd.seq = state.seq;
                if prev.fillers == 0 {
                    let want = held.position();
                    let actual = state.joints_raw();
                    tracing::debug!(
                        seq = cmd.seq,
                        prev_real = prev.was_real,
                        j1_held = want[0],
                        j1_actual = actual[0],
                        rail_held = want[6],
                        rail_actual = actual[6],
                        "STMO queue starved, holding setpoint"
                    );
                }
                prev.record(cmd, false);
                if self.send_retrying(
                    "motion_command_filler",
                    TxPackets::MotionCommand(cmd),
                    io,
                    self.buffer.runway(),
                    None,
                ) {
                    self.buffer.commanded(cmd.seq);
                    if cmd.last_command {
                        self.buffer.stream_ended();
                    }
                }
            } else {
                // Nothing has ever been commanded, so there is no setpoint to
                // hold and no debt for this cycle.
                self.buffer.settled(state.seq);
            }
        } else {
            self.buffer.settled(state.seq);
        }
    }

    pub fn context_loop(mut self, thread_handle: ThreadHandle, mut poll: Poll) {
        let mut events = Events::with_capacity(64);
        let mut io = IoBufs::new();
        let mut prev = PrevCommand::default();
        let mut stop_sent = false;

        while thread_handle.should_live() {
            if let Err(e) = poll.poll(&mut events, None) {
                if e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                tracing::error!(error = %e, "STMO poll error, breaking event loop");
                break;
            }

            for ev in events.iter() {
                match ev.token() {
                    TOK_SOCKET => {
                        self.drain_tx_errors();
                        self.pump_rx(&mut io);
                        // Answering can take most of a cycle when a send is
                        // being retried, so re-check for a newer status rather
                        // than waiting for the next wakeup to notice it.
                        let mut answered = 0;
                        while let Some(state) = self.pending_status.take() {
                            self.respond(&state, &mut io, &mut prev);
                            self.pump_rx(&mut io);
                            answered += 1;
                            if answered >= Self::MAX_RESPONSES_PER_WAKE {
                                break;
                            }
                        }

                        if self.last_command_position_request_time.elapsed()
                            >= Self::COMMAND_POSITION_RATE
                            && self.protocol_version != 0
                        {
                            self.send_retrying(
                                "command_position_request",
                                TxPackets::CommandPositionRequest(CommandPositionRequestPacket {}),
                                &mut io,
                                Duration::from_millis(2),
                                None,
                            );
                            self.last_command_position_request_time = snare::time::Instant::now();
                        }
                    }

                    TOK_WAKER => {
                        // drain commands from driver
                        while let Ok(tx) = self.from_driver.try_recv() {
                            match tx {
                                ToThreadMessage::Start(pkt) => {
                                    self.send_retrying(
                                        "start",
                                        TxPackets::Start(pkt),
                                        &mut io,
                                        Self::CONTROL_SEND_TIMEOUT,
                                        Some(3),
                                    );
                                    self.send_retrying(
                                        "version_number_request",
                                        TxPackets::VersionNumberRequest(
                                            VersionNumberRequestPacket {},
                                        ),
                                        &mut io,
                                        Self::CONTROL_SEND_TIMEOUT,
                                        Some(3),
                                    );
                                }
                                ToThreadMessage::MotionCommandDouble(pkt, handle) => {
                                    self.motion_command_queue.push_back((pkt, handle));
                                }
                                ToThreadMessage::Stop(pkt) => {
                                    stop_sent |= self.send_retrying(
                                        "stop",
                                        TxPackets::Stop(pkt),
                                        &mut io,
                                        Self::CONTROL_SEND_TIMEOUT,
                                        None,
                                    );
                                    self.buffer.stream_ended();
                                }
                                ToThreadMessage::ThresholdTableRequest(pkt) => {
                                    self.send_retrying(
                                        "threshold_table_request",
                                        TxPackets::ThresholdTableRequest(pkt),
                                        &mut io,
                                        Self::CONTROL_SEND_TIMEOUT,
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

        // A stop queued by disconnect() races the death signal: join() stores
        // should_die before waking us, so the loop can exit without ever
        // draining it. Cover for that here rather than leave the controller
        // streaming.
        if self.protocol_version == 0 {
            tracing::info!("StreamMotionContext exiting (never started)");
            thread_handle.has_died();
            return;
        }
        if !stop_sent {
            self.send_retrying(
                "stop",
                TxPackets::Stop(StopPacket {}),
                &mut io,
                Self::CONTROL_SEND_TIMEOUT,
                None,
            );
        }
        tracing::info!("StreamMotionContext exited");
        thread_handle.has_died();
    }
}

/// The last motion command put on the wire, used to synthesize fillers that
/// hold the setpoint when the queue runs dry.
#[derive(Debug, Default)]
struct PrevCommand {
    packet: Option<MotionCommandPacket>,
    was_real: bool,
    fillers: u32,
}

impl PrevCommand {
    fn record(&mut self, cmd: MotionCommandPacket, real: bool) {
        self.packet = Some(cmd);
        self.was_real = real;
        if real {
            self.fillers = 0;
        } else {
            self.fillers += 1;
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn stream_motion_runtime(
    mut thread_handle: ThreadHandle,
    socket: snare::net::UdpSocket,
    thread_config: Option<ThreadConfig>,
    to_driver: Sender<RxPackets>,
    from_driver: Receiver<ToThreadMessage>,
    waker_tx: Sender<Arc<Waker>>,
    itl: Arc<(Event, AtomicBool)>,
    send_last_command: bool,
    telemetry: Option<StmoTelemetry>,
    counters: Arc<StmoCounters>,
    err_flag: Arc<AtomicBool>,
    buffer_size_before_drain: u8,
) -> Result<(), GeneralThreadError> {
    if let Some(cfg) = thread_config {
        cfg.configure_this_thread_print_failure();
    }
    if let Some(sink) = &telemetry {
        sink.warmup();
    }

    // Taken before the socket moves into mio; the descriptor stays owned by
    // the socket, which outlives the context loop.
    let tx_error_fd = enable_tx_error_reporting(&socket);
    let mut socket = MioUdpSocket::from_std(socket);

    let poll = Poll::new().map_err(|_| GeneralThreadError::FailedToCreatePoll)?;
    poll.registry()
        .register(&mut socket, TOK_SOCKET, Interest::READABLE)
        .map_err(|_| GeneralThreadError::FailedSocketRegistry)?;

    let waker = Arc::new(
        Waker::new(poll.registry(), TOK_WAKER)
            .map_err(|_| GeneralThreadError::FailedWakerCreation)?,
    );
    // send a clone to the driver so API calls can wake the poller
    waker_tx.send(waker.clone())?;
    thread_handle.set_waker_mio(waker);

    tracing::debug!("Stream motion thread started, entering context loop");

    let context = StreamMotionContext::new(
        from_driver,
        to_driver,
        socket,
        itl,
        send_last_command,
        telemetry,
        counters,
        err_flag,
        tx_error_fd,
        buffer_size_before_drain,
    );
    context.context_loop(thread_handle, poll);

    Ok(())
}

#[derive(Debug)]
struct StreamMotionConnection {
    thread_handle: ThreadHandle,
    to_thread: Sender<ToThreadMessage>,
    from_thread: Receiver<RxPackets>,
    is_started: bool,
    err_flag: Arc<AtomicBool>,
    itl: Arc<(Event, AtomicBool)>,
    counters: Arc<StmoCounters>,
}

/// Driver for FANUC Stream Motion (STMO), a UDP protocol in which the controller
/// requests a position command every interpolation cycle (typically 8ms).
/// A dedicated I/O thread answers each cycle from a queue of motion commands;
/// dropping the driver disconnects, joining that thread.
#[cfg_attr(feature = "py", pyo3::pyclass(str))]
#[derive(Debug)]
pub struct StreamMotionDriver {
    remote_addr: IpAddr,
    send_last_command: bool,
    connection: Option<StreamMotionConnection>,
    cached_movement_limits: Option<JointMovementLimits>,
    rx_storage: RxStorage,
    telemetry: Option<StmoTelemetry>,
    buffer_size_before_drain: u8,
}

/// Commands the controller queues before it faults on overflow.
pub const BUFFER_CAPACITY: u8 = crate::stmo::buffer::CAPACITY;

impl StreamMotionDriver {
    #[inline]
    fn send_packet(&self, tx: ToThreadMessage) {
        if let Some(conn) = &self.connection {
            if let Err(e) = conn.to_thread.send(tx) {
                tracing::error!(error = %e, "Error sending packet to thread");
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
    /// Creates a driver targeting the controller at `addr`.
    ///
    /// `buffer_size_before_drain` must match the controller's `$STMO.$START_MOVE`:
    /// how many commands it queues before it begins executing them. The driver
    /// mirrors that queue to decide how many commands may go out at once and
    /// how long a blocked send may be retried before the robot runs out of
    /// motion, so a value that disagrees with the controller costs resilience
    /// in both directions — too low and the driver abandons a stalled send
    /// while the robot still had motion buffered, too high and it keeps
    /// retrying past the point the robot has already faulted. Clamped to
    /// `1..=`[`BUFFER_CAPACITY`].
    ///
    /// `send_last_command` sets the last-command flag on filler packets, ending
    /// the stream once the command queue runs dry instead of holding position
    /// indefinitely.
    ///
    /// # Errors
    /// `addr` is not a valid IP address.
    #[cfg(on)]
    #[on(pyo3(signature = (addr, buffer_size_before_drain, send_last_command = false)))]
    #[on(new)]
    pub fn new(
        addr: Bound<PyAny>,
        buffer_size_before_drain: u8,
        send_last_command: bool,
    ) -> DriverResult<Self> {
        let addr = addr.extract::<IpAddr>()?;

        Ok(Self {
            remote_addr: addr,
            send_last_command,
            connection: None,
            cached_movement_limits: None,
            rx_storage: RxStorage::new(),
            telemetry: None,
            buffer_size_before_drain: buffer_size_before_drain.clamp(1, BUFFER_CAPACITY),
        })
    }

    /// Creates a driver targeting the controller at `remote_addr`.
    ///
    /// `buffer_size_before_drain` must match the controller's `$STMO.$START_MOVE`:
    /// how many commands it queues before it begins executing them. The driver
    /// mirrors that queue to decide how many commands may go out at once and
    /// how long a blocked send may be retried before the robot runs out of
    /// motion, so a value that disagrees with the controller costs resilience
    /// in both directions — too low and the driver abandons a stalled send
    /// while the robot still had motion buffered, too high and it keeps
    /// retrying past the point the robot has already faulted. Clamped to
    /// `1..=`[`BUFFER_CAPACITY`].
    ///
    /// `send_last_command` sets the last-command flag on filler packets, ending
    /// the stream once the command queue runs dry instead of holding position
    /// indefinitely.
    #[cfg(off)]
    pub fn new<T: Into<IpAddr>>(
        remote_addr: T,
        buffer_size_before_drain: u8,
        send_last_command: bool,
    ) -> Self {
        let remote_addr = remote_addr.into();
        Self {
            remote_addr,
            send_last_command,
            connection: None,
            cached_movement_limits: None,
            rx_storage: RxStorage::new(),
            telemetry: None,
            buffer_size_before_drain: buffer_size_before_drain.clamp(1, BUFFER_CAPACITY),
        }
    }

    /// Like [`new`](Self::new), with a telemetry sink observing every packet on
    /// the wire, from every connection this driver makes.
    #[cfg(off)]
    pub fn new_with_telemetry<T: Into<IpAddr>, S: TelemetrySink<TxPackets, RxPackets>>(
        remote_addr: T,
        buffer_size_before_drain: u8,
        send_last_command: bool,
        telemetry: S,
    ) -> Self {
        let mut driver = Self::new(remote_addr, buffer_size_before_drain, send_last_command);
        driver.telemetry = Some(Arc::new(telemetry));
        driver
    }

    /// Returns the controller's IP address as a string.
    #[on(pyo3(signature = ()))]
    pub fn get_remote_addr(&self) -> String {
        self.remote_addr.to_string()
    }

    /// Drains packets received by the I/O thread into the driver's internal buffers.
    pub fn refresh(&mut self) {
        let connection = match &self.connection {
            Some(c) => c,
            None => return,
        };
        while let Ok(pkt) = connection.from_thread.try_recv() {
            match pkt {
                RxPackets::RobotStatus(state) => self.rx_storage.status.push_back(state),
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

    /// Queues motion commands; the I/O thread sends one per controller cycle.
    /// The returned handle is set once the whole batch has been sent.
    ///
    /// # Errors
    /// [`StreamMotionError::NotConnected`] or [`StreamMotionError::NotStarted`] if
    /// [`connect`](Self::connect) and [`start`](Self::start) have not succeeded.
    pub fn command_motion(
        &mut self,
        mut motions: Vec<MotionCommandPacket>,
    ) -> DriverResult<StmoHandle> {
        if self.connection.is_none() {
            return Err(StreamMotionError::NotConnected).map_err(Into::into);
        }
        if !self.is_started() {
            return Err(StreamMotionError::NotStarted).map_err(Into::into);
        }
        let handle = StmoHandle::new();
        if motions.is_empty() {
            handle.set();
            return Ok(handle);
        }
        motions.reverse();
        self.send_packet(ToThreadMessage::MotionCommandDouble(
            MaybeMany::Many(motions),
            Some(handle.clone()),
        ));
        self.refresh();
        Ok(handle)
    }

    pub(crate) fn command_motion_single(
        &mut self,
        motion: MotionCommandPacket,
    ) -> DriverResult<()> {
        if self.connection.is_none() {
            return Err(StreamMotionError::NotConnected).map_err(Into::into);
        }
        if !self.is_started() {
            return Err(StreamMotionError::NotStarted).map_err(Into::into);
        }
        self.send_packet(ToThreadMessage::MotionCommandDouble(
            MaybeMany::One(motion),
            None,
        ));
        self.refresh();
        Ok(())
    }

    /// Sends a stop packet, halting the stream on the controller side.
    pub fn stop(&mut self) {
        self.send_packet(ToThreadMessage::Stop(StopPacket {}));
        self.refresh();
    }

    /// Binds a local UDP socket to the controller's Stream Motion port (60015)
    /// and spawns the I/O thread. No-op if already connected.
    ///
    /// # Errors
    /// I/O failure binding or connecting the socket, or failure to spawn the I/O thread.
    #[on(pyo3(signature = (thread_config=None)))]
    pub fn connect(&mut self, thread_config: Option<ThreadConfig>) -> DriverResult<()> {
        tracing::info!(addr = %self.remote_addr, "Attempting to connect StreamMotionDriver");
        if let Some(conn) = &self.connection
            && conn.thread_handle.is_alive()
        {
            return Ok(());
        }
        let port = openport::pick_unused_port(57000..60000).unwrap_or(60000);
        let local_addr = SocketAddrV4::new(Ipv4Addr::new(0, 0, 0, 0), port);
        let socket = snare::net::UdpSocket::bind(local_addr).map_err(StreamMotionError::from)?;
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

        let local_err_flag = Arc::new(AtomicBool::new(false));
        let thread_err_flag = local_err_flag.clone();

        let itl = Arc::new((Event::new(), AtomicBool::new(false)));
        let thread_itl = itl.clone();

        let (waker_tx, waker_rx) = flume::bounded(1);

        let send_last_command = self.send_last_command;
        let telemetry = self.telemetry.clone();

        let counters = Arc::new(StmoCounters::default());
        let thread_counters = counters.clone();
        let runtime_err_flag = local_err_flag.clone();
        let buffer_size_before_drain = self.buffer_size_before_drain;

        let thread = snare::thread::Builder::new()
            .name("fanuc-stmo-runner".to_string())
            .spawn(move || {
                if let Err(e) = stream_motion_runtime(
                    thread_handle_mv,
                    socket,
                    thread_config,
                    to_driver,
                    from_driver,
                    waker_tx,
                    thread_itl,
                    send_last_command,
                    telemetry,
                    thread_counters,
                    runtime_err_flag,
                    buffer_size_before_drain,
                ) {
                    tracing::error!(error = ?e, "Stream motion thread error");
                    thread_err_flag.store(true, std::sync::atomic::Ordering::SeqCst);
                }
            })?;

        let thread_waker = waker_rx
            .recv()
            .map_err(|_| StreamMotionError::NotConnected)?;
        thread_handle.set_waker_mio(thread_waker);
        thread_handle.set_handle(thread);

        self.connection = Some(StreamMotionConnection {
            thread_handle,
            to_thread,
            from_thread,
            is_started: false,
            err_flag: local_err_flag,
            itl,
            counters,
        });

        tracing::info!(addr = %self.remote_addr, "StreamMotionDriver connected");

        Ok(())
    }

    /// I/O health counters for the current connection, all zero when there is
    /// none. Non-zero `send_failures` or `tx_errors` mean commands did not
    /// reach the wire; non-zero `missed_status_cycles` means the controller's
    /// side of the exchange dropped.
    pub fn stats(&self) -> StmoStats {
        self.connection
            .as_ref()
            .map(|conn| conn.counters.snapshot())
            .unwrap_or_default()
    }

    /// Returns `true` if the I/O thread exited with an error, or if sends have
    /// failed for several consecutive interpolation cycles.
    pub fn has_connection_errored(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.err_flag.load(std::sync::atomic::Ordering::SeqCst)
        } else {
            false
        }
    }

    /// Sends the start packet and waits for the controller's version response.
    ///
    /// # Errors
    /// [`StreamMotionError::NotConnected`] if [`connect`](Self::connect) has not succeeded;
    /// [`StreamMotionError::Timeout`] if no version response arrives within `timeout_secs`.
    #[on(pyo3(signature = (timeout_secs=2.0)))]
    pub fn start(&mut self, timeout_secs: f32) -> DriverResult<()> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        // Real clock: the deadline feeds flume's recv_timeout, which waits in
        // real time regardless of snare's shim clock.
        let start_time = Instant::now();
        let end_time = start_time + timeout;
        if let Some(conn) = &self.connection {
            self.send_packet(ToThreadMessage::Start(StartPacket {}));
            let mut started = false;
            while start_time.elapsed() < timeout {
                let remaining = end_time.saturating_duration_since(Instant::now());
                if let Ok(RxPackets::VersionNumberResponse(_)) =
                    conn.from_thread.recv_timeout(remaining)
                {
                    started = true;
                    break;
                }
            }
            if !started {
                tracing::error!(
                    timeout_secs = timeout.as_secs_f32(),
                    "STMO start timed out waiting for version response"
                );
                Err(StreamMotionError::Timeout)?;
            }
        } else {
            Err(StreamMotionError::NotConnected)?;
        };
        if let Some(conn) = &mut self.connection {
            conn.is_started = true;
        }
        tracing::info!(addr = %self.remote_addr, "StreamMotionDriver started");
        Ok(())
    }

    /// Stops the stream and joins the I/O thread, blocking until it exits.
    /// Called automatically on drop.
    pub fn disconnect(&mut self) {
        if let Some(conn) = self.connection.take() {
            tracing::info!(addr = %self.remote_addr, "StreamMotionDriver disconnecting");
            let _ = conn.to_thread.send(ToThreadMessage::Stop(StopPacket {}));
            let _ = conn.thread_handle.wake();
            conn.thread_handle.join();
            tracing::info!(addr = %self.remote_addr, "StreamMotionDriver disconnected");
        }
        self.rx_storage.clear();
    }

    /// Returns `true` if the I/O thread is alive.
    pub fn is_connected(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.thread_handle.is_alive()
        } else {
            false
        }
    }

    /// Returns `true` once [`start`](Self::start) has completed on the current connection.
    pub fn is_started(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.is_started
        } else {
            false
        }
    }

    /// Requests the per-axis velocity, acceleration, and jerk threshold tables, blocking
    /// until all are received. `extra_axis` is the number of axes beyond the standard six.
    /// Results are cached after the first successful fetch.
    ///
    /// # Errors
    /// [`StreamMotionError::NotConnected`] or [`StreamMotionError::NotStarted`] before
    /// [`connect`](Self::connect) and [`start`](Self::start), or if the connection drops
    /// mid-fetch; [`StreamMotionError::JointDataSizeError`] if `extra_axis > 3`.
    #[on(pyo3(signature = (extra_axis=0)))]
    #[allow(clippy::needless_range_loop)]
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
            return Err(StreamMotionError::JointDataSizeError(JointDataSizeError(9)))
                .map_err(Into::into);
        }

        let axis_cnt = 6 + extra_axis as usize;

        let mut seen = vec![[false; 3]; axis_cnt];
        let mut limits = JointMovementLimits::default();

        // Real clock throughout this loop: retransmit pacing pairs with the
        // real sleeps below, and the loop must keep making progress even under
        // a paused shim clock.
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
                            match req {
                                Ok(r) => {
                                    self.send_packet(ToThreadMessage::ThresholdTableRequest(r))
                                }
                                Err(e) => tracing::error!(
                                    error = ?e,
                                    "Invalid ThresholdTableRequestPacket parameters"
                                ),
                            }
                            std::thread::sleep(Duration::from_millis(24));
                        }
                    }
                }
                last_send = Instant::now();
            }

            self.refresh();

            while let Some(pkt) = self.rx_storage.threshold_table.pop_front() {
                tracing::debug!(
                    axis = pkt.axis_number,
                    limit_type = pkt.limit_type,
                    vmax = pkt.vmax,
                    "Received movement limit"
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

                    if let Some(entry) = entry {
                        match deriv {
                            0 => entry.velocity = cons,
                            1 => entry.acceleration = cons,
                            2 => entry.jerk = cons,
                            _ => {}
                        }
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

    /// Drains and returns all buffered robot status packets.
    pub fn pull_states(&mut self) -> Vec<RobotStatusPacket> {
        self.refresh();
        self.rx_storage.status.drain(..).collect()
    }

    /// Drains and returns all buffered command position packets.
    pub fn pull_command_positions(&mut self) -> Vec<CommandPositionResponsePacket> {
        self.refresh();
        self.rx_storage.command_position.drain(..).collect()
    }

    /// Blocks until a command position packet arrives, or returns `None` after `timeout_secs`.
    #[on(pyo3(signature = (timeout_secs = 0.2)))]
    pub fn wait_for_command_position(
        &mut self,
        timeout_secs: f64,
    ) -> Option<CommandPositionResponsePacket> {
        // Real clock watchdog: a virtual deadline would park this loop under a
        // paused shim clock and stop refresh() from draining responses.
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs_f64(timeout_secs) {
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

/// A cycle-by-cycle control session: suspends the I/O thread's automatic filler
/// replies so the caller can answer each robot status with [`send_command`](Self::send_command).
/// Filler replies resume on drop.
#[derive(Debug)]
pub struct StmoControlLoop<'a> {
    driver: &'a mut StreamMotionDriver,
}

impl<'a> StmoControlLoop<'a> {
    /// Begins a control session on the given driver.
    ///
    /// # Errors
    /// [`StreamMotionError::NotConnected`] if the driver has no live connection.
    pub fn try_new(driver: &'a mut StreamMotionDriver) -> Result<Self, StreamMotionError> {
        if let Some(cnx) = &mut driver.connection {
            cnx.itl.1.store(true, Ordering::SeqCst);
            Ok(Self { driver })
        } else {
            Err(StreamMotionError::NotConnected)
        }
    }

    /// Blocks until the next robot status packet arrives.
    ///
    /// # Errors
    /// [`StreamMotionError::Timeout`] if no status arrives within `timeout`;
    /// [`StreamMotionError::NotConnected`] or [`StreamMotionError::NotStarted`]
    /// if the connection or session is gone.
    pub fn wait_for_status(
        &mut self,
        timeout: Duration,
    ) -> Result<RobotStatusPacket, StreamMotionError> {
        // Register the listener and verify ITL state before draining pending
        // packets — otherwise a status that arrives between refresh() and
        // listen() fires notify() with no listeners and is silently lost.
        let listener = match &self.driver.connection {
            Some(cnx) => {
                if !cnx.itl.1.load(Ordering::SeqCst) {
                    return Err(StreamMotionError::NotStarted);
                }
                cnx.itl.0.listen()
            }
            None => return Err(StreamMotionError::NotConnected),
        };
        self.driver.refresh();
        if let Some(pkt) = self.driver.rx_storage.status.pop_back() {
            return Ok(pkt);
        }
        if listener.wait_timeout(timeout).is_some() {
            self.driver.refresh();
            if let Some(pkt) = self.driver.rx_storage.status.pop_back() {
                return Ok(pkt);
            }
        }
        Err(StreamMotionError::Timeout)
    }

    /// Sends a single motion command in reply to the most recent status.
    ///
    /// # Errors
    /// [`StreamMotionError::NotConnected`] or [`StreamMotionError::NotStarted`] if the
    /// driver is no longer connected and started.
    #[inline]
    pub fn send_command(&mut self, motion: MotionCommandPacket) -> DriverResult<()> {
        self.driver
            .command_motion_single(motion)
            .map_err(Into::into)
    }
}

impl Drop for StmoControlLoop<'_> {
    fn drop(&mut self) {
        if let Some(cnx) = &mut self.driver.connection {
            cnx.itl.1.store(false, Ordering::SeqCst);
        }
    }
}

impl StreamMotionDriver {
    /// Begins an [`StmoControlLoop`] session on this driver.
    ///
    /// # Errors
    /// [`StreamMotionError::NotConnected`] if the driver has no live connection.
    pub fn control_loop(&mut self) -> Result<StmoControlLoop<'_>, StreamMotionError> {
        StmoControlLoop::try_new(self)
    }
}

/// Python bindings for the STMO driver.
#[cfg(feature = "py")]
pub mod py {
    use crate::stmo::types::JointMovementLimit;

    use super::*;

    /// Python context-manager counterpart of [`StmoControlLoop`].
    #[derive(Debug)]
    #[pyclass(name = "StmoControlLoop")]
    pub struct PyStmoControlLoop {
        inner: Py<StreamMotionDriver>,
    }

    #[pymethods]
    impl PyStmoControlLoop {
        fn __enter__<'p>(slf: PyRef<'p, Self>, py: Python<'p>) -> PyResult<PyRef<'p, Self>> {
            if let Some(cnx) = &mut slf.inner.borrow_mut(py).connection {
                cnx.itl.1.store(true, Ordering::SeqCst);
            } else {
                return Err(StreamMotionError::NotConnected.into());
            }
            Ok(slf)
        }

        fn __exit__<'a>(
            &mut self,
            py: Python<'a>,
            _exc_type: Bound<'a, PyAny>,
            _exc_value: Bound<'a, PyAny>,
            _traceback: Bound<'a, PyAny>,
        ) -> PyResult<()> {
            if let Some(cnx) = &mut self.inner.borrow_mut(py).connection {
                cnx.itl.1.store(false, Ordering::SeqCst);
            }
            Ok(())
        }

        /// Blocks (GIL released) until the next robot status packet arrives.
        ///
        /// # Errors
        /// Timeout if no status arrives within `timeout_secs`; not-connected or
        /// not-started if the connection or session is gone.
        pub fn wait_for_status(
            &mut self,
            py: Python<'_>,
            timeout_secs: f32,
        ) -> PyResult<RobotStatusPacket> {
            let timeout = Duration::from_secs_f32(timeout_secs);

            // Clone the shared itl Arc and register the listener BEFORE
            // refresh()/drain — otherwise a status that arrives between
            // refresh() and listen() fires notify() with no listeners and is
            // silently lost (event_listener doesn't buffer for unsubscribed
            // listeners). The borrow is also dropped before blocking so other
            // stmo_driver methods can run during the timeout window.
            let listener = {
                let driver = self.inner.borrow(py);
                match &driver.connection {
                    Some(cnx) => {
                        if !cnx.itl.1.load(Ordering::SeqCst) {
                            return Err(StreamMotionError::NotStarted.into());
                        }
                        cnx.itl.0.listen()
                    }
                    None => return Err(StreamMotionError::NotConnected.into()),
                }
            };

            // Drain any status that arrived before we registered the listener.
            {
                let mut driver = self.inner.borrow_mut(py);
                driver.refresh();
                if let Some(pkt) = driver.rx_storage.status.pop_back() {
                    return Ok(pkt);
                }
            }

            // Wait for the next status notification with the GIL released so
            // other Python threads (and other stmo_driver methods) can run.
            let woke = py.detach(|| listener.wait_timeout(timeout).is_some());

            if woke {
                let mut driver = self.inner.borrow_mut(py);
                driver.refresh();
                if let Some(pkt) = driver.rx_storage.status.pop_back() {
                    return Ok(pkt);
                }
            }
            Err(StreamMotionError::Timeout.into())
        }

        /// Sends a single motion command in reply to the most recent status.
        ///
        /// # Errors
        /// Not-connected or not-started if used outside an entered context manager.
        pub fn send_command(
            &mut self,
            py: Python<'_>,
            motion: MotionCommandPacket,
        ) -> PyResult<()> {
            let mut driver = self.inner.borrow_mut(py);
            if let Some(cnx) = &mut driver.connection {
                if !cnx.itl.1.load(Ordering::SeqCst) {
                    return Err(StreamMotionError::NotStarted.into());
                }
                driver.command_motion_single(motion)
            } else {
                Err(StreamMotionError::NotConnected.into())
            }
        }
    }

    #[pymethods]
    impl StreamMotionDriver {
        /// Returns a control-loop context manager for this driver.
        #[pyo3(name = "control_loop")]
        pub fn py_control_loop(slf: Bound<'_, StreamMotionDriver>) -> PyResult<PyStmoControlLoop> {
            Ok(PyStmoControlLoop {
                inner: slf.unbind(),
            })
        }
    }

    /// Registers the STMO driver classes on the given Python module.
    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<AxisMotionConstraint>()?;
        parent_module.add_class::<JointMovementLimit>()?;
        parent_module.add_class::<JointMovementLimits>()?;
        parent_module.add_class::<StmoStats>()?;
        parent_module.add_class::<StreamMotionDriver>()?;
        parent_module.add_class::<PyStmoControlLoop>()?;

        Ok(())
    }
}
