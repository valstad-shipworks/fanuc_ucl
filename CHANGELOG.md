# Changelog

## 2.0.0 — 2026-10-05

Real-time tuning moves to fast-talker 0.3 option lists, and fixes found by
new decoder fuzzing, simulated-controller and pytest suites. Requires
`fast-talker` 0.3; `snare` is no longer a runtime dependency.

### Breaking changes

- `ThreadConfig` is removed (Rust and Python). Every `connect` and
  `hspo::initialize_broker` takes `thread: &[ThreadOption]` and
  `socket: &[SocketOption]` (re-exported from fast-talker) instead of
  `Option<ThreadConfig>`; in Python `thread=` / `socket=` keywords taking any
  shape fast-talker accepts (`[("rt_priority", 80)]`, `{"dscp": 46}`, ...).
  An empty list applies nothing; `ThreadConfig`'s priority below 1 used to
  set `SCHED_OTHER` with nice -8.
- Each driver accepts only the options that suit its role (stmo: all; hspo:
  all thread options but `MacOsTimeConstraint`, receive-side socket options;
  rmi/hmi: no real-time thread classes, socket `BindDevice`/`Dscp`/
  `LinuxPriority`). A refused option fails `connect` with the new
  `InvalidOption` variant on `StreamMotionError`, `RmiError` and `HmiError`
  (`ValueError` in Python): `rmi does not accept option rt_priority (RtPriority(80))`.
  An option that is attempted and fails now fails `connect` instead of being
  logged.
- `hspo::initialize_broker` returns `HspoBrokerError` (`InvalidOption`,
  `Io`) instead of `HspoBrokerNotInitializedError`.
- `StmoStats` gains `mid_stream_fillers` and `idle_holds`, and is cumulative
  across reconnects of the same driver; `stats()` no longer resets to zero
  while disconnected.
- Python exception types: RMI `Timeout` raises `TimeoutError`; I/O errors
  from RMI and HMI raise the matching `OSError` subclass; HMI `NotConnected`
  raises `ConnectionError` and index errors `IndexError`.
- The `async` feature is an empty alias: `HspoChannel::recv_async` and
  `RmiQueueGeneric::wait_all_async` are always available.

### Added

- `tuning_report()` on `StreamMotionDriver`, `RmiDriver` and `HmiDriver`,
  and `hspo::broker_tuning_report()`: what the connection's options did, as
  `TuningReport` (`ReportSummary` lists) or a `{"thread", "socket"}` dict in
  Python. Options the platform skipped or adjusted are logged.
- Python `fanuc_ucl.apply_process_options(options, *, strict=False)` and
  `ProcessGuard` for process-wide settings.
- STMO: `next_status()`, `recv_status_timeout()`, `set_hold_read_io()`,
  `command_motion_with(motions, more_follows)`, `stats_handle()` /
  `StmoStatsHandle`.
- Python: `packet_name` on RMI `SendPacket`/`ResponsePacket`,
  `AlarmSeverity.None_`, `rmi.PalletizingMode` registered; stubs cover
  `TimeData`, `ApplicationType`, `StmoHandle`, `StmoStats`,
  `has_broker_errored`.

### Fixed

- RMI: replies are framed on `
` across reads and matched out of order
  (instructions by SequenceID, commands by name and echoed fields) instead
  of strictly FIFO; `FRC_Terminate` resolves every pending request;
  `FRC_Disconnect` is sent even when every slot is held and its reply is
  awaited for up to 500 ms; queued and pending requests fail with
  `Disconnected` when the runner exits; the SequenceID wraps to 1 instead of
  overflowing; the connect handshake honours the configured timeout end to
  end, including the TCP connect.
- HMI: requests never reuse a sequence number still awaiting a reply; pending
  and queued requests fail with `NotConnected` whenever the runner exits;
  `connect` reconnects after a dropped connection and tears down a failed
  handshake; reads near the top of the address space and short or malformed
  replies return errors instead of overflowing or panicking; joint-position
  ASG size corrected to 38 bytes.
- STMO: duplicate or overtaken statuses are not answered (counted as
  `stale_statuses`); a sequence restart needs a run of forward-stepping
  statuses; datagrams whose length does not match their type are dropped;
  queued caller messages are applied before a status is answered;
  `fetch_movement_limits` returns as soon as every axis is filled.
- HSPO: one corrupted clock no longer shifts the stream: wraps are settled
  against the packets before them and a contradicted wrap is undone; a
  user-space receive stamp no longer moves the clock offset once a
  kernel-stamped packet has arrived; `destroy_broker` wakes the broker.
- Handles wake every task awaiting any clone, not only the last to poll; a
  lost-wakeup window in STMO and HMI blocking waits is closed.
- Socket, poller and thread-option failures surface from `connect` /
  `initialize_broker` instead of only in the I/O thread's log.
- Python: blocking calls release the GIL; `__version__` reads the
  `fanuc_ucl` distribution; `fanuc_ucl.hmi` etc. resolve as attributes.

### Changed

- Sockets are created with fast-talker's `bind_udp` / `connect_tcp`, so
  options apply before bind/connect. hspo receives through `Timestamped`
  with kernel stamps only and logs datagrams the socket dropped; stmo reads
  transmit errors with `sockets::socket_errors`.
- STMO answer changes (motion batches, `stop`, `set_hold_read_io`, entering
  or leaving a control loop) apply only to statuses received after the call:
  a change made while reacting to status `k` takes effect at `k + 1`.
- `snare` and `atomic-waker` are no longer dependencies; `libc` is only
  pulled in by `stmo`; `hmi` enables `event-listener`.
