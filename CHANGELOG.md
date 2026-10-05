# Changelog

## Unreleased

Depends on snare 1.5.0, which is not on crates.io yet; the manifest takes it
by path. Publish snare 1.5.0 first, then replace the path with the version
before releasing this crate.

### Running under a snare driver

Everything in this section only takes effect when
`snare::sched::is_driven()` is true on the thread involved: snare's `shim`
feature is on, an accounting driver owns the thread's clock, and the thread
is not background, helper or driver-class. On hardware none of it runs.

- Every wait on a caller thread is snare-visible (`block_on`,
  `block_on_until`, `block_on_timeout`) with its deadline on the virtual
  clock: `ResponseHandle::wait`/`wait_timeout` on every handle,
  `RmiQueue` waits, `StreamMotionDriver::start`, `fetch_movement_limits`,
  `wait_for_command_position`, `StmoControlLoop::wait_for_status`,
  `HspoChannel::wait_for`, and the HMI connect handshake.
- The RMI, HMI and HSPO runner threads poll without a timeout.
  `HspoReceiver::is_connected` is computed from the last packet's time
  instead of a periodic sweep.
- STMO send retries sleep with `snare::thread::sleep` on the virtual clock.
- STMO binds its local socket to port 0.
- Disconnecting any driver, and `hspo::destroy_broker(true)`, waits for the
  runner thread's exit in a way snare sees (bounded at 5 s virtual) before
  joining it.
- STMO answer changes are stamped with the instant the caller made them and
  apply only to statuses received strictly after that instant: motion
  batches, `stop`, `set_hold_read_io` and entering or leaving a control loop.
  A change made while reacting to status `k` therefore always takes effect at
  status `k + 1`, whatever the thread interleaving. A stamped `stop` goes out
  in place of the reply to the first status after it.

### Changes that also apply on hardware

- `StmoHandle`, `RmiHandle`/`RmiHandleGeneric` and `HmiHandle`/
  `HmiHandleGeneric` wake every task awaiting any clone of the handle
  (`snare::sched::WakerSet`), not only the last one to poll. Blocking waits
  register before checking, which closes a lost-wakeup window in the STMO
  and HMI handles.
- The runner threads receive their `mio` poller from the caller instead of
  handing a waker back over a channel. Socket and poller setup failures now
  surface from `connect`/`initialize_broker`.
- Received statuses carry the instant the I/O thread read them.
- The STMO I/O thread applies queued caller messages before answering a
  status, so a batch queued just before a status arrives is used for it.
- RMI: the runner exits once told to die, after a final attempt to flush the
  queue (the `FRC_Disconnect` included); pending handles fail with
  `Disconnected` when it exits; `is_connected` turns false and
  `has_connection_errored` true when it stops on an error.
- HMI: pending and queued requests fail with `NotConnected` whenever the
  runner exits, including on an I/O error; `has_connection_errored` is set
  then.
- `hspo::destroy_broker` wakes the broker so it exits without waiting for a
  poll timeout.
- `flume`'s `async` feature is always on. The crate's `async` feature is kept
  as an empty alias, and `HspoChannel::recv_async` and
  `RmiQueueGeneric::wait_all_async` no longer need it. The `hmi` feature pulls
  in `event-listener`, which it always used. `atomic-waker` is no longer a
  dependency.
- `TelemetrySink` documents that hooks run on the reactive I/O thread and must
  be O(1) and non-blocking.

### Real-time tuning on fast-talker 0.3

- Socket options are applied before the socket is bound (stmo, hspo) or
  connected (rmi, hmi). stmo and hspo now accept `WinCpuAffinity`, and rmi
  and hmi accept `BindDevice`.
- An option the platform applied with a different value (a buffer capped by
  `net.core.rmem_max`, say) is logged like a skipped one.
- `tuning_report()` on `StreamMotionDriver`, `RmiDriver` and `HmiDriver`, and
  `hspo::broker_tuning_report()`, return what the connection's options did:
  `TuningReport` in Rust, `{"thread": ..., "socket": ...}` dicts in Python.
- A refused option's error names it as Python spells it:
  `rmi does not accept option rt_priority (RtPriority(80))`.
- Python: `fanuc_ucl.apply_process_options(options, *, strict=False)` and the
  `ProcessGuard` it returns, for process-wide settings.
- hspo receives through fast-talker's `Timestamped` with kernel stamps only.
  A packet's user-space stamp no longer sets the controller-to-system clock
  offset once a kernel-stamped packet has arrived, and on Linux a datagram
  the socket dropped for lack of buffer is logged.
- stmo reads transmit errors with fast-talker's `sockets::socket_errors`.
- The `hspo` feature no longer pulls in `libc`.

### New STMO API

- `StreamMotionDriver::next_status()`: a future for the first status received
  strictly after the call. Statuses already received are discarded and never
  returned. On hardware "already received" is everything buffered at the
  call.
- `StreamMotionDriver::recv_status_timeout(timeout)`: the blocking form.
- `StreamMotionDriver::set_hold_read_io(Some((io_type, index, mask)))`: a
  sticky `read_io` request on every hold filler, until cleared with `None`.
- `StreamMotionDriver::command_motion_with(motions, more_follows)`.
  `command_motion(m)` is `command_motion_with(m, false)`.
- `StmoStats::mid_stream_fillers` (hold fillers sent while the last finished
  batch said more follows, with a `fanuc_ucl::stmo` warning once per run of
  them) and `StmoStats::idle_holds`.
- `StreamMotionDriver::stats_handle()` returns a shareable `StmoStatsHandle`.
  Stats are now cumulative across reconnects of the same driver, and
  `stats()` no longer resets to zero while disconnected.
