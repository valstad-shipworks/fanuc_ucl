use std::time::SystemTime;

/// The host clock protocol timestamps are anchored to, used for telemetry
/// stamps, response-handle fulfill times, and packet receive times when the
/// kernel supplied no rx timestamp.
///
/// hspo's `StreamClock::system_time_of` rebuilds a buffered packet's receive
/// time as `controller clock + (host time − clock) of the newest packet`, so
/// this clock and the controller's have to run at the same rate.
pub(crate) fn host_now() -> SystemTime {
    SystemTime::now()
}
