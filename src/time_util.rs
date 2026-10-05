use std::time::SystemTime;

/// The host clock protocol timestamps are anchored to, used for telemetry
/// stamps and response-handle fulfill times. It is the clock fast-talker's
/// receive stamps are in, kernel and user-space alike.
pub(crate) fn host_now() -> SystemTime {
    SystemTime::now()
}
