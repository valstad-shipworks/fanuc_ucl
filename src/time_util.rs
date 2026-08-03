use std::time::{Duration, SystemTime};

/// The host clock protocol timestamps are anchored to, used for telemetry
/// stamps, response-handle fulfill times, and packet receive times when the
/// kernel supplied no rx timestamp.
///
/// `snare::time` is `std::time` until something turns snare's clock shim on, so
/// this is the wall clock everywhere it matters. Under a simulator it is the
/// virtual clock instead — the same clock the emulated controller's own `clock`
/// field advances on. The two have to share a rate: hspo's
/// `StreamClock::system_time_of` rebuilds a buffered packet's receive time as
/// `controller clock + (host time − clock) of the newest packet`, so if the
/// controller clock runs at a multiple of the host's, every packet that waited
/// in the channel comes back dated by that multiple.
pub(crate) fn host_now() -> SystemTime {
    let since_epoch = snare::time::SystemTime::now()
        .duration_since(snare::time::UNIX_EPOCH)
        .unwrap_or(Duration::ZERO);
    SystemTime::UNIX_EPOCH + since_epoch
}
