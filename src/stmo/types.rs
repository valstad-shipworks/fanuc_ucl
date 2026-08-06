use std::{
    collections::VecDeque,
    sync::atomic::{AtomicU64, Ordering},
};

use cfg_mixin::cfg_mixin;
use serde::{Deserialize, Serialize};

use crate::{
    ResponseNotFulfilled,
    joints::JointDataSizeError,
    stmo::proto::{
        self, CommandPositionResponsePacket, RobotStatusPacket, ThresholdTableResponsePacket,
    },
};

/**
 * Motion constraints for a derivative (velocity, acceleration, jerk) on a single axes.
 * The 20 entries correspond to a lookup table based on the TCP speed in 20ths of vmax.
 * Index 0 is 5% of vmax, index 19 is 100% of vmax.
 * You can interpolate between entries for intermediate speeds and interpolate between
 * no_payload and max_payload for intermediate payloads.
 * All units are in degrees per second (velocity), degrees per second^2 (acceleration),
 * or degrees per second^3 (jerk).
 */
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(frozen, str, from_py_object))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct AxisMotionConstraint {
    pub no_payload: [f32; proto::THRESHOLD_TABLE_LENGTH],
    pub max_payload: [f32; proto::THRESHOLD_TABLE_LENGTH],
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl AxisMotionConstraint {
    #[cfg(off)]
    pub fn new(
        no_payload: [f32; proto::THRESHOLD_TABLE_LENGTH],
        max_payload: [f32; proto::THRESHOLD_TABLE_LENGTH],
    ) -> Self {
        Self {
            no_payload,
            max_payload,
        }
    }

    #[cfg(on)]
    #[new]
    #[pyo3(signature = (no_payload, max_payload))]
    pub fn new(no_payload: Vec<f32>, max_payload: Vec<f32>) -> pyo3::PyResult<Self> {
        let no_payload_array: [f32; proto::THRESHOLD_TABLE_LENGTH] =
            no_payload.try_into().map_err(|_| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "no_payload must have exactly {} elements",
                    proto::THRESHOLD_TABLE_LENGTH
                ))
            })?;
        let max_payload_array: [f32; proto::THRESHOLD_TABLE_LENGTH] =
            max_payload.try_into().map_err(|_| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "max_payload must have exactly {} elements",
                    proto::THRESHOLD_TABLE_LENGTH
                ))
            })?;
        Ok(Self {
            no_payload: no_payload_array,
            max_payload: max_payload_array,
        })
    }

    pub fn calculate(&self, tcp_speed: f64, payload: f64, vmax: f64, max_payload: f64) -> f64 {
        let n = proto::THRESHOLD_TABLE_LENGTH;
        let step = 1.0 / n as f64;

        let speed_ratio = if vmax > 0.0 { tcp_speed / vmax } else { 0.0 };
        let payload_ratio = if max_payload > 0.0 {
            (payload / max_payload).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let lookup = |table: &[f32; proto::THRESHOLD_TABLE_LENGTH]| -> f64 {
            if speed_ratio <= step {
                table[0] as f64
            } else if speed_ratio > 1.0 {
                let slope = (table[n - 1] as f64 - table[n - 2] as f64) / step;
                table[n - 1] as f64 + slope * (speed_ratio - 1.0)
            } else {
                let ci = speed_ratio / step - 1.0;
                let below = (ci.floor() as usize).min(n - 2);
                let above = below + 1;
                let t = ci - below as f64;
                table[below] as f64 + (table[above] as f64 - table[below] as f64) * t
            }
        };

        let no_payload_value = lookup(&self.no_payload);
        let max_payload_value = lookup(&self.max_payload);

        no_payload_value + (max_payload_value - no_payload_value) * payload_ratio
    }
}

impl Default for AxisMotionConstraint {
    fn default() -> Self {
        Self {
            no_payload: [0.0; proto::THRESHOLD_TABLE_LENGTH],
            max_payload: [0.0; proto::THRESHOLD_TABLE_LENGTH],
        }
    }
}

#[cfg(feature = "py")]
#[pyo3::pymethods]
impl AxisMotionConstraint {
    #[getter]
    fn get_no_payload(&self) -> Vec<f32> {
        self.no_payload.to_vec()
    }

    #[getter]
    fn get_max_payload(&self) -> Vec<f32> {
        self.max_payload.to_vec()
    }
}

impl std::fmt::Display for AxisMotionConstraint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opening = if cfg!(feature = "py") { "(" } else { "{" };
        let closing = if cfg!(feature = "py") { ")" } else { "}" };
        write!(
            f,
            "AxisMotionConstraint{}no_payload: {:?}, max_payload: {:?}{}",
            opening, self.no_payload, self.max_payload, closing
        )
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(frozen, str, from_py_object))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct JointMovementLimit {
    #[on(pyo3(get))]
    pub velocity: AxisMotionConstraint,
    #[on(pyo3(get))]
    pub acceleration: AxisMotionConstraint,
    #[on(pyo3(get))]
    pub jerk: AxisMotionConstraint,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl JointMovementLimit {
    #[on(new)]
    #[on(pyo3(signature = (velocity, acceleration, jerk)))]
    pub fn new(
        velocity: AxisMotionConstraint,
        acceleration: AxisMotionConstraint,
        jerk: AxisMotionConstraint,
    ) -> Self {
        Self {
            velocity,
            acceleration,
            jerk,
        }
    }

    pub fn calculate(
        &self,
        tcp_speed: f64,
        payload: f64,
        vmax: f64,
        max_payload: f64,
    ) -> (f64, f64, f64) {
        (
            self.velocity
                .calculate(tcp_speed, payload, vmax, max_payload),
            self.acceleration
                .calculate(tcp_speed, payload, vmax, max_payload),
            self.jerk.calculate(tcp_speed, payload, vmax, max_payload),
        )
    }
}

impl std::fmt::Display for JointMovementLimit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opening = if cfg!(feature = "py") { "(" } else { "{" };
        let closing = if cfg!(feature = "py") { ")" } else { "}" };
        write!(
            f,
            "JointMovementLimit{}velocity: {}, acceleration: {}, jerk: {}{}",
            opening, self.velocity, self.acceleration, self.jerk, closing
        )
    }
}

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(frozen, str, from_py_object))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct JointMovementLimits {
    pub vmax: u32,
    pub joints: [Option<JointMovementLimit>; 9],
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl JointMovementLimits {
    #[cfg(off)]
    pub fn new(vmax: u32, joints: [Option<JointMovementLimit>; 9]) -> Self {
        Self { vmax, joints }
    }

    #[cfg(on)]
    #[new]
    #[pyo3(signature = (vmax, joints))]
    pub fn new(vmax: u32, joints: Vec<JointMovementLimit>) -> pyo3::PyResult<Self> {
        if joints.len() > 9 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "joints must have at most 9 elements",
            ));
        } else if joints.len() < 6 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "joints must have at least 6 elements",
            ));
        }
        let mut joints_array: [Option<JointMovementLimit>; 9] = [None; 9];
        for (i, joint) in joints.into_iter().enumerate() {
            joints_array[i] = Some(joint);
        }
        Ok(Self {
            vmax,
            joints: joints_array,
        })
    }
}

#[cfg(feature = "py")]
#[pyo3::pymethods]
impl JointMovementLimits {
    #[getter]
    fn get_vmax(&self) -> u32 {
        self.vmax
    }

    #[getter]
    fn get_joints(&self) -> Vec<JointMovementLimit> {
        self.joints.iter().filter_map(|&j| j).collect()
    }

    fn as_json(&self) -> pyo3::PyResult<String> {
        serde_json::to_string(self).map_err(|e| {
            pyo3::exceptions::PyValueError::new_err(format!("Serialization error: {}", e))
        })
    }

    #[staticmethod]
    fn from_json(json_str: &str) -> pyo3::PyResult<Self> {
        serde_json::from_str(json_str).map_err(|e| {
            pyo3::exceptions::PyValueError::new_err(format!("Deserialization error: {}", e))
        })
    }
}

impl std::fmt::Display for JointMovementLimits {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opening = if cfg!(feature = "py") { "(" } else { "{" };
        let closing = if cfg!(feature = "py") { ")" } else { "}" };
        write!(
            f,
            "JointMovementLimits{}vmax: {}, joints: {:?}{}",
            opening,
            self.vmax,
            self.joints.iter().filter_map(|&j| j).collect::<Vec<_>>(),
            closing
        )
    }
}

#[derive(Debug)]
pub(crate) struct RxStorage {
    pub status: VecDeque<RobotStatusPacket>,
    pub command_position: VecDeque<CommandPositionResponsePacket>,
    pub threshold_table: VecDeque<ThresholdTableResponsePacket>,
}

impl RxStorage {
    pub fn new() -> Self {
        Self {
            status: VecDeque::with_capacity(64),
            command_position: VecDeque::with_capacity(16),
            threshold_table: VecDeque::with_capacity(32),
        }
    }

    pub fn prune(&mut self) {
        while self.status.len() > 50 {
            let _ = self.status.pop_front();
        }
        while self.command_position.len() > 10 {
            let _ = self.command_position.pop_front();
        }
        while self.threshold_table.len() > 25 {
            let _ = self.threshold_table.pop_front();
        }
    }

    pub fn clear(&mut self) {
        self.status.clear();
        self.command_position.clear();
        self.threshold_table.clear();
    }
}

/// I/O health counters for one Stream Motion connection, cumulative since
/// [`connect`](super::StreamMotionDriver::connect).
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(frozen, get_all, str, from_py_object))]
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct StmoStats {
    /// Sends that failed outright, including those whose retry window expired.
    pub send_failures: u64,
    /// Sends the kernel accepted for fewer bytes than the encoded packet.
    pub short_sends: u64,
    /// Sends that entered the retry loop because the socket refused the packet
    /// outright. These do not reach the controller twice — the first attempt
    /// put nothing on the wire.
    pub send_retries: u64,
    /// Status packets ignored because answering them would have repeated a
    /// sequence number the controller already consumed.
    pub stale_statuses: u64,
    /// Transmit errors read off the kernel error queue. Linux only; these are
    /// the drops that `send` would otherwise have reported as success.
    pub tx_errors: u64,
    /// Status packets the controller sent that never arrived, counted from
    /// gaps between consecutively received sequence numbers. Genuine loss on
    /// the wire.
    pub lost_statuses: u64,
    /// Controller cycles the driver did not answer one at a time. Includes
    /// [`lost_statuses`](Self::lost_statuses), plus cycles whose status did
    /// arrive but landed in the same read as a newer one — under host load only
    /// the newest is answered directly and the rest are refilled in a burst.
    pub missed_status_cycles: u64,
    /// Motion commands sent to refill the controller's buffer after a gap.
    pub catchup_commands: u64,
    /// Cycles left unanswered because the controller's buffer was full and
    /// another command would have overflowed it.
    pub overflow_skips: u64,
    /// Times the controller's buffer is believed to have run dry mid-motion.
    /// Non-zero means the robot faulted.
    pub underruns: u64,
    /// Status packets received while a send was being retried. These are
    /// forwarded to consumers as they arrive and commanded once the send lands.
    pub statuses_during_retry: u64,
    /// Commands currently believed to be queued on the controller, out of
    /// [`BUFFER_CAPACITY`](super::BUFFER_CAPACITY). A live gauge, not a total.
    pub buffer_depth: u8,
    /// Measured interpolation period in microseconds. A live gauge.
    pub cycle_us: u64,
}

impl std::fmt::Display for StmoStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opening = if cfg!(feature = "py") { "(" } else { "{" };
        let closing = if cfg!(feature = "py") { ")" } else { "}" };
        write!(
            f,
            "StmoStats{}send_failures: {}, short_sends: {}, send_retries: {}, stale_statuses: {}, tx_errors: {}, lost_statuses: {}, missed_status_cycles: {}, catchup_commands: {}, overflow_skips: {}, underruns: {}, statuses_during_retry: {}, buffer_depth: {}, cycle_us: {}{}",
            opening,
            self.send_failures,
            self.short_sends,
            self.send_retries,
            self.stale_statuses,
            self.tx_errors,
            self.lost_statuses,
            self.missed_status_cycles,
            self.catchup_commands,
            self.overflow_skips,
            self.underruns,
            self.statuses_during_retry,
            self.buffer_depth,
            self.cycle_us,
            closing
        )
    }
}

#[derive(Debug, Default)]
pub(crate) struct StmoCounters {
    pub send_failures: AtomicU64,
    pub short_sends: AtomicU64,
    pub send_retries: AtomicU64,
    pub stale_statuses: AtomicU64,
    pub tx_errors: AtomicU64,
    pub lost_statuses: AtomicU64,
    pub missed_status_cycles: AtomicU64,
    pub catchup_commands: AtomicU64,
    pub overflow_skips: AtomicU64,
    pub underruns: AtomicU64,
    pub statuses_during_retry: AtomicU64,
    pub buffer_depth: AtomicU64,
    pub cycle_us: AtomicU64,
}

impl StmoCounters {
    pub fn snapshot(&self) -> StmoStats {
        StmoStats {
            send_failures: self.send_failures.load(Ordering::Relaxed),
            short_sends: self.short_sends.load(Ordering::Relaxed),
            send_retries: self.send_retries.load(Ordering::Relaxed),
            stale_statuses: self.stale_statuses.load(Ordering::Relaxed),
            tx_errors: self.tx_errors.load(Ordering::Relaxed),
            lost_statuses: self.lost_statuses.load(Ordering::Relaxed),
            missed_status_cycles: self.missed_status_cycles.load(Ordering::Relaxed),
            catchup_commands: self.catchup_commands.load(Ordering::Relaxed),
            overflow_skips: self.overflow_skips.load(Ordering::Relaxed),
            underruns: self.underruns.load(Ordering::Relaxed),
            statuses_during_retry: self.statuses_during_retry.load(Ordering::Relaxed),
            buffer_depth: self.buffer_depth.load(Ordering::Relaxed) as u8,
            cycle_us: self.cycle_us.load(Ordering::Relaxed),
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum StreamMotionError {
    #[error("Timeout")]
    Timeout,
    #[error("Not connected")]
    NotConnected,
    #[error("Not started")]
    NotStarted,
    #[error("I/O Error: {0}")]
    Io(std::io::Error),
    #[error("{0}")]
    JointDataSizeError(#[from] JointDataSizeError),
    #[error("Invalid Joint Count: {0}")]
    InvalidJointCount(u8),
    #[error("Encoding Error: {0}")]
    EncodingError(String),
    #[error("Decoding Error: {0}")]
    DecodingError(String),
    #[error("{0}")]
    ResponseNotFulfilled(#[from] ResponseNotFulfilled),
    #[error("Other Error: {0}")]
    Other(String),
}

#[cfg(feature = "valuable")]
error_valuable!(StreamMotionError, "StreamMotionError");
impl From<std::io::Error> for StreamMotionError {
    fn from(err: std::io::Error) -> Self {
        if err.kind() == std::io::ErrorKind::TimedOut {
            StreamMotionError::Timeout
        } else {
            StreamMotionError::Io(err)
        }
    }
}

impl From<bincode::error::EncodeError> for StreamMotionError {
    fn from(err: bincode::error::EncodeError) -> Self {
        StreamMotionError::EncodingError(err.to_string())
    }
}

impl From<bincode::error::DecodeError> for StreamMotionError {
    fn from(err: bincode::error::DecodeError) -> Self {
        StreamMotionError::DecodingError(err.to_string())
    }
}

#[cfg(feature = "py")]
impl From<StreamMotionError> for pyo3::PyErr {
    fn from(err: StreamMotionError) -> Self {
        match err {
            StreamMotionError::Timeout => pyo3::exceptions::PyTimeoutError::new_err("Timeout"),
            StreamMotionError::Io(e) => {
                pyo3::exceptions::PyIOError::new_err(format!("I/O Error: {}", e))
            }
            _ => pyo3::exceptions::PyException::new_err(format!("StreamMotionError: {}", err)),
        }
    }
}
