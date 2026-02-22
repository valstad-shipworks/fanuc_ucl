#![allow(clippy::useless_conversion)]
mod asg_handle;
mod hmi_handle;
mod proto;
mod runner;

#[cfg(feature = "py")]
pub mod py;

use bincode::config;
use flume::Sender;
use std::collections::HashMap;
use std::convert::TryFrom;
use std::io::{Error as IoError, ErrorKind};
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::time::{Duration, Instant};

use crate::hmi::asg::AsgEntry;
use crate::hmi::proto::ports::{
    self, ReadableDataPort, UnsafelyWritableDataPort, WritableDataPort,
};
use crate::hmi::proto::wire::Message;
use crate::hmi::runner::{HmiRunner, RunnerMessage};
use crate::{
    ResponseNotFulfilled,
    thread_util::{ThreadConfig, ThreadHandle},
};

pub use hmi_handle::{HmiHandle, HmiHandleGeneric};
use hmi_handle::{caster_array, caster_null, caster_singular};

pub use asg_handle::{
    AlarmArgs, AsgArgument, AsgVarInterface, BoolIoArgs, CurPosArgs, IntIoArgs, NumRegArgs,
    PosRegArgs, ProgramStatusArgs, StringRegArgs, SysVarArgs,
};

pub use ports::{
    AnalogInput, AnalogOutput, Command, DigitalInput, DigitalOutput, GroupInput, GroupOutput,
    Register, RobotInput, RobotOutput, SopInput, SopOutput, UopInput, UopOutput, WeldInput,
    WeldOutput, WireStickInput, WireStickOutput,
};
pub use proto::asg;

/// Errors that can occur during HMI communication with a FANUC robot controller.
#[derive(Debug, thiserror::Error)]
pub enum HmiError {
    #[error("Timeout")]
    Timeout,
    #[error("Not connected")]
    NotConnected,
    #[error("Not started")]
    NotStarted,
    #[error("I/O Error: {0}")]
    Io(std::io::Error),
    #[error("Invalid Int Size: {0}")]
    InvalidIntSize(#[from] std::num::TryFromIntError),
    #[error("Zero Index")]
    ZeroIndex,
    #[error("Encoding Error: {0}")]
    Encoding(#[from] bincode::error::EncodeError),
    #[error("Decoding Error: {0}")]
    Decoding(#[from] bincode::error::DecodeError),
    #[error("Bincode Error: {0}")]
    Bincode(String),
    #[error("Unexpected Sequence Number: {0}")]
    UnexpectedSequenceNumber(u8),
    #[error("Malformed Response")]
    MalformedResponse,
    #[error("{0}")]
    ResponseNotFulfilled(#[from] ResponseNotFulfilled),
    #[error("Other Error: {0}")]
    Other(String),
}

impl From<std::io::Error> for HmiError {
    fn from(err: std::io::Error) -> Self {
        if err.kind() == std::io::ErrorKind::TimedOut {
            HmiError::Timeout
        } else {
            HmiError::Io(err)
        }
    }
}

impl Clone for HmiError {
    fn clone(&self) -> Self {
        match self {
            HmiError::Timeout => HmiError::Timeout,
            HmiError::NotConnected => HmiError::NotConnected,
            HmiError::NotStarted => HmiError::NotStarted,
            HmiError::Io(e) => HmiError::Io(std::io::Error::new(e.kind(), e.to_string())),
            HmiError::InvalidIntSize(e) => HmiError::InvalidIntSize(*e),
            HmiError::ZeroIndex => HmiError::ZeroIndex,
            HmiError::Encoding(e) => HmiError::Bincode(format!("{}", e)),
            HmiError::Decoding(e) => HmiError::Bincode(format!("{}", e)),
            HmiError::Bincode(s) => HmiError::Bincode(s.clone()),
            HmiError::UnexpectedSequenceNumber(n) => HmiError::UnexpectedSequenceNumber(*n),
            HmiError::MalformedResponse => HmiError::MalformedResponse,
            HmiError::ResponseNotFulfilled(_e) => {
                HmiError::ResponseNotFulfilled(ResponseNotFulfilled)
            }
            HmiError::Other(s) => HmiError::Other(s.clone()),
        }
    }
}

#[cfg(feature = "py")]
impl From<HmiError> for pyo3::PyErr {
    fn from(err: HmiError) -> Self {
        match err {
            HmiError::Timeout => pyo3::exceptions::PyTimeoutError::new_err("Timeout"),
            HmiError::Io(e) => pyo3::exceptions::PyIOError::new_err(format!("I/O Error: {}", e)),
            _ => pyo3::exceptions::PyException::new_err(format!("HMI: {}", err)),
        }
    }
}

#[cfg(feature = "py")]
type DriverResult<T> = pyo3::PyResult<T>;
#[cfg(not(feature = "py"))]
type DriverResult<T> = Result<T, HmiError>;

pub(crate) const BINCODE_CFG: config::Configuration<config::LittleEndian, config::Fixint> =
    bincode::config::standard()
        .with_little_endian()
        .with_fixed_int_encoding();

const HMI_DEFAULT_PORT: u16 = 60008;
const DEFAULT_CONNECT_TIMEOUT_SECS: f64 = 1.0;

#[derive(Debug)]
struct HmiConnection {
    handle: ThreadHandle,
    to_runner: Sender<RunnerMessage>,
    err_flag: Arc<AtomicBool>,
}

/// The main driver struct for interfacing with a FANUC robot via SNPX HMI.
/// This struct manages the connection to the HMI, sending commands, reading/writing data ports, and registering ASG variables.
#[cfg_attr(feature = "py", pyo3::pyclass)]
#[derive(Debug)]
pub struct HmiDriver {
    remote_addr: IpAddr,
    connection: Option<HmiConnection>,
    seq: AtomicU8,
    asg_entries: HashMap<String, Arc<AsgEntry>>,
}

impl HmiDriver {
    /// Creates a new HmiDriver instance with the specified remote IP address of the HMI.
    ///
    /// This does not immediately establish a connection to the HMI; the [connect] method must be called to do so.
    pub fn new<T: Into<IpAddr>>(remote_addr: T) -> Self {
        Self {
            remote_addr: remote_addr.into(),
            connection: None,
            seq: AtomicU8::new(0),
            asg_entries: HashMap::new(),
        }
    }

    /// Connects to the HMI and performs the necessary handshake to establish communication.
    ///
    /// This method is blocking and will wait for the connection to be established and the handshake to complete, with an optional timeout.
    pub fn connect(
        &mut self,
        timeout: Option<Duration>,
        thread_config: Option<ThreadConfig>,
    ) -> DriverResult<()> {
        let timeout =
            timeout.unwrap_or_else(|| Duration::from_secs_f64(DEFAULT_CONNECT_TIMEOUT_SECS));
        if timeout.is_zero() {
            return Err(HmiError::Other("Timeout must be positive".into()).into());
        }
        if self.connection.is_some() {
            return Ok(());
        }
        let addr = SocketAddr::new(self.remote_addr, HMI_DEFAULT_PORT);
        let (to_runner, from_driver) = flume::unbounded();
        let mut handle = ThreadHandle::new();
        let (join_handle, waker, err_flag) =
            HmiRunner::start(addr, handle.to_pass_in(), from_driver, thread_config)?;
        handle.set_handle(join_handle);
        handle.set_waker_mio(waker);
        self.seq.store(0, Ordering::SeqCst);
        self.connection = Some(HmiConnection {
            handle,
            to_runner,
            err_flag,
        });
        let start = Instant::now();
        let ack = self.send_message(Message::INIT)?.wait_timeout(timeout)?;
        self.next_seq(); // INIT uses seq 0
        if ack == Message::INIT_ACK {
            self.send_message(Message::MAGIC)?
                .wait_timeout(timeout.saturating_sub(start.elapsed()))?;
            self.next_seq(); // magic uses seq 1
            self.write::<ports::Command>(0, "CLRASG".to_string())?
                .wait_timeout(timeout.saturating_sub(start.elapsed()))?;
            Ok(())
        } else {
            Err(HmiError::Other("Failed to receive ACK for INIT".into()).into())
        }
    }

    /// Disconnects from the HMI, shutting down the runner thread and cleaning up resources.
    pub fn disconnect(&mut self, ignore_join: bool) -> DriverResult<()> {
        if let Some(conn) = self.connection.take() {
            let _ = conn.to_runner.send(RunnerMessage::Shutdown);
            conn.handle.wake().map_err(HmiError::from)?;
            if !ignore_join {
                conn.handle.join();
            }
            Ok(())
        } else {
            Err(HmiError::NotConnected.into())
        }
    }

    /// Checks if the driver is currently connected to the HMI.
    pub fn is_connected(&self) -> bool {
        self.connection
            .as_ref()
            .map(|conn| conn.handle.is_alive())
            .unwrap_or(false)
    }

    pub fn has_connection_errored(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.err_flag.load(Ordering::Relaxed)
        } else {
            false
        }
    }

    fn next_seq(&self) -> u8 {
        self.seq.fetch_add(1, Ordering::SeqCst)
    }

    fn get_connection(&self) -> DriverResult<&HmiConnection> {
        self.connection
            .as_ref()
            .filter(|conn| conn.handle.is_alive())
            .ok_or_else(|| HmiError::NotConnected.into())
    }

    fn send_message(&self, msg: Message) -> DriverResult<HmiHandleGeneric> {
        let conn = self.get_connection()?;
        let seq = msg.seq();
        log::trace!("Sending message: {:?}", msg);
        let encoded = bincode::encode_to_vec(msg, BINCODE_CFG).map_err(HmiError::from)?;
        let handle = HmiHandleGeneric::new();
        conn.to_runner
            .send(RunnerMessage::Send {
                seq,
                data: encoded,
                handle: handle.clone(),
            })
            .map_err(|e| HmiError::Io(IoError::new(ErrorKind::BrokenPipe, e)))?;
        log::trace!("Sent message");
        conn.handle.wake().map_err(HmiError::from)?;
        log::trace!("Woke wakere");
        Ok(handle)
    }

    fn send_asg_cmd(&self, entry: Arc<AsgEntry>, timeout: Duration) -> DriverResult<()> {
        let cmd = format!(
            "SETASG {} {} {} {}",
            entry.address, entry.size, entry.var_name, entry.multiply
        );
        log::debug!("Sending ASG command: {}", cmd);
        self.write::<ports::Command>(0, cmd)?
            .wait_timeout(timeout)
            .map_err(Into::into)
    }

    /// Writes to multiple contiguous port indexes for a given data port type, returning a handle to the asynchronous success response.
    ///
    /// # Safety
    /// This function is unsafe because it allows writing to read-only ports.
    /// Read only ports are advised against writing to but it is technically possible and mostly functional so it is exposed through a nuanced API here.
    pub fn write_array_unsafe<T: UnsafelyWritableDataPort>(
        &self,
        index: usize,
        values: &[T::ValueType],
    ) -> DriverResult<HmiHandle<()>> {
        let index = u16::try_from(index)?;
        if !T::ZERO_INDEXED && index == 0 {
            return Err(HmiError::ZeroIndex.into());
        }
        log::trace!(
            "Writing to port {} at index {} values {:?}",
            T::NAME,
            index,
            values
        );
        let seq = self.next_seq();
        let msg = Message::new_write_req::<T>(seq, index, values);
        let generic = self.send_message(msg)?;
        Ok(HmiHandle::new_from_generic(
            &generic,
            0,
            0,
            caster_null::<T>,
        ))
    }

    /// Writes a single value to a data port, returning a handle to the asynchronous success response.
    ///
    /// # Safety
    /// This function is unsafe because it allows writing to read-only ports.
    /// Read only ports are advised against writing to but it is technically possible and mostly functional so it is exposed through a nuanced API here.
    #[inline]
    pub fn write_unsafe<T: UnsafelyWritableDataPort>(
        &self,
        index: usize,
        value: T::ValueType,
    ) -> DriverResult<HmiHandle<()>> {
        self.write_array_unsafe::<T>(index, &[value])
    }

    /// Writes to multiple contiguous port indexes for a given writable data port type, returning a handle to the asynchronous success response.
    #[inline]
    pub fn write_array<T: WritableDataPort>(
        &self,
        index: usize,
        values: &[T::ValueType],
    ) -> DriverResult<HmiHandle<()>> {
        self.write_array_unsafe::<T>(index, values)
    }

    /// Writes a single value to a writable data port at the given index, returning a handle to the asynchronous success response.
    #[inline]
    pub fn write<T: WritableDataPort>(
        &self,
        index: usize,
        value: T::ValueType,
    ) -> DriverResult<HmiHandle<()>> {
        self.write_array::<T>(index, &[value])
    }

    /// Reads multiple contiguous values from a readable data port starting at the given index, returning a handle to the asynchronous response containing the values.
    pub fn read_array<T: ReadableDataPort>(
        &self,
        index: usize,
        count: usize,
    ) -> DriverResult<HmiHandle<Box<[T::ValueType]>>>
    where
        T::ValueType: Send + Sync + 'static,
    {
        let mut index = u16::try_from(index)?;
        let count = u16::try_from(count)?;
        if !T::ZERO_INDEXED && index == 0 {
            return Err(HmiError::ZeroIndex.into());
        }
        if !T::ZERO_INDEXED {
            index -= 1;
        }
        let seq = self.next_seq();
        let (index, count) = T::align_read(index, count);
        let msg = Message::new_read_req::<T>(seq, index, count);
        let generic = self.send_message(msg)?;
        Ok(HmiHandle::new_from_generic(
            &generic,
            index,
            count,
            caster_array::<T>,
        ))
    }

    /// Reads a single value from a readable data port at the given index, returning a handle to the asynchronous response.
    pub fn read<T: ReadableDataPort>(&self, index: usize) -> DriverResult<HmiHandle<T::ValueType>>
    where
        T::ValueType: Send + Sync + 'static,
    {
        let mut index = u16::try_from(index)?;
        if !T::ZERO_INDEXED && index == 0 {
            return Err(HmiError::ZeroIndex.into());
        }
        if !T::ZERO_INDEXED {
            index -= 1;
        }
        let seq = self.next_seq();
        let (index, count) = T::align_read(index, 1);
        let msg = Message::new_read_req::<T>(seq, index, 1);
        let generic = self.send_message(msg)?;
        Ok(HmiHandle::new_from_generic(
            &generic,
            index,
            count,
            caster_singular::<T>,
        ))
    }

    pub fn register_asg<T: AsgArgument>(
        &mut self,
        arg: T,
        timeout: Duration,
    ) -> DriverResult<AsgVarInterface<T::Ret>> {
        let mut entry = arg.to_asg_entry();
        if self.asg_entries.is_empty() {
            entry.address = 1;
            let entry_arc = Arc::new(entry);
            self.send_asg_cmd(entry_arc.clone(), timeout)?;
            self.asg_entries
                .insert(entry_arc.var_name.clone(), entry_arc.clone());
            return Ok(AsgVarInterface::new(entry_arc));
        }
        if let Some(entry) = self.asg_entries.get(&entry.var_name) {
            return Ok(AsgVarInterface::new(entry.clone()));
        }
        let mut max_address = 0u16;
        for (_, existing_entry) in self.asg_entries.iter() {
            let end_address = existing_entry.address + existing_entry.size;
            if end_address > max_address {
                max_address = end_address;
            }
        }
        entry.address = max_address;
        let entry_arc = Arc::new(entry);
        self.send_asg_cmd(entry_arc.clone(), timeout)?;
        self.asg_entries
            .insert(entry_arc.var_name.clone(), entry_arc.clone());
        Ok(AsgVarInterface::new(entry_arc))
    }

    pub fn register_asg_array<T: AsgArgument, const N: usize>(
        &mut self,
        arg: T,
        timeout: Duration,
    ) -> DriverResult<AsgVarInterface<T::Ret, N>> {
        let mut entry = arg.to_asg_entry();
        entry.size *= N as u16;
        if self.asg_entries.is_empty() {
            entry.address = 1;
            let entry_arc = Arc::new(entry);
            self.send_asg_cmd(entry_arc.clone(), timeout)?;
            self.asg_entries
                .insert(entry_arc.var_name.clone(), entry_arc.clone());
            return Ok(AsgVarInterface::new(entry_arc));
        }
        if let Some(entry) = self.asg_entries.get(&entry.var_name) {
            return Ok(AsgVarInterface::new(entry.clone()));
        }
        let mut max_address = 0u16;
        for (_, existing_entry) in self.asg_entries.iter() {
            let end_address = existing_entry.address + existing_entry.size;
            if end_address > max_address {
                max_address = end_address;
            }
        }
        entry.address = max_address;
        let entry_arc = Arc::new(entry);
        self.send_asg_cmd(entry_arc.clone(), timeout)?;
        self.asg_entries
            .insert(entry_arc.var_name.clone(), entry_arc.clone());
        Ok(AsgVarInterface::<T::Ret, N>::new(entry_arc))
    }

    /// Sends a command to clear all active alarms on the controller.
    pub fn clear_alarms(&self) -> DriverResult<HmiHandle<()>> {
        self.write::<ports::Command>(0, "CLRALM".to_string())
    }
}
