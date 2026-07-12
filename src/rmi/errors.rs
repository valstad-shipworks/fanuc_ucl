use int_enum::IntEnum;
use serde::{Deserialize, Serialize};
use std::fmt;

use crate::{ResponseNotFulfilled, rmi::FeatureGateFailure};

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketMismatchError;
impl std::fmt::Display for PacketMismatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Packet type does not match expected type")
    }
}
impl std::error::Error for PacketMismatchError {}

#[derive(Debug, thiserror::Error)]
pub enum RmiError {
    #[error("SerDe error: {0}")]
    Serde(String),
    #[error("Rmi packet structure error: {0}")]
    Structure(String),
    #[error("{0}")]
    PacketMismatch(#[from] PacketMismatchError),
    #[error("{0}")]
    ResponseNotFulfilled(#[from] ResponseNotFulfilled),
    #[error("Operation timed out")]
    Timeout,
    #[error("Rmi feature gate validation error: {0}")]
    FeatureGate(#[from] FeatureGateFailure),
    #[error("Received an unrecognized packet from the robot")]
    UnrecognizedPacket,
    #[error("Failed to parse RMI string: {0}")]
    RmiStringParseError(#[from] std::str::Utf8Error),
    #[error("Fanuc returned error#: {0}")]
    FanucErrorCode(#[from] RmiProtocolError),
    #[error("Failed to send data: {0}")]
    CommunicationError(#[from] std::io::Error),
    #[error("Fanuc appears to be disconnected")]
    Disconnected,
    #[error("Failed to initialize: {0}")]
    Initialization(String),
    #[error("Rmi system fault or terminate received")]
    SystemFaultOrTerminate,
}

#[cfg(feature = "valuable")]
error_valuable!(RmiError, "RmiError");

impl Clone for RmiError {
    fn clone(&self) -> Self {
        match self {
            RmiError::Serde(e) => RmiError::Serde(e.clone()),
            RmiError::Structure(s) => RmiError::Structure(s.clone()),
            RmiError::PacketMismatch(e) => RmiError::PacketMismatch(*e),
            RmiError::ResponseNotFulfilled(e) => RmiError::ResponseNotFulfilled(*e),
            RmiError::Timeout => RmiError::Timeout,
            RmiError::FeatureGate(e) => RmiError::FeatureGate(e.clone()),
            RmiError::UnrecognizedPacket => RmiError::UnrecognizedPacket,
            RmiError::RmiStringParseError(e) => RmiError::RmiStringParseError(*e),
            RmiError::FanucErrorCode(e) => RmiError::FanucErrorCode(*e),
            RmiError::CommunicationError(e) => RmiError::CommunicationError(e.kind().into()),
            RmiError::Disconnected => RmiError::Disconnected,
            RmiError::Initialization(s) => RmiError::Initialization(s.clone()),
            RmiError::SystemFaultOrTerminate => RmiError::SystemFaultOrTerminate,
        }
    }
}

impl From<serde_json::Error> for RmiError {
    fn from(e: serde_json::Error) -> Self {
        RmiError::Serde(e.to_string())
    }
}

pub type RmiResult<T> = Result<T, RmiError>;

/// Error codes reported by the RMI protocol.
///
/// Display messages intentionally mirror the wording in FANUC's RMI
/// documentation (including capitalization) so they can be cross-referenced
/// against the manual.
#[repr(u32)]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(Debug, Serialize, Deserialize, IntEnum, Clone, Default, Copy, PartialEq, Eq)]
pub enum RmiProtocolError {
    InternalSystemError = 2556929,
    InvalidUToolNumber = 2556930,
    InvalidUFrameNumber = 2556931,
    InvalidPositionRegister = 2556932,
    InvalidSpeedOverride = 2556933,
    CannotExecuteTPProgram = 2556934,
    ControllerServoOff = 2556935,
    CannotExecuteTPProgramDuplicate = 2556936,
    RMINotRunning = 2556937,
    TPProgramNotPaused = 2556938,
    CannotResumeTPProgram = 2556939,
    CannotResetController = 2556940,
    InvalidRMICommand = 2556941,
    RMICommandFail = 2556942,
    InvalidControllerState = 2556943,
    PleaseCyclePower = 2556944,
    InvalidPayloadSchedule = 2556945,
    InvalidMotionOption = 2556946,
    InvalidVisionRegister = 2556947,
    InvalidRMIInstruction = 2556948,
    InvalidValue = 2556949,
    InvalidTextString = 2556950,
    InvalidPositionData = 2556951,
    RMIInHoldState = 2556952,
    RemoteDeviceDisconnected = 2556953,
    RobotAlreadyConnected = 2556954,
    WaitForCommandDone = 2556955,
    WaitForInstructionDone = 2556956,
    InvalidSequenceIDNumber = 2556957,
    InvalidSpeedType = 2556958,
    InvalidSpeedValue = 2556959,
    InvalidTermType = 2556960,
    InvalidTermValue = 2556961,
    InvalidLCBPortType = 2556962,
    InvalidACCValue = 2556963,
    InvalidDestinationPosition = 2556964,
    InvalidVIAPosition = 2556965,
    InvalidPortNumber = 2556966,
    InvalidGroupNumber = 2556967,
    InvalidGroupMask = 2556968,
    JointMotionWithCOORD = 2556969,
    IncrementalMotionWithCOORD = 2556970,
    RobotInSingleStepMode = 2556971,
    InvalidPositionDataType = 2556972,
    ReadyForASCIIPacket = 2556973,
    ASCIIConversionFailed = 2556974,
    InvalidASCIIInstruction = 2556975,
    InvalidNumberOfGroups = 2556976,
    InvalidInstructionPacket = 2556977,
    InvalidASCIIStringPacket = 2556978,
    InvalidASCIIStringSize = 2556979,
    InvalidApplicationTool = 2556980,
    InvalidCallProgramName = 2556981,
    #[default]
    UnrecognizedFrcError = 0,
}

impl std::fmt::Display for RmiProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msg = match self {
            RmiProtocolError::InternalSystemError => "Internal System Error.",
            RmiProtocolError::InvalidUToolNumber => "Invalid UTool Number.",
            RmiProtocolError::InvalidUFrameNumber => "Invalid UFrame Number.",
            RmiProtocolError::InvalidPositionRegister => "Invalid Position Register.",
            RmiProtocolError::InvalidSpeedOverride => "Invalid Speed Override.",
            RmiProtocolError::CannotExecuteTPProgram => "Cannot Execute TP program.",
            RmiProtocolError::ControllerServoOff => "Controller Servo is Off.",
            RmiProtocolError::CannotExecuteTPProgramDuplicate => "Cannot Execute TP program.",
            RmiProtocolError::RMINotRunning => "RMI is Not Running.",
            RmiProtocolError::TPProgramNotPaused => "TP Program is Not Paused.",
            RmiProtocolError::CannotResumeTPProgram => "Cannot Resume TP Program.",
            RmiProtocolError::CannotResetController => "Cannot Reset Controller.",
            RmiProtocolError::InvalidRMICommand => "Invalid RMI Command.",
            RmiProtocolError::RMICommandFail => "RMI Command Fail.",
            RmiProtocolError::InvalidControllerState => "Invalid Controller State.",
            RmiProtocolError::PleaseCyclePower => "Please Cycle Power.",
            RmiProtocolError::InvalidPayloadSchedule => "Invalid Payload Schedule.",
            RmiProtocolError::InvalidMotionOption => "Invalid Motion Option.",
            RmiProtocolError::InvalidVisionRegister => "Invalid Vision Register.",
            RmiProtocolError::InvalidRMIInstruction => "Invalid RMI Instruction.",
            RmiProtocolError::InvalidValue => "Invalid Value.",
            RmiProtocolError::InvalidTextString => "Invalid Text String.",
            RmiProtocolError::InvalidPositionData => "Invalid Position Data.",
            RmiProtocolError::RMIInHoldState => "RMI is In HOLD State.",
            RmiProtocolError::RemoteDeviceDisconnected => "Remote Device Disconnected.",
            RmiProtocolError::RobotAlreadyConnected => "Robot is Already Connected.",
            RmiProtocolError::WaitForCommandDone => "Wait for Command Done.",
            RmiProtocolError::WaitForInstructionDone => "Wait for Instruction Done.",
            RmiProtocolError::InvalidSequenceIDNumber => "Invalid sequence ID number.",
            RmiProtocolError::InvalidSpeedType => "Invalid Speed Type.",
            RmiProtocolError::InvalidSpeedValue => "Invalid Speed Value.",
            RmiProtocolError::InvalidTermType => "Invalid Term Type.",
            RmiProtocolError::InvalidTermValue => "Invalid Term Value.",
            RmiProtocolError::InvalidLCBPortType => "Invalid LCB Port Type.",
            RmiProtocolError::InvalidACCValue => "Invalid ACC Value.",
            RmiProtocolError::InvalidDestinationPosition => "Invalid Destination Position.",
            RmiProtocolError::InvalidVIAPosition => "Invalid VIA Position.",
            RmiProtocolError::InvalidPortNumber => "Invalid Port Number.",
            RmiProtocolError::InvalidGroupNumber => "Invalid Group Number.",
            RmiProtocolError::InvalidGroupMask => "Invalid Group Mask.",
            RmiProtocolError::JointMotionWithCOORD => "Joint motion with COORD.",
            RmiProtocolError::IncrementalMotionWithCOORD => "Incremental motn with COORD.",
            RmiProtocolError::RobotInSingleStepMode => "Robot in Single Step Mode.",
            RmiProtocolError::InvalidPositionDataType => "Invalid Position Data Type.",
            RmiProtocolError::ReadyForASCIIPacket => "Ready for ASCII Packet.",
            RmiProtocolError::ASCIIConversionFailed => "ASCII Conversion Failed.",
            RmiProtocolError::InvalidASCIIInstruction => "Invalid ASCII Instruction.",
            RmiProtocolError::InvalidNumberOfGroups => "Invalid Number of Groups.",
            RmiProtocolError::InvalidInstructionPacket => "Invalid Instruction packet.",
            RmiProtocolError::InvalidASCIIStringPacket => "Invalid ASCII String packet.",
            RmiProtocolError::InvalidASCIIStringSize => "Invalid ASCII string size.",
            RmiProtocolError::InvalidApplicationTool => "Invalid Application Tool.",
            RmiProtocolError::InvalidCallProgramName => "Invalid Call Program Name.",
            RmiProtocolError::UnrecognizedFrcError => "Unrecognized FANUC Error ID",
        };

        write!(f, "{} ({})", msg, *self as u32)
    }
}

impl std::error::Error for RmiProtocolError {}

#[cfg(feature = "py")]
impl From<RmiError> for pyo3::PyErr {
    fn from(e: RmiError) -> Self {
        match e {
            RmiError::CommunicationError(e) => {
                pyo3::exceptions::PyIOError::new_err(format!("{}", e))
            }
            RmiError::Disconnected => {
                pyo3::exceptions::PyConnectionError::new_err("Fanuc appears to be disconnected")
            }
            RmiError::RmiStringParseError(e) => pyo3::exceptions::PyUnicodeDecodeError::new_err(
                format!("Failed to parse RMI string: {}", e),
            ),
            other => pyo3::exceptions::PyRuntimeError::new_err(format!("{}", other)),
        }
    }
}
