pub mod driver;
pub mod errors;
pub mod proto;
pub mod rmi_handle;

use serde::{Deserialize, Serialize};

use proto::{
    commands::{Command, CommandResponse},
    communication::{Communication, CommunicationResponse},
    instructions::{Instruction, InstructionResponse},
};

use crate::rmi::errors::PacketMismatchError;

pub use driver::{RmiDriver, RmiDriverConfig};

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum SoftwareOptions {
    // Acceleration limiter
    R921 = 921,
    // Real time singularity avoidance
    R792 = 792,
    // MROT
    R640 = 640,
    // Spline
    R904 = 904,
    // Advanced Constant Path
    R806 = 806,
}
impl SoftwareOptions {
    pub(crate) const fn gate(self) -> FeatureGates {
        FeatureGates::SoftwareOption(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FeatureGates {
    MajorVersion(u8),
    SoftwareOption(SoftwareOptions),
    FieldPresent(&'static str),
    #[allow(dead_code)]
    FieldAbsent(&'static str),
}

impl FeatureGates {
    pub(crate) fn validate(
        &self,
        name: String,
        major_version: u8,
        options: &[SoftwareOptions],
        fields: &[&str],
    ) -> Result<(), FeatureGateFailure> {
        match self {
            FeatureGates::MajorVersion(v) if major_version < *v => {
                Err(FeatureGateFailure::VersionTooLow(name, *v))
            }
            FeatureGates::SoftwareOption(opt) if !options.contains(opt) => {
                Err(FeatureGateFailure::MissingOption(name, *opt))
            }
            FeatureGates::FieldPresent(field) if !fields.contains(field) => {
                Err(FeatureGateFailure::MissingField(name, *field, "unknown"))
            }
            FeatureGates::FieldAbsent(field) if fields.contains(field) => Err(
                FeatureGateFailure::UnsupportedField(name, *field, "unknown"),
            ),
            _ => Ok(()),
        }
    }
}

#[derive(Debug, Clone, thiserror::Error)]
pub enum FeatureGateFailure {
    #[error("{0}: Field '{1}' is missing, required by '{2}'")]
    MissingField(String, &'static str, &'static str),
    #[error("{0}: Field '{1}' is unsupported when used with '{2}'")]
    UnsupportedField(String, &'static str, &'static str),
    #[error("{0}: Requires major version {1} or higher")]
    VersionTooLow(String, u8),
    #[error("{0}: Missing software option '{1:?}'")]
    MissingOption(String, SoftwareOptions),
}

pub(crate) struct FeatureLockEntry {
    pub packet_name: &'static str,
    pub field_gates: &'static [(&'static str, &'static [FeatureGates])],
}
impl FeatureLockEntry {
    const EMPTY: Self = Self {
        packet_name: "",
        field_gates: &[],
    };

    pub(crate) fn get(packet_name: &str) -> &'static Self {
        for entry in inventory::iter::<FeatureLockEntry> {
            if entry.packet_name == packet_name {
                return entry;
            }
        }
        &Self::EMPTY
    }
}
inventory::collect!(FeatureLockEntry);

#[macro_export]
#[doc(hidden)]
macro_rules! def_feature_lock {
    ($name:literal { $($field:literal => $gates:expr),* $(,)? } ) => {
        inventory::submit!($crate::rmi::FeatureLockEntry {
            packet_name: $name,
            field_gates: &[$(($field, $gates)),*],
        });
    };
}

const LCB_GATES: &[FeatureGates] = &[
    FeatureGates::FieldPresent("LCBType"),
    FeatureGates::FieldPresent("LCBValue"),
    FeatureGates::FieldPresent("PortType"),
    FeatureGates::FieldPresent("portNumber"),
    FeatureGates::FieldPresent("portValue"),
];

def_feature_lock!(
    "ALL" {
        "ALIM" => &[FeatureGates::MajorVersion(5), SoftwareOptions::R921.gate()],
        "ALIMREG" => &[FeatureGates::MajorVersion(5), SoftwareOptions::R921.gate()],
        "ToolOffsetPRNumber" => &[FeatureGates::MajorVersion(4)],
        "LCBType" => LCB_GATES,
        "LCBValue" => LCB_GATES,
        "PortType" => LCB_GATES,
        "portNumber" => LCB_GATES,
        "portValue" => LCB_GATES,
        "MROT" => &[SoftwareOptions::R640.gate(), FeatureGates::FieldPresent("WristJoint")],
    }
);
def_feature_lock!(
    "PACKET" {
        "FRC_CALL" => &[FeatureGates::MajorVersion(4)],
    }
);

const fn clamp_value(term_value: u8, lo: u8, hi: u8) -> u8 {
    if term_value < lo {
        lo
    } else if term_value > hi {
        hi
    } else {
        term_value
    }
}

#[macro_export]
#[doc(hidden)]
macro_rules! packet_wrap {
    ($enm:ident :: $variant:ident) => {
        ::paste::paste! {
            impl From<$variant> for $enm {
                #[inline]
                fn from(v: $variant) -> Self {
                    $enm::$variant(v)
                }
            }
            impl From<$variant> for $crate::rmi::SendPacket {
                #[inline]
                fn from(v: $variant) -> Self {
                    $crate::rmi::SendPacket::$enm($enm::from(v))
                }
            }
            impl TryFrom<$crate::rmi::SendPacket> for $variant {
                type Error = $crate::rmi::PacketMismatchError;
                #[inline]
                fn try_from(value: $crate::rmi::SendPacket) -> Result<Self, Self::Error> {
                    $enm::try_from(value).and_then(|v| {
                        if let $enm::$variant(inner) = v {
                            Ok(inner)
                        } else {
                            Err($crate::rmi::PacketMismatchError)
                        }
                    })
                }
            }
            impl $crate::rmi::SendablePacket for $variant {
                type Counterpart = [<$variant Response>];
            }
            #[cfg(feature = "py")]
            #[pyo3::pymethods]
            impl $variant {
                pub fn into_send_packet(&self) -> $crate::rmi::SendPacket {
                    $crate::rmi::SendPacket::from(self.clone())
                }
                #[staticmethod]
                pub fn from_send_packet(packet: crate::rmi::SendPacket) -> pyo3::PyResult<Self> {
                    let slf: Self = packet.try_into()
                        .map_err(|_| pyo3::exceptions::PyValueError::new_err("Packet type mismatch"))?;
                    Ok(slf)
                }
                #[staticmethod]
                pub fn counterpart_typeinfo<'a>(py: pyo3::Python<'a>) -> pyo3::Bound<'a, pyo3::types::PyType> {
                    pyo3::types::PyType::new::<<Self as $crate::rmi::SendablePacket>::Counterpart>(py)
                }
            }
            impl std::fmt::Display for $variant {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let json_string = serde_json::to_string_pretty(&$crate::rmi::SendPacket::from(self.clone()))
                        .map_err(|_| std::fmt::Error)?;
                    write!(f, "{}", json_string)
                }
            }
        }
    };
    ($enm:ident { $($variant:ident),* $(,)? }) => {
        $(packet_wrap!($enm :: $variant);)*
    };
}

#[macro_export]
#[doc(hidden)]
macro_rules! packet_response_wrap {
    ($enm:ident :: $variant:ident) => {
        ::paste::paste! {
            impl From<[<$variant Response>]> for $enm {
                #[inline]
                fn from(v: [<$variant Response>]) -> Self {
                    $enm::$variant(v)
                }
            }
            impl From<[<$variant Response>]> for $crate::rmi::ResponsePacket {
                #[inline]
                fn from(v: [<$variant Response>]) -> Self {
                    $crate::rmi::ResponsePacket::from($enm::from(v))
                }
            }
            impl TryFrom<$crate::rmi::ResponsePacket> for [<$variant Response>] {
                type Error = $crate::rmi::PacketMismatchError;
                #[inline]
                fn try_from(value: $crate::rmi::ResponsePacket) -> Result<Self, Self::Error> {
                    $enm::try_from(value).and_then(|v| {
                        if let $enm::$variant(inner) = v {
                            Ok(inner)
                        } else {
                            Err($crate::rmi::PacketMismatchError)
                        }
                    })
                }
            }
            impl $crate::rmi::ReceivablePacket for [<$variant Response>] {
                type Counterpart = $variant;
            }
            #[cfg(feature = "py")]
            #[pyo3::pymethods]
            impl [<$variant Response>] {
                pub fn into_response_packet(&self) -> $crate::rmi::ResponsePacket {
                    $crate::rmi::ResponsePacket::from(self.clone())
                }
                #[staticmethod]
                pub fn from_response_packet(packet: crate::rmi::ResponsePacket) -> pyo3::PyResult<Self> {
                    let slf: Self = packet.try_into()
                        .map_err(|_| pyo3::exceptions::PyValueError::new_err("Packet type mismatch"))?;
                    Ok(slf)
                }
                #[staticmethod]
                pub fn counterpart_typeinfo<'a>(py: pyo3::Python<'a>) -> pyo3::Bound<'a, pyo3::types::PyType> {
                    pyo3::types::PyType::new::<<Self as $crate::rmi::ReceivablePacket>::Counterpart>(py)
                }
            }
            impl std::fmt::Display for [<$variant Response>] {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let json_string = serde_json::to_string_pretty(&$crate::rmi::ResponsePacket::from(self.clone()))
                        .map_err(|_| std::fmt::Error)?;
                    write!(f, "{}", json_string)
                }
            }
        }
    };
    ($enm:ident { $($variant:ident),* $(,)? }) => {
        $(packet_response_wrap!($enm :: $variant);)*
    };
}

pub trait Packet<P>:
    Serialize
    + for<'de> Deserialize<'de>
    + Into<P>
    + TryFrom<P, Error = PacketMismatchError>
    + std::fmt::Debug
    + Clone
    + Sized
    + Send
    + Sync
    + 'static
{
}
pub trait SendablePacket: Packet<SendPacket> {
    type Counterpart: ReceivablePacket;
}
pub trait ReceivablePacket: Packet<ResponsePacket> {
    type Counterpart: SendablePacket;
}

impl<T, P> Packet<P> for T where
    T: Serialize
        + for<'de> Deserialize<'de>
        + Into<P>
        + TryFrom<P, Error = PacketMismatchError>
        + std::fmt::Debug
        + Clone
        + Sized
        + Send
        + Sync
        + 'static
{
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(deny_unknown_fields)]
#[doc(hidden)]
pub struct NeverPacket;
impl From<NeverPacket> for SendPacket {
    fn from(_: NeverPacket) -> Self {
        unreachable!("NeverPacket should never be instantiated")
    }
}
impl TryFrom<SendPacket> for NeverPacket {
    type Error = PacketMismatchError;
    fn try_from(_: SendPacket) -> Result<Self, Self::Error> {
        Err(PacketMismatchError)
    }
}
impl From<NeverPacket> for ResponsePacket {
    fn from(_: NeverPacket) -> Self {
        unreachable!("NeverPacket should never be instantiated")
    }
}
impl TryFrom<ResponsePacket> for NeverPacket {
    type Error = PacketMismatchError;
    fn try_from(_: ResponsePacket) -> Result<Self, Self::Error> {
        Err(PacketMismatchError)
    }
}
impl SendablePacket for NeverPacket {
    type Counterpart = NeverPacket;
}
impl ReceivablePacket for NeverPacket {
    type Counterpart = NeverPacket;
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(untagged)]
pub enum SendPacket {
    Communication(Communication),
    Command(Command),
    Instruction(Instruction),
    #[serde(skip)]
    #[doc(hidden)]
    #[non_exhaustive]
    Never(NeverPacket),
}
impl SendPacket {
    pub fn packet_name(&self) -> &'static str {
        match self {
            SendPacket::Communication(v) => v.packet_name(),
            SendPacket::Command(v) => v.packet_name(),
            SendPacket::Instruction(v) => v.packet_name(),
            SendPacket::Never(_) => unreachable!("NeverPacket should never be instantiated"),
        }
    }
}
impl From<Communication> for SendPacket {
    fn from(v: Communication) -> Self {
        SendPacket::Communication(v)
    }
}
impl From<Command> for SendPacket {
    fn from(v: Command) -> Self {
        SendPacket::Command(v)
    }
}
impl From<Instruction> for SendPacket {
    fn from(v: Instruction) -> Self {
        SendPacket::Instruction(v)
    }
}
impl TryFrom<SendPacket> for Communication {
    type Error = PacketMismatchError;
    fn try_from(value: SendPacket) -> Result<Self, Self::Error> {
        if let SendPacket::Communication(v) = value {
            Ok(v)
        } else {
            Err(PacketMismatchError)
        }
    }
}
impl TryFrom<SendPacket> for Command {
    type Error = PacketMismatchError;
    fn try_from(value: SendPacket) -> Result<Self, Self::Error> {
        if let SendPacket::Command(v) = value {
            Ok(v)
        } else {
            Err(PacketMismatchError)
        }
    }
}
impl TryFrom<SendPacket> for Instruction {
    type Error = PacketMismatchError;
    fn try_from(value: SendPacket) -> Result<Self, Self::Error> {
        if let SendPacket::Instruction(v) = value {
            Ok(v)
        } else {
            Err(PacketMismatchError)
        }
    }
}
impl SendablePacket for Communication {
    type Counterpart = CommunicationResponse;
}
impl SendablePacket for Command {
    type Counterpart = CommandResponse;
}
impl SendablePacket for Instruction {
    type Counterpart = InstructionResponse;
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(untagged)]
pub enum ResponsePacket {
    Communication(CommunicationResponse),
    Command(CommandResponse),
    Instruction(InstructionResponse),
    #[serde(skip)]
    #[doc(hidden)]
    #[non_exhaustive]
    Never(NeverPacket),
}
impl ResponsePacket {
    pub fn packet_name(&self) -> &'static str {
        match self {
            ResponsePacket::Communication(v) => v.packet_name(),
            ResponsePacket::Command(v) => v.packet_name(),
            ResponsePacket::Instruction(v) => v.packet_name(),
            ResponsePacket::Never(_) => unreachable!("NeverPacket should never be instantiated"),
        }
    }

    pub fn error_id(&self) -> u32 {
        match self {
            ResponsePacket::Communication(v) => v.error_id(),
            ResponsePacket::Command(v) => v.error_id(),
            ResponsePacket::Instruction(v) => v.error_id(),
            ResponsePacket::Never(_) => unreachable!("NeverPacket should never be instantiated"),
        }
    }
}
impl From<CommunicationResponse> for ResponsePacket {
    fn from(v: CommunicationResponse) -> Self {
        ResponsePacket::Communication(v)
    }
}
impl From<CommandResponse> for ResponsePacket {
    fn from(v: CommandResponse) -> Self {
        ResponsePacket::Command(v)
    }
}
impl From<InstructionResponse> for ResponsePacket {
    fn from(v: InstructionResponse) -> Self {
        ResponsePacket::Instruction(v)
    }
}
impl TryFrom<ResponsePacket> for CommunicationResponse {
    type Error = PacketMismatchError;
    fn try_from(value: ResponsePacket) -> Result<Self, Self::Error> {
        if let ResponsePacket::Communication(v) = value {
            Ok(v)
        } else {
            Err(PacketMismatchError)
        }
    }
}
impl TryFrom<ResponsePacket> for CommandResponse {
    type Error = PacketMismatchError;
    fn try_from(value: ResponsePacket) -> Result<Self, Self::Error> {
        if let ResponsePacket::Command(v) = value {
            Ok(v)
        } else {
            Err(PacketMismatchError)
        }
    }
}
impl TryFrom<ResponsePacket> for InstructionResponse {
    type Error = PacketMismatchError;
    fn try_from(value: ResponsePacket) -> Result<Self, Self::Error> {
        if let ResponsePacket::Instruction(v) = value {
            Ok(v)
        } else {
            Err(PacketMismatchError)
        }
    }
}
impl ReceivablePacket for CommunicationResponse {
    type Counterpart = Communication;
}
impl ReceivablePacket for CommandResponse {
    type Counterpart = Command;
}
impl ReceivablePacket for InstructionResponse {
    type Counterpart = Instruction;
}

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register_child_module(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        let child_module = PyModule::new(parent_module.py(), "rmi")?;
        driver::py::register(&child_module)?;
        rmi_handle::py::register(&child_module)?;
        proto::commands::py::register(&child_module)?;
        proto::communication::py::register(&child_module)?;
        proto::instructions::py::register(&child_module)?;
        proto::member_structs::py::register(&child_module)?;
        child_module.add_class::<SendPacket>()?;
        child_module.add_class::<ResponsePacket>()?;
        child_module.add_class::<SoftwareOptions>()?;
        parent_module.add_submodule(&child_module)
    }
}
