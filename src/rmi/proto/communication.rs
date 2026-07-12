use crate::{
    packet_response_wrap, packet_wrap,
    rmi::{NeverPacket, ResponsePacket},
    zst_filler,
};
use cfg_mixin::cfg_mixin;
use serde::{Deserialize, Serialize};

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcConnectResponse {
    #[on(pyo3(get))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get))]
    #[serde(rename = "PortNumber")]
    pub port_number: u16,
    #[on(pyo3(get))]
    #[serde(rename = "MajorVersion")]
    pub major_version: u16,
    #[on(pyo3(get))]
    #[serde(rename = "MinorVersion")]
    pub minor_version: u16,
}
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcDisconnectResponse {
    #[on(pyo3(get))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
}
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSystemFaultResponse {
    #[on(pyo3(get))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

zst_filler!(FrcConnect);
zst_filler!(FrcDisconnect);
zst_filler!(FrcTerminateResponse);

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(tag = "Communication")]
pub enum Communication {
    #[serde(rename = "FRC_Connect")]
    FrcConnect(FrcConnect),
    #[serde(rename = "FRC_Disconnect")]
    FrcDisconnect(FrcDisconnect),
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl Communication {
    #[on(getter)]
    pub fn packet_name(&self) -> &'static str {
        match self {
            Communication::FrcConnect(_) => "FRC_Connect",
            Communication::FrcDisconnect(_) => "FRC_Disconnect",
        }
    }
}

packet_wrap!(Communication {
    FrcConnect,
    FrcDisconnect
});

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(tag = "Communication")]
pub enum CommunicationResponse {
    #[serde(rename = "FRC_Connect")]
    FrcConnect(FrcConnectResponse),
    #[serde(rename = "FRC_Disconnect")]
    FrcDisconnect(FrcDisconnectResponse),
    #[serde(rename = "FRC_Terminate")]
    FrcTerminate(FrcTerminateResponse),
    #[serde(rename = "FRC_SystemFault")]
    FrcSystemFault(FrcSystemFaultResponse),
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl CommunicationResponse {
    #[on(getter)]
    pub fn packet_name(&self) -> &'static str {
        match self {
            CommunicationResponse::FrcConnect(_) => "FRC_Connect",
            CommunicationResponse::FrcDisconnect(_) => "FRC_Disconnect",
            CommunicationResponse::FrcTerminate(_) => "FRC_Terminate",
            CommunicationResponse::FrcSystemFault(_) => "FRC_SystemFault",
        }
    }

    #[on(getter)]
    pub fn error_id(&self) -> u32 {
        match self {
            CommunicationResponse::FrcConnect(v) => v.error_id,
            CommunicationResponse::FrcDisconnect(v) => v.error_id,
            CommunicationResponse::FrcTerminate(_) => 0,
            CommunicationResponse::FrcSystemFault(_) => 0,
        }
    }
}

packet_response_wrap!(CommunicationResponse {
    FrcConnect,
    FrcDisconnect
});

impl From<FrcTerminateResponse> for CommunicationResponse {
    fn from(v: FrcTerminateResponse) -> Self {
        CommunicationResponse::FrcTerminate(v)
    }
}
impl From<FrcTerminateResponse> for crate::rmi::ResponsePacket {
    fn from(v: FrcTerminateResponse) -> Self {
        crate::rmi::ResponsePacket::from(CommunicationResponse::from(v))
    }
}
impl TryFrom<crate::rmi::ResponsePacket> for FrcTerminateResponse {
    type Error = crate::rmi::PacketMismatchError;
    fn try_from(value: crate::rmi::ResponsePacket) -> Result<Self, Self::Error> {
        CommunicationResponse::try_from(value).and_then(|v| {
            if let CommunicationResponse::FrcTerminate(inner) = v {
                Ok(inner)
            } else {
                Err(crate::rmi::PacketMismatchError)
            }
        })
    }
}
impl crate::rmi::ReceivablePacket for FrcTerminateResponse {
    type Counterpart = NeverPacket;
}
impl std::fmt::Display for FrcTerminateResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let json_string = serde_json::to_string_pretty(&ResponsePacket::from(*self))
            .map_err(|_| std::fmt::Error)?;
        write!(f, "{}", json_string)
    }
}
impl From<FrcSystemFaultResponse> for CommunicationResponse {
    fn from(v: FrcSystemFaultResponse) -> Self {
        CommunicationResponse::FrcSystemFault(v)
    }
}
impl From<FrcSystemFaultResponse> for crate::rmi::ResponsePacket {
    fn from(v: FrcSystemFaultResponse) -> Self {
        crate::rmi::ResponsePacket::from(CommunicationResponse::from(v))
    }
}
impl TryFrom<crate::rmi::ResponsePacket> for FrcSystemFaultResponse {
    type Error = crate::rmi::PacketMismatchError;
    fn try_from(value: crate::rmi::ResponsePacket) -> Result<Self, Self::Error> {
        CommunicationResponse::try_from(value).and_then(|v| {
            if let CommunicationResponse::FrcSystemFault(inner) = v {
                Ok(inner)
            } else {
                Err(crate::rmi::PacketMismatchError)
            }
        })
    }
}
impl crate::rmi::ReceivablePacket for FrcSystemFaultResponse {
    type Counterpart = NeverPacket;
}
impl std::fmt::Display for FrcSystemFaultResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let json_string = serde_json::to_string_pretty(&ResponsePacket::from(*self))
            .map_err(|_| std::fmt::Error)?;
        write!(f, "{}", json_string)
    }
}

pub(crate) fn connect_json() -> serde_json::Map<String, serde_json::Value> {
    let mut map = serde_json::Map::new();
    map.insert(
        "Communication".to_string(),
        serde_json::Value::String("FRC_Connect".to_string()),
    );
    map
}

pub(crate) fn disconnect_json() -> serde_json::Map<String, serde_json::Value> {
    let mut map = serde_json::Map::new();
    map.insert(
        "Communication".to_string(),
        serde_json::Value::String("FRC_Disconnect".to_string()),
    );
    map
}

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<FrcConnect>()?;
        parent_module.add_class::<FrcConnectResponse>()?;
        parent_module.add_class::<FrcDisconnect>()?;
        parent_module.add_class::<FrcDisconnectResponse>()?;
        parent_module.add_class::<FrcTerminateResponse>()?;
        parent_module.add_class::<FrcSystemFaultResponse>()?;
        parent_module.add_class::<Communication>()?;
        parent_module.add_class::<CommunicationResponse>()?;

        Ok(())
    }
}
