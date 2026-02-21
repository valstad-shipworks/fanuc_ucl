use crate::{
    joints::{JointFormat, JointTemplate},
    packet_response_wrap, packet_wrap, zst_filler,
};

use super::member_structs::*;
use cfg_mixin::cfg_mixin;
use serde::{Deserialize, Serialize};

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcWriteUToolData {
    #[on(pyo3(get, set))]
    #[serde(rename = "ToolNumber")]
    pub tool_number: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "Frame")]
    pub frame: FrameData,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcWriteUToolData {
    #[on(new)]
    #[on(pyo3(signature = (tool_number, frame, group=None)))]
    pub fn new(tool_number: i8, frame: FrameData, group: Option<u8>) -> Self {
        Self {
            group,
            tool_number,
            frame,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWriteUToolDataResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcWriteUFrameData {
    #[on(pyo3(get, set))]
    #[serde(rename = "FrameNumber")]
    pub frame_number: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "Frame")]
    pub frame: FrameData,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcWriteUFrameData {
    #[on(new)]
    #[on(pyo3(signature = (frame_number, frame, group=None)))]
    pub fn new(frame_number: i8, frame: FrameData, group: Option<u8>) -> Self {
        Self {
            group,
            frame_number,
            frame,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWriteUFrameDataResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcWritePositionRegister {
    #[on(pyo3(get, set))]
    #[serde(rename = "RegisterNumber")]
    pub register_number: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub congifuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcWritePositionRegister {
    #[on(new)]
    #[on(pyo3(signature = (register_number, configuration, position, group=None)))]
    pub fn new(
        register_number: u16,
        configuration: Configuration,
        position: Position,
        group: Option<u8>,
    ) -> Self {
        Self {
            group,
            register_number,
            position,
            congifuration: configuration,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWritePositionRegisterResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWriteDOUT {
    #[on(pyo3(get, set))]
    #[serde(rename = "PortNumber")]
    pub port_number: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "PortValue")]
    pub port_value: OnOff,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcWriteDOUT {
    #[on(new)]
    #[on(pyo3(signature = (port_num, port_val)))]
    pub fn new(port_num: u16, port_val: OnOff) -> Self {
        Self {
            port_number: port_num,
            port_value: port_val,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWriteDOUTResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetUFrameUTool {
    #[on(pyo3(get, set))]
    #[serde(rename = "UFrameNumber")]
    pub u_frame_number: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "UToolNumber")]
    pub u_tool_number: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcSetUFrameUTool {
    #[on(new)]
    #[on(pyo3(signature = (u_tool_number, u_frame_number, group=None)))]
    pub fn new(u_tool_number: u8, u_frame_number: u8, group: Option<u8>) -> Self {
        Self {
            group,
            u_tool_number,
            u_frame_number,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetUFrameUToolResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetOverRide {
    #[on(pyo3(get, set))]
    #[serde(rename = "Value")]
    pub value: u8,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcSetOverRide {
    #[on(new)]
    #[on(pyo3(signature = (value)))]
    pub fn new(value: u8) -> Self {
        Self { value }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetOverRideResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u16,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcResetResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadUToolData {
    #[on(pyo3(get, set))]
    #[serde(rename = "FrameNumber")]
    pub frame_number: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadUToolData {
    #[on(new)]
    #[on(pyo3(signature = (frame_number, group=None)))]
    pub fn new(frame_number: i8, group: Option<u8>) -> Self {
        Self {
            group,
            frame_number,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcReadUToolDataResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "UToolNumber")]
    pub utool_number: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "Frame")]
    pub frame: FrameData,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadUFrameData {
    #[on(pyo3(get, set))]
    #[serde(rename = "FrameNumber")]
    pub frame_number: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadUFrameData {
    #[on(new)]
    #[on(pyo3(signature = (frame_number, group=None)))]
    pub fn new(frame_number: i8, group: Option<u8>) -> Self {
        Self {
            group,
            frame_number,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcReadUFrameDataResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "UFrameNumber")]
    pub u_frame_number: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "Frame")]
    pub frame: FrameData,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcReadTCPSpeedResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TimeTag")]
    pub time_tag: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: f32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadPositionRegister {
    #[on(pyo3(get, set))]
    #[serde(rename = "RegisterNumber")]
    register_number: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadPositionRegister {
    #[on(new)]
    #[on(pyo3(signature = (register_number, group=None)))]
    pub fn new(register_number: u16, group: Option<u8>) -> Self {
        Self {
            group,
            register_number,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcReadPositionRegisterResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "RegisterNumber")]
    pub register_number: i16,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub config: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadJointAngles {
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadJointAngles {
    #[on(new)]
    #[on(pyo3(signature = (group=None)))]
    pub fn new(group: Option<u8>) -> Self {
        Self { group }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcReadJointAnglesResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TimeTag")]
    pub time_tag: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "JointAngle")]
    joint_angles: JointAngles,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadJointAnglesResponse {
    pub fn joints(&self, format: JointFormat, template: JointTemplate) -> JointAngles {
        format.convert_from(JointFormat::FanucDeg, template, self.joint_angles.clone())
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadError {
    #[on(pyo3(get, set))]
    #[serde(rename = "Count", default, skip_serializing_if = "Option::is_none")]
    pub count: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadError {
    #[on(new)]
    #[on(pyo3(signature = (count=None)))]
    pub fn new(count: Option<u8>) -> Self {
        Self { count }
    }
}

impl Default for FrcReadError {
    fn default() -> Self {
        FrcReadError::new(None)
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FrcReadErrorResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "Count", default, skip_serializing_if = "Option::is_none")]
    pub count: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorData")]
    pub error_data: String,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "ErrorData2",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub error_data2: Option<String>,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "ErrorData3",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub error_data3: Option<String>,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "ErrorData4",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub error_data4: Option<String>,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "ErrorData5",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub error_data5: Option<String>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadDIN {
    #[on(pyo3(get, set))]
    #[serde(rename = "PortNumber")]
    pub port_number: u16,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadDIN {
    #[on(new)]
    #[on(pyo3(signature = (port_number)))]
    pub fn new(port_number: u16) -> Self {
        Self { port_number }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadDINResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "PortNumber")]
    pub port_number: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "PortValue")]
    pub port_value: u8,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcReadCartesianPosition {
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcReadCartesianPosition {
    #[on(new)]
    #[on(pyo3(signature = (group=None)))]
    pub fn new(group: Option<u8>) -> Self {
        Self { group }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcReadCartesianPositionResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TimeTag")]
    pub time_tag: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub config: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub pos: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcPauseResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcInitialize {
    #[on(pyo3(get, set))]
    #[serde(rename = "GroupMask", default, skip_serializing_if = "Option::is_none")]
    pub group_mask: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "Application",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub application: Option<ApplicationType>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Equipment", default, skip_serializing_if = "Option::is_none")]
    pub equipment: Option<u8>,
    #[serde(rename = "RTSA", default, skip_serializing_if = "Option::is_none")]
    pub rtsa: Option<OnOnly>,
    #[on(pyo3(get, set))]
    #[serde(rename = "PLTZMODE", default, skip_serializing_if = "Option::is_none")]
    pub palletizing_mode: Option<PalletizingMode>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcInitialize {
    #[on(new)]
    #[on(pyo3(signature = (group_mask=None, application=None, equipment=None, rtsa=false, palletizing_mode=None)))]
    pub fn new(
        group_mask: Option<u8>,
        application: Option<ApplicationType>,
        equipment: Option<u8>,
        rtsa: bool,
        palletizing_mode: Option<PalletizingMode>,
    ) -> Self {
        Self {
            group_mask,
            application,
            equipment,
            rtsa: if rtsa {
                Some(::monostate::MustBe!("ON"))
            } else {
                None
            },
            palletizing_mode,
        }
    }
}

crate::impl_monostate_member!(FrcInitialize.rtsa = "ON");

// Keep trait impls *outside* of #[pymethods]; this is a normal Rust Default.
impl Default for FrcInitialize {
    fn default() -> Self {
        FrcInitialize::new(None, None, None, false, None)
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcInitializeResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "GroupMask", default, skip_serializing_if = "Option::is_none")]
    pub group_mask: Option<u16>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcGetUFrameUTool {
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcGetUFrameUTool {
    #[on(new)]
    #[on(pyo3(signature = (group=None)))]
    pub fn new(group: Option<u8>) -> Self {
        Self { group }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcGetUFrameUToolResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "UFrameNumber")]
    pub u_frame_number: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "UToolNumber")]
    pub u_tool_number: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "Group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<u8>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcGetStatusResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "ServoReady")]
    pub servo_ready: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "TPMode")]
    pub tp_mode: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "RMIMotionStatus")]
    pub rmi_motion_status: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ProgramStatus")]
    pub program_status: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "SingleStepMode")]
    pub single_step_mode: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "NumberUTool")]
    pub number_utool: i8,
    #[on(pyo3(get, set))]
    #[serde(rename = "NumberUFrame")]
    pub number_uframe: i8,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcContinueResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcAbortResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
}

zst_filler!(FrcAbort);
zst_filler!(FrcPause);
zst_filler!(FrcContinue);
zst_filler!(FrcReadTCPSpeed);
zst_filler!(FrcReset);
zst_filler!(FrcGetStatus);

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(tag = "Command")]
pub enum Command {
    #[serde(rename = "FRC_Initialize")]
    FrcInitialize(FrcInitialize),
    #[serde(rename = "FRC_Abort")]
    FrcAbort(FrcAbort),
    #[serde(rename = "FRC_Pause")]
    FrcPause(FrcPause),
    #[serde(rename = "FRC_ReadError")]
    FrcReadError(FrcReadError),
    #[serde(rename = "FRC_Continue")]
    FrcContinue(FrcContinue),
    #[serde(rename = "FRC_SetUFrameUTool")]
    FrcSetUFrameUTool(FrcSetUFrameUTool),
    #[serde(rename = "FRC_ReadPositionRegister")]
    FrcReadPositionRegister(FrcReadPositionRegister),
    #[serde(rename = "FRC_WritePositionRegister")]
    FrcWritePositionRegister(FrcWritePositionRegister),
    #[serde(rename = "FRC_SetOverRide")]
    FrcSetOverRide(FrcSetOverRide),
    #[serde(rename = "FRC_GetStatus")]
    FrcGetStatus(FrcGetStatus),
    #[serde(rename = "FRC_GetUFrameUTool")]
    FrcGetUFrameUTool(FrcGetUFrameUTool),
    #[serde(rename = "FRC_WriteUToolData")]
    FrcWriteUToolData(FrcWriteUToolData),
    #[serde(rename = "FRC_ReadUFrameData")]
    FrcReadUFrameData(FrcReadUFrameData),
    #[serde(rename = "FRC_WriteUFrameData")]
    FrcWriteUFrameData(FrcWriteUFrameData),
    #[serde(rename = "FRC_ReadUToolData")]
    FrcReadUToolData(FrcReadUToolData),
    #[serde(rename = "FRC_Reset")]
    FrcReset(FrcReset),
    #[serde(rename = "FRC_ReadDIN")]
    FrcReadDIN(FrcReadDIN),
    #[serde(rename = "FRC_WriteDOUT")]
    FrcWriteDOUT(FrcWriteDOUT),
    #[serde(rename = "FRC_ReadCartesianPosition")]
    FrcReadCartesianPosition(FrcReadCartesianPosition),
    #[serde(rename = "FRC_ReadJointAngles")]
    FrcReadJointAngles(FrcReadJointAngles),
    #[serde(rename = "FRC_ReadTCPSpeed")]
    FrcReadTCPSpeed(FrcReadTCPSpeed),
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl Command {
    #[on(getter)]
    pub fn packet_name(&self) -> &'static str {
        match self {
            Command::FrcInitialize(_) => "FRC_Initialize",
            Command::FrcAbort(_) => "FRC_Abort",
            Command::FrcPause(_) => "FRC_Pause",
            Command::FrcContinue(_) => "FRC_Continue",
            Command::FrcReadError(_) => "FRC_ReadError",
            Command::FrcSetUFrameUTool(_) => "FRC_SetUFrameUTool",
            Command::FrcGetUFrameUTool(_) => "FRC_GetUFrameUTool",
            Command::FrcGetStatus(_) => "FRC_GetStatus",
            Command::FrcReadUFrameData(_) => "FRC_ReadUFrameData",
            Command::FrcWriteUFrameData(_) => "FRC_WriteUFrameData",
            Command::FrcReadUToolData(_) => "FRC_ReadUToolData",
            Command::FrcWriteUToolData(_) => "FRC_WriteUToolData",
            Command::FrcReadDIN(_) => "FRC_ReadDIN",
            Command::FrcReadCartesianPosition(_) => "FRC_ReadCartesianPosition",
            Command::FrcWriteDOUT(_) => "FRC_WriteDOUT",
            Command::FrcReadJointAngles(_) => "FRC_ReadJointAngles",
            Command::FrcSetOverRide(_) => "FRC_SetOverRide",
            Command::FrcReadPositionRegister(_) => "FRC_ReadPositionRegister",
            Command::FrcWritePositionRegister(_) => "FRC_WritePositionRegister",
            Command::FrcReset(_) => "FRC_Reset",
            Command::FrcReadTCPSpeed(_) => "FRC_ReadTCPSpeed",
        }
    }
}

packet_wrap! {
    Command {
        FrcInitialize,
        FrcAbort,
        FrcPause,
        FrcContinue,
        FrcReadError,
        FrcSetUFrameUTool,
        FrcGetUFrameUTool,
        FrcGetStatus,
        FrcReadUFrameData,
        FrcWriteUFrameData,
        FrcReadUToolData,
        FrcWriteUToolData,
        FrcReadDIN,
        FrcReadCartesianPosition,
        FrcWriteDOUT,
        FrcReadJointAngles,
        FrcSetOverRide,
        FrcReadPositionRegister,
        FrcWritePositionRegister,
        FrcReset,
        FrcReadTCPSpeed
    }
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "Command")]
pub enum CommandResponse {
    #[serde(rename = "FRC_Initialize")]
    FrcInitialize(FrcInitializeResponse),

    #[serde(rename = "FRC_Abort")]
    FrcAbort(FrcAbortResponse),

    #[serde(rename = "FRC_Pause")]
    FrcPause(FrcPauseResponse),

    #[serde(rename = "FRC_Continue")]
    FrcContinue(FrcContinueResponse),

    #[serde(rename = "FRC_ReadError")]
    FrcReadError(FrcReadErrorResponse),

    #[serde(rename = "FRC_SetUFrameUTool")]
    FrcSetUFrameUTool(FrcSetUFrameUToolResponse),

    #[serde(rename = "FRC_GetUFrameUTool")]
    FrcGetUFrameUTool(FrcGetUFrameUToolResponse),

    #[serde(rename = "FRC_GetStatus")]
    FrcGetStatus(FrcGetStatusResponse),

    #[serde(rename = "FRC_ReadUFrameData")]
    FrcReadUFrameData(FrcReadUFrameDataResponse),

    #[serde(rename = "FRC_WriteUFrameData")]
    FrcWriteUFrameData(FrcWriteUFrameDataResponse),

    #[serde(rename = "FRC_ReadUToolData")]
    FrcReadUToolData(FrcReadUToolDataResponse),

    #[serde(rename = "FRC_WriteUToolData")]
    FrcWriteUToolData(FrcWriteUToolDataResponse),

    #[serde(rename = "FRC_ReadDIN")]
    FrcReadDIN(FrcReadDINResponse),

    #[serde(rename = "FRC_ReadCartesianPosition")]
    FrcReadCartesianPosition(FrcReadCartesianPositionResponse),

    #[serde(rename = "FRC_WriteDOUT")]
    FrcWriteDOUT(FrcWriteDOUTResponse),

    #[serde(rename = "FRC_ReadJointAngles")]
    FrcReadJointAngles(FrcReadJointAnglesResponse),

    #[serde(rename = "FRC_SetOverRide")]
    FrcSetOverRide(FrcSetOverRideResponse),

    #[serde(rename = "FRC_ReadPositionRegister")]
    FrcReadPositionRegister(FrcReadPositionRegisterResponse),

    #[serde(rename = "FRC_WritePositionRegister")]
    FrcWritePositionRegister(FrcWritePositionRegisterResponse),

    #[serde(rename = "FRC_Reset")]
    FrcReset(FrcResetResponse),

    #[serde(rename = "FRC_ReadTCPSpeed")]
    FrcReadTCPSpeed(FrcReadTCPSpeedResponse),
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl CommandResponse {
    #[on(getter)]
    pub fn packet_name(&self) -> &'static str {
        match self {
            CommandResponse::FrcInitialize(_) => "FRC_Initialize",
            CommandResponse::FrcAbort(_) => "FRC_Abort",
            CommandResponse::FrcPause(_) => "FRC_Pause",
            CommandResponse::FrcContinue(_) => "FRC_Continue",
            CommandResponse::FrcReadError(_) => "FRC_ReadError",
            CommandResponse::FrcSetUFrameUTool(_) => "FRC_SetUFrameUTool",
            CommandResponse::FrcGetUFrameUTool(_) => "FRC_GetUFrameUTool",
            CommandResponse::FrcGetStatus(_) => "FRC_GetStatus",
            CommandResponse::FrcReadUFrameData(_) => "FRC_ReadUFrameData",
            CommandResponse::FrcWriteUFrameData(_) => "FRC_WriteUFrameData",
            CommandResponse::FrcReadUToolData(_) => "FRC_ReadUToolData",
            CommandResponse::FrcWriteUToolData(_) => "FRC_WriteUToolData",
            CommandResponse::FrcReadDIN(_) => "FRC_ReadDIN",
            CommandResponse::FrcReadCartesianPosition(_) => "FRC_ReadCartesianPosition",
            CommandResponse::FrcWriteDOUT(_) => "FRC_WriteDOUT",
            CommandResponse::FrcReadJointAngles(_) => "FRC_ReadJointAngles",
            CommandResponse::FrcSetOverRide(_) => "FRC_SetOverRide",
            CommandResponse::FrcReadPositionRegister(_) => "FRC_ReadPositionRegister",
            CommandResponse::FrcWritePositionRegister(_) => "FRC_WritePositionRegister",
            CommandResponse::FrcReset(_) => "FRC_Reset",
            CommandResponse::FrcReadTCPSpeed(_) => "FRC_ReadTCPSpeed",
        }
    }

    #[on(getter)]
    pub fn error_id(&self) -> u32 {
        match self {
            CommandResponse::FrcInitialize(r) => r.error_id,
            CommandResponse::FrcAbort(r) => r.error_id,
            CommandResponse::FrcPause(r) => r.error_id,
            CommandResponse::FrcContinue(r) => r.error_id,
            CommandResponse::FrcReadError(_) => 0, // errors prevent reading that packet
            CommandResponse::FrcSetUFrameUTool(r) => r.error_id,
            CommandResponse::FrcGetUFrameUTool(r) => r.error_id,
            CommandResponse::FrcGetStatus(r) => r.error_id,
            CommandResponse::FrcReadUFrameData(r) => r.error_id,
            CommandResponse::FrcWriteUFrameData(r) => r.error_id,
            CommandResponse::FrcReadUToolData(r) => r.error_id,
            CommandResponse::FrcWriteUToolData(r) => r.error_id,
            CommandResponse::FrcReadDIN(r) => r.error_id,
            CommandResponse::FrcReadCartesianPosition(r) => r.error_id,
            CommandResponse::FrcWriteDOUT(r) => r.error_id,
            CommandResponse::FrcReadJointAngles(r) => r.error_id,
            CommandResponse::FrcSetOverRide(r) => r.error_id as u32,
            CommandResponse::FrcReadPositionRegister(r) => r.error_id,
            CommandResponse::FrcWritePositionRegister(r) => r.error_id,
            CommandResponse::FrcReset(r) => r.error_id,
            CommandResponse::FrcReadTCPSpeed(r) => r.error_id,
        }
    }
}

packet_response_wrap! {
    CommandResponse {
        FrcInitialize,
        FrcAbort,
        FrcPause,
        FrcContinue,
        FrcReadError,
        FrcSetUFrameUTool,
        FrcGetUFrameUTool,
        FrcGetStatus,
        FrcReadUFrameData,
        FrcWriteUFrameData,
        FrcReadUToolData,
        FrcWriteUToolData,
        FrcReadDIN,
        FrcReadCartesianPosition,
        FrcWriteDOUT,
        FrcReadJointAngles,
        FrcSetOverRide,
        FrcReadPositionRegister,
        FrcWritePositionRegister,
        FrcReset,
        FrcReadTCPSpeed
    }
}

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<FrcWriteUToolData>()?;
        parent_module.add_class::<FrcWriteUToolDataResponse>()?;
        parent_module.add_class::<FrcWriteUFrameData>()?;
        parent_module.add_class::<FrcWriteUFrameDataResponse>()?;
        parent_module.add_class::<FrcWritePositionRegister>()?;
        parent_module.add_class::<FrcWritePositionRegisterResponse>()?;
        parent_module.add_class::<FrcWriteDOUT>()?;
        parent_module.add_class::<FrcWriteDOUTResponse>()?;
        parent_module.add_class::<FrcSetUFrameUTool>()?;
        parent_module.add_class::<FrcSetUFrameUToolResponse>()?;
        parent_module.add_class::<FrcSetOverRide>()?;
        parent_module.add_class::<FrcSetOverRideResponse>()?;
        parent_module.add_class::<FrcReset>()?;
        parent_module.add_class::<FrcResetResponse>()?;
        parent_module.add_class::<FrcReadUToolData>()?;
        parent_module.add_class::<FrcReadUToolDataResponse>()?;
        parent_module.add_class::<FrcReadUFrameData>()?;
        parent_module.add_class::<FrcReadUFrameDataResponse>()?;
        parent_module.add_class::<FrcReadTCPSpeed>()?;
        parent_module.add_class::<FrcReadTCPSpeedResponse>()?;
        parent_module.add_class::<FrcReadPositionRegister>()?;
        parent_module.add_class::<FrcReadPositionRegisterResponse>()?;
        parent_module.add_class::<FrcReadJointAngles>()?;
        parent_module.add_class::<FrcReadJointAnglesResponse>()?;
        parent_module.add_class::<FrcReadError>()?;
        parent_module.add_class::<FrcReadErrorResponse>()?;
        parent_module.add_class::<FrcReadDIN>()?;
        parent_module.add_class::<FrcReadDINResponse>()?;
        parent_module.add_class::<FrcReadCartesianPosition>()?;
        parent_module.add_class::<FrcReadCartesianPositionResponse>()?;
        parent_module.add_class::<FrcPause>()?;
        parent_module.add_class::<FrcPauseResponse>()?;
        parent_module.add_class::<FrcInitialize>()?;
        parent_module.add_class::<FrcInitializeResponse>()?;
        parent_module.add_class::<FrcGetUFrameUTool>()?;
        parent_module.add_class::<FrcGetUFrameUToolResponse>()?;
        parent_module.add_class::<FrcGetStatus>()?;
        parent_module.add_class::<FrcGetStatusResponse>()?;
        parent_module.add_class::<FrcContinue>()?;
        parent_module.add_class::<FrcContinueResponse>()?;
        parent_module.add_class::<FrcAbort>()?;
        parent_module.add_class::<FrcAbortResponse>()?;
        parent_module.add_class::<Command>()?;
        parent_module.add_class::<CommandResponse>()?;

        Ok(())
    }
}
