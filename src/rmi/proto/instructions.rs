use std::num::NonZeroU32;
#[cfg(not(feature = "py"))]
use std::time::Duration;

use crate::{impl_monostate_member, packet_response_wrap, packet_wrap, rmi::clamp_value};

use super::member_structs::*;
use cfg_mixin::cfg_mixin;
use serde::{Deserialize, Serialize};

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcWaitTime {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Time")]
    pub time: f32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcWaitTime {
    #[on(new)]
    #[on(pyo3(signature = (time)))]
    #[cfg(on)]
    pub fn new(time: f32) -> Self {
        Self {
            sequence_id: None,
            time,
        }
    }

    #[cfg(off)]
    pub fn new(time: Duration) -> Self {
        Self {
            sequence_id: None,
            time: time.as_secs_f32(),
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWaitTimeResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWaitDIN {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "PortNumber")]
    pub port_number: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "PortValue")]
    pub port_value: OnOff,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcWaitDIN {
    #[on(new)]
    #[on(pyo3(signature = (port_number, port_value)))]
    pub fn new(port_number: u32, port_value: OnOff) -> Self {
        Self {
            sequence_id: None,
            port_number,
            port_value,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcWaitDINResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetUTool {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ToolNumber")]
    pub tool_number: u8,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcSetUTool {
    #[on(new)]
    #[on(pyo3(signature = (tool_number)))]
    pub fn new(tool_number: u8) -> Self {
        Self {
            sequence_id: None,
            tool_number,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetUToolResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetUFrame {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "FrameNumber")]
    pub frame_number: u8,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcSetUFrame {
    #[on(new)]
    #[on(pyo3(signature = (frame_number)))]
    pub fn new(frame_number: u8) -> Self {
        Self {
            sequence_id: None,
            frame_number,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetUFrameResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetPayLoad {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ScheduleNumber")]
    pub schedule_number: u8,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcSetPayLoad {
    #[on(new)]
    #[on(pyo3(signature = (schedule_number)))]
    pub fn new(schedule_number: u8) -> Self {
        Self {
            sequence_id: None,
            schedule_number,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcSetPayLoadResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcLinearRelativeJRep {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "JointAngle")]
    pub joint_angles: JointAngles,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIM", default, skip_serializing_if = "Option::is_none")]
    pub alim: Option<i32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIMREG", default, skip_serializing_if = "Option::is_none")]
    pub alim_reg: Option<u16>,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(
        rename = "WristJoint",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    wrist_joint: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOnly>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcLinearRelativeJRep {
    #[on(new)]
    #[on(pyo3(signature = (joint_angles, speed_type, speed, term_type, term_value)))]
    pub fn new(
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: u16,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            joint_angles,
            speed_type,
            speed,
            term_type,
            term_value: clamp_value(term_value, 1, 100),
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            alim: None,
            alim_reg: None,
            no_blend: None,
            wrist_joint: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcLinearRelativeJRep.no_blend = "ON");
impl_monostate_member!(FrcLinearRelativeJRep.wrist_joint = "ON");
impl_monostate_member!(FrcLinearRelativeJRep.mrot = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcLinearRelativeJRepResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcLinearRelative {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIM", default, skip_serializing_if = "Option::is_none")]
    pub alim: Option<i32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIMREG", default, skip_serializing_if = "Option::is_none")]
    pub alim_reg: Option<u16>,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(
        rename = "WristJoint",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    wrist_joint: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOnly>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcLinearRelative {
    #[on(new)]
    #[on(pyo3(signature = (configuration, position, speed_type, speed, term_type, term_value)))]
    pub fn new(
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            configuration,
            position,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            alim: None,
            alim_reg: None,
            no_blend: None,
            wrist_joint: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcLinearRelative.no_blend = "ON");
impl_monostate_member!(FrcLinearRelative.wrist_joint = "ON");
impl_monostate_member!(FrcLinearRelative.mrot = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcLinearRelativeResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcLinearMotionJRep {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "JointAngle")]
    pub joint_angles: JointAngles,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "Coord",
        alias = "COORD",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub coord: Option<OnOff>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIM", default, skip_serializing_if = "Option::is_none")]
    pub alim: Option<i32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIMREG", default, skip_serializing_if = "Option::is_none")]
    pub alim_reg: Option<u16>,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(
        rename = "WristJoint",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    wrist_joint: Option<OnOff>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOff>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcLinearMotionJRep {
    #[on(new)]
    #[on(pyo3(signature = (joint_angles, speed_type, speed, term_type, term_value)))]
    pub fn new(
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            joint_angles,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            coord: None,
            alim: None,
            alim_reg: None,
            no_blend: None,
            wrist_joint: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcLinearMotionJRep.no_blend = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcLinearMotionJRepResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcLinearMotion {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "Coord",
        alias = "COORD",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub coord: Option<OnOff>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIM", default, skip_serializing_if = "Option::is_none")]
    pub alim: Option<i32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ALIMREG", default, skip_serializing_if = "Option::is_none")]
    pub alim_reg: Option<u16>,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(
        rename = "WristJoint",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    wrist_joint: Option<OnOff>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOff>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcLinearMotion {
    #[on(new)]
    #[on(pyo3(signature = (configuration, position, speed_type, speed, term_type, term_value)))]
    pub fn new(
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            configuration,
            position,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            coord: None,
            alim: None,
            alim_reg: None,
            no_blend: None,
            wrist_joint: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcLinearMotion.no_blend = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcLinearMotionResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcJointRelativeJRep {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "JointAngle")]
    pub joint_angles: JointAngles,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOff>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcJointRelativeJRep {
    #[on(new)]
    #[on(pyo3(signature = (joint_angles, speed_type, speed, term_type, term_value)))]
    pub fn new(
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            joint_angles,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            no_blend: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcJointRelativeJRep.no_blend = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcJointRelativeJRepResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcJointRelative {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOff>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcJointRelative {
    #[on(new)]
    #[on(pyo3(signature = (configuration, position, speed_type, speed, term_type, term_value)))]
    pub fn new(
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            configuration,
            position,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            no_blend: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcJointRelative.no_blend = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcJointRelativeResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcJointMotionJRep {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "JointAngle")]
    pub joint_angles: JointAngles,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOff>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcJointMotionJRep {
    #[on(new)]
    #[on(pyo3(signature = (joint_angles, speed_type, speed, term_type, term_value)))]
    pub fn new(
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            joint_angles,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            no_blend: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcJointMotionJRep.no_blend = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcJointMotionJRepResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcJointMotion {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOff>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcJointMotion {
    #[on(new)]
    #[on(pyo3(signature = (configuration, position, speed_type, speed, term_type, term_value)))]
    pub fn new(
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            configuration,
            position,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            no_blend: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcJointMotion.no_blend = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcJointMotionResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcCircularRelative {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "ViaConfiguration")]
    pub via_configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "ViaPosition")]
    pub via_position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(
        rename = "WristJoint",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    wrist_joint: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOnly>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcCircularRelative {
    #[on(new)]
    #[on(pyo3(signature = (configuration, position, via_configuration, via_position, speed_type, speed, term_type, term_value)))]
    pub fn new(
        configuration: Configuration,
        position: Position,
        via_configuration: Configuration,
        via_position: Position,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            configuration,
            position,
            via_configuration,
            via_position,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            no_blend: None,
            wrist_joint: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcCircularRelative.no_blend = "ON");
impl_monostate_member!(FrcCircularRelative.wrist_joint = "ON");
impl_monostate_member!(FrcCircularRelative.mrot = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcCircularRelativeResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct FrcCircularMotion {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "Configuration")]
    pub configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "Position")]
    pub position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "ViaConfiguration")]
    pub via_configuration: Configuration,
    #[on(pyo3(get, set))]
    #[serde(rename = "ViaPosition")]
    pub via_position: Position,
    #[on(pyo3(get, set))]
    #[serde(rename = "SpeedType")]
    pub speed_type: SpeedType,
    #[on(pyo3(get, set))]
    #[serde(rename = "Speed")]
    pub speed: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermType")]
    pub term_type: TermType,
    #[on(pyo3(get, set))]
    #[serde(rename = "TermValue")]
    pub term_value: u8,
    #[on(pyo3(get, set))]
    #[serde(rename = "ACC", default, skip_serializing_if = "Option::is_none")]
    pub accel_override: Option<u8>,
    #[on(pyo3(get, set))]
    #[serde(flatten, default, skip_serializing_if = "Option::is_none")]
    pub lcb: Option<LocalConditionBlock>,
    #[serde(flatten, default)]
    pub offset_register_numbers: OffsetRegisterNumbers,
    #[serde(rename = "NoBlend", default, skip_serializing_if = "Option::is_none")]
    no_blend: Option<OnOnly>,
    #[serde(
        rename = "WristJoint",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    wrist_joint: Option<OnOnly>,
    #[serde(rename = "MROT", default, skip_serializing_if = "Option::is_none")]
    mrot: Option<OnOnly>,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcCircularMotion {
    #[on(new)]
    #[on(pyo3(signature = (configuration, position, via_configuration, via_position, speed_type, speed, term_type, term_value)))]
    pub fn new(
        configuration: Configuration,
        position: Position,
        via_configuration: Configuration,
        via_position: Position,
        speed_type: SpeedType,
        speed: u32,
        term_type: TermType,
        term_value: u8,
    ) -> Self {
        Self {
            sequence_id: None,
            configuration,
            position,
            via_configuration,
            via_position,
            speed_type,
            speed,
            term_type,
            term_value,
            accel_override: None,
            offset_register_numbers: Default::default(),
            lcb: None,
            no_blend: None,
            wrist_joint: None,
            mrot: None,
        }
    }
}
impl_monostate_member!(FrcCircularMotion.no_blend = "ON");
impl_monostate_member!(FrcCircularMotion.wrist_joint = "ON");
impl_monostate_member!(FrcCircularMotion.mrot = "ON");

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcCircularMotionResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FrcCall {
    #[serde(rename = "SequenceID")]
    sequence_id: Option<NonZeroU32>,
    #[on(pyo3(get, set))]
    #[serde(rename = "ProgramName")]
    pub program_name: String,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl FrcCall {
    #[on(new)]
    #[on(pyo3(signature = (program_name)))]
    pub fn new(program_name: String) -> Self {
        Self {
            sequence_id: None,
            program_name,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrcCallResponse {
    #[on(pyo3(get, set))]
    #[serde(rename = "ErrorID")]
    pub error_id: u32,
    #[on(pyo3(get, set))]
    #[serde(rename = "SequenceID")]
    pub sequence_id: u32,
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "Instruction")]
pub enum Instruction {
    #[serde(rename = "FRC_WaitDIN")]
    FrcWaitDIN(FrcWaitDIN), // Wait for DIN Instruction

    #[serde(rename = "FRC_SetUFrame")]
    FrcSetUFrame(FrcSetUFrame), // Set User Frame Instruction

    #[serde(rename = "FRC_SetUTool")]
    FrcSetUTool(FrcSetUTool), // Set User Tool Instruction

    #[serde(rename = "FRC_WaitTime")]
    FrcWaitTime(FrcWaitTime), // Add Wait Time Instruction

    #[serde(rename = "FRC_SetPayLoad")]
    FrcSetPayLoad(FrcSetPayLoad), // Set Payload Instruction

    #[serde(rename = "FRC_Call")]
    FrcCall(FrcCall), // Call a Program

    #[serde(rename = "FRC_LinearMotion")]
    FrcLinearMotion(FrcLinearMotion), // Add Linear Motion Instruction

    #[serde(rename = "FRC_LinearRelative")]
    FrcLinearRelative(FrcLinearRelative), // Add Linear Incremental Motion Instruction

    #[serde(rename = "FRC_LinearRelativeJRep")]
    FrcLinearRelativeJRep(FrcLinearRelativeJRep), // Add Linear Relative Motion with Joint Representation

    #[serde(rename = "FRC_JointMotion")]
    FrcJointMotion(FrcJointMotion), // Add Joint Motion Instruction

    #[serde(rename = "FRC_JointRelative")]
    FrcJointRelative(FrcJointRelative), // Add Joint Incremental Motion Instruction

    #[serde(rename = "FRC_CircularMotion")]
    FrcCircularMotion(FrcCircularMotion), // Add Circular Motion Instruction

    #[serde(rename = "FRC_CircularRelative")]
    FrcCircularRelative(FrcCircularRelative), // Add Circular Incremental Motion Instruction

    #[serde(rename = "FRC_JointMotionJRep")]
    FrcJointMotionJRep(FrcJointMotionJRep), // Add Joint Motion with Joint Representation

    #[serde(rename = "FRC_JointRelativeJRep")]
    FrcJointRelativeJRep(FrcJointRelativeJRep), // Add Joint Incremental Motion with Joint Representation

    #[serde(rename = "FRC_LinearMotionJRep")]
    FrcLinearMotionJRep(FrcLinearMotionJRep), // Add Linear Motion with Joint Representation
}

impl Instruction {
    pub(crate) fn set_seq_id(&mut self, value: u32) {
        let value = NonZeroU32::new(value).unwrap_or(NonZeroU32::MIN);
        match self {
            Instruction::FrcWaitDIN(instr) => instr.sequence_id = Some(value),
            Instruction::FrcSetUFrame(instr) => instr.sequence_id = Some(value),
            Instruction::FrcSetUTool(instr) => instr.sequence_id = Some(value),
            Instruction::FrcWaitTime(instr) => instr.sequence_id = Some(value),
            Instruction::FrcSetPayLoad(instr) => instr.sequence_id = Some(value),
            Instruction::FrcCall(instr) => instr.sequence_id = Some(value),
            Instruction::FrcLinearMotion(instr) => instr.sequence_id = Some(value),
            Instruction::FrcLinearRelative(instr) => instr.sequence_id = Some(value),
            Instruction::FrcLinearRelativeJRep(instr) => instr.sequence_id = Some(value),
            Instruction::FrcJointMotion(instr) => instr.sequence_id = Some(value),
            Instruction::FrcJointRelative(instr) => instr.sequence_id = Some(value),
            Instruction::FrcCircularMotion(instr) => instr.sequence_id = Some(value),
            Instruction::FrcCircularRelative(instr) => instr.sequence_id = Some(value),
            Instruction::FrcJointMotionJRep(instr) => instr.sequence_id = Some(value),
            Instruction::FrcJointRelativeJRep(instr) => instr.sequence_id = Some(value),
            Instruction::FrcLinearMotionJRep(instr) => instr.sequence_id = Some(value),
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl Instruction {
    #[on(getter)]
    pub fn packet_name(&self) -> &'static str {
        match self {
            Instruction::FrcWaitDIN(_) => "FRC_WaitDIN",
            Instruction::FrcSetUFrame(_) => "FRC_SetUFrame",
            Instruction::FrcSetUTool(_) => "FRC_SetUTool",
            Instruction::FrcWaitTime(_) => "FRC_WaitTime",
            Instruction::FrcSetPayLoad(_) => "FRC_SetPayLoad",
            Instruction::FrcCall(_) => "FRC_Call",
            Instruction::FrcLinearMotion(_) => "FRC_LinearMotion",
            Instruction::FrcLinearRelative(_) => "FRC_LinearRelative",
            Instruction::FrcLinearRelativeJRep(_) => "FRC_LinearRelativeJRep",
            Instruction::FrcJointMotion(_) => "FRC_JointMotion",
            Instruction::FrcJointRelative(_) => "FRC_JointRelative",
            Instruction::FrcCircularMotion(_) => "FRC_CircularMotion",
            Instruction::FrcCircularRelative(_) => "FRC_CircularRelative",
            Instruction::FrcJointMotionJRep(_) => "FRC_JointMotionJRep",
            Instruction::FrcJointRelativeJRep(_) => "FRC_JointRelativeJRep",
            Instruction::FrcLinearMotionJRep(_) => "FRC_LinearMotionJRep",
        }
    }
}

packet_wrap! {
    Instruction {
        FrcWaitDIN,
        FrcSetUFrame,
        FrcSetUTool,
        FrcWaitTime,
        FrcSetPayLoad,
        FrcCall,
        FrcLinearMotion,
        FrcLinearRelative,
        FrcLinearRelativeJRep,
        FrcJointMotion,
        FrcJointRelative,
        FrcCircularMotion,
        FrcCircularRelative,
        FrcJointMotionJRep,
        FrcJointRelativeJRep,
        FrcLinearMotionJRep
    }
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(tag = "Instruction")]
pub enum InstructionResponse {
    #[serde(rename = "FRC_WaitDIN")]
    FrcWaitDIN(FrcWaitDINResponse),
    #[serde(rename = "FRC_SetUFrame")]
    FrcSetUFrame(FrcSetUFrameResponse),
    #[serde(rename = "FRC_SetUTool")]
    FrcSetUTool(FrcSetUToolResponse),
    #[serde(rename = "FRC_WaitTime")]
    FrcWaitTime(FrcWaitTimeResponse),
    #[serde(rename = "FRC_SetPayLoad")]
    FrcSetPayLoad(FrcSetPayLoadResponse),
    #[serde(rename = "FRC_Call")]
    FrcCall(FrcCallResponse),
    #[serde(rename = "FRC_LinearMotion")]
    FrcLinearMotion(FrcLinearMotionResponse),
    #[serde(rename = "FRC_LinearRelative")]
    FrcLinearRelative(FrcLinearRelativeResponse),
    #[serde(rename = "FRC_LinearRelativeJRep")]
    FrcLinearRelativeJRep(FrcLinearRelativeJRepResponse),
    #[serde(rename = "FRC_JointMotion")]
    FrcJointMotion(FrcJointMotionResponse),
    #[serde(rename = "FRC_JointRelative")]
    FrcJointRelative(FrcJointRelativeResponse),
    #[serde(rename = "FRC_CircularMotion")]
    FrcCircularMotion(FrcCircularMotionResponse),
    #[serde(rename = "FRC_CircularRelative")]
    FrcCircularRelative(FrcCircularRelativeResponse),
    #[serde(rename = "FRC_JointMotionJRep")]
    FrcJointMotionJRep(FrcJointMotionJRepResponse),
    #[serde(rename = "FRC_JointRelativeJRep")]
    FrcJointRelativeJRep(FrcJointRelativeJRepResponse),
    #[serde(rename = "FRC_LinearMotionJRep")]
    FrcLinearMotionJRep(FrcLinearMotionJRepResponse),
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl InstructionResponse {
    #[on(getter)]
    pub fn sequence_id(&self) -> u32 {
        match self {
            InstructionResponse::FrcWaitDIN(resp) => resp.sequence_id,
            InstructionResponse::FrcSetUFrame(resp) => resp.sequence_id,
            InstructionResponse::FrcSetUTool(resp) => resp.sequence_id,
            InstructionResponse::FrcWaitTime(resp) => resp.sequence_id,
            InstructionResponse::FrcSetPayLoad(resp) => resp.sequence_id,
            InstructionResponse::FrcCall(resp) => resp.sequence_id,
            InstructionResponse::FrcLinearMotion(resp) => resp.sequence_id,
            InstructionResponse::FrcLinearRelative(resp) => resp.sequence_id,
            InstructionResponse::FrcLinearRelativeJRep(resp) => resp.sequence_id,
            InstructionResponse::FrcJointMotion(resp) => resp.sequence_id,
            InstructionResponse::FrcJointRelative(resp) => resp.sequence_id,
            InstructionResponse::FrcCircularMotion(resp) => resp.sequence_id,
            InstructionResponse::FrcCircularRelative(resp) => resp.sequence_id,
            InstructionResponse::FrcJointMotionJRep(resp) => resp.sequence_id,
            InstructionResponse::FrcJointRelativeJRep(resp) => resp.sequence_id,
            InstructionResponse::FrcLinearMotionJRep(resp) => resp.sequence_id,
        }
    }

    #[on(getter)]
    pub fn error_id(&self) -> u32 {
        match self {
            InstructionResponse::FrcWaitDIN(resp) => resp.error_id,
            InstructionResponse::FrcSetUFrame(resp) => resp.error_id,
            InstructionResponse::FrcSetUTool(resp) => resp.error_id,
            InstructionResponse::FrcWaitTime(resp) => resp.error_id,
            InstructionResponse::FrcSetPayLoad(resp) => resp.error_id,
            InstructionResponse::FrcCall(resp) => resp.error_id,
            InstructionResponse::FrcLinearMotion(resp) => resp.error_id,
            InstructionResponse::FrcLinearRelative(resp) => resp.error_id,
            InstructionResponse::FrcLinearRelativeJRep(resp) => resp.error_id,
            InstructionResponse::FrcJointMotion(resp) => resp.error_id,
            InstructionResponse::FrcJointRelative(resp) => resp.error_id,
            InstructionResponse::FrcCircularMotion(resp) => resp.error_id,
            InstructionResponse::FrcCircularRelative(resp) => resp.error_id,
            InstructionResponse::FrcJointMotionJRep(resp) => resp.error_id,
            InstructionResponse::FrcJointRelativeJRep(resp) => resp.error_id,
            InstructionResponse::FrcLinearMotionJRep(resp) => resp.error_id,
        }
    }

    #[on(getter)]
    pub fn packet_name(&self) -> &'static str {
        match self {
            InstructionResponse::FrcWaitDIN(_) => "FRC_WaitDIN",
            InstructionResponse::FrcSetUFrame(_) => "FRC_SetUFrame",
            InstructionResponse::FrcSetUTool(_) => "FRC_SetUTool",
            InstructionResponse::FrcWaitTime(_) => "FRC_WaitTime",
            InstructionResponse::FrcSetPayLoad(_) => "FRC_SetPayLoad",
            InstructionResponse::FrcCall(_) => "FRC_Call",
            InstructionResponse::FrcLinearMotion(_) => "FRC_LinearMotion",
            InstructionResponse::FrcLinearRelative(_) => "FRC_LinearRelative",
            InstructionResponse::FrcLinearRelativeJRep(_) => "FRC_LinearRelativeJRep",
            InstructionResponse::FrcJointMotion(_) => "FRC_JointMotion",
            InstructionResponse::FrcJointRelative(_) => "FRC_JointRelative",
            InstructionResponse::FrcCircularMotion(_) => "FRC_CircularMotion",
            InstructionResponse::FrcCircularRelative(_) => "FRC_CircularRelative",
            InstructionResponse::FrcJointMotionJRep(_) => "FRC_JointMotionJRep",
            InstructionResponse::FrcJointRelativeJRep(_) => "FRC_JointRelativeJRep",
            InstructionResponse::FrcLinearMotionJRep(_) => "FRC_LinearMotionJRep",
        }
    }
}

packet_response_wrap! {
    InstructionResponse {
        FrcWaitDIN,
        FrcSetUFrame,
        FrcSetUTool,
        FrcWaitTime,
        FrcSetPayLoad,
        FrcCall,
        FrcLinearMotion,
        FrcLinearRelative,
        FrcLinearRelativeJRep,
        FrcJointMotion,
        FrcJointRelative,
        FrcCircularMotion,
        FrcCircularRelative,
        FrcJointMotionJRep,
        FrcJointRelativeJRep,
        FrcLinearMotionJRep
    }
}

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<FrcWaitTime>()?;
        parent_module.add_class::<FrcWaitTimeResponse>()?;
        parent_module.add_class::<FrcWaitDIN>()?;
        parent_module.add_class::<FrcWaitDINResponse>()?;
        parent_module.add_class::<FrcSetUTool>()?;
        parent_module.add_class::<FrcSetUToolResponse>()?;
        parent_module.add_class::<FrcSetUFrame>()?;
        parent_module.add_class::<FrcSetUFrameResponse>()?;
        parent_module.add_class::<FrcSetPayLoad>()?;
        parent_module.add_class::<FrcSetPayLoadResponse>()?;
        parent_module.add_class::<FrcLinearRelativeJRep>()?;
        parent_module.add_class::<FrcLinearRelativeJRepResponse>()?;
        parent_module.add_class::<FrcLinearRelative>()?;
        parent_module.add_class::<FrcLinearRelativeResponse>()?;
        parent_module.add_class::<FrcLinearMotionJRep>()?;
        parent_module.add_class::<FrcLinearMotionJRepResponse>()?;
        parent_module.add_class::<FrcLinearMotion>()?;
        parent_module.add_class::<FrcLinearMotionResponse>()?;
        parent_module.add_class::<FrcJointRelativeJRep>()?;
        parent_module.add_class::<FrcJointRelativeJRepResponse>()?;
        parent_module.add_class::<FrcJointRelative>()?;
        parent_module.add_class::<FrcJointRelativeResponse>()?;
        parent_module.add_class::<FrcJointMotionJRep>()?;
        parent_module.add_class::<FrcJointMotionJRepResponse>()?;
        parent_module.add_class::<FrcJointMotion>()?;
        parent_module.add_class::<FrcJointMotionResponse>()?;
        parent_module.add_class::<FrcCircularRelative>()?;
        parent_module.add_class::<FrcCircularRelativeResponse>()?;
        parent_module.add_class::<FrcCircularMotion>()?;
        parent_module.add_class::<FrcCircularMotionResponse>()?;
        parent_module.add_class::<FrcCall>()?;
        parent_module.add_class::<FrcCallResponse>()?;
        parent_module.add_class::<Instruction>()?;
        parent_module.add_class::<InstructionResponse>()?;

        Ok(())
    }
}
