#![allow(non_camel_case_types)]

use std::fmt::Display;

use bincode::{Decode, Encode};
use cfg_mixin::cfg_mixin;
use serde::Serialize;

use crate::{
    joints::{JointFormat, JointRepr, JointTemplate},
    stmo::StreamMotionError,
};

pub trait Packet {
    const PACKET_TYPE: u32;
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct PoseData {
    #[on(pyo3(get, set))]
    pub x: f32,
    #[on(pyo3(get, set))]
    pub y: f32,
    #[on(pyo3(get, set))]
    pub z: f32,
    #[on(pyo3(get, set))]
    pub w: f32,
    #[on(pyo3(get, set))]
    pub p: f32,
    #[on(pyo3(get, set))]
    pub r: f32,
    #[on(pyo3(get, set))]
    pub e1: f32,
    #[on(pyo3(get, set))]
    pub e2: f32,
    #[on(pyo3(get, set))]
    pub e3: f32,
}

impl Display for PoseData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PoseData {{ x: {}, y: {}, z: {}, w: {}, p: {}, r: {}, e1: {}, e2: {}, e3: {} }}",
            self.x, self.y, self.z, self.w, self.p, self.r, self.e1, self.e2, self.e3
        )
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl PoseData {
    #[on(new)]
    #[on(pyo3(signature = (x, y, z, w, p, r, e1=0.0, e2=0.0, e3=0.0)))]
    #[allow(clippy::too_many_arguments)]
    pub fn new(x: f32, y: f32, z: f32, w: f32, p: f32, r: f32, e1: f32, e2: f32, e3: f32) -> Self {
        Self {
            x,
            y,
            z,
            w,
            p,
            r,
            e1,
            e2,
            e3,
        }
    }
    pub fn to_array(&self) -> [f32; 9] {
        [
            self.x, self.y, self.z, self.w, self.p, self.r, self.e1, self.e2, self.e3,
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
struct MotionCommandPacketSingle {
    seq: u32,
    last_command: bool,
    read_io_type: IoType,
    read_io_index: u16,
    read_io_mask: u16,
    joint_format: bool,
    write_io_type: IoType,
    write_io_index: u16,
    write_io_mask: u16,
    write_io_value: u16,
    #[serde(skip_serializing)]
    unused: u16,
    position: [f32; 9],
}

impl std::fmt::Display for MotionCommandPacketSingle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "MotionCommandPacketSingle {{ seq: {}, last_command: {}, read_io_type: {}, read_io_index: {}, read_io_mask: {}, joint_format: {}, write_io_type: {:?}, write_io_index: {}, write_io_mask: {}, write_io_value: {}, joints: {:?} }}",
            self.seq,
            self.last_command,
            self.read_io_type,
            self.read_io_index,
            self.read_io_mask,
            self.joint_format,
            self.write_io_type,
            self.write_io_index,
            self.write_io_mask,
            self.write_io_value,
            self.position
        )
    }
}

impl From<MotionCommandPacket> for MotionCommandPacketSingle {
    fn from(pkt: MotionCommandPacket) -> Self {
        Self {
            seq: pkt.seq,
            last_command: pkt.last_command,
            read_io_type: pkt.read_io_type,
            read_io_index: pkt.read_io_index,
            read_io_mask: pkt.read_io_mask,
            joint_format: pkt.joint_format,
            write_io_type: pkt.write_io_type,
            write_io_index: pkt.write_io_index,
            write_io_mask: pkt.write_io_mask,
            write_io_value: pkt.write_io_value,
            unused: 0xFFFF,
            position: pkt.position.map(|j| j as f32),
        }
    }
}

impl Packet for MotionCommandPacketSingle {
    const PACKET_TYPE: u32 = 1;
}

#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct MotionCommandPacket {
    pub(crate) seq: u32,
    pub(crate) last_command: bool,
    read_io_type: IoType,
    read_io_index: u16,
    read_io_mask: u16,
    joint_format: bool,
    write_io_type: IoType,
    write_io_index: u16,
    write_io_mask: u16,
    write_io_value: u16,
    #[serde(skip_serializing)]
    unused: [u16; 3],
    position: [f64; 9],
}

impl MotionCommandPacket {
    pub fn try_from_joints(
        format: JointFormat,
        template: JointTemplate,
        joints: impl JointRepr,
    ) -> Result<Self, StreamMotionError> {
        let axis_cnt = template.axis.len();
        let joints = JointFormat::FanucDeg.convert_from(format, &template, joints);
        let mut full_joints = [0.0; 9];
        match axis_cnt {
            4 => full_joints[..axis_cnt].copy_from_slice(&joints.to_array::<4, f64>(false)?),
            5 => full_joints[..axis_cnt].copy_from_slice(&joints.to_array::<5, f64>(false)?),
            6 => full_joints[..axis_cnt].copy_from_slice(&joints.to_array::<6, f64>(false)?),
            7 => full_joints[..axis_cnt].copy_from_slice(&joints.to_array::<7, f64>(false)?),
            8 => full_joints[..axis_cnt].copy_from_slice(&joints.to_array::<8, f64>(false)?),
            9 => full_joints[..axis_cnt].copy_from_slice(&joints.to_array::<9, f64>(false)?),
            _ => {
                return Err(StreamMotionError::InvalidJointCount(axis_cnt as u8));
            }
        };

        Ok(Self {
            seq: 0,
            last_command: false,
            read_io_type: IoType::None,
            read_io_index: 0,
            read_io_mask: 0,
            joint_format: true,
            write_io_type: IoType::None,
            write_io_index: 0,
            write_io_mask: 0,
            write_io_value: 0,
            unused: [0xFFFF, 0x3333, 0x3333],
            position: full_joints,
        })
    }

    pub fn from_pose(pose: PoseData) -> Result<Self, StreamMotionError> {
        Ok(Self {
            seq: 0,
            last_command: false,
            read_io_type: IoType::None,
            read_io_index: 0,
            read_io_mask: 0,
            joint_format: false,
            write_io_type: IoType::None,
            write_io_index: 0,
            write_io_mask: 0,
            write_io_value: 0,
            unused: [0xFFFF, 0x3333, 0x3333],
            position: [
                pose.x as f64,
                pose.y as f64,
                pose.z as f64,
                pose.w as f64,
                pose.p as f64,
                pose.r as f64,
                pose.e1 as f64,
                pose.e2 as f64,
                pose.e3 as f64,
            ],
        })
    }

    pub(crate) fn filler(
        status: &RobotStatusPacket,
        previous_command: &MotionCommandPacket,
        is_last_command: bool,
    ) -> Self {
        Self {
            seq: status.seq,
            last_command: is_last_command,
            read_io_type: IoType::None,
            read_io_index: 0,
            read_io_mask: 0,
            joint_format: true,
            write_io_type: IoType::None,
            write_io_index: 0,
            write_io_mask: 0,
            write_io_value: 0,
            unused: [0xFFFF, 0x3333, 0x3333],
            position: previous_command.position,
        }
    }

    pub(crate) fn should_cast_to_single(&self, version: u32) -> bool {
        version < 2 || !self.joint_format
    }
}

#[cfg_attr(feature = "py", pyo3::pymethods)]
impl MotionCommandPacket {
    #[cfg(feature = "py")]
    #[staticmethod]
    #[pyo3(name = "try_from_joints")]
    #[pyo3(signature = (format, template, joints))]
    pub fn py_try_from_joints(
        format: JointFormat,
        template: JointTemplate,
        joints: pyo3::Bound<pyo3::types::PySequence>,
    ) -> pyo3::PyResult<Self> {
        use pyo3::types::PyAnyMethods;

        let joints: Vec<f64> = joints
            .try_iter()?
            .map(|item| item.and_then(|obj| obj.extract::<f64>()))
            .collect::<pyo3::PyResult<Vec<f64>>>()?;
        Self::try_from_joints(format, template, joints).map_err(Into::into)
    }

    #[cfg(feature = "py")]
    #[staticmethod]
    #[pyo3(name = "from_pose")]
    pub fn py_from_pose(pose: PoseData) -> pyo3::PyResult<Self> {
        Self::from_pose(pose).map_err(Into::into)
    }

    pub fn set_read_io(&mut self, io_type: IoType, index: u16, mask: u16) {
        self.read_io_type = io_type;
        self.read_io_index = index;
        self.read_io_mask = mask;
    }

    pub fn set_write_io(&mut self, io_type: IoType, index: u16, mask: u16, value: u16) {
        self.write_io_type = io_type;
        self.write_io_index = index;
        self.write_io_mask = mask;
        self.write_io_value = value;
    }

    pub fn set_last_command(&mut self, last: bool) {
        self.last_command = last;
    }
}

impl Display for MotionCommandPacket {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "MotionCommandPacketDouble {{ seq: {}, last_command: {}, read_io_type: {}, read_io_index: {}, read_io_mask: {}, joint_format: {}, write_io_type: {:?}, write_io_index: {}, write_io_mask: {}, write_io_value: {}, joints: {:?} }}",
            self.seq,
            self.last_command,
            self.read_io_type,
            self.read_io_index,
            self.read_io_mask,
            self.joint_format,
            self.write_io_type,
            self.write_io_index,
            self.write_io_mask,
            self.write_io_value,
            self.position
        )
    }
}

impl From<MotionCommandPacketSingle> for MotionCommandPacket {
    fn from(pkt: MotionCommandPacketSingle) -> Self {
        Self {
            seq: pkt.seq,
            last_command: pkt.last_command,
            read_io_type: pkt.read_io_type,
            read_io_index: pkt.read_io_index,
            read_io_mask: pkt.read_io_mask,
            joint_format: pkt.joint_format,
            write_io_type: pkt.write_io_type,
            write_io_index: pkt.write_io_index,
            write_io_mask: pkt.write_io_mask,
            write_io_value: pkt.write_io_value,
            unused: [0xFFFF, 0x3333, 0x3333],
            position: pkt.position.map(|j| j as f64),
        }
    }
}

impl Packet for MotionCommandPacket {
    const PACKET_TYPE: u32 = 5;
}

#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
#[repr(u8)]
pub enum IoType {
    None = 0,
    DI = 1,
    DO = 2,
    RI = 8,
    RO = 9,
    SI = 11,
    SO = 12,
    WI = 16,
    WO = 17,
    UI = 20,
    UO = 21,
    WSI = 26,
    WSO = 27,
    F = 35,
    M = 36,
}

impl bincode::Encode for IoType {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        (*self as u8).encode(encoder)
    }
}

impl<Ctx> bincode::Decode<Ctx> for IoType {
    fn decode<D: bincode::de::Decoder<Context = Ctx>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let variant = <u8 as bincode::Decode<D::Context>>::decode(decoder)?;
        match variant {
            0u8 => Ok(Self::None),
            1u8 => Ok(Self::DI),
            2u8 => Ok(Self::DO),
            8u8 => Ok(Self::RI),
            9u8 => Ok(Self::RO),
            11u8 => Ok(Self::SI),
            12u8 => Ok(Self::SO),
            16u8 => Ok(Self::WI),
            17u8 => Ok(Self::WO),
            20u8 => Ok(Self::UI),
            21u8 => Ok(Self::UO),
            26u8 => Ok(Self::WSI),
            27u8 => Ok(Self::WSO),
            35u8 => Ok(Self::F),
            36u8 => Ok(Self::M),
            unknown => Err(bincode::error::DecodeError::UnexpectedVariant {
                found: unknown as u32,
                type_name: "IoType",
                allowed: &bincode::error::AllowedEnumVariants::Allowed(&[
                    0, 1, 2, 8, 9, 11, 12, 16, 17, 20, 21, 26, 27, 35, 36,
                ]),
            }),
        }
    }
}

impl<Ctx> bincode::de::BorrowDecode<'_, Ctx> for IoType {
    fn borrow_decode<D: bincode::de::Decoder<Context = Ctx>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        <Self as bincode::Decode<Ctx>>::decode(decoder)
    }
}

impl Display for IoType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            IoType::None => "None",
            IoType::DI => "DI",
            IoType::DO => "DO",
            IoType::RI => "RI",
            IoType::RO => "RO",
            IoType::SI => "SI",
            IoType::SO => "SO",
            IoType::WI => "WI",
            IoType::WO => "WO",
            IoType::UI => "UI",
            IoType::UO => "UO",
            IoType::WSI => "WSI",
            IoType::WSO => "WSO",
            IoType::F => "F",
            IoType::M => "M",
        };
        write!(f, "{}", s)
    }
}

#[cfg(not(feature = "py"))]
bitflags::bitflags! {
    #[repr(transparent)]
    pub struct StatusBitfield: u8 {
        const READY_FOR_COMMANDS = 0b0000_0001;
        const COMMAND_RECEIVED   = 0b0000_0010;
        const SYSRDY             = 0b0000_0100;
        const IN_MOTION          = 0b0000_1000;
        const PACKET_RATE_MASK   = 0b1111_0000;
    }
}

#[cfg(feature = "py")]
#[pyo3::pyclass(str, from_py_object)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StatusBitfield {
    ready_for_commands: bool,
    command_received: bool,
    sysrdy: bool,
    in_motion: bool,
    packet_rate: u8,
}

#[cfg_attr(feature = "py", pyo3::pymethods)]
impl StatusBitfield {
    #[cfg(feature = "py")]
    pub fn in_motion(&self) -> bool {
        self.in_motion
    }
    #[cfg(not(feature = "py"))]
    pub fn in_motion(&self) -> bool {
        self.contains(StatusBitfield::IN_MOTION)
    }
    #[cfg(feature = "py")]
    pub fn ready_for_commands(&self) -> bool {
        self.ready_for_commands
    }
    #[cfg(not(feature = "py"))]
    pub fn ready_for_commands(&self) -> bool {
        self.contains(StatusBitfield::READY_FOR_COMMANDS)
    }
    #[cfg(feature = "py")]
    pub fn command_received(&self) -> bool {
        self.command_received
    }
    #[cfg(not(feature = "py"))]
    pub fn command_received(&self) -> bool {
        self.contains(StatusBitfield::COMMAND_RECEIVED)
    }
    #[cfg(feature = "py")]
    pub fn sysrdy(&self) -> bool {
        self.sysrdy
    }
    #[cfg(not(feature = "py"))]
    pub fn sysrdy(&self) -> bool {
        self.contains(StatusBitfield::SYSRDY)
    }
    #[cfg(feature = "py")]
    pub fn packet_rate(&self) -> u8 {
        self.packet_rate
    }
    #[cfg(not(feature = "py"))]
    pub fn packet_rate(&self) -> u8 {
        (self.bits() & StatusBitfield::PACKET_RATE_MASK.bits()) >> 4
    }
}

impl std::fmt::Display for StatusBitfield {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opening = if cfg!(feature = "py") { "(" } else { "{" };
        let closing = if cfg!(feature = "py") { ")" } else { "}" };
        write!(
            f,
            "StatusBitfield{}ready_for_commands: {}, command_received: {}, sysrdy: {}, in_motion: {}, packet_rate: {}{}",
            opening,
            self.ready_for_commands(),
            self.command_received(),
            self.sysrdy(),
            self.in_motion(),
            self.packet_rate(),
            closing
        )
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct RobotStatusPacket {
    #[on(pyo3(get))]
    pub seq: u32,
    #[on(pyo3(get))]
    pub status: u8,
    #[on(pyo3(get))]
    pub read_io_type: u8,
    #[on(pyo3(get))]
    pub read_io_index: u16,
    #[on(pyo3(get))]
    pub read_io_mask: u16,
    #[on(pyo3(get))]
    pub read_io_value: u16,
    #[on(pyo3(get))]
    pub time_stamp: u32,
    #[on(pyo3(get))]
    pub pose: PoseData,
    joints: [f32; 9],
    #[on(pyo3(get))]
    pub motor_current: [f32; 9],
}

#[cfg_attr(feature = "py", pyo3::pymethods)]
impl RobotStatusPacket {
    #[cfg(feature = "py")]
    pub fn status_bits(&self) -> StatusBitfield {
        StatusBitfield {
            ready_for_commands: (self.status & 0b0000_0001) != 0,
            command_received: (self.status & 0b0000_0010) != 0,
            sysrdy: (self.status & 0b0000_0100) != 0,
            in_motion: (self.status & 0b0000_1000) != 0,
            packet_rate: (self.status & 0b1111_0000) >> 4,
        }
    }

    #[cfg(not(feature = "py"))]
    pub fn status_bits(&self) -> StatusBitfield {
        StatusBitfield::from_bits_truncate(self.status)
    }
}

#[cfg_attr(feature = "py", pyo3::pymethods)]
impl RobotStatusPacket {
    pub fn joints(&self, format: JointFormat, template: JointTemplate) -> [f32; 9] {
        format.convert_from(JointFormat::FanucDeg, &template, self.joints)
    }
}

impl std::fmt::Display for RobotStatusPacket {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let opening = if cfg!(feature = "py") { "(" } else { "{" };
        let closing = if cfg!(feature = "py") { ")" } else { "}" };
        write!(
            f,
            "RobotStatePacket {} seq: {}, status: {}, read_io_type: {}, read_io_index: {}, read_io_mask: {}, read_io_value: {}, time_stamp: {}, pose: {}, joints: {:?}, motor_current: {:?} {}",
            opening,
            self.seq,
            self.status_bits(),
            self.read_io_type,
            self.read_io_index,
            self.read_io_mask,
            self.read_io_value,
            self.time_stamp,
            self.pose,
            self.joints.map(|j| j),
            self.motor_current.map(|j| j),
            closing
        )
    }
}

impl Packet for RobotStatusPacket {
    const PACKET_TYPE: u32 = 0;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Encode, Decode, Serialize)]
pub struct StopPacket {}

impl Packet for StopPacket {
    const PACKET_TYPE: u32 = 2;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Encode, Decode, Serialize)]
pub struct StartPacket {}

impl Packet for StartPacket {
    const PACKET_TYPE: u32 = 0;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Encode, Decode, Serialize)]
pub struct CommandPositionRequestPacket {}

impl Packet for CommandPositionRequestPacket {
    const PACKET_TYPE: u32 = 4;
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct CommandPositionResponsePacket {
    #[on(pyo3(get))]
    pub timestamp: u32,
    #[on(pyo3(get))]
    pub position: PoseData,
    joints: [f32; 9],
}

#[cfg_attr(feature = "py", pyo3::pymethods)]
impl CommandPositionResponsePacket {
    pub fn joints(&self, format: JointFormat, template: JointTemplate) -> [f32; 9] {
        format.convert_from(JointFormat::FanucDeg, &template, self.joints)
    }
}

impl Packet for CommandPositionResponsePacket {
    const PACKET_TYPE: u32 = 4;
}

#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct ThresholdTableRequestPacket {
    pub axis_number: u32,
    pub limit_type: u32,
}

impl Packet for ThresholdTableRequestPacket {
    const PACKET_TYPE: u32 = 3;
}

impl TryFrom<(u32, u32)> for ThresholdTableRequestPacket {
    type Error = String;
    fn try_from(value: (u32, u32)) -> Result<Self, Self::Error> {
        Ok(Self {
            axis_number: value.0,
            limit_type: match value.1 {
                0 => 0,
                1 => 1,
                2 => 2,
                _ => return Err("Invalid threshold type".into()),
            },
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct ThresholdTableResponsePacket {
    pub axis_number: u32,
    pub limit_type: u32,
    pub vmax: u32,
    pub inter_check_time: u32,
    pub no_payload: [f32; 20],
    pub max_payload: [f32; 20],
}

pub const THRESHOLD_TABLE_LENGTH: usize = 20;

impl Packet for ThresholdTableResponsePacket {
    const PACKET_TYPE: u32 = 3;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Encode, Decode, Serialize)]
pub struct VersionNumberRequestPacket {}

impl Packet for VersionNumberRequestPacket {
    const PACKET_TYPE: u32 = 6;
}

#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize)]
#[repr(C)]
pub struct VersionNumberResponsePacket {
    pub version: u32,
}

impl Packet for VersionNumberResponsePacket {
    const PACKET_TYPE: u32 = 6;
}

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(tag = "packet", content = "data")]
pub enum TxPackets {
    MotionCommand(MotionCommandPacket),
    Stop(StopPacket),
    Start(StartPacket),
    CommandPositionRequest(CommandPositionRequestPacket),
    ThresholdTableRequest(ThresholdTableRequestPacket),
    VersionNumberRequest(VersionNumberRequestPacket),
}

impl TxPackets {
    pub fn encode_into(self, version: u32, buffer: &mut [u8]) -> Result<usize, StreamMotionError> {
        let pkt_type = match self {
            TxPackets::MotionCommand(ref pkt) => {
                if pkt.should_cast_to_single(version) {
                    MotionCommandPacketSingle::PACKET_TYPE
                } else {
                    MotionCommandPacket::PACKET_TYPE
                }
            }
            TxPackets::Stop(_) => StopPacket::PACKET_TYPE,
            TxPackets::Start(_) => StartPacket::PACKET_TYPE,
            TxPackets::CommandPositionRequest(_) => CommandPositionRequestPacket::PACKET_TYPE,
            TxPackets::ThresholdTableRequest(_) => ThresholdTableRequestPacket::PACKET_TYPE,
            TxPackets::VersionNumberRequest(_) => VersionNumberRequestPacket::PACKET_TYPE,
        };
        buffer[0..4].copy_from_slice(&pkt_type.to_be_bytes());
        buffer[4..8].copy_from_slice(&version.to_be_bytes());
        let data_buf = &mut buffer[8..];
        let cfg = bincode::config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        let n = match self {
            TxPackets::MotionCommand(ref pkt) => {
                if pkt.should_cast_to_single(version) {
                    let single: MotionCommandPacketSingle = (*pkt).into();
                    bincode::encode_into_slice(single, data_buf, cfg)?
                } else {
                    bincode::encode_into_slice(pkt, data_buf, cfg)?
                }
            }
            TxPackets::Stop(ref pkt) => bincode::encode_into_slice(pkt, data_buf, cfg)?,
            TxPackets::Start(ref pkt) => bincode::encode_into_slice(pkt, data_buf, cfg)?,
            TxPackets::CommandPositionRequest(ref pkt) => {
                bincode::encode_into_slice(pkt, data_buf, cfg)?
            }
            TxPackets::ThresholdTableRequest(ref pkt) => {
                bincode::encode_into_slice(pkt, data_buf, cfg)?
            }
            TxPackets::VersionNumberRequest(ref pkt) => {
                bincode::encode_into_slice(pkt, data_buf, cfg)?
            }
        };
        log::trace!("Encoded packet type {} with {} bytes of data", pkt_type, n);
        Ok(n + 8)
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(tag = "packet", content = "data")]
pub enum RxPackets {
    RobotStatus(RobotStatusPacket),
    CommandPositionResponse(CommandPositionResponsePacket),
    ThresholdTableResponse(ThresholdTableResponsePacket),
    VersionNumberResponse(VersionNumberResponsePacket),
}

impl RxPackets {
    pub fn decode_from(buf: &[u8]) -> Option<Self> {
        if buf.len() < 8 {
            log::warn!("Received packet too short: {} bytes", buf.len());
            return None;
        }
        let packet_type = u32::from_be_bytes(buf[0..4].try_into().ok()?);
        let version = u32::from_be_bytes(buf[4..8].try_into().ok()?);
        if packet_type == VersionNumberResponsePacket::PACKET_TYPE {
            return Some(RxPackets::VersionNumberResponse(
                VersionNumberResponsePacket { version },
            ));
        }
        let data_buf = &buf[8..];
        let cfg = bincode::config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        match packet_type {
            RobotStatusPacket::PACKET_TYPE => {
                let (pkt, n) = bincode::decode_from_slice(data_buf, cfg).ok()?;
                if n != data_buf.len() {
                    log::warn!(
                        "Warning: RobotStatusPacket decoded length {} does not match data length {}",
                        n,
                        data_buf.len()
                    );
                }
                Some(RxPackets::RobotStatus(pkt))
            }
            ThresholdTableResponsePacket::PACKET_TYPE => {
                let (pkt, n) = bincode::decode_from_slice(data_buf, cfg).ok()?;
                if n != data_buf.len() {
                    log::warn!(
                        "Warning: ThresholdTableResponsePacket decoded length {} does not match data length {}",
                        n,
                        data_buf.len()
                    );
                }
                Some(RxPackets::ThresholdTableResponse(pkt))
            }
            CommandPositionResponsePacket::PACKET_TYPE => {
                let (pkt, n) = bincode::decode_from_slice(data_buf, cfg).ok()?;
                if n != data_buf.len() {
                    log::warn!(
                        "Warning: CommandPositionResponsePacket decoded length {} does not match data length {}",
                        n,
                        data_buf.len()
                    );
                }
                Some(RxPackets::CommandPositionResponse(pkt))
            }
            _ => None,
        }
    }
}

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<PoseData>()?;
        parent_module.add_class::<MotionCommandPacket>()?;
        parent_module.add_class::<IoType>()?;
        parent_module.add_class::<StatusBitfield>()?;
        parent_module.add_class::<RobotStatusPacket>()?;
        parent_module.add_class::<CommandPositionResponsePacket>()?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motion_command_packet_size() {
        let bincode_mcp = MotionCommandPacketSingle {
            seq: 0xabcd_ef01,
            last_command: false,
            read_io_type: IoType::None,
            read_io_index: 0xff00,
            read_io_mask: 0xabcd,
            joint_format: true,
            write_io_type: IoType::WI,
            write_io_index: 0b010_1010_1101_0101,
            write_io_mask: 0b010_1010_1101_0101,
            write_io_value: 0b010_1010_1101_0101,
            unused: 0xFFFF,
            position: [
                0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1_000.0, 10_000.0, 100_000.0,
            ],
        };
        let cfg = bincode::config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        let bincode_bytes = bincode::encode_to_vec(bincode_mcp, cfg).unwrap();

        assert_eq!(bincode_bytes.len(), size_of::<MotionCommandPacketSingle>());
    }

    #[test]
    fn test_motion_command_double_packet_size() {
        let bincode_mcpd = MotionCommandPacket {
            seq: 0xabcd_ef01,
            last_command: false,
            read_io_type: IoType::None,
            read_io_index: 0xff00,
            read_io_mask: 0xabcd,
            joint_format: true,
            write_io_type: IoType::WI,
            write_io_index: 0b010_1010_1101_0101,
            write_io_mask: 0b010_1010_1101_0101,
            write_io_value: 0b010_1010_1101_0101,
            unused: [0xFFFF, 0x3333, 0x3333],
            position: [
                0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1_000.0, 10_000.0, 100_000.0,
            ],
        };
        let cfg = bincode::config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        let bincode_bytes = bincode::encode_to_vec(bincode_mcpd, cfg).unwrap();

        assert_eq!(bincode_bytes.len(), size_of::<MotionCommandPacket>());

        assert_eq!(
            u32::from_be_bytes(bincode_bytes[0..4].try_into().unwrap()),
            0xabcd_ef01
        );
        assert_eq!(bincode_bytes[4], 0);
        assert_eq!(bincode_bytes[5], 0);
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[6..8].try_into().unwrap()),
            0xff00
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[8..10].try_into().unwrap()),
            0xabcd
        );
        assert_eq!(bincode_bytes[10], 1);
        assert_eq!(bincode_bytes[11], IoType::WI as u8);
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[12..14].try_into().unwrap()),
            0b010_1010_1101_0101
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[14..16].try_into().unwrap()),
            0b010_1010_1101_0101
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[16..18].try_into().unwrap()),
            0b010_1010_1101_0101
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[18..20].try_into().unwrap()),
            0xFFFF
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[20..22].try_into().unwrap()),
            0x3333
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[22..24].try_into().unwrap()),
            0x3333
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[24..32].try_into().unwrap()),
            0.001
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[32..40].try_into().unwrap()),
            0.01
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[40..48].try_into().unwrap()),
            0.1
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[48..56].try_into().unwrap()),
            1.0
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[56..64].try_into().unwrap()),
            10.0
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[64..72].try_into().unwrap()),
            100.0
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[72..80].try_into().unwrap()),
            1_000.0
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[80..88].try_into().unwrap()),
            10_000.0
        );
        assert_eq!(
            f64::from_be_bytes(bincode_bytes[88..96].try_into().unwrap()),
            100_000.0
        );
    }

    #[test]
    fn test_robot_status_packet_size() {
        let bincode_rsp = RobotStatusPacket {
            seq: 0xabcd_ef01,
            status: 0b1010_1010,
            read_io_type: IoType::None as u8,
            read_io_index: 0xff00,
            read_io_mask: 0xabcd,
            read_io_value: 0x1234,
            time_stamp: 0xdead_beef,
            pose: PoseData {
                x: 0.001,
                y: 0.01,
                z: 0.1,
                w: 1.0,
                p: 10.0,
                r: 100.0,
                e1: 1_000.0,
                e2: 10_000.0,
                e3: 100_000.0,
            },
            joints: [
                0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1_000.0, 10_000.0, 100_000.0,
            ],
            motor_current: [
                0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1_000.0, 10_000.0, 100_000.0,
            ],
        };
        let cfg = bincode::config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        let bincode_bytes = bincode::encode_to_vec(bincode_rsp, cfg).unwrap();
        assert_eq!(bincode_bytes.len(), size_of::<RobotStatusPacket>());

        assert_eq!(
            u32::from_be_bytes(bincode_bytes[0..4].try_into().unwrap()),
            0xabcd_ef01
        );
        assert_eq!(bincode_bytes[4], 0b1010_1010);
        assert_eq!(bincode_bytes[5], IoType::None as u8);
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[6..8].try_into().unwrap()),
            0xff00
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[8..10].try_into().unwrap()),
            0xabcd
        );
        assert_eq!(
            u16::from_be_bytes(bincode_bytes[10..12].try_into().unwrap()),
            0x1234
        );
        assert_eq!(
            u32::from_be_bytes(bincode_bytes[12..16].try_into().unwrap()),
            0xdead_beef
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[16..20].try_into().unwrap()),
            0.001
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[20..24].try_into().unwrap()),
            0.01
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[24..28].try_into().unwrap()),
            0.1
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[28..32].try_into().unwrap()),
            1.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[32..36].try_into().unwrap()),
            10.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[36..40].try_into().unwrap()),
            100.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[40..44].try_into().unwrap()),
            1_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[44..48].try_into().unwrap()),
            10_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[48..52].try_into().unwrap()),
            100_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[52..56].try_into().unwrap()),
            0.001
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[56..60].try_into().unwrap()),
            0.01
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[60..64].try_into().unwrap()),
            0.1
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[64..68].try_into().unwrap()),
            1.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[68..72].try_into().unwrap()),
            10.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[72..76].try_into().unwrap()),
            100.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[76..80].try_into().unwrap()),
            1_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[80..84].try_into().unwrap()),
            10_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[84..88].try_into().unwrap()),
            100_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[88..92].try_into().unwrap()),
            0.001
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[92..96].try_into().unwrap()),
            0.01
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[96..100].try_into().unwrap()),
            0.1
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[100..104].try_into().unwrap()),
            1.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[104..108].try_into().unwrap()),
            10.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[108..112].try_into().unwrap()),
            100.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[112..116].try_into().unwrap()),
            1_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[116..120].try_into().unwrap()),
            10_000.0
        );
        assert_eq!(
            f32::from_be_bytes(bincode_bytes[120..124].try_into().unwrap()),
            100_000.0
        );
    }

    // ---- MotionCommandPacket::try_from_joints / try_from_pose ----

    use crate::joints::{JointFormat, JointTemplate};

    #[test]
    fn try_from_joints_six_axis_stores_in_first_six_slots() {
        let joints = [-90.0, 0.0, 0.0, -180.0, 90.0, 180.0];
        let pkt = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            joints,
        )
        .unwrap();
        // Position is stored in FanucDeg internally (j3 -= j2; j2 == 0 here so
        // values match input exactly).
        for (i, &v) in joints.iter().enumerate() {
            assert!(
                (pkt.position[i] - v as f64).abs() < 1e-6,
                "axis {i}: got {} expected {}",
                pkt.position[i],
                v
            );
        }
        // Slots beyond the template length must remain at the constructor
        // default (0.0), not stale memory.
        for i in 6..9 {
            assert_eq!(pkt.position[i], 0.0, "axis {i} should be zero-filled");
        }
        assert!(pkt.joint_format, "joint_format flag must be set");
        assert!(!pkt.last_command);
        assert_eq!(pkt.seq, 0);
    }

    #[test]
    fn try_from_joints_converts_input_format_to_fanuc_deg() {
        // Same physical pose in two formats must produce the same `position`.
        let abs_deg = [-90.0, 30.0, 10.0, 0.0, 0.0, 0.0];
        let abs_rad = [
            -90.0_f64.to_radians(),
            30.0_f64.to_radians(),
            10.0_f64.to_radians(),
            0.0,
            0.0,
            0.0,
        ];

        let pkt_deg = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            abs_deg,
        )
        .unwrap();
        let pkt_rad = MotionCommandPacket::try_from_joints(
            JointFormat::AbsRad,
            JointTemplate::SIX,
            abs_rad,
        )
        .unwrap();

        for i in 0..6 {
            assert!(
                (pkt_deg.position[i] - pkt_rad.position[i]).abs() < 1e-4,
                "axis {i}: deg-input={} rad-input={} (should match in FanucDeg)",
                pkt_deg.position[i],
                pkt_rad.position[i]
            );
        }
    }

    #[test]
    fn try_from_joints_applies_fanuc_j3_relative_convention() {
        // AbsDeg → FanucDeg: j3 = j3_abs - j2.
        let pkt = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            [10.0, 25.0, 10.0, 0.0, 0.0, 0.0],
        )
        .unwrap();
        assert!((pkt.position[0] - 10.0).abs() < 1e-9);
        assert!((pkt.position[1] - 25.0).abs() < 1e-9);
        assert!(
            (pkt.position[2] - (10.0 - 25.0)).abs() < 1e-9,
            "j3 should be abs-relative converted to fanuc: got {}",
            pkt.position[2]
        );
    }

    #[test]
    fn try_from_joints_too_few_axes_for_template_returns_error() {
        // Template SIX wants 6 joint values; a [f64; 4] doesn't have enough
        // entries. With fill_missing_nan=false (the path try_from_joints
        // takes), this must return JointDataSizeError, not panic.
        let result = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            [10.0, 20.0, 30.0, 40.0],
        );
        assert!(
            result.is_err(),
            "expected error for under-sized input, got {result:?}"
        );
    }

    #[test]
    fn try_from_joints_too_few_axes_via_vec_returns_error() {
        // Same case via the Vec<f64> JointRepr impl — a separate code path
        // with the same contract.
        let v: Vec<f64> = vec![10.0, 20.0, 30.0, 40.0];
        let result = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            v,
        );
        assert!(
            result.is_err(),
            "expected error for under-sized vec input, got {result:?}"
        );
    }

    #[test]
    fn try_from_joints_seven_axis_linear_track() {
        // SIX_LINEAR_TRACK: 6 rotary + 1 linear (track in mm). The linear
        // axis must be passed through unchanged regardless of deg/rad input.
        let joints = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1234.5];
        let pkt = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX_LINEAR_TRACK,
            joints,
        )
        .unwrap();
        assert!((pkt.position[6] - 1234.5).abs() < 1e-6);
        for i in 7..9 {
            assert_eq!(pkt.position[i], 0.0);
        }
    }

    #[test]
    fn set_last_command_flips_flag() {
        let mut pkt = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .unwrap();
        assert!(!pkt.last_command);
        pkt.set_last_command(true);
        assert!(pkt.last_command);
        pkt.set_last_command(false);
        assert!(!pkt.last_command);
    }

    #[test]
    fn from_pose_disables_joint_format_flag() {
        let pose = PoseData {
            x: 100.0,
            y: 200.0,
            z: 300.0,
            w: 1.0,
            p: 2.0,
            r: 3.0,
            e1: 0.0,
            e2: 0.0,
            e3: 0.0,
        };
        let pkt = MotionCommandPacket::from_pose(pose).unwrap();
        assert!(!pkt.joint_format, "from_pose must clear joint_format");
        assert_eq!(pkt.position[0], 100.0);
        assert_eq!(pkt.position[1], 200.0);
        assert_eq!(pkt.position[2], 300.0);
    }

    #[test]
    fn should_cast_to_single_obeys_protocol_version_and_format() {
        let mut pkt = MotionCommandPacket::try_from_joints(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            [0.0; 6],
        )
        .unwrap();
        // version < 2 → always cast to single
        assert!(pkt.should_cast_to_single(0));
        assert!(pkt.should_cast_to_single(1));
        // version >= 2 with joint_format=true → don't cast
        assert!(!pkt.should_cast_to_single(2));
        // version >= 2 with joint_format=false → cast
        pkt.joint_format = false;
        assert!(pkt.should_cast_to_single(2));
    }
}
