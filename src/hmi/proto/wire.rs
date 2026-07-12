use bincode::{Decode, Encode};
use bitflags::bitflags;
use int_enum::IntEnum;

use crate::{
    bincode_enum,
    hmi::proto::ports::{ReadableDataPort, UnsafelyWritableDataPort},
};

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum, Default)]
#[repr(u8)]
pub enum ServiceRequestCode {
    PLCStatus = 0x00,
    ReturnProgName = 0x03,
    ReadSysMemory = 0x04, // Used to read general memory register (Example: %R12344)
    ReadTaskMemory = 0x05,
    ReadProgMemory = 0x06,
    WriteSysMemory = 0x07, // Used to write general memory
    WriteTaskMemory = 0x08,
    WriteProgMemory = 0x09,
    ProgLogon = 0x20,
    ChangePriv = 0x21,
    SetCpuId = 0x22,
    SetPlcRun = 0x23,
    SetPlcTime = 0x24,
    ReturnDateTime = 0x25,
    GetFault = 0x38,
    ClearFault = 0x39,
    ReturnControllerType = 0x43,
    Magic = 0x4f,
    #[default]
    Unknown = 0xff,
}
bincode_enum!(with_default ServiceRequestCode : u8);

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum, Default)]
#[repr(u8)]
pub enum SegmentSelector {
    Init = 0x00,
    Magic = 0x01,
    InputBit = 0x46,
    InputByte = 0x10,
    OutputBit = 0x48,
    OutputByte = 0x12,
    TimerBit = 0x4a,
    TimerByte = 0x14,
    FlagBit = 0x4c,
    FlagByte = 0x16,
    SABit = 0x4e,
    SAByte = 0x18,
    SBBit = 0x50,
    SBByte = 0x1a,
    SCBit = 0x52,
    SCByte = 0x1c,
    SBit = 0x54,
    SByte = 0x1e,
    GlobalBit = 0x56,
    GlobalByte = 0x38,
    AnalogInput = 0x0a,
    AnalogOutput = 0x0c,
    Registers = 0x08,
    ProgramBlockData = 0x04,
    #[default]
    Unknown = 0xff,
}
bincode_enum!(with_default SegmentSelector : u8);

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum, Default)]
#[repr(u16)]
pub(super) enum PktType {
    InitTx = 0x00,
    InitRx = 0x01,
    Transmit = 0x02,
    Receive = 0x03,
    #[default]
    Unknown = 0x08,
}
bincode_enum!(with_default PktType : u16);

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum, Default)]
#[repr(u32)]
pub(super) enum OperationType {
    Init = 0x00000000,
    InitFinish = 0x00000001,
    Writing = 0x00000100,
    Reading = 0x00000200,
    #[default]
    Invalid = 0xffffffff,
}
bincode_enum!(with_default OperationType : u32);

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum, Default)]
#[repr(u8)]
pub(super) enum MsgTyp {
    Init = 0x00,
    Error = 0xd1,
    Req = 0xc0,
    Resp = 0xd4,
    ExtReq = 0x80,
    ExtResp = 0x94,
    #[default]
    Invalid = 0xff,
}
bincode_enum!(with_default MsgTyp : u8);

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum, Default)]
#[repr(u8)]
pub enum PlcState {
    RunIoEnabled = 0,
    RunIoDisabled = 1,
    StopIoDisabled = 2,
    CpuStopFaulted = 3,
    CpuHalted = 4,
    CpuSuspended = 5,
    StopIoEnabled = 6,
    #[default]
    Invalid = 0b1111,
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PlcStatusFlags: u16 {
        const OVERSWEEP = 0b0000_0000_0000_0001;
        const CONSTANT_SWEEP_MODE = 0b0000_0000_0000_0010;
        const PLC_FAULT_ENTRY_SINCE_LAST_READ = 0b0000_0000_0000_0100;
        const IO_FAULT_ENTRY_SINCE_LAST_READ = 0b0000_0000_0000_1000;
        const PLC_FAULT_ENTRY_PRESENT = 0b0000_0000_0001_0000;
        const IO_FAULT_ENTRY_PRESENT = 0b0000_0000_0010_0000;
        const PROGRAMMER_ATTACHMENT_FOUND = 0b0000_0000_0100_0000;
        const OUTPUTS_DISABLED = 0b0000_0000_1000_0000;
        const RUN_MODE = 0b0000_0001_0000_0000;
        const OEM_PROTECTION_IN_EFFECT = 0b0000_0010_0000_0000;
        const PLC_STATE_MASK = 0b1111_0000_0000_0000;
    }
}

impl bincode::Encode for PlcStatusFlags {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.bits().encode(encoder)
    }
}
impl<Ctx> bincode::Decode<Ctx> for PlcStatusFlags {
    fn decode<D: bincode::de::Decoder<Context = Ctx>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let variant_int = <u16 as bincode::Decode<D::Context>>::decode(decoder)?;
        Ok(PlcStatusFlags::from_bits_retain(variant_int))
    }
}
impl<Ctx> bincode::de::BorrowDecode<'_, Ctx> for PlcStatusFlags {
    fn borrow_decode<D: bincode::de::Decoder<Context = Ctx>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        <Self as bincode::Decode<Ctx>>::decode(decoder)
    }
}

impl PlcStatusFlags {
    pub fn plc_state(&self) -> PlcState {
        let state_bits = ((self.bits() & Self::PLC_STATE_MASK.bits()) >> 12) as u8;
        PlcState::try_from(state_bits).unwrap_or_default()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode)]
pub struct PlcStatus {
    ctrl_prog_num: u8,
    priv_lvl: u8,
    last_sweep: u16,
    status_flags: PlcStatusFlags,
}

#[cfg_attr(feature = "valuable", derive(valuable::Valuable))]
#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode)]
#[repr(C)]
pub struct Header {
    pkt_type: PktType,
    seq1: u16,
    text_len: u16,
    unk1: u16,
    operation1: OperationType,
    unk2: u32,
    operation2: OperationType,
    unk3: u32,
    unk4: u16,
    time: (u8, u8, u8),
    unk5: u8,
    seq2: u8,
    msg_type: MsgTyp,
    mbox_src: u32,
    mbox_dst: u32,
    pkt_num: u8,
    total_pkt_num: u8,
}

impl Header {
    pub const INIT: Header = Header {
        pkt_type: PktType::InitTx,
        seq1: 0,
        text_len: 0,
        unk1: 0,
        operation1: OperationType::Init,
        unk2: 0,
        operation2: OperationType::Init,
        unk3: 0,
        unk4: 0,
        time: (0, 0, 0),
        unk5: 0,
        seq2: 0,
        msg_type: MsgTyp::Init,
        mbox_src: 0,
        mbox_dst: 0,
        pkt_num: 0,
        total_pkt_num: 0,
    };

    pub const INIT_ACK: Header = Header {
        pkt_type: PktType::InitRx,
        operation1: OperationType::InitFinish,
        ..Header::INIT
    };

    pub const TEMPLATE: Header = Header {
        pkt_type: PktType::Transmit,
        mbox_dst: 0x00000e10,
        pkt_num: 1,
        total_pkt_num: 1,
        ..Header::INIT
    };

    pub const MAGIC: Header = Header {
        pkt_type: PktType::Unknown,
        seq1: 1,
        text_len: 0,
        unk1: 0,
        operation1: OperationType::Writing,
        unk2: 0,
        operation2: OperationType::Writing,
        unk3: 0,
        unk4: 0,
        time: (0, 0, 0),
        unk5: 0,
        seq2: 1,
        msg_type: MsgTyp::Req,
        mbox_src: 0,
        mbox_dst: 0x00000e10,
        pkt_num: 1,
        total_pkt_num: 1,
    };

    pub fn payload_len(&self) -> u16 {
        self.text_len
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Decode)]
pub enum Body {
    Req {
        service_request: ServiceRequestCode,
        segment: SegmentSelector,
        target_index: u16,
        target_size: u16,
        payload: [u8; 6],
        unk: u16,
    },
    Resp {
        unk1: u16,
        payload: [u8; 6],
        plc_status: PlcStatus,
    },
    ExtReq {
        unk: [u8; 6],
        pkt_num: u8,
        total_pkt_num: u8,
        service_request: ServiceRequestCode,
        segment: SegmentSelector,
        target_index: u16,
        target_size: u16,
        payload: Box<[u8]>,
    },
    ExtResp {
        unk: [u8; 6],
        pkt_num: u8,
        total_pkt_num: u8,
        service_request: ServiceRequestCode,
        segment: SegmentSelector,
        unk2: u16,
        plc_flags: PlcStatusFlags,
        payload: Box<[u8]>,
    },
}

impl Body {
    pub const INIT: Body = Body::Req {
        service_request: ServiceRequestCode::PLCStatus,
        segment: SegmentSelector::Init,
        target_index: 0,
        target_size: 0,
        payload: [0; 6],
        unk: 0,
    };

    pub const MAGIC: Body = Body::Req {
        service_request: ServiceRequestCode::Magic,
        segment: SegmentSelector::Magic,
        target_index: 0,
        target_size: 0,
        payload: [0; 6],
        unk: 0,
    };

    pub fn size(&self) -> usize {
        match self {
            Self::Req { .. } => 16,
            Self::Resp { .. } => 16,
            Self::ExtReq { payload, .. } => 16 + payload.len(),
            Self::ExtResp { payload, .. } => 16 + payload.len(),
        }
    }
}

impl Encode for Body {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        match self {
            Self::Req {
                service_request,
                segment,
                target_index,
                target_size,
                payload,
                unk,
            } => {
                Encode::encode(service_request, encoder)?;
                Encode::encode(segment, encoder)?;
                Encode::encode(target_index, encoder)?;
                Encode::encode(target_size, encoder)?;
                Encode::encode(payload, encoder)?;
                Encode::encode(unk, encoder)?;
                Ok(())
            }
            Self::Resp {
                unk1,
                payload,
                plc_status,
            } => {
                Encode::encode(unk1, encoder)?;
                Encode::encode(payload, encoder)?;
                Encode::encode(plc_status, encoder)?;
                Ok(())
            }
            Self::ExtReq {
                unk,
                pkt_num,
                total_pkt_num,
                service_request,
                segment: memory_type,
                target_index,
                target_size,
                payload,
            } => {
                Encode::encode(unk, encoder)?;
                Encode::encode(pkt_num, encoder)?;
                Encode::encode(total_pkt_num, encoder)?;
                Encode::encode(service_request, encoder)?;
                Encode::encode(memory_type, encoder)?;
                Encode::encode(target_index, encoder)?;
                Encode::encode(target_size, encoder)?;
                for byte in payload.iter() {
                    Encode::encode(byte, encoder)?;
                }
                Ok(())
            }
            Self::ExtResp {
                unk,
                pkt_num,
                total_pkt_num,
                service_request,
                segment: memory_type,
                unk2,
                plc_flags,
                payload,
            } => {
                Encode::encode(unk, encoder)?;
                Encode::encode(pkt_num, encoder)?;
                Encode::encode(total_pkt_num, encoder)?;
                Encode::encode(service_request, encoder)?;
                Encode::encode(memory_type, encoder)?;
                Encode::encode(unk2, encoder)?;
                Encode::encode(plc_flags, encoder)?;
                for byte in payload.iter() {
                    Encode::encode(byte, encoder)?;
                }
                Ok(())
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Encode)]
pub struct Message {
    pub header: Header,
    pub body: Body,
}

fn decode_bytes<Context, D: bincode::de::Decoder<Context = Context>>(
    decoder: &mut D,
    count: usize,
) -> Result<Box<[u8]>, bincode::error::DecodeError> {
    let mut arr = Vec::with_capacity(count);
    for _ in 0..count {
        let byte = <u8 as bincode::Decode<D::Context>>::decode(decoder)?;
        arr.push(byte);
    }
    Ok(arr.into_boxed_slice())
}

impl<Context> bincode::Decode<Context> for Message {
    fn decode<D: bincode::de::Decoder<Context = Context>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let header: Header = Decode::decode(decoder)?;
        let body = match header.msg_type {
            MsgTyp::Init => Ok(Body::INIT),
            MsgTyp::Req => Ok(Body::Req {
                service_request: Decode::<D::Context>::decode(decoder)?,
                segment: Decode::<D::Context>::decode(decoder)?,
                target_index: Decode::<D::Context>::decode(decoder)?,
                target_size: Decode::<D::Context>::decode(decoder)?,
                payload: Decode::<D::Context>::decode(decoder)?,
                unk: Decode::<D::Context>::decode(decoder)?,
            }),
            MsgTyp::Resp => Ok(Body::Resp {
                unk1: Decode::<D::Context>::decode(decoder)?,
                payload: Decode::<D::Context>::decode(decoder)?,
                plc_status: Decode::<D::Context>::decode(decoder)?,
            }),
            MsgTyp::ExtReq => Ok(Body::ExtReq {
                unk: Decode::<D::Context>::decode(decoder)?,
                pkt_num: Decode::<D::Context>::decode(decoder)?,
                total_pkt_num: Decode::<D::Context>::decode(decoder)?,
                service_request: Decode::<D::Context>::decode(decoder)?,
                segment: Decode::<D::Context>::decode(decoder)?,
                target_index: Decode::<D::Context>::decode(decoder)?,
                target_size: Decode::<D::Context>::decode(decoder)?,
                payload: decode_bytes(decoder, header.text_len as usize)?,
            }),
            MsgTyp::ExtResp => Ok(Body::ExtResp {
                unk: Decode::<D::Context>::decode(decoder)?,
                pkt_num: Decode::<D::Context>::decode(decoder)?,
                total_pkt_num: Decode::<D::Context>::decode(decoder)?,
                service_request: Decode::<D::Context>::decode(decoder)?,
                segment: Decode::<D::Context>::decode(decoder)?,
                unk2: Decode::<D::Context>::decode(decoder)?,
                plc_flags: Decode::<D::Context>::decode(decoder)?,
                payload: decode_bytes(decoder, header.text_len as usize)?,
            }),
            MsgTyp::Invalid | MsgTyp::Error => {
                Err(::bincode::error::DecodeError::UnexpectedVariant {
                    found: header.msg_type as u32,
                    type_name: "Message",
                    allowed: &bincode::error::AllowedEnumVariants::Allowed(&[
                        MsgTyp::Req as u32,
                        MsgTyp::Resp as u32,
                        MsgTyp::ExtReq as u32,
                        MsgTyp::ExtResp as u32,
                    ]),
                })
            }
        }?;
        Ok(Self { header, body })
    }
}
impl<'de, Context> bincode::BorrowDecode<'de, Context> for Message {
    fn borrow_decode<D: bincode::de::BorrowDecoder<'de, Context = Context>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        <Self as bincode::Decode<Context>>::decode(decoder)
    }
}

impl Message {
    pub const INIT: Message = Message {
        header: Header::INIT,
        body: Body::INIT,
    };

    pub const INIT_ACK: Message = Message {
        header: Header::INIT_ACK,
        body: Body::INIT,
    };

    pub const MAGIC: Message = Message {
        header: Header::MAGIC,
        body: Body::MAGIC,
    };

    pub fn seq(&self) -> u8 {
        self.header.seq1 as u8
    }

    pub fn payload(&self) -> &[u8] {
        match &self.body {
            Body::Req { payload, .. } => payload,
            Body::Resp { payload, .. } => payload,
            Body::ExtReq { payload, .. } => payload,
            Body::ExtResp { payload, .. } => payload,
        }
    }

    pub(crate) fn new_read_req<T: ReadableDataPort>(seq: u8, index: u16, count: u16) -> Message {
        Message {
            header: Header {
                seq1: seq as u16,
                seq2: seq,
                msg_type: MsgTyp::Req,
                operation1: OperationType::Reading,
                operation2: OperationType::Reading,
                ..Header::TEMPLATE
            },
            body: Body::Req {
                service_request: ServiceRequestCode::ReadSysMemory,
                segment: T::SEGMENT,
                payload: [0; 6],
                target_index: T::OFFSET + index,
                target_size: count,
                unk: 0,
            },
        }
    }

    pub(crate) fn new_write_req<T: UnsafelyWritableDataPort>(
        seq: u8,
        mut index: u16,
        data: &[T::ValueType],
    ) -> Message {
        index += T::OFFSET;
        if !T::ZERO_INDEXED {
            index -= 1;
        }
        let data_count = T::item_count(index, data);
        let data_byte_cnt = T::size_of_array(index, data);
        if data_byte_cnt <= 6 {
            log::trace!(
                "Packing write request for port {} at index {} and count {} with data {:?} into inline payload",
                T::NAME,
                index,
                data_count,
                data
            );
            let mut payload = [0u8; 6];
            T::pack_array_into(index, data, &mut payload);
            Message {
                header: Header {
                    seq1: seq as u16,
                    seq2: seq,
                    text_len: 0,
                    msg_type: MsgTyp::Req,
                    operation1: OperationType::Writing,
                    operation2: OperationType::Writing,
                    ..Header::TEMPLATE
                },
                body: Body::Req {
                    service_request: ServiceRequestCode::WriteSysMemory,
                    segment: T::SEGMENT,
                    payload,
                    target_index: index,
                    target_size: data_count,
                    unk: 0,
                },
            }
        } else {
            log::trace!(
                "Packing write request for port {} at index {} and count {} with data {:?} into extended payload",
                T::NAME,
                index,
                data_count,
                data
            );
            let payload = T::pack_array(index, data);
            log::debug!("Extended write payload size: {} bytes", payload.len());
            Message {
                header: Header {
                    seq1: seq as u16,
                    seq2: seq,
                    text_len: payload.len() as u16,
                    msg_type: MsgTyp::ExtReq,
                    operation1: OperationType::Writing,
                    operation2: OperationType::Writing,
                    ..Header::TEMPLATE
                },
                body: Body::ExtReq {
                    unk: [0u8; 6],
                    pkt_num: 1,
                    total_pkt_num: 1,
                    service_request: ServiceRequestCode::WriteSysMemory,
                    segment: T::SEGMENT,
                    target_index: index,
                    target_size: data_count,
                    payload,
                },
            }
        }
    }

    /// A short (inline-payload) `Resp` acknowledging `seq`. Used device-side to
    /// answer writes and handshake steps the driver only checks for arrival.
    pub(crate) fn resp(seq: u8, payload: [u8; 6]) -> Message {
        Message {
            header: Header {
                pkt_type: PktType::Receive,
                seq1: seq as u16,
                seq2: seq,
                text_len: 0,
                msg_type: MsgTyp::Resp,
                operation1: OperationType::Reading,
                operation2: OperationType::Reading,
                mbox_src: 0x00000e10,
                ..Header::TEMPLATE
            },
            body: Body::Resp {
                unk1: 0,
                payload,
                plc_status: PlcStatus {
                    ctrl_prog_num: 0,
                    priv_lvl: 0,
                    last_sweep: 0,
                    status_flags: PlcStatusFlags::empty(),
                },
            },
        }
    }

    /// An `ExtResp` carrying `payload` for `seq`. Used device-side for reads whose
    /// data exceeds the 6-byte inline `Resp` payload (registers, bit arrays).
    pub(crate) fn ext_resp(
        seq: u8,
        service_request: ServiceRequestCode,
        segment: SegmentSelector,
        payload: Vec<u8>,
    ) -> Message {
        Message {
            header: Header {
                pkt_type: PktType::Receive,
                seq1: seq as u16,
                seq2: seq,
                text_len: payload.len() as u16,
                msg_type: MsgTyp::ExtResp,
                operation1: OperationType::Reading,
                operation2: OperationType::Reading,
                mbox_src: 0x00000e10,
                ..Header::TEMPLATE
            },
            body: Body::ExtResp {
                unk: [0u8; 6],
                pkt_num: 1,
                total_pkt_num: 1,
                service_request,
                segment,
                unk2: 0,
                plc_flags: PlcStatusFlags::empty(),
                payload: payload.into_boxed_slice(),
            },
        }
    }

    #[cfg(test)]
    pub fn new_test_resp(seq: u8, payload: [u8; 6], plc_status: PlcStatus) -> Message {
        Message {
            header: Header {
                pkt_type: PktType::Receive,
                seq1: seq as u16,
                seq2: seq,
                text_len: 0,
                msg_type: MsgTyp::Resp,
                operation1: OperationType::Reading,
                operation2: OperationType::Reading,
                mbox_src: 0x00000e10,
                ..Header::TEMPLATE
            },
            body: Body::Resp {
                unk1: 0,
                payload,
                plc_status,
            },
        }
    }

    #[cfg(test)]
    pub fn new_test_ext_resp(
        seq: u8,
        service_request: ServiceRequestCode,
        segment: SegmentSelector,
        payload: Vec<u8>,
    ) -> Message {
        Message {
            header: Header {
                pkt_type: PktType::Receive,
                seq1: seq as u16,
                seq2: seq,
                text_len: payload.len() as u16,
                msg_type: MsgTyp::ExtResp,
                operation1: OperationType::Reading,
                operation2: OperationType::Reading,
                mbox_src: 0x00000e10,
                ..Header::TEMPLATE
            },
            body: Body::ExtResp {
                unk: [0u8; 6],
                pkt_num: 1,
                total_pkt_num: 1,
                service_request,
                segment,
                unk2: 0,
                plc_flags: PlcStatusFlags::empty(),
                payload: payload.into_boxed_slice(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use bincode::config;

    #[test]
    fn test_message_size() {
        use super::*;
        let msg = Message::new_read_req::<crate::hmi::proto::ports::DigitalInput>(1, 0, 10);
        let encoded = bincode::encode_to_vec(msg, crate::hmi::BINCODE_CFG).unwrap();
        assert_eq!(encoded.len(), 56);
    }

    #[test]
    fn test_ext_message_size() {
        use super::*;
        let msg = Message::new_write_req::<crate::hmi::proto::ports::Command>(
            1,
            0,
            &["123456789012345678901234567890".to_string()],
        );
        println!("{}", msg.payload().len());
        let encoded = bincode::encode_to_vec(msg, crate::hmi::BINCODE_CFG).unwrap();
        assert_eq!(encoded.len(), 56 + 30);
    }

    #[test]
    fn test_message_decode() {
        use super::*;
        const BYTES: [u8; 136] = [
            0x3, 0x0, 0x4, 0x0, 0x50, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x94, 0x10, 0xe,
            0x0, 0x0, 0x30, 0x3a, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0xff,
            0x4, 0x0, 0x0, 0x7c, 0x21, 0x31, 0x30, 0x2e, 0x32, 0x33, 0x2e, 0x31, 0x36, 0x2e, 0x32,
            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x0, 0x0,
        ];
        const BINCODE_CFG: config::Configuration<config::LittleEndian, config::Fixint> =
            bincode::config::standard()
                .with_little_endian()
                .with_fixed_int_encoding();

        let (msg, _consumed) =
            bincode::decode_from_slice::<Message, _>(&BYTES, BINCODE_CFG).unwrap();
        println!("Decoded message: {:?}", msg);
    }

    #[test]
    fn test_message_decode2() {
        use super::*;
        const BYTES: [u8; 56] = [
            0x3, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0xd4, 0x10, 0xe,
            0x0, 0x0, 0x30, 0x3a, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0xff,
            0x4, 0x0, 0x0, 0x7c, 0x21,
        ];
        const BINCODE_CFG: config::Configuration<config::LittleEndian, config::Fixint> =
            bincode::config::standard()
                .with_little_endian()
                .with_fixed_int_encoding();

        let (msg, _consumed) =
            bincode::decode_from_slice::<Message, _>(&BYTES, BINCODE_CFG).unwrap();
        println!("Decoded message: {:?}", msg);
    }
}
