//! Device/controller-side HMI (SNP-X/SRTP) codec: frame and classify the driver's
//! requests, and build the responses its runner accepts. A thin re-expose of the
//! private [`wire`](super::proto::wire) encoders/decoders, mirroring STMO's device
//! split so the wire format keeps a single source of truth.

use super::BINCODE_CFG;
use super::proto::asg::AlarmSeverity;
use super::proto::wire::{Body, Header, Message, SegmentSelector, ServiceRequestCode};

/// Memory segment a request targets, named from the controller's frame (as the
/// wire encodes it). DO writes/reads arrive on [`InputBit`](Segment::InputBit)
/// and DI on [`OutputBit`](Segment::OutputBit) — FANUC names I/O inverted from
/// the host's perspective.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Segment {
    /// `DigitalOutput` (host DO) bank.
    InputBit,
    /// `DigitalInput` (host DI) bank.
    OutputBit,
    /// Integer register (`R[..]`) bank.
    Registers,
}

/// A single decoded request, from the controller's perspective.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum HmiRequest {
    /// Connection init (`MsgTyp::Init`). Answer with [`init_ack`].
    Init { seq: u8 },
    /// The post-init `Magic` probe. Answer with [`ack_resp`].
    Magic { seq: u8 },
    /// Bit-bank write. `index` is the on-wire target (DO number − 1 for DO writes);
    /// `bits[i]` is the value for wire bit `index + i`.
    WriteBits {
        seq: u8,
        segment: Segment,
        index: u16,
        bits: Vec<bool>,
    },
    /// Register block write. `index` is `R[addr] − 1`.
    WriteRegs {
        seq: u8,
        index: u16,
        values: Vec<i16>,
    },
    /// Bit-bank read of `count` bits from wire `index`.
    ReadBits {
        seq: u8,
        segment: Segment,
        index: u16,
        count: u16,
    },
    /// Register block read of `count` registers from `R[addr] − 1 == index`.
    ReadRegs { seq: u8, index: u16, count: u16 },
    /// A `GlobalByte` text command: `CLRASG` / `SETASG …` / `CLRALM`.
    Command { seq: u8, text: String },
    /// A request the emulator does not model; answer with [`ack_resp`] to keep the
    /// driver's runner unblocked.
    Other { seq: u8 },
}

impl HmiRequest {
    /// The request's sequence number, which the matching response must echo.
    pub fn seq(&self) -> u8 {
        match self {
            HmiRequest::Init { seq }
            | HmiRequest::Magic { seq }
            | HmiRequest::WriteBits { seq, .. }
            | HmiRequest::WriteRegs { seq, .. }
            | HmiRequest::ReadBits { seq, .. }
            | HmiRequest::ReadRegs { seq, .. }
            | HmiRequest::Command { seq, .. }
            | HmiRequest::Other { seq } => *seq,
        }
    }
}

/// Frame and classify one HMI message from the front of `buf`, returning the
/// request and the number of bytes it consumed, or `None` if `buf` does not yet
/// hold a complete message. Framing matches the driver's runner: a 42-byte header
/// decode gives `text_len`, and the whole message is `56 + text_len` bytes.
pub fn parse_request(buf: &[u8]) -> Option<(HmiRequest, usize)> {
    if buf.len() < 42 {
        return None;
    }
    let (header, _) = bincode::decode_from_slice::<Header, _>(&buf[..42], BINCODE_CFG).ok()?;
    let total = 56 + header.payload_len() as usize;
    if buf.len() < total {
        return None;
    }
    let (msg, _) = bincode::decode_from_slice::<Message, _>(&buf[..total], BINCODE_CFG).ok()?;
    Some((classify(&msg), total))
}

fn classify(msg: &Message) -> HmiRequest {
    let seq = msg.seq();
    match &msg.body {
        Body::Req {
            service_request,
            segment,
            target_index,
            target_size,
            payload,
            ..
        } => classify_fields(
            seq,
            *service_request,
            *segment,
            *target_index,
            *target_size,
            payload,
        ),
        Body::ExtReq {
            service_request,
            segment,
            target_index,
            target_size,
            payload,
            ..
        } => classify_fields(
            seq,
            *service_request,
            *segment,
            *target_index,
            *target_size,
            payload,
        ),
        _ => HmiRequest::Other { seq },
    }
}

fn classify_fields(
    seq: u8,
    service: ServiceRequestCode,
    segment: SegmentSelector,
    index: u16,
    size: u16,
    payload: &[u8],
) -> HmiRequest {
    use SegmentSelector as S;
    use ServiceRequestCode as Sr;
    if segment == S::Init {
        return HmiRequest::Init { seq };
    }
    if service == Sr::Magic || segment == S::Magic {
        return HmiRequest::Magic { seq };
    }
    match service {
        Sr::WriteSysMemory => match segment {
            S::InputBit => HmiRequest::WriteBits {
                seq,
                segment: Segment::InputBit,
                index,
                bits: unpack_bits(index, size, payload),
            },
            S::OutputBit => HmiRequest::WriteBits {
                seq,
                segment: Segment::OutputBit,
                index,
                bits: unpack_bits(index, size, payload),
            },
            S::Registers => HmiRequest::WriteRegs {
                seq,
                index,
                values: unpack_regs(size, payload),
            },
            S::GlobalByte => HmiRequest::Command {
                seq,
                text: text_of(size, payload),
            },
            _ => HmiRequest::Other { seq },
        },
        Sr::ReadSysMemory => match segment {
            S::Registers => HmiRequest::ReadRegs { seq, index, count: size },
            S::InputBit => HmiRequest::ReadBits {
                seq,
                segment: Segment::InputBit,
                index,
                count: size,
            },
            S::OutputBit => HmiRequest::ReadBits {
                seq,
                segment: Segment::OutputBit,
                index,
                count: size,
            },
            _ => HmiRequest::Other { seq },
        },
        _ => HmiRequest::Other { seq },
    }
}

/// `bits[i]` for `i in 0..count` sits at wire bit `index + i`, packed from bit
/// `index % 8` of the first payload byte upward (`pack_array_into`, ports.rs).
fn unpack_bits(index: u16, count: u16, payload: &[u8]) -> Vec<bool> {
    let start = (index % 8) as usize;
    (0..count as usize)
        .map(|i| {
            let pos = start + i;
            payload
                .get(pos / 8)
                .map(|b| (b >> (pos % 8)) & 1 != 0)
                .unwrap_or(false)
        })
        .collect()
}

fn unpack_regs(count: u16, payload: &[u8]) -> Vec<i16> {
    (0..count as usize)
        .map(|i| {
            let s = i * 2;
            if s + 1 < payload.len() {
                i16::from_le_bytes([payload[s], payload[s + 1]])
            } else {
                0
            }
        })
        .collect()
}

fn text_of(size: u16, payload: &[u8]) -> String {
    let n = (size as usize).min(payload.len());
    String::from_utf8_lossy(&payload[..n]).into_owned()
}

/// The verbatim init acknowledgement the driver byte-compares against
/// `Message::INIT_ACK` (`HmiDriver::connect`).
pub fn init_ack() -> Vec<u8> {
    bincode::encode_to_vec(Message::INIT_ACK, BINCODE_CFG).unwrap_or_default()
}

/// An empty-payload acknowledgement for `seq` (writes, magic, commands).
pub fn ack_resp(seq: u8) -> Vec<u8> {
    bincode::encode_to_vec(Message::resp(seq, [0u8; 6]), BINCODE_CFG).unwrap_or_default()
}

/// An extended response for `seq` carrying `regs` as little-endian `i16`s — the
/// register read reply (`caster_array::<Register>` decodes the payload).
pub fn ext_resp_regs(seq: u8, regs: &[i16]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(regs.len() * 2);
    for r in regs {
        payload.extend_from_slice(&r.to_le_bytes());
    }
    let msg = Message::ext_resp(seq, ServiceRequestCode::ReadSysMemory, SegmentSelector::Registers, payload);
    bincode::encode_to_vec(msg, BINCODE_CFG).unwrap_or_default()
}

/// An extended response for `seq` carrying a raw bit-bank byte block.
pub fn ext_resp_bits(seq: u8, bytes: &[u8]) -> Vec<u8> {
    let msg = Message::ext_resp(
        seq,
        ServiceRequestCode::ReadSysMemory,
        SegmentSelector::InputBit,
        bytes.to_vec(),
    );
    bincode::encode_to_vec(msg, BINCODE_CFG).unwrap_or_default()
}

/// A 100-register (`AlarmData`, 200-byte) block that decodes to an empty alarm
/// slot. `severity` (5th register) must be [`AlarmSeverity::None`] (128); left
/// zero it would decode as `Warning`, a phantom active alarm on every read.
pub fn empty_alarm_registers() -> [i16; 100] {
    let mut regs = [0i16; 100];
    regs[4] = AlarmSeverity::None as i16;
    regs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hmi::proto::ports::{DigitalOutput, Register};
    use crate::hmi::proto::wire::Message;

    fn encode(msg: Message) -> Vec<u8> {
        bincode::encode_to_vec(msg, BINCODE_CFG).unwrap()
    }

    #[test]
    fn parses_a_digital_output_write() {
        // DO20..=DO23 = 1,1,0,1 (gripper "active"): driver wire index = 20 - 1 = 19.
        let msg = Message::new_write_req::<DigitalOutput>(7, 20, &[true, true, false, true]);
        let bytes = encode(msg);
        let (req, consumed) = parse_request(&bytes).expect("frames one message");
        assert_eq!(consumed, bytes.len());
        match req {
            HmiRequest::WriteBits {
                seq,
                segment,
                index,
                bits,
            } => {
                assert_eq!(seq, 7);
                assert_eq!(segment, Segment::InputBit);
                assert_eq!(index, 19);
                assert_eq!(bits, vec![true, true, false, true]);
            }
            other => panic!("expected WriteBits, got {other:?}"),
        }
    }

    #[test]
    fn parses_init_and_magic() {
        assert!(matches!(
            parse_request(&encode(Message::INIT)).unwrap().0,
            HmiRequest::Init { seq: 0 }
        ));
        assert!(matches!(
            parse_request(&encode(Message::MAGIC)).unwrap().0,
            HmiRequest::Magic { seq: 1 }
        ));
    }

    #[test]
    fn parses_a_clrasg_command() {
        let msg = Message::new_write_req::<crate::hmi::proto::ports::Command>(
            2,
            0,
            &["CLRASG".to_string()],
        );
        match parse_request(&encode(msg)).unwrap().0 {
            HmiRequest::Command { seq, text } => {
                assert_eq!(seq, 2);
                assert_eq!(text, "CLRASG");
            }
            other => panic!("expected Command, got {other:?}"),
        }
    }

    #[test]
    fn init_ack_bytes_equal_init_ack_message() {
        let (msg, _) =
            bincode::decode_from_slice::<Message, _>(&init_ack(), BINCODE_CFG).unwrap();
        assert_eq!(msg, Message::INIT_ACK);
    }

    #[test]
    fn empty_alarm_block_reads_back_as_severity_none() {
        let regs = empty_alarm_registers();
        // 5th register (offset 4) is the AlarmData severity slot.
        assert_eq!(regs[4], 128);
        assert_eq!(regs[0], 0);
    }

    #[test]
    fn read_regs_roundtrips_through_ext_resp() {
        // The driver's `read_array` decrements a one-indexed address before
        // building the request, so R[1] arrives as wire index 0.
        let msg = Message::new_read_req::<Register>(9, 0, 100);
        match parse_request(&encode(msg)).unwrap().0 {
            HmiRequest::ReadRegs { seq, index, count } => {
                assert_eq!(seq, 9);
                assert_eq!(index, 0);
                assert_eq!(count, 100);
            }
            other => panic!("expected ReadRegs, got {other:?}"),
        }
    }
}
