use super::wire::SegmentSelector;

#[inline]
fn bits_to_bools(bits: u8) -> [bool; 8] {
    [
        (bits & 0x01) != 0,
        (bits & 0x02) != 0,
        (bits & 0x04) != 0,
        (bits & 0x08) != 0,
        (bits & 0x10) != 0,
        (bits & 0x20) != 0,
        (bits & 0x40) != 0,
        (bits & 0x80) != 0,
    ]
}

pub trait DataPort {
    #[cfg(not(feature = "py"))]
    type ValueType: Sized + std::fmt::Debug;
    #[cfg(feature = "py")]
    type ValueType: Sized
        + for<'a> pyo3::conversion::IntoPyObject<'a>
        + for<'py> pyo3::conversion::FromPyObjectOwned<'py, Error = pyo3::PyErr>
        + Clone
        + std::fmt::Debug;
    #[cfg(feature = "py")]
    type PyType: pyo3::type_object::PyTypeInfo;
    const SEGMENT: SegmentSelector;
    const OFFSET: u16;
    const ZERO_INDEXED: bool = true;
    const NAME: &'static str;
}

pub trait UnsafelyWritableDataPort: DataPort {
    fn pack_single(target_idx: u16, value: &Self::ValueType) -> Box<[u8]> {
        let mut buffer = vec![0u8; Self::size_of_single(target_idx, value)];
        Self::pack_single_into(target_idx, value, &mut buffer);
        buffer.into_boxed_slice()
    }
    fn pack_single_into(target_idx: u16, value: &Self::ValueType, buffer: &mut [u8]);
    fn pack_array(target_idx: u16, values: &[Self::ValueType]) -> Box<[u8]> {
        let mut buffer = vec![0u8; Self::size_of_array(target_idx, values)];
        Self::pack_array_into(target_idx, values, &mut buffer);
        buffer.into_boxed_slice()
    }
    fn pack_array_into(target_idx: u16, values: &[Self::ValueType], buffer: &mut [u8]);
    fn size_of_single(_target_idx: u16, _value: &Self::ValueType) -> usize {
        std::mem::size_of::<Self::ValueType>()
    }
    fn size_of_array(_target_idx: u16, values: &[Self::ValueType]) -> usize {
        std::mem::size_of::<Self::ValueType>() * values.len()
    }
    fn item_count(_target_idx: u16, values: &[Self::ValueType]) -> u16 {
        values.len() as u16
    }
}
pub trait WritableDataPort: DataPort + UnsafelyWritableDataPort {}

pub trait ReadableDataPort: DataPort {
    fn align_read(target_idx: u16, count: u16) -> (u16, u16) {
        (target_idx, count)
    }
    fn expected_size(_target_idx: u16, count: u16) -> usize {
        std::mem::size_of::<Self::ValueType>() * count as usize
    }
    fn unpack_single(target_idx: u16, data: &[u8]) -> Self::ValueType;
    fn unpack_array(target_idx: u16, data: &[u8], count: u16) -> Box<[Self::ValueType]>;
}

#[cfg(not(feature = "py"))]
macro_rules! python_type {
    ($value_type:ident) => {};
}

#[cfg(feature = "py")]
macro_rules! python_type {
    ( bool ) => {
        type PyType = pyo3::types::PyBool;
    };
    ( i16 ) => {
        type PyType = pyo3::types::PyInt;
    };
    ( u8 ) => {
        type PyType = pyo3::types::PyInt;
    };
    ( String ) => {
        type PyType = pyo3::types::PyString;
    };
}

macro_rules! port {
    (port: $name:ident, value_type: $value_type:ident, segment: $segment:expr, offset: $offset:expr, zero_indexed: $zero_indexed:expr) => {
        pub struct $name;
        impl DataPort for $name {
            type ValueType = $value_type;
            python_type!($value_type);
            const SEGMENT: SegmentSelector = $segment;
            const OFFSET: u16 = $offset;
            const ZERO_INDEXED: bool = $zero_indexed;
            const NAME: &'static str = stringify!($name);
        }
    };
}

macro_rules! readable_impl {
    ($name:ident, bool) => {
        impl ReadableDataPort for $name {
            // all reading for bools should be done byte aligned
            // so if reading starting at idx 10 for count 5
            // the actual read should be from idx 8 for count 8

            #[inline]
            fn align_read(target_idx: u16, count: u16) -> (u16, u16) {
                let target_idx_aligned = target_idx - (target_idx % 8);
                let end_idx = target_idx + count;
                let end_idx_aligned = if end_idx % 8 == 0 {
                    end_idx
                } else {
                    end_idx + (8 - (end_idx % 8))
                };
                let count_aligned = end_idx_aligned - target_idx_aligned;
                (target_idx_aligned, count_aligned)
            }

            #[inline]
            fn expected_size(target_idx: u16, count: u16) -> usize {
                let (_, count_aligned) = Self::align_read(target_idx, count);
                (count_aligned / 8) as usize
            }

            #[inline]
            fn unpack_single(target_idx: u16, data: &[u8]) -> Self::ValueType {
                let byte = data[0];
                let bit_pos = target_idx % 8;
                (byte & (1 << bit_pos)) != 0
            }

            #[inline]
            fn unpack_array(target_idx: u16, data: &[u8], count: u16) -> Box<[Self::ValueType]> {
                let mut out = Vec::with_capacity(count as usize);
                let start_byte = (target_idx / 8) as usize;
                let end_bit = target_idx + count;
                let end_byte = ((end_bit + 7) / 8) as usize;
                let mut current_bit = target_idx;
                for byte_idx in start_byte..end_byte {
                    let byte = data[byte_idx];
                    let bits = bits_to_bools(byte);
                    for bit in 0..8 {
                        if current_bit >= target_idx + count {
                            break;
                        }
                        if current_bit >= target_idx {
                            out.push(bits[bit]);
                        }
                        current_bit += 1;
                    }
                }
                out.into_boxed_slice()
            }
        }
    };
    ($name:ident, i16) => {
        impl ReadableDataPort for $name {
            #[inline]
            fn unpack_single(_: u16, data: &[u8]) -> Self::ValueType {
                i16::from_le_bytes([data[0], data[1]])
            }

            #[inline]
            fn unpack_array(_: u16, data: &[u8], count: u16) -> Box<[Self::ValueType]> {
                let mut out = Vec::with_capacity(count as usize);
                for i in 0..count as usize {
                    let start = i * 2;
                    let value = i16::from_le_bytes([data[start], data[start + 1]]);
                    out.push(value);
                }
                out.into_boxed_slice()
            }
        }
    };
    ($name:ident, u8) => {
        impl ReadableDataPort for $name {
            #[inline]
            fn unpack_single(_: u16, data: &[u8]) -> Self::ValueType {
                u8::from_le_bytes([data[0]])
            }

            #[inline]
            fn unpack_array(_: u16, data: &[u8], count: u16) -> Box<[Self::ValueType]> {
                Box::from(&data[0..count as usize])
            }
        }
    };
}

macro_rules! writable_impl {
    ($name:ident, bool) => {
        impl UnsafelyWritableDataPort for $name {
            // bool packing is done by bit banging into bytes with 1 bytes representing up to 8 bools
            // the bits are also byte aligned based on the target idx,
            // if the target idx is 16 then the first bool goes into the first bit,\
            // but if the target idx is 18 then the first bool goes into the third bit
            #[inline]
            fn pack_single_into(target_idx: u16, value: &Self::ValueType, buffer: &mut [u8]) {
                buffer[0] = 0;
                if *value {
                    buffer[0] |= 1 << (target_idx % 8);
                }
            }

            #[inline]
            fn pack_array_into(target_idx: u16, mut values: &[Self::ValueType], buffer: &mut [u8]) {
                let mut byte_idx = 0;
                let mut bit_offset = target_idx % 8;
                while !values.is_empty() {
                    let mut byte = 0u8;
                    for bit in bit_offset..8 {
                        if values.is_empty() {
                            break;
                        }
                        if values[0] {
                            byte |= 1 << bit;
                        }
                        values = &values[1..];
                    }
                    buffer[byte_idx as usize] = byte;
                    byte_idx += 1;
                    bit_offset = 0;
                }
            }

            #[inline]
            fn size_of_array(target_idx: u16, values: &[Self::ValueType]) -> usize {
                let total_bits = target_idx as usize % 8 + values.len();
                (total_bits + 7) / 8
            }
        }
    };
    ($name:ident, i16) => {
        impl UnsafelyWritableDataPort for $name {
            #[inline]
            fn pack_single_into(_: u16, value: &Self::ValueType, buffer: &mut [u8]) {
                buffer[..2].copy_from_slice(&value.to_le_bytes());
            }

            #[inline]
            fn pack_array_into(_: u16, values: &[Self::ValueType], buffer: &mut [u8]) {
                for (i, &value) in values.iter().enumerate() {
                    let start = i * 2;
                    buffer[start..start + 2].copy_from_slice(&value.to_le_bytes());
                }
            }
        }
    };
    ($name:ident, u8) => {
        impl UnsafelyWritableDataPort for $name {
            #[inline]
            fn pack_single_into(value: &Self::ValueType, buffer: &mut [u8]) {
                buffer[0] = *value;
            }

            #[inline]
            fn pack_array_into(values: &[Self::ValueType], buffer: &mut [u8]) {
                buffer[..values.len()].copy_from_slice(values);
            }
        }
    };
}

macro_rules! input_port {
    {port: $name:ident, value_type: $value_type:ident, segment: $segment:expr, offset: $offset:expr, zero_indexed: $zero_indexed:tt} => {
        port! {
            port: $name,
            value_type: $value_type,
            segment: $segment,
            offset: $offset,
            zero_indexed: $zero_indexed
        }
        writable_impl!($name, $value_type);
        readable_impl!($name, $value_type);
    };
    {port: $name:ident, value_type: $value_type:ident, segment: $segment:expr, offset: $offset:expr} => {
        port! {
            port: $name,
            value_type: $value_type,
            segment: $segment,
            offset: $offset,
            zero_indexed: true
        }
        writable_impl!($name, $value_type);
        readable_impl!($name, $value_type);
    };
}

macro_rules! output_port {
    {port: $name:ident, value_type: $value_type:ident, segment: $segment:expr, offset: $offset:expr, zero_indexed: $zero_indexed:tt} => {
        port! {
            port: $name,
            value_type: $value_type,
            segment: $segment,
            offset: $offset,
            zero_indexed: $zero_indexed
        }
        writable_impl!($name, $value_type);
        readable_impl!($name, $value_type);
        impl WritableDataPort for $name {}
    };
    {port: $name:ident, value_type: $value_type:ident, segment: $segment:expr, offset: $offset:expr $(,)?} => {
        port! {
            port: $name,
            value_type: $value_type,
            segment: $segment,
            offset: $offset,
            zero_indexed: false
        }
        writable_impl!($name, $value_type);
        readable_impl!($name, $value_type);
        impl WritableDataPort for $name {}
    };
}

input_port! {
    port: DigitalInput,
    value_type: bool,
    segment: SegmentSelector::OutputBit,
    offset: 0
}
output_port! {
    port: DigitalOutput,
    value_type: bool,
    segment: SegmentSelector::InputBit,
    offset: 0
}

input_port! {
    port: RobotInput,
    value_type: bool,
    segment: SegmentSelector::OutputBit,
    offset: 5000
}
output_port! {
    port: RobotOutput,
    value_type: bool,
    segment: SegmentSelector::InputBit,
    offset: 5000
}

input_port! {
    port: UopInput,
    value_type: bool,
    segment: SegmentSelector::OutputBit,
    offset: 6000
}
output_port! {
    port: UopOutput,
    value_type: bool,
    segment: SegmentSelector::InputBit,
    offset: 6000
}

input_port! {
    port: SopInput,
    value_type: bool,
    segment: SegmentSelector::OutputBit,
    offset: 7000,
    zero_indexed: false
}
output_port! {
    port: SopOutput,
    value_type: bool,
    segment: SegmentSelector::InputBit,
    offset: 7000,
    zero_indexed: false
}

input_port! {
    port: WeldInput,
    value_type: bool,
    segment: SegmentSelector::OutputBit,
    offset: 8000
}
output_port! {
    port: WeldOutput,
    value_type: bool,
    segment: SegmentSelector::InputBit,
    offset: 8000
}

input_port! {
    port: WireStickInput,
    value_type: bool,
    segment: SegmentSelector::OutputBit,
    offset: 8400
}
output_port! {
    port: WireStickOutput,
    value_type: bool,
    segment: SegmentSelector::InputBit,
    offset: 8400
}

input_port! {
    port: GroupInput,
    value_type: i16,
    segment: SegmentSelector::AnalogOutput,
    offset: 0
}
output_port! {
    port: GroupOutput,
    value_type: i16,
    segment: SegmentSelector::AnalogInput,
    offset: 0
}
input_port! {
    port: AnalogInput,
    value_type: i16,
    segment: SegmentSelector::AnalogOutput,
    offset: 1000
}
output_port! {
    port: AnalogOutput,
    value_type: i16,
    segment: SegmentSelector::AnalogInput,
    offset: 1000
}

output_port! {
    port: Register,
    value_type: i16,
    segment: SegmentSelector::Registers,
    offset: 0
}

port! {
    port: Command,
    value_type: String,
    segment: SegmentSelector::GlobalByte,
    offset: 0,
    zero_indexed: true
}
impl UnsafelyWritableDataPort for Command {
    #[inline]
    fn pack_single(_: u16, value: &String) -> Box<[u8]> {
        value.clone().into_bytes().into_boxed_slice()
    }

    fn pack_single_into(_: u16, value: &String, buffer: &mut [u8]) {
        buffer.copy_from_slice(value.as_bytes());
    }

    fn pack_array_into(_: u16, values: &[Self::ValueType], buffer: &mut [u8]) {
        let separator = b'\r';
        let mut write_index = 0;
        for (i, value) in values.iter().enumerate() {
            let bytes = value.as_bytes();
            let end_index = write_index + bytes.len();
            buffer[write_index..end_index].copy_from_slice(bytes);
            write_index = end_index;
            if i < values.len() - 1 {
                buffer[write_index] = separator;
                write_index += 1;
            }
        }
    }

    fn size_of_single(_: u16, value: &String) -> usize {
        value.len()
    }

    fn size_of_array(_: u16, values: &[String]) -> usize {
        let mut size_sum = 0;
        for value in values {
            if size_sum > 0 {
                size_sum += 1; // for separator
            }
            size_sum += Self::size_of_single(0, value);
        }
        size_sum
    }

    fn item_count(_: u16, values: &[Self::ValueType]) -> u16 {
        Self::size_of_array(0, values) as u16
    }
}
impl WritableDataPort for Command {}
