#![no_main]

use fanuc_ucl::hspo::{JointAnglesPacket, TcpCartesianPositionPacket, VariablesPacket};
use libfuzzer_sys::fuzz_target;

fn roundtrip<T: bincode::Decode<()> + bincode::Encode>(data: &[u8]) {
    let config = bincode::config::standard()
        .with_fixed_int_encoding()
        .with_big_endian();
    if let Ok((p, _)) = bincode::decode_from_slice::<T, _>(data, config) {
        let again = bincode::encode_to_vec(&p, config).unwrap();
        assert_eq!(&data[..again.len()], &again[..]);
    }
}

fuzz_target!(|data: &[u8]| {
    match data.get(12..14).map(|t| u16::from_be_bytes([t[0], t[1]])) {
        Some(1) => roundtrip::<TcpCartesianPositionPacket>(data),
        Some(4) => roundtrip::<JointAnglesPacket>(data),
        Some(16) => roundtrip::<VariablesPacket>(data),
        _ => {}
    }
});
