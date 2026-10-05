#![no_main]

use fanuc_ucl::stmo::proto::RxPackets;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Some(rx) = RxPackets::decode_from(data) {
        let version = u32::from_be_bytes(data[4..8].try_into().unwrap());
        let mut buf = [0u8; 512];
        let n = rx.encode_into(version, &mut buf).unwrap();
        assert_eq!(&data[..n], &buf[..n]);
    }
});
