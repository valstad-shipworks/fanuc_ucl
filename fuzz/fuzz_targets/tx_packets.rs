#![no_main]

use fanuc_ucl::stmo::proto::TxPackets;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = TxPackets::decode_from(data);
});
