#![no_main]

use fanuc_ucl::rmi::ResponsePacket;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    for line in data.split(|&b| b == b'\n') {
        if let Ok(packet) = serde_json::from_slice::<ResponsePacket>(line) {
            let text = serde_json::to_string(&packet).unwrap();
            let again: ResponsePacket = serde_json::from_str(&text).unwrap();
            assert_eq!(again.packet_name(), packet.packet_name());
        }
    }
});
