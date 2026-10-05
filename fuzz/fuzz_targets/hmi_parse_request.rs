#![no_main]

use fanuc_ucl::hmi::server::parse_request;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut rest = data;
    while let Some((_, used)) = parse_request(rest) {
        assert!(used >= 56 && used <= rest.len());
        rest = &rest[used..];
    }
});
