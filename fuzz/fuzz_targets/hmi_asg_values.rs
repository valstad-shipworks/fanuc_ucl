#![no_main]

use fanuc_ucl::hmi::asg::{
    AlarmData, CartesianData, FrameData, HmiWireable, JointData, PositionData, TimeData,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = f32::unpack(data);
    let _ = i16::unpack_sysvar(data);
    let _ = u16::unpack(data);
    let _ = bool::unpack_sysvar(data);
    let _ = String::unpack(data);
    let _ = CartesianData::unpack(data);
    let _ = JointData::unpack(data);
    let _ = FrameData::unpack(data);
    let _ = PositionData::unpack(data);
    let _ = TimeData::unpack(data);
    let _ = AlarmData::unpack(data);
    if let [offset, view, rest @ ..] = data {
        let _ = PositionData::partial_unpack::<200>(rest, *offset as usize, *view as usize);
    }
});
