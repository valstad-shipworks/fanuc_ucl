use fanuc_ucl::joints::{JointFormat, JointTemplate};

fn main() {
    let joints = vec![-90.0, 0.0, 0.0, -180.0, 90.0, 180.0];
    let joints_conv = JointFormat::AbsRad.convert_from(JointFormat::FanucDeg, JointTemplate::SIX, joints);
    println!("Converted joints: {:?}", joints_conv);
}