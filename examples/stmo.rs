use std::time::Duration;

use fanuc_ucl::{
    ThreadConfig,
    joints::{JointFormat, JointTemplate},
    stmo::{StreamMotionDriver, proto::MotionCommandPacket},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = StreamMotionDriver::new([10, 0, 0, 1], false);
    driver.connect(Some(ThreadConfig::new(80, None)))?;
    driver.start(2.0)?;

    let limits = driver.fetch_movement_limits(0)?;
    println!("Movement limits: {:?}", limits);

    let home = [0.0f32, 0.0, 0.0, 0.0, -90.0, 0.0];

    // One command per interpolation cycle (8ms): sweep J1 through a
    // 10 degree sine wave over 4 seconds. For real control you should abide by movement limits
    let mut commands = Vec::with_capacity(500);
    for i in 0..500 {
        let mut joints = home;
        joints[0] += 10.0 * (i as f32 / 500.0 * std::f32::consts::TAU).sin();
        commands.push(MotionCommandPacket::try_from_joints(
            JointFormat::FanucDeg,
            JointTemplate::SIX,
            joints,
        )?);
    }
    let handle = driver.command_motion(commands)?;
    // free to do other work here while the batch streams out
    handle.wait_timeout(Duration::from_secs(10))?;

    // In-the-loop control: answer each status cycle with a fresh setpoint
    // instead of pre-computing the whole trajectory.
    {
        let mut ctl = driver.control_loop()?;
        for _ in 0..500 {
            let status = ctl.wait_for_status(Duration::from_millis(100))?;
            let mut joints = status.joints(JointFormat::FanucDeg, JointTemplate::SIX);
            joints[5] += 0.05; // random movement, would not pass limits validator
            ctl.send_command(MotionCommandPacket::try_from_joints(
                JointFormat::FanucDeg,
                JointTemplate::SIX,
                joints,
            )?)?;
        }
    }

    driver.stop();
    driver.disconnect();
    Ok(())
}
