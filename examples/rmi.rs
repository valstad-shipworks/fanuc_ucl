use std::time::Duration;

use fanuc_ucl::{
    ThreadConfig,
    joints::{JointFormat, JointTemplate},
    rmi::{RmiDriver, RmiDriverConfig, proto},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = RmiDriver::new(RmiDriverConfig::default_with_ip([10, 0, 0, 1]));
    driver.connect(Some(ThreadConfig::new(80, None)))?;

    driver
        .send_full_reset()?
        .wait_timeout(Duration::from_secs(20))?;
    driver
        .send(proto::FrcInitialize::default())?
        .wait_timeout(Duration::from_secs(20))?;
    driver
        .send(proto::FrcSetOverRide::new(80))?
        .wait_timeout(Duration::from_secs(1))?;
    driver
        .send(proto::FrcSetUTool::new(1))?
        .wait_timeout(Duration::from_secs(1))?;
    driver
        .send(proto::FrcSetUFrame::new(1))?
        .wait_timeout(Duration::from_secs(1))?;

    let movement_cmd1 = proto::FrcJointMotionJRep::new(
        proto::JointAngles::new6(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            -90.0,
            0.0,
            0.0,
            -180.0,
            90.0,
            180.0,
        ),
        proto::SpeedType::MilliSeconds,
        528,
        proto::TermType::FINE,
        0,
    );
    let movement_cmd2 = proto::FrcJointMotionJRep::new(
        proto::JointAngles::new6(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            90.0,
            0.0,
            0.0,
            180.0,
            -90.0,
            -180.0,
        ),
        proto::SpeedType::MilliSeconds,
        528,
        proto::TermType::FINE,
        0,
    );
    driver
        .send(movement_cmd1)?
        .wait_timeout(Duration::from_millis(600))?;
    driver
        .send(proto::FrcWaitTime::new(Duration::from_secs(1)))?
        .wait()?;
    driver
        .send(movement_cmd2)?
        .wait_timeout(Duration::from_millis(600))?;

    let pos_resp = driver
        .send(proto::FrcReadJointAngles::new(None))?
        .wait_timeout(Duration::from_millis(200))?;
    println!(
        "Current joint angles: {:?}",
        pos_resp
            .joints(JointFormat::AbsDeg, JointTemplate::SIX)
            .as_array()
    );

    Ok(())
}
