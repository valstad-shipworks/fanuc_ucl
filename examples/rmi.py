from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig, rmi

def main():
    driver = rmi.RmiDriver(rmi.RmiDriverConfig("10.0.0.1"))
    driver.connect(ThreadConfig(80, None))

    driver.send_full_reset().wait_timeout(20.0)
    driver.send(rmi.FrcInitialize()).wait_timeout(20.0)
    driver.send(rmi.FrcSetOverRide(80)).wait_timeout(1.0)
    driver.send(rmi.FrcSetUTool(1)).wait_timeout(1.0)
    driver.send(rmi.FrcSetUFrame(1)).wait_timeout(1.0)

    movement_cmd1 = rmi.FrcJointMotionJRep(
        rmi.JointAngles(
            JointFormat.AbsDeg,
            JointTemplate.SIX,
            *[-90.0, 0.0, 0.0, -180.0, 90.0, 180.0]
        ),
        rmi.SpeedType.MilliSeconds,
        528,
        rmi.TermType.FINE,
        0,
    )
    movement_cmd2 = rmi.FrcJointMotionJRep(
        rmi.JointAngles(
            JointFormat.AbsDeg,
            JointTemplate.SIX,
            *[90.0, 0.0, 0.0, 180.0, -90.0, -180.0]
        ),
        rmi.SpeedType.MilliSeconds,
        528,
        rmi.TermType.FINE,
        0,
    )
    driver.send(movement_cmd1).wait_timeout(0.6)
    driver.send(rmi.FrcWaitTime(1.0)).wait()
    driver.send(movement_cmd2).wait_timeout(0.6)

    pos_resp = driver.send(rmi.FrcReadJointAngles()).wait_timeout(0.2)
    print(f"Current joint angles: {pos_resp.joints(JointFormat.AbsDeg, JointTemplate.SIX).as_array()}")
