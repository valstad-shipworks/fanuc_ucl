import math

from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig, stmo


def main():
    driver = stmo.StreamMotionDriver("10.0.0.1")
    driver.connect(ThreadConfig(80, None))
    driver.start(2.0)

    limits = driver.fetch_movement_limits(0)
    print(f"Movement limits: {limits}")

    home = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]

    # One command per interpolation cycle (8ms): sweep J1 through a
    # 10 degree sine wave over 4 seconds.
    commands = []
    for i in range(500):
        joints = home.copy()
        joints[0] += 10.0 * math.sin(i / 500.0 * math.tau)
        commands.append(
            stmo.MotionCommandPacket.try_from_joints(
                JointFormat.FanucDeg, JointTemplate.SIX, joints,
            ),
        )
    handle = driver.command_motion(commands)
    # free to do other work here while the batch streams out
    handle.wait_timeout(10.0)

    # In-the-loop control: answer each status cycle with a fresh setpoint
    # instead of pre-computing the whole trajectory.
    with driver.control_loop() as ctl:
        for _ in range(500):
            status = ctl.wait_for_status(0.1)
            joints = list(status.joints(JointFormat.FanucDeg, JointTemplate.SIX))
            joints[5] += 0.05
            ctl.send_command(
                stmo.MotionCommandPacket.try_from_joints(
                    JointFormat.FanucDeg, JointTemplate.SIX, joints,
                ),
            )

    driver.stop()
    driver.disconnect()
