from fanuc_ucl import JointFormat, JointTemplate


def main():
    joints = [-90.0, 0.0, 0.0, -180.0, 90.0, 180.0]
    joints_conv = JointFormat.AbsRad.convert_from(
        JointFormat.FanucDeg,
        JointTemplate.SIX,
        joints,
    )
    print(f"Converted joints: {joints_conv}")
