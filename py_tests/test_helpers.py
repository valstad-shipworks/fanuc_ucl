"""Pure helpers exposed to Python: joint format conversions and the value
types the drivers take and return."""

import itertools
import math
import random

import pytest
from fanuc_ucl import JointFormat, JointTemplate, JointType, rmi, stmo

FORMATS = [
    JointFormat.AbsRad,
    JointFormat.FanucRad,
    JointFormat.AbsDeg,
    JointFormat.FanucDeg,
]
TEMPLATES = {
    "SIX": (JointTemplate.SIX, 6),
    "SIX_LINEAR_TRACK": (JointTemplate.SIX_LINEAR_TRACK, 7),
    "FIVE_LINEAR_TRACK": (JointTemplate.FIVE_LINEAR_TRACK, 6),
    "FOUR_LINEAR_TRACK": (JointTemplate.FOUR_LINEAR_TRACK, 5),
}


def joints(rng: random.Random, n: int) -> list[float]:
    return [rng.uniform(-400.0, 400.0) for _ in range(n)] + [0.0] * (9 - n)


@pytest.mark.parametrize("name", TEMPLATES)
@pytest.mark.parametrize("src, dst", list(itertools.permutations(FORMATS, 2)))
def test_conversions_roundtrip(name, src, dst):
    template, n = TEMPLATES[name]
    rng = random.Random(f"{name}{src}{dst}")
    for _ in range(50):
        original = joints(rng, n)
        there = dst.convert_from(src, template, original)
        back = src.convert_from(dst, template, there)
        assert back == pytest.approx(original, rel=1e-12, abs=1e-9)


def test_fanuc_j3_is_relative_to_j2():
    absolute = [10.0, 20.0, 50.0, 1.0, 2.0, 3.0]
    fanuc = JointFormat.FanucDeg.convert_from(
        JointFormat.AbsDeg, JointTemplate.SIX, absolute
    )
    assert fanuc[:6] == [10.0, 20.0, 30.0, 1.0, 2.0, 3.0]


def test_degrees_to_radians_skips_linear_axes():
    deg = [180.0, 90.0, 45.0, 0.0, -90.0, 360.0, 1500.0]
    rad = JointFormat.AbsRad.convert_from(
        JointFormat.AbsDeg, JointTemplate.SIX_LINEAR_TRACK, deg
    )
    assert rad[:6] == pytest.approx(
        [math.pi, math.pi / 2, math.pi / 4, 0.0, -math.pi / 2, 2 * math.pi]
    )
    assert rad[6] == 1500.0


def test_a_custom_template_converts_by_axis_type():
    template = JointTemplate(
        [JointType.Rotary] * 6 + [JointType.Linear, JointType.Rotary]
    )
    deg = [90.0] * 8
    rad = JointFormat.AbsRad.convert_from(JointFormat.AbsDeg, template, deg)
    assert rad[6] == 90.0
    assert rad[7] == pytest.approx(math.pi / 2)


def test_too_few_joints_is_a_value_error():
    with pytest.raises(ValueError, match="at least 6"):
        JointFormat.FanucDeg.convert_from(
            JointFormat.AbsDeg, JointTemplate.SIX, [1.0, 2.0]
        )


@pytest.mark.parametrize("fmt", FORMATS)
def test_rmi_joint_angles_store_fanuc_degrees(fmt):
    rng = random.Random(str(fmt))
    for _ in range(20):
        given = joints(rng, 6)[:6]
        ja = rmi.JointAngles(fmt, JointTemplate.SIX, *given)
        want = JointFormat.FanucDeg.convert_from(fmt, JointTemplate.SIX, given)
        assert ja.as_array()[:6] == pytest.approx(want[:6], rel=1e-6, abs=1e-4)
        assert [ja.j1, ja.j2, ja.j3, ja.j4, ja.j5, ja.j6] == pytest.approx(
            want[:6], rel=1e-6, abs=1e-4
        )


def test_pose_data_keeps_its_fields():
    pose = stmo.PoseData(1.5, -2.5, 3.25, 10.0, 20.0, 30.0, e1=7.0)
    assert (pose.x, pose.y, pose.z, pose.w, pose.p, pose.r) == (
        1.5,
        -2.5,
        3.25,
        10.0,
        20.0,
        30.0,
    )
    assert (pose.e1, pose.e2, pose.e3) == (7.0, 0.0, 0.0)
    pose.e3 = 4.0
    assert pose.e3 == 4.0


def test_motion_command_packets_build_from_joints_and_poses():
    cmd = stmo.MotionCommandPacket.try_from_joints(
        JointFormat.AbsDeg, JointTemplate.SIX, [0.0, 10.0, 20.0, 0.0, 0.0, 0.0]
    )
    cmd.set_read_io(stmo.IoType.DI, 1, 0xFFFF)
    cmd.set_write_io(stmo.IoType.DO, 2, 0x00FF, 0x0001)
    cmd.set_last_command(True)
    stmo.MotionCommandPacket.from_pose(stmo.PoseData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


def limits(rng: random.Random) -> stmo.JointMovementLimits:
    def axis():
        return stmo.AxisMotionConstraint(
            [rng.uniform(0, 100) for _ in range(20)],
            [rng.uniform(0, 100) for _ in range(20)],
        )

    return stmo.JointMovementLimits(
        rng.randrange(1, 4000),
        [stmo.JointMovementLimit(axis(), axis(), axis()) for _ in range(6)],
    )


def test_movement_limits_roundtrip_through_json():
    rng = random.Random(7)
    for _ in range(10):
        original = limits(rng)
        again = stmo.JointMovementLimits.from_json(original.as_json())
        assert again.as_json() == original.as_json()
        assert again.vmax == original.vmax
        assert len(again.joints) == 6
