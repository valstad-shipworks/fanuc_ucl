from enum import IntEnum
from typing import Sequence

from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig

__all__ = [
    "AxisMotionConstraint",
    "CommandPositionResponsePacket",
    "IoType",
    "JointMovementLimit",
    "JointMovementLimits",
    "MotionCommandPacket",
    "PoseData",
    "RobotStatusPacket",
    "StatusBitfield",
    "StreamMotionDriver",
]

class PoseData:
    x: float
    y: float
    z: float
    w: float
    p: float
    r: float
    e1: float
    e2: float
    e3: float

    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        w: float,
        p: float,
        r: float,
        e1: float = 0.0,
        e2: float = 0.0,
        e3: float = 0.0,
    ) -> None: ...

class IoType(IntEnum):
    DI = 1
    DO = 2
    RI = 8
    RO = 9
    SI = 11
    SO = 12
    WI = 16
    WO = 17
    UI = 20
    UO = 21
    WSI = 26
    WSO = 27
    F = 35
    M = 36

class MotionCommandPacket:
    @staticmethod
    def try_from_joints(
        format: JointFormat,
        template: JointTemplate,
        joints: Sequence[float],
    ) -> MotionCommandPacket: ...
    def set_read_io(self, io_type: IoType, index: int, mask: int) -> None: ...
    def set_write_io(
        self,
        io_type: IoType,
        index: int,
        mask: int,
        value: int,
    ) -> None: ...
    def set_last_command(self, last: bool) -> None: ...

class StatusBitfield:
    def in_motion(self) -> bool: ...
    def ready_for_commands(self) -> bool: ...
    def command_received(self) -> bool: ...
    def sysrdy(self) -> bool: ...
    def packet_rate(self) -> int: ...

class RobotStatusPacket:
    seq: int
    status: int
    read_io_type: int
    read_io_index: int
    read_io_mask: int
    read_io_value: int
    time_stamp: int
    pose: PoseData
    """
    Length of 9
    """
    motor_current: list[float]
    """
    Length of 9
    """

    def status_bits(self) -> StatusBitfield: ...
    def joints(self, format: JointFormat) -> list[float]: ...

class CommandPositionResponsePacket:
    timestamp: int
    position: PoseData

    def joints(self, format: JointFormat) -> list[float]: ...

class AxisMotionConstraint:
    no_payload: Sequence[float]
    max_payload: Sequence[float]

    def __init__(self, no_payload: list[float], max_payload: list[float]) -> None: ...
    def calculate(
        self,
        tcp_speed: float,
        payload: float,
        vmax: float,
        max_payload: float,
    ) -> float: ...

class JointMovementLimit:
    velocity: AxisMotionConstraint
    acceleration: AxisMotionConstraint
    jerk: AxisMotionConstraint

    def __init__(
        self,
        velocity: AxisMotionConstraint,
        acceleration: AxisMotionConstraint,
        jerk: AxisMotionConstraint,
    ) -> None: ...
    def calculate(
        self,
        tcp_speed: float,
        payload: float,
        vmax: float,
        max_payload: float,
    ) -> tuple[float, float, float]: ...

class JointMovementLimits:
    joints: Sequence[JointMovementLimit]
    vmax: int

    def to_json(self) -> str: ...
    @staticmethod
    def from_json(json_str: str) -> JointMovementLimits: ...
    def __init__(self, vmax: int, joints: list[JointMovementLimit]) -> None: ...

class StreamMotionDriver:
    def __init__(self, remote_addr_string: str) -> None: ...
    def get_remote_addr(self) -> str: ...
    def refresh(self) -> None: ...
    def control_motion(
        self,
        command: MotionCommandPacket,
    ) -> None: ...
    def command_motion_batch(
        self,
        commands: Sequence[MotionCommandPacket],
    ) -> None: ...
    def connect(self, config: ThreadConfig | None = None) -> str: ...
    def disconnect(self) -> None: ...
    def start(self, timeout_secs: float = 2.0) -> str: ...
    def stop(self) -> None: ...
    def is_connected(self) -> bool: ...
    def is_started(self) -> bool: ...
    def fetch_movement_limits(
        self,
        extra_axis: int = 0,
    ) -> JointMovementLimits | None: ...
    def pull_states(self) -> list[RobotStatusPacket]: ...
    def pull_command_positions(self) -> list[CommandPositionResponsePacket]: ...
    def wait_for_command_position(
        self,
        timeout_ms: int = 200,
    ) -> CommandPositionResponsePacket | None: ...
