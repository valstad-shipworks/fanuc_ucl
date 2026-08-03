from enum import IntEnum
from typing import Generic, TypeAlias, TypeVar

from ..hmi import HmiDriver, HmiHandle

_V = TypeVar("_V")

AsgRange: TypeAlias = tuple[int, int] | None

class FlipState(IntEnum):
    Unflipped = 0
    Flipped = 1

class LeftRight(IntEnum):
    Right = 0
    Left = 1

class UpDown(IntEnum):
    Down = 0
    Up = 1

class FrontBack(IntEnum):
    Back = 0
    Front = 1

class CartesianData:
    x: float
    y: float
    z: float
    w: float
    p: float
    r: float
    e1: float
    e2: float
    e3: float
    flip: FlipState
    lr: LeftRight
    ud: UpDown
    fb: FrontBack
    turn4: int
    turn5: int
    turn6: int
    is_valid: bool

class JointData:
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    j7: float
    j8: float
    j9: float
    is_valid: bool

class FrameData:
    uf: int
    ut: int

class PositionData:
    cartesian: CartesianData
    joint: JointData
    frame: FrameData

class AlarmSeverity(IntEnum):
    None_ = 128
    Warning = 0
    PauseLocal = 2
    PauseGlobal = 34
    StopLocal = 6
    StopGlobal = 38
    ServoAlarm = 54
    AbortLocal = 11
    AbortGlobal = 43
    ServoAlarm2 = 58
    SystemAlarm = 123

class TimeData:
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: int

class AlarmData:
    id: int
    number: int
    cause_id: int
    cause_cnt: int
    severity: AlarmSeverity
    time: TimeData
    msg: str
    cause_msg: str
    severity_msg: str

class ProgramState(IntEnum):
    Aborted = 0
    Paused = 1
    Running = 2

class ProgramStatus:
    name: str
    line_number: int
    state: ProgramState
    parent_name: str

class BoolIoSignal(IntEnum):
    """Boolean I/O signal types available on a FANUC controller."""

    DigitalInput = 0
    DigitalOutput = 1
    RobotInput = 2
    RobotOutput = 3
    UopInput = 4
    UopOutput = 5
    SopInput = 6
    SopOutput = 7
    WeldInput = 8
    WeldOutput = 9
    WireStickInput = 10
    WireStickOutput = 11

class IntIoSignal(IntEnum):
    """Integer (group/analog) I/O signal types available on a FANUC controller."""

    GroupInput = 0
    GroupOutput = 1
    AnalogInput = 2
    AnalogOutput = 3

class AlarmSource(IntEnum):
    """Source log from which to read alarms on the controller."""

    Active = 0
    History = 1
    PasswordLog = 2
    Motion = 3
    Application = 4
    System = 5

class ProgramStatusKind(IntEnum):
    """The kind of program task to query status for."""

    Default = 0
    MacroCaller = 1
    KarelCaller = 2
    MacroOrKarel = 3

SysVarCompat: TypeAlias = int | float | str | bool | PositionData

class ReadOnlyAsgInterface(Generic[_V]):
    """A read-only interface for a registered ASG variable on the controller."""

    def read(self, driver: HmiDriver) -> HmiHandle[_V]:
        """Reads the current value of this ASG variable from the controller, returning a handle to the asynchronous response."""

class AsgInterface(ReadOnlyAsgInterface[_V], Generic[_V]):
    """A typed interface for reading and writing a registered ASG variable on the controller."""

    def read(self, driver: HmiDriver) -> HmiHandle[_V]:
        """Reads the current value of this ASG variable from the controller, returning a handle to the asynchronous response."""
    def write(self, driver: HmiDriver, value: _V) -> HmiHandle[None]:
        """Writes a value to this ASG variable on the controller, returning a handle to the asynchronous response."""
