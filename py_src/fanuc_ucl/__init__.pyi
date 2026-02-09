from enum import Enum

from . import hspo, rmi, stmo

__all__ = [
    "stmo",
    "rmi",
    "hspo",
    "ThreadConfig",
    "JointFormat",
]

class ThreadConfig:
    def __init__(self, priority: int = 0, cpu_affinity: int | None = None) -> None:
        self.priority = priority
        self.cpu_affinity = cpu_affinity

    def configure_this_thread(self) -> None:
        ...

class JointType(Enum):
    Linear = "Linear"
    Rotary = "Rotary"

class JointTemplate:
    def __init__(self, axis: list[JointType]) -> None: ...
    SIX: JointTemplate
    SIX_LINEAR_TRACK: JointTemplate
    FOUR: JointTemplate
    FOUR_LINEAR_TRACK: JointTemplate
    FIVE: JointTemplate
    FIVE_LINEAR_TRACK: JointTemplate

class JointFormat(Enum):
    AbsRad = 1
    FanucRad = 2
    AbsDeg = 3
    FanucDeg = 4

    def convert_from(self, format: JointFormat, template: JointTemplate, joints: list[float]) -> list[float]: ...

class LoggingLevel(Enum):
    Err = "ERR"
    Warn = "WARN"
    Info = "INFO"
    Debug = "DEBUG"
    Trace = "TRACE"

def config_logging(level: LoggingLevel) -> None:
    """
    Configure logging level for the Fanuc driver.

    :param LoggingLevel level: The desired logging level.
    """
    ...
