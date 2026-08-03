from enum import Enum

from . import hspo, rmi, stmo
from ._common import JointFormat, JointTemplate, JointType, ThreadConfig

__all__ = [
    "JointFormat",
    "JointTemplate",
    "JointType",
    "LogLevel",
    "ThreadConfig",
    "hspo",
    "rmi",
    "set_log_level",
    "stmo",
]

class LogLevel(Enum):
    """Runtime log verbosity for the fanuc_ucl Rust core. Used with :func:`set_log_level`."""

    Off = 0
    Error = 1
    Warn = 2
    Info = 3
    Debug = 4
    Trace = 5

def set_log_level(level: LogLevel) -> LogLevel:
    """Raise the Rust core's log verbosity if `level` is more verbose than the current setting.

    The effective level is ``max(current, level)`` — calling this never lowers
    the level below what the ``RUST_LOG`` environment variable established at
    module import. Returns the effective level after the call.
    """
