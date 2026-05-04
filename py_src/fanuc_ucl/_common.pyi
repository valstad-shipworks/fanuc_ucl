"""Shared type declarations imported by both `fanuc_ucl/__init__.pyi` and the
submodule stubs (`rmi/proto.pyi`, `hspo/__init__.pyi`, etc.). Lives here so
the submodules don't have to reach back into `fanuc_ucl/__init__.pyi`, which
would create import cycles that thrash pyright's incremental cache.

Re-exported from `fanuc_ucl` for users — import the public names from there.
"""

from enum import Enum

__all__ = [
    "JointFormat",
    "JointTemplate",
    "JointType",
    "ThreadConfig",
]

class ThreadConfig:
    """Configuration for thread scheduling and CPU affinity."""

    def __init__(self, priority: int = 0, cpu_affinity: int | None = None) -> None:
        self.priority = priority
        self.cpu_affinity = cpu_affinity

    def configure_this_thread(self) -> None: ...

class JointType(Enum):
    """The unit quantity for a joint and how to handle conversions between different formats."""

    Linear = "Linear"
    Rotary = "Rotary"

class JointTemplate:
    """Represents the units/joint types of each joint in a robot, used to convert between different formats and units.

    Premade templates:
        - ``SIX``: A 6-axis robot with all rotary joints.
        - ``SIX_LINEAR_TRACK``: A 7-axis robot with 6 rotary joints and a linear track.
        - ``FOUR``: A 4-axis robot with all rotary joints.
        - ``FOUR_LINEAR_TRACK``: A 5-axis robot with 4 rotary joints and a linear track.
        - ``FIVE``: A 5-axis robot with all rotary joints.
        - ``FIVE_LINEAR_TRACK``: A 6-axis robot with 5 rotary joints and a linear track.
    """

    def __init__(self, axis: list[JointType]) -> None: ...
    SIX: JointTemplate
    SIX_LINEAR_TRACK: JointTemplate
    FOUR: JointTemplate
    FOUR_LINEAR_TRACK: JointTemplate
    FIVE: JointTemplate
    FIVE_LINEAR_TRACK: JointTemplate

class JointFormat(Enum):
    """The format of joint data, used to convert between different formats and units.

    This enum is often passed into APIs in this library to specify the desired output format,
    but ``convert_from`` can also be used directly to convert your own data.
    """

    AbsRad = 1
    FanucRad = 2
    AbsDeg = 3
    FanucDeg = 4

    def convert_from(
        self,
        format: JointFormat,
        template: JointTemplate,
        joints: list[float],
    ) -> list[float]:
        """Converts joint data from ``format`` into ``self``, using the given template to determine per-joint conversions."""
