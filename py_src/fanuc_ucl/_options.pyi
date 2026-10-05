"""Types for the real-time thread and socket options the drivers accept,
as defined by fast-talker.

Names are listed in their canonical snake_case form. At runtime any case
and separators work (`"RtPriority"`, `"RT_PRIORITY"`, `"rt-priority"`),
`"type"` works in place of `"kind"`, and any object with `kind`/`type` and
`value` attributes (dataclass, SimpleNamespace, attrs or pydantic model) is
read like the matching mapping. No class from any module is required.
"""

from collections.abc import Iterable, Mapping
from enum import Enum
from typing import (
    Any,
    Literal,
    NotRequired,
    Protocol,
    SupportsIndex,
    TypeAlias,
    TypedDict,
    type_check_only,
)

Int: TypeAlias = int | SupportsIndex
"""Any int or object with `__index__`. Out-of-range values raise ValueError."""

Cpus: TypeAlias = Int | Iterable[Int]
"""One CPU index or several."""

SchedulerName = Literal["other", "batch", "idle", "fifo", "round_robin"]
ThreadPriorityName = Literal[
    "idle", "lowest", "below_normal", "normal", "above_normal", "highest", "time_critical"
]
QosClassName = Literal["user_interactive", "user_initiated", "default", "utility", "background"]

ThreadOptionName = Literal[
    "cpu_affinity",
    "rt_priority",
    "prefault_stack",
    "unix_scheduler",
    "linux_nice",
    "win_priority",
    "win_disable_power_throttling",
    "win_mmcss",
    "macos_qos",
    "macos_time_constraint",
]
SocketOptionName = Literal[
    "recv_buffer",
    "send_buffer",
    "bind_device",
    "dont_fragment",
    "dscp",
    "linux_priority",
    "linux_busy_poll",
    "linux_prefer_busy_poll",
    "linux_busy_poll_budget",
    "win_cpu_affinity",
]

@type_check_only
class Tagged(Protocol):
    """An object read by its `kind` and `value` attributes."""

    @property
    def kind(self) -> str | Enum: ...
    @property
    def value(self) -> Any: ...

@type_check_only
class TaggedByType(Protocol):
    """An object read by its `type` and `value` attributes."""

    @property
    def type(self) -> str | Enum: ...
    @property
    def value(self) -> Any: ...

@type_check_only
class KindOnly(Protocol):
    """An object with just `kind`: an option without a value, or one with
    fields as further attributes."""

    @property
    def kind(self) -> str | Enum: ...

AnyTagged: TypeAlias = Tagged | TaggedByType | KindOnly | Enum

ThreadPriorityLike: TypeAlias = ThreadPriorityName | str | Enum
QosClassLike: TypeAlias = QosClassName | str | Enum

class SchedulerDict(TypedDict):
    """Canonical form of a scheduler; `value` only for fifo and round_robin."""

    kind: SchedulerName
    value: NotRequired[int]

SchedulerLike: TypeAlias = (
    Literal["other", "batch", "idle"]
    | tuple[Literal["fifo", "round_robin"], Int]
    | tuple[Literal["other", "batch", "idle"]]
    | SchedulerDict
    | Mapping[str, Any]
    | AnyTagged
)
"""`"other"`, `("fifo", 80)`, `{"fifo": 80}`,
`{"kind": "round_robin", "value": 10}`, or an Enum member."""

class TimeConstraintFields(TypedDict):
    """Mach time-constraint scheduling, in microseconds."""

    period_us: int
    computation_us: int
    constraint_us: int

class CpuAffinityDict(TypedDict):
    kind: Literal["cpu_affinity"]
    value: list[int]

class RtPriorityDict(TypedDict):
    kind: Literal["rt_priority"]
    value: int

class PrefaultStackDict(TypedDict):
    kind: Literal["prefault_stack"]
    value: int

class UnixSchedulerDict(TypedDict):
    kind: Literal["unix_scheduler"]
    value: SchedulerDict

class LinuxNiceDict(TypedDict):
    kind: Literal["linux_nice"]
    value: int

class WinThreadPriorityDict(TypedDict):
    kind: Literal["win_priority"]
    value: ThreadPriorityName

class WinDisablePowerThrottlingDict(TypedDict):
    kind: Literal["win_disable_power_throttling"]

class WinMmcssDict(TypedDict):
    kind: Literal["win_mmcss"]
    value: str

class MacOsQosDict(TypedDict):
    kind: Literal["macos_qos"]
    value: QosClassName

class MacOsTimeConstraintDict(TypedDict):
    kind: Literal["macos_time_constraint"]
    period_us: int
    computation_us: int
    constraint_us: int

ThreadOptionDict: TypeAlias = (
    CpuAffinityDict
    | RtPriorityDict
    | PrefaultStackDict
    | UnixSchedulerDict
    | LinuxNiceDict
    | WinThreadPriorityDict
    | WinDisablePowerThrottlingDict
    | WinMmcssDict
    | MacOsQosDict
    | MacOsTimeConstraintDict
)
"""Canonical form of a thread option, as returned by extension modules."""

ThreadOptionPair: TypeAlias = (
    tuple[Literal["cpu_affinity"], Cpus]
    | tuple[Literal["rt_priority", "prefault_stack", "linux_nice"], Int]
    | tuple[Literal["unix_scheduler"], SchedulerLike]
    | tuple[Literal["win_priority"], ThreadPriorityLike]
    | tuple[Literal["win_mmcss"], str]
    | tuple[Literal["macos_qos"], QosClassLike]
    | tuple[Literal["macos_time_constraint"], TimeConstraintFields | tuple[int, int, int]]
    | tuple[Literal["win_disable_power_throttling"]]
)

ThreadOptionLike: TypeAlias = (
    Literal["win_disable_power_throttling"]
    | ThreadOptionPair
    | ThreadOptionDict
    | Mapping[str, Any]
    | AnyTagged
)
"""One thread option: `("rt_priority", 80)`, `{"cpu_affinity": [3]}`,
`{"kind": "macos_qos", "value": "user_interactive"}`,
`"win_disable_power_throttling"`, an object with `kind`/`value`, or an Enum
member named after the option (its value is the option's value)."""

@type_check_only
class ThreadConfigLike(Protocol):
    """The older per-crate `ThreadConfig`: `priority < 1` means normal
    scheduling at nice -8, otherwise SCHED_FIFO at that priority; and an
    optional CPU to pin to."""

    @property
    def priority(self) -> int: ...
    @property
    def cpu_affinity(self) -> int | None: ...

class ThreadConfigDict(TypedDict, total=False):
    priority: int
    cpu_affinity: int | None

ThreadOptionsLike: TypeAlias = (
    None
    | ThreadOptionLike
    | Iterable[ThreadOptionLike]
    | Mapping[str, Any]
    | ThreadConfigLike
    | ThreadConfigDict
)
"""`None` for no options, one option, a list (or any iterable) of options,
a `{name: value, ...}` mapping of several, or a ThreadConfig-like object or
mapping."""

class SocketIntDict(TypedDict):
    kind: Literal[
        "recv_buffer",
        "send_buffer",
        "dscp",
        "linux_priority",
        "linux_busy_poll",
        "linux_busy_poll_budget",
        "win_cpu_affinity",
    ]
    value: int

class SocketBoolDict(TypedDict):
    kind: Literal["dont_fragment", "linux_prefer_busy_poll"]
    value: bool

class BindDeviceDict(TypedDict):
    kind: Literal["bind_device"]
    value: str

SocketOptionDict: TypeAlias = SocketIntDict | SocketBoolDict | BindDeviceDict
"""Canonical form of a socket option. `linux_busy_poll` is in microseconds."""

SocketOptionPair: TypeAlias = (
    tuple[
        Literal[
            "recv_buffer",
            "send_buffer",
            "dscp",
            "linux_priority",
            "linux_busy_poll",
            "linux_busy_poll_budget",
            "win_cpu_affinity",
        ],
        Int,
    ]
    | tuple[Literal["dont_fragment", "linux_prefer_busy_poll"], bool]
    | tuple[Literal["bind_device"], str]
)

SocketOptionLike: TypeAlias = SocketOptionPair | SocketOptionDict | Mapping[str, Any] | AnyTagged

SocketOptionsLike: TypeAlias = None | SocketOptionLike | Iterable[SocketOptionLike] | Mapping[str, Any]
