from ipaddress import IPv4Address, IPv6Address
from typing import Generic, Literal, Protocol, Sequence, TypeVar, overload

from fanuc_ucl import ThreadConfig

from .asg import *

__all__ = [
    "DigitalInput",
    "DigitalOutput",
    "RobotInput",
    "RobotOutput",
    "UopInput",
    "UopOutput",
    "SopInput",
    "SopOutput",
    "WeldInput",
    "WeldOutput",
    "WireStickInput",
    "WireStickOutput",
    "GroupInput",
    "GroupOutput",
    "AnalogInput",
    "AnalogOutput",
    "Register",
    "Command",
    "HmiDriver",
]

_T_co = TypeVar("_T_co", covariant=True)
_T_contra = TypeVar("_T_contra", contravariant=True)
_T = TypeVar("_T")
_H = TypeVar("_H")
_U = TypeVar("_U", bound=SysVarCompat)

class ReadablePort(Protocol[_T_co]):
    @staticmethod
    def __CAN_READ__() -> None: ...

class WriteablePort(Protocol[_T_contra]):
    @staticmethod
    def __CAN_WRITE__() -> None: ...
    @staticmethod
    def __USE__(_unused: _T_contra) -> None: ...

class UnsafelyWriteablePort(Protocol[_T_contra]):
    @staticmethod
    def __CAN_WRITE_UNSAFE__() -> None: ...
    @staticmethod
    def __USE__(_unused: _T_contra) -> None: ...

# The key change: inherit from the capability protocols.
class __READONLY__(ReadablePort[_T_co], Protocol[_T_co]):
    @staticmethod
    def __CAN_READ__() -> None: ...

class __WRITEONLY__(
    WriteablePort[_T_contra], UnsafelyWriteablePort[_T_contra], Protocol[_T_contra]
):
    @staticmethod
    def __CAN_WRITE__() -> None: ...
    @staticmethod
    def __CAN_WRITE_UNSAFE__() -> None: ...
    @staticmethod
    def __USE__(_unused: _T_contra) -> None: ...

class __READWRITE__(
    ReadablePort[_T_co],
    WriteablePort[_T_contra],
    UnsafelyWriteablePort[_T_contra],
    Protocol[_T_co, _T_contra],
):
    @staticmethod
    def __CAN_READ__() -> None: ...
    @staticmethod
    def __CAN_WRITE__() -> None: ...
    @staticmethod
    def __CAN_WRITE_UNSAFE__() -> None: ...
    @staticmethod
    def __USE__(_unused: _T_contra) -> None: ...

class __READUNSAFEWRITE__(
    ReadablePort[_T_co], UnsafelyWriteablePort[_T_contra], Protocol[_T_co, _T_contra]
):
    @staticmethod
    def __CAN_READ__() -> None: ...
    @staticmethod
    def __CAN_WRITE_UNSAFE__() -> None: ...
    @staticmethod
    def __USE__(_unused: _T_contra) -> None: ...

DigitalInput = __READUNSAFEWRITE__[bool, bool]
DigitalOutput = __READWRITE__[bool, bool]
RobotInput = __READUNSAFEWRITE__[bool, bool]
RobotOutput = __READWRITE__[bool, bool]
UopInput = __READUNSAFEWRITE__[bool, bool]
UopOutput = __READWRITE__[bool, bool]
SopInput = __READUNSAFEWRITE__[bool, bool]
SopOutput = __READWRITE__[bool, bool]
WeldInput = __READUNSAFEWRITE__[bool, bool]
WeldOutput = __READWRITE__[bool, bool]
WireStickInput = __READUNSAFEWRITE__[bool, bool]
WireStickOutput = __READWRITE__[bool, bool]
GroupInput = __READUNSAFEWRITE__[int, int]
GroupOutput = __READWRITE__[int, int]
AnalogInput = __READUNSAFEWRITE__[int, int]
AnalogOutput = __READWRITE__[int, int]
Register = __READWRITE__[int, int]
Command = __WRITEONLY__[str]

class HmiHandle(Generic[_H]):
    """A typed handle to a pending HMI response that decodes the raw message into the target type."""

    def is_set(self) -> bool:
        """Returns True if the response (or an error) has been received."""
        ...
    def get(self) -> _H:
        """Returns the response value if available, or raises if the request failed or the response has not yet arrived."""
        ...
    def wait_timeout(self, timeout_secs: float) -> _H:
        """Blocks until the response arrives or the timeout elapses, raising TimeoutError on expiry."""
        ...
    def wait(self) -> _H:
        """Blocks indefinitely until the response arrives."""
        ...

class HmiDriver:
    """The main driver for interfacing with a FANUC robot via SNPX HMI.

    Manages the connection to the HMI, sending commands, reading/writing data ports, and registering ASG variables.
    """

    def __init__(self, address: str | IPv4Address | IPv6Address) -> None:
        """Creates a new HmiDriver instance. Does not connect; call ``connect()`` to establish a connection."""
        ...
    def connect(
        self, timeout_secs: float = 1.0, thread_config: ThreadConfig | None = None
    ) -> None:
        """Connects to the HMI and performs the necessary handshake to establish communication."""
        ...
    def disconnect(self) -> None:
        """Disconnects from the HMI, shutting down the runner thread and cleaning up resources."""
        ...
    def clear_alarms(self) -> HmiHandle[None]:
        """Sends a command to clear all active alarms on the controller."""
        ...
    def write(
        self, port: WriteablePort[_T], index: int, value: _T | Sequence[_T]
    ) -> HmiHandle[None]:
        """Writes a value to a writable data port at the given index, returning a handle to the asynchronous response."""
        ...
    def write_unsafe(
        self, port: UnsafelyWriteablePort[_T], index: int, value: _T | Sequence[_T]
    ) -> HmiHandle[None]:
        """Writes a value to a data port, bypassing write-safety checks.

        This allows writing to read-only ports which is technically possible but not advised.
        """
        ...
    @overload
    def read(self, port: ReadablePort[_T], index: int) -> HmiHandle[_T]: ...
    @overload
    def read(
        self, port: ReadablePort[_T], index: int, count: int
    ) -> HmiHandle[list[_T]]: ...
    @overload
    def register_asg(
        self,
        tag: type[PositionData],
        current: Literal[False],
        *,
        index: int,
        group: int | None = None,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[PositionData]: ...
    @overload
    def register_asg(
        self,
        tag: type[PositionData],
        current: Literal[True],
        *,
        frame: int,
        group: int | None = None,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> ReadOnlyAsgInterface[PositionData]: ...
    @overload
    def register_asg(
        self,
        tag: BoolIoSignal,
        *,
        index: int,
        simulation: bool,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[bool]: ...
    @overload
    def register_asg(
        self,
        tag: IntIoSignal,
        *,
        index: int,
        simulation: bool,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[int]: ...
    @overload
    def register_asg(
        self,
        tag: type[AlarmData],
        *,
        source: AlarmSource,
        line: int,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> ReadOnlyAsgInterface[AlarmData]: ...
    @overload
    def register_asg(
        self,
        tag: type[ProgramStatus],
        *,
        task: int,
        kind: ProgramStatusKind,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> ReadOnlyAsgInterface[ProgramStatus]: ...
    @overload
    def register_asg(
        self,
        tag: type[str],
        *,
        index: int,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[str]: ...
    @overload
    def register_asg(
        self,
        tag: type[float],
        *,
        index: int,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[float]: ...
    @overload
    def register_asg_array(
        self,
        tag: type[PositionData],
        count: int,
        *,
        index: int,
        group: int | None = None,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[list[PositionData]]: ...
    @overload
    def register_asg_array(
        self,
        tag: BoolIoSignal,
        count: int,
        *,
        index: int,
        simulation: bool,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[list[bool]]: ...
    @overload
    def register_asg_array(
        self,
        tag: IntIoSignal,
        count: int,
        *,
        index: int,
        simulation: bool,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[list[int]]: ...
    @overload
    def register_asg_array(
        self,
        tag: type[AlarmData],
        count: int,
        *,
        source: AlarmSource,
        line: int,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> ReadOnlyAsgInterface[list[AlarmData]]: ...
    @overload
    def register_asg_array(
        self,
        tag: type[ProgramStatus],
        count: int,
        *,
        task: int,
        kind: ProgramStatusKind,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> ReadOnlyAsgInterface[list[ProgramStatus]]: ...
    @overload
    def register_asg_array(
        self,
        tag: type[str],
        count: int,
        *,
        index: int,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[list[str]]: ...
    @overload
    def register_asg_array(
        self,
        tag: type[float],
        count: int,
        *,
        index: int,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[list[float]]: ...
    def register_sysvar_asg(
        self,
        tag: type[_U],
        *,
        name: str,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[_U]: ...
    def register_sysvar_asg_array(
        self,
        tag: type[_U],
        count: int,
        *,
        name: str,
        range: AsgRange = None,
        timeout_secs: float = 0.016,
    ) -> AsgInterface[list[_U]]: ...
