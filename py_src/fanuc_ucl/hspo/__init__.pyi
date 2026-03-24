from ipaddress import IPv4Address, IPv6Address
from typing import Generic, TypeVar

from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig

_T = TypeVar("_T")

all = [
    "TcpCartesianPositionPacket",
    "JointAnglesPacket",
    "VariablesPacket",
    "initialize_broker",
    "destroy_broker",
    "HspoReceiver",
    "HspoChannel",
]

class TcpCartesianPositionPacket:
    """A packet from a FANUC controller containing the TCP (Tool Center Point) cartesian position."""

    version: int
    index: int
    clock: int
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float
    status: int
    io: int

class JointAnglesPacket:
    """A packet from a FANUC controller containing joint angle values."""

    version: int
    index: int
    clock: int
    motion_group: int
    status: int
    io: int

    def joints(self, format: JointFormat, template: JointTemplate) -> list[float]:
        """Returns the joint angles converted from the internal FANUC radian format to the specified format and template."""

class VariablesPacket:
    """A packet from a FANUC controller containing up to 10 user-configured variable values."""

    version: int
    index: int
    clock: int
    data: list[float]

class HspoChannel(Generic[_T]):
    """A channel for receiving HSPO packets of a specific type."""

    def wait_for(self, timeout_secs: float) -> _T | None:
        """Blocks until a packet is received or the timeout elapses."""
    def try_recv(self) -> _T | None:
        """Returns the next buffered packet without blocking, or ``None`` if the buffer is empty."""
    def recv_all(self) -> list[_T]:
        """Drains and returns all buffered packets."""
    def clear(self) -> None:
        """Discards all buffered packets."""

def initialize_broker(listen_on: str, thread_config: ThreadConfig) -> None:
    """Initializes the global HSPO broker, binding a socket to ``listen_on`` and spawning a background listener thread.

    This must be called before creating any ``HspoReceiver``. Calling it again after initialization is a no-op.
    """

def destroy_broker(wait_for_thread: bool = True) -> None:
    """Shuts down the global HSPO broker.

    If ``wait_for_thread`` is ``True``, blocks until the broker thread has fully exited.
    """

def has_broker_errored() -> bool:
    """Returns ``True`` if the HSPO broker thread encountered an error during setup.

    When this returns ``True`` the broker is likely non-functional and should
    be destroyed and re-initialized.
    """

class HspoReceiver:
    """Receives HSPO (High Speed Position Output) packets from a specific FANUC controller.

    Packets are buffered internally and can be consumed via the ``tcp``, ``joint``, and ``var`` channels.
    """

    def __init__(
        self,
        ip_of_interest: str | IPv4Address | IPv6Address,
        packet_buffer_size: int = 128,
        connection_timeout_secs: float = 0.016,
    ) -> None:
        """Creates a new receiver for the given robot IP address with the specified packet buffer size."""
    def is_connected(self) -> bool:
        """Returns True if a packet has been received from this robot recently."""
    def clock_micros(self) -> int:
        """Returns the cumulative HSPO clock in microseconds, accounting for wraps of the controller's 32-bit clock."""
    def clock_ms(self) -> float:
        """Returns the cumulative HSPO clock in milliseconds."""
    def clock_pair_micros(self) -> tuple[int, int]:
        """Returns a pair of ``(hspo_clock_micros, system_time_micros)`` for correlating controller time with system time."""
    @property
    def tcp(self) -> HspoChannel[TcpCartesianPositionPacket]:
        """Channel for TCP cartesian position packets."""
    @property
    def joint(self) -> HspoChannel[JointAnglesPacket]:
        """Channel for joint angles packets."""
    @property
    def var(self) -> HspoChannel[VariablesPacket]:
        """Channel for variables packets."""
