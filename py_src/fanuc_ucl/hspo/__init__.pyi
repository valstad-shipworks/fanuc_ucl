from ipaddress import IPv4Address, IPv6Address

from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig

all = [
    "TcpCartesianPositionPacket",
    "JointAnglesPacket",
    "VariablesPacket",
    "initialize_broker",
    "destroy_broker",
    "HspoReceiver",
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
        ...

class VariablesPacket:
    """A packet from a FANUC controller containing up to 10 user-configured variable values."""

    version: int
    index: int
    clock: int
    data: list[float]

def initialize_broker(listen_on: str, thread_config: ThreadConfig) -> None:
    """Initializes the global HSPO broker, binding a socket to ``listen_on`` and spawning a background listener thread.

    This must be called before creating any ``HspoReceiver``. Calling it again after initialization is a no-op.
    """
    ...
def destroy_broker() -> None:
    """Shuts down the global HSPO broker and joins its background thread."""
    ...

class HspoReceiver:
    """Receives HSPO (High Speed Position Output) packets from a specific FANUC controller.

    Packets are buffered internally and can be consumed with blocking or non-blocking methods.
    """

    def __init__(
        self,
        ip_of_interest: str | IPv4Address | IPv6Address,
        packet_buffer_size: int = 128,
    ) -> None:
        """Creates a new receiver for the given robot IP address with the specified packet buffer size."""
        ...
    def is_connected(self) -> bool:
        """Returns True if a packet has been received from this robot recently."""
        ...
    def clock_micros(self) -> int:
        """Returns the cumulative HSPO clock in microseconds, accounting for wraps of the controller's 32-bit clock."""
        ...
    def clock_ms(self) -> float:
        """Returns the cumulative HSPO clock in milliseconds."""
        ...
    def clock_pair_micros(self) -> tuple[int, int]:
        """Returns a pair of ``(hspo_clock_micros, system_time_micros)`` for correlating controller time with system time."""
        ...
    def wait_for_tcp_packet(
        self, timeout_secs: float
    ) -> TcpCartesianPositionPacket:
        """Blocks until a TCP cartesian position packet is received or the timeout elapses."""
        ...
    def try_recv_tcp_packet(self) -> TcpCartesianPositionPacket | None:
        """Returns the next buffered TCP cartesian position packet without blocking, or ``None`` if the buffer is empty."""
        ...
    def recv_all_tcp_packets(self) -> list[TcpCartesianPositionPacket]:
        """Drains and returns all buffered TCP cartesian position packets."""
        ...
    def clear_tcp_packet_buffer(self) -> None:
        """Discards all buffered TCP cartesian position packets."""
        ...
    def wait_for_joint_packet(self, timeout_secs: float) -> JointAnglesPacket:
        """Blocks until a joint angles packet is received or the timeout elapses."""
        ...
    def try_recv_joint_packet(self) -> JointAnglesPacket | None:
        """Returns the next buffered joint angles packet without blocking, or ``None`` if the buffer is empty."""
        ...
    def recv_all_joint_packets(self) -> list[JointAnglesPacket]:
        """Drains and returns all buffered joint angles packets."""
        ...
    def clear_joint_packet_buffer(self) -> None:
        """Discards all buffered joint angles packets."""
        ...
    def wait_for_var_packet(self, timeout_secs: float) -> VariablesPacket:
        """Blocks until a variables packet is received or the timeout elapses."""
        ...
    def try_recv_var_packet(self) -> VariablesPacket | None:
        """Returns the next buffered variables packet without blocking, or ``None`` if the buffer is empty."""
        ...
    def recv_all_var_packets(self) -> list[VariablesPacket]:
        """Drains and returns all buffered variables packets."""
        ...
    def clear_var_packet_buffer(self) -> None:
        """Discards all buffered variables packets."""
        ...
