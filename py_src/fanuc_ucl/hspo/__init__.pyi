from ipaddress import IPv4Address, IPv6Address

from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig

all = [
    "TcpCartesianPositionPacket",
    "JointAnglesPacket",
    "VariablesPacket",
    "initialize_broker",
    "HspoReceiver",
]

class TcpCartesianPositionPacket:
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
    version: int
    index: int
    clock: int
    motion_group: int
    status: int
    io: int

    def joints(self, format: JointFormat, template: JointTemplate) -> list[float]: ...

class VariablesPacket:
    version: int
    index: int
    clock: int
    data: list[float]

def initialize_broker(listen_on: str, thread_config: ThreadConfig) -> None: ...
def destroy_broker() -> None: ...

class HspoReceiver:
    def __init__(
        self,
        ip_of_interest: str | IPv4Address | IPv6Address,
        packet_buffer_size: int = 128,
    ) -> None: ...
    def is_connected(self) -> bool: ...
    def clock_micros(self) -> int: ...
    def clock_ms(self) -> float: ...
    def clock_pair_micros(self) -> tuple[int, int]: ...
    def wait_for_tcp_packet(
        self, timeout_secs: float
    ) -> TcpCartesianPositionPacket: ...
    def try_recv_tcp_packet(self) -> TcpCartesianPositionPacket | None: ...
    def recv_all_tcp_packets(self) -> list[TcpCartesianPositionPacket]: ...
    def clear_tcp_packet_buffer(self) -> None: ...
    def wait_for_joint_packet(self, timeout_secs: float) -> JointAnglesPacket: ...
    def try_recv_joint_packet(self) -> JointAnglesPacket | None: ...
    def recv_all_joint_packets(self) -> list[JointAnglesPacket]: ...
    def clear_joint_packet_buffer(self) -> None: ...
    def wait_for_var_packet(self, timeout_secs: float) -> VariablesPacket: ...
    def try_recv_var_packet(self) -> VariablesPacket | None: ...
    def recv_all_var_packets(self) -> list[VariablesPacket]: ...
    def clear_var_packet_buffer(self) -> None: ...
