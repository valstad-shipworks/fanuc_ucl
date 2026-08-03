from enum import Enum
from typing import Generic, TypeVar, overload

from fanuc_ucl._common import ThreadConfig

# Explicit imports (no wildcard) to avoid the per-symbol provenance hops
# that pyright/pylance pays on every overload type-check. Cuts re-resolution
# cost on hot edits in files using `RmiDriver.send`.
from .proto import (
    Command,
    CommandResponse,
    Communication,
    CommunicationResponse,
    FrcAbort,
    FrcAbortResponse,
    FrcCall,
    FrcCallResponse,
    FrcCircularMotion,
    FrcCircularMotionResponse,
    FrcCircularRelative,
    FrcCircularRelativeResponse,
    FrcConnect,
    FrcConnectResponse,
    FrcContinue,
    FrcContinueResponse,
    FrcDisconnect,
    FrcDisconnectResponse,
    FrcGetStatus,
    FrcGetStatusResponse,
    FrcGetUFrameUTool,
    FrcGetUFrameUToolResponse,
    FrcInitialize,
    FrcInitializeResponse,
    FrcJointMotion,
    FrcJointMotionJRep,
    FrcJointMotionJRepResponse,
    FrcJointMotionResponse,
    FrcJointRelative,
    FrcJointRelativeJRep,
    FrcJointRelativeJRepResponse,
    FrcJointRelativeResponse,
    FrcLinearMotion,
    FrcLinearMotionJRep,
    FrcLinearMotionJRepResponse,
    FrcLinearMotionResponse,
    FrcLinearRelative,
    FrcLinearRelativeJRep,
    FrcLinearRelativeJRepResponse,
    FrcLinearRelativeResponse,
    FrcPause,
    FrcPauseResponse,
    FrcReadCartesianPosition,
    FrcReadCartesianPositionResponse,
    FrcReadDIN,
    FrcReadDINResponse,
    FrcReadError,
    FrcReadErrorResponse,
    FrcReadJointAngles,
    FrcReadJointAnglesResponse,
    FrcReadPositionRegister,
    FrcReadPositionRegisterResponse,
    FrcReadTCPSpeed,
    FrcReadTCPSpeedResponse,
    FrcReadUFrameData,
    FrcReadUFrameDataResponse,
    FrcReadUToolData,
    FrcReadUToolDataResponse,
    FrcReset,
    FrcResetResponse,
    FrcSetOverRide,
    FrcSetOverRideResponse,
    FrcSetPayLoad,
    FrcSetPayLoadResponse,
    FrcSetUFrame,
    FrcSetUFrameResponse,
    FrcSetUFrameUTool,
    FrcSetUFrameUToolResponse,
    FrcSetUTool,
    FrcSetUToolResponse,
    FrcSystemFaultResponse,
    FrcTerminateResponse,
    FrcWaitDIN,
    FrcWaitDINResponse,
    FrcWaitTime,
    FrcWaitTimeResponse,
    FrcWriteDOUT,
    FrcWriteDOUTResponse,
    FrcWritePositionRegister,
    FrcWritePositionRegisterResponse,
    FrcWriteUFrameData,
    FrcWriteUFrameDataResponse,
    FrcWriteUToolData,
    FrcWriteUToolDataResponse,
    Instruction,
    InstructionResponse,
    ResponsePacket,
    SendPacket,
)

__all__ = [
    "Command",
    "CommandResponse",
    "Communication",
    "CommunicationResponse",
    "FrcAbort",
    "FrcAbortResponse",
    "FrcCall",
    "FrcCallResponse",
    "FrcCircularMotion",
    "FrcCircularMotionResponse",
    "FrcCircularRelative",
    "FrcCircularRelativeResponse",
    "FrcConnect",
    "FrcConnectResponse",
    "FrcContinue",
    "FrcContinueResponse",
    "FrcDisconnect",
    "FrcDisconnectResponse",
    "FrcGetStatus",
    "FrcGetStatusResponse",
    "FrcGetUFrameUTool",
    "FrcGetUFrameUToolResponse",
    "FrcInitialize",
    "FrcInitializeResponse",
    "FrcJointMotion",
    "FrcJointMotionJRep",
    "FrcJointMotionJRepResponse",
    "FrcJointMotionResponse",
    "FrcJointRelative",
    "FrcJointRelativeJRep",
    "FrcJointRelativeJRepResponse",
    "FrcJointRelativeResponse",
    "FrcLinearMotion",
    "FrcLinearMotionJRep",
    "FrcLinearMotionJRepResponse",
    "FrcLinearMotionResponse",
    "FrcLinearRelative",
    "FrcLinearRelativeJRep",
    "FrcLinearRelativeJRepResponse",
    "FrcLinearRelativeResponse",
    "FrcPause",
    "FrcPauseResponse",
    "FrcReadCartesianPosition",
    "FrcReadCartesianPositionResponse",
    "FrcReadDIN",
    "FrcReadDINResponse",
    "FrcReadError",
    "FrcReadErrorResponse",
    "FrcReadJointAngles",
    "FrcReadJointAnglesResponse",
    "FrcReadPositionRegister",
    "FrcReadPositionRegisterResponse",
    "FrcReadTCPSpeed",
    "FrcReadTCPSpeedResponse",
    "FrcReadUFrameData",
    "FrcReadUFrameDataResponse",
    "FrcReadUToolData",
    "FrcReadUToolDataResponse",
    "FrcReset",
    "FrcResetResponse",
    "FrcSetOverRide",
    "FrcSetOverRideResponse",
    "FrcSetPayLoad",
    "FrcSetPayLoadResponse",
    "FrcSetUFrame",
    "FrcSetUFrameResponse",
    "FrcSetUFrameUTool",
    "FrcSetUFrameUToolResponse",
    "FrcSetUTool",
    "FrcSetUToolResponse",
    "FrcSystemFaultResponse",
    "FrcTerminateResponse",
    "FrcWaitDIN",
    "FrcWaitDINResponse",
    "FrcWaitTime",
    "FrcWaitTimeResponse",
    "FrcWriteDOUT",
    "FrcWriteDOUTResponse",
    "FrcWritePositionRegister",
    "FrcWritePositionRegisterResponse",
    "FrcWriteUFrameData",
    "FrcWriteUFrameDataResponse",
    "FrcWriteUToolData",
    "FrcWriteUToolDataResponse",
    "Instruction",
    "InstructionResponse",
    "ResponsePacket",
    "RmiDriver",
    "RmiDriverConfig",
    "RmiHandle",
    "RmiHandleQueue",
    "SendPacket",
    "SoftwareOptions",
]

class SoftwareOptions(Enum):
    R921 = 921
    R792 = 792
    R640 = 640
    R904 = 904
    R806 = 806

_P_co = TypeVar("_P_co", bound=ResponsePacket, covariant=True)
_PM = TypeVar("_PM", bound=ResponsePacket)

class RmiHandle(Generic[_P_co]):
    def is_set(self) -> bool: ...
    def get(self) -> _P_co: ...
    def wait_timeout(self, timeout_secs: float) -> _P_co: ...
    def wait(self) -> _P_co: ...
    def timestamp(self) -> float | None: ...

class RmiHandleQueue(Generic[_PM]):
    def __init__(self) -> None: ...
    def push(self, handle: RmiHandle[_PM]) -> None: ...
    def handles(self) -> list[RmiHandle[_PM]]: ...
    def responses(self) -> list[_PM]: ...
    def wait_all_timeout(self, timeout_secs: float) -> list[_PM]: ...
    def wait_all(self) -> list[_PM]: ...
    def wait_next_timeout(self, timeout_secs: float) -> _PM: ...
    def wait_next(self) -> _PM: ...
    def all_set(self) -> bool: ...
    def raise_errors(self) -> None: ...
    def prune(self) -> None: ...
    def clear(self) -> None: ...

class RmiDriverConfig:
    address: str
    expected_major_version: int
    software_options: list[SoftwareOptions]
    buffer_cnt: int
    timeout_secs: float

    def __init__(
        self,
        address: str,
        expected_major_version: int = 7,
        software_options: list[SoftwareOptions] | None = None,
        buffer_cnt: int = 8,
        timeout_secs: float = 2.0,
    ) -> None: ...

class RmiDriver:
    def __init__(self, config: RmiDriverConfig) -> None: ...
    def connect(
        self,
        thread_config: ThreadConfig | None = None,
    ) -> FrcConnectResponse: ...
    def disconnect(self) -> RmiHandle[FrcDisconnectResponse]: ...
    def is_connected(self) -> bool: ...
    def has_connection_errored(self) -> bool:
        """Returns ``True`` if the background runner thread failed during setup.

        When this returns ``True`` the connection is non-functional and should
        be disconnected and re-established.
        """
    def version(self) -> tuple[int, int] | None: ...
    def send_full_reset(self) -> RmiHandle[FrcResetResponse]: ...

    # ---- send() — full per-packet overload tower ----
    #
    # Each sendable packet gets its own `@overload` so `driver.send(FrcXxx())`
    # statically resolves to `RmiHandle[FrcXxxResponse]` with the precise
    # response type and all its fields/methods autocompletable.
    #
    # PERFORMANCE NOTE: pyright/pylance evaluates every overload against
    # every completion candidate in the calling scope when it builds the
    # autocomplete dropdown for an argument position. With ~40 overloads
    # × the ~100 names visible in `fanuc_ucl.rmi`, completion at
    # `driver.send(rmi.|)` can take noticeable time on cold cache.
    #
    # Two ways to make this faster in practice:
    #
    #   1. Use a more focused namespace at the call site, e.g.
    #          from fanuc_ucl.rmi import FrcInitialize
    #          driver.send(FrcInitialize())  # completion is on a 1-name namespace
    #      instead of `driver.send(rmi.FrcInitialize())`.
    #
    #   2. Trigger completion AFTER typing the packet's leading characters.
    #      Pyright filters candidates by prefix before scoring, so
    #      `driver.send(rmi.FrcGet|)` is far cheaper than `driver.send(rmi.|)`.
    @overload
    def send(self, packet: FrcAbort) -> RmiHandle[FrcAbortResponse]: ...
    @overload
    def send(self, packet: FrcCall) -> RmiHandle[FrcCallResponse]: ...
    @overload
    def send(
        self, packet: FrcCircularMotion
    ) -> RmiHandle[FrcCircularMotionResponse]: ...
    @overload
    def send(
        self, packet: FrcCircularRelative
    ) -> RmiHandle[FrcCircularRelativeResponse]: ...
    @overload
    def send(self, packet: FrcConnect) -> RmiHandle[FrcConnectResponse]: ...
    @overload
    def send(self, packet: FrcContinue) -> RmiHandle[FrcContinueResponse]: ...
    @overload
    def send(self, packet: FrcDisconnect) -> RmiHandle[FrcDisconnectResponse]: ...
    @overload
    def send(self, packet: FrcGetStatus) -> RmiHandle[FrcGetStatusResponse]: ...
    @overload
    def send(
        self, packet: FrcGetUFrameUTool
    ) -> RmiHandle[FrcGetUFrameUToolResponse]: ...
    @overload
    def send(self, packet: FrcInitialize) -> RmiHandle[FrcInitializeResponse]: ...
    @overload
    def send(self, packet: FrcJointMotion) -> RmiHandle[FrcJointMotionResponse]: ...
    @overload
    def send(
        self, packet: FrcJointMotionJRep
    ) -> RmiHandle[FrcJointMotionJRepResponse]: ...
    @overload
    def send(self, packet: FrcJointRelative) -> RmiHandle[FrcJointRelativeResponse]: ...
    @overload
    def send(
        self, packet: FrcJointRelativeJRep
    ) -> RmiHandle[FrcJointRelativeJRepResponse]: ...
    @overload
    def send(self, packet: FrcLinearMotion) -> RmiHandle[FrcLinearMotionResponse]: ...
    @overload
    def send(
        self, packet: FrcLinearMotionJRep
    ) -> RmiHandle[FrcLinearMotionJRepResponse]: ...
    @overload
    def send(
        self, packet: FrcLinearRelative
    ) -> RmiHandle[FrcLinearRelativeResponse]: ...
    @overload
    def send(
        self, packet: FrcLinearRelativeJRep
    ) -> RmiHandle[FrcLinearRelativeJRepResponse]: ...
    @overload
    def send(self, packet: FrcPause) -> RmiHandle[FrcPauseResponse]: ...
    @overload
    def send(
        self, packet: FrcReadCartesianPosition
    ) -> RmiHandle[FrcReadCartesianPositionResponse]: ...
    @overload
    def send(self, packet: FrcReadDIN) -> RmiHandle[FrcReadDINResponse]: ...
    @overload
    def send(self, packet: FrcReadError) -> RmiHandle[FrcReadErrorResponse]: ...
    @overload
    def send(
        self, packet: FrcReadJointAngles
    ) -> RmiHandle[FrcReadJointAnglesResponse]: ...
    @overload
    def send(
        self, packet: FrcReadPositionRegister
    ) -> RmiHandle[FrcReadPositionRegisterResponse]: ...
    @overload
    def send(self, packet: FrcReadTCPSpeed) -> RmiHandle[FrcReadTCPSpeedResponse]: ...
    @overload
    def send(
        self, packet: FrcReadUFrameData
    ) -> RmiHandle[FrcReadUFrameDataResponse]: ...
    @overload
    def send(self, packet: FrcReadUToolData) -> RmiHandle[FrcReadUToolDataResponse]: ...
    @overload
    def send(self, packet: FrcReset) -> RmiHandle[FrcResetResponse]: ...
    @overload
    def send(self, packet: FrcSetOverRide) -> RmiHandle[FrcSetOverRideResponse]: ...
    @overload
    def send(self, packet: FrcSetPayLoad) -> RmiHandle[FrcSetPayLoadResponse]: ...
    @overload
    def send(self, packet: FrcSetUFrame) -> RmiHandle[FrcSetUFrameResponse]: ...
    @overload
    def send(
        self, packet: FrcSetUFrameUTool
    ) -> RmiHandle[FrcSetUFrameUToolResponse]: ...
    @overload
    def send(self, packet: FrcSetUTool) -> RmiHandle[FrcSetUToolResponse]: ...
    @overload
    def send(self, packet: FrcWaitDIN) -> RmiHandle[FrcWaitDINResponse]: ...
    @overload
    def send(self, packet: FrcWaitTime) -> RmiHandle[FrcWaitTimeResponse]: ...
    @overload
    def send(self, packet: FrcWriteDOUT) -> RmiHandle[FrcWriteDOUTResponse]: ...
    @overload
    def send(
        self, packet: FrcWritePositionRegister
    ) -> RmiHandle[FrcWritePositionRegisterResponse]: ...
    @overload
    def send(
        self, packet: FrcWriteUFrameData
    ) -> RmiHandle[FrcWriteUFrameDataResponse]: ...
    @overload
    def send(
        self, packet: FrcWriteUToolData
    ) -> RmiHandle[FrcWriteUToolDataResponse]: ...
    # Fallback for base-typed dispatch (`pkt: SendPacket = make_packet()`).
    @overload
    def send(self, packet: SendPacket) -> RmiHandle[ResponsePacket]: ...
