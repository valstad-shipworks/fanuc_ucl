from enum import Enum

from fanuc_ucl import JointFormat, JointTemplate

__all__ = [
    "OnOff",
    "FrameData",
    "Configuration",
    "Position",
    "JointAngles",
    "LcbType",
    "PortType",
    "LocalConditionBlock",
    "OffsetRegisterNumbers",
    "TermType",
    "SpeedType",
    "ResponsePacket",
    "SendPacket",
    "Command",
    "CommandResponse",
    "Instruction",
    "InstructionResponse",
    "Communication",
    "CommunicationResponse",
    "FrcWriteUToolData",
    "FrcWriteUToolDataResponse",
    "FrcWriteUFrameData",
    "FrcWriteUFrameDataResponse",
    "FrcWritePositionRegister",
    "FrcWritePositionRegisterResponse",
    "FrcWriteDOUT",
    "FrcWriteDOUTResponse",
    "FrcSetUFrameUTool",
    "FrcSetUFrameUToolResponse",
    "FrcSetOverRide",
    "FrcSetOverRideResponse",
    "FrcReset",
    "FrcResetResponse",
    "FrcReadUToolData",
    "FrcReadUToolDataResponse",
    "FrcReadUFrameData",
    "FrcReadUFrameDataResponse",
    "FrcReadTCPSpeed",
    "FrcReadTCPSpeedResponse",
    "FrcReadPositionRegister",
    "FrcReadPositionRegisterResponse",
    "FrcReadJointAngles",
    "FrcReadJointAnglesResponse",
    "FrcReadError",
    "FrcReadErrorResponse",
    "FrcReadDIN",
    "FrcReadDINResponse",
    "FrcReadCartesianPosition",
    "FrcReadCartesianPositionResponse",
    "FrcPause",
    "FrcPauseResponse",
    "FrcInitialize",
    "FrcInitializeResponse",
    "FrcGetUFrameUTool",
    "FrcGetUFrameUToolResponse",
    "FrcGetStatus",
    "FrcGetStatusResponse",
    "FrcContinue",
    "FrcContinueResponse",
    "FrcAbort",
    "FrcAbortResponse",
    "FrcWaitTime",
    "FrcWaitTimeResponse",
    "FrcWaitDIN",
    "FrcWaitDINResponse",
    "FrcSetUTool",
    "FrcSetUToolResponse",
    "FrcSetUFrame",
    "FrcSetUFrameResponse",
    "FrcSetPayLoad",
    "FrcSetPayLoadResponse",
    "FrcLinearRelativeJRep",
    "FrcLinearRelativeJRepResponse",
    "FrcLinearRelative",
    "FrcLinearRelativeResponse",
    "FrcLinearMotionJRep",
    "FrcLinearMotionJRepResponse",
    "FrcLinearMotion",
    "FrcLinearMotionResponse",
    "FrcJointRelativeJRep",
    "FrcJointRelativeJRepResponse",
    "FrcJointRelative",
    "FrcJointRelativeResponse",
    "FrcJointMotionJRep",
    "FrcJointMotionJRepResponse",
    "FrcJointMotion",
    "FrcJointMotionResponse",
    "FrcCircularRelative",
    "FrcCircularRelativeResponse",
    "FrcCircularMotion",
    "FrcCircularMotionResponse",
    "FrcCall",
    "FrcCallResponse",
    "FrcConnect",
    "FrcDisconnect",
    "FrcConnectResponse",
    "FrcDisconnectResponse",
    "FrcTerminateResponse",
    "FrcSystemFaultResponse",
]

class OnOff(Enum):
    ON = 0
    OFF = 1

class FrameData:
    x: float
    y: float
    z: float
    w: float
    p: float
    r: float

class Configuration:
    u_tool_number: int
    u_frame_number: int
    front: int
    up: int
    left: int
    flip: int
    turn4: int
    turn5: int
    turn6: int

class Position:
    x: float
    y: float
    z: float
    w: float
    p: float
    r: float
    ext1: float
    ext2: float
    ext3: float

class JointAngles:
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    j7: float
    j8: float
    j9: float

    def __init__(
        self,
        format: JointFormat,
        template: JointTemplate,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        j5: float,
        j6: float,
        j7: float = 0.0,
        j8: float = 0.0,
        j9: float = 0.0,
    ) -> None: ...

    def as_array(self) -> list[float]: ...

class LcbType(Enum):
    TB = 0
    DB = 1
    TA = 2

class PortType(Enum):
    DOUT = 1
    ROUT = 2

class LocalConditionBlock:
    lcb_type: LcbType
    lcb_value: int
    port_type: PortType
    port_number: int
    port_value: OnOff

class OffsetRegisterNumbers:
    offset: int | None
    vision_offset: int | None
    tool_offset: int | None

class TermType(Enum):
    FINE = 0
    CNT = 1
    CR = 2

class SpeedType(Enum):
    MMSec = 0
    InchMin = 1
    TimeSec = 2
    MilliSeconds = 3

class ResponsePacket:
    def packet_name(self) -> str: ...

class SendPacket:
    def packet_name(self) -> str: ...

class Command(SendPacket): ...

class CommandResponse(ResponsePacket):
    def error_id(self) -> int: ...

class Instruction(SendPacket): ...

class InstructionResponse(ResponsePacket):
    def error_id(self) -> int: ...
    def sequence_id(self) -> int: ...

class Communication(SendPacket): ...

class CommunicationResponse(ResponsePacket):
    def error_id(self) -> int: ...

class FrcWriteUToolData(Command):
    tool_number: int
    frame: FrameData
    group: int

    def __init__(
        self, tool_number: int, frame: FrameData, group: int | None = None
    ) -> None: ...

class FrcWriteUToolDataResponse(CommandResponse):
    group: int | None

class FrcWriteUFrameData(Command):
    frame_number: int
    frame: FrameData
    group: int | None

    def __init__(
        self, frame_number: int, frame: FrameData, group: int | None = None
    ) -> None: ...

class FrcWriteUFrameDataResponse(CommandResponse):
    group: int | None

class FrcWritePositionRegister(Command):
    register_number: int
    congifuration: Configuration
    position: Position
    group: int | None

    def __init__(
        self,
        register_number: int,
        configuration: Configuration,
        position: Position,
        group: int | None = None,
    ) -> None: ...

class FrcWritePositionRegisterResponse(CommandResponse): ...

class FrcWriteDOUT(Command):
    port_number: int
    port_value: OnOff

    def __init__(self, port_num: int, port_val: OnOff) -> None: ...

class FrcWriteDOUTResponse(CommandResponse): ...

class FrcSetUFrameUTool(Command):
    u_frame_number: int
    u_tool_number: int
    group: int | None

    def __init__(
        self, u_tool_number: int, u_frame_number: int, group: int | None = None
    ) -> None: ...

class FrcSetUFrameUToolResponse(CommandResponse):
    group: int | None

class FrcSetOverRide(Command):
    value: int

    def __init__(self, value: int) -> None: ...

class FrcSetOverRideResponse(CommandResponse): ...

class FrcReset(Command):
    def __init__(self) -> None: ...

class FrcResetResponse(CommandResponse): ...

class FrcReadUToolData(Command):
    frame_number: int
    group: int | None

    def __init__(self, frame_number: int, group: int | None = None) -> None: ...

class FrcReadUToolDataResponse(CommandResponse):
    utool_number: int
    frame: FrameData
    group: int | None

class FrcReadUFrameData(Command):
    frame_number: int
    group: int | None

    def __init__(self, frame_number: int, group: int | None = None) -> None: ...

class FrcReadUFrameDataResponse(CommandResponse):
    u_frame_number: int
    frame: FrameData
    group: int | None

class FrcReadTCPSpeed(Command):
    def __init__(self) -> None: ...

class FrcReadTCPSpeedResponse(CommandResponse):
    time_tag: int
    speed: float

class FrcReadPositionRegister(Command):
    register_number: int
    group: int | None

    def __init__(self, register_number: int, group: int | None = None) -> None: ...

class FrcReadPositionRegisterResponse(CommandResponse):
    register_number: int
    config: Configuration
    position: Position
    group: int | None

class FrcReadJointAngles(Command):
    group: int | None

    def __init__(self, group: int | None = None) -> None: ...

class FrcReadJointAnglesResponse(CommandResponse):
    time_tag: int
    group: int | None

    def joints(self, format: JointFormat, template: JointTemplate) -> JointAngles: ...

class FrcReadError(Command):
    count: int | None

    def __init__(self, count: int | None = None) -> None: ...

class FrcReadErrorResponse(CommandResponse):
    error: int
    count: int
    error_data: str

class FrcReadDIN(Command):
    port_number: int

    def __init__(self, port_number: int) -> None: ...

class FrcReadDINResponse(CommandResponse):
    port_number: int
    port_value: int

class FrcReadCartesianPosition(Command):
    group: int | None

    def __init__(self, group: int | None = None) -> None: ...

class FrcReadCartesianPositionResponse(CommandResponse):
    time_tag: int
    config: Configuration
    pos: Position
    group: int | None

class FrcPause(Command):
    def __init__(self) -> None: ...

class FrcPauseResponse(CommandResponse): ...

class FrcInitialize(Command):
    group_mask: int

    def __init__(self, group_mask: int | None = None) -> None: ...

class FrcInitializeResponse(CommandResponse):
    group_mask: int

class FrcGetUFrameUTool(Command):
    group: int | None

    def __init__(self, group: int | None = None) -> None: ...

class FrcGetUFrameUToolResponse(CommandResponse):
    u_frame_number: int
    u_tool_number: int
    group: int | None

class FrcGetStatus(Command):
    def __init__(self) -> None: ...

class FrcGetStatusResponse(CommandResponse):
    servo_ready: int
    tp_mode: int
    rmi_motion_status: int
    program_status: int
    single_step_mode: int
    number_utool: int
    number_uframe: int

class FrcContinue(Command):
    def __init__(self) -> None: ...

class FrcContinueResponse(CommandResponse): ...

class FrcAbort(Command):
    def __init__(self) -> None: ...

class FrcAbortResponse(CommandResponse): ...

class FrcWaitTime(Instruction):
    time: float

    def __init__(self, time: float) -> None: ...

class FrcWaitTimeResponse(InstructionResponse): ...

class FrcWaitDIN(Instruction):
    port_number: int
    port_value: OnOff

    def __init__(self, port_number: int, port_value: OnOff) -> None: ...

class FrcWaitDINResponse(InstructionResponse): ...

class FrcSetUTool(Instruction):
    tool_number: int

    def __init__(self, tool_number: int) -> None: ...

class FrcSetUToolResponse(InstructionResponse): ...

class FrcSetUFrame(Instruction):
    frame_number: int

    def __init__(self, frame_number: int) -> None: ...

class FrcSetUFrameResponse(InstructionResponse): ...

class FrcSetPayLoad(Instruction):
    schedule_number: int

    def __init__(self, schedule_number: int) -> None: ...

class FrcSetPayLoadResponse(InstructionResponse): ...

class FrcLinearRelativeJRep(Instruction):
    joint_angles: JointAngles
    speed_type: SpeedType
    speed: int
    term_type: TermType
    term_value: int
    accel_override: int | None
    lcb: LocalConditionBlock | None
    alim: int | None
    alim_reg: int | None
    no_blend: bool
    wrist_joint: bool
    mrot: bool

    def __init__(
        self,
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: int,
        term_type: TermType,
        term_value: int
    ) -> None: ...

class FrcLinearRelativeJRepResponse(InstructionResponse): ...

class FrcLinearRelative(Instruction):
    configuration: Configuration
    position: Position
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcLinearRelativeResponse(InstructionResponse): ...

class FrcLinearMotionJRep(Instruction):
    joint_angles: JointAngles
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int
    accel_override: int | None
    lcb: LocalConditionBlock | None
    alim: int | None
    alim_reg: int | None
    no_blend: bool
    wrist_joint: bool
    mrot: bool

    def __init__(
        self,
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcLinearMotionJRepResponse(InstructionResponse): ...

class FrcLinearMotion(Instruction):
    configuration: Configuration
    position: Position
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcLinearMotionResponse(InstructionResponse): ...

class FrcJointRelativeJRep(Instruction):
    joint_angles: JointAngles
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcJointRelativeJRepResponse(InstructionResponse): ...

class FrcJointRelative(Instruction):
    configuration: Configuration
    position: Position
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcJointRelativeResponse(InstructionResponse): ...

class FrcJointMotionJRep(Instruction):
    joint_angles: JointAngles
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        joint_angles: JointAngles,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcJointMotionJRepResponse(InstructionResponse): ...

class FrcJointMotion(Instruction):
    configuration: Configuration
    position: Position
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        configuration: Configuration,
        position: Position,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcJointMotionResponse(InstructionResponse): ...

class FrcCircularRelative(Instruction):
    configuration: Configuration
    position: Position
    via_configuration: Configuration
    via_position: Position
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        configuration: Configuration,
        position: Position,
        via_configuration: Configuration,
        via_position: Position,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcCircularRelativeResponse(InstructionResponse): ...

class FrcCircularMotion(Instruction):
    configuration: Configuration
    position: Position
    via_configuration: Configuration
    via_position: Position
    speed_type: SpeedType
    speed: float
    term_type: TermType
    term_value: int

    def __init__(
        self,
        configuration: Configuration,
        position: Position,
        via_configuration: Configuration,
        via_position: Position,
        speed_type: SpeedType,
        speed: float,
        term_type: TermType,
        term_value: int,
    ) -> None: ...

class FrcCircularMotionResponse(InstructionResponse): ...

class FrcCall(Instruction):
    program_name: str

    def __init__(self, program_name: str) -> None: ...

class FrcCallResponse(InstructionResponse): ...

class FrcConnect(Communication):
    def __init__(self) -> None: ...

class FrcDisconnect(Communication):
    def __init__(self) -> None: ...

class FrcConnectResponse(CommunicationResponse):
    port_number: int
    major_version: int
    minor_version: int

class FrcDisconnectResponse(CommunicationResponse): ...
class FrcTerminateResponse(CommunicationResponse): ...

class FrcSystemFaultResponse(CommunicationResponse):
    sequence_id: int
