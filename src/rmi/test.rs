use snare::{TesterAction, TimerState, connect_tester, run_testers};

use super::*;
use proto::commands::*;
use proto::instructions::*;
use proto::member_structs::*;
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;

// --- Packetable impl for RMI JSON over TCP ---

#[derive(Debug, Clone)]
struct RmiPacket(String);

impl snare::Packetable for RmiPacket {
    const CAN_BE_FLATTENED: bool = false;
    const SOCKET_TYPE: snare::SocketType = snare::SocketType::Tcp;

    fn encode(&self) -> Vec<u8> {
        let mut bytes = self.0.as_bytes().to_vec();
        bytes.extend_from_slice(b"\r\n");
        bytes
    }

    fn decode(data: &[u8]) -> Option<(Self, usize)> {
        let s = std::str::from_utf8(data).ok()?;
        let pos = s.find("\r\n")?;
        let json_str = &s[..pos];
        // Validate it's valid JSON
        serde_json::from_str::<serde_json::Value>(json_str).ok()?;
        Some((RmiPacket(json_str.to_string()), pos + 2))
    }
}

// --- Simulated robot state ---

struct RobotState {
    override_pct: u8,
    u_frame_number: u8,
    u_tool_number: u8,
    u_frame_data: HashMap<i8, FrameData>,
    u_tool_data: HashMap<i8, FrameData>,
    position_registers: HashMap<u16, (Configuration, Position)>,
    din_values: HashMap<u16, u8>,
    dout_values: HashMap<u16, OnOff>,
    cartesian_position: Position,
    cartesian_config: Configuration,
    joint_angles: JointAngles,
    tcp_speed: f32,
    initialized: bool,
    // Status fields
    servo_ready: i8,
    tp_mode: i8,
    rmi_motion_status: i8,
    program_status: i8,
    single_step_mode: i8,
    number_utool: i8,
    number_uframe: i8,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            override_pct: 100,
            u_frame_number: 1,
            u_tool_number: 1,
            u_frame_data: HashMap::new(),
            u_tool_data: HashMap::new(),
            position_registers: HashMap::new(),
            din_values: HashMap::new(),
            dout_values: HashMap::new(),
            cartesian_position: Position::default(),
            cartesian_config: Configuration::default(),
            joint_angles: JointAngles {
                j1: 0.0,
                j2: 0.0,
                j3: 0.0,
                j4: 0.0,
                j5: 0.0,
                j6: 0.0,
                j7: 0.0,
                j8: 0.0,
                j9: 0.0,
            },
            tcp_speed: 0.0,
            initialized: false,
            servo_ready: 1,
            tp_mode: 0,
            rmi_motion_status: 0,
            program_status: 0,
            single_step_mode: 0,
            number_utool: 10,
            number_uframe: 9,
        }
    }
}

// --- Connect handler (port 16001) ---

const NEGOTIATED_PORT: u16 = 16002;

fn handle_connect_request(packet: RmiPacket, src: SocketAddr) -> TesterAction<RmiPacket> {
    let json: serde_json::Value = serde_json::from_str(&packet.0).unwrap();
    if json.get("Communication") == Some(&serde_json::Value::String("FRC_Connect".into())) {
        let response = serde_json::json!({
            "Communication": "FRC_Connect",
            "ErrorID": 0,
            "PortNumber": NEGOTIATED_PORT,
            "MajorVersion": 7,
            "MinorVersion": 1
        });
        TesterAction::Send(src, RmiPacket(response.to_string()))
    } else {
        TesterAction::Send(src, RmiPacket(r#"{"ErrorID":0}"#.to_string()))
    }
}

// --- Command/Instruction handler (negotiated port) ---

fn handle_rmi_request(
    state: &mut RobotState,
    packet: RmiPacket,
    src: SocketAddr,
) -> TesterAction<RmiPacket> {
    let json: serde_json::Value = serde_json::from_str(&packet.0).unwrap();

    if let Some(comm_name) = json.get("Communication").and_then(|v| v.as_str()) {
        return handle_communication(state, comm_name, &json, src);
    }
    if let Some(cmd_name) = json.get("Command").and_then(|v| v.as_str()) {
        return handle_command(state, cmd_name, &json, src);
    }
    if let Some(inst_name) = json.get("Instruction").and_then(|v| v.as_str()) {
        return handle_instruction(state, inst_name, &json, src);
    }

    TesterAction::Send(src, RmiPacket(r#"{"ErrorID":1}"#.to_string()))
}

fn handle_communication(
    _state: &mut RobotState,
    name: &str,
    _json: &serde_json::Value,
    src: SocketAddr,
) -> TesterAction<RmiPacket> {
    match name {
        // The RMI runner closes the connection immediately after sending disconnect,
        // so we don't send a response (the connection is already gone).
        "FRC_Disconnect" => TesterAction::Multiple(vec![]),
        _ => TesterAction::Send(src, RmiPacket(r#"{"ErrorID":0}"#.to_string())),
    }
}

fn handle_command(
    state: &mut RobotState,
    name: &str,
    json: &serde_json::Value,
    src: SocketAddr,
) -> TesterAction<RmiPacket> {
    let resp = match name {
        "FRC_Initialize" => {
            state.initialized = true;
            serde_json::json!({
                "Command": "FRC_Initialize",
                "ErrorID": 0
            })
        }
        "FRC_Abort" => serde_json::json!({ "Command": "FRC_Abort", "ErrorID": 0 }),
        "FRC_Pause" => serde_json::json!({ "Command": "FRC_Pause", "ErrorID": 0 }),
        "FRC_Continue" => serde_json::json!({ "Command": "FRC_Continue", "ErrorID": 0 }),
        "FRC_Reset" => serde_json::json!({ "Command": "FRC_Reset", "ErrorID": 0 }),
        "FRC_ReadError" => serde_json::json!({
            "Command": "FRC_ReadError",
            "ErrorID": 0,
            "ErrorData": ""
        }),
        "FRC_SetOverRide" => {
            if let Some(v) = json.get("Value").and_then(|v| v.as_u64()) {
                state.override_pct = v as u8;
            }
            serde_json::json!({ "Command": "FRC_SetOverRide", "ErrorID": 0 })
        }
        "FRC_GetStatus" => serde_json::json!({
            "Command": "FRC_GetStatus",
            "ErrorID": 0,
            "ServoReady": state.servo_ready,
            "TPMode": state.tp_mode,
            "RMIMotionStatus": state.rmi_motion_status,
            "ProgramStatus": state.program_status,
            "SingleStepMode": state.single_step_mode,
            "NumberUTool": state.number_utool,
            "NumberUFrame": state.number_uframe
        }),
        "FRC_SetUFrameUTool" => {
            if let Some(v) = json.get("UFrameNumber").and_then(|v| v.as_u64()) {
                state.u_frame_number = v as u8;
            }
            if let Some(v) = json.get("UToolNumber").and_then(|v| v.as_u64()) {
                state.u_tool_number = v as u8;
            }
            serde_json::json!({ "Command": "FRC_SetUFrameUTool", "ErrorID": 0 })
        }
        "FRC_GetUFrameUTool" => serde_json::json!({
            "Command": "FRC_GetUFrameUTool",
            "ErrorID": 0,
            "UFrameNumber": state.u_frame_number,
            "UToolNumber": state.u_tool_number
        }),
        "FRC_WriteUFrameData" => {
            if let (Some(num), Some(frame)) = (
                json.get("FrameNumber").and_then(|v| v.as_i64()),
                json.get("Frame"),
            ) && let Ok(f) = serde_json::from_value::<FrameData>(frame.clone())
            {
                state.u_frame_data.insert(num as i8, f);
            }
            serde_json::json!({ "Command": "FRC_WriteUFrameData", "ErrorID": 0 })
        }
        "FRC_ReadUFrameData" => {
            let num = json
                .get("FrameNumber")
                .and_then(|v| v.as_i64())
                .unwrap_or(1) as i8;
            let frame = state.u_frame_data.get(&num).copied().unwrap_or(FrameData {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
                p: 0.0,
                r: 0.0,
            });
            serde_json::json!({
                "Command": "FRC_ReadUFrameData",
                "ErrorID": 0,
                "UFrameNumber": num,
                "Frame": {
                    "x": frame.x, "y": frame.y, "z": frame.z,
                    "w": frame.w, "p": frame.p, "r": frame.r
                }
            })
        }
        "FRC_WriteUToolData" => {
            if let (Some(num), Some(frame)) = (
                json.get("ToolNumber").and_then(|v| v.as_i64()),
                json.get("Frame"),
            ) && let Ok(f) = serde_json::from_value::<FrameData>(frame.clone())
            {
                state.u_tool_data.insert(num as i8, f);
            }
            serde_json::json!({ "Command": "FRC_WriteUToolData", "ErrorID": 0 })
        }
        "FRC_ReadUToolData" => {
            let num = json
                .get("FrameNumber")
                .and_then(|v| v.as_i64())
                .unwrap_or(1) as i8;
            let frame = state.u_tool_data.get(&num).copied().unwrap_or(FrameData {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
                p: 0.0,
                r: 0.0,
            });
            serde_json::json!({
                "Command": "FRC_ReadUToolData",
                "ErrorID": 0,
                "UToolNumber": num,
                "Frame": {
                    "x": frame.x, "y": frame.y, "z": frame.z,
                    "w": frame.w, "p": frame.p, "r": frame.r
                }
            })
        }
        "FRC_WritePositionRegister" => {
            if let (Some(reg), Some(cfg), Some(pos)) = (
                json.get("RegisterNumber").and_then(|v| v.as_u64()),
                json.get("Configuration"),
                json.get("Position"),
            ) && let (Ok(c), Ok(p)) = (
                serde_json::from_value::<Configuration>(cfg.clone()),
                serde_json::from_value::<Position>(pos.clone()),
            ) {
                state.position_registers.insert(reg as u16, (c, p));
            }
            serde_json::json!({ "Command": "FRC_WritePositionRegister", "ErrorID": 0 })
        }
        "FRC_ReadPositionRegister" => {
            let reg = json
                .get("RegisterNumber")
                .and_then(|v| v.as_u64())
                .unwrap_or(1) as u16;
            let (config, position) = state
                .position_registers
                .get(&reg)
                .cloned()
                .unwrap_or_default();
            serde_json::json!({
                "Command": "FRC_ReadPositionRegister",
                "ErrorID": 0,
                "RegisterNumber": reg,
                "Configuration": config,
                "Position": position
            })
        }
        "FRC_ReadDIN" => {
            let port = json.get("PortNumber").and_then(|v| v.as_u64()).unwrap_or(1) as u16;
            let val = state.din_values.get(&port).copied().unwrap_or(0);
            serde_json::json!({
                "Command": "FRC_ReadDIN",
                "ErrorID": 0,
                "PortNumber": port,
                "PortValue": val
            })
        }
        "FRC_WriteDOUT" => {
            if let (Some(port), Some(val)) = (
                json.get("PortNumber").and_then(|v| v.as_u64()),
                json.get("PortValue").and_then(|v| v.as_str()),
            ) {
                let on_off = if val == "ON" { OnOff::ON } else { OnOff::OFF };
                state.dout_values.insert(port as u16, on_off);
            }
            serde_json::json!({ "Command": "FRC_WriteDOUT", "ErrorID": 0 })
        }
        "FRC_ReadCartesianPosition" => {
            let p = &state.cartesian_position;
            let c = &state.cartesian_config;
            serde_json::json!({
                "Command": "FRC_ReadCartesianPosition",
                "ErrorID": 0,
                "TimeTag": 0,
                "Configuration": c,
                "Position": p
            })
        }
        "FRC_ReadJointAngles" => {
            let j = &state.joint_angles;
            serde_json::json!({
                "Command": "FRC_ReadJointAngles",
                "ErrorID": 0,
                "TimeTag": 0,
                "JointAngle": j
            })
        }
        "FRC_ReadTCPSpeed" => serde_json::json!({
            "Command": "FRC_ReadTCPSpeed",
            "ErrorID": 0,
            "TimeTag": 0,
            "Speed": state.tcp_speed
        }),
        _ => serde_json::json!({ "ErrorID": 0 }),
    };
    TesterAction::Send(src, RmiPacket(resp.to_string()))
}

fn handle_instruction(
    _state: &mut RobotState,
    name: &str,
    json: &serde_json::Value,
    src: SocketAddr,
) -> TesterAction<RmiPacket> {
    let seq_id = json.get("SequenceID").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
    let resp = serde_json::json!({
        "Instruction": name,
        "ErrorID": 0,
        "SequenceID": seq_id
    });
    TesterAction::Send(src, RmiPacket(resp.to_string()))
}

// --- Test runner helper ---

fn noop_state_setup(_: &mut RobotState) {}

fn run_rmi_test<F>(ip: Ipv4Addr, setup_state: fn(&mut RobotState), client_fn: F)
where
    F: FnOnce(IpAddr) + Send + 'static,
{
    snare::register_test();
    let addr = IpAddr::V4(ip);
    snare::add_ip_addr(addr);

    let connect_addr = SocketAddr::new(addr, 16001);
    let command_addr = SocketAddr::new(addr, NEGOTIATED_PORT);

    let mut conn_tester = connect_tester::<RmiPacket>(connect_addr)
        .then_action(handle_connect_request)
        .until_stateful_condition::<TimerState>(|t| t.poll_elapsed() >= Duration::from_secs(5));

    let mut cmd_tester = connect_tester::<RmiPacket>(command_addr)
        .with_state::<RobotState>(setup_state)
        .then_stateful_action::<RobotState>(handle_rmi_request)
        .until_stateful_condition::<TimerState>(|t| t.poll_elapsed() >= Duration::from_secs(5));

    let thread_id = std::thread::current().id();
    let client_handle = std::thread::spawn(move || {
        snare::register_thread_child_of(thread_id);
        client_fn(addr);
    });

    run_testers!(conn_tester, cmd_tester);
    client_handle.join().unwrap();
}

fn make_config(ip: IpAddr) -> RmiDriverConfig {
    RmiDriverConfig::default_with_ip(ip)
}

fn default_position() -> Position {
    Position::default()
}

fn default_config() -> Configuration {
    Configuration::default()
}

fn default_joint_angles() -> JointAngles {
    JointAngles {
        j1: 10.0,
        j2: 20.0,
        j3: 30.0,
        j4: 40.0,
        j5: 50.0,
        j6: 60.0,
        j7: 0.0,
        j8: 0.0,
        j9: 0.0,
    }
}

// =====================================================================
// Connection tests
// =====================================================================

#[test]
fn test_connect_disconnect() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 1), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        let resp = driver.connect(None).expect("connect failed");
        assert_eq!(resp.error_id, 0);
        assert_eq!(resp.major_version, 7);
        assert_eq!(resp.minor_version, 1);
        assert!(driver.is_connected());
        driver.disconnect().expect("disconnect failed");
    });
}

#[test]
fn test_not_connected_error() {
    snare::register_test();
    let driver = RmiDriver::new(RmiDriverConfig::default_with_ip(IpAddr::V4(Ipv4Addr::new(
        10, 0, 1, 2,
    ))));
    assert!(!driver.is_connected());
    assert!(driver.send(FrcGetStatus).is_err());
}

// =====================================================================
// Simple command tests (return ErrorID only)
// =====================================================================

#[test]
fn test_initialize() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 3), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcInitialize::default())
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_abort() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 4), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcAbort)
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_pause() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 5), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcPause)
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_continue() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 6), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcContinue)
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_reset() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 7), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcReset)
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_read_error() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 8), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcReadError::default())
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error, 0);

        driver.disconnect().ok();
    });
}

// =====================================================================
// Stateful command tests
// =====================================================================

#[test]
fn test_set_override() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 10), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcSetOverRide::new(50))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_get_status() {
    run_rmi_test(
        Ipv4Addr::new(10, 0, 1, 11),
        |state: &mut RobotState| {
            state.servo_ready = 1;
            state.tp_mode = 2;
            state.number_utool = 10;
            state.number_uframe = 9;
        },
        |addr| {
            let mut driver = RmiDriver::new(make_config(addr));
            driver.connect(None).unwrap();

            let resp = driver
                .send(FrcGetStatus)
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            assert_eq!(resp.error_id, 0);
            assert_eq!(resp.servo_ready, 1);
            assert_eq!(resp.tp_mode, 2);
            assert_eq!(resp.number_utool, 10);
            assert_eq!(resp.number_uframe, 9);

            driver.disconnect().ok();
        },
    );
}

#[test]
fn test_set_get_uframe_utool() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 12), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        // Set UFrame=3, UTool=5
        let resp = driver
            .send(FrcSetUFrameUTool::new(5, 3, None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        // Read back
        let resp = driver
            .send(FrcGetUFrameUTool::new(None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert_eq!(resp.u_frame_number, 3);
        assert_eq!(resp.u_tool_number, 5);

        driver.disconnect().ok();
    });
}

#[test]
fn test_write_read_uframe_data() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 13), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let frame = FrameData {
            x: 100.0,
            y: 200.0,
            z: 300.0,
            w: 10.0,
            p: 20.0,
            r: 30.0,
        };

        // Write UFrame 2
        let resp = driver
            .send(FrcWriteUFrameData::new(2, frame, None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        // Read UFrame 2
        let resp = driver
            .send(FrcReadUFrameData::new(2, None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert_eq!(resp.frame.x, 100.0);
        assert_eq!(resp.frame.y, 200.0);
        assert_eq!(resp.frame.z, 300.0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_write_read_utool_data() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 14), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let frame = FrameData {
            x: 50.0,
            y: 60.0,
            z: 70.0,
            w: 1.0,
            p: 2.0,
            r: 3.0,
        };

        // Write UTool 3
        let resp = driver
            .send(FrcWriteUToolData::new(3, frame, None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        // Read UTool 3
        let resp = driver
            .send(FrcReadUToolData::new(3, None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert_eq!(resp.frame.x, 50.0);
        assert_eq!(resp.frame.y, 60.0);
        assert_eq!(resp.frame.z, 70.0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_write_read_position_register() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 15), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let config = default_config();
        let position = Position {
            x: 500.0,
            y: 600.0,
            z: 700.0,
            w: 0.0,
            p: 0.0,
            r: 0.0,
            ext1: 0.0,
            ext2: 0.0,
            ext3: 0.0,
        };

        // Write PR[5]
        let resp = driver
            .send(FrcWritePositionRegister::new(5, config, position, None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        // Read PR[5]
        let resp = driver
            .send(FrcReadPositionRegister::new(5, None))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert_eq!(resp.position.x, 500.0);
        assert_eq!(resp.position.y, 600.0);
        assert_eq!(resp.position.z, 700.0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_read_din() {
    run_rmi_test(
        Ipv4Addr::new(10, 0, 1, 16),
        |state: &mut RobotState| {
            state.din_values.insert(5, 1);
            state.din_values.insert(6, 0);
        },
        |addr| {
            let mut driver = RmiDriver::new(make_config(addr));
            driver.connect(None).unwrap();

            let resp = driver
                .send(FrcReadDIN::new(5))
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            assert_eq!(resp.error_id, 0);
            assert_eq!(resp.port_number, 5);
            assert_eq!(resp.port_value, 1);

            let resp = driver
                .send(FrcReadDIN::new(6))
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            assert_eq!(resp.port_value, 0);

            driver.disconnect().ok();
        },
    );
}

#[test]
fn test_write_dout() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 17), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcWriteDOUT::new(3, OnOff::ON))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_read_cartesian_position() {
    run_rmi_test(
        Ipv4Addr::new(10, 0, 1, 18),
        |state: &mut RobotState| {
            state.cartesian_position = Position {
                x: 100.0,
                y: 200.0,
                z: 300.0,
                w: 0.0,
                p: -90.0,
                r: 0.0,
                ext1: 0.0,
                ext2: 0.0,
                ext3: 0.0,
            };
        },
        |addr| {
            let mut driver = RmiDriver::new(make_config(addr));
            driver.connect(None).unwrap();

            let resp = driver
                .send(FrcReadCartesianPosition::new(None))
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            assert_eq!(resp.error_id, 0);
            assert_eq!(resp.pos.x, 100.0);
            assert_eq!(resp.pos.y, 200.0);
            assert_eq!(resp.pos.z, 300.0);
            assert_eq!(resp.pos.p, -90.0);

            driver.disconnect().ok();
        },
    );
}

#[test]
fn test_read_joint_angles() {
    run_rmi_test(
        Ipv4Addr::new(10, 0, 1, 19),
        |state: &mut RobotState| {
            state.joint_angles = default_joint_angles();
        },
        |addr| {
            let mut driver = RmiDriver::new(make_config(addr));
            driver.connect(None).unwrap();

            let resp = driver
                .send(FrcReadJointAngles::new(None))
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            assert_eq!(resp.error_id, 0);
            // Use joints() to get raw FanucDeg values (identity transform)
            use crate::joints::{JointFormat, JointTemplate};
            let j = resp.joints(JointFormat::FanucDeg, JointTemplate::SIX);
            assert_eq!(j.j1, 10.0);
            assert_eq!(j.j2, 20.0);
            assert_eq!(j.j6, 60.0);

            driver.disconnect().ok();
        },
    );
}

#[test]
fn test_read_tcp_speed() {
    run_rmi_test(
        Ipv4Addr::new(10, 0, 1, 20),
        |state: &mut RobotState| {
            state.tcp_speed = 123.456;
        },
        |addr| {
            let mut driver = RmiDriver::new(make_config(addr));
            driver.connect(None).unwrap();

            let resp = driver
                .send(FrcReadTCPSpeed)
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            assert_eq!(resp.error_id, 0);
            assert!((resp.speed - 123.456).abs() < 0.01);

            driver.disconnect().ok();
        },
    );
}

// =====================================================================
// Instruction tests (with SequenceID)
// =====================================================================

#[test]
fn test_wait_time() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 30), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcWaitTime::new(Duration::from_secs_f32(1.5)))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0, "SequenceID should be set");

        driver.disconnect().ok();
    });
}

#[test]
fn test_wait_din() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 31), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcWaitDIN::new(1, OnOff::ON))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_set_uframe_instruction() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 32), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcSetUFrame::new(3))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_set_utool_instruction() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 33), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcSetUTool::new(2))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_set_payload() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 34), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcSetPayLoad::new(1))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_call() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 35), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcCall::new("TEST_PROG".to_string()))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_linear_motion() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 36), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcLinearMotion::new(
                default_config(),
                default_position(),
                SpeedType::MMSec,
                100,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_joint_motion() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 37), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcJointMotion::new(
                default_config(),
                default_position(),
                SpeedType::MMSec,
                50,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_linear_relative() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 38), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcLinearRelative::new(
                default_config(),
                default_position(),
                SpeedType::MMSec,
                100,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_linear_motion_jrep() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 39), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcLinearMotionJRep::new(
                default_joint_angles(),
                SpeedType::MMSec,
                100,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_linear_relative_jrep() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 40), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcLinearRelativeJRep::new(
                default_joint_angles(),
                SpeedType::MMSec,
                100,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_joint_relative() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 41), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcJointRelative::new(
                default_config(),
                default_position(),
                SpeedType::MMSec,
                50,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_joint_motion_jrep() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 42), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcJointMotionJRep::new(
                default_joint_angles(),
                SpeedType::MMSec,
                50,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_joint_relative_jrep() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 43), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcJointRelativeJRep::new(
                default_joint_angles(),
                SpeedType::MMSec,
                50,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_circular_motion() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 44), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcCircularMotion::new(
                default_config(),
                default_position(),
                default_config(),
                default_position(),
                SpeedType::MMSec,
                100,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_circular_relative() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 45), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send(FrcCircularRelative::new(
                default_config(),
                default_position(),
                default_config(),
                default_position(),
                SpeedType::MMSec,
                100,
                TermType::FINE,
                100,
            ))
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
        assert!(resp.sequence_id > 0);

        driver.disconnect().ok();
    });
}

// =====================================================================
// Full reset test
// =====================================================================

#[test]
fn test_full_reset() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 50), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(None).unwrap();

        let resp = driver
            .send_full_reset()
            .unwrap()
            .wait_timeout(Duration::from_secs(5))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}
