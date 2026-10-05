#![cfg(all(
    unix,
    any(
        all(
            target_os = "linux",
            target_env = "gnu",
            any(target_arch = "x86_64", target_arch = "aarch64")
        ),
        target_os = "macos",
        windows
    )
))]

use snare::{CrLf, Delimited, Sim, TesterAction, connect_tester, run_testers};

use super::*;
use errors::{RmiError, RmiProtocolError};
use proto::commands::*;
use proto::instructions::*;
use proto::member_structs::*;
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

type RmiPacket = Delimited<CrLf>;

fn frame(json: serde_json::Value) -> RmiPacket {
    Delimited::new(json.to_string())
}

/// Deterministic: under snare 2.0.0-alpha.1 a plain sim can skip a sleeper's
/// deadline while other sims in the process are polling.
fn sim() -> Sim {
    Sim::builder()
        .deterministic()
        .strict_sockopts()
        .stuck_after(Duration::from_secs(30))
        .build()
}

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

const NEGOTIATED_PORT: u16 = 16002;

fn handle_connect_request(packet: RmiPacket, _src: SocketAddr) -> TesterAction<RmiPacket> {
    let json: serde_json::Value = serde_json::from_slice(packet.body()).unwrap();
    if json.get("Communication") == Some(&serde_json::Value::String("FRC_Connect".into())) {
        let response = serde_json::json!({
            "Communication": "FRC_Connect",
            "ErrorID": 0,
            "PortNumber": NEGOTIATED_PORT,
            "MajorVersion": 7,
            "MinorVersion": 1
        });
        TesterAction::Send(frame(response))
    } else {
        TesterAction::Send(Delimited::new(r#"{"ErrorID":0}"#))
    }
}

fn handle_rmi_request(
    state: &mut RobotState,
    packet: RmiPacket,
    _src: SocketAddr,
) -> TesterAction<RmiPacket> {
    let json: serde_json::Value = serde_json::from_slice(packet.body()).unwrap();

    if let Some(comm_name) = json.get("Communication").and_then(|v| v.as_str()) {
        return handle_communication(state, comm_name, &json);
    }
    if let Some(cmd_name) = json.get("Command").and_then(|v| v.as_str()) {
        return handle_command(state, cmd_name, &json);
    }
    if let Some(inst_name) = json.get("Instruction").and_then(|v| v.as_str()) {
        return handle_instruction(state, inst_name, &json);
    }

    TesterAction::Send(Delimited::new(r#"{"ErrorID":1}"#))
}

fn handle_communication(
    _state: &mut RobotState,
    name: &str,
    _json: &serde_json::Value,
) -> TesterAction<RmiPacket> {
    match name {
        "FRC_Disconnect" => TesterAction::Send(frame(serde_json::json!({
            "Communication": "FRC_Disconnect",
            "ErrorID": 0
        }))),
        _ => TesterAction::Send(Delimited::new(r#"{"ErrorID":0}"#)),
    }
}

fn handle_command(
    state: &mut RobotState,
    name: &str,
    json: &serde_json::Value,
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
    TesterAction::Send(frame(resp))
}

fn handle_instruction(
    _state: &mut RobotState,
    name: &str,
    json: &serde_json::Value,
) -> TesterAction<RmiPacket> {
    let seq_id = json.get("SequenceID").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
    let resp = serde_json::json!({
        "Instruction": name,
        "ErrorID": 0,
        "SequenceID": seq_id
    });
    TesterAction::Send(frame(resp))
}

/// Marks the client finished even when it panics, so the testers stop and the
/// panic reaches the test instead of a hang.
struct Finished(Arc<AtomicBool>);

impl Drop for Finished {
    fn drop(&mut self) {
        self.0.store(true, Ordering::SeqCst);
    }
}

/// Runs `client` against a controller at `ip`: the handshake listener on 16001
/// and `session` on the negotiated port, until the client returns.
fn run_against<S: Send + 'static>(
    ip: Ipv4Addr,
    state: S,
    session: impl FnMut(&mut S, RmiPacket, SocketAddr) -> TesterAction<RmiPacket> + Send + 'static,
    client: impl FnOnce(IpAddr) + Send + 'static,
) {
    sim().run(|| {
        let addr = IpAddr::V4(ip);
        let done = Arc::new(AtomicBool::new(false));
        let finished = done.clone();
        let handshake =
            connect_tester::<RmiPacket>((addr, 16001)).then_action(handle_connect_request);
        let commands = connect_tester::<RmiPacket>((addr, NEGOTIATED_PORT))
            .with_state(state)
            .then_stateful_action(session)
            .until(move |_| finished.load(Ordering::SeqCst));
        let guard = Finished(done);
        let client = std::thread::spawn(move || {
            let _guard = guard;
            client(addr)
        });
        run_testers!(handshake, commands);
        if let Err(panic) = client.join() {
            std::panic::resume_unwind(panic);
        }
    });
}

fn noop_state_setup(_: &mut RobotState) {}

fn run_rmi_test<F>(ip: Ipv4Addr, setup_state: fn(&mut RobotState), client_fn: F)
where
    F: FnOnce(IpAddr) + Send + 'static,
{
    let mut state = RobotState::default();
    setup_state(&mut state);
    run_against(ip, state, handle_rmi_request, client_fn);
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

#[test]
fn test_connect_disconnect() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 1), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        let resp = driver.connect(&[], &[]).expect("connect failed");
        assert_eq!(resp.error_id, 0);
        assert_eq!(resp.major_version, 7);
        assert_eq!(resp.minor_version, 1);
        assert!(driver.is_connected());
        driver.disconnect().expect("disconnect failed");
    });
}

#[test]
fn test_not_connected_error() {
    let driver = RmiDriver::new(RmiDriverConfig::default_with_ip(IpAddr::V4(Ipv4Addr::new(
        10, 0, 1, 2,
    ))));
    assert!(!driver.is_connected());
    assert!(driver.send(FrcGetStatus).is_err());
}

#[test]
fn test_initialize() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 3), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

        let resp = driver
            .send(FrcReadError::default())
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_set_override() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 10), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(&[], &[]).unwrap();

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
            driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
            driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
            driver.connect(&[], &[]).unwrap();

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
            driver.connect(&[], &[]).unwrap();

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
            driver.connect(&[], &[]).unwrap();

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

#[test]
fn test_wait_time() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 30), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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
        driver.connect(&[], &[]).unwrap();

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

#[test]
fn test_full_reset() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 50), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(&[], &[]).unwrap();

        let resp = driver
            .send_full_reset()
            .unwrap()
            .wait_timeout(Duration::from_secs(5))
            .unwrap();
        assert_eq!(resp.error_id, 0);

        driver.disconnect().ok();
    });
}

#[test]
fn test_disconnect_handle_resolves_with_the_reply() {
    run_rmi_test(Ipv4Addr::new(10, 0, 1, 51), noop_state_setup, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(&[], &[]).unwrap();

        let resp = driver
            .disconnect()
            .unwrap()
            .wait_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(resp.error_id, 0);
    });
}

/// The runner parks in `poll` between messages; with the controller gone
/// silent nothing on the socket wakes it, so `disconnect` returning promptly
/// shows the driver wakes the runner itself.
#[test]
fn test_disconnect_runs_when_peer_silent() {
    let mut step = 0u8;
    let session = move |_: &mut (), _: RmiPacket, _: SocketAddr| {
        step = step.saturating_add(1);
        match step {
            1 => TesterAction::Send(Delimited::new(
                r#"{"Command":"FRC_Initialize","ErrorID":0}"#,
            )),
            2 => TesterAction::Quiesce(Duration::from_secs(5)),
            _ => TesterAction::Nothing,
        }
    };
    run_against(Ipv4Addr::new(10, 0, 99, 1), (), session, |addr| {
        let mut driver = RmiDriver::new(make_config(addr));
        driver.connect(&[], &[]).expect("connect failed");

        driver
            .send(FrcInitialize::default())
            .expect("send FrcInitialize")
            .wait_timeout(Duration::from_secs(2))
            .expect("await FrcInitialize");

        let _unanswered = driver.send(FrcGetStatus).expect("send sentinel");
        std::thread::sleep(Duration::from_millis(100));

        let start = Instant::now();
        driver.disconnect().expect("disconnect call");
        let elapsed = start.elapsed();
        assert!(
            elapsed < Duration::from_secs(1),
            "disconnect() took {elapsed:?}: the runner was not woken explicitly"
        );
    });
}

mod handle_unit_tests {
    use super::super::ResponsePacket;
    use super::super::proto::commands::FrcInitializeResponse;
    use super::super::rmi_handle::RmiHandleGeneric;
    use super::sim;
    use crate::rmi::errors::RmiError;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::thread;
    use std::time::{Duration, Instant};

    fn make_init_response(error_id: u32) -> ResponsePacket {
        ResponsePacket::from(FrcInitializeResponse {
            error_id,
            group_mask: None,
        })
    }

    #[test]
    fn wait_timeout_returns_immediately_when_already_set() {
        sim().run(|| {
            let handle = RmiHandleGeneric::new("FRC_Initialize", 1);
            handle.set_generic(make_init_response(0)).unwrap();

            let start = Instant::now();
            let resp = handle.wait_timeout(Duration::from_secs(5)).unwrap();
            let elapsed = start.elapsed();
            assert!(
                elapsed < Duration::from_millis(1),
                "wait_timeout on a pre-set handle took {elapsed:?}"
            );
            match resp {
                ResponsePacket::Command(_) => {}
                other => panic!("unexpected response variant: {other:?}"),
            }
        });
    }

    /// The waiter registers its listener before checking the state, so a
    /// response set while it waits wakes it at that instant.
    #[test]
    fn wait_timeout_observes_response_set_after_call_starts() {
        sim().run(|| {
            let handle = RmiHandleGeneric::new("FRC_Initialize", 1);
            let setter_handle = handle.clone();
            let setter_thread = thread::spawn(move || {
                thread::sleep(Duration::from_millis(50));
                setter_handle.set_generic(make_init_response(0)).unwrap();
            });

            let start = Instant::now();
            let resp = handle.wait_timeout(Duration::from_secs(5)).unwrap();
            let elapsed = start.elapsed();
            setter_thread.join().unwrap();

            assert!(
                elapsed >= Duration::from_millis(50) && elapsed < Duration::from_millis(51),
                "woke {elapsed:?} after a response set at 50 ms"
            );
            assert_eq!(resp.error_id(), 0);
        });
    }

    #[test]
    fn wait_timeout_returns_timeout_error_when_unset() {
        sim().run(|| {
            let handle = RmiHandleGeneric::new("FRC_Initialize", 1);
            let start = Instant::now();
            let res = handle.wait_timeout(Duration::from_millis(100));
            let elapsed = start.elapsed();
            assert!(
                matches!(res, Err(RmiError::Timeout)),
                "expected RmiError::Timeout, got {res:?}"
            );
            assert!(
                elapsed >= Duration::from_millis(100) && elapsed < Duration::from_millis(101),
                "a 100 ms timeout fired after {elapsed:?}"
            );
        });
    }

    #[test]
    fn set_generic_with_mismatched_packet_name_returns_packet_mismatch() {
        let handle = RmiHandleGeneric::new("FRC_Abort", 1);
        let res = handle.set_generic(make_init_response(0));
        assert!(
            matches!(res, Err(RmiError::PacketMismatch(_))),
            "expected PacketMismatch, got {res:?}"
        );
        assert!(handle.is_set());
        let resp = handle.wait_timeout(Duration::from_millis(100));
        assert!(
            matches!(resp, Err(RmiError::ResponseNotFulfilled(_))),
            "expected ResponseNotFulfilled, got {resp:?}"
        );
    }

    #[test]
    fn set_generic_propagates_fanuc_error_code_in_response() {
        let handle = RmiHandleGeneric::new("FRC_Initialize", 1);
        handle.set_generic(make_init_response(2)).unwrap();
        let res = handle.wait_timeout(Duration::from_millis(100));
        assert!(
            matches!(res, Err(RmiError::FanucErrorCode(_))),
            "expected FanucErrorCode, got {res:?}"
        );
    }

    #[test]
    fn timestamp_unset_before_set_then_present_after() {
        let handle = RmiHandleGeneric::new("FRC_Initialize", 1);
        assert!(handle.timestamp().is_none());
        handle.set_generic(make_init_response(0)).unwrap();
        assert!(handle.timestamp().is_some());
    }

    #[test]
    fn many_concurrent_waiters_all_observe_response() {
        sim().run(|| {
            let handle = RmiHandleGeneric::new("FRC_Initialize", 1);
            let all_succeeded = Arc::new(AtomicBool::new(true));
            let mut threads = Vec::new();
            for _ in 0..8 {
                let h = handle.clone();
                let flag = all_succeeded.clone();
                threads.push(thread::spawn(move || {
                    if h.wait_timeout(Duration::from_secs(3)).is_err() {
                        flag.store(false, Ordering::Relaxed);
                    }
                }));
            }
            thread::sleep(Duration::from_millis(50));
            handle.set_generic(make_init_response(0)).unwrap();
            for t in threads {
                t.join().unwrap();
            }
            assert!(
                all_succeeded.load(Ordering::Relaxed),
                "at least one waiter timed out instead of seeing the notify"
            );
        });
    }

    #[test]
    fn second_set_is_no_op_first_response_wins() {
        let handle = RmiHandleGeneric::new("FRC_Initialize", 1);
        handle.set_generic(make_init_response(0)).unwrap();
        handle.set_generic(make_init_response(7)).ok();
        let resp = handle.wait_timeout(Duration::from_millis(100)).unwrap();
        assert_eq!(
            resp.error_id(),
            0,
            "second set unexpectedly overwrote first"
        );
    }
}

fn seeded_sim(seed: u64) -> Sim {
    Sim::builder()
        .deterministic()
        .seed(seed)
        .strict_sockopts()
        .stuck_after(Duration::from_secs(30))
        .build()
}

/// The controller side of a session connection: `\r\n`-framed JSON both ways.
struct Session {
    reader: std::io::BufReader<std::net::TcpStream>,
    writer: std::net::TcpStream,
    line: String,
}

impl Session {
    fn new(stream: std::net::TcpStream) -> Self {
        Self {
            reader: std::io::BufReader::new(stream.try_clone().unwrap()),
            writer: stream,
            line: String::new(),
        }
    }

    /// The next request, or `None` once the driver closes its end. A read
    /// timeout surfaces as `Err` with any partial line kept for the next call.
    fn try_request(&mut self) -> std::io::Result<Option<serde_json::Value>> {
        use std::io::BufRead;
        match self.reader.read_line(&mut self.line) {
            Ok(0) => Ok(None),
            Ok(_) => {
                let v = serde_json::from_str(self.line.trim_end()).unwrap();
                self.line.clear();
                Ok(Some(v))
            }
            Err(e) => Err(e),
        }
    }

    fn request(&mut self) -> Option<serde_json::Value> {
        self.try_request().unwrap_or(None)
    }

    fn write(&mut self, bytes: &str) {
        use std::io::Write;
        let _ = self.writer.write_all(bytes.as_bytes());
    }

    fn reply(&mut self, json: &str) {
        self.write(&format!("{json}\r\n"));
    }

    /// Answers everything that arrives the way a healthy controller does,
    /// until the driver closes the connection. Returns whether FRC_Disconnect
    /// was among it.
    fn serve(&mut self) -> bool {
        let mut disconnected = false;
        while let Some(req) = self.request() {
            disconnected |= is_disconnect(&req);
            let reply = ok_reply(&req);
            self.reply(&reply);
        }
        disconnected
    }
}

fn is_disconnect(req: &serde_json::Value) -> bool {
    req.get("Communication").and_then(|v| v.as_str()) == Some("FRC_Disconnect")
}

/// The reply a healthy controller gives `req`.
fn ok_reply(req: &serde_json::Value) -> String {
    let text = |k: &str| req.get(k).and_then(|v| v.as_str());
    let resp = if let Some(name) = text("Instruction") {
        serde_json::json!({
            "Instruction": name,
            "ErrorID": 0,
            "SequenceID": req.get("SequenceID").cloned().unwrap_or(0.into()),
        })
    } else if text("Command") == Some("FRC_ReadDIN") {
        let port = req.get("PortNumber").and_then(|v| v.as_u64()).unwrap_or(0);
        serde_json::json!({
            "Command": "FRC_ReadDIN",
            "ErrorID": 0,
            "PortNumber": port,
            "PortValue": port % 2,
        })
    } else if let Some(name) = text("Command") {
        serde_json::json!({ "Command": name, "ErrorID": 0 })
    } else {
        serde_json::json!({ "Communication": text("Communication").unwrap_or(""), "ErrorID": 0 })
    };
    resp.to_string()
}

/// Runs `client` against a controller at `ip` that answers the FRC_Connect
/// handshake on 16001 and hands the session connection on 16002 to `session`.
/// Returns what `session` returns.
fn with_session<T: Send + 'static>(
    sim: Sim,
    ip: Ipv4Addr,
    session: impl FnOnce(Session) -> T + Send + 'static,
    client: impl FnOnce(IpAddr) + Send + 'static,
) -> T {
    sim.run(|| {
        let addr = IpAddr::V4(ip);
        let control = std::net::TcpListener::bind((addr, 16001)).unwrap();
        let negotiated = std::net::TcpListener::bind((addr, NEGOTIATED_PORT)).unwrap();
        let setup = snare::sched::setup_scope("test-spawn");
        let handshake = std::thread::spawn(move || {
            let (stream, _) = control.accept().unwrap();
            let mut s = Session::new(stream);
            if s.request().is_some() {
                s.reply(&format!(
                    r#"{{"Communication":"FRC_Connect","ErrorID":0,"PortNumber":{NEGOTIATED_PORT},"MajorVersion":7,"MinorVersion":1}}"#
                ));
            }
        });
        let server = std::thread::spawn(move || {
            let (stream, _) = negotiated.accept().unwrap();
            session(Session::new(stream))
        });
        let client = std::thread::spawn(move || client(addr));
        drop(setup);
        if let Err(panic) = client.join() {
            std::panic::resume_unwind(panic);
        }
        handshake.join().unwrap();
        server.join().unwrap()
    })
}

fn connected(addr: IpAddr) -> RmiDriver {
    let mut driver = RmiDriver::new(make_config(addr));
    driver.connect(&[], &[]).unwrap();
    driver
}

#[test]
fn a_response_split_across_reads_resolves_its_handle() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 1),
        |mut s| {
            s.request();
            s.write(r#"{"Command":"FRC_Initialize","#);
            std::thread::sleep(Duration::from_millis(1));
            s.write("\"ErrorID\":0}\r\n");
            s.serve();
        },
        |addr| {
            let mut driver = connected(addr);
            let resp = driver
                .send(FrcInitialize::default())
                .unwrap()
                .wait_timeout(Duration::from_secs(2));
            assert!(resp.is_ok(), "{resp:?}");
            driver.disconnect().unwrap();
        },
    );
}

#[test]
fn two_responses_in_one_read_resolve_in_order() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 2),
        |mut s| {
            let a = s.request().unwrap();
            let b = s.request().unwrap();
            let both = format!("{}\r\n{}\r\n", ok_reply(&a), ok_reply(&b));
            s.write(&both);
            s.serve();
        },
        |addr| {
            let mut driver = connected(addr);
            let first = driver.send(FrcReadDIN::new(3)).unwrap();
            let second = driver.send(FrcReadDIN::new(4)).unwrap();
            let second = second.wait_timeout(Duration::from_secs(2)).unwrap();
            let first = first.wait_timeout(Duration::from_secs(2)).unwrap();
            assert_eq!((first.port_number, first.port_value), (3, 1));
            assert_eq!((second.port_number, second.port_value), (4, 0));
            driver.disconnect().unwrap();
        },
    );
}

#[test]
fn malformed_json_fails_only_its_own_request() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 3),
        |mut s| {
            s.request();
            s.reply(r#"{"Command":"FRC_ReadDIN","ErrorID":0,"PortNumber":}"#);
            s.serve();
        },
        |addr| {
            let mut driver = connected(addr);
            let bad = driver
                .send(FrcReadDIN::new(1))
                .unwrap()
                .wait_timeout(Duration::from_secs(2));
            assert!(matches!(bad, Err(RmiError::Serde(_))), "{bad:?}");
            let good = driver
                .send(FrcReadDIN::new(7))
                .unwrap()
                .wait_timeout(Duration::from_secs(2))
                .unwrap();
            assert_eq!(good.port_number, 7);
            assert!(driver.is_connected());
            driver.disconnect().unwrap();
        },
    );
}

#[test]
fn controller_error_replies_resolve_as_errors() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 4),
        |mut s| {
            s.request();
            s.reply(r#"{"Command":"FRC_Initialize","ErrorID":2556935}"#);
            let wait = s.request().unwrap();
            let seq = wait.get("SequenceID").unwrap().as_u64().unwrap();
            s.reply(&format!(
                r#"{{"Communication":"FRC_SystemFault","SequenceID":{seq}}}"#
            ));
            s.serve();
        },
        |addr| {
            let mut driver = connected(addr);
            let servo_off = driver
                .send(FrcInitialize::default())
                .unwrap()
                .wait_timeout(Duration::from_secs(2));
            assert!(
                matches!(
                    servo_off,
                    Err(RmiError::FanucErrorCode(
                        RmiProtocolError::ControllerServoOff
                    ))
                ),
                "{servo_off:?}"
            );
            let faulted = driver
                .send(FrcWaitTime::new(Duration::from_millis(10)))
                .unwrap()
                .wait_timeout(Duration::from_secs(2));
            assert!(
                matches!(faulted, Err(RmiError::SystemFaultOrTerminate)),
                "{faulted:?}"
            );
            let next = driver
                .send(FrcReadDIN::new(2))
                .unwrap()
                .wait_timeout(Duration::from_secs(2));
            assert!(
                next.is_ok(),
                "the session did not survive an error reply: {next:?}"
            );
            driver.disconnect().unwrap();
        },
    );
}

#[test]
fn buffer_cnt_bounds_the_requests_in_flight() {
    let max_in_flight = with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 5),
        |mut s| {
            s.writer
                .set_read_timeout(Some(Duration::from_millis(2)))
                .unwrap();
            let mut unanswered = std::collections::VecDeque::new();
            let mut max_in_flight = 0;
            loop {
                match s.try_request() {
                    Ok(Some(req)) => {
                        if is_disconnect(&req) {
                            s.reply(&ok_reply(&req));
                            continue;
                        }
                        unanswered.push_back(req);
                        max_in_flight = max_in_flight.max(unanswered.len());
                    }
                    Ok(None) => return max_in_flight,
                    Err(_) => {
                        if let Some(req) = unanswered.pop_front() {
                            s.reply(&ok_reply(&req));
                        }
                    }
                }
            }
        },
        |addr| {
            let mut config = make_config(addr);
            config.buffer_cnt = 3;
            let mut driver = RmiDriver::new(config);
            driver.connect(&[], &[]).unwrap();
            let handles: Vec<_> = (0..10)
                .map(|_| {
                    driver
                        .send(FrcWaitTime::new(Duration::from_millis(1)))
                        .unwrap()
                })
                .collect();
            for (i, h) in handles.iter().enumerate() {
                let resp = h.wait_timeout(Duration::from_secs(2)).unwrap();
                assert_eq!(resp.sequence_id, i as u32 + 1);
            }
            driver.disconnect().unwrap();
        },
    );
    assert_eq!(
        max_in_flight, 3,
        "buffer_cnt 3 let {max_in_flight} requests out"
    );
}

#[test]
fn a_peer_close_mid_instruction_fails_its_handle_at_once() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 6),
        |mut s| {
            s.request();
        },
        |addr| {
            let driver = connected(addr);
            let handle = driver
                .send(FrcWaitTime::new(Duration::from_secs(1)))
                .unwrap();
            let t0 = Instant::now();
            let res = handle.wait_timeout(Duration::from_secs(2));
            let waited = t0.elapsed();
            assert!(matches!(res, Err(RmiError::Disconnected)), "{res:?}");
            assert!(
                waited < Duration::from_millis(1),
                "the handle took {waited:?} to notice the close"
            );
            std::thread::sleep(Duration::from_millis(1));
            assert!(!driver.is_connected());
            assert!(driver.has_connection_errored());
        },
    );
}

#[test]
fn a_peer_reset_fails_requests_in_flight_and_queued() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 7),
        |mut s| {
            s.request();
            while s.try_request().is_ok_and(|r| r.is_some()) {}
        },
        |addr| {
            let mut config = make_config(addr);
            config.buffer_cnt = 1;
            let mut driver = RmiDriver::new(config);
            driver.connect(&[], &[]).unwrap();
            let handles: Vec<_> = (0..3)
                .map(|_| {
                    driver
                        .send(FrcWaitTime::new(Duration::from_secs(1)))
                        .unwrap()
                })
                .collect();
            std::thread::sleep(Duration::from_millis(1));
            snare::raise_socket_error(
                SocketAddr::new(addr, NEGOTIATED_PORT),
                std::io::Error::from(std::io::ErrorKind::ConnectionReset),
            );
            for (i, h) in handles.iter().enumerate() {
                let res = h.wait_timeout(Duration::from_secs(2));
                assert!(
                    matches!(res, Err(RmiError::Disconnected)),
                    "request {i} after a reset: {res:?}"
                );
            }
            std::thread::sleep(Duration::from_millis(1));
            assert!(!driver.is_connected());
            assert!(driver.has_connection_errored());
        },
    );
}

#[test]
fn a_refused_handshake_fails_connect_at_once() {
    sim().run(|| {
        let addr = IpAddr::V4(Ipv4Addr::new(10, 0, 2, 8));
        snare::set_listener_behavior((addr, 16001), snare::ListenerBehavior::Refusing);
        let mut driver = RmiDriver::new(make_config(addr));
        let t0 = Instant::now();
        let err = driver.connect(&[], &[]).unwrap_err();
        assert!(
            matches!(&err, RmiError::CommunicationError(e) if e.kind() == std::io::ErrorKind::ConnectionRefused),
            "{err:?}"
        );
        assert!(t0.elapsed() < Duration::from_millis(1));
        assert!(!driver.is_connected());
    });
}

#[test]
fn a_refused_session_port_fails_connect() {
    sim().run(|| {
        let addr = IpAddr::V4(Ipv4Addr::new(10, 0, 2, 9));
        let control = std::net::TcpListener::bind((addr, 16001)).unwrap();
        let setup = snare::sched::setup_scope("test-spawn");
        let handshake = std::thread::spawn(move || {
            let (stream, _) = control.accept().unwrap();
            let mut s = Session::new(stream);
            s.request();
            s.reply(&format!(
                r#"{{"Communication":"FRC_Connect","ErrorID":0,"PortNumber":{NEGOTIATED_PORT},"MajorVersion":7,"MinorVersion":1}}"#
            ));
        });
        let client = std::thread::spawn(move || {
            let mut driver = RmiDriver::new(make_config(addr));
            let t0 = Instant::now();
            let err = driver.connect(&[], &[]).unwrap_err();
            assert!(
                matches!(&err, RmiError::CommunicationError(e) if e.kind() == std::io::ErrorKind::ConnectionRefused),
                "{err:?}"
            );
            assert!(t0.elapsed() < Duration::from_millis(1));
            assert!(!driver.is_connected());
            assert!(matches!(
                driver.send(FrcInitialize::default()),
                Err(RmiError::Disconnected)
            ));
        });
        drop(setup);
        if let Err(panic) = client.join() {
            std::panic::resume_unwind(panic);
        }
        handshake.join().unwrap();
    });
}

#[test]
fn disconnect_waits_for_a_late_reply() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 10),
        |mut s| {
            while let Some(req) = s.request() {
                if is_disconnect(&req) {
                    std::thread::sleep(Duration::from_millis(300));
                }
                s.reply(&ok_reply(&req));
            }
        },
        |addr| {
            let mut driver = connected(addr);
            let t0 = Instant::now();
            let bye = driver.disconnect().unwrap();
            let waited = t0.elapsed();
            assert!(
                waited >= Duration::from_millis(300) && waited < Duration::from_millis(301),
                "disconnect returned after {waited:?} for a reply sent at 300 ms"
            );
            assert_eq!(bye.get().unwrap().error_id, 0);
        },
    );
}

#[test]
fn disconnect_gives_up_on_a_silent_controller_after_500_ms() {
    with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 11),
        |mut s| while s.request().is_some() {},
        |addr| {
            let mut driver = connected(addr);
            let t0 = Instant::now();
            let bye = driver.disconnect().unwrap();
            let waited = t0.elapsed();
            assert!(
                waited >= Duration::from_millis(500) && waited < Duration::from_millis(501),
                "disconnect gave up after {waited:?}"
            );
            assert!(
                matches!(bye.get(), Err(RmiError::Disconnected)),
                "{:?}",
                bye.get()
            );
        },
    );
}

#[test]
fn a_disconnect_right_after_connect_still_reaches_the_controller() {
    for seed in 0..8 {
        let reached = with_session(
            seeded_sim(seed),
            Ipv4Addr::new(10, 0, 2, 12),
            |mut s| s.serve(),
            move |addr| {
                let mut driver = connected(addr);
                let bye = driver.disconnect().unwrap();
                assert!(bye.get().is_ok(), "seed {seed}: {:?}", bye.get());
            },
        );
        assert!(
            reached,
            "seed {seed}: FRC_Disconnect never reached the controller"
        );
    }
}

#[test]
fn unsent_requests_resolve_disconnected_when_the_runner_exits() {
    let reached = with_session(
        sim(),
        Ipv4Addr::new(10, 0, 2, 13),
        |mut s| {
            let mut reached = false;
            while let Some(req) = s.request() {
                reached |= is_disconnect(&req);
            }
            reached
        },
        |addr| {
            let mut config = make_config(addr);
            config.buffer_cnt = 1;
            let mut driver = RmiDriver::new(config);
            driver.connect(&[], &[]).unwrap();
            let handles: Vec<_> = (0..3)
                .map(|_| {
                    driver
                        .send(FrcWaitTime::new(Duration::from_secs(1)))
                        .unwrap()
                })
                .collect();
            std::thread::sleep(Duration::from_millis(1));
            let t0 = Instant::now();
            let bye = driver.disconnect().unwrap();
            let waited = t0.elapsed();
            assert!(
                waited >= Duration::from_millis(500) && waited < Duration::from_millis(501),
                "disconnect returned after {waited:?}"
            );
            for (i, h) in handles.iter().enumerate() {
                assert!(
                    matches!(h.get(), Err(RmiError::Disconnected)),
                    "request {i}: {:?}",
                    h.get()
                );
            }
            assert!(
                matches!(bye.get(), Err(RmiError::Disconnected)),
                "{:?}",
                bye.get()
            );
        },
    );
    assert!(reached, "FRC_Disconnect never reached the controller");
}

#[test]
fn requests_from_several_threads_each_get_their_own_reply() {
    for seed in 0..4 {
        let sequence_ids = with_session(
            seeded_sim(seed),
            Ipv4Addr::new(10, 0, 2, 14),
            |mut s| {
                let mut ids = Vec::new();
                while let Some(req) = s.request() {
                    if let Some(id) = req.get("SequenceID").and_then(|v| v.as_u64()) {
                        ids.push(id);
                    }
                    s.reply(&ok_reply(&req));
                }
                ids
            },
            move |addr| {
                let driver = Arc::new(connected(addr));
                let workers: Vec<_> = (0..4u16)
                    .map(|t| {
                        let driver = driver.clone();
                        std::thread::spawn(move || {
                            for i in 0..10u16 {
                                let port = t * 100 + i;
                                let din = driver.send(FrcReadDIN::new(port)).unwrap();
                                let wait = driver
                                    .send(FrcWaitTime::new(Duration::from_millis(1)))
                                    .unwrap();
                                let din = din.wait_timeout(Duration::from_secs(2)).unwrap();
                                assert_eq!(din.port_number, port, "seed {seed}");
                                wait.wait_timeout(Duration::from_secs(2)).unwrap();
                            }
                        })
                    })
                    .collect();
                for w in workers {
                    w.join().unwrap();
                }
                let mut driver = Arc::into_inner(driver).unwrap();
                driver.disconnect().unwrap();
            },
        );
        let mut sorted = sequence_ids.clone();
        sorted.sort_unstable();
        assert_eq!(
            sorted,
            (1..=40).collect::<Vec<u64>>(),
            "seed {seed}: {sequence_ids:?}"
        );
    }
}

#[test]
fn a_connect_reply_split_across_segments_still_connects() {
    seeded_sim(3).run(|| {
        let addr = IpAddr::V4(Ipv4Addr::new(10, 0, 2, 40));
        let control = std::net::TcpListener::bind((addr, 16001)).unwrap();
        let negotiated = std::net::TcpListener::bind((addr, NEGOTIATED_PORT)).unwrap();
        let setup = snare::sched::setup_scope("test-spawn");
        let handshake = std::thread::spawn(move || {
            let (stream, _) = control.accept().unwrap();
            let mut s = Session::new(stream);
            s.request();
            let reply = format!(
                r#"{{"Communication":"FRC_Connect","ErrorID":0,"PortNumber":{NEGOTIATED_PORT},"MajorVersion":7,"MinorVersion":1}}"#
            );
            let (head, tail) = reply.split_at(reply.len() / 2);
            s.write(head);
            std::thread::sleep(Duration::from_millis(2));
            s.write(&format!("{tail}\r\n"));
        });
        let server = std::thread::spawn(move || {
            let (stream, _) = negotiated.accept().unwrap();
            Session::new(stream).serve()
        });
        let client = std::thread::spawn(move || {
            let mut driver = RmiDriver::new(make_config(addr));
            let connected = driver.connect(&[], &[]);
            assert!(connected.is_ok(), "{connected:?}");
            driver.disconnect().unwrap();
        });
        drop(setup);
        if let Err(panic) = client.join() {
            std::panic::resume_unwind(panic);
        }
        handshake.join().unwrap();
        let _ = server.join();
    });
}
