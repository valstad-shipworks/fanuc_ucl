![Showcase video](examples/showcase.gif)

# Unofficial Control Library for FANUC Robots

A library implementing a variety of FANUC robot proprietary protocols such as:
- Stream Motion (stmo), UDP to controller port 60015
- High Speed Position Output (hspo), UDP received on a caller-chosen address
- Remote Motion Interface (rmi), TCP handshake on port 16001, then the controller-assigned session port
- SNPX based "HMI" (hmi), TCP port 60008

The library is implemented in rust with the ability to be used as a crate in rust or a python module via pyo3.
Has been tested with Linux(x86_64 and arm64), Windows(x86_64) and MacOS(arm64) although it likely works on all architectures that windows and macos support.

## Installation

### Rust
Add the following to your Cargo.toml:
```toml
[dependencies]
fanuc_ucl = "2"
```
or run `cargo add fanuc_ucl` in your project directory.

Each protocol is a cargo feature (`stmo`, `rmi`, `hspo`, `hmi`), all enabled by default; use `default-features = false` to pick a subset.
The `py` feature builds the Python bindings and changes some Rust signatures (for example `hspo::initialize_broker` takes a `String` address, and `RmiDriverConfig::default_with_ip` is not available), so Rust consumers should leave it off.

### Python
The library is available on PyPI as `fanuc_ucl`, so you can install it using pip:
```bash
pip install fanuc_ucl==2
```

## Usage

Python and rust have nearly identical APIs.
The Python API is as strongly typed as possible with advanced type hints.

### Unit and Format Safe Joint Representations

Every instance of working with direct joint representations requires also specifying the format and template of the joints, ensuring that the user is always aware of the units and the j2/j3 convention being used. Utilities to convert between different formats and templates are also provided as an independent API.

`JointFormat`s:
- FanucDeg: Angles in degrees, with the j3 angle relative to the ground plane
- FanucRad: Angles in radians, with the j3 angle relative to the ground plane
- AbsDeg: Angles in degrees, with the j3 angle relative to the previous joint
- AbsRad: Angles in radians, with the j3 angle relative to the previous joint

`JointTemplate`s can be arbitrarily created but the struct comes with a few pre-defined ones:
- SIX: The common 6 axis joint configuration of most FANUC robots
- SIX_LINEAR_TRACK: A 6 axis robot on a linear track, with the track being the 7th joint
- FOUR: A 4 axis joint configuration, common for FANUC palletizing robots
- FOUR_LINEAR_TRACK: A 4 axis robot on a linear track, with the track being the 5th joint
- FIVE: A 5 axis joint configuration, somewhat common for FANUC palletizing robots
- FIVE_LINEAR_TRACK: A 5 axis robot on a linear track, with the track being the 6th joint

```rust
use fanuc_ucl::joints::{JointFormat, JointTemplate};

fn main() {
    let joints = vec![-90.0, 0.0, 0.0, -180.0, 90.0, 180.0];
    let joints_conv = JointFormat::AbsRad.convert_from(JointFormat::FanucDeg, JointTemplate::SIX, joints);
    println!("Converted joints: {:?}", joints_conv);
}
```

```python
from fanuc_ucl import JointFormat, JointTemplate


def main():
    joints = [-90.0, 0.0, 0.0, -180.0, 90.0, 180.0]
    joints_conv = JointFormat.AbsRad.convert_from(
        JointFormat.FanucDeg, JointTemplate.SIX, joints
    )
    print(f"Converted joints: {joints_conv}")
```


### Stream Motion

Stream Motion gives real-time joint-level control of the robot at the interpolation
rate (typically 8ms). The controller requests a position every cycle and the driver's
I/O thread answers from a queue of motion commands; `command_motion` returns a handle
that is set once the whole batch has been transmitted. The last argument to
`StreamMotionDriver::new` controls whether the stream is marked finished when the
queue runs dry, and individual packets can be flagged with `set_last_command`.

The second argument must equal the controller's `$STMO.$START_MOVE` — how many commands it
queues before it begins executing them. The controller faults both when it receives a
command for a cycle it never announced and when its queue runs dry mid-motion, so the
driver mirrors that queue to decide how many commands may go out at once and how long
a blocked send may be retried before the robot runs out of motion. A value that
disagrees with the controller costs resilience in both directions. `driver.stats()`
reports what the model sees: buffer depth, measured cycle time, dropped cycles,
refills, send retries, and underruns.

#### Batch streaming

```rust
use std::time::Duration;

use fanuc_ucl::{
    ThreadOption,
    joints::{JointFormat, JointTemplate},
    stmo::{StreamMotionDriver, proto::MotionCommandPacket},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = StreamMotionDriver::new([10, 0, 0, 1], 5, false);
    driver.connect(&[ThreadOption::RtPriority(80)], &[])?;
    driver.start(2.0)?;

    let limits = driver.fetch_movement_limits(0)?;
    println!("Velocity cap: {}", limits.vmax);

    // one command per cycle: sweep J1 through a 10 degree sine wave over 4 seconds
    let home = [0.0f32, 0.0, 0.0, 0.0, -90.0, 0.0];
    let mut commands = Vec::with_capacity(500);
    for i in 0..500 {
        let mut joints = home;
        joints[0] += 10.0 * (i as f32 / 500.0 * std::f32::consts::TAU).sin();
        commands.push(MotionCommandPacket::try_from_joints(
            JointFormat::FanucDeg,
            JointTemplate::SIX,
            joints,
        )?);
    }

    let handle = driver.command_motion(commands)?;
    // do other work while the batch streams out …
    handle.wait_timeout(Duration::from_secs(10))?;

    driver.stop();
    driver.disconnect();
    Ok(())
}
```

```python
import math

from fanuc_ucl import JointFormat, JointTemplate, stmo


def main():
    driver = stmo.StreamMotionDriver("10.0.0.1", 5)
    driver.connect(thread=[("rt_priority", 80)])
    driver.start(2.0)

    limits = driver.fetch_movement_limits(0)
    print(f"Velocity cap: {limits.vmax}")

    # one command per cycle: sweep J1 through a 10 degree sine wave over 4 seconds
    home = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]
    commands = []
    for i in range(500):
        joints = home.copy()
        joints[0] += 10.0 * math.sin(i / 500.0 * math.tau)
        commands.append(
            stmo.MotionCommandPacket.try_from_joints(
                JointFormat.FanucDeg,
                JointTemplate.SIX,
                joints,
            )
        )

    handle = driver.command_motion(commands)
    # do other work while the batch streams out …
    handle.wait_timeout(10.0)

    driver.stop()
    driver.disconnect()
```

#### In-the-loop control

The control loop interface (`StmoControlLoop`) reacts to each robot status cycle
as it arrives — useful for sensor-based feedback or adaptive trajectories.

```rust
use std::time::Duration;

use fanuc_ucl::{
    ThreadOption,
    joints::{JointFormat, JointTemplate},
    stmo::{StreamMotionDriver, proto::MotionCommandPacket},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = StreamMotionDriver::new([10, 0, 0, 1], 5, false);
    driver.connect(&[ThreadOption::RtPriority(80)], &[])?;
    driver.start(2.0)?;

    {
        let mut ctl = driver.control_loop()?;
        for _ in 0..500 {
            let status = ctl.wait_for_status(Duration::from_millis(100))?;
            let mut joints = status.joints(JointFormat::FanucDeg, JointTemplate::SIX);
            // compute the next setpoint from current feedback …
            joints[5] += 0.05;
            ctl.send_command(MotionCommandPacket::try_from_joints(
                JointFormat::FanucDeg,
                JointTemplate::SIX,
                joints,
            )?)?;
        }
    } // control loop dropped — the driver resumes normal queue behaviour

    driver.stop();
    driver.disconnect();
    Ok(())
}
```

```python
from fanuc_ucl import JointFormat, JointTemplate, stmo


def main():
    driver = stmo.StreamMotionDriver("10.0.0.1", 5)
    driver.connect(thread=[("rt_priority", 80)])
    driver.start(2.0)

    with driver.control_loop() as ctl:
        for _ in range(500):
            status = ctl.wait_for_status(0.1)
            joints = list(status.joints(JointFormat.FanucDeg, JointTemplate.SIX))
            # compute the next setpoint from current feedback …
            joints[5] += 0.05
            ctl.send_command(
                stmo.MotionCommandPacket.try_from_joints(
                    JointFormat.FanucDeg,
                    JointTemplate.SIX,
                    joints,
                )
            )

    driver.stop()
    driver.disconnect()
```

### RMI

```rust
use std::time::Duration;

use fanuc_ucl::{rmi::{RmiDriver, RmiDriverConfig, proto}, joints::{JointFormat, JointTemplate}};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = RmiDriver::new(RmiDriverConfig::default_with_ip([10, 0, 0, 1]));
    driver.connect(&[], &[])?;

    driver.send_full_reset()?.wait_timeout(Duration::from_secs(15))?;
    driver.send(proto::FrcInitialize::default())?.wait_timeout(Duration::from_secs(15))?;
    driver.send(proto::FrcSetOverRide::new(80))?.wait_timeout(Duration::from_secs(1))?;
    driver.send(proto::FrcSetUTool::new(1))?.wait_timeout(Duration::from_secs(1))?;
    driver.send(proto::FrcSetUFrame::new(1))?.wait_timeout(Duration::from_secs(1))?;

    let movement_cmd1 = proto::FrcJointMotionJRep::new(
        proto::JointAngles::new6(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            -90.0, 0.0, 0.0, -180.0, 90.0, 180.0
        ),
        proto::SpeedType::MilliSeconds,
        528,
        proto::TermType::FINE,
        0
    );
    let movement_cmd2 = proto::FrcJointMotionJRep::new(
        proto::JointAngles::new6(
            JointFormat::AbsDeg,
            JointTemplate::SIX,
            90.0, 0.0, 0.0, 180.0, -90.0, -180.0
        ),
        proto::SpeedType::MilliSeconds,
        528,
        proto::TermType::FINE,
        0
    );
    driver.send(movement_cmd1)?.wait_timeout(Duration::from_millis(600))?;
    driver.send(proto::FrcWaitTime::new(Duration::from_secs(1)))?.wait()?;
    driver.send(movement_cmd2)?.wait_timeout(Duration::from_millis(600))?;

    let pos_resp = driver.send(proto::FrcReadJointAngles::new(None))?.wait()?;
    println!("Current joint angles: {:?}", pos_resp.joints(JointFormat::AbsDeg, JointTemplate::SIX).as_array());

    Ok(())
}
```

```python
from fanuc_ucl import JointFormat, JointTemplate, rmi


def main():
    driver = rmi.RmiDriver(rmi.RmiDriverConfig("10.0.0.1"))
    driver.connect()

    driver.send_full_reset().wait_timeout(20.0)
    driver.send(rmi.FrcInitialize()).wait_timeout(20.0)
    driver.send(rmi.FrcSetOverRide(80)).wait_timeout(1.0)
    driver.send(rmi.FrcSetUTool(1)).wait_timeout(1.0)
    driver.send(rmi.FrcSetUFrame(1)).wait_timeout(1.0)

    movement_cmd1 = rmi.FrcJointMotionJRep(
        rmi.JointAngles(
            JointFormat.AbsDeg,
            JointTemplate.SIX,
            *[-90.0, 0.0, 0.0, -180.0, 90.0, 180.0],
        ),
        rmi.SpeedType.MilliSeconds,
        528,
        rmi.TermType.FINE,
        0,
    )
    movement_cmd2 = rmi.FrcJointMotionJRep(
        rmi.JointAngles(
            JointFormat.AbsDeg,
            JointTemplate.SIX,
            *[90.0, 0.0, 0.0, 180.0, -90.0, -180.0],
        ),
        rmi.SpeedType.MilliSeconds,
        528,
        rmi.TermType.FINE,
        0,
    )
    driver.send(movement_cmd1).wait_timeout(0.6)
    driver.send(rmi.FrcWaitTime(1.0)).wait()
    driver.send(movement_cmd2).wait_timeout(0.6)

    pos_resp = driver.send(rmi.FrcReadJointAngles()).wait_timeout(0.2)
    print(
        f"Current joint angles: {pos_resp.joints(JointFormat.AbsDeg, JointTemplate.SIX).as_array()}"
    )
```

### HSPO

```rust
use std::{net::SocketAddr, thread::sleep, time::Duration};

use fanuc_ucl::{ThreadOption, hspo::{HspoReceiver, destroy_broker, initialize_broker}, joints::{JointFormat, JointTemplate}};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    initialize_broker(
        SocketAddr::from(([0, 0, 0, 0], 15000)),
        &[ThreadOption::RtPriority(55)],
        &[],
    ).expect("Broker couldnt be started");

    let receiver = HspoReceiver::try_new([10, 0, 0, 1], 128, Duration::from_millis(10))?;

    if let Some(joint_packet) = receiver.joint.wait_for(Duration::from_millis(16)) {
        println!(
            "Received joint packet: {:?}",
            joint_packet.joints(JointFormat::AbsDeg, JointTemplate::SIX)
        );
    }

    receiver.tcp.clear();
    sleep(Duration::from_secs(1));
    let opt_tcp_packet = receiver.tcp.try_recv();
    match opt_tcp_packet {
        Some(packet) => println!("Received TCP packet: {:?}", packet),
        None => println!("No TCP packet received within the timeout."),
    }
    let var_packets = receiver.var.recv_all();
    println!(
        "Received {} Variables packets: {:?}",
        var_packets.len(),
        var_packets
    );

    destroy_broker(true);
    Ok(())
}
```

```python
from fanuc_ucl import JointFormat, JointTemplate, hspo


def main():
    hspo.initialize_broker("0.0.0.0:15000", thread=[("rt_priority", 55)])

    receiver = hspo.HspoReceiver("10.0.0.1", 128)

    joint_packet = receiver.joint.wait_for(0.016)
    if joint_packet is not None:
        print(
            f"Received joint packet: {joint_packet.joints(JointFormat.AbsDeg, JointTemplate.SIX)}"
        )

    receiver.tcp.clear()
    tcp_packet = receiver.tcp.try_recv()
    if tcp_packet is not None:
        print(f"Received TCP packet: {tcp_packet}")
    else:
        print("No TCP packet received within the timeout.")
    var_packets = receiver.var.recv_all()
    print(f"Received {len(var_packets)} Variables packets: {var_packets}")

    hspo.destroy_broker()
```

### HMI

```rust
use std::time::Duration;

use fanuc_ucl::hmi::{DigitalOutput, GroupInput, GroupOutput, HmiDriver, SysVarArgs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = HmiDriver::new([10, 0, 0, 1]);
    driver.connect(Some(Duration::from_secs(1)), &[], &[])?;

    if driver.read::<DigitalOutput>(1)?.wait_timeout(Duration::from_millis(10))? {
        println!("DO1 is on");
    } else {
        println!("DO1 is off");
    }

    let groups = driver.read_array::<GroupInput>(1, 40)?.wait_timeout(Duration::from_millis(10))?;
    for (i, group) in groups.iter().enumerate() {
        println!("Group {}: {:?}", i + 1, group);
    }

    driver.write::<DigitalOutput>(1, true)?;
    driver.write_array::<GroupOutput>(1, &[10, 11, 12, 13])?;
    driver.write_array_unsafe::<GroupInput>(1, &[10, 11, 12, 13])?;

    let hspo_enable_var = driver.register_asg(
        SysVarArgs{
            var_name: "$MCGR_CFG.$ENABLE".to_string(),
            ..Default::default()
        },
        Duration::from_millis(24)
    )?;
    if hspo_enable_var.read(&driver)?.wait_timeout(Duration::from_millis(10))? {
        println!("HSPO is already enabled");
    } else {
        hspo_enable_var.write(&driver, true)?;
        println!("HSPO is now enabled");
    }

    // ASG has ALOT of functionality, so we won't cover it all here.
    // When the Rustdocs are finished it will delve more into it.

    Ok(())
}
```

```python
from fanuc_ucl import hmi


def main():
    driver = hmi.HmiDriver("10.0.0.1")
    driver.connect()

    if driver.read(hmi.DigitalOutput, 1).wait_timeout(0.01):
        print("DO1 is on")
    else:
        print("DO1 is off")

    groups = driver.read(hmi.GroupInput, 1, 40).wait_timeout(0.01)
    for i, group in enumerate(groups):
        print(f"Group {i + 1}: {group}")

    driver.write(hmi.DigitalOutput, 1, True)
    driver.write(hmi.GroupOutput, 1, [10, 11, 12, 13])
    driver.write_unsafe(hmi.GroupInput, 1, [10, 11, 12, 13])

    hspo_enable_var = driver.register_sysvar_asg(bool, name="$MCGR_CFG.$ENABLE")
    if hspo_enable_var.read(driver).wait_timeout(0.01):
        print("HSPO is already enabled")
    else:
        hspo_enable_var.write(driver, True)
        print("HSPO is now enabled")

    # ASG has ALOT of functionality, so we won't cover it all here.
    # When the Rustdocs are finished it will delve more into it.
```

## Real-time tuning

Every driver's `connect` (and `hspo::initialize_broker`) takes two lists of
[fast-talker](https://docs.rs/fast-talker) options, re-exported as
`fanuc_ucl::ThreadOption` and `fanuc_ucl::SocketOption`: `thread` is applied
by the driver's I/O thread to itself before it starts, and `socket` to the
driver's socket before it is bound or connected. Both default to empty,
meaning no tuning. An option a driver does not accept fails `connect` before
anything is spawned, and one that is attempted and fails (`RtPriority`
without `CAP_SYS_NICE`, say) fails `connect` too. Options for another
platform, or that this platform cannot do, are skipped with a warning, so one
configuration works on Linux, macOS and Windows. So is an option the platform
applied with a different value, such as a `RecvBuffer` capped by
`net.core.rmem_max`.

| Driver | Thread options | Socket options |
|---|---|---|
| stmo | all | all (UDP, sent and received every cycle) |
| hspo | all except `MacOsTimeConstraint` | `RecvBuffer`, `BindDevice`, `LinuxBusyPoll`, `LinuxPreferBusyPoll`, `LinuxBusyPollBudget`, `WinCpuAffinity` |
| rmi | `CpuAffinity`, `PrefaultStack`, `LinuxNice`, `UnixScheduler` (`Other`/`Batch`/`Idle`), `WinPriority` (not `TimeCritical`), `WinDisablePowerThrottling`, `MacOsQos` | `BindDevice`, `Dscp`, `LinuxPriority` |
| hmi | same as rmi | same as rmi |

Why the rest are refused:

- `MacOsTimeConstraint` reserves a computation slice per period; only stmo's
  loop has a period.
- hspo's socket only receives, so `SendBuffer`, `DontFragment`, `Dscp` and
  `LinuxPriority`, which shape outgoing traffic, do nothing for it.
- rmi and hmi threads block on TCP round-trips: a real-time class
  (`RtPriority`, `UnixScheduler` `Fifo`/`RoundRobin`, `WinPriority(TimeCritical)`,
  `WinMmcss`, `MacOsTimeConstraint`) there only risks starving the rest of the
  system.
- On their TCP sockets, setting buffer sizes would turn off TCP autotuning;
  busy polling burns a core on a slow loop; `DontFragment` and
  `WinCpuAffinity` do nothing useful for this traffic.

What the options did is kept for the life of the connection:
`tuning_report()` on each driver and `hspo::broker_tuning_report()` return
the options applied, those the platform adjusted, and those skipped with the
reason.

Process-wide settings (memory locking, `cpu_dma_latency`, the Windows
priority class, timer resolution and working set) belong to the application,
which applies them once with `fast_talker::options::ProcessOption::apply_all`,
or from Python with `fanuc_ucl.apply_process_options`.

From Python, the lists take any shape fast-talker accepts:

```python
import fanuc_ucl

with fanuc_ucl.apply_process_options(["lock_memory", ("linux_cpu_dma_latency", 0)]):
    driver.connect(
        thread=[("cpu_affinity", [3]), ("rt_priority", 80)],
        socket={"dscp": 46},
    )
    # {"thread": {"applied": [...], ...}, "socket": {...}}
    print(driver.tuning_report())
```

hspo stamps each datagram with the kernel's receive time where the platform
has one (fast-talker's `Timestamped`, never touching the NIC's hardware
timestamping), and on Linux logs a warning when the socket drops datagrams
for lack of receive buffer.

## Roadmap
- Pydocs and Rustdocs for all public APIs
- Switch python terminal logging to pylog instead of tracing (currently tracing to stderr, filtered by `RUST_LOG` and `fanuc_ucl.set_log_level`)
- ~~Implement an "In The Loop" interface for the `StreamMotionDriver` to make using feedback from sensors easier.~~
- Implement a unit-safe api for working with Cartesian poses.
- ~~Update docs to show the usage of stream motion and in-the-loop examples~~
- Update docs to show the usage of async rmi/hmi response handles
- ~~Add async to hspo~~
- Add support for async to python
- ~~Removing all possible panic locations and have graceful error handling for all failure modes.~~
- ~~Extensive unit testing, I wrote a special network testing library for this I just need to write the actual tests using it~~
- A C api for the library, to allow usage from other languages like c#. The main issue is the extensive usage of rust style enums and traits in the API, so this will require some careful design to make a clean and safe C api. This is a long term goal and will likely be a separate crate that depends on this one.

## Testing

`cargo test` runs the unit and property tests. The simulated-controller tests run on [snare](https://docs.rs/snare) and need [cargo-snare](https://crates.io/crates/cargo-snare): `cargo install cargo-snare --locked`, then `cargo snare test` (Linux gnu, macOS and Windows).

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
