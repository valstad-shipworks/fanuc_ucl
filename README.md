![Showcase video](examples/showcase.gif)

# Unofficial Control Library for FANUC Robots

A library implementing a variety of FANUC robot proprietary protocols such as:
- Stream Motion (stmo)
- High Speed Position Output (hspo)
- Remote Motion Interface (rmi)
- SNPX based "HMI" (hmi)

The library is implemented in rust with the ability to be used as a crate in rust or a python module via pyo3.
Has been tested with Linux(x86_64 and arm64), Windows(x86_64) and MacOS(arm64) although it likely works on all architectures that windows and macos support.

## Installation

### Rust
Add the following to your Cargo.toml:
```toml
[dependencies]
fanuc_ucl = "1"
```
or run `cargo add fanuc_ucl` in your project directory.

### Python
The library is available on PyPI as `fanuc_ucl`, so you can install it using pip:
```bash
pip install fanuc_ucl==1
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
    joints_conv = JointFormat.AbsRad.convert_from(JointFormat.FanucDeg, JointTemplate.SIX, joints)
    print(f"Converted joints: {joints_conv}")
```


### RMI

```rust
use std::time::Duration;

use fanuc_ucl::{rmi::{RmiDriver, RmiDriverConfig, proto}, joints::{JointFormat, JointTemplate}};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = RmiDriver::new(RmiDriverConfig::default_with_ip([10, 0, 0, 1]));
    driver.connect(Some(ThreadConfig::new(80, None)))?;

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
from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig, rmi

def main():
    driver = rmi.RmiDriver(rmi.RmiDriverConfig("10.0.0.1"))
    driver.connect(ThreadConfig(80, None))

    driver.send_full_reset().wait_timeout(20.0)
    driver.send(rmi.FrcInitialize()).wait_timeout(20.0)
    driver.send(rmi.FrcSetOverRide(80)).wait_timeout(1.0)
    driver.send(rmi.FrcSetUTool(1)).wait_timeout(1.0)
    driver.send(rmi.FrcSetUFrame(1)).wait_timeout(1.0)

    movement_cmd1 = rmi.FrcJointMotionJRep(
        rmi.JointAngles(
            JointFormat.AbsDeg,
            JointTemplate.SIX,
            *[-90.0, 0.0, 0.0, -180.0, 90.0, 180.0]
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
            *[90.0, 0.0, 0.0, 180.0, -90.0, -180.0]
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
    print(f"Current joint angles: {pos_resp.joints(JointFormat.AbsDeg, JointTemplate.SIX).as_array()}")
```

### HSPO

```rust
use std::{net::SocketAddr, thread::sleep, time::Duration};

use fanuc_ucl::{ThreadConfig, hspo::{HspoReceiver, destroy_broker, initialize_broker}, joints::{JointFormat, JointTemplate}};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    initialize_broker(
        SocketAddr::from(([0, 0, 0, 0], 15000)),
        Some(ThreadConfig::new(55, None)),
    ).expect("Broker couldnt be started");

    let receiver = HspoReceiver::try_new([10, 0, 0, 1], 128, Duration::from_millis(10))?;

    if let Some(joint_packet) = receiver.wait_for_joint_packet(Duration::from_millis(16)) {
        println!(
            "Received joint packet: {:?}",
            joint_packet.joints(JointFormat::AbsDeg, JointTemplate::SIX)
        );
    }

    receiver.clear_tcp_packet_buffer();
    sleep(Duration::from_secs(1));
    let opt_tcp_packet = receiver.try_recv_tcp_packet();
    match opt_tcp_packet {
        Some(packet) => println!("Received TCP packet: {:?}", packet),
        None => println!("No TCP packet received within the timeout."),
    }
    let var_packets = receiver.recv_all_var_packets();
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
from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig, hspo

def main():
    hspo.initialize_broker("0.0.0.0:15000", ThreadConfig(55, None))

    receiver = hspo.HspoReceiver("10.0.0.1", 128)

    joint_packet = receiver.wait_for_joint_packet(0.016)
    if joint_packet is not None:
        print(f"Received joint packet: {joint_packet.joints(JointFormat.AbsDeg, JointTemplate.SIX)}")

    receiver.clear_tcp_packet_buffer()
    tcp_packet = receiver.try_recv_tcp_packet()
    if tcp_packet is not None:
        print(f"Received TCP packet: {tcp_packet}")
    else:
        print("No TCP packet received within the timeout.")
    var_packets = receiver.recv_all_var_packets()
    print(f"Received {len(var_packets)} Variables packets: {var_packets}")

    hspo.destroy_broker()
```

### HMI

```rust
use std::time::Duration;

use fanuc_ucl::hmi::{DigitalOutput, GroupInput, GroupOutput, HmiDriver, SysVarArgs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = HmiDriver::new([10, 0, 0, 1]);
    driver.connect(Some(Duration::from_secs(1)), None)?;

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

### Stream Motion
TODO: The api is "done" and fully functional but I don't want to write examples yet because it is likely to change very soon as I add the "In The Loop" interface and async support.

## Roadmap
- Pydocs and Rustdocs for all public APIs
- ~Switch python terminal logging to pylog instead of tracing~
- ~Implement an "In The Loop" interface for the `StreamMotionDriver` to make using feedback from sensors easier.~
- Implement a unit-safe api for working with Cartesian poses.
- Update docs to show the usage of async rmi/hmi response handles
- Add async to hspo
- Add support for async to python
- ~Removing all possible panic locations and have graceful error handling for all failure modes.~
- ~Extensive unit testing, I wrote a special network testing library for this I just need to write the actual tests using it~
- A C api for the library, to allow usage from other languages like c#. The main issue is the extensive usage of rust style enums and traits in the API, so this will require some careful design to make a clean and safe C api. This is a long term goal and will likely be a separate crate that depends on this one.