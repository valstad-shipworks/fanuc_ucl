use std::{net::SocketAddr, thread::sleep, time::Duration};

use fanuc_ucl::{
    ThreadConfig,
    hspo::{HspoReceiver, destroy_broker, initialize_broker},
    joints::{JointFormat, JointTemplate},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    initialize_broker(
        SocketAddr::from(([0, 0, 0, 0], 15000)),
        Some(ThreadConfig::new(55, None)),
    )
    .expect("Broker couldnt be started");

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
