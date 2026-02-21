from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig, hspo


def main():
    hspo.initialize_broker("0.0.0.0:15000", ThreadConfig(55, None))

    receiver = hspo.HspoReceiver("10.0.0.1", 128)

    joint_packet = receiver.wait_for_joint_packet(0.016)
    if joint_packet is not None:
        print(
            f"Received joint packet: {joint_packet.joints(JointFormat.AbsDeg, JointTemplate.SIX)}",
        )

    receiver.clear_tcp_packet_buffer()
    tcp_packet = receiver.try_recv_tcp_packet()
    if tcp_packet is not None:
        print(f"Received TCP packet: {tcp_packet}")
    else:
        print("No TCP packet received within the timeout.")
    var_packets = receiver.recv_all_var_packets()
    print(f"Received {len(var_packets)} Variables packets: {var_packets}")

    hspo.destroy_broker()
