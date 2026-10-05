import pytest


def test_byte_multi_array_accepts_compact_frame_on_ros_host():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("std_msgs")
    from rclpy.serialization import serialize_message
    from std_msgs.msg import ByteMultiArray
    from living_map_beacons.protocol import BeaconPacket, encode

    msg = ByteMultiArray()
    msg.data = encode(BeaconPacket(beacon_id=1, sequence=1))
    serialized = serialize_message(msg)
    assert serialized
