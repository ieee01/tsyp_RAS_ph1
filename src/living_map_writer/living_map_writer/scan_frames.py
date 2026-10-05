import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class ScanFrames(Node):
    def __init__(self):
        super().__init__('scan_frames', namespace='/living_map')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.wp = self.create_publisher(LaserScan, '/writer/scan', 10)
        self.ep = self.create_publisher(LaserScan, '/executor/scan', 10)
        self.fp = self.create_publisher(LaserScan, '/firebot/scan', 10)
        self.create_subscription(
            LaserScan, '/writer/scan_raw',
            lambda m: self.fix_scan(m, 'writer/laser', self.wp), 10)
        self.create_subscription(
            LaserScan, '/executor/scan_raw',
            lambda m: self.fix_scan(m, 'executor/laser', self.ep), 10)
        self.create_subscription(
            LaserScan, '/firebot/scan_raw',
            lambda m: self.fix_scan(m, 'firebot/laser', self.fp), 10)
        self.wo = self.create_publisher(Odometry, "/writer/odom", 20)
        self.eo = self.create_publisher(Odometry, "/executor/odom", 20)
        self.fo = self.create_publisher(Odometry, "/firebot/odom", 20)
        # ros_gz's Pose_V bridge can publish transforms from different simulated
        # instants in one TFMessage.  Publishing odometry transforms with the
        # odometry header timestamp keeps SLAM, Nav2 and RViz on one stable tree.
        self.create_subscription(
            Odometry, '/writer/world_odom',
            lambda m: self.fix_odom(m, 'writer', -11.0, 0.0, self.wo), 20)
        self.create_subscription(
            Odometry, '/executor/world_odom',
            lambda m: self.fix_odom(m, 'executor', -13.0, -2.0, self.eo), 20)
        self.create_subscription(
            Odometry, '/firebot/world_odom',
            lambda m: self.fix_odom(m, 'firebot', -13.0, 2.0, self.fo), 20)

    def fix_odom(self, msg, robot, spawn_x, spawn_y, publisher):
        # Ideal simulation odometry from the actual physics pose. Wheel encoder
        # integration can continue while pushing against a wall; that must not
        # be allowed to declare a mission completed without actual movement.
        msg.header.frame_id = f"{robot}/odom"
        msg.child_frame_id = f"{robot}/base_link"
        msg.pose.pose.position.x -= spawn_x
        msg.pose.pose.position.y -= spawn_y
        msg.pose.pose.position.z = 0.0
        publisher.publish(msg)
        self.publish_odom_tf(msg, msg.header.frame_id, msg.child_frame_id)

    @staticmethod
    def fix_scan(msg, frame, publisher):
        msg.header.frame_id = frame
        publisher.publish(msg)

    def publish_odom_tf(self, msg, parent, child):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = ScanFrames()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
