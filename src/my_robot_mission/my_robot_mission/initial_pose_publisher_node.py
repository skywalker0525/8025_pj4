from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from my_robot_mission.utils import load_yaml_file, yaw_to_quaternion


class InitialPosePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher_node')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('repeat_count', 20)
        self.declare_parameter('period_sec', 1.0)

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            raise RuntimeError('waypoints_file parameter is required.')

        cfg = load_yaml_file(waypoints_file)
        self.point_a = cfg['points']['A']
        self.target_frame = cfg.get('frames', {}).get('target_frame', 'map')
        self.repeat_count = int(self.get_parameter('repeat_count').value)
        period_sec = float(self.get_parameter('period_sec').value)
        self.publish_count = 0

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.timer = self.create_timer(period_sec, self.publish_initial_pose)

    def publish_initial_pose(self) -> None:
        if self.publish_count >= self.repeat_count:
            self.timer.cancel()
            self.get_logger().info('Finished publishing initial pose.')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.point_a['x'])
        msg.pose.pose.position.y = float(self.point_a['y'])
        quat = yaw_to_quaternion(float(self.point_a.get('yaw', 0.0)))
        msg.pose.pose.orientation.x = quat['x']
        msg.pose.pose.orientation.y = quat['y']
        msg.pose.pose.orientation.z = quat['z']
        msg.pose.pose.orientation.w = quat['w']
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05

        self.initial_pose_pub.publish(msg)
        self.publish_count += 1
        if self.publish_count == 1:
            self.event_pub.publish(String(data='INITIAL_POSE_PUBLISHED'))
        self.get_logger().info(
            f'Published initial pose A ({self.point_a["x"]}, {self.point_a["y"]}), '
            f'attempt {self.publish_count}/{self.repeat_count}.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
