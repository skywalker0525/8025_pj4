from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from my_robot_mission.utils import clamp, load_yaml_file


class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control_node')
        self.declare_parameter('waypoints_file', '')

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            raise RuntimeError('waypoints_file parameter is required.')
        cfg = load_yaml_file(waypoints_file)
        safety = cfg.get('safety', {})
        self.max_linear_speed = float(safety.get('max_linear_speed', 0.35))
        self.max_angular_speed = float(safety.get('max_angular_speed', 1.2))
        self.pan_limit = float(safety.get('pan_limit', 3.14159))
        self.tilt_lower_limit = float(safety.get('tilt_lower_limit', -0.9))
        self.tilt_upper_limit = float(safety.get('tilt_upper_limit', 0.7))

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gimbal_pub = self.create_publisher(
            Float64MultiArray, '/camera_gimbal_position_controller/commands', 10
        )
        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.manual_cmd_sub = self.create_subscription(
            Twist, '/mission/manual_cmd_vel', self.on_manual_cmd, 10
        )
        self.gimbal_cmd_sub = self.create_subscription(
            Float64MultiArray, '/mission/manual_gimbal', self.on_gimbal_cmd, 10
        )
        self.command_sub = self.create_subscription(String, '/mission/command', self.on_command, 10)

    def on_manual_cmd(self, msg: Twist) -> None:
        cmd = Twist()
        cmd.linear.x = clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        cmd.angular.z = clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

    def on_gimbal_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warn('Manual gimbal command requires [pan, tilt].')
            return
        pan = clamp(float(msg.data[0]), -self.pan_limit, self.pan_limit)
        tilt = clamp(float(msg.data[1]), self.tilt_lower_limit, self.tilt_upper_limit)
        self.gimbal_pub.publish(Float64MultiArray(data=[pan, tilt]))

    def on_command(self, msg: String) -> None:
        if msg.data.strip().upper() in ('STOP', 'RESET'):
            self.cmd_pub.publish(Twist())
            self.event_pub.publish(String(data='MANUAL_CONTROL_STOPPED'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
