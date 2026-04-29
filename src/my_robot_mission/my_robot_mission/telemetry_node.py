import json
import math
from typing import Dict, List

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String

from my_robot_mission.utils import load_yaml_file, quaternion_to_yaw


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__('telemetry_node')
        self.declare_parameter('waypoints_file', '')

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            raise RuntimeError('waypoints_file parameter is required.')
        cfg = load_yaml_file(waypoints_file)
        self.safety = cfg.get('safety', {})
        self.points = cfg.get('points', {})
        self.target = cfg.get('target_object', {})
        self.apriltag_target = cfg.get('apriltag_target', {})

        self.state = 'WAIT_FOR_GOAL'
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.velocity = {'linear': 0.0, 'angular': 0.0}
        self.command_velocity = {'linear': 0.0, 'angular': 0.0}
        self.joints = {'camera_pan_joint': 0.0, 'camera_tilt_joint': 0.0}
        self.nearest_obstacle = None
        self.orbit_status: Dict = {}
        self.video_status: Dict = {'recording': False, 'path': ''}
        self.events: List[str] = []
        self.warnings: List[str] = []

        self.telemetry_pub = self.create_publisher(String, '/mission/telemetry', 10)
        self.event_pub = self.create_publisher(String, '/mission/events', 10)

        self.create_subscription(String, '/mission/state', self.on_state, 10)
        self.create_subscription(String, '/mission/events', self.on_event, 10)
        self.create_subscription(String, '/mission/orbit_status', self.on_orbit_status, 10)
        self.create_subscription(String, '/mission/video_status', self.on_video_status, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_timer(0.2, self.publish_telemetry)

    def on_state(self, msg: String) -> None:
        self.state = msg.data

    def on_event(self, msg: String) -> None:
        self.events.append(msg.data)
        self.events = self.events[-30:]

    def on_orbit_status(self, msg: String) -> None:
        try:
            self.orbit_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.orbit_status = {}

    def on_video_status(self, msg: String) -> None:
        try:
            self.video_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.video_status = {'recording': False, 'path': ''}

    def on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        self.pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
        }
        self.velocity = {
            'linear': msg.twist.twist.linear.x,
            'angular': msg.twist.twist.angular.z,
        }

    def on_cmd_vel(self, msg: Twist) -> None:
        self.command_velocity = {
            'linear': msg.linear.x,
            'angular': msg.angular.z,
        }

    def on_scan(self, msg: LaserScan) -> None:
        valid_ranges = [
            value for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max
        ]
        self.nearest_obstacle = min(valid_ranges) if valid_ranges else None

    def on_joint_states(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.joints:
                self.joints[name] = position

    def publish_warning_event(self, warning: str) -> None:
        if warning not in self.warnings:
            self.event_pub.publish(String(data=f'WARNING:{warning}'))

    def evaluate_warnings(self) -> List[str]:
        warnings = []
        warning_distance = float(self.safety.get('warning_obstacle_distance', 0.75))
        max_linear = float(self.safety.get('max_linear_speed', 0.35))
        max_angular = float(self.safety.get('max_angular_speed', 1.2))
        max_radius_error = float(self.safety.get('max_orbit_radius_error', 0.35))
        tilt_lower = float(self.safety.get('tilt_lower_limit', -0.9))
        tilt_upper = float(self.safety.get('tilt_upper_limit', 0.7))

        if self.nearest_obstacle is not None and self.nearest_obstacle < warning_distance:
            warnings.append('obstacle_distance_low')
        if abs(self.command_velocity['linear']) > max_linear + 1.0e-3:
            warnings.append('linear_speed_limit_exceeded')
        if abs(self.command_velocity['angular']) > max_angular + 1.0e-3:
            warnings.append('angular_speed_limit_exceeded')
        radius_error = abs(float(self.orbit_status.get('radius_error', 0.0)))
        if self.state == 'ORBITING' and radius_error > max_radius_error:
            warnings.append('orbit_radius_error_high')
        tilt = self.joints.get('camera_tilt_joint', 0.0)
        if tilt < tilt_lower + 0.03 or tilt > tilt_upper - 0.03:
            warnings.append('camera_tilt_near_limit')

        for warning in warnings:
            self.publish_warning_event(warning)
        self.warnings = warnings
        return warnings

    def publish_telemetry(self) -> None:
        payload = {
            'stamp': self.get_clock().now().nanoseconds / 1.0e9,
            'mission_state': self.state,
            'pose': self.pose,
            'velocity': self.velocity,
            'command_velocity': self.command_velocity,
            'joints': self.joints,
            'nearest_obstacle': self.nearest_obstacle,
            'warnings': self.evaluate_warnings(),
            'orbit': self.orbit_status,
            'video': self.video_status,
            'events': self.events,
            'points': self.points,
            'target_object': self.target,
            'apriltag_target': self.apriltag_target,
        }
        self.telemetry_pub.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
