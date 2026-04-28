import json
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_mission.utils import clamp, load_yaml_file, normalize_angle, quaternion_to_yaw


class OrbitControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('orbit_controller_node')
        self.declare_parameter('waypoints_file', '')

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            raise RuntimeError('waypoints_file parameter is required.')

        cfg = load_yaml_file(waypoints_file)
        target = cfg['target_object']
        orbit = cfg['orbit']
        safety = cfg.get('safety', {})
        self.target_x = float(target['x'])
        self.target_y = float(target['y'])
        self.target_z = float(target.get('z', 0.75))
        self.camera_height = float(orbit.get('camera_height', 0.43))
        self.desired_radius = float(orbit['radius'])
        self.linear_speed = min(
            float(orbit['linear_speed']),
            float(safety.get('max_linear_speed', 0.35)),
        )
        self.base_frame = cfg.get('frames', {}).get('base_frame', 'base_link')
        self.target_frame = cfg.get('frames', {}).get('target_frame', 'map')
        self.pan_limit = float(safety.get('pan_limit', math.pi))
        self.tilt_lower_limit = float(safety.get('tilt_lower_limit', -0.9))
        self.tilt_upper_limit = float(safety.get('tilt_upper_limit', 0.7))

        self.heading_kp = 2.2
        self.radius_kp = 1.4
        self.max_angular_speed = float(safety.get('max_angular_speed', 1.2))
        self.required_sweep = 2.0 * math.pi
        self.state = 'WAIT_FOR_GOAL'
        self.orbit_active = False
        self.last_phase = None
        self.accumulated_angle = 0.0
        self.orbit_complete_sent = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gimbal_pub = self.create_publisher(
            Float64MultiArray, '/camera_gimbal_position_controller/commands', 10
        )
        self.complete_pub = self.create_publisher(Empty, '/mission/orbit_complete', 10)
        self.status_pub = self.create_publisher(String, '/mission/orbit_status', 10)
        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.state_sub = self.create_subscription(String, '/mission/state', self.on_state, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def publish_event(self, event: str) -> None:
        self.event_pub.publish(String(data=event))

    def on_state(self, msg: String) -> None:
        self.state = msg.data
        if self.state == 'ORBITING' and not self.orbit_active:
            self.orbit_active = True
            self.last_phase = None
            self.accumulated_angle = 0.0
            self.orbit_complete_sent = False
            self.publish_event('ORBIT_STARTED')
            self.get_logger().info('Orbit controller engaged.')
        elif self.state != 'ORBITING' and self.orbit_active:
            self.orbit_active = False
            self.stop_robot()

    def control_loop(self) -> None:
        if not self.orbit_active or self.orbit_complete_sent:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Waiting for transform {self.target_frame}->{self.base_frame}: {exc}')
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)

        dx = tx - self.target_x
        dy = ty - self.target_y
        distance = math.hypot(dx, dy)
        phase = math.atan2(dy, dx)

        if self.last_phase is None:
            self.last_phase = phase
        delta_phase = normalize_angle(phase - self.last_phase)
        self.accumulated_angle += abs(delta_phase)
        self.last_phase = phase

        tangent_heading = phase + math.pi / 2.0
        heading_error = normalize_angle(tangent_heading - yaw)
        radius_error = distance - self.desired_radius

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = (
            (self.linear_speed / max(self.desired_radius, 0.1))
            + (self.heading_kp * heading_error)
            + (self.radius_kp * radius_error)
        )
        cmd.angular.z = clamp(cmd.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

        pan = normalize_angle(math.atan2(self.target_y - ty, self.target_x - tx) - yaw)
        pan = clamp(pan, -self.pan_limit, self.pan_limit)
        horizontal = max(math.hypot(self.target_x - tx, self.target_y - ty), 0.05)
        tilt = math.atan2(self.target_z - self.camera_height, horizontal)
        tilt = clamp(tilt, self.tilt_lower_limit, self.tilt_upper_limit)
        self.gimbal_pub.publish(Float64MultiArray(data=[pan, tilt]))

        progress = min(self.accumulated_angle / self.required_sweep, 1.0)
        self.status_pub.publish(String(data=json.dumps({
            'active': True,
            'progress': progress,
            'accumulated_angle': self.accumulated_angle,
            'radius_error': radius_error,
            'distance_to_target': distance,
            'pan_command': pan,
            'tilt_command': tilt,
        })))

        if self.accumulated_angle >= self.required_sweep:
            self.stop_robot()
            self.complete_pub.publish(Empty())
            self.orbit_complete_sent = True
            self.orbit_active = False
            self.publish_event('ORBIT_CAPTURE_FINISHED')
            self.status_pub.publish(String(data=json.dumps({'active': False, 'progress': 1.0})))
            self.get_logger().info('Completed full 360-degree orbit.')

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrbitControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
