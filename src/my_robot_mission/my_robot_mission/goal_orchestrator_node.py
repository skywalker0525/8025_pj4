from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
import math
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_mission.utils import load_yaml_file, yaw_to_quaternion


class GoalOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__('goal_orchestrator_node')
        self.declare_parameter('waypoints_file', '')

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            raise RuntimeError('waypoints_file parameter is required.')
        cfg = load_yaml_file(waypoints_file)
        self.goal_b = cfg['points']['B']
        self.apriltag_target = cfg.get('apriltag_target', {})
        frames = cfg.get('frames', {})
        self.target_frame = frames.get('target_frame', 'map')
        self.base_frame = frames.get('base_frame', 'base_link')
        self.tag_x = float(self.apriltag_target.get('x', self.goal_b['x']))
        self.tag_y = float(self.apriltag_target.get('y', self.goal_b['y']))
        self.tag_completion_radius = float(self.apriltag_target.get('completion_radius', 1.4))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.state_pub = self.create_publisher(String, '/mission/state', qos)
        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.command_sub = self.create_subscription(String, '/mission/command', self.on_command, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'WAIT_FOR_GOAL'
        self.goal_handle = None
        self.finish_timer = None
        self.nav_ready_announced = False
        self.readiness_timer = self.create_timer(2.0, self.publish_readiness_when_available)
        self.publish_state(self.state)

    def publish_event(self, event: str) -> None:
        self.event_pub.publish(String(data=event))
        self.get_logger().info(event)

    def publish_state(self, state: str) -> None:
        self.state = state
        self.state_pub.publish(String(data=state))
        self.get_logger().info(f'Mission state -> {state}')

    def nav2_ready(self, publish_failure: bool = True, publish_success: bool = True) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            if publish_failure:
                self.publish_event('NAV2_NOT_READY:ACTION_SERVER_UNAVAILABLE')
            return False
        try:
            self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            if publish_failure:
                self.publish_event(f'NAV2_NOT_READY:TF_{self.target_frame}_TO_{self.base_frame}_UNAVAILABLE')
                self.get_logger().warn(f'Navigation TF is not ready: {exc}')
            return False
        if publish_success:
            self.publish_event('NAV2_READY')
        return True

    def publish_readiness_when_available(self) -> None:
        if self.nav_ready_announced:
            return
        if self.nav2_ready(publish_failure=False, publish_success=False):
            self.nav_ready_announced = True
            self.publish_event('NAV2_READY')

    def on_command(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == 'START_AUTO':
            self.publish_event('AUTO_MISSION_REQUESTED')
            if not self.nav2_ready(publish_success=not self.nav_ready_announced):
                return
            self.nav_ready_announced = True
            self.send_named_goal_b()
        elif command == 'STOP':
            self.publish_event('MISSION_STOP_REQUESTED')
            self.stop_mission(reset=False)
        elif command == 'RESET':
            self.publish_event('MISSION_RESET')
            self.stop_mission(reset=True)
        else:
            self.get_logger().warn(f'Unknown mission command: {msg.data}')

    def send_named_goal_b(self) -> None:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.goal_b['x'])
        pose.pose.position.y = float(self.goal_b['y'])
        quat = yaw_to_quaternion(float(self.goal_b.get('yaw', 0.0)))
        pose.pose.orientation.x = quat['x']
        pose.pose.orientation.y = quat['y']
        pose.pose.orientation.z = quat['z']
        pose.pose.orientation.w = quat['w']
        self.on_goal_pose(pose)

    def stop_mission(self, reset: bool) -> None:
        self.cmd_vel_pub.publish(Twist())
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        if self.finish_timer is not None:
            self.finish_timer.cancel()
            self.finish_timer = None
        self.publish_state('WAIT_FOR_GOAL' if reset else 'STOPPED')

    def on_goal_pose(self, msg: PoseStamped) -> None:
        if self.state == 'NAVIGATING':
            self.get_logger().warn('Ignoring new goal while mission is busy.')
            return

        if not self.nav2_ready(publish_success=not self.nav_ready_announced):
            return
        self.nav_ready_announced = True

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.publish_state('NAVIGATING')
        self.publish_event('NAVIGATION_STARTED')
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal.')
            self.publish_event('NAVIGATION_REJECTED')
            self.publish_state('WAIT_FOR_GOAL')
            return

        self.goal_handle = goal_handle
        self.publish_event('NAVIGATION_ACCEPTED')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error('Navigation result did not return.')
            self.publish_event('NAVIGATION_RESULT_MISSING')
            self.publish_state('WAIT_FOR_GOAL')
            self.goal_handle = None
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_event('NAVIGATION_SUCCEEDED')
            self.publish_event('APRILTAG_REACHED')
            self.publish_state('DONE')
        else:
            self.get_logger().warn(f'Navigation did not succeed. Status code: {result.status}')
            self.publish_event(f'NAVIGATION_FAILED_STATUS_{result.status}')
            if self.robot_is_near_apriltag():
                self.publish_event('NAVIGATION_FALLBACK_APRILTAG_REACHED')
                self.publish_state('DONE')
            else:
                self.publish_state('WAIT_FOR_GOAL')
        self.goal_handle = None

    def robot_is_near_apriltag(self) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Cannot check AprilTag fallback pose: {exc}')
            return False

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        distance = math.hypot(x - self.tag_x, y - self.tag_y)
        is_near = distance <= self.tag_completion_radius
        self.get_logger().info(
            f'AprilTag fallback distance: {distance:.2f} m '
            f'(completion radius {self.tag_completion_radius:.2f} m).'
        )
        return is_near


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
