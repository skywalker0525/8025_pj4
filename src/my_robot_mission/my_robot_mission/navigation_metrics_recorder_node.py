from datetime import datetime
import csv
import json
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_mission.utils import expand_path, quaternion_to_yaw


def stamp_sec(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1.0e-9


class NavigationMetricsRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('navigation_metrics_recorder_node')
        self.declare_parameter('output_dir', '~/livo_image_pose_exports')
        self.declare_parameter('run_name', '')
        self.declare_parameter('mission_event_topic', '/mission/events')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('pose_parent_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sample_period_sec', 0.50)
        self.declare_parameter('record_states', ['NAVIGATING'])
        self.declare_parameter('finish_states', ['DONE', 'STOPPED'])
        self.declare_parameter('tf_lookup_timeout_sec', 0.15)

        run_name = self.get_parameter('run_name').value
        if not run_name:
            run_name = datetime.now().strftime('capture_%Y%m%d_%H%M%S')
        base_output_dir = expand_path(self.get_parameter('output_dir').value)
        self.output_dir = os.path.join(base_output_dir, run_name)
        os.makedirs(self.output_dir, exist_ok=True)

        self.run_name = run_name
        self.event_topic = self.get_parameter('mission_event_topic').value
        self.state_topic = self.get_parameter('mission_state_topic').value
        self.pose_parent_frame = self.get_parameter('pose_parent_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.record_states = [
            str(state) for state in self.get_parameter('record_states').value if str(state)
        ]
        self.finish_states = [
            str(state) for state in self.get_parameter('finish_states').value if str(state)
        ]
        self.tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)

        self.events_csv_path = os.path.join(self.output_dir, 'navigation_events.csv')
        self.trajectory_csv_path = os.path.join(self.output_dir, 'navigation_trajectory.csv')
        self.summary_json_path = os.path.join(self.output_dir, 'navigation_summary.json')

        self.event_handle = open(self.events_csv_path, 'w', newline='', encoding='utf-8')
        self.event_writer = csv.writer(self.event_handle)
        self.event_writer.writerow(['event_index', 'wall_time', 'ros_time', 'state', 'event'])

        self.trajectory_handle = open(self.trajectory_csv_path, 'w', newline='', encoding='utf-8')
        self.trajectory_writer = csv.writer(self.trajectory_handle)
        self.trajectory_writer.writerow([
            'sample_index',
            'wall_time',
            'ros_time',
            'state',
            'parent_frame',
            'base_frame',
            'x',
            'y',
            'yaw',
            'step_distance_m',
            'path_length_m',
        ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, self.event_topic, self.on_event, 50)
        self.create_subscription(String, self.state_topic, self.on_state, 10)
        self.create_timer(max(0.05, self.sample_period_sec), self.sample_pose)

        self.current_state = ''
        self.final_state: Optional[str] = None
        self.events: List[str] = []
        self.expected_waypoints: List[str] = []
        self.started_waypoints: List[str] = []
        self.reached_waypoints: List[str] = []
        self.failed_waypoints: List[Dict[str, str]] = []
        self.failed_statuses: List[str] = []
        self.video_records: List[Dict[str, str]] = []
        self.livo_export: Optional[Dict[str, str]] = None
        self.pointcloud_export: Optional[Dict[str, str]] = None
        self.navigation_success = False
        self.apriltag_reached = False
        self.completed_with_failures = False
        self.done = False
        self.start_ros_time: Optional[float] = None
        self.end_ros_time: Optional[float] = None
        self.start_wall_time = time.time()
        self.last_pose: Optional[Tuple[float, float]] = None
        self.path_length_m = 0.0
        self.pose_samples = 0
        self.tf_skip_count = 0

        self.write_summary()
        self.get_logger().info(f'Recording navigation metrics to {self.output_dir}')

    def on_state(self, msg: String) -> None:
        self.current_state = msg.data
        if self.current_state in self.finish_states:
            self.done = True
            self.final_state = self.final_state or self.current_state
            self.end_ros_time = self.end_ros_time or stamp_sec(self)
            self.write_summary()

    def on_event(self, msg: String) -> None:
        event = msg.data.strip()
        if not event:
            return
        ros_time = stamp_sec(self)
        self.events.append(event)
        self.event_writer.writerow([
            len(self.events),
            f'{time.time():.6f}',
            f'{ros_time:.9f}',
            self.current_state,
            event,
        ])
        self.event_handle.flush()
        self.update_from_event(event, ros_time)
        self.write_summary()

    def update_from_event(self, event: str, ros_time: float) -> None:
        if event.startswith('AUTO_SEQUENCE_STARTED:'):
            raw_sequence = event.split(':', 1)[1]
            self.expected_waypoints = [item for item in raw_sequence.split(',') if item]
            self.start_ros_time = self.start_ros_time or ros_time
            return
        if event.startswith('NAVIGATION_STARTED:'):
            waypoint = event.split(':', 1)[1]
            self.started_waypoints.append(waypoint)
            self.start_ros_time = self.start_ros_time or ros_time
            return
        if event.startswith('NAVIGATION_WAYPOINT_REACHED:'):
            self.reached_waypoints.append(event.split(':', 1)[1])
            return
        if event.startswith('NAVIGATION_WAYPOINT_FAILED:'):
            parts = event.split(':')
            waypoint = parts[1] if len(parts) > 1 else ''
            status = parts[2] if len(parts) > 2 else ''
            self.failed_waypoints.append({'waypoint': waypoint, 'status': status})
            return
        if event.startswith('NAVIGATION_FAILED_STATUS_'):
            self.failed_statuses.append(event.rsplit('_', 1)[-1])
            return
        if event == 'NAVIGATION_SUCCEEDED':
            self.navigation_success = True
            self.done = True
            self.final_state = self.final_state or 'DONE'
            self.end_ros_time = ros_time
            return
        if event == 'APRILTAG_REACHED' or event == 'NAVIGATION_FALLBACK_APRILTAG_REACHED':
            self.apriltag_reached = True
            return
        if event.startswith('AUTO_SEQUENCE_COMPLETED_WITH_FAILURES:'):
            self.completed_with_failures = True
            self.done = True
            self.final_state = self.final_state or 'DONE'
            self.end_ros_time = ros_time
            return
        if event.startswith('LIVO_EXPORT_DONE:'):
            parts = event.split(':')
            self.livo_export = {
                'output_dir': parts[1] if len(parts) > 1 else '',
                'image_pose_pairs': parts[2] if len(parts) > 2 else '',
            }
            return
        if event.startswith('POINTCLOUD_EXPORT_DONE:'):
            parts = event.split(':')
            self.pointcloud_export = {
                'output_dir': parts[1] if len(parts) > 1 else '',
                'scan_count': parts[2] if len(parts) > 2 else '',
                'point_count': parts[3] if len(parts) > 3 else '',
            }
            return
        if event.startswith('VIDEO_RECORDING_DONE:'):
            parts = event.split(':')
            self.video_records.append({
                'path': parts[1] if len(parts) > 1 else '',
                'frame_count': parts[2] if len(parts) > 2 else '',
            })

    def sample_pose(self) -> None:
        if self.done:
            return
        if self.record_states and self.current_state not in self.record_states:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.pose_parent_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException:
            self.tf_skip_count += 1
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        x = float(t.x)
        y = float(t.y)
        yaw = quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        step = 0.0
        if self.last_pose is not None:
            step = math.hypot(x - self.last_pose[0], y - self.last_pose[1])
            if step < 5.0:
                self.path_length_m += step
        self.last_pose = (x, y)
        self.pose_samples += 1
        self.trajectory_writer.writerow([
            self.pose_samples,
            f'{time.time():.6f}',
            f'{stamp_sec(self):.9f}',
            self.current_state,
            self.pose_parent_frame,
            self.base_frame,
            f'{x:.6f}',
            f'{y:.6f}',
            f'{yaw:.6f}',
            f'{step:.6f}',
            f'{self.path_length_m:.6f}',
        ])
        self.trajectory_handle.flush()
        if self.pose_samples % 10 == 0:
            self.write_summary()

    def write_summary(self) -> None:
        runtime_ros = None
        if self.start_ros_time is not None:
            runtime_ros = (self.end_ros_time or stamp_sec(self)) - self.start_ros_time
        summary = {
            'run_name': self.run_name,
            'output_dir': self.output_dir,
            'created_at': datetime.now().isoformat(timespec='seconds'),
            'mission_event_topic': self.event_topic,
            'mission_state_topic': self.state_topic,
            'pose_parent_frame': self.pose_parent_frame,
            'base_frame': self.base_frame,
            'final_state': self.final_state or self.current_state,
            'expected_waypoints': self.expected_waypoints,
            'started_waypoints': self.started_waypoints,
            'reached_waypoints': self.reached_waypoints,
            'failed_waypoints': self.failed_waypoints,
            'failed_statuses': self.failed_statuses,
            'expected_waypoint_count': len(self.expected_waypoints),
            'started_waypoint_count': len(self.started_waypoints),
            'reached_waypoint_count': len(self.reached_waypoints),
            'failed_waypoint_count': len(self.failed_waypoints),
            'navigation_success': self.navigation_success,
            'apriltag_reached': self.apriltag_reached,
            'completed_with_failures': self.completed_with_failures,
            'done': self.done,
            'runtime_ros_sec': runtime_ros,
            'runtime_wall_sec': time.time() - self.start_wall_time,
            'path_length_m': self.path_length_m,
            'trajectory_samples': self.pose_samples,
            'tf_skip_count': self.tf_skip_count,
            'event_count': len(self.events),
            'livo_export': self.livo_export,
            'pointcloud_export': self.pointcloud_export,
            'video_records': self.video_records,
        }
        with open(self.summary_json_path, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2)

    def close(self) -> None:
        self.write_summary()
        if self.event_handle is not None:
            self.event_handle.close()
            self.event_handle = None
        if self.trajectory_handle is not None:
            self.trajectory_handle.close()
            self.trajectory_handle = None


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavigationMetricsRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
