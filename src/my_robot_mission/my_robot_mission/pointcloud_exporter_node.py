from datetime import datetime
import csv
import json
import math
import os
import shutil
import time
from typing import List, Optional, Sequence, Tuple

from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_mission.utils import expand_path


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quat_xyzw_to_matrix(x: float, y: float, z: float, w: float) -> List[List[float]]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


class PointCloudExporterNode(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_exporter_node')
        self.declare_parameter('pointcloud_topic', '/points_raw')
        self.declare_parameter('overhead_image_topic', '/overhead_rgb_sensor/image_raw')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('record_states', ['NAVIGATING'])
        self.declare_parameter('finish_states', ['DONE', 'STOPPED'])
        self.declare_parameter('pose_parent_frame', 'map')
        self.declare_parameter('output_dir', '~/livo_image_pose_exports')
        self.declare_parameter('run_name', '')
        self.declare_parameter('sample_period_sec', 0.50)
        self.declare_parameter('point_stride', 4)
        self.declare_parameter('max_scans', 0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.20)
        self.declare_parameter('tf_use_latest_on_extrapolation', True)
        self.declare_parameter('tf_warning_period_sec', 5.0)
        self.declare_parameter('overlay_filename', 'overhead_lidar.mp4')
        self.declare_parameter('overlay_fps', 10.0)
        self.declare_parameter('max_overlay_points', 1800)
        self.declare_parameter('overhead_camera_x', 0.0)
        self.declare_parameter('overhead_camera_y', 0.0)
        self.declare_parameter('overhead_camera_z', 24.0)
        self.declare_parameter('overhead_camera_yaw', 0.0)
        self.declare_parameter('overhead_horizontal_fov', 1.55)
        self.declare_parameter('publish_events', True)

        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.overhead_image_topic = self.get_parameter('overhead_image_topic').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.record_states = [
            str(state) for state in self.get_parameter('record_states').value if str(state)
        ]
        self.finish_states = [
            str(state) for state in self.get_parameter('finish_states').value if str(state)
        ]
        self.pose_parent_frame = self.get_parameter('pose_parent_frame').value
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.point_stride = max(1, int(self.get_parameter('point_stride').value))
        self.max_scans = int(self.get_parameter('max_scans').value)
        self.tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self.tf_use_latest_on_extrapolation = bool(
            self.get_parameter('tf_use_latest_on_extrapolation').value
        )
        self.tf_warning_period_sec = float(self.get_parameter('tf_warning_period_sec').value)
        self.overlay_filename = self.get_parameter('overlay_filename').value
        self.overlay_fps = float(self.get_parameter('overlay_fps').value)
        self.max_overlay_points = max(0, int(self.get_parameter('max_overlay_points').value))
        self.overhead_camera_x = float(self.get_parameter('overhead_camera_x').value)
        self.overhead_camera_y = float(self.get_parameter('overhead_camera_y').value)
        self.overhead_camera_z = float(self.get_parameter('overhead_camera_z').value)
        self.overhead_camera_yaw = float(self.get_parameter('overhead_camera_yaw').value)
        self.overhead_horizontal_fov = float(self.get_parameter('overhead_horizontal_fov').value)
        self.publish_events = bool(self.get_parameter('publish_events').value)

        run_name = self.get_parameter('run_name').value
        if not run_name:
            run_name = datetime.now().strftime('capture_%Y%m%d_%H%M%S')
        base_output_dir = expand_path(self.get_parameter('output_dir').value)
        self.output_dir = os.path.join(base_output_dir, run_name)
        self.point_cloud_dir = os.path.join(self.output_dir, 'point_cloud')
        os.makedirs(self.point_cloud_dir, exist_ok=True)

        self.csv_path = os.path.join(self.point_cloud_dir, 'lidar_points_world.csv')
        self.ply_path = os.path.join(self.point_cloud_dir, 'lidar_points_world.ply')
        self.ply_body_path = os.path.join(self.point_cloud_dir, '.lidar_points_world.body')
        self.scan_metadata_path = os.path.join(self.point_cloud_dir, 'lidar_scan_metadata.json')
        self.overlay_path = os.path.join(self.output_dir, self.overlay_filename)
        self.overlay_metadata_path = os.path.join(
            self.output_dir,
            f'{os.path.splitext(self.overlay_filename)[0]}_metadata.json',
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_mission_state = ''
        self.done_announced = False
        self.csv_handle = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_handle)
        self.csv_writer.writerow([
            'scan_index',
            'timestamp',
            'source_frame',
            'parent_frame',
            'x',
            'y',
            'z',
            'range_m',
        ])
        self.ply_body_handle = open(self.ply_body_path, 'w', encoding='utf-8')
        self.overlay_writer: Optional[cv2.VideoWriter] = None
        self.last_scan_stamp_ns: Optional[int] = None
        self.latest_overlay_points: List[Tuple[float, float, float]] = []
        self.scan_count = 0
        self.point_count = 0
        self.overlay_frame_count = 0
        self.tf_exact_count = 0
        self.tf_latest_fallback_count = 0
        self.tf_skip_count = 0
        self.last_tf_warning_monotonic = 0.0

        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_pointcloud, 10)
        self.create_subscription(Image, self.overhead_image_topic, self.on_overhead_image, 10)
        self.create_subscription(String, self.mission_state_topic, self.on_mission_state, 10)
        self.write_metadata()
        self.publish_event(f'POINTCLOUD_EXPORT_STARTED:{self.point_cloud_dir}')
        self.get_logger().info(
            f'Exporting {self.pointcloud_topic} to {self.point_cloud_dir} in {self.pose_parent_frame} frame.'
        )

    def on_mission_state(self, msg: String) -> None:
        self.current_mission_state = msg.data
        if self.current_mission_state in self.finish_states and self.scan_count > 0:
            self.announce_done_once()

    def should_record(self) -> bool:
        if self.done_announced:
            return False
        if self.record_states and self.current_mission_state not in self.record_states:
            return False
        if self.max_scans > 0 and self.scan_count >= self.max_scans:
            self.announce_done_once()
            return False
        return True

    def on_pointcloud(self, msg: PointCloud2) -> None:
        if not self.should_record():
            return
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if (
            self.last_scan_stamp_ns is not None
            and stamp_ns - self.last_scan_stamp_ns < int(self.sample_period_sec * 1e9)
        ):
            return
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.pose_parent_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
            self.tf_exact_count += 1
        except TransformException as exact_exc:
            if self.tf_use_latest_on_extrapolation:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.pose_parent_frame,
                        msg.header.frame_id,
                        Time(),
                        timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                    )
                    self.tf_latest_fallback_count += 1
                except TransformException as fallback_exc:
                    self.tf_skip_count += 1
                    self.warn_tf_skip_throttled(msg, fallback_exc, exact_exc)
                    return
            else:
                self.tf_skip_count += 1
                self.warn_tf_skip_throttled(msg, exact_exc)
                return

        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = quat_xyzw_to_matrix(q.x, q.y, q.z, q.w)
        translation = (float(t.x), float(t.y), float(t.z))
        timestamp = stamp_to_float(msg.header.stamp)
        scan_index = self.scan_count + 1
        points_for_overlay: List[Tuple[float, float, float]] = []
        written_this_scan = 0

        for raw_index, point in enumerate(
            point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ):
            if raw_index % self.point_stride != 0:
                continue
            lx, ly, lz = float(point[0]), float(point[1]), float(point[2])
            wx = rotation[0][0] * lx + rotation[0][1] * ly + rotation[0][2] * lz + translation[0]
            wy = rotation[1][0] * lx + rotation[1][1] * ly + rotation[1][2] * lz + translation[1]
            wz = rotation[2][0] * lx + rotation[2][1] * ly + rotation[2][2] * lz + translation[2]
            range_m = math.sqrt(lx * lx + ly * ly + lz * lz)
            self.csv_writer.writerow([
                scan_index,
                f'{timestamp:.9f}',
                msg.header.frame_id,
                self.pose_parent_frame,
                f'{wx:.6f}',
                f'{wy:.6f}',
                f'{wz:.6f}',
                f'{range_m:.6f}',
            ])
            self.ply_body_handle.write(f'{wx:.6f} {wy:.6f} {wz:.6f} 255 210 35\n')
            if len(points_for_overlay) < self.max_overlay_points and -0.5 <= wz <= 3.0:
                points_for_overlay.append((wx, wy, wz))
            written_this_scan += 1

        self.csv_handle.flush()
        self.ply_body_handle.flush()
        self.latest_overlay_points = points_for_overlay
        self.last_scan_stamp_ns = stamp_ns
        self.scan_count = scan_index
        self.point_count += written_this_scan
        if self.scan_count % 10 == 0 or self.scan_count == 1:
            self.get_logger().info(
                f'Exported {self.scan_count} point-cloud scans, {self.point_count} points.'
            )
        self.write_metadata()
        if self.max_scans > 0 and self.scan_count >= self.max_scans:
            self.announce_done_once()

    def warn_tf_skip_throttled(
        self,
        msg: PointCloud2,
        exc: TransformException,
        exact_exc: Optional[TransformException] = None,
    ) -> None:
        now = time.monotonic()
        if now - self.last_tf_warning_monotonic < self.tf_warning_period_sec:
            return
        self.last_tf_warning_monotonic = now
        detail = f'{exc}'
        if exact_exc is not None:
            detail = f'exact lookup failed: {exact_exc}; latest fallback failed: {exc}'
        self.get_logger().warn(
            f'Skipping point cloud at {stamp_to_float(msg.header.stamp):.6f}: TF unavailable: {detail}'
        )

    def on_overhead_image(self, msg: Image) -> None:
        if self.done_announced:
            return
        if self.record_states and self.current_mission_state not in self.record_states:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.overlay_writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.overlay_writer = cv2.VideoWriter(self.overlay_path, fourcc, self.overlay_fps, (width, height))
            if not self.overlay_writer.isOpened():
                self.get_logger().error(f'Failed to open overlay video writer for {self.overlay_path}')
                self.overlay_writer = None
                self.announce_done_once()
                return
            self.publish_event(f'POINTCLOUD_OVERLAY_STARTED:{self.overlay_path}')

        self.draw_lidar_overlay(frame)
        self.overlay_writer.write(frame)
        self.overlay_frame_count += 1

    def draw_lidar_overlay(self, frame) -> None:
        height, width = frame.shape[:2]
        view_width = 2.0 * self.overhead_camera_z * math.tan(0.5 * self.overhead_horizontal_fov)
        view_height = view_width * float(height) / float(width)
        c = math.cos(-self.overhead_camera_yaw)
        s = math.sin(-self.overhead_camera_yaw)
        for x, y, _z in self.latest_overlay_points:
            dx = x - self.overhead_camera_x
            dy = y - self.overhead_camera_y
            rx = c * dx - s * dy
            ry = s * dx + c * dy
            u = int((rx + 0.5 * view_width) / view_width * width)
            v = int((0.5 * view_height - ry) / view_height * height)
            if 0 <= u < width and 0 <= v < height:
                cv2.circle(frame, (u, v), 2, (0, 230, 255), -1, cv2.LINE_AA)
        cv2.putText(
            frame,
            f'Gazebo overhead + /points_raw | scans={self.scan_count} points={self.point_count}',
            (14, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (20, 20, 20),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f'Gazebo overhead + /points_raw | scans={self.scan_count} points={self.point_count}',
            (14, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )

    def finalize_ply(self) -> None:
        if self.ply_body_handle is not None:
            self.ply_body_handle.close()
            self.ply_body_handle = None
        with open(self.ply_path, 'w', encoding='utf-8') as ply_handle:
            ply_handle.write('ply\n')
            ply_handle.write('format ascii 1.0\n')
            ply_handle.write(f'element vertex {self.point_count}\n')
            ply_handle.write('property float x\n')
            ply_handle.write('property float y\n')
            ply_handle.write('property float z\n')
            ply_handle.write('property uchar red\n')
            ply_handle.write('property uchar green\n')
            ply_handle.write('property uchar blue\n')
            ply_handle.write('end_header\n')
            if os.path.exists(self.ply_body_path):
                with open(self.ply_body_path, 'r', encoding='utf-8') as body_handle:
                    shutil.copyfileobj(body_handle, ply_handle)
        if os.path.exists(self.ply_body_path):
            os.remove(self.ply_body_path)

    def close_outputs(self) -> None:
        if self.overlay_writer is not None:
            self.overlay_writer.release()
            self.overlay_writer = None
        if self.csv_handle is not None:
            self.csv_handle.close()
            self.csv_handle = None
        self.finalize_ply()
        self.write_metadata()

    def write_metadata(self) -> None:
        metadata = {
            'pointcloud_topic': self.pointcloud_topic,
            'overhead_image_topic': self.overhead_image_topic,
            'mission_state_topic': self.mission_state_topic,
            'record_states': self.record_states,
            'finish_states': self.finish_states,
            'pose_parent_frame': self.pose_parent_frame,
            'sample_period_sec': self.sample_period_sec,
            'point_stride': self.point_stride,
            'tf_lookup_timeout_sec': self.tf_lookup_timeout_sec,
            'tf_use_latest_on_extrapolation': self.tf_use_latest_on_extrapolation,
            'tf_exact_count': self.tf_exact_count,
            'tf_latest_fallback_count': self.tf_latest_fallback_count,
            'tf_skip_count': self.tf_skip_count,
            'scan_count': self.scan_count,
            'point_count': self.point_count,
            'overlay_frame_count': self.overlay_frame_count,
            'csv_path': self.csv_path,
            'ply_path': self.ply_path,
            'overlay_path': self.overlay_path,
        }
        with open(self.scan_metadata_path, 'w', encoding='utf-8') as handle:
            json.dump(metadata, handle, indent=2)
        with open(self.overlay_metadata_path, 'w', encoding='utf-8') as handle:
            json.dump(metadata, handle, indent=2)

    def publish_event(self, event: str) -> None:
        if self.publish_events:
            self.event_pub.publish(String(data=event))

    def announce_done_once(self) -> None:
        if self.done_announced:
            return
        self.done_announced = True
        self.close_outputs()
        self.publish_event(f'POINTCLOUD_EXPORT_DONE:{self.point_cloud_dir}:{self.scan_count}:{self.point_count}')
        self.get_logger().info(
            f'Finished point-cloud export: {self.scan_count} scans, {self.point_count} points in {self.point_cloud_dir}'
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = PointCloudExporterNode()
    try:
        rclpy.spin(node)
    finally:
        node.announce_done_once()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
