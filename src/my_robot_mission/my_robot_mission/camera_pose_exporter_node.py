from collections import deque
from datetime import datetime
import csv
import json
import math
import os
from typing import Dict, List, Optional, Tuple

from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_mission.utils import expand_path


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def normalize_quat_xyzw(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quat_xyzw_to_matrix(q: Tuple[float, float, float, float]) -> List[List[float]]:
    x, y, z, w = normalize_quat_xyzw(q)
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


def matrix_to_quat_wxyz(r: List[List[float]]) -> Tuple[float, float, float, float]:
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    qx, qy, qz, qw = normalize_quat_xyzw((qx, qy, qz, qw))
    return qw, qx, qy, qz


def transform_to_matrix(transform: TransformStamped) -> List[List[float]]:
    t = transform.transform.translation
    q = transform.transform.rotation
    r = quat_xyzw_to_matrix((q.x, q.y, q.z, q.w))
    return [
        [r[0][0], r[0][1], r[0][2], t.x],
        [r[1][0], r[1][1], r[1][2], t.y],
        [r[2][0], r[2][1], r[2][2], t.z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def invert_pose_from_transform(transform: TransformStamped) -> Tuple[Tuple[float, float, float, float], Tuple[float, float, float]]:
    t = transform.transform.translation
    q = transform.transform.rotation
    r_wc = quat_xyzw_to_matrix((q.x, q.y, q.z, q.w))
    r_cw = [
        [r_wc[0][0], r_wc[1][0], r_wc[2][0]],
        [r_wc[0][1], r_wc[1][1], r_wc[2][1]],
        [r_wc[0][2], r_wc[1][2], r_wc[2][2]],
    ]
    twc = [t.x, t.y, t.z]
    tcw = tuple(-sum(r_cw[row][col] * twc[col] for col in range(3)) for row in range(3))
    return matrix_to_quat_wxyz(r_cw), tcw


def frame_to_transform(frame: Dict) -> TransformStamped:
    matrix = frame['transform_matrix']
    transform = TransformStamped()
    transform.transform.translation.x = matrix[0][3]
    transform.transform.translation.y = matrix[1][3]
    transform.transform.translation.z = matrix[2][3]
    qw, qx, qy, qz = matrix_to_quat_wxyz([
        [matrix[0][0], matrix[0][1], matrix[0][2]],
        [matrix[1][0], matrix[1][1], matrix[1][2]],
        [matrix[2][0], matrix[2][1], matrix[2][2]],
    ])
    transform.transform.rotation.w = qw
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    return transform


class CameraPoseExporterNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_pose_exporter_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('pointcloud_topic', '/points_raw')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('record_states', [''])
        self.declare_parameter('finish_states', ['DONE', 'STOPPED'])
        self.declare_parameter('pose_parent_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('pose_source_note', '')
        self.declare_parameter('output_dir', '~/livo_image_pose_exports')
        self.declare_parameter('run_name', '')
        self.declare_parameter('max_frames', 0)
        self.declare_parameter('sample_period_sec', 0.25)
        self.declare_parameter('tf_lookup_delay_sec', 0.20)
        self.declare_parameter('max_tf_wait_sec', 3.0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.15)
        self.declare_parameter('require_lio_inputs', True)
        self.declare_parameter('publish_events', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.record_states = [
            str(state) for state in self.get_parameter('record_states').value if str(state)
        ]
        self.finish_states = [
            str(state) for state in self.get_parameter('finish_states').value if str(state)
        ]
        self.pose_parent_frame = self.get_parameter('pose_parent_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.pose_source_note = self.get_parameter('pose_source_note').value
        self.max_frames = int(self.get_parameter('max_frames').value)
        self.sample_period_sec = float(self.get_parameter('sample_period_sec').value)
        self.tf_lookup_delay_sec = float(self.get_parameter('tf_lookup_delay_sec').value)
        self.max_tf_wait_sec = float(self.get_parameter('max_tf_wait_sec').value)
        self.tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self.require_lio_inputs = bool(self.get_parameter('require_lio_inputs').value)
        self.publish_events = bool(self.get_parameter('publish_events').value)

        run_name = self.get_parameter('run_name').value
        if not run_name:
            run_name = datetime.now().strftime('capture_%Y%m%d_%H%M%S')
        base_output_dir = expand_path(self.get_parameter('output_dir').value)
        self.output_dir = os.path.join(base_output_dir, run_name)
        self.images_dir = os.path.join(self.output_dir, 'images')
        self.sparse_dir = os.path.join(self.output_dir, 'sparse', '0')
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.sparse_dir, exist_ok=True)

        self.poses_csv_path = os.path.join(self.output_dir, 'poses.csv')
        self.poses_colmap_w2c_path = os.path.join(self.output_dir, 'poses_colmap_w2c.txt')
        self.camera_centers_world_path = os.path.join(self.output_dir, 'camera_centers_world.txt')
        self.image_name_mapping_path = os.path.join(self.output_dir, 'image_name_mapping.csv')
        self.transforms_json_path = os.path.join(self.output_dir, 'transforms.json')
        self.metadata_json_path = os.path.join(self.output_dir, 'metadata.json')
        self.colmap_cameras_path = os.path.join(self.sparse_dir, 'cameras.txt')
        self.colmap_images_path = os.path.join(self.sparse_dir, 'images.txt')
        self.colmap_points_path = os.path.join(self.sparse_dir, 'points3D.txt')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending_images = deque(maxlen=300)
        self.frames: List[Dict] = []
        self.camera_info: Optional[CameraInfo] = None
        self.seen_imu = False
        self.seen_pointcloud = False
        self.last_queued_stamp_ns: Optional[int] = None
        self.current_mission_state = ''
        self.saved_count = 0
        self.done_announced = False
        self.waiting_logged = False

        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_pointcloud, 10)
        self.create_subscription(String, self.mission_state_topic, self.on_mission_state, 10)
        self.create_timer(0.05, self.flush_pending_images)

        self.init_output_files()
        self.publish_event(f'LIVO_EXPORT_STARTED:{self.output_dir}')
        self.get_logger().info(
            f'Exporting camera images and poses to {self.output_dir} '
            f'using {self.pose_parent_frame}->{self.camera_frame} at image timestamps.'
        )

    def init_output_files(self) -> None:
        with open(self.poses_csv_path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'image_id',
                'timestamp',
                'image_path',
                'parent_frame',
                'camera_frame',
                'tx',
                'ty',
                'tz',
                'qx',
                'qy',
                'qz',
                'qw',
            ])
        with open(self.colmap_points_path, 'w', encoding='utf-8') as handle:
            handle.write('# 3D point list is intentionally empty for pose/image export.\n')
        self.write_metadata()
        self.write_transforms_json()
        self.write_colmap_files()
        self.write_desktop_pose_files()

    def write_metadata(self) -> None:
        metadata = {
            'created_at': datetime.now().isoformat(timespec='seconds'),
            'image_topic': self.image_topic,
            'camera_info_topic': self.camera_info_topic,
            'imu_topic': self.imu_topic,
            'pointcloud_topic': self.pointcloud_topic,
            'mission_state_topic': self.mission_state_topic,
            'record_states': self.record_states,
            'finish_states': self.finish_states,
            'pose_parent_frame': self.pose_parent_frame,
            'camera_frame': self.camera_frame,
            'pose_source_note': self.pose_source_note,
            'pose_convention': 'T_parent_camera, ROS camera optical frame, camera-to-parent transform',
            'desktop_pose_files': {
                'poses_colmap_w2c.txt': 'name qw qx qy qz tx ty tz, COLMAP world-to-camera pose',
                'camera_centers_world.txt': 'name Cx Cy Cz in pose_parent_frame',
                'image_name_mapping.csv': 'identity mapping for exported segment image names',
            },
            'lio_note': (
                'This exporter requires IMU and PointCloud2 topics before saving. '
                'Use a LIO/FAST-LIVO2-style odometry/TF source as pose_parent_frame when available.'
            ),
        }
        with open(self.metadata_json_path, 'w', encoding='utf-8') as handle:
            json.dump(metadata, handle, indent=2)

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def on_imu(self, _msg: Imu) -> None:
        self.seen_imu = True

    def on_pointcloud(self, _msg: PointCloud2) -> None:
        self.seen_pointcloud = True

    def on_mission_state(self, msg: String) -> None:
        self.current_mission_state = msg.data
        if self.current_mission_state in self.finish_states and self.saved_count > 0:
            self.pending_images.clear()
            self.announce_done_once()
            return
        if self.record_states and self.current_mission_state not in self.record_states:
            self.pending_images.clear()

    def on_image(self, msg: Image) -> None:
        if self.max_frames > 0 and self.saved_count >= self.max_frames:
            self.announce_done_once()
            return
        if self.record_states and self.current_mission_state not in self.record_states:
            return
        if self.require_lio_inputs and not (self.seen_imu and self.seen_pointcloud):
            if not self.waiting_logged:
                self.get_logger().warn(
                    f'Waiting for LIO inputs before exporting images: '
                    f'imu={self.seen_imu}, pointcloud={self.seen_pointcloud}.'
                )
                self.waiting_logged = True
            return

        stamp_ns = stamp_to_ns(msg.header.stamp)
        if (
            self.last_queued_stamp_ns is not None
            and stamp_ns - self.last_queued_stamp_ns < int(self.sample_period_sec * 1e9)
        ):
            return
        self.last_queued_stamp_ns = stamp_ns
        self.pending_images.append(msg)

    def flush_pending_images(self) -> None:
        if not self.pending_images:
            return
        now_ns = self.get_clock().now().nanoseconds
        while self.pending_images:
            if self.max_frames > 0 and self.saved_count >= self.max_frames:
                self.pending_images.clear()
                self.announce_done_once()
                return
            msg = self.pending_images[0]
            image_stamp_ns = stamp_to_ns(msg.header.stamp)
            age_sec = (now_ns - image_stamp_ns) * 1e-9 if now_ns > 0 and image_stamp_ns > 0 else self.max_tf_wait_sec
            if age_sec < self.tf_lookup_delay_sec:
                return
            try:
                self.export_image(msg)
            except TransformException as exc:
                if age_sec < self.max_tf_wait_sec:
                    return
                self.get_logger().warn(
                    f'Skipping image at {stamp_to_float(msg.header.stamp):.6f}: TF unavailable: {exc}'
                )
            except Exception as exc:
                self.get_logger().error(f'Failed to export image pose pair: {exc}')
            finally:
                self.pending_images.popleft()

    def export_image(self, msg: Image) -> None:
        if self.max_frames > 0 and self.saved_count >= self.max_frames:
            self.announce_done_once()
            return
        transform = self.tf_buffer.lookup_transform(
            self.pose_parent_frame,
            self.camera_frame,
            Time.from_msg(msg.header.stamp),
            timeout=Duration(seconds=self.tf_lookup_timeout_sec),
        )
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_id = self.saved_count + 1
        image_name = f'{image_id:06d}.png'
        image_path = os.path.join(self.images_dir, image_name)
        if not cv2.imwrite(image_path, frame):
            raise RuntimeError(f'cv2.imwrite failed for {image_path}')

        t = transform.transform.translation
        q = transform.transform.rotation
        rel_image_path = os.path.join('images', image_name)
        timestamp = stamp_to_float(msg.header.stamp)
        with open(self.poses_csv_path, 'a', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                image_id,
                f'{timestamp:.9f}',
                rel_image_path,
                self.pose_parent_frame,
                self.camera_frame,
                f'{t.x:.9f}',
                f'{t.y:.9f}',
                f'{t.z:.9f}',
                f'{q.x:.12f}',
                f'{q.y:.12f}',
                f'{q.z:.12f}',
                f'{q.w:.12f}',
            ])

        self.frames.append({
            'image_id': image_id,
            'file_path': rel_image_path,
            'timestamp': timestamp,
            'transform_matrix': transform_to_matrix(transform),
        })
        self.saved_count = image_id
        self.write_transforms_json()
        self.write_colmap_files()
        self.write_desktop_pose_files()
        if self.saved_count % 10 == 0 or self.saved_count == 1:
            self.get_logger().info(f'Exported {self.saved_count} image/pose pairs.')
        if self.max_frames > 0 and self.saved_count >= self.max_frames:
            self.announce_done_once()

    def camera_intrinsics(self) -> Dict:
        if self.camera_info is None:
            return {}
        k = self.camera_info.k
        return {
            'w': int(self.camera_info.width),
            'h': int(self.camera_info.height),
            'fl_x': float(k[0]),
            'fl_y': float(k[4]),
            'cx': float(k[2]),
            'cy': float(k[5]),
            'distortion_model': self.camera_info.distortion_model,
            'distortion': [float(value) for value in self.camera_info.d],
        }

    def write_transforms_json(self) -> None:
        payload = {
            'camera_model': 'PINHOLE',
            'pose_convention': 'T_parent_camera, ROS optical frame',
            'parent_frame': self.pose_parent_frame,
            'camera_frame': self.camera_frame,
            **self.camera_intrinsics(),
            'frames': self.frames,
        }
        with open(self.transforms_json_path, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2)

    def write_colmap_files(self) -> None:
        with open(self.colmap_cameras_path, 'w', encoding='utf-8') as handle:
            handle.write('# Camera list with one line of data per camera:\n')
            handle.write('# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n')
            if self.camera_info is not None:
                intr = self.camera_intrinsics()
                handle.write(
                    f'1 PINHOLE {intr["w"]} {intr["h"]} '
                    f'{intr["fl_x"]:.12f} {intr["fl_y"]:.12f} {intr["cx"]:.12f} {intr["cy"]:.12f}\n'
                )

        with open(self.colmap_images_path, 'w', encoding='utf-8') as handle:
            handle.write('# Image list with two lines of data per image:\n')
            handle.write('# IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n')
            handle.write('# POINTS2D[] as (X, Y, POINT3D_ID)\n')
            for frame in self.frames:
                transform = frame_to_transform(frame)
                (colmap_qw, colmap_qx, colmap_qy, colmap_qz), (tx, ty, tz) = invert_pose_from_transform(transform)
                handle.write(
                    f'{frame["image_id"]} '
                    f'{colmap_qw:.12f} {colmap_qx:.12f} {colmap_qy:.12f} {colmap_qz:.12f} '
                    f'{tx:.12f} {ty:.12f} {tz:.12f} 1 {frame["file_path"]}\n\n'
                )

    def write_desktop_pose_files(self) -> None:
        with open(self.poses_colmap_w2c_path, 'w', encoding='utf-8') as handle:
            handle.write('# name qw qx qy qz tx ty tz\n')
            for frame in self.frames:
                image_name = os.path.basename(frame['file_path'])
                transform = frame_to_transform(frame)
                (qw, qx, qy, qz), (tx, ty, tz) = invert_pose_from_transform(transform)
                handle.write(
                    f'{image_name} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} '
                    f'{tx:.9f} {ty:.9f} {tz:.9f}\n'
                )

        with open(self.camera_centers_world_path, 'w', encoding='utf-8') as handle:
            handle.write('# name Cx Cy Cz, computed as -R^T*t from COLMAP world-to-camera pose\n')
            for frame in self.frames:
                image_name = os.path.basename(frame['file_path'])
                matrix = frame['transform_matrix']
                handle.write(
                    f'{image_name} {matrix[0][3]:.9f} {matrix[1][3]:.9f} {matrix[2][3]:.9f}\n'
                )

        with open(self.image_name_mapping_path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'segment_image',
                'source_colmap_image_id',
                'source_colmap_image',
                'aligned_raw_bag_frame_index',
                'raw_image',
            ])
            for frame in self.frames:
                image_name = os.path.basename(frame['file_path'])
                writer.writerow([
                    image_name,
                    frame['image_id'],
                    image_name,
                    frame['image_id'],
                    image_name,
                ])

    def publish_event(self, event: str) -> None:
        if self.publish_events:
            self.event_pub.publish(String(data=event))

    def announce_done_once(self) -> None:
        if self.done_announced:
            return
        self.done_announced = True
        self.publish_event(f'LIVO_EXPORT_DONE:{self.output_dir}:{self.saved_count}')
        self.get_logger().info(f'Finished export: {self.saved_count} image/pose pairs in {self.output_dir}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPoseExporterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
