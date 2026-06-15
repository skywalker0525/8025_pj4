from datetime import datetime
import json
import os
from typing import Optional

from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from my_robot_mission.utils import expand_path


class ImageVideoRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('image_video_recorder_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('record_states', ['NAVIGATING'])
        self.declare_parameter('finish_states', ['DONE', 'STOPPED'])
        self.declare_parameter('output_dir', '~/livo_image_pose_exports')
        self.declare_parameter('run_name', '')
        self.declare_parameter('filename', 'camera.mp4')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('max_frames', 0)
        self.declare_parameter('publish_events', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.record_states = [
            str(state) for state in self.get_parameter('record_states').value if str(state)
        ]
        self.finish_states = [
            str(state) for state in self.get_parameter('finish_states').value if str(state)
        ]
        self.filename = self.get_parameter('filename').value
        self.fps = float(self.get_parameter('fps').value)
        self.max_frames = int(self.get_parameter('max_frames').value)
        self.publish_events = bool(self.get_parameter('publish_events').value)

        run_name = self.get_parameter('run_name').value
        if not run_name:
            run_name = datetime.now().strftime('capture_%Y%m%d_%H%M%S')
        base_output_dir = expand_path(self.get_parameter('output_dir').value)
        self.output_dir = os.path.join(base_output_dir, run_name)
        os.makedirs(self.output_dir, exist_ok=True)
        self.video_path = os.path.join(self.output_dir, self.filename)
        self.metadata_path = os.path.join(
            self.output_dir,
            f'{os.path.splitext(self.filename)[0]}_metadata.json',
        )

        self.bridge = CvBridge()
        self.current_mission_state = ''
        self.writer: Optional[cv2.VideoWriter] = None
        self.frame_count = 0
        self.done_announced = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_sub = self.create_subscription(
            String,
            self.mission_state_topic,
            self.on_mission_state,
            qos,
        )
        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.event_pub = self.create_publisher(String, '/mission/events', 10)

        self.get_logger().info(
            f'Recording {self.image_topic} to {self.video_path} '
            f'during states {self.record_states}.'
        )

    def on_mission_state(self, msg: String) -> None:
        self.current_mission_state = msg.data
        if self.current_mission_state in self.finish_states and self.frame_count > 0:
            self.close_writer()
            self.announce_done_once()

    def on_image(self, msg: Image) -> None:
        if self.done_announced:
            return
        if self.max_frames > 0 and self.frame_count >= self.max_frames:
            self.close_writer()
            self.announce_done_once()
            return
        if self.record_states and self.current_mission_state not in self.record_states:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.video_path, fourcc, self.fps, (width, height))
            if not self.writer.isOpened():
                self.get_logger().error(f'Failed to open video writer for {self.video_path}')
                self.writer = None
                self.announce_done_once()
                return
            self.publish_event(f'VIDEO_RECORDING_STARTED:{self.video_path}')

        self.writer.write(frame)
        self.frame_count += 1

    def close_writer(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        self.write_metadata()

    def write_metadata(self) -> None:
        with open(self.metadata_path, 'w', encoding='utf-8') as handle:
            json.dump(
                {
                    'image_topic': self.image_topic,
                    'mission_state_topic': self.mission_state_topic,
                    'record_states': self.record_states,
                    'finish_states': self.finish_states,
                    'video_path': self.video_path,
                    'fps': self.fps,
                    'frame_count': self.frame_count,
                },
                handle,
                indent=2,
            )

    def publish_event(self, event: str) -> None:
        if self.publish_events:
            self.event_pub.publish(String(data=event))

    def announce_done_once(self) -> None:
        if self.done_announced:
            return
        self.done_announced = True
        self.publish_event(f'VIDEO_RECORDING_DONE:{self.video_path}:{self.frame_count}')
        self.get_logger().info(
            f'Finished video recording: {self.frame_count} frames in {self.video_path}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageVideoRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.close_writer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
