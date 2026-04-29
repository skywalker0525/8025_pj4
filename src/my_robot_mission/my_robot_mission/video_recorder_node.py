from datetime import datetime
import json
import os

from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from my_robot_mission.utils import expand_path, load_yaml_file


class VideoRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('video_recorder_node')
        self.declare_parameter('waypoints_file', '')

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            raise RuntimeError('waypoints_file parameter is required.')

        cfg = load_yaml_file(waypoints_file)
        self.output_dir = expand_path(cfg.get('video', {}).get('output_dir', '~/ros_videos'))
        self.fps = float(cfg.get('video', {}).get('fps', 20))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.bridge = CvBridge()
        self.current_state = 'WAIT_FOR_GOAL'
        self.writer = None
        self.writer_path = None

        self.state_sub = self.create_subscription(String, '/mission/state', self.on_state, qos)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.status_pub = self.create_publisher(String, '/mission/video_status', qos)
        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.publish_status(False)

    def publish_status(self, recording: bool) -> None:
        self.status_pub.publish(String(data=json.dumps({
            'recording': recording,
            'path': self.writer_path or '',
        })))

    def on_state(self, msg: String) -> None:
        previous_state = self.current_state
        self.current_state = msg.data

        if previous_state == 'ORBITING' and self.current_state in ('SAVING', 'DONE', 'WAIT_FOR_GOAL', 'STOPPED'):
            self.close_writer()

    def on_image(self, msg: Image) -> None:
        if self.current_state != 'ORBITING':
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.writer is None:
            os.makedirs(self.output_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.writer_path = os.path.join(self.output_dir, f'orbit_capture_{timestamp}.mp4')
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.writer_path, fourcc, self.fps, (width, height))
            if not self.writer.isOpened():
                self.get_logger().error(f'Failed to open video writer for {self.writer_path}')
                self.writer = None
                self.writer_path = None
                self.publish_status(False)
                return
            self.get_logger().info(f'Recording orbit video to {self.writer_path}')
            self.event_pub.publish(String(data='VIDEO_RECORDING_STARTED'))
            self.publish_status(True)

        self.writer.write(frame)

    def close_writer(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f'Saved video to {self.writer_path}')
            self.event_pub.publish(String(data=f'VIDEO_SAVED:{self.writer_path}'))
            self.publish_status(False)
            self.writer = None
            self.writer_path = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.close_writer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
