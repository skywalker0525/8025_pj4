import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_mission.utils import load_yaml_file, yaw_to_quaternion


class WaypointMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_marker_node')
        self.declare_parameter('waypoints_file', '')

        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            raise RuntimeError('waypoints_file parameter is required.')

        self.cfg = load_yaml_file(waypoints_file)
        self.frame_id = self.cfg.get('frames', {}).get('target_frame', 'map')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/markers', qos)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self) -> None:
        marker_array = MarkerArray()
        marker_id = 0

        for name, point in self.cfg['points'].items():
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = float(point['x'])
            marker.pose.position.y = float(point['y'])
            marker.pose.position.z = 0.05
            quat = yaw_to_quaternion(float(point['yaw']))
            marker.pose.orientation.x = quat['x']
            marker.pose.orientation.y = quat['y']
            marker.pose.orientation.z = quat['z']
            marker.pose.orientation.w = quat['w']
            marker.scale.x = 0.45
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 0.1
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.95
            marker_array.markers.append(marker)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = marker.header.stamp
            label.ns = 'waypoint_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(point['x'])
            label.pose.position.y = float(point['y'])
            label.pose.position.z = 0.45
            label.scale.z = 0.3
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = name
            marker_array.markers.append(label)

        cylinder = Marker()
        cylinder.header.frame_id = self.frame_id
        cylinder.header.stamp = self.get_clock().now().to_msg()
        cylinder.ns = 'target_object'
        cylinder.id = marker_id
        marker_id += 1
        cylinder.type = Marker.CYLINDER
        cylinder.action = Marker.ADD
        cylinder.pose.position.x = float(self.cfg['target_object']['x'])
        cylinder.pose.position.y = float(self.cfg['target_object']['y'])
        cylinder.pose.position.z = 0.6
        cylinder.scale.x = float(self.cfg['target_object']['radius']) * 2.0
        cylinder.scale.y = float(self.cfg['target_object']['radius']) * 2.0
        cylinder.scale.z = 1.2
        cylinder.color.r = 0.95
        cylinder.color.g = 0.95
        cylinder.color.b = 0.95
        cylinder.color.a = 0.9
        marker_array.markers.append(cylinder)

        orbit_ring = Marker()
        orbit_ring.header.frame_id = self.frame_id
        orbit_ring.header.stamp = cylinder.header.stamp
        orbit_ring.ns = 'orbit_ring'
        orbit_ring.id = marker_id
        orbit_ring.type = Marker.LINE_STRIP
        orbit_ring.action = Marker.ADD
        orbit_ring.scale.x = 0.04
        orbit_ring.color.r = 0.9
        orbit_ring.color.g = 0.2
        orbit_ring.color.b = 0.2
        orbit_ring.color.a = 0.95

        cx = float(self.cfg['target_object']['x'])
        cy = float(self.cfg['target_object']['y'])
        radius = float(self.cfg['orbit']['radius'])
        for idx in range(33):
            angle = (2.0 * math.pi * idx) / 32.0
            orbit_ring.points.append(
                Point(
                    x=cx + radius * math.cos(angle),
                    y=cy + radius * math.sin(angle),
                    z=0.05,
                )
            )
        marker_array.markers.append(orbit_ring)

        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
