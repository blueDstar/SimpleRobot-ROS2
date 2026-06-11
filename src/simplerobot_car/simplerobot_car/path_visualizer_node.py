#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster


class PathVisualizerNode(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/simplerobot_path')
        self.declare_parameter('heading_marker_topic', '/simplerobot_heading')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_path_length', 3000)
        self.declare_parameter('min_distance_step', 0.02)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.heading_marker_topic = str(self.get_parameter('heading_marker_topic').value)
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.max_path_length = int(self.get_parameter('max_path_length').value)
        self.min_distance_step = float(self.get_parameter('min_distance_step').value)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.fixed_frame

        self.last_x = None
        self.last_y = None

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.heading_marker_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        self.get_logger().info('Path visualizer started')
        self.get_logger().info(f'Subscribe: {self.odom_topic}')
        self.get_logger().info(f'Publish path: {self.path_topic}')
        self.get_logger().info(f'Publish heading marker: {self.heading_marker_topic}')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None and self.last_y is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_distance_step:
                self.publish_tf_and_marker(msg)
                return

        self.last_x = x
        self.last_y = y

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.fixed_frame
        pose.pose = msg.pose.pose

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > self.max_path_length:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_length:]

        self.path_pub.publish(self.path_msg)

        self.publish_tf_and_marker(msg)

    def publish_tf_and_marker(self, odom_msg):
        now = self.get_clock().now().to_msg()

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.fixed_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = odom_msg.pose.pose.position.x
            t.transform.translation.y = odom_msg.pose.pose.position.y
            t.transform.translation.z = odom_msg.pose.pose.position.z

            t.transform.rotation = odom_msg.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)

        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.fixed_frame
        marker.ns = 'simplerobot_heading'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position = odom_msg.pose.pose.position
        marker.pose.orientation = odom_msg.pose.pose.orientation

        marker.scale.x = 0.35
        marker.scale.y = 0.06
        marker.scale.z = 0.06

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
