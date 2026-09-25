#!/usr/bin/env python3
# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Publish moving odometry TF for the supplied Gazebo P3D pose stream.

Simulation assumption: the odom coordinate system is chosen coincident with
Gazebo world. No initial-pose subtraction is performed. /odom remains unchanged.
This is a TF adapter, not wheel odometry or a localisation estimator.
Developed with AI coding assistance.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('source_frame', 'world')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.source = self.get_parameter('source_frame').value
        self.parent = self.get_parameter('odom_frame').value
        self.child = self.get_parameter('base_frame').value
        self.broadcaster = TransformBroadcaster(self)
        self.warned = set()
        self.received = False
        self.subscription = self.create_subscription(
            Odometry, self.get_parameter('input_topic').value,
            self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(
            f'Using Gazebo {self.source} coordinates as {self.parent}; '
            f'publishing {self.parent} -> {self.child}. Input is ground truth.')

    def reject(self, reason):
        if reason not in self.warned:
            self.get_logger().error(reason)
            self.warned.add(reason)

    def on_odom(self, msg):
        # Do not silently interpret an unrelated coordinate frame as odometry.
        if msg.header.frame_id.lstrip('/') != self.source:
            self.reject(f'Expected input frame {self.source!r}; got {msg.header.frame_id!r}.')
            return
        # Gazebo may include a model scope in the child link name.
        child = msg.child_frame_id.lstrip('/').split('::')[-1]
        if child != self.child:
            self.reject(f'Expected input child {self.child!r}; got {msg.child_frame_id!r}.')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not all(math.isfinite(v) for v in values) or norm < 1e-9:
            self.reject('Skipping invalid odometry pose.')
            return
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.parent
        tf.child_frame_id = self.child
        tf.transform.translation.x = p.x
        tf.transform.translation.y = p.y
        tf.transform.translation.z = p.z
        tf.transform.rotation.x = q.x / norm
        tf.transform.rotation.y = q.y / norm
        tf.transform.rotation.z = q.z / norm
        tf.transform.rotation.w = q.w / norm
        self.broadcaster.sendTransform(tf)
        if not self.received:
            self.get_logger().info('First odometry received; moving TF is publishing.')
            self.received = True


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
