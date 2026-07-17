#!/usr/bin/env python3
# TF Mode Relay — GPS/SLAM frame selector
# Maintains single TF tree: map → map_slam → odom → base_footprint
# Adapted from existing leo_gz_bringup/tf_mode_relay.py

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import (
    TransformStamped, Transform, Vector3, Quaternion
)
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf2_ros


def _identity_transform() -> Transform:
    t = Transform()
    t.translation = Vector3(x=0.0, y=0.0, z=0.0)
    t.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return t


def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    )


def _quat_conjugate(q: Quaternion) -> Quaternion:
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def _rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    vq = Quaternion(x=v.x, y=v.y, z=v.z, w=0.0)
    result = _quat_multiply(_quat_multiply(q, vq), _quat_conjugate(q))
    return Vector3(x=result.x, y=result.y, z=result.z)


def _invert_transform(t: Transform) -> Transform:
    q_inv = _quat_conjugate(t.rotation)
    neg_t = Vector3(
        x=-t.translation.x, y=-t.translation.y, z=-t.translation.z)
    t_inv = _rotate_vector(q_inv, neg_t)
    return Transform(translation=t_inv, rotation=q_inv)


def _compose_transforms(t_ab: Transform, t_bc: Transform) -> Transform:
    rotated = _rotate_vector(t_ab.rotation, t_bc.translation)
    return Transform(
        translation=Vector3(
            x=t_ab.translation.x + rotated.x,
            y=t_ab.translation.y + rotated.y,
            z=t_ab.translation.z + rotated.z),
        rotation=_quat_multiply(t_ab.rotation, t_bc.rotation),
    )


class TFModeRelay(Node):
    """Relays map→odom TF based on GPS/SLAM mode."""

    def __init__(self):
        super().__init__('tf_mode_relay')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_slam_frame', 'map_slam')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('transition_duration', 1.0)
        self.declare_parameter('publish_rate', 50.0)

        self._map_frame = self.get_parameter('map_frame').value
        self._map_slam_frame = self.get_parameter('map_slam_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._transition_dur = self.get_parameter('transition_duration').value
        rate = self.get_parameter('publish_rate').value

        self._current_mode = 'GPS'
        self._last_published = _identity_transform()

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(String, '/nav_mode', self._on_mode, 10)
        self.create_timer(1.0 / rate, self._publish_tf)

        self.get_logger().info('TF Mode Relay started')

    def _on_mode(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode != self._current_mode:
            self.get_logger().info(f'TF mode: {self._current_mode} → {new_mode}')
            self._current_mode = new_mode

    def _publish_tf(self):
        stamp = self.get_clock().now().to_msg()

        if self._current_mode == 'SLAM':
            # Identity: map = map_slam (SLAM is truth)
            transform = _identity_transform()
        else:
            # GPS mode: try to get map_gps→base_footprint from UKF
            try:
                t_map_base = self._tf_buffer.lookup_transform(
                    'map_gps', self._base_frame, rclpy.time.Time())
                t_slam_base = self._tf_buffer.lookup_transform(
                    self._map_slam_frame, self._base_frame, rclpy.time.Time())

                # map→map_slam = map_gps→base * (map_slam→base)^-1
                t_mb = t_map_base.transform
                t_sb_inv = _invert_transform(t_slam_base.transform)
                transform = _compose_transforms(t_mb, t_sb_inv)
            except Exception:
                transform = self._last_published

        # Publish map → map_slam
        t_msg = TransformStamped()
        t_msg.header.stamp = stamp
        t_msg.header.frame_id = self._map_frame
        t_msg.child_frame_id = self._map_slam_frame
        t_msg.transform = transform
        self._tf_broadcaster.sendTransform(t_msg)
        self._last_published = transform


def main(args=None):
    rclpy.init(args=args)
    node = TFModeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
