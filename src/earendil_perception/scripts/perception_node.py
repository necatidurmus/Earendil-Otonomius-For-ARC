#!/usr/bin/env python3
"""perception_node.py — Perception node (ArUco + Rock + Peak), ROS 2 stub."""
import rclpy
from rclpy.node import Node


class PerceptionNode(Node):
    """
    Tüm algılama task'larını koordine eden node.
    Mission manager'dan gelen task_request'e göre aktif modülü seçer.
    TODO (Faz 2): Task-based algılama modülü aktivasyonu
    """

    def __init__(self):
        super().__init__('perception')
        self.get_logger().info('PerceptionNode başlatıldı. [STUB]')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
