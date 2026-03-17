#!/usr/bin/env python3
"""waypoint_manager_node.py — Waypoint Manager ROS 2 node (stub)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WaypointManagerNode(Node):
    """
    Waypoint Manager ROS 2 node'u.
    TODO (Faz 1): Nav2 NavigateToPose action client entegrasyonu
    """

    def __init__(self):
        super().__init__('waypoint_manager')
        self.get_logger().info('WaypointManagerNode başlatıldı.')

    # TODO (Faz 1): Nav2 action client, waypoint YAML yükleme, mission topic bağlantısı


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
