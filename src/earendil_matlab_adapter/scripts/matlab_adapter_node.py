#!/usr/bin/env python3
"""matlab_adapter_node.py — MATLAB adapter stub ROS 2 node."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from earendil_matlab_adapter.matlab_bridge_stub import MatlabBridgeStub


class MatlabAdapterNode(Node):
    """
    MATLAB northbound adapter node — STUB.
    TODO (Faz 4): rosbridge_server veya native WebSocket ile MATLAB bağlantısı
    """

    def __init__(self):
        super().__init__('matlab_adapter')
        self._bridge = MatlabBridgeStub()
        self.get_logger().info('MatlabAdapterNode başlatıldı. [NORTHBOUND STUB]')


def main(args=None):
    rclpy.init(args=args)
    node = MatlabAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
