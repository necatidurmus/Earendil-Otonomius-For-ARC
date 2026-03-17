#!/usr/bin/env python3
"""teleop_node.py — Teleop node stub."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TeleopNode(Node):
    """
    Teleop ve manual override yöneticisi.
    Joystick / klavye girişini alır ve override switch yönetir.
    TODO (Faz 2): Joystick, web teleop ve override switch implementasyonu
    """

    def __init__(self):
        super().__init__('teleop_manager')

        self.declare_parameter('mode', 'auto')

        self._mode = self.get_parameter('mode').value

        self._cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_teleop', 10)
        self._mode_pub = self.create_publisher(
            String, '/teleop/mode', 10)

        # Timer: mod durumunu yayınla
        self.create_timer(1.0, self._publish_mode)

        self.get_logger().info(f'TeleopNode başlatıldı. Mod: {self._mode} [STUB]')

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
