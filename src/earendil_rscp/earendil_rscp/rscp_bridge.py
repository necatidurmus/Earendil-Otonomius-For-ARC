"""
rscp_bridge.py

RSCP (Remote Station Communication Protocol) köprüsü.
Uzak istasyon ↔ Rover haberleşmesini yönetir.

Protokol desteği (şimdilik stub):
  - WebSocket (simülasyon ve LAN)
  - Serial (gerçek rover)
  - UDP (düşük bant genişliği, ileride)

Davranış:
  - Bağlantı kesilirse güvenli mod tetiklenir (e_stop yayınlanır)
  - Telemetri düzenli gönderilir
  - Komutlar /rscp/command topic'ine iletilir
"""
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from earendil_msgs.msg import RscpCommand, MissionState


class RscpBridge(Node):
    """
    RSCP haberleşme köprüsü node'u.
    TODO (Faz 2): WebSocket implementasyonu
    """

    def __init__(self):
        super().__init__('rscp_bridge')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 9090)
        self.declare_parameter('protocol', 'websocket')
        self.declare_parameter('heartbeat_hz', 1.0)
        self.declare_parameter('command_timeout_secs', 5.0)

        self._connected = False
        self._last_heartbeat_time: Optional[float] = None

        # Publishers
        self._cmd_pub = self.create_publisher(
            RscpCommand, '/rscp/command', 10)
        self._conn_pub = self.create_publisher(
            String, '/rscp/connection_status', 10)
        self._estop_pub = self.create_publisher(
            Bool, '/e_stop', 10)

        # Subscriber (mission state için telemetri)
        self.create_subscription(
            MissionState, '/mission/state', self._mission_state_cb, 10)

        rate = self.get_parameter('heartbeat_hz').value
        self.create_timer(1.0 / rate, self._heartbeat_cb)

        self.get_logger().info(
            f'RscpBridge başlatıldı. '
            f'Host: {self.get_parameter("host").value}:'
            f'{self.get_parameter("port").value} '
            f'[{self.get_parameter("protocol").value}] — STUB')

    def _heartbeat_cb(self) -> None:
        """Bağlantı durumunu yayınlar."""
        # TODO (Faz 2): Gerçek WebSocket bağlantı kontrolü
        status = String()
        status.data = 'CONNECTED' if self._connected else 'DISCONNECTED_STUB'
        self._conn_pub.publish(status)

    def _mission_state_cb(self, msg: MissionState) -> None:
        """Görev durumunu uzak istasyona iletir (telemetri)."""
        # TODO (Faz 2): WebSocket üzerinden JSON telemetri gönder
        pass

    def send_command(self, command_type: int, payload: str = '') -> None:
        """
        ROS üzerinden RSCP komutu yayınlar.
        Test ve debug için kullanılabilir.
        """
        msg = RscpCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = command_type
        msg.payload = payload
        msg.source = 'rscp_bridge'
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RscpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
