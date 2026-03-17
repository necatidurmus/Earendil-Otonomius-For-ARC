"""
session_logger.py

Rosbag session yöneticisi ve event logger.
Görev başında/sonunda rosbag kaydını otomatik başlatır/durdurur.

TODO (Faz 3): subprocess ile rosbag2 kaydını yönet
"""
import datetime
import os
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from earendil_msgs.msg import MissionState


class SessionLogger(Node):
    """Görev session'larını loglar ve rosbag kaydını yönetir."""

    def __init__(self):
        super().__init__('session_logger')

        self.declare_parameter('auto_record', True)
        self.declare_parameter('output_dir', '~/rosbags')

        self._auto_record = self.get_parameter('auto_record').value
        self._output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value)
        self._bag_process: Optional[subprocess.Popen] = None
        self._session_id: Optional[str] = None

        # Mission state izle
        self.create_subscription(
            MissionState, '/mission/state', self._mission_state_cb, 10)

        self.get_logger().info(
            f'SessionLogger başlatıldı. '
            f'Çıktı: {self._output_dir}, Auto-record: {self._auto_record}')

    def _mission_state_cb(self, msg: MissionState) -> None:
        """Görev başlayınca kaydı başlat, bitince durdur."""
        if not self._auto_record:
            return

        if msg.state == MissionState.LOCALIZING and self._bag_process is None:
            self._start_recording()
        elif msg.state in (MissionState.COMPLETE, MissionState.EMERGENCY):
            self._stop_recording()

    def _start_recording(self) -> None:
        """Rosbag kaydını başlatır."""
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._session_id = f'arc_mission_{timestamp}'
        bag_path = os.path.join(self._output_dir, self._session_id)
        os.makedirs(self._output_dir, exist_ok=True)

        # TODO (Faz 3): Kaydedilecek topic'ler sim_config.yaml'dan okunacak
        cmd = [
            'ros2', 'bag', 'record', '-o', bag_path,
            '/cmd_vel', '/odom', '/imu/data', '/fix',
            '/mission/state', '/rscp/command', '/safety/status',
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
        ]
        self.get_logger().info(f'Rosbag kaydı başladı: {bag_path}')
        # TODO (Faz 3): subprocess.Popen ile gerçek kayıt başlat
        # self._bag_process = subprocess.Popen(cmd)

    def _stop_recording(self) -> None:
        """Rosbag kaydını durdurur."""
        if self._bag_process is not None:
            self._bag_process.terminate()
            self._bag_process = None
            self.get_logger().info(
                f'Rosbag kaydı tamamlandı: {self._session_id}')
        self._session_id = None


def main(args=None):
    rclpy.init(args=args)
    node = SessionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
