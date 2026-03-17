"""
mission_manager.py

Ana mission manager node — RSCP komutlarını alır, state machine'i yönetir,
navigasyon ve görev yürütmeyi koordine eder.

Earendil-Otonomius'taki mission_manager.py'den ilham alınarak modüler
ve ARC odaklı yeniden yazılmıştır.

Topics:
  Subscribe: /rscp/command          [earendil_msgs/RscpCommand]
             /localization/mode     [std_msgs/String]
  Publish:   /mission/state         [earendil_msgs/MissionState]
             /mission/task_status   [earendil_msgs/TaskStatus]

Actions (Client):
  NavigateToPose [nav2_msgs]
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from earendil_mission.mission_state_machine import MissionStateMachine, MissionState
from earendil_mission.task_executor import TaskExecutor
from earendil_msgs.msg import MissionState as MissionStateMsg
from earendil_msgs.msg import RscpCommand, TaskStatus


class MissionManager(Node):
    """
    Görev yöneticisi node'u.
    RSCP komutları, lokalizasyon modu ve görev durumunu koordine eder.
    """

    def __init__(self):
        super().__init__('mission_manager')

        # Parametreler
        self.declare_parameter('state_publish_rate_hz', 2.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('max_mission_time_secs', 1200.0)

        self._state_machine = MissionStateMachine(
            on_state_change=self._on_state_change)
        self._task_executor = TaskExecutor()
        self._mission_start_time: float = 0.0

        # Publishers
        self._state_pub = self.create_publisher(
            MissionStateMsg, '/mission/state', 10)
        self._task_status_pub = self.create_publisher(
            TaskStatus, '/mission/task_status', 10)

        # Subscribers
        self.create_subscription(
            RscpCommand, '/rscp/command', self._rscp_cb, 10)
        self.create_subscription(
            String, '/localization/mode', self._loc_mode_cb, 10)

        # State yayın timer'ı
        rate = self.get_parameter('state_publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_state)

        # Auto-start
        if self.get_parameter('auto_start').value:
            self.get_logger().info('Auto-start aktif — görev başlatılıyor.')
            self._state_machine.handle_event('START')
            self._mission_start_time = time.monotonic()

        self.get_logger().info('MissionManager başlatıldı. RSCP START komutunu bekliyor...')

    def _rscp_cb(self, msg: RscpCommand) -> None:
        """RSCP komutu geldiğinde işler."""
        cmd_map = {
            RscpCommand.START:    'START',
            RscpCommand.STOP:     'STOP',
            RscpCommand.PAUSE:    'PAUSE',
            RscpCommand.RESUME:   'RESUME',
            RscpCommand.E_STOP:   'E_STOP',
            RscpCommand.GO_HOME:  'GO_HOME',
        }
        event = cmd_map.get(msg.command_type)
        if event:
            self.get_logger().info(
                f'RSCP komutu: {msg.command_name} (seq={msg.sequence_id})')
            if event == 'START' and self._state_machine.current_state == MissionState.IDLE:
                self._mission_start_time = time.monotonic()
            self._state_machine.handle_event(event)
        else:
            self.get_logger().warn(
                f'Bilinmeyen RSCP komut tipi: {msg.command_type}')

    def _loc_mode_cb(self, msg: String) -> None:
        """Lokalizasyon modu güncellemesini işler."""
        mode = msg.data.lower()
        current = self._state_machine.current_state

        if current in (MissionState.GPS_NAV, MissionState.SLAM_NAV,
                       MissionState.LOCALIZING):
            if mode == 'gps':
                self._state_machine.handle_event('GPS_OK')
            elif mode == 'slam':
                self._state_machine.handle_event('NO_GPS')

    def _on_state_change(
            self,
            old_state: MissionState,
            new_state: MissionState,
            event: str,
    ) -> None:
        """Durum değişiminde tetiklenir."""
        old_name = self._state_machine.STATE_NAMES.get(old_state, '?')
        new_name = self._state_machine.STATE_NAMES.get(new_state, '?')
        self.get_logger().info(
            f'Mission: {old_name} → {new_name} [event={event}]')

    def _publish_state(self) -> None:
        """Görev durumunu yayınlar."""
        sm = self._state_machine
        msg = MissionStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = int(sm.current_state)
        msg.state_name = sm.current_state_name
        msg.active_task = sm.active_task or ''
        msg.mission_elapsed_secs = float(
            time.monotonic() - self._mission_start_time
            if self._mission_start_time > 0 else 0.0
        )
        msg.e_stop_active = (sm.current_state == MissionState.EMERGENCY)
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
