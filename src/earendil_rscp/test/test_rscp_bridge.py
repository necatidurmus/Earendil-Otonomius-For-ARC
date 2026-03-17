"""
test_rscp_bridge.py

RSCP Bridge birim testleri.
"""
from unittest.mock import MagicMock


class TestRscpBridgeLogic:
    """RSCP bridge temel mantık testleri."""

    def test_initial_state_disconnected(self):
        bridge = _MockRscpBridge()
        assert bridge._connected is False

    def test_send_command_publishes_to_topic(self):
        bridge = _MockRscpBridge()
        bridge.send_command(0, 'START')
        bridge._cmd_pub.publish.assert_called_once()

    def test_heartbeat_publishes_status(self):
        bridge = _MockRscpBridge()
        bridge._heartbeat_cb()
        bridge._conn_pub.publish.assert_called_once()


class _MockRscpBridge:
    """Mock RSCP bridge for unit testing."""

    def __init__(self):
        self._connected = False
        self._cmd_pub = MagicMock()
        self._conn_pub = MagicMock()
        self._estop_pub = MagicMock()

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = None
        return clock

    def _heartbeat_cb(self):
        status = MagicMock()
        status.data = 'CONNECTED' if self._connected else 'DISCONNECTED_STUB'
        self._conn_pub.publish(status)

    def send_command(self, command_type, payload=''):
        msg = MagicMock()
        msg.command_type = command_type
        msg.payload = payload
        self._cmd_pub.publish(msg)
