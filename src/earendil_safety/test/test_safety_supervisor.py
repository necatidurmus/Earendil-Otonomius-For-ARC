"""
test_safety_supervisor.py

Safety Supervisor birim testleri.
ROS bağımlılıkları mock'lanır — saf Python mantığı test edilir.
"""
import math
from unittest.mock import MagicMock


class _MockLogger:
    def error(self, m): pass

    def warn(self, m): pass

    def info(self, m): pass


class _MockSafetyNode:
    """Safety Supervisor'ın iç mantığını test etmek için minimal mock."""

    def __init__(self):
        self._e_stop_active = False
        self._tilt_fault = False
        self._battery_fault = False
        self._max_tilt_rad = math.radians(30.0)
        self._min_battery = 15.0
        self._watchdog_timeout = 2.0
        self._last_cmd_time = None
        self._cmd_pub = MagicMock()
        self._status_pub = MagicMock()
        self._zero_published = False

    def get_logger(self):
        return _MockLogger()

    def _any_fault(self):
        return self._e_stop_active or self._tilt_fault or self._battery_fault

    def _publish_zero(self):
        self._zero_published = True

    def _estop_cb(self, msg):
        if msg.data and not self._e_stop_active:
            self._e_stop_active = True
            self._publish_zero()
        elif not msg.data and self._e_stop_active:
            self._e_stop_active = False

    def _imu_cb(self, msg):
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = abs(math.atan2(sinr_cosp, cosr_cosp))
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = abs(math.asin(max(-1.0, min(1.0, sinp))))
        tilt = max(roll, pitch)
        if tilt > self._max_tilt_rad:
            self._tilt_fault = True
        else:
            self._tilt_fault = False

    def _battery_cb(self, msg):
        if msg.data < self._min_battery:
            self._battery_fault = True
        else:
            self._battery_fault = False

    def _cmd_cb(self, msg):
        if self._any_fault():
            self._publish_zero()
            return
        self._cmd_pub.publish(msg)


def _make_imu_msg(roll_deg=0.0, pitch_deg=0.0):
    """Test IMU mesajı."""
    msg = MagicMock()
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    msg.orientation.w = math.cos(r / 2) * math.cos(p / 2)
    msg.orientation.x = math.sin(r / 2) * math.cos(p / 2)
    msg.orientation.y = math.cos(r / 2) * math.sin(p / 2)
    msg.orientation.z = -math.sin(r / 2) * math.sin(p / 2)
    return msg


class TestSafetySupervisorLogic:
    """Safety supervisor iç mantığı birim testleri."""

    def setup_method(self):
        self.node = _MockSafetyNode()

    def test_no_fault_initially(self):
        assert self.node._any_fault() is False

    def test_estop_sets_any_fault(self):
        self.node._e_stop_active = True
        assert self.node._any_fault() is True

    def test_tilt_sets_any_fault(self):
        self.node._tilt_fault = True
        assert self.node._any_fault() is True

    def test_battery_sets_any_fault(self):
        self.node._battery_fault = True
        assert self.node._any_fault() is True

    def test_estop_activated_on_true_msg(self):
        msg = MagicMock()
        msg.data = True
        self.node._estop_cb(msg)
        assert self.node._e_stop_active is True

    def test_estop_publishes_zero_on_activate(self):
        msg = MagicMock()
        msg.data = True
        self.node._estop_cb(msg)
        assert self.node._zero_published is True

    def test_estop_cleared_on_false_msg(self):
        self.node._e_stop_active = True
        msg = MagicMock()
        msg.data = False
        self.node._estop_cb(msg)
        assert self.node._e_stop_active is False

    def test_tilt_fault_when_tilt_exceeds_30deg(self):
        imu = _make_imu_msg(roll_deg=45.0)
        self.node._imu_cb(imu)
        assert self.node._tilt_fault is True

    def test_no_tilt_fault_within_limit(self):
        imu = _make_imu_msg(roll_deg=10.0)
        self.node._imu_cb(imu)
        assert self.node._tilt_fault is False

    def test_battery_fault_below_threshold(self):
        msg = MagicMock()
        msg.data = 5.0
        self.node._battery_cb(msg)
        assert self.node._battery_fault is True

    def test_no_battery_fault_above_threshold(self):
        msg = MagicMock()
        msg.data = 80.0
        self.node._battery_cb(msg)
        assert self.node._battery_fault is False

    def test_cmd_blocked_when_estop(self):
        self.node._e_stop_active = True
        cmd = MagicMock()
        self.node._cmd_cb(cmd)
        assert self.node._zero_published is True
        self.node._cmd_pub.publish.assert_not_called()

    def test_cmd_forwarded_when_safe(self):
        cmd = MagicMock()
        self.node._cmd_cb(cmd)
        self.node._cmd_pub.publish.assert_called_once_with(cmd)
