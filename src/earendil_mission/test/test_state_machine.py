"""
test_state_machine.py

Mission State Machine birim testleri.
"""
import pytest

from earendil_mission.mission_state_machine import MissionStateMachine, MissionState


class TestMissionStateMachine:
    """MissionStateMachine birim testleri."""

    def setup_method(self):
        self.sm = MissionStateMachine()

    def test_initial_state_is_idle(self):
        assert self.sm.current_state == MissionState.IDLE

    def test_start_transitions_to_localizing(self):
        self.sm.handle_event('START')
        assert self.sm.current_state == MissionState.LOCALIZING

    def test_gps_ok_transitions_from_localizing_to_gps_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        assert self.sm.current_state == MissionState.GPS_NAV

    def test_no_gps_transitions_from_localizing_to_slam_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('NO_GPS')
        assert self.sm.current_state == MissionState.SLAM_NAV

    def test_no_gps_transitions_gps_nav_to_slam_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('NO_GPS')
        assert self.sm.current_state == MissionState.SLAM_NAV

    def test_gps_ok_transitions_slam_nav_to_gps_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('NO_GPS')
        self.sm.handle_event('GPS_OK')
        assert self.sm.current_state == MissionState.GPS_NAV

    def test_task_start_from_gps_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('TASK_START', task_name='aruco')
        assert self.sm.current_state == MissionState.TASK_EXEC
        assert self.sm.active_task == 'aruco'

    def test_task_done_returns_to_gps_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('TASK_START', task_name='aruco')
        self.sm.handle_event('TASK_DONE')
        assert self.sm.current_state == MissionState.GPS_NAV
        assert self.sm.active_task is None

    def test_task_fail_returns_to_gps_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('TASK_START', task_name='rock')
        self.sm.handle_event('TASK_FAIL')
        assert self.sm.current_state == MissionState.GPS_NAV

    def test_waypoints_done_from_gps_nav_to_returning(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('WAYPOINTS_DONE')
        assert self.sm.current_state == MissionState.RETURNING

    def test_returning_waypoints_done_to_complete(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('WAYPOINTS_DONE')
        self.sm.handle_event('WAYPOINTS_DONE')
        assert self.sm.current_state == MissionState.COMPLETE

    def test_e_stop_from_gps_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('E_STOP')
        assert self.sm.current_state == MissionState.EMERGENCY

    def test_e_stop_from_task_exec(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('TASK_START', task_name='aruco')
        self.sm.handle_event('E_STOP')
        assert self.sm.current_state == MissionState.EMERGENCY

    def test_e_stop_from_every_state(self):
        """E-Stop her aktif durumdan EMERGENCY'e geçmeli."""
        non_idle_states = [
            MissionState.LOCALIZING,
            MissionState.GPS_NAV,
            MissionState.SLAM_NAV,
            MissionState.TASK_EXEC,
            MissionState.RETURNING,
        ]
        for state in non_idle_states:
            sm = MissionStateMachine()
            sm._state = state
            sm.handle_event('E_STOP')
            assert sm.current_state == MissionState.EMERGENCY, \
                f'{state} durumundan E_STOP → EMERGENCY beklendi'

    def test_stop_returns_to_idle(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('STOP')
        assert self.sm.current_state == MissionState.IDLE

    def test_pause_from_gps_nav(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('PAUSE')
        assert self.sm.current_state == MissionState.PAUSED

    def test_resume_from_paused(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('PAUSE')
        assert self.sm.current_state == MissionState.PAUSED
        self.sm.handle_event('RESUME')
        assert self.sm.current_state == MissionState.GPS_NAV

    def test_on_state_change_callback(self):
        changes = []

        def cb(old, new, event):
            changes.append((old, new, event))

        sm = MissionStateMachine(on_state_change=cb)
        sm.handle_event('START')
        sm.handle_event('GPS_OK')

        assert len(changes) == 2
        assert changes[0] == (MissionState.IDLE, MissionState.LOCALIZING, 'START')
        assert changes[1] == (MissionState.LOCALIZING, MissionState.GPS_NAV, 'GPS_OK')

    def test_state_name_property(self):
        assert self.sm.current_state_name == 'IDLE'
        self.sm.handle_event('START')
        assert self.sm.current_state_name == 'LOCALIZING'

    def test_emergency_resume_resets_to_idle(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        self.sm.handle_event('E_STOP')
        assert self.sm.current_state == MissionState.EMERGENCY
        self.sm.handle_event('RESUME')
        assert self.sm.current_state == MissionState.IDLE

    def test_unknown_event_keeps_current_state(self):
        self.sm.handle_event('START')
        self.sm.handle_event('GPS_OK')
        current = self.sm.current_state
        self.sm.handle_event('COMPLETELY_INVALID_EVENT')
        assert self.sm.current_state == current
