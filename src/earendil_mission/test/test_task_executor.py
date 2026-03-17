"""
test_task_executor.py

TaskExecutor birim testleri.
"""
import pytest

from earendil_mission.task_executor import TaskExecutor, TaskResult


class TestTaskExecutor:
    """TaskExecutor birim testleri."""

    def setup_method(self):
        self.executor = TaskExecutor()

    def test_available_tasks_contains_arc_tasks(self):
        tasks = self.executor.available_tasks
        assert 'aruco' in tasks
        assert 'rock' in tasks
        assert 'peak' in tasks
        assert 'tunnel' in tasks

    def test_execute_aruco_returns_success(self):
        result = self.executor.execute('aruco', {'marker_ids': [0, 1]})
        assert result.success

    def test_execute_rock_returns_success(self):
        result = self.executor.execute('rock', {})
        assert result.success

    def test_execute_peak_returns_success(self):
        result = self.executor.execute('peak', {'search_radius_m': 10.0})
        assert result.success

    def test_execute_tunnel_returns_success(self):
        result = self.executor.execute('tunnel', {})
        assert result.success

    def test_unknown_task_raises_value_error(self):
        with pytest.raises(ValueError, match='Bilinmeyen görev'):
            self.executor.execute('nonexistent_task', {})

    def test_task_result_success_property(self):
        r = TaskResult(TaskResult.SUCCESS)
        assert r.success is True

    def test_task_result_failed_property(self):
        r = TaskResult(TaskResult.FAILED, error='test error')
        assert r.success is False
        assert r.error == 'test error'

    def test_execute_with_empty_params(self):
        result = self.executor.execute('aruco')
        assert result is not None
        assert result.status in (TaskResult.SUCCESS, TaskResult.FAILED)
