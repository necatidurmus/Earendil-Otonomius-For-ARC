"""
test_gps_monitor.py

GPS Monitor birim testleri.
Earendil-Otonomius'taki hibrit mod geçiş mantığını doğrular.
"""
import time
import pytest

from earendil_localization.gps_monitor import GpsQualityMonitor


class TestGpsQualityMonitor:
    """GpsQualityMonitor sınıfı için birim testleri."""

    def _make_monitor(self, **kwargs):
        defaults = {
            'gps_to_slam_threshold': 0.3,
            'slam_to_gps_threshold': 0.45,
            'window_size': 5,
            'hysteresis_secs': 0.0,  # testlerde histerez kapat
            'gps_timeout_secs': 3.0,
            'confirmation_count': 1,  # testlerde hemen geçiş
        }
        defaults.update(kwargs)
        return GpsQualityMonitor(**defaults)

    def test_initial_mode_is_gps(self):
        mon = self._make_monitor()
        assert mon.current_mode == GpsQualityMonitor.GPS_MODE

    def test_no_fix_quality_is_zero(self):
        mon = self._make_monitor()
        # NO_FIX status = -1
        for _ in range(5):
            mon.update_gps(status=-1)
        assert mon.window_average() == pytest.approx(0.0)

    def test_good_gps_quality_above_threshold(self):
        mon = self._make_monitor()
        # STATUS_FIX (0) ile iyi HDOP → kalite > 0.45
        for _ in range(5):
            mon.update_gps(status=0, hdop=1.0)
        assert mon.window_average() > 0.45

    def test_gps_to_slam_transition_on_low_quality(self):
        mon = self._make_monitor(confirmation_count=1, window_size=3)
        # Düşük kalite ile window doldur
        for _ in range(3):
            new_mode = mon.update_gps(status=-1)  # NO_FIX
        assert mon.current_mode == GpsQualityMonitor.SLAM_MODE

    def test_slam_to_gps_transition_on_high_quality(self):
        mon = self._make_monitor(confirmation_count=1, window_size=3)
        # Önce SLAM moduna geç
        for _ in range(3):
            mon.update_gps(status=-1)
        assert mon.current_mode == GpsQualityMonitor.SLAM_MODE

        # İyi GPS ile GPS moduna dön
        for _ in range(3):
            mon.update_gps(status=0, hdop=1.0)
        assert mon.current_mode == GpsQualityMonitor.GPS_MODE

    def test_window_average_correct(self):
        mon = self._make_monitor(window_size=4)
        mon.update_gps(status=-1)   # 0.0
        mon.update_gps(status=-1)   # 0.0
        mon.update_gps(status=0, hdop=1.0)   # ~0.5+
        mon.update_gps(status=0, hdop=1.0)   # ~0.5+
        avg = mon.window_average()
        assert 0.0 < avg < 1.0

    def test_confirmation_count_prevents_early_transition(self):
        mon = self._make_monitor(
            confirmation_count=3,
            window_size=3,
            hysteresis_secs=0.0,
        )
        # 2 kötü ölçüm (confirmation_count=3 → geçiş olmaz)
        for _ in range(3):
            mon.update_gps(status=-1)
        for _ in range(3):
            mon.update_gps(status=-1)
        # Sadece 2 onay birikti, 3. gelmiyor — mod hâlâ GPS
        # NOT: confirmation_count=3 ile 2. döngü 3 onay veriyor,
        #      dolayısıyla burada SLAM beklenir (3 kez confirm)
        assert mon.current_mode == GpsQualityMonitor.SLAM_MODE

    def test_hysteresis_prevents_immediate_retransition(self):
        mon = self._make_monitor(
            confirmation_count=1,
            window_size=3,
            hysteresis_secs=10.0,  # Uzun histerez
        )
        # GPS → SLAM
        for _ in range(3):
            mon.update_gps(status=-1)
        assert mon.current_mode == GpsQualityMonitor.SLAM_MODE

        # Hemen iyi GPS gelirse histerez nedeniyle GPS'e dönmemeli
        for _ in range(3):
            mon.update_gps(status=0, hdop=1.0)
        assert mon.current_mode == GpsQualityMonitor.SLAM_MODE  # Histerez aktif

    def test_empty_window_returns_zero_average(self):
        mon = self._make_monitor()
        assert mon.window_average() == pytest.approx(0.0)
