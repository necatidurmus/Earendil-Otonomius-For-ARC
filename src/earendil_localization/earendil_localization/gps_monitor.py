"""
gps_monitor.py

GPS kalite izleme ve GPS↔SLAM mod geçiş kontrolü.
Earendil-Otonomius v0.2'deki gps_monitor.py'den modüler hale getirildi.

Kalite hesaplama:
  - NavSatStatus.STATUS_NO_FIX   → kalite = 0.0
  - NavSatStatus.STATUS_FIX      → kalite = 0.5 (temel)
  - NavSatStatus.STATUS_SBAS_FIX → kalite = 0.75
  - NavSatStatus.STATUS_GBAS_FIX → kalite = 1.0
  HDOP varsa ek düzeltme uygulanır.

Mod geçiş mantığı:
  GPS → SLAM: window_avg < gps_to_slam_threshold (3 ardışık onay)
  SLAM → GPS: window_avg > slam_to_gps_threshold (3 ardışık onay)
  Histerez: min hysteresis_secs beklenir
"""
import collections
import time
from typing import Optional


class GpsQualityMonitor:
    """GPS kalitesini izler ve mod geçiş kararı verir."""

    GPS_MODE = 'gps'
    SLAM_MODE = 'slam'

    def __init__(
        self,
        gps_to_slam_threshold: float = 0.3,
        slam_to_gps_threshold: float = 0.45,
        window_size: int = 10,
        hysteresis_secs: float = 3.0,
        gps_timeout_secs: float = 3.0,
        confirmation_count: int = 3,
    ):
        self._gps_to_slam_threshold = gps_to_slam_threshold
        self._slam_to_gps_threshold = slam_to_gps_threshold
        self._window_size = window_size
        self._hysteresis_secs = hysteresis_secs
        self._gps_timeout_secs = gps_timeout_secs
        self._confirmation_count = confirmation_count

        self._quality_window: collections.deque = collections.deque(
            maxlen=window_size
        )
        self._current_mode = self.GPS_MODE
        self._last_mode_change_time = 0.0
        self._last_gps_time: Optional[float] = None
        self._pending_mode: Optional[str] = None
        self._pending_count = 0

    @property
    def current_mode(self) -> str:
        return self._current_mode

    def update_gps(self, status: int, hdop: float = 1.0) -> Optional[str]:
        """
        Yeni GPS ölçümü ile kaliteyi günceller.

        Akış:
          1. GPS zaman damgasını kaydet
          2. Kalite skorunu hesapla ve pencereye ekle
          3. Mod geçişi gerekip gerekmediğini kontrol et

        :param status: NavSatStatus.status değeri (NO_FIX=−1, FIX=0, SBAS=1, GBAS=2)
        :param hdop: Yatay dilüsyon (küçük = iyi)
        :return: Yeni mod (geçiş olduysa) veya None
        """
        self._last_gps_time = time.monotonic()
        self._update_quality_window(status, hdop)
        return self._check_transition()

    def _update_quality_window(self, status: int, hdop: float) -> None:
        """Kalite skorunu hesaplar ve kayan pencereye ekler."""
        quality = self._compute_quality(status, hdop)
        self._quality_window.append(quality)

    def update_timeout_check(self) -> Optional[str]:
        """
        GPS mesajı timeout kontrolü.
        Bu metot düzenli olarak çağrılmalıdır.
        """
        if self._last_gps_time is None:
            return None
        elapsed = time.monotonic() - self._last_gps_time
        if elapsed > self._gps_timeout_secs:
            # Timeout: NO_FIX gibi davran, kalite = 0.0
            self._update_quality_window(status=-1, hdop=10.0)
            return self._check_transition()
        return None

    def window_average(self) -> float:
        if not self._quality_window:
            return 0.0
        return sum(self._quality_window) / len(self._quality_window)

    def _compute_quality(self, status: int, hdop: float) -> float:
        """GPS kalite skoru hesaplar (0.0–1.0)."""
        # Temel skor status'a göre
        base_scores = {-1: 0.0, 0: 0.5, 1: 0.75, 2: 1.0}
        base = base_scores.get(status, 0.0)
        if base == 0.0:
            return 0.0
        # HDOP düzeltmesi: HDOP < 1.5 iyi, > 5.0 kötü
        hdop_factor = max(0.0, min(1.0, (5.0 - hdop) / (5.0 - 1.5)))
        return base * (0.7 + 0.3 * hdop_factor)

    def _check_transition(self) -> Optional[str]:
        """Mod geçişi gerekli mi kontrol eder. Geçiş olduysa yeni modu döndürür."""
        if len(self._quality_window) < self._window_size:
            return None  # Yeterli veri yok

        avg = self.window_average()
        now = time.monotonic()
        elapsed_since_change = now - self._last_mode_change_time

        if elapsed_since_change < self._hysteresis_secs:
            return None  # Histerez süresi dolmadı

        desired_mode = self._current_mode
        if self._current_mode == self.GPS_MODE and avg < self._gps_to_slam_threshold:
            desired_mode = self.SLAM_MODE
        elif self._current_mode == self.SLAM_MODE and avg > self._slam_to_gps_threshold:
            desired_mode = self.GPS_MODE

        if desired_mode != self._current_mode:
            if self._pending_mode == desired_mode:
                self._pending_count += 1
            else:
                self._pending_mode = desired_mode
                self._pending_count = 1

            if self._pending_count >= self._confirmation_count:
                self._current_mode = desired_mode
                self._last_mode_change_time = now
                self._pending_mode = None
                self._pending_count = 0
                return desired_mode
        else:
            self._pending_mode = None
            self._pending_count = 0

        return None
