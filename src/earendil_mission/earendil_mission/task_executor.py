"""
task_executor.py

ARC görev yürütücü — modüler görev plug-in sistemi.
Her görev bağımsız bir sınıf olarak tanımlanır ve TaskExecutor tarafından yönetilir.

Mevcut görevler:
  - ArucoTask     : ArUco marker arama ve yaklaşma
  - RockTask      : Kaya/engel tespit (STUB)
  - PeakTask      : Tepe noktası arama (STUB)
  - TunnelTask    : Tünel geçişi koordinasyonu (STUB)

Yeni görev eklemek için:
  1. BaseTask'tan türet
  2. execute() metodunu uygula
  3. TaskExecutor.TASK_REGISTRY'ye kaydet
"""
import time
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any


class TaskResult:
    """Görev tamamlama sonucu."""

    SUCCESS = 'success'
    FAILED = 'failed'
    CANCELLED = 'cancelled'

    def __init__(self, status: str, data: Optional[Dict[str, Any]] = None,
                 error: str = ''):
        self.status = status
        self.data = data or {}
        self.error = error
        self.elapsed_secs = 0.0

    @property
    def success(self) -> bool:
        return self.status == self.SUCCESS

    def __repr__(self) -> str:
        return f'TaskResult(status={self.status}, error={self.error!r})'


class BaseTask(ABC):
    """Tüm görevlerin türetildiği temel sınıf."""

    def __init__(self, name: str):
        self.name = name
        self._cancelled = False
        self._start_time: Optional[float] = None

    def cancel(self) -> None:
        self._cancelled = True

    @property
    def is_cancelled(self) -> bool:
        return self._cancelled

    @property
    def elapsed_secs(self) -> float:
        if self._start_time is None:
            return 0.0
        return time.monotonic() - self._start_time

    @abstractmethod
    def execute(self, params: Dict[str, Any]) -> TaskResult:
        """
        Görevi çalıştır.

        :param params: Görev parametreleri
        :returns: TaskResult
        """

    def _start(self) -> None:
        self._start_time = time.monotonic()
        self._cancelled = False


class ArucoTask(BaseTask):
    """
    ArUco marker arama ve yaklaşma görevi.

    Davranış:
      1. Kamera görüntüsünü izle
      2. Belirtilen marker ID'lerini ara
      3. Marker bulununca yaklaş (approach_dist_m)
      4. Durumu döndür
    """

    def __init__(self):
        super().__init__('aruco')

    def execute(self, params: Dict[str, Any]) -> TaskResult:
        self._start()
        marker_ids = params.get('marker_ids', [0])
        approach_dist = params.get('approach_dist_m', 1.0)

        # TODO (Faz 2): Gerçek ArUco tespit implementasyonu
        # - /perception/aruco topic'ini dinle
        # - Hedef marker ID görününce Nav2 ile yaklaş
        # - approach_dist_m mesafede dur

        result = TaskResult(TaskResult.SUCCESS,
                            data={'marker_ids_found': [], 'approach_dist': approach_dist})
        result.elapsed_secs = self.elapsed_secs
        return result


class RockTask(BaseTask):
    """
    Kaya/engel tespit görevi — STUB.
    TODO (Faz 2): Kamera tabanlı kaya sınıflandırması
    """

    def __init__(self):
        super().__init__('rock')

    def execute(self, params: Dict[str, Any]) -> TaskResult:
        self._start()
        # TODO (Faz 2): LiDAR + kamera tabanlı kaya tespiti
        result = TaskResult(TaskResult.SUCCESS,
                            data={'rocks_found': 0})
        result.elapsed_secs = self.elapsed_secs
        return result


class PeakTask(BaseTask):
    """
    Tepe noktası arama görevi — STUB.
    TODO (Faz 2): Elevation map tabanlı tepe arama
    """

    def __init__(self):
        super().__init__('peak')

    def execute(self, params: Dict[str, Any]) -> TaskResult:
        self._start()
        search_radius = params.get('search_radius_m', 15.0)
        # TODO (Faz 2): /elevation_map'ten en yüksek noktayı bul, navigasyon yap
        result = TaskResult(TaskResult.SUCCESS,
                            data={'peak_position': None, 'search_radius': search_radius})
        result.elapsed_secs = self.elapsed_secs
        return result


class TunnelTask(BaseTask):
    """
    Tünel geçiş koordinasyon görevi — STUB.
    GPS→SLAM geçişini tetikler, tünel boyunca navigasyonu yönetir.
    TODO (Faz 2): Tünel giriş/çıkış tespiti, SLAM mod doğrulaması
    """

    def __init__(self):
        super().__init__('tunnel')

    def execute(self, params: Dict[str, Any]) -> TaskResult:
        self._start()
        # TODO (Faz 2): Tünel içinde SLAM navigasyonu yönet
        result = TaskResult(TaskResult.SUCCESS,
                            data={'tunnel_traversed': True})
        result.elapsed_secs = self.elapsed_secs
        return result


class TaskExecutor:
    """
    Görev kaydı ve yürütme yöneticisi.
    Mission Manager tarafından kullanılır.
    """

    TASK_REGISTRY: Dict[str, type] = {
        'aruco':  ArucoTask,
        'rock':   RockTask,
        'peak':   PeakTask,
        'tunnel': TunnelTask,
    }

    def __init__(self):
        self._current_task: Optional[BaseTask] = None

    def execute(self, task_name: str,
                params: Optional[Dict[str, Any]] = None) -> TaskResult:
        """
        Görevi çalıştırır.

        :param task_name: Görev adı ('aruco', 'rock', 'peak', 'tunnel')
        :param params: Görev parametreleri
        :returns: TaskResult
        :raises ValueError: Bilinmeyen görev adı
        """
        if task_name not in self.TASK_REGISTRY:
            raise ValueError(
                f"Bilinmeyen görev: '{task_name}'. "
                f"Geçerli görevler: {list(self.TASK_REGISTRY)}"
            )

        task_class = self.TASK_REGISTRY[task_name]
        self._current_task = task_class()
        return self._current_task.execute(params or {})

    def cancel_current(self) -> None:
        """Mevcut görevi iptal eder."""
        if self._current_task is not None:
            self._current_task.cancel()
            self._current_task = None

    @property
    def available_tasks(self):
        return list(self.TASK_REGISTRY.keys())
