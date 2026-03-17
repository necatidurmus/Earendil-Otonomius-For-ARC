"""
waypoint_manager.py

Waypoint yöneticisi — ARC görev noktalarını sırayla Nav2'ye iletir.
Earendil-Otonomius'taki mission_manager.py'deki waypoint döngüsünden
bağımsız modüle ayrıldı.

Earendil-Otonomius referansı:
  9 waypoint, 3 faz: GPS(4) → SLAM(2) → GPS(3)

Bu sınıf:
  - YAML'dan waypoint listesi yükler
  - Nav2 NavigateToPose action client'ı yönetir
  - Başarı/başarısızlık callback'lerini mission manager'a bildirir
"""
import yaml
from typing import List, Dict, Any, Optional, Callable


class Waypoint:
    """Tekil waypoint tanımı."""

    def __init__(self, name: str, x: float, y: float,
                 yaw: float = 0.0, nav_mode: str = 'gps',
                 task: Optional[str] = None):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.nav_mode = nav_mode  # 'gps' | 'slam'
        self.task = task          # Opsiyonel görev adı

    def __repr__(self) -> str:
        return f'Waypoint({self.name}, x={self.x}, y={self.y}, mode={self.nav_mode})'


class WaypointManager:
    """
    ARC waypoint listesi yöneticisi.

    Kullanım::

        wm = WaypointManager()
        wm.load_from_yaml('/path/to/arc_waypoints.yaml')
        wp = wm.next_waypoint()
        while wp:
            # Nav2'ye gönder
            wm.mark_reached(wp.name)
            wp = wm.next_waypoint()
    """

    def __init__(self):
        self._waypoints: List[Waypoint] = []
        self._current_index: int = 0
        self._completed: List[str] = []

    def load_from_yaml(self, path: str) -> None:
        """YAML dosyasından waypoint listesi yükler."""
        with open(path) as f:
            data = yaml.safe_load(f)

        self._waypoints = []
        for wp_data in data.get('waypoints', []):
            self._waypoints.append(Waypoint(
                name=wp_data['name'],
                x=wp_data['x'],
                y=wp_data['y'],
                yaw=wp_data.get('yaw', 0.0),
                nav_mode=wp_data.get('nav_mode', 'gps'),
                task=wp_data.get('task'),
            ))
        self._current_index = 0
        self._completed = []

    def load_from_list(self, waypoints: List[Dict[str, Any]]) -> None:
        """Sözlük listesinden waypoint yükler (test için)."""
        self._waypoints = [
            Waypoint(**{k: v for k, v in wp.items()}) for wp in waypoints
        ]
        self._current_index = 0
        self._completed = []

    def next_waypoint(self) -> Optional[Waypoint]:
        """Sıradaki waypoint'i döndürür. Tümü tamamlandıysa None."""
        if self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index]
        return None

    def mark_reached(self, name: str) -> None:
        """Waypoint'i tamamlandı olarak işaretler."""
        if (self._current_index < len(self._waypoints)
                and self._waypoints[self._current_index].name == name):
            self._completed.append(name)
            self._current_index += 1

    def reset(self) -> None:
        """Waypoint listesini başa alır."""
        self._current_index = 0
        self._completed = []

    @property
    def is_complete(self) -> bool:
        return self._current_index >= len(self._waypoints)

    @property
    def progress(self) -> float:
        if not self._waypoints:
            return 0.0
        return len(self._completed) / len(self._waypoints) * 100.0

    @property
    def completed_count(self) -> int:
        return len(self._completed)

    @property
    def total_count(self) -> int:
        return len(self._waypoints)
