"""
mission_state_machine.py

ARC yarışması görev durum makinesi.
Earendil-Otonomius'taki faz tabanlı mission_manager.py'den esinlenilerek
temiz, test edilebilir bir state machine mimarisine dönüştürüldü.

Durumlar:
  IDLE        → Bekleme (RSCP START komutunu bekler)
  LOCALIZING  → Lokalizasyon bekleniyor
  GPS_NAV     → GPS modunda waypoint navigasyonu
  SLAM_NAV    → SLAM modunda navigasyon (GPS-denied bölge)
  TASK_EXEC   → Görev yürütme (ArUco/rock/peak/tunnel)
  RETURNING   → Başlangıç noktasına dönüş
  COMPLETE    → Görev tamamlandı
  PAUSED      → Geçici durdurma
  EMERGENCY   → Acil durum (e-stop)

Olaylar:
  START, STOP, PAUSE, RESUME, E_STOP, GO_HOME
  GPS_OK, NO_GPS
  TASK_START(<name>), TASK_DONE, TASK_FAIL
  LOC_READY, WAYPOINTS_DONE
"""
from enum import IntEnum, auto
from typing import Optional, Dict, Callable


class MissionState(IntEnum):
    IDLE = 0
    LOCALIZING = 1
    GPS_NAV = 2
    SLAM_NAV = 3
    TASK_EXEC = 4
    RETURNING = 5
    COMPLETE = 6
    PAUSED = 7
    EMERGENCY = 8


class MissionStateMachine:
    """
    ARC görev durum makinesi.

    Kullanım::

        sm = MissionStateMachine()
        sm.handle_event("START")       # IDLE → LOCALIZING
        sm.handle_event("GPS_OK")      # LOCALIZING → GPS_NAV
        sm.handle_event("TASK_START", task_name="aruco")  # GPS_NAV → TASK_EXEC
        sm.handle_event("TASK_DONE")   # TASK_EXEC → GPS_NAV
        sm.handle_event("WAYPOINTS_DONE")  # GPS_NAV → RETURNING
        sm.handle_event("WAYPOINTS_DONE")  # RETURNING → COMPLETE
    """

    # Durum adları
    STATE_NAMES: Dict[MissionState, str] = {
        MissionState.IDLE:       'IDLE',
        MissionState.LOCALIZING: 'LOCALIZING',
        MissionState.GPS_NAV:    'GPS_NAV',
        MissionState.SLAM_NAV:   'SLAM_NAV',
        MissionState.TASK_EXEC:  'TASK_EXEC',
        MissionState.RETURNING:  'RETURNING',
        MissionState.COMPLETE:   'COMPLETE',
        MissionState.PAUSED:     'PAUSED',
        MissionState.EMERGENCY:  'EMERGENCY',
    }

    # E-stop her durumdan tetiklenebilir
    _GLOBAL_EVENTS = {'E_STOP', 'STOP'}

    def __init__(self,
                 on_state_change: Optional[Callable] = None):
        """
        :param on_state_change: Durum değişiminde çağrılır (old_state, new_state, event)
        """
        self._state = MissionState.IDLE
        self._active_task: Optional[str] = None
        self._previous_state: Optional[MissionState] = None
        self._on_state_change = on_state_change

    @property
    def current_state(self) -> MissionState:
        return self._state

    @property
    def current_state_name(self) -> str:
        return self.STATE_NAMES.get(self._state, 'UNKNOWN')

    @property
    def active_task(self) -> Optional[str]:
        return self._active_task

    def handle_event(self, event: str, **kwargs) -> MissionState:
        """
        Olay işler ve durum geçişi yapar.

        :param event: Olay adı (büyük harf)
        :param kwargs: Ek parametreler (ör. task_name='aruco')
        :returns: Yeni durum
        :raises ValueError: Geçersiz olay veya geçiş
        """
        old_state = self._state
        new_state = self._transition(event, **kwargs)

        if new_state != old_state:
            self._previous_state = old_state
            self._state = new_state
            if self._on_state_change:
                self._on_state_change(old_state, new_state, event)

        return self._state

    def _transition(self, event: str, **kwargs) -> MissionState:
        """Durum geçiş tablosu."""
        s = self._state
        e = event.upper()

        # Global olaylar: her durumdan çalışır
        if e == 'E_STOP':
            return MissionState.EMERGENCY
        if e == 'STOP' and s != MissionState.IDLE:
            self._active_task = None
            return MissionState.IDLE

        # PAUSED → RESUME
        if s == MissionState.PAUSED:
            if e == 'RESUME' and self._previous_state is not None:
                return self._previous_state
            return s

        # Durum bazlı geçişler
        if s == MissionState.IDLE:
            if e == 'START':
                return MissionState.LOCALIZING

        elif s == MissionState.LOCALIZING:
            if e == 'LOC_READY':
                return MissionState.GPS_NAV
            if e == 'GPS_OK':
                return MissionState.GPS_NAV
            if e == 'NO_GPS':
                return MissionState.SLAM_NAV
            if e == 'PAUSE':
                return MissionState.PAUSED

        elif s == MissionState.GPS_NAV:
            if e == 'NO_GPS':
                return MissionState.SLAM_NAV
            if e == 'TASK_START':
                self._active_task = kwargs.get('task_name', 'unknown')
                return MissionState.TASK_EXEC
            if e == 'WAYPOINTS_DONE':
                return MissionState.RETURNING
            if e == 'PAUSE':
                return MissionState.PAUSED

        elif s == MissionState.SLAM_NAV:
            if e == 'GPS_OK':
                return MissionState.GPS_NAV
            if e == 'TASK_START':
                self._active_task = kwargs.get('task_name', 'unknown')
                return MissionState.TASK_EXEC
            if e == 'WAYPOINTS_DONE':
                return MissionState.RETURNING
            if e == 'PAUSE':
                return MissionState.PAUSED

        elif s == MissionState.TASK_EXEC:
            if e == 'TASK_DONE':
                self._active_task = None
                return MissionState.GPS_NAV
            if e == 'TASK_FAIL':
                self._active_task = None
                return MissionState.GPS_NAV
            if e == 'NO_GPS':
                return MissionState.SLAM_NAV  # Görev devam eder

        elif s == MissionState.RETURNING:
            if e == 'WAYPOINTS_DONE':
                return MissionState.COMPLETE
            if e == 'PAUSE':
                return MissionState.PAUSED

        elif s == MissionState.COMPLETE:
            if e == 'START':
                return MissionState.LOCALIZING  # Yeni görev başlat

        elif s == MissionState.EMERGENCY:
            if e == 'RESUME':
                return MissionState.IDLE  # Güvenlik onayı sonrası sıfırla

        # Geçiş tanımsızsa mevcut durumu koru
        return s
