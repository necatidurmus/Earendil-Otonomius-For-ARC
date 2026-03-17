# Test Stratejisi — Earendil Otonomius For ARC

**Yaklaşım:** Simulation-first  
**Çerçeve:** ROS 2 Humble | pytest | ament_cmake_pytest | Gazebo Ignition

---

## 1. Test Felsefesi

**"Simülasyonda kırılmayan kod, rovera binmez."**

Her özellik şu piramit mantığıyla test edilir:

```
        ▲ Sistem Testi (End-to-End, Gazebo)
       ▲▲▲ Entegrasyon Testi (Node-to-Node)
      ▲▲▲▲▲ Unit Test (Modül/Fonksiyon)
```

- **Unit testler hızlıdır** — CI'da her PR'da çalışır
- **Entegrasyon testleri orta hızda** — günlük CI'da çalışır
- **Sistem testleri yavaştır** — merge öncesi veya haftalık çalışır

---

## 2. Unit Test Yaklaşımı

### 2.1 Araçlar
- `pytest` + `ament_cmake_pytest`
- `unittest.mock` (ROS node'ları mock'lamak için)
- `rclpy` minimal node test pattern'i

### 2.2 Her Paket için Test Hedefleri

#### `earendil_mission`
```python
# test/test_state_machine.py
# Test edilecekler:
# - IDLE → LOCALIZING geçişi (START komutu)
# - GPS_NAV → SLAM_NAV geçişi (gps_quality < threshold)
# - TASK_EXEC → RETURNING geçişi (görev tamamlandı)
# - Her durumdan EMERGENCY geçişi (e_stop)
# - Geçersiz geçiş denemeleri reddedilir
```

#### `earendil_localization`
```python
# test/test_gps_monitor.py
# Test edilecekler:
# - GPS kalitesi hesaplama (status + HDOP)
# - Histerez penceresi (window_size ölçümü ortalaması)
# - Mod geçişi gecikme (hysteresis_secs)
# - GPS timeout → kalite=0.0
# - Confirmation count öncesi geçiş yapılmaz
```

#### `earendil_safety`
```python
# test/test_safety_supervisor.py
# Test edilecekler:
# - cmd_vel watchdog: timeout süre sonra dur komutu
# - E-stop aktifken cmd_vel geçmez
# - Tilt aşımı → e-stop tetiklenir
# - Normal operasyonda cmd_vel iletilir
```

#### `earendil_rscp`
```python
# test/test_rscp_bridge.py
# Test edilecekler:
# - RscpCommand parse
# - Bağlantı kesilmesi → güvenli mod
# - Telemetri formatı doğrulama
```

### 2.3 Unit Test Örnek Şablonu

```python
# src/earendil_mission/test/test_state_machine.py
import pytest
from earendil_mission.mission_state_machine import MissionStateMachine, MissionState

class TestMissionStateMachine:
    def setup_method(self):
        self.sm = MissionStateMachine()

    def test_initial_state_is_idle(self):
        assert self.sm.current_state == MissionState.IDLE

    def test_start_command_transitions_to_localizing(self):
        self.sm.handle_event("START")
        assert self.sm.current_state == MissionState.LOCALIZING

    def test_estop_always_transitions_to_emergency(self):
        for state in [MissionState.GPS_NAV, MissionState.TASK_EXEC]:
            self.sm.current_state = state
            self.sm.handle_event("E_STOP")
            assert self.sm.current_state == MissionState.EMERGENCY

    def test_invalid_transition_raises(self):
        with pytest.raises(ValueError):
            self.sm.handle_event("INVALID_EVENT")
```

---

## 3. Entegrasyon Testi Yaklaşımı

### 3.1 Node-to-Node Testler

ROS 2 launch_testing ile:

```python
# test/test_localization_integration.py
import launch_testing
import pytest
import rclpy
from std_msgs.msg import String

@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        Node(package='earendil_localization', executable='gps_monitor_node'),
        launch_testing.actions.ReadyToTest()
    ])

class TestGpsMonitorIntegration(unittest.TestCase):
    def test_gps_mode_published(self):
        # GPS mesajı yayınla, mod topic'ini dinle
        # GPS kalitesi iyi → "gps" modu yayınlanmalı
        ...
```

### 3.2 Mission-Navigation Entegrasyon

```python
# Simülasyonsuz, Nav2 Action mock ile:
# - Mission manager bir hedef noktası gönderir
# - Waypoint manager Nav2 action'a iletir
# - Başarı callback'i mission manager'a döner
```

---

## 4. Sistem Testi (End-to-End Simülasyon)

### 4.1 Test Senaryoları

#### Senaryo 1: GPS Navigasyon
```bash
# Koşul: GPS sinyali var, açık alan
# Beklenen:
# 1. RSCP START komutu alınır
# 2. LOCALIZING → GPS_NAV
# 3. 3 waypoint sırayla geçilir
# 4. COMPLETE
# Başarı: 3/3 waypoint ✅, süre < 120s
```

#### Senaryo 2: SLAM Geçişi (Tünel)
```bash
# Koşul: GPS_NAV → tünel girişi → GPS kaybı
# Beklenen:
# 1. GPS kalitesi 0.3'ün altına düşer
# 2. SLAM_NAV moduna geçilir
# 3. SLAM Toolbox aktif, tünel geçilir
# 4. Tünel çıkışında GPS kalitesi 0.45'in üzerine çıkar
# 5. GPS_NAV moduna dönülür
# Başarı: GPS/SLAM geçişleri doğru ✅
```

#### Senaryo 3: E-Stop Güvenliği
```bash
# Koşul: Görev devam ederken e_stop=true
# Beklenen:
# 1. cmd_vel sıfırlanır (dur)
# 2. Mission state → EMERGENCY
# 3. RSCP'ye acil durum bildirimi
# Başarı: Rover 0.5s içinde duruyor ✅
```

#### Senaryo 4: ArUco Görevi
```bash
# Koşul: ArUco marker simülasyona yerleştirilmiş
# Beklenen:
# 1. TASK_EXEC: aruco_search başlar
# 2. Marker tespit edilir
# 3. Rover marker önünde durur (1.0m)
# 4. Task tamamlandı sinyali
# Başarı: Marker tespit edildi ✅, mesafe < 1.5m
```

### 4.2 Sistem Test Scripti

```bash
# scripts/run_arc_full_test.sh
#!/bin/bash
# 1. Docker başlat
# 2. colcon build
# 3. arc_arena.sdf dünya başlat
# 4. Tüm stack başlat (lokalizasyon + navigasyon + mission)
# 5. RSCP üzerinden START komutu
# 6. Tüm ARC görevlerini çalıştır
# 7. COMPLETE beklenir, süre ölçülür
# 8. PASS/FAIL raporu
```

---

## 5. CI/CD Test Pipeline

```yaml
# .github/workflows/ci.yml

on: [push, pull_request]

jobs:
  lint:
    # ament_flake8, ament_pep257, ament_copyright
    
  build:
    # colcon build --packages-select earendil_*
    
  unit-test:
    # colcon test --packages-select earendil_*
    # pytest ile unit testler
    
  # integration-test:  (ileride, sim gerektirir)
  #   Gazebo headless modda
```

---

## 6. Test Coverage Hedefleri

| Faz | Unit | Entegrasyon | Sistem |
|-----|------|------------|--------|
| Faz 1 | %30 | - | Manuel |
| Faz 2 | %50 | %20 | Senaryo 1-2 |
| Faz 3 | %70 | %40 | Senaryo 1-4 |
| Faz 4 | %70 | %50 | Gerçek + Sim |

---

## 7. Simülasyon Test Araçları

### Rosbag Replay
```bash
# Bir kaydı tekrar oynat ve sonuçları analiz et
ros2 bag play rosbags/arc_test_2025-03-01/
ros2 bag info rosbags/arc_test_2025-03-01/
```

### Topic İzleme
```bash
# Gerçek zamanlı topic izleme
ros2 topic echo /mission/state
ros2 topic hz /odometry/filtered
ros2 topic echo /localization/mode
```

### RViz ile Görsel Doğrulama
```bash
ros2 launch earendil_bringup rviz.launch.py
# → Lokalizasyon, navigasyon yolu, algılama sonuçları görülür
```

---

## 8. Bilinen Test Kısıtları

| Kısıt | Açıklama |
|-------|----------|
| Gazebo GPU | Bazı CI ortamlarında headless Gazebo çalışmaz |
| SLAM başlatma | SLAM Toolbox 15–20s başlamayı bekliyor |
| GPS spoofer | Sadece Gazebo simülasyonunda çalışır |
| Gerçek rover | Faz 4'e kadar testler simülasyonla sınırlı |
