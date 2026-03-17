# Yazılım Mimarisi — Earendil Otonomius For ARC

**Sürüm:** 0.1-dev  
**Platform:** ROS 2 Humble | Gazebo Ignition | Jetson Orin Nano (hedef)  
**Yaklaşım:** Simulation-first, modüler katmanlı mimari

---

## 1. Mimari Genel Bakış

```
┌───────────────────────────────────────────────────────────────────┐
│                     UZAK İSTASYON (Ground)                        │
│              Web UI / MATLAB / RSCP Terminal                      │
└───────────────────────────┬───────────────────────────────────────┘
                            │ WebSocket / Serial (RSCP protokolü)
                            ▼
╔═══════════════════════════════════════════════════════════════════╗
║                        RSCP Bridge                                ║
║  earendil_rscp · rscp_bridge_node                                 ║
║  /rscp/command [RscpCommand] → /mission/rscp_in                   ║
║  /mission/telemetry → uzak istasyona                              ║
╚══════════════════════════╦════════════════════════════════════════╝
                           ║
           ┌───────────────▼────────────────┐
           │         Mission Manager         │
           │   earendil_mission              │
           │   ┌─────────────────────────┐   │
           │   │   Mission State Machine │   │
           │   │ IDLE→LOCALIZING→        │   │
           │   │ GPS_NAV / SLAM_NAV→     │   │
           │   │ TASK_EXEC→RETURNING→    │   │
           │   │ COMPLETE / EMERGENCY    │   │
           │   └─────────────────────────┘   │
           │   ┌─────────────────────────┐   │
           │   │   Task Executor         │   │
           │   │ aruco / rock / peak /   │   │
           │   │ tunnel tasks            │   │
           │   └─────────────────────────┘   │
           └──┬────────────┬────────────┬────┘
              │            │            │
   ┌──────────▼──┐  ┌──────▼────┐  ┌───▼──────────────┐
   │ Navigation  │  │Perception │  │  Localization     │
   │ earendil_   │  │earendil_  │  │  earendil_        │
   │ navigation  │  │perception │  │  localization     │
   │             │  │           │  │                   │
   │ Nav2 Stack  │  │ArUco Det. │  │ UKF Local         │
   │ WP Manager  │  │Rock Det.  │  │ UKF Global        │
   │ Costmap     │  │Peak Find  │  │ GPS Monitor       │
   │             │  │(stub)     │  │ TF Mode Relay     │
   └──────┬──────┘  └────┬──────┘  │ SLAM Toolbox      │
          │              │         └──────────────────┘
          └──────────────┼──────────────────┐
                         │                  │
              ┌──────────▼──────────┐       │
              │   Safety Supervisor │       │
              │   earendil_safety   │       │
              │ cmd_vel → filtered  │       │
              │ watchdog / e-stop   │       │
              │ tilt / battery      │       │
              └──────────┬──────────┘       │
                         │                  │
              ┌──────────▼──────────┐       │
              │   /cmd_vel (gerçek) │       │
              │   Motor Controller  │       │
              └─────────────────────┘       │
                                            │
              ┌─────────────────────────────▼──┐
              │      Session Logger             │
              │      earendil_logging           │
              │  Rosbag · Event Log · Replay    │
              └─────────────────────────────────┘

  Teleop (earendil_teleop) ─► Safety Supervisor (override path)
  MATLAB Adapter (stub)    ─► Mission Manager (northbound stub)
```

---

## 2. Katmanlar ve Sorumluluklar

### 2.1 RSCP Katmanı (`earendil_rscp`)

**Amaç:** Uzak istasyon ile rover arasındaki haberleşme köprüsü.

**Sorumluluklar:**
- Uzak istasyondan `RscpCommand` mesajlarını alır
- Telemetri verilerini uzak istasyona iletir
- Bağlantı kesilmesinde güvenli mod tetikler
- Protokol: WebSocket (simülasyon) / Serial/UDP (gerçek rover)

**Topics/Services:**
```
Publish:  /rscp/command          [earendil_msgs/RscpCommand]
          /rscp/connection_status [std_msgs/String]
Subscribe:/rscp/telemetry_out    [earendil_msgs/MissionState]
```

---

### 2.2 Mission Katmanı (`earendil_mission`)

**Amaç:** Yarışma görevlerini yönetir, state machine ile stage orchestration yapar.

**Sorumluluklar:**
- RSCP komutlarını alır, mission state machine'i yönetir
- Stage-based görev sıralaması (ARC görev aşamaları)
- Task Executor'ı tetikler (ArUco, rock, peak, tunnel)
- Nav2 Action Client aracılığıyla navigasyonu yönetir
- Durum güncellemelerini RSCP'ye iletir

**State Machine:**
```
IDLE ──(START)──► LOCALIZING ──(gps_ok)──► GPS_NAV
                      │                        │
                      └──(no_gps)──► SLAM_NAV  │
                                         │     │
                              ◄──────────┘     │
                              TASK_EXEC ◄───────┘
                                  │
                              RETURNING ──► COMPLETE
                              
  Her durumdan ──(e_stop)──► EMERGENCY
```

**Topics/Services:**
```
Subscribe: /rscp/command           [earendil_msgs/RscpCommand]
           /localization/mode      [std_msgs/String]
Publish:   /mission/state          [earendil_msgs/MissionState]
           /mission/task_request   [earendil_msgs/TaskStatus]
Action:    NavigateToPose (client)
```

---

### 2.3 Localization Katmanı (`earendil_localization`)

**Amaç:** Mevcut Earendil-Otonomius'taki Dual-UKF GPS+SLAM hibrit lokalizasyon.

**Sorumluluklar:**
- UKF Local: `odom + IMU` füzyonu → `/odometry/local`
- UKF Global: `local + GPS` → `/odometry/filtered`
- GPS Monitor: GPS kalitesini izler, SLAM/GPS geçişini tetikler
- TF Mode Relay: Aktif moda göre `map→odom` TF yayınlar
- SLAM Toolbox: GPS-denied ortamda harita ve lokalizasyon

**Mod Geçiş Mantığı (mevcut repos'tan alındı):**
```
GPS Kalitesi (0.0–1.0):
  < 0.30 → SLAM moduna geç
  > 0.45 → GPS moduna dön
  Histerez: 3.0s, onay sayısı: 3
```

**Topics:**
```
Subscribe: /fix                    [sensor_msgs/NavSatFix]
           /imu/data               [sensor_msgs/Imu]
           /odom                   [nav_msgs/Odometry]
Publish:   /odometry/local        [nav_msgs/Odometry]
           /odometry/filtered     [nav_msgs/Odometry]
           /localization/mode     [std_msgs/String]  # "gps" | "slam"
           /tf (map→odom)
```

---

### 2.4 Navigation Katmanı (`earendil_navigation`)

**Amaç:** Nav2 navigasyon stack'inin wrapper'ı ve waypoint yöneticisi.

**Sorumluluklar:**
- Nav2 stack'ini başlatır ve yönetir
- ARC waypoint listesini yükler ve sırayla çalıştırır
- Görev tabanlı hedef noktaları alır, Nav2 Action'a çevirir
- Engel tespitinde yeniden planlama tetikler

---

### 2.5 Perception Katmanı (`earendil_perception`)

**Amaç:** Görev bazlı genişletilebilir algılama modülü.

**Task Modülleri:**

| Modül | Durum | Açıklama |
|-------|-------|----------|
| `aruco_detector` | 🔧 Stub | ArUco marker tespiti (OpenCV) |
| `rock_detector` | 🔧 Stub | Kaya/engel tespiti (kamera) |
| `peak_finder` | 🔧 Stub | Tepe noktası arama (elevation map) |
| `tunnel_detector` | 🔧 Stub | Tünel giriş/çıkış tespiti |

**Topics:**
```
Subscribe: /camera/image_raw      [sensor_msgs/Image]
           /scan                  [sensor_msgs/LaserScan]
Publish:   /perception/aruco      [geometry_msgs/PoseArray]
           /perception/targets    [earendil_msgs/TaskStatus]
```

---

### 2.6 Safety Katmanı (`earendil_safety`)

**Amaç:** Merkezi safety supervisor — tüm actuator komutları buradan geçer.

**Sorumluluklar:**
- `cmd_vel` mesajlarını filtreler/engeller
- IMU'dan tilt kontrolü
- Watchdog: görev komutları gelmezse dur
- E-Stop: RSCP veya yerel tetikleyici
- Batarya izleme (stub, gerçek rover için)
- Safety durumunu düzenli yayınlar

**Topics:**
```
Subscribe: /cmd_vel_unsafe        [geometry_msgs/Twist]  # gelen
           /imu/data              [sensor_msgs/Imu]
           /e_stop                [std_msgs/Bool]
           /safety/battery_pct   [std_msgs/Float32]
Publish:   /cmd_vel               [geometry_msgs/Twist]  # onaylı
           /safety/status        [std_msgs/String]
```

---

### 2.7 Teleop Katmanı (`earendil_teleop`)

**Amaç:** Manuel operasyon ve otonomdan manuel'e override.

**Sorumluluklar:**
- Joystick / klavye girişi → `/cmd_vel_teleop`
- Web arayüzü teleop (opsiyonel, stub)
- Manual override switch (öncelik: teleop > otonom)
- Override durumunda mission manager'ı durdurur

**Topics:**
```
Subscribe: /joy                   [sensor_msgs/Joy]
Publish:   /cmd_vel_teleop        [geometry_msgs/Twist]
           /teleop/mode           [std_msgs/String]  # "manual" | "auto"
```

---

### 2.8 RSCP Katmanı (`earendil_rscp`)

Bkz. 2.1

---

### 2.9 Logging Katmanı (`earendil_logging`)

**Amaç:** Rosbag session yönetimi ve olay kaydı.

**Sorumluluklar:**
- Görev başında/sonunda rosbag kaydını başlatır/durdurur
- Önemli olayları (görev geçişleri, hatalar) event log'a yazar
- Replay için rosbag meta verisi üretir

---

### 2.10 MATLAB Adapter (`earendil_matlab_adapter`)

**Amaç:** MATLAB tarafından gönderilen komutları ROS 2'ye köprüler (northbound stub).

**Durum:** 🔧 Stub — gerçek implementasyon ileride

---

## 3. Topic ve Servis Haritası

```
Sensörler:
  /fix                → sensor_msgs/NavSatFix
  /imu/data           → sensor_msgs/Imu
  /odom               → nav_msgs/Odometry
  /scan               → sensor_msgs/LaserScan
  /camera/image_raw   → sensor_msgs/Image

Lokalizasyon:
  /odometry/local     → nav_msgs/Odometry
  /odometry/filtered  → nav_msgs/Odometry
  /localization/mode  → std_msgs/String

Navigasyon:
  /cmd_vel_nav        → geometry_msgs/Twist

Görev:
  /rscp/command       → earendil_msgs/RscpCommand
  /mission/state      → earendil_msgs/MissionState
  /mission/task_request → earendil_msgs/TaskStatus

Güvenlik:
  /e_stop             → std_msgs/Bool
  /cmd_vel            → geometry_msgs/Twist  (onaylı)
  /safety/status      → std_msgs/String

Teleop:
  /cmd_vel_teleop     → geometry_msgs/Twist
  /teleop/mode        → std_msgs/String
```

---

## 4. Simülasyon vs Gerçek Rover

| Bileşen | Simülasyon | Gerçek Rover |
|---------|-----------|-------------|
| Robot | Gazebo URDF | Donanım sürücüleri |
| GPS | Gazebo plugin + spoofer | U-blox / NMEA |
| IMU | Gazebo plugin | BNO055 / ICM-42688 |
| LiDAR | Gazebo ray sensor | RPLiDAR / Velodyne |
| Kamera | Gazebo camera | USB / MIPI CSI |
| Motor | DiffDrive plugin | PWM / CAN sürücüsü |
| Bringup | `sim_bringup.launch.py` | `robot_bringup.launch.py` |

Simülasyon ve gerçek rover bringup tamamen ayrı launch dosyalarındadır.
Ortak paketler (mission, safety, navigation, perception) her iki ortamda da aynı çalışır.

---

## 5. Isimlendirme Kuralları

- **Paket:** `earendil_<katman>` (snake_case)
- **Node:** `<katman>_node` (örn: `mission_manager_node`)
- **Topic:** `/<katman>/<isim>` (örn: `/mission/state`)
- **Mesaj:** `PascalCase` (örn: `MissionState.msg`)
- **Python modül:** `snake_case` (örn: `mission_state_machine.py`)
- **Launch:** `<amaç>.launch.py` (örn: `sim_bringup.launch.py`)
- **Config:** `<konu>_params.yaml` (örn: `safety_params.yaml`)

---

## 6. Bağımlılık Grafiği

```
earendil_msgs (bağımsız, base)
    ↑
    ├─ earendil_localization
    ├─ earendil_navigation
    ├─ earendil_mission ← earendil_perception
    ├─ earendil_safety
    ├─ earendil_teleop
    ├─ earendil_rscp
    └─ earendil_logging

earendil_description (bağımsız, base)
    ↑
    ├─ earendil_bringup
    └─ earendil_simulation
```
