# Earendil-Otonomius-For-ARC

> **ARC (Anatolian Rover Challenge) odaklı, simülasyon-first modüler rover yazılım platformu**
> ROS 2 Humble | Gazebo Ignition | Dual-UKF GPS+SLAM | Stage-based Mission Orchestration

[![CI](https://github.com/necatidurmus/Earendil-Otonomius-For-ARC/actions/workflows/ci.yml/badge.svg)](https://github.com/necatidurmus/Earendil-Otonomius-For-ARC/actions/workflows/ci.yml)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/Platform-Simulation%20%7C%20Jetson%20Orin%20Nano-green)
![Status](https://img.shields.io/badge/Status-Development-yellow)

---

## Proje Hakkında

Bu repository, ARC yarışması için sıfırdan tasarlanmış modüler bir rover yazılım platformudur.
Earendil-Otonomius'taki **Dual-UKF GPS+SLAM hibrit navigasyon** mimarisi baz alınmış;
üzerine yarışma görev mantığı (RSCP, stage-based mission, ArUco/rock/peak/tunnel görevleri),
safety supervisor, merkezi localization manager ve temiz katmanlı bir monorepo yapısı eklenmiştir.

**Temel ilkeler:**
- 🧪 **Simulation-first** — Her özellik önce Gazebo'da test edilir, gerçek rovera sonra taşınır
- 🧩 **Modüler** — Her katman bağımsız pakettir, birbirini değiştirmez
- 🏁 **ARC odaklı** — RSCP komut akışı, stage-based mission, ARC-spesifik görevler
- 🔒 **Safety-first** — Merkezi safety supervisor tüm actuator komutlarının önündedir

---

## Mimari Genel Bakış

```
┌─────────────────────────────────────────────────────────────┐
│                     RSCP Bridge                             │
│              (Uzak İstasyon ↔ Rover haberleşme)             │
└───────────────────────┬─────────────────────────────────────┘
                        │ rscp_command
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                   Mission Manager                           │
│        (State Machine | Stage Orchestration | Tasks)        │
└────┬──────────────┬──────────────┬───────────────┬──────────┘
     │              │              │               │
     ▼              ▼              ▼               ▼
┌─────────┐  ┌──────────┐  ┌──────────┐  ┌───────────────┐
│Navigation│  │Perception│  │ Locali-  │  │    Teleop     │
│(Nav2+WP) │  │(ArUco/   │  │ zation   │  │ (Manual Over- │
│          │  │rock/peak)│  │(UKF/SLAM)│  │    ride)      │
└─────────┘  └──────────┘  └──────────┘  └───────────────┘
                        │
              ┌─────────┴──────────┐
              │   Safety Supervisor│  ← tüm cmd_vel geçer
              └─────────┬──────────┘
                        │
              ┌─────────┴──────────┐
              │   Session Logger   │  ← rosbag + event log
              └────────────────────┘
```

---

## Repo Yapısı

```
Earendil-Otonomius-For-ARC/
├── .github/
│   └── workflows/
│       └── ci.yml                    # CI: build + lint + test
├── docs/
│   ├── software_architecture.md      # Katman mimarisi, topic/service şeması
│   ├── roadmap.md                    # Fazlara bölünmüş geliştirme planı
│   └── testing_strategy.md          # Simulation-first test yaklaşımı
├── sim_config.yaml                   # Merkezi simülasyon konfigürasyonu
├── Dockerfile                        # ROS 2 Humble + Nav2 + SLAM geliştirme imajı
├── docker-compose.yml                # NVIDIA GPU + X11 ortamı
├── ros_entrypoint.sh
└── src/
    ├── earendil_msgs/                # Custom msg/srv/action tanımları
    ├── earendil_description/         # Robot URDF/xacro + mesh
    ├── earendil_bringup/             # Sistem bringup (sim + real ayrı)
    ├── earendil_simulation/          # Gazebo dünyaları + GPS spoofer
    ├── earendil_localization/        # Dual-UKF + SLAM + GPS monitor
    ├── earendil_navigation/          # Nav2 wrapper + waypoint manager
    ├── earendil_mission/             # State machine + task executor
    ├── earendil_perception/          # ArUco, rock, peak, tunnel task'ları
    ├── earendil_safety/              # Merkezi safety supervisor
    ├── earendil_teleop/              # Teleop + manual override
    ├── earendil_rscp/                # RSCP haberleşme köprüsü
    ├── earendil_logging/             # Rosbag + session log yöneticisi
    └── earendil_matlab_adapter/      # MATLAB northbound stub
```

---

## Paketler ve Görevleri

| Paket | Tür | Görev |
|-------|-----|-------|
| `earendil_msgs` | ament_cmake | Custom ROS mesaj/servis/action tanımları |
| `earendil_description` | ament_cmake | Robot URDF, mesh, RViz config |
| `earendil_bringup` | ament_cmake | Sim ve real rover için bringup launch'ları |
| `earendil_simulation` | ament_cmake | Gazebo dünyaları, GPS spoofer, sim araçları |
| `earendil_localization` | ament_python | Dual-UKF, GPS monitor, SLAM mod geçişi, TF relay |
| `earendil_navigation` | ament_python | Nav2 wrapper, waypoint manager, costmap config |
| `earendil_mission` | ament_python | State machine, stage orchestration, task executor |
| `earendil_perception` | ament_python | ArUco, rock detection, peak finder, tunnel traversal |
| `earendil_safety` | ament_python | Merkezi safety supervisor, e-stop, watchdog |
| `earendil_teleop` | ament_python | Teleop joystick/web, manual override switch |
| `earendil_rscp` | ament_python | RSCP komut köprüsü (uzak istasyon ↔ rover) |
| `earendil_logging` | ament_python | Rosbag session yönetimi, event logger |
| `earendil_matlab_adapter` | ament_python | MATLAB northbound adapter (stub) |

---

## Hızlı Başlangıç

### Gereksinimler
- Docker & Docker Compose
- NVIDIA GPU + CUDA sürücüsü
- X11 display (Gazebo GUI)

### Simülasyon Başlatma

```bash
# 1. Container'ı başlat
docker compose up -d --build

# 2. Workspace build
docker exec -it earendil-arc bash -c "cd ~/ws && colcon build --symlink-install"

# 3. Simülasyonu başlat (tüm stack)
docker exec -it earendil-arc bash -c "source ~/ws/install/setup.bash && \
  ros2 launch earendil_bringup sim_bringup.launch.py"
```

### Geliştirme Ortamı (Docker'sız)

```bash
# ROS 2 Humble kurulu sistemde
cd ~/ros2_ws && colcon build --symlink-install --packages-select earendil_msgs
source install/setup.bash
ros2 launch earendil_simulation sim_full.launch.py
```

---

## ARC Görev Mimarisi

```
Görev Başlangıcı (RSCP START komutu alınır)
         │
         ▼
┌────────────────┐
│  IDLE          │ ← Başlangıç durumu
└───────┬────────┘
        │ rscp_command = START
        ▼
┌────────────────┐     GPS iyi
│  LOCALIZING    │ ──────────────► GPS_NAV modu
└───────┬────────┘
        │ GPS yok / tünel
        ▼
┌────────────────┐
│  SLAM_NAV      │ ← SLAM Toolbox aktif
└───────┬────────┘
        │ hedef yakın
        ▼
┌────────────────┐
│  TASK_EXEC     │ ← ArUco / Rock / Peak / Tunnel task
└───────┬────────┘
        │ görev tamam
        ▼
┌────────────────┐
│  RETURNING     │
└───────┬────────┘
        │
        ▼
┌────────────────┐
│  COMPLETE      │
└────────────────┘

Her durumda Safety Supervisor devrede → E-STOP → EMERGENCY modu
```

---

## Konfigürasyon

`sim_config.yaml` ile tüm simülasyon parametreleri merkezi olarak yönetilir:
- Gazebo fizik motoru ayarları
- Robot hız/ivme limitleri
- GPS/SLAM mod geçiş eşikleri
- Tünel GPS spoof bölgeleri
- Timing parametreleri

---

## Geliştirme Rehberi

Detaylı mimari, roadmap ve test stratejisi için `docs/` klasörüne bakınız:
- [Yazılım Mimarisi](docs/software_architecture.md)
- [Geliştirme Roadmap'i](docs/roadmap.md)
- [Test Stratejisi](docs/testing_strategy.md)

---

## Branch Stratejisi

| Branch | Amaç |
|--------|------|
| `main` | Kararlı, çalışır sürüm |
| `develop` | Aktif geliştirme |
| `feature/<name>` | Yeni özellik |
| `fix/<name>` | Bug düzeltme |
| `sim/<name>` | Simülasyon denemeleri |

---

## Commit Convention

```
feat(mission): ARC stage-based task executor eklendi
fix(localization): GPS monitor hysteresis düzeltildi
docs(arch): yazılım mimarisi güncellendi
test(safety): e-stop watchdog test eklendi
chore(deps): nav2 bağımlılıkları güncellendi
```

---

## Lisans

Bu proje eğitim ve yarışma amaçlıdır.
