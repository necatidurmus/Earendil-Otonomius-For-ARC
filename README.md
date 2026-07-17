# Earendil RPi5 Vehicle

**ARC 2026 otonom görevleri için RSCP protokolüyle komut alan, Raspberry Pi 5 + ROS 2 Jazzy üzerinde çalışan gerçek rover üst seviye kontrol sistemi.**

> Bu proje genel bir GPS waypoint rover değildir. Ana hedef: [Anatolian Rover Challenge (ARC) 2026](https://www.anatolianrover.space/) yarışmasında kullanılacak, [RSCP (Rover Serial Communication Protocol)](https://github.com/anatolianroverchallenge/rscp) ile Competition Module'dan komut alan, gerçek donanıma entegre otonom rover kontrol sistemidir.

---

## Mimari

```
Competition Module (RSCP)          Jetson Nano (Stereo Vision)
         |                                  |
   [seri / COBS+protobuf]           [Ethernet / ROS 2 DDS]
         |                                  |
         v                                  v
   Raspberry Pi 5 — ROS 2 Jazzy (Ubuntu 24.04, native)
   ┌──────────────────────────────────────────────────┐
   │ earendil_rscp_bridge  ← RSCP görev komutları     │
   │          │                                       │
   │ mission_manager  → Nav2  → /cmd_vel_nav          │
   │                               │                  │
   │ earendil_web      → /cmd_vel_manual (deadman)    │
   │                               │                  │
   │ safety_mux  ←── e-stop, deadman, timeout         │
   │          │                                       │
   │      /cmd_vel_safe  (tek güvenli çıkış)          │
   │          │                                       │
   │ stm_bridge  → H723 (UART3 ASCII, ACK 500ms)     │
   └──────────────────────────────────────────────────┘
                    │
         [4× UART ASCII]
                    │
              STM32H723ZG (ara beyin)
                    │
         [4× UART ASCII]
                    │
           F411×4 BLDC motor sürücü
```

**Safety zinciri:** `/cmd_vel_nav` + `/cmd_vel_manual` → safety_mux → `/cmd_vel_safe` → stm_bridge → H723 → F411×4.
`/cmd_vel_safe`'e yalnız safety_mux yazar; stm_bridge yalnızca H723 ile konuşur. Hiçbir node bu zinciri bypass edemez.

**RSCP görev zinciri:** Competition Module → RSCP bridge → mission manager → Nav2 goal → `/cmd_vel_nav` → safety_mux → stm_bridge. RSCP bridge motor komutu üretmez.

---

## ARC 2026 Görev Akışı

4 stage'li otonom görev (RSCP protokolüyle komut alır):

| Stage | Görev | RSCP Komut Akışı |
|---|---|---|
| 1 — Antenna Installation | Arama alanında zirveyi bul, anteni kur | `SetStage(1)` → `ArmDisarm(true)` → `SearchArea` → `GPSCoordinate(zirve)` → `TaskFinished` |
| 2 — Shackleton Crater | Koyu ilmenit-bazalt kaya bul | `SetStage(2)` → `SearchArea` → `GPSCoordinate(kaya)` → `TaskFinished` |
| 3 — Lava Tube | Lav tüpüne gir, keşfet, mesafe ölç | `SetStage(3)` → `NavigateToGPS(giriş)` → `TaskFinished` → `StartExploration` → `distance` → `TaskFinished` |
| 4 — Return to Airlock | Airlock'a dön, dock et, disarm | `SetStage(4)` → `NavigateToGPS(airlock)` → `TaskFinished` → `ArmDisarm(false)` → `Acknowledge` |

---

## Paketler (11)

| Paket | Görev |
|---|---|
| `earendil_interfaces` | Özel msg/srv/action tanımları (StmTelemetry, MissionStatus, JetsonArucoDetections, vb.) |
| `earendil_rscp_bridge` | **RSCP seri + COBS + protobuf bridge** — CM ile iletişim, görev komutu dağıtımı, rover durumu yanıtı |
| `earendil_bringup` | Gerçek araç launch dosyaları |
| `earendil_safety` | safety_mux, watchdog, e-stop, deadman, komut timeout |
| `earendil_control` | stm_bridge, cmd_vel→RPM, teker odom |
| `earendil_navigation` | Nav2 entegrasyonu, mission manager, RTK-bilgili hedef onayı |
| `earendil_sensors` | RTK GPS, LiDAR, STM republish, TF, diagnostics |
| `earendil_jetson_bridge` | Jetson stereo özet topic'leri (ArUco, engel, derinlik) relay/validate |
| `earendil_web` | İzleme paneli, deadman teleop, e-stop |
| `earendil_description` | URDF/Xacro robot modeli, TF ağacı |
| `earendil_tools` | `earendil.py` (PC-side debug GUI, karantina) + yardımcılar |

---

## Donanım

| Katman | Donanım | Rol |
|---|---|---|
| L0 — Motor sürücü | 4× STM32F411 (FL/FR/RL/RR) | BLDC 6-step gate-drive, Hall→RPM, Speed PI |
| L1 — Ara beyin | STM32H723ZG | 4 F411 komut dağıtımı, ana IMU (MPU9250) + mag (QMC5883P), DISARM/MANUAL/AUTO gate |
| L2 — Ana bilgisayar | Raspberry Pi 5 + Ubuntu 24.04 + ROS 2 Jazzy | RSCP bridge, mission manager, Nav2, safety, localization, systemd |
| L3 — Vision | Jetson Nano + Waveshare stereo | ArUco tespiti, derinlik/engel, opsiyonel VO |

**Araç:** Skid-steer/tank-turn, BLDC hub motor. dingil mesafesi 0.825 m, iz açıklığı 1.10 m (merkez-merkez), teker yarıçapı 0.125 m, teker eni 0.15 m, şasi 0.70×0.60 m.

---

## Topic Yapısı

```
RSCP:     /rscp/current_stage, /rscp/command, /mission/command, /mission/status, /mission/result
Sensör:   /gps/fix, /rtk/status, /scan, /stm/imu/data, /stm/magnetic_field, /stm/wheel_odom, /stm/status, /stm/fault_flags
Vision:   /jetson/aruco_detections, /jetson/obstacles, /jetson/depth_summary, /jetson/visual_odom, /jetson/imu/data
Hareket:  /cmd_vel_nav (Nav2), /cmd_vel_manual (web), /cmd_vel_safe (tek güvenli çıkış)
```

Frame: REP-105 (`base_link`, `imu_link`, `lidar_link`, `gps_link`, `odom→base_link`, `map→odom`).

### Lokalizasyon Zinciri

İki aşamalı UKF füzyon:
1. **Local UKF** — `/stm/wheel_odom` + `/stm/imu/data` → `/odometry/local` (odom frame)
2. **NavSat Transform** — `/odometry/local` + `/gps/fix` → map frame koordinat dönüşümü
3. **Global UKF** — `/odometry/local` + `/odometry/gps` → `/odometry/global` (map frame)

Nav2 `/odometry/global` kullanır. GPS koordinatları `gps_utils.py` ile WGS84'den yerel ENU'ya dönüştürülür.

---

## Safety Kuralları

1. **Tek güvenli çıkış:** `/cmd_vel_safe` — yalnız safety_mux yazar, yalnız stm_bridge okur.
2. **Öncelik:** e-stop > deadman > watchdog > komut timeout (300–500 ms).
3. **RSCP bridge motor komutu üretmez.** Görev komutu → mission manager → Nav2 → `/cmd_vel_nav`.
4. **Jetson motor komutu göndermez.** Yalnız işlenmiş özet topic publish eder.
5. **GPS yoksa otonom hareket yok.** RTK onayı: FIXED → güvenilir; FLOAT/DGPS → 40 cm tolerans; SPS/NO_FIX → onay yok.
6. **H723 boot'ta DISARM.** F411 `CMD_WATCHDOG=800ms` + `HOST_LOST=2000ms` fiziksel failsafe.

---

## Hızlı Başlangıç (RPi5)

```bash
# 1. ROS 2 Jazzy + bağımlılıklar kur
bash deploy/install_rpi5_ubuntu24.sh

# 2. Derle
bash deploy/build.sh

# 3. Çalıştır
bash deploy/run_vehicle.sh

# Systemd servisi
sudo cp deploy/earendil-vehicle.service /etc/systemd/system/
sudo systemctl enable --now earendil-vehicle
```

RSCP bridge testi (fake module, motor bağlı değil):
```bash
# Fake RSCP module ile framing testi
python3 test/rscp_fake_module.py

# Unit testler (ROS gerektirmez)
python3 test/test_safety_mux.py
python3 test/test_stm_bridge.py
python3 test/test_gps_utils.py
python3 test/test_rscp_parser.py
```

---

## Geliştirme

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Tek paket derle
colcon build --packages-select earendil_rscp_bridge --symlink-install

# RSCP proto doğrula
python3 -c "import rscp_protobuf; print(rscp_protobuf.RequestEnvelope)"

# Arayüzleri listele
ros2 interface list | grep earendil
```

---

## Kaynaklar

- **RSCP protokolü:** https://github.com/anatolianroverchallenge/rscp
- **RSCP proto:** `rscp protokol/rscp/proto/rscp.proto`
- **RSCP Python paketi:** `pip3 install https://github.com/anatolianroverchallenge/rscp/releases/latest/download/rscp_protobuf.zip`
- **STM firmware:** `stm codes/earendil-mainfirmware/` (H723), `stm codes/earendilmotorcontroller/` (F411)
- **Milestone planı:** `ROADMAP.md`
- **Ajan kuralları:** `AGENT.md` (= `AGENTS.md`)
- **Hedefler:** `GOALS.md`
- **Anatolian Rover Challenge:** https://www.anatolianrover.space/

---

## Bilinen Eksikler / TBD

| Konu | Açıklama |
|---|---|
| H723 timeout | Mevcut 3000ms → 400-500ms (firmware değişikliği gerekli) |
| Battery telemetry | H723'den batarya verisi yok, RoverStatus'ta 0.0 gönderilir |
| H723 binary frame | Mevcut ASCII protokol → binary + CRC (firmware değişikliği) |
| RoverState mapping | RSCP (0=DISARMED,1=AUTO,2=MANUAL) vs H723 (0=DISARMED,1=MANUAL,2=AUTO) sıra farkı |
| gear_ratio | 1.0 varsayım (hub motor), doğrulanmalı |
| RTK düzeltme kaynağı | NTRIP/hücresel bağlantı henüz entegre değil |
| LiDAR modeli | Vendor-specific driver entegrasyonu gerekli |
| ArUco sözlüğü | DICT_ARUCO_ORIGINAL varsayım, yarışma sözlüğü doğrulanmalı |
