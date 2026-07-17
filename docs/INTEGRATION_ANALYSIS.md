# INTEGRATION_ANALYSIS.md — Earendil RPi5 Vehicle ARC 2026

> Tarih: 2026-07-17
> Kapsam: Ana proje (earendil_rpi5_vehicle) + Arkadaş projesi 1 (earendil/ - Arduino/BTS7960) + Arkadaş projesi 2 (Earendil-Otonomius-demo - Leo/Gazebo/Humble)

---

## 1. Kaynak Envanteri

### 1.1 Ana Proje — 11 Paket, ~40 Dosya

| Paket | Node/Dosya | Satır | Durum |
|---|---|---|---|
| earendil_interfaces | 13 msg, 3 srv, 2 action | — | Tam, çalışıyor |
| earendil_rscp_bridge | rscp_bridge_node.py | 499 | Tam, RSCP+COBS+protobuf |
| earendil_rscp_bridge | rscp_parser.py | ~250 | Tam, WhichOneof dispatch |
| earendil_control | stm_bridge.py | 448 | Tam, skid-steer kinematik |
| earendil_control | telemetry_parser.py | 244 | Tam, H723 ASCII parse |
| earendil_safety | safety_mux.py | 364 | Tam, 5-gate priority |
| earendil_safety | watchdog_node.py | ~100 | Var |
| earendil_navigation | mission_manager.py | 1059 | **Cok buyuk, refactor gerekli** |
| earendil_navigation | gps_utils.py | 61 | Tam, WGS84 to ENU |
| earendil_navigation | gps_monitor.py | 182 | Tam, RTK quality+mode switch |
| earendil_navigation | mission_report.py | 265 | Tam, ROS-bagimsiz pure fonksiyonlar |
| earendil_sensors | gps_adapter.py | 268 | Tam, NMEA GGA parse + RTK status |
| earendil_sensors | diagnostics_node.py | 218 | Tam, STM/GPS/LiDAR health |
| earendil_bringup | vehicle.launch.py | ~160 | Tam, staged delayed launch |
| earendil_bringup | nav2_params.yaml | — | Var |
| earendil_description | URDF/xacro | — | Var |
| earendil_web | web_server.py | ~? | Var |
| earendil_sim | sim.launch.py | — | Var |
| earendil_tools | earendil.py | — | Karantina |

### 1.2 Arkadas Projesi 1 (earendil/) — Arduino/BTS7960

| Dosya | Satir | Algoritma |
|---|---|---|
| motion_planner.py | 56 | 7-segment ArUco yonlendirme + angle-based servo step |
| aruco_detector.py | 59 | DICT_5X5_100, pixel area threshold, sequential tag queue |
| main_controller.py | 97 | search to scanning to orient state machine |
| serial_comm.py | — | Arduino MOTOR:FWD/LEFT/RIGHT string protocol |
| scanner_node.py | — | Servo tarama + angle callback |

**Entegrasyon degerlendirmesi:**
- ArUco sozlugu: DICT_5X5_100 (arkadas) vs DICT_ARUCO_ORIGINAL (ana varsayim) — sozluk dogrulama gerekli
- 7-segment yonlendirme: cx/frame_width ile sol/merkez/sag karari — basit ama etkili, Jetson tarafina adapte edilebilir
- Pixel area threshold: tag_width >= 220 cok yakin — mesafe bazli kalibrasyon gerekli
- Sequential tag queue (current_index): ARC stage bazli ArUco ID mapping ile birlestirilmeli
- Arduino protocol: MOTOR:FWD/LEFT/RIGHT — H723 ASCII RPM protokoluyle uyumsuz, dogrudan tasinamaz

### 1.3 Arkadas Projesi 2 (Earendil-Otonomius-demo) — Humble/Gazebo

| Dosya | Satir | Algoritma |
|---|---|---|
| mission_manager.py | 588 | YAML phase-based mission, GPS-SLAM mode switching, /fromLL service, TF offset calculation, costmap clearing |
| gps_monitor.py | — | GPS quality tracking |
| tf_mode_relay.py | — | map to map_slam TF switching |
| tunnel_gps_spoofer.py | — | GPS OFF during SLAM phases |
| navigation_hybrid.launch.py | — | Hybrid GPS+SLAM navigation |

**Entegrasyon degerlendirmesi:**
- Phase-based mission (GPS/SLAM mode switching): Ana projedeki mission_manager zaten GPS-SLAM mode gecisini implemente etti (_update_loc_mode)
- /fromLL servisi (robot_localization): Ana projede gps_utils.py var ama /fromLL servisi kullanilmiyor — kendi ENU donusumu var
- TF offset calculation (odom frame relative waypoints): Tunel kesfi icin faydali, Stage 3te adapte edilebilir
- Costmap clearing on phase transition: Faydali, ana projeye eklenebilir
- GPS spoofer: Ana projede GAZEBO_YOK, bu mekanizma gereksiz

---

## 2. Topic Envanteri

### 2.1 RSCP Topics
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| /rscp/current_stage | UInt32 | rscp_bridge | mission_manager |
| /rscp/command | RscpCommand | rscp_bridge | safety_mux, mission_manager |
| /rscp/status | RscpStatus | rscp_bridge | web, diagnostics |
| /mission/result | String | mission_manager | rscp_bridge |
| /mission/status | MissionStatus | mission_manager | web |
| /mission/control | MissionCommand | web, test | mission_manager |

### 2.2 Sensor Topics
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| /gps/fix | NavSatFix | gps_adapter | mission_manager, safety_mux, diagnostics, navsat_transform |
| /rtk/status | String | gps_adapter | mission_manager, gps_monitor, diagnostics |
| /scan | LaserScan | lidar_adapter | Nav2, diagnostics |
| /stm/imu/data | Imu | stm_bridge | navsat_transform, rscp_bridge |
| /stm/magnetic_field | MagneticField | stm_bridge | — |
| /stm/wheel_odom | Odometry | stm_bridge | ukf_local, mission_manager |
| /stm/status | StmStatus | stm_bridge | safety_mux, diagnostics |
| /stm/fault_flags | StmFaultFlags | stm_bridge | diagnostics |

### 2.3 Motion Topics
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| /cmd_vel_nav | Twist | mission_manager/Nav2 | safety_mux |
| /cmd_vel_manual | Twist | web_server | safety_mux |
| /cmd_vel_safe | Twist | **safety_mux ONLY** | **stm_bridge ONLY** |

### 2.4 Vision Topics (Jetson)
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| /jetson/aruco_detections | JetsonArucoDetections | jetson_bridge | mission_manager |
| /jetson/obstacles | — | jetson_bridge | — |
| /jetson/depth_summary | — | jetson_bridge | — |

### 2.5 Localization Topics
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| /odometry/local | Odometry | ukf_local | navsat_transform |
| /odometry/global | Odometry | ukf_global | Nav2 |
| /odometry/gps | Odometry | navsat_transform | ukf_global |
| /nav_mode | String | gps_monitor | tf_mode_relay |
| /gps_quality | Float32 | gps_monitor | — |
| /localization/mode | String | mission_manager | — |

---

## 3. Safety Zincir Analizi

### 3.1 Mevcut Zincir (DOGRU)
```
/cmd_vel_nav (Nav2/mission_manager)
/cmd_vel_manual (web_server)
    |
safety_mux (5-gate priority: e-stop > arm > deadman > stm_watchdog > cmd_timeout)
    |
/cmd_vel_safe (TEK guvenli cikis — yalniz safety_mux yazar)
    |
stm_bridge (TEK seri sahibi — yalniz bu node H723e yazar)
    |
H723 to F411x4 to BLDC motorlar
```

### 3.2 Audit Sonuclari
- /cmd_vel_safe'e yalniz safety_mux yaziyor ✓
- stm_bridge yalnizca H723 seri portunu aciyor ✓
- RSCP bridge motor komutu URETMEZ — sadece gorev komutu publish eder ✓
- Jetson motor komutu gondermez ✓
- E-stop latching modu (yalnizca servis ile reset) ✓
- Arm gate: arm_required=true disarida motor yok ✓
- BUG: safety_mux GPS subscriber'i Twist type ile subscribe ediyor (satir 131-132) — NavSatFix yerine Twist
- stm_bridge'de _send_serial simulation mode'da bile cagrilabilir (use_hardware=false iken _on_cmd_vel hala RPM hesaplar ama gondermez — dogru)
- H723 link_loss_timeout 3000ms (CLAUDE.md hedefi: 400-500ms) — firmware degisikligi gerekli

### 3.3 RoverState ve RoverMode_t Haritalamasi
```
RSCP RoverState:  DISARMED=0, AUTONOMOUS=1, MANUAL=2
H723 RoverMode_t: DISARM=0,   MANUAL=1,    AUTO=2

FARK: AUTO ve MANUAL sirasi ters!
```
- rscp_bridge satir 312: self._rover_state = 1 if cmd.arm_value else 0 — ARM to AUTONOMOUS(1), DISARM to DISARMED(0)
- stm_bridge satir 170-175: RSCP CMD_ARM to H723 'auto', CMD_DISARM to H723 'disarm'
- ACIK SORU: H723 'MANUAL' modu ne zaman kullanilir? RSCP MANUAL(2) state'i var ama bridge'de implemente yok.

---

## 4. Mission Manager Analizi (1059 satir — REFACTOR GEREKLI)

### 4.1 Mevcut Fonksiyonlar
1. RSCP Command Dispatch (satir 416-445): SetStage, ArmDisarm, NavigateToGPS, SearchArea, StartExploration
2. Stage State Machine (satir 87-108): IDLE, WAIT_STAGE, SEARCHING, FOUND, NAVIGATING, REACHED_ENTRY, EXPLORING, FOUND_EXIT, DOCKING, REACHED, DONE
3. Lifecycle (satir 111-116): RUNNING/PAUSED/CANCELLED — RSCPden bagimsiz operator kontrolu
4. Nav2 Integration (satir 519-616): GPS to ENU to PoseStamped to NavigateToPose goal
5. Search Pattern (satir 619-669): Spiral waypoint generation
6. Exploration (satir 674-699): GPS-based cumulative distance measurement
7. RTK Quality Check (satir 727-746): FIXED to 0.3m, FLOAT to 0.4m, NO_FIX to stop
8. Stuck Detection (satir 748-807): Progress window + retry + skip
9. GPS-SLAM Mode (satir 810-839): RTK degrade/recover timeout-based switching
10. Mission Report (satir 842-924): JSON report, CSV export, path metrics
11. Preset/CSV/Sweep (satir 926-999): Parameter preset loading, CSV waypoint import, parameter sweep

### 4.2 Refactor Onerisi
```
mission_manager.py (1059 satir) -> 4 dosya:
  mission_manager.py (~300 satir) — RSCP dispatch + lifecycle + stage state machine
  search_executor.py (~150 satir) — spiral/grid search pattern + waypoint generation
  exploration_executor.py (~100 satir) — tunnel exploration distance measurement
  mission_report.py (zaten ayri — 265 satir) ✓
```

---

## 5. Arkadas Projeleri Algoritma Transfer Tablosu

| Algoritma | Kaynak | Hedef | Uyumluluk | Aksiyon |
|---|---|---|---|---|
| 7-segment ArUco yonlendirme | earendil/motion_planner.py | jetson_bridge / mission_manager | Orta | Adapt: pixel to distance, DICT dogrula |
| ArUco tag queue (sequential) | earendil/aruco_detector.py | mission_manager Stage 1/2 | Yuksek | Stage-specific tag ID mapping |
| Phase-based mission (YAML) | Otonomius/mission_manager.py | mission_manager lifecycle | Dusuk | Ana zaten RSCP-driven, YAML opsiyonel |
| GPS-SLAM mode switching | Otonomius/mission_manager.py | gps_monitor + mission_manager | **Zaten var** | Ikisi de implemente |
| TF offset (odom-relative WP) | Otonomius/mission_manager.py | exploration_executor | Yuksek | Tunel kesfi icin faydali |
| Costmap clearing | Otonomius/mission_manager.py | mission_manager | Yuksek | Phase gecisinde ekle |
| /fromLL service | Otonomius/mission_manager.py | — | Dusuk | Ana gps_utils kendi ENU yapiyor |
| GPS spoofer | Otonomius/tunnel_gps_spoofer.py | — | Gereksiz | Gazebo-only mekanizma |
| Servo tarama | earendil/scanner_node.py | — | Gereksiz | H723/F411de servo yok |

---

## 6. Donanim Protokol Dogrulama

### 6.1 H723 ASCII Protokolu (stm_bridge to H723)
```
Gonderim: "FL rpm <signed>\r\n" x 4 motor
Mod komutu: "mode disarm\r\n" / "mode manual\r\n" / "mode auto\r\n"
Dur: "stop\r\n"

Alinan telemetri:
  Motor: FL|RPM:...,T:...,D:...,DIR:...,APP_PH:...,SP:...,BRAKE:...,FC:...,H:...,PWM_SET:...,PWM_ACT:...,QDROP:...,RXB:...\r\n
  IMU:   IMU|AX:...,AY:...,AZ:...,GX:...,GY:...,GZ:...\r\n
  MAG:   MAG|MX:...,MY:...,MZ:...\r\n
  Mode:  MODE:DISARM|MANUAL|AUTONOMOUS\r\n
  ACK:   ACK\r\n
  NACK:  NACK\r\n
```

### 6.2 Fiziksel Parametreler
| Parametre | Deger | Kaynak | Dogrulama |
|---|---|---|---|
| track_width | 1.10 m | CLAUDE.md, control_params.yaml | Merkez-merkez mi distan-dista mi? M11 |
| wheel_base | 0.825 m | CLAUDE.md, control_params.yaml | Dogrulandi |
| wheel_radius | 0.125 m | CLAUDE.md, control_params.yaml | Dogrulandi |
| wheel_circumference | 0.785 m | control_params.yaml | 2pi x 0.125 = 0.785 dogrulandi |
| hall_pulses_per_motor_rev | 90 | control_params.yaml | POLE_PAIRS=15x6=90 dogrulandi |
| gear_ratio | 1.0 | control_params.yaml | Hub motor varsayim — dogrula |
| max_motor_rpm | 300 | control_params.yaml | Dogrula saha testi |
| CMD_WATCHDOG | 800 ms | F411 firmware | Fiziksel failsafe |
| HOST_DISCONNECT | 2000 ms | F411 firmware | Fiziksel failsafe |
| ACK_TIMEOUT | 500 ms | control_params.yaml | Dogrulandi |
| LINK_LOSS | 3000 ms | control_params.yaml | Hedef: 400-500ms (firmware degisikligi) |

### 6.3 RoverState Haritalama Detayi
```
RSCP (proto): DISARMED=0, AUTONOMOUS=1, MANUAL=2
H723 firmware: DISARM=0, MANUAL=1, AUTO=2

stm_bridge._on_rscp_command():
  CMD_ARM (1)    -> "mode auto\r\n"    -> H723 AUTO(2)
  CMD_DISARM (2) -> "mode disarm\r\n"  -> H723 DISARM(0)

rscp_bridge._handle_arm_disarm():
  arm_value=True  -> _rover_state=1 (AUTONOMOUS)
  arm_value=False -> _rover_state=0 (DISARMED)

GAP: RSCP MANUAL(2) state hicbir yerde set edilmiyor.
     H723 "mode manual\r\n" komutu stm_bridge'de var ama tetiklenmiyor.
```

---

## 7. Eksik Bilesenler ve Acik Sorular

### 7.1 Kritik Eksikler (ARC gorev akisi icin zorunlu)
1. Search executor refactor — mission_manager.py cok buyuk, spiral search ayri module cikmali
2. Exploration distance — GPS-based cumulative distance (mevcut) + LiDAR-based tunnel depth estimation (yok)
3. Docking logic — Stage 4te airlocka docking icin ArUco-relative positioning (yok)
4. ArUco stage mapping — Her stage icin hangi tag IDleri aranacak? (configde yok)
5. Battery telemetry — H723den batarya verisi yok, RoverStatusda 0.0

### 7.2 Orta Seviye Eksikler
6. Costmap clearing — Phase gecisinde Nav2 costmap temizligi (arkadas projesinden transfer)
7. Waypoint heading — NavigateToPosea heading hesaplama (mevcut: orientation.w=1.0 duz bakis)
8. RTK datum management — gps_origin ilk fixte set ediliyor ama persist edilmiyor
9. LiDAR modeli — Vendor-specific driver entegrasyonu gerekli
10. Jetson bridge — ArUco/obstacle/depth topic relay implementasyonu

### 7.3 Dusuk Seviye Eksikler
11. Mission presets — Conservative/aggressive/test preset YAMLlar yok
12. Web dashboard — Deadman, e-stop, mission control UI
13. udev rules — Kararli seri port isimleri
14. systemd service — Otomatik baslatma
15. ArUco sozlugu — DICT_ARUCO_ORIGINAL mu DICT_5X5_100 mu? Yarismma dogrulamasi gerekli

---

## 8. Entegrasyon Plani (Asamali)

### Phase 1: Safety + Telemetry (GUNCEL — tamamlandi)
- safety_mux (5-gate priority) ✓
- stm_bridge (skid-steer kinematik, H723 ASCII) ✓
- telemetry_parser (H723 motor/IMU/mag/mode/ACK) ✓
- gps_adapter (NMEA GGA, RTK status) ✓
- rscp_bridge (COBS+protobuf, WhichOneof dispatch) ✓
- safety_mux GPS subscriber bug (Twist -> NavSatFix) ⚠️

### Phase 2: Navigation Core (devam ediyor)
- mission_manager refactor (4 dosya)
- Nav2 integration (mevcut — gps_to_local -> PoseStamped)
- Costmap clearing (transfer from Otonomius)
- Waypoint heading hesaplama

### Phase 3: Stage Executors
- Search executor (spiral + grid pattern)
- Exploration executor (GPS cumulative + LiDAR depth)
- Docking executor (ArUco-relative positioning)
- ArUco stage mapping config

### Phase 4: Vision + Jetson
- Jetson bridge (ArUco/obstacle/depth relay)
- ArUco detector adaptation (7-segment algorithm from friend)
- Stage-specific tag queue

### Phase 5: System Integration
- Launch files (vehicle.launch.py guncellemesi)
- Config YAMLlar (mission presets, ArUco mapping)
- Diagnostics (mevcut — diagnostics_node)
- Web dashboard (deadman, e-stop, mission control)
- udev rules + systemd service
- Test suite

---

## 9. Risk Matrisi

| Risk | Seviye | Etki | Azaltma |
|---|---|---|---|
| H723 timeout 3000ms (hedef 400-500ms) | Yuksek | Safety zinciri yavas | Firmware degisikligi gerekli |
| RoverState-RoverMode_t mapping hatasi | Kritik | Motor kontrolu ters | Unit test + integration test |
| Battery telemetry yok | Orta | RoverStatus eksik | H723 firmwarede ekle |
| RTK datum kaybi | Orta | Nav2 hedef sapmasi | Datum persist + recovery |
| ArUco sozlugu belirsiz | Orta | Tag tespit basarisiz | Yarismma sozlugu dogrulama |
| gear_ratio=1.0 dogrulanmamis | Dusuk | Hiz hesaplama hatasi | Saha testi |
| LiDAR driver yok | Dusuk | SLAM modu calismaz | Vendor-specific driver ekle |
