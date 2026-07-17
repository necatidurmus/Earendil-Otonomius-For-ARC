# CLAUDE.md — Earendil RPi5 Vehicle (Hızlı Mimari Özet & Kesin Kurallar)

> Ajan önce bu dosyayı, sonra `AGENT.md` (=`AGENTS.md`), `GOALS.md` ve `ROADMAP.md`'yi okur.
> Bu dosya özet kontrattır. Ayrıntı/gerekçe `AGENT.md`'de, hedefler `GOALS.md`'de, milestone planı `ROADMAP.md`'dedir.

## 1. Proje Özeti ve Ana Hedef
Bu proje sadece genel bir GPS waypoint rover projesi **değildir**. Ana hedef: **Anatolian Rover Challenge (ARC) 2026** otonom görevlerinde kullanılacak, **RSCP protokolüyle komut alan**, Raspberry Pi 5 + Ubuntu 24.04 + ROS 2 Jazzy (native) üzerinde çalışan **gerçek rover üst seviye kontrol sistemi**. Simülasyon değil; gerçek araçta güvenli otonom görev yürütme hedeflenir.

## 2. RSCP'nin Merkezi Rolü
RSCP (Rover Serial Communication Protocol) ARC'ın resmi protokolüdür. Pi'deki **`earendil_rscp_bridge`** seri üzerinden **Competition Module (CM)** ile konuşur; resmi repo + üretilmiş protobuf sınıfları kullanılır: GitHub `anatolianroverchallenge/rscp`, Python paketi `rscp_protobuf`, `cobs` paketi. Yerel kopya: `rscp protokol/rscp/`.
- **Alıcı:** seri → COBS decode (`0x00` sınırlayıcı) → `rscp.RequestEnvelope.ParseFromString()` → `request.WhichOneof('request')` ile dağıt.
- **Üretici:** rover durumu → `rscp.ResponseEnvelope` → `SerializeToString()` → COBS encode + `0x00` → seri.
- **Manuel serialize/deserialize YAZILMAZ.** Yalnız resmi `.proto` + üretilmiş sınıflar. `rscp protokol/rscp/proto/rscp.proto` referanstır.
- **RSCP bridge motor komutu ÜRETMEZ.** Sadece görev komutlarını mission manager'a çevirir.

Komutlar (RequestEnvelope oneof `request`): `ArmDisarm(arm)`, `SetStage(value)`, `NavigateToGPS(coordinate)`, `SearchArea(center_coordinate, radius)`, `StartExploration(dummy_field)`.
Yanıtlar (ResponseEnvelope oneof `response`): `Acknowledge`, `TaskFinished` *(README'de "TaskCompleted")*, `GPSCoordinate(lat,lon,alt — EGM96)`, `distance` (double), `message` (string), `RoverStatus(state, coordinate, heading, battery_state)` ≤1 Hz.
`RoverState` enum: `DISARMED=0, AUTONOMOUS=1, MANUAL=2` (H723 `RoverMode_t` ile sıra farkı → haritalama gerekli).

## 3. ARC 2026 Görev Akışı (4 stage)
- **Stage 1 Antenna Installation:** `SetStage(1)`→Ack→`ArmDisarm(true)`→Ack→`SearchArea`→Ack→ara→anteni kur→`GPSCoordinate`(tepe)→`TaskFinished`.
- **Stage 2 Shackleton Crater:** `SetStage(2)`→Ack→`SearchArea`→Ack→ara→`GPSCoordinate`(koyu ilmenit-bazalt)→`TaskFinished`.
- **Stage 3 Lava Tube:** `SetStage(3)`→Ack→`NavigateToGPS`→Ack→git→`TaskFinished`(giriş)→tag i→`StartExploration`→Ack→keşfet+ölç→`distance`→tag j→`TaskFinished`.
- **Stage 4 Return to Airlock:** `SetStage(4)`→Ack→`NavigateToGPS`→Ack→git→`TaskFinished`→tag k→dock→`ArmDisarm(false)`→Ack→görev bitti.

## 4. Referans Klasörlerin Rolü
- `rscp protokol/rscp/` — **resmi RSCP repo klonu** (proto, örnekler, release sınıfları). RSCP bridge referansı; **proto'yu/sınıfları kullan/üret, kodu kopyalama.**
- `stm codes/` — **gerçek firmware** (H723 `earendil-mainfirmware` + F411 `earendilmotorcontroller`). Low-level gerçeği; redesign kaynağı. **Kodu yeni yola kopyalama, referans oku.**
- `eski proje/` — **dondurulmuş** MATLAB/Gazebo/Leo eski demonturg. Sadece fikir referansı; **doğrudan kopyalama yok.**
- `src/`, `deploy/` — yeni ROS 2 iskeleti (rafine-in-place).

## 5. Donanım Mimarisi (katmanlar)
| Katman | Donanım | Rol | Motor yetkisi |
|---|---|---|---|
| L0 | 4× STM32F411 (FL/FR/RL/RR) | BLDC gate-drive, Hall→RPM, Speed PI | Fiziksel MOSFET sürer |
| L1 | STM32H723ZG (ara beyin) | 4 F411 komut dağıtımı; ana IMU (MPU9250)+mag (QMC5883P); DISARM/MANUAL/AUTO gate; link-loss watchdog | Dolaylı (F411'lere komut) |
| L2 | RPi5 (ROS 2 Jazzy) | RSCP bridge, mission manager, Nav2, safety_mux, localization, RTK GPS, LiDAR, stm_bridge, jetson_bridge, web, systemd | `/cmd_vel_safe`→stm_bridge (tek üretici) |
| L3 | Jetson Nano | Waveshare stereo: ArUco, derinlik/engel, opsiyonel VO; kamera 9-eksen IMU'su yalnız vision aux | **Yok** |

Zincir: `CM (RSCP) → Pi →(USART3 ASCII, ACK 500ms, LINK_LOSS 3000ms)→ H723 →(4×UART ASCII)→ F411×4 → BLDC`. **`stm_bridge` yalnızca H723 ile konuşur.**

## 6. Yazılım Mimarisi — 11 Paket
`earendil_interfaces` (msg/srv/action), `earendil_rscp_bridge` (RSCP serial+COBS+protobuf, **yeni**), `earendil_bringup`, `earendil_safety` (safety_mux, watchdog, e-stop, deadman, komut timeout), `earendil_control` (stm_bridge, cmd_vel→RPM, teker odom), `earendil_navigation` (Nav2, mission_manager, RTK onayı), `earendil_sensors` (RTK GPS, LiDAR, STM republish, TF, diagnostics), `earendil_jetson_bridge` (Jetson özet relay/validate), `earendil_web` (panel, deadman teleop, e-stop), `earendil_description` (URDF/xacro), `earendil_tools` (`earendil.py` karantina + yardımcılar).

## 7. Topic İsminlendirme
RSCP: `/rscp/request_raw`(opt), `/rscp/response_raw`(opt), `/rscp/current_stage`, `/rscp/command`, `/mission/command`, `/mission/status`, `/mission/result`.
Sensör: `/gps/fix`, `/rtk/status`, `/scan`, `/stm/imu/data`, `/stm/magnetic_field`, `/stm/wheel_odom`, `/stm/status`, `/stm/fault_flags`.
Vision: `/jetson/aruco_detections`, `/jetson/obstacles`, `/jetson/depth_summary`, `/jetson/visual_odom`, `/jetson/imu/data`.
Hareket: `/cmd_vel_nav` (Nav2), `/cmd_vel_manual` (web), `/cmd_vel_safe` (tek güvenli çıkış → stm_bridge). Frame REP-105: `base_link`, `imu_link`, `lidar_link`, `gps_link`, `odom→base_link`, `map→odom`.
**`/cmd_vel_safe`'e yazan yalnız `safety_mux`; okuyan yalnız `stm_bridge`.**

## 8. Safety Kuralları (değişmez)
- Hiçbir node doğrudan motor sürmez: `/cmd_vel_{nav,manual}` → safety_mux → `/cmd_vel_safe` → stm_bridge → H723 → F411.
- Öncelik: e-stop > deadman > watchdog > komut timeout (300–500 ms). Her gate deterministik sıfır.
- **RSCP bridge motor topic'ine YAZMAZ.** Görev→mission manager→goal→Nav2→`/cmd_vel_nav`.
- Jetson motor komutu **GÖNDERMEZ** (CI audit). Kamera-IMU ana IMU **değil**; STM IMU+mag ana.
- H723 boot'ta DISARM; F411 `CMD_WATCHDOG=800ms` + `HOST_LOST=2000ms` fiziksel failsafe katları. F411 akım sensörü YOK; TIM1 gate-drive kuralları `stm codes/.../AGENTS.md`'de geçerli.
- **GPS hiç yoksa otonom hareket başlatılmaz.** RTK onayı: FIXED→güvenilir varış; FLOAT/DGPS→40 cm tolerans; SPS/NO_FIX→**varış onayı verilmez**.

## 9. Araç Fiziksel Spec (korunur)
Skid-steer/tank-turn, BLDC hub motor. dingil mesafesi `wheel_base=0.825 m`, iz açıklığı `track_width=1.10 m` (merkez-merkez; dıştan-dışa ~0.95 m — açık soru), teker eni 0.15 m, teker çapı 0.25 m, teker yarıçapı `wheel_radius=0.125 m`, teker çevresi ~0.785 m, şasi 0.70×0.60 m. `hall_pulses_per_motor_rev=90` (`POLE_PAIRS=15×6`), `gear_ratio=1.0` (hub varsayım, doğrula). Kalibrasyon parametreleri (`effective_track_width`, `angular_correction_gain`, `left/right_motor_gain`) M11 saha kalibrasyonu; **hardcode edilmez**.

## 10. STM / Jetson / RPi Görev Ayrımı
- **STM:** gerçek-zamanlı motor + Hall + teker odom + ana IMU + mag + low-level watchdog + F411 firmware-side safety. **STM doğrudan görev kararı vermez**; düşük seviye kontrolcü + güvenlik katmanı. Mevcut GAP: CRC/seq/timestamp yok, odometry/pose yok, battery/status yok, timeout 3000ms≠300–500ms, cmd_vel değil RPM/duty+direction+turn-ratio → redesign (firmware sahipliği gerekir).
- **Jetson:** vision işleme; yalnız işlenmiş özet topic. Kamera-IMU/VO aux; primary localization değil. Ana karar bilgisayarı **değil**.
- **RPi:** RSCP bridge, otonomi stack, localization füzyon, mission, Nav2, web, deploy, kalibrasyon. Ana karar bilgisayarı budur.

## 11. Eski Projeden TAŞINABİLECEK / TAŞINMAYACAK
**Taşınabilir (fikir):** mission lifecycle (load/start/pause/resume/cancel/reset/retry/skip-on-stuck), CSV import/export, parametre preset + sweep, mission report (path error/RMSE, hız profili, jerk), GPS↔SLAM mode geçişi, RTK datum (`/fromLL`, EGM96), teleop web fikirleri, JSON status topic, waypoint güvenlik kontrolü, e-stop'ta `/cmd_vel_nav`'e sıfır basma (defense-in-depth).
**Taşınmaz (bağımlılık):** MATLAB + App Designer + ROS Toolbox; Gazebo/Ignition ve `leo_*`/`clearpath_*` SDF; Docker/Docker Compose/`docker exec` ana deploy; NVIDIA GPU+X11; `leo_robot`/`leo_simulator`/`leo_common`; `tunnel_gps_spoofer.py`/`/gps_spoofer/control` fake; `use_sim_time=True` default; Gazebo DiffDrive `/odom`.

## 12. Geliştirme Kuralları
- Native ROS 2 Jazzy, Ubuntu 24.04. Docker yalnız dev/test; **kontrol/deploy Docker'a BAĞLI DEĞİL.**
- Her davranış `*_params.yaml`'dan; hardcode yok. Tek-yazıcı: `/cmd_vel_safe` + H723 serial → tek node.
- Build: `colcon build --symlink-install`. Firmware: `pio run -d <module>` (PlatformIO yoksa **rapor et, uydurma**). RSCP sınıfları: resmi release `rscp_protobuf` + `cobs`; **manuel ser./deser. yazma**.
- Çalışan kod öncesi `ROADMAP.md` milestone'una bak.

## 13. Test ve Doğrulama
Temel: `colcon build` temiz; `ros2 interface list` yenileri gösterir; launch açılır-kapanır; udev kararlı isim. RSCP bridge: **fake/recorded RSCP module** ile COBS+protobuf framing ve `WhichOneof` dağıtım testi — **motor bağlı değil**. stm_bridge: loopback/fake-H723. M3: `/cmd_vel_safe` + serial tek-yazıcı audit. M11: ARC saha testleri.

## 14. YAPMAMA
- RSCP'yi opsiyonel/debug özellik gibi gösterme; **merkezi giriş noktasıdır.**
- Sistemi sadece waypoint navigasyon projesi gibi anlatma.
- Jetson'u/kamera-IMU'yu ana karar/ana IMU gibi gösterme; web teleop'u ana görev kontrolü gibi gösterme.
- Safety zincirini **bypass eden motor komut yolu tanımlama**; RSCP bridge doğrudan motor komutu üretme.
- Eski MATLAB/Gazebo/Docker'u yeni ana mimariye taşıma.
- Saha kalibrasyon parametresini hardcode etme; Docker'ı ana deploy yapma.
- TIM1 gate-drive hot-path kurallarını esnetme (`stm codes/.../AGENTS.md`); watchdog/safety'leri "test geçsin" diye devre dışı bırakma; sorunu gizleme.
- `earendil.py`'yi `tools/` karantinasından çıkarma.
- Kullanıcı bilgi/doküman istiyorsa: yalnız oku/raporla, değişiklik yapma.

## 15. Açık Sorular / TBD
- İz açıklığı merkez-merkez (1.10 m) mi dıştan-dışa (~0.95 m) mi? `effective_track_width` yüzeye-bağlı → M11.
- H723 firmware sahipliği + saha erişimi (timeout 3000→400–500ms, binary frame, odometry, battery/status redesign) — en büyük bilinmezen; RSCP `RoverStatus`/`battery_state` için battery telemetry gap'i kapatılmalı.
- `RoverState` (DISARMED/AUTONOMOUS/MANUAL) ↔ H723 `RoverMode_t` haritalaması (arm/disarm güvenlik kritik).
- hub `gear_ratio` (1.0 varsayım) ve `hall_pulses=90` doğrulanmalı; `max_motor_rpm`.
- RTK düzeltme kaynağı (NTRIP/hücresel); LiDAR modeli; Waveshare stereo modeli; ArUco sözlüğü (`DICT_ARUCO_ORIGINAL` örnek referans).

> Ayrıntılı ajan kuralları: `AGENT.md` (=`AGENTS.md`). Hedefler: `GOALS.md`. Milestone planı: `ROADMAP.md`.
