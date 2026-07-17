# ROADMAP.md — Earendil RPi5 Vehicle: ARC 2026 Entegrasyon Planı

> **Bu doküman uygulanabilir mühendislik planıdır.** Her milestone bağımsız çalışabilir,
> test edilebilir çıktılar üretir. Referans dokümanlar: `CLAUDE.md` (özet kontrat),
> `AGENT.md` (ajan kuralları), `GOALS.md` (hedefler), `rscp protokol/rscp/proto/rscp.proto`
> (RSCP doğruluk kaynağı).

---

## 1. Proje Amacı

Anatolian Rover Challenge (ARC) 2026 yarışmasında, 4 stage'li otonom görevi başarıyla
tamamlayan, RSCP protokolüyle Competition Module'dan komut alan, Raspberry Pi 5 + ROS 2 Jazzy
(native Ubuntu 24.04) üzerinde çalışan gerçek rover üst seviye kontrol sistemi geliştirmek.

Bu proje genel bir GPS waypoint rover **değildir**. RSCP merkezlidir. Simülasyon değil;
gerçek donanımda güvenli otonom görev yürütme hedeflenir.

---

## 2. ARC ve RSCP Bağlamı

### 2.1 ARC 2026

Anatolian Rover Challenge — üniversite öğrencileri arası otonom rover yarışması.
Rover, Competition Module (CM) ile seri hat üzerinden RSCP protokolüyle konuşur.
4 stage sırasıyla çalıştırılır; her stage belirli komut akışlarıyla ilerler.

### 2.2 RSCP Protokolü

RSCP = Rover Serial Communication Protocol. Protokol tanımı `rscp protokol/rscp/proto/rscp.proto`
dosyasında tanımlıdır. **Bu dosya tek doğruluk kaynağıdır; README'deki `TaskCompleted` ifadeleri
yanlıştır, gerçek mesaj `TaskFinished`'dir.**

**Fiziksel katman:** Seri port, COBS framing (`0x00` sınırlayıcı).

**Request mesajları** (`RequestEnvelope.oneof request`):
| Mesaj | Alan | Açıklama |
|---|---|---|
| `ArmDisarm` | `oneof value_wrapper { bool value }` | `true`=arm, `false`=disarm |
| `SetStage` | `uint32 value` (1–4) | Görev aşaması |
| `NavigateToGPS` | `GPSCoordinate coordinate` | GPS hedef navigasyonu |
| `SearchArea` | `GPSCoordinate center_coordinate` + `float radius` | Alan arama |
| `StartExploration` | `bool dummy_field` | Keşif başlat (payload yok) |

**Response mesajları** (`ResponseEnvelope.oneof response`):
| Mesaj | Alan | Açıklama |
|---|---|---|
| `Acknowledge` | (boş) | Komut alındı |
| `TaskFinished` | (boş) | Görev/alt-görev tamamlandı |
| `GPSCoordinate` | `double lat`, `double lon` (derece), `float alt` (m, EGM96) | Bulunan konum |
| `distance` | `double` | Ölçülen mesafe (metre) |
| `message` | `string` | Hata/durum mesajı |
| `RoverStatus` | `RoverState state`, `GPSCoordinate coordinate`, `float heading`, `BatteryState battery_state` | Durum raporu (≤1 Hz) |

**RoverState enum:** `DISARMED=0, AUTONOMOUS=1, MANUAL=2` — H723 `RoverMode_t` sırasıyla farklıdır, eşleme gerekir.

**BatteryState:** `float voltage`, `float current`, `float state_of_charge` (0..1).

---

## 3. Sistem Mimarisi

```
Competition Module (RSCP)         Jetson Nano (Stereo Vision)
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

**RSCP görev zinciri:** CM → RSCP bridge → mission manager → Nav2 goal → `/cmd_vel_nav` → safety_mux → `/cmd_vel_safe` → stm_bridge → H723 → F411. RSCP bridge motor komutu **üretmez**.

**Safety zinciri:** `/cmd_vel_nav` + `/cmd_vel_manual` → safety_mux → `/cmd_vel_safe` → stm_bridge → H723 → F411. Bu zinciri bypass eden hiçbir yol yoktur.

---

## 4. Donanım Katmanları

| Katman | Donanım | Rol | Motor yetkisi |
|---|---|---|---|
| L0 | 4× STM32F411 (FL/FR/RL/RR) | BLDC 6-step gate-drive, Hall→RPM, Speed PI, CMD_WATCHDOG=800ms, HOST_LOST=2000ms | Fiziksel MOSFET sürer |
| L1 | STM32H723ZG | 4 F411 komut dağıtımı, ana IMU (MPU9250)+mag (QMC5883P), DISARM/MANUAL/AUTO gate, link-loss watchdog (3000ms) | Dolaylı (F411'lere komut) |
| L2 | RPi5 + Ubuntu 24.04 + ROS 2 Jazzy | RSCP bridge, mission manager, Nav2, safety_mux, localization, RTK GPS, LiDAR, stm_bridge, jetson_bridge, web, systemd | `/cmd_vel_safe` → stm_bridge |
| L3 | Jetson Nano | Waveshare stereo: ArUco, derinlik/engel, opsiyonel VO; kamera IMU'su yalnız vision aux | **Yok** |

**H723 ↔ Pi protokolü:** USART3 ASCII, `<cmd>\r\n`, ACK 500ms/3 retry, LINK_LOSS 3000ms.
**H723 ↔ F411 protokolü:** 4×UART ASCII, `rpm <signed>`, komut + telemetri.

---

## 5. Yazılım Paketleri (11)

| Paket | Sorumluluk | Durum |
|---|---|---|
| `earendil_interfaces` | msg/srv/action tanımları | Kısmen mevcut (RSCP msg eksik) |
| `earendil_rscp_bridge` | RSCP seri + COBS + protobuf bridge | **Yeni — yok** |
| `earendil_bringup` | Gerçek araç launch dosyaları | Mevcut (güncellenecek) |
| `earendil_safety` | safety_mux, watchdog, e-stop, deadman | Mevcut (RSCP arm gate eklenecek) |
| `earendil_control` | stm_bridge, cmd_vel→RPM, teker odom | Mevcut (H723 protokolüne adapte) |
| `earendil_navigation` | Nav2, mission manager, RTK onayı | Mevcut (RSCP stage-aware yeniden yazılacak) |
| `earendil_sensors` | RTK GPS, LiDAR, STM republish, TF | Mevcut (STM topic'leri eklenecek) |
| `earendil_jetson_bridge` | Jetson özet topic'leri relay/validate | **Yeni — yok** |
| `earendil_web` | Panel, deadman teleop, e-stop | Mevcut (genişletilecek) |
| `earendil_description` | URDF/Xacro, TF ağacı | Mevcut (boş) |
| `earendil_tools` | `earendil.py` karantina + yardımcılar | Mevcut (karantina) |

---

## 6. RSCP Protokol Entegrasyonu

### 6.1 Fiziksel Katman

```
Seri port (USB-UART adaptör) → /dev/earendil_rscp (udev kuralı)
Baud: protokol dokümanına göre (varsayılan 115200)
Frame: COBS kodlanmış, 0x00 sınırlayıcı
```

### 6.2 Alma Akışı

```
byte-byte seri okuma
  → 0x00 gelince buffer'ı al
  → cobs.cobs.decode(buffer)
  → rscp_protobuf.RequestEnvelope().ParseFromString(decoded)
  → request.WhichOneof('request')
  → arm_disarm / set_stage / navigate_to_gps / search_area / start_exploration
  → ilgili internal topic'e publish
  → Acknowledge gönder
```

### 6.3 Gönderme Akışı

```
/mission/status, /mission/result, /gps/fix, /rtk/status, /stm/imu/data dinle
  → ResponseEnvelope oluştur (task_finished / gps_coordinate / distance / rover_status)
  → SerializeToString()
  → cobs.cobs.encode(bytes)
  → + b"\x00"
  → seri port.write()
```

### 6.4 Komut Dağıtım Tablosu

| RSCP komutu | Bridge aksiyonu | Internal topic | Sonraki RSCP yanıtı |
|---|---|---|---|
| `set_stage(v)` | Stage ayarla | `/rscp/current_stage` + `/rscp/command` | `Acknowledge` |
| `arm_disarm(true)` | Arm sinyali | `/rscp/command` | `Acknowledge` |
| `arm_disarm(false)` | Disarm sinyali | `/rscp/command` | `Acknowledge` |
| `navigate_to_gps(coord)` | Nav2 hedefi | `/mission/command` | `Acknowledge` → varışta `TaskFinished` |
| `search_area(center,r)` | Arama görevi | `/mission/command` | `Acknowledge` → bulunca `GPSCoordinate` → `TaskFinished` |
| `start_exploration` | Keşif modu | `/mission/command` | `Acknowledge` → ölçümde `distance` → çıkışta `TaskFinished` |

### 6.5 Yanıt Üretim Tablosu

| Yanıt | Kaynak topic | Ne zaman gönderilir |
|---|---|---|
| `Acknowledge` | (komut alındı) | Her `RequestEnvelope` sonrası |
| `TaskFinished` | `/mission/result` | Görev/alt-görev tamamlandığında |
| `GPSCoordinate` | `/gps/fix` (RTK onaylı) | Hedef konum bulunduğunda (antenna tepe, bazalt kaya) |
| `distance` | `/stm/wheel_odom` integrasyonu | StartExploration boyunca kat edilen mesafe |
| `message` | hata/durum | Opsiyonel hata mesajı |
| `RoverStatus` | `/rtk/status` + `/gps/fix` + `/stm/imu/data` + battery | Periyodik ≤1 Hz |

---

## 7. ARC Stage State Machine

### Stage 1 — Antenna Installation

| State | Gelen RSCP komutu | Rover aksiyonu | Sensörler | Gönderilen RSCP cevabı | Başarı şartı | Hata/timeout |
|---|---|---|---|---|---|---|
| WAIT_STAGE | `SetStage(1)` | Stage'ı 1 yap | — | `Acknowledge` | Stage değişti | Bilinmeyen stage → `message` |
| WAIT_ARM | `ArmDisarm(true)` | H723 AUTONOMOUS moduna geç | STM status | `Acknowledge` | Arm başarılı | Arm reddedildi → `message` |
| WAIT_SEARCH | `SearchArea(center,r)` | Arama alanına git, spiral/grid ara | RTK GPS, LiDAR, STM odom | `Acknowledge` | Komut alındı | Geçersiz koordinat → `message` |
| SEARCHING | (otonom) | Spiral/grid arama, zirveyi bul | RTK GPS, LiDAR, IMU, (Jetson opsiyonel) | — | Zirve tespit edildi | Timeout → `TaskFinished` (hata) |
| FOUND | — | Zirve konumunu raporla | RTK GPS | `GPSCoordinate(tepe)` | Koordinat gönderildi | GPS yok → retry |
| DONE | — | Görev bitti | — | `TaskFinished` | CM'ye iletildi | — |

### Stage 2 — Shackleton Crater

| State | Gelen RSCP komutu | Rover aksiyonu | Sensörler | Gönderilen RSCP cevabı | Başarı şartı | Hata/timeout |
|---|---|---|---|---|---|---|
| WAIT_STAGE | `SetStage(2)` | Stage'ı 2 yap | — | `Acknowledge` | Stage değişti | — |
| WAIT_SEARCH | `SearchArea(center,r)` | Arama alanına git | RTK GPS, LiDAR | `Acknowledge` | Komut alındı | — |
| SEARCHING | (otonom) | Koyu ilmenit-bazalt kaya ara | RTK GPS, LiDAR, (Jetson) | — | Kaya tespit edildi | Timeout → `TaskFinished` (hata) |
| FOUND | — | Kaya konumunu raporla | RTK GPS | `GPSCoordinate(kaya)` | Koordinat gönderildi | — |
| DONE | — | Görev bitti | — | `TaskFinished` | — | — |

### Stage 3 — Lava Tube

| State | Gelen RSCP komutu | Rover aksiyonu | Sensörler | Gönderilen RSCP cevabı | Başarı şartı | Hata/timeout |
|---|---|---|---|---|---|---|
| WAIT_STAGE | `SetStage(3)` | Stage'ı 3 yap | — | `Acknowledge` | Stage değişti | — |
| WAIT_NAV | `NavigateToGPS(coord)` | Git | RTK GPS, LiDAR, Nav2 | `Acknowledge` | Komut alındı | — |
| NAVIGATING | (otonom) | Git, ArUco tag i ara | RTK, LiDAR, Jetson ArUco | — | Giriş tag'i tespit + varış | Timeout → `TaskFinished` (hata) |
| REACHED_ENTRY | — | Giriş ulaşıldı | Jetson ArUco | `TaskFinished` | — | — |
| WAIT_EXPLORE | `StartExploration` | Keşif modu başlat | — | `Acknowledge` | Komut alındı | — |
| EXPLORING | (otonom) | Tüpü keşfet, mesafe ölç, tag j ara | STM odom, LiDAR, Jetson ArUco | — | Çıkış tag'i tespit | Timeout → mesafe gönder + `TaskFinished` |
| FOUND_EXIT | — | Mesafe raporla | STM wheel odom | `distance(m)` | Mesafe gönderildi | — |
| DONE | — | Görev bitti | — | `TaskFinished` | — | — |

### Stage 4 — Return to Airlock

| State | Gelen RSCP komutu | Rover aksiyonu | Sensörler | Gönderilen RSCP cevabı | Başarı şartı | Hata/timeout |
|---|---|---|---|---|---|---|
| WAIT_STAGE | `SetStage(4)` | Stage'ı 4 yap | — | `Acknowledge` | Stage değişti | — |
| WAIT_NAV | `NavigateToGPS(coord)` | Git | RTK GPS, LiDAR, Nav2 | `Acknowledge` | Komut alındı | — |
| NAVIGATING | (otonom) | Airlock'a git, tag k ara | RTK, LiDAR, Jetson ArUco | — | Airlock varış + tag k | Timeout → `TaskFinished` (hata) |
| DOCKING | — | Dock yaklaşımı | Jetson ArUco, LiDAR | — | Dock başarılı | — |
| REACHED | — | Airlock ulaşıldı | — | `TaskFinished` | — | — |
| WAIT_DISARM | `ArmDisarm(false)` | H723 DISARM, motor dur | STM status | `Acknowledge` | Disarm başarılı | Disarm reddedildi → `message` |
| DONE | — | Görev tamamlandı | — | — | — | — |

---

## 8. Milestone Genel Tablosu

| # | Milestone | Efor | Kritik yol | Bloklananlar |
|---|---|---|---|---|
| M1 | Mimari Dokümantasyon ve Paket Kontratı | S | Evet | Tümü |
| M2 | RSCP Bridge (COBS + Protobuf + Request/Response) | L | Evet | M7, M11 |
| M3 | STM Bridge ve Güvenli Motor Zinciri | M | Evet | M4, M6, M7 |
| M4 | Safety Mux ve Güvenli Komut Kapısı | M | Evet | M7, M9, M11 |
| M5 | Sensör Bringup | M | — | M6, M7, M8 |
| M6 | Localization (RTK + STM odom + STM IMU) | M | Evet | M7 |
| M7 | Nav2, Mission Manager ve Görev Akışı | L | Evet | M9, M10, M11 |
| M8 | Jetson Stereo Vision ve ArUco | L | Hayır | — |
| M9 | Web Panel | M | Hayır | M11 |
| M10 | Deploy ve Systemd | S–M | — | M11 |
| M11 | ARC Saha Testleri | L | Evet (final) | — |

**Kritik yol:** M1 → M2 → (M3 ∥ M4) → M5 → M6 → M7 → M11.
M8/M9 paralel çalışabilir. M10 M7 sonrasında. M11 her şeyin üstünde.

---

## 9. Detaylı Milestone Planları

---

### M1 — Mimari Dokümantasyon ve Paket Kontratı

**Amaç:** Mimari kontratı dondur. RSCP'yi merkeze alan dokümantasyonu hazırla; 11-paket iskeletini aç; interface/msg tanımlarını yap; config dosyalarını oluştur; `colcon build` temiz çalışsın.

**Ön koşullar:** Yok — bu ilk milestone.

**Yapılacak işler:**

1. **Dokümantasyon güncelle:**
   - `CLAUDE.md` — özet kontrat (mevcut, küçük düzeltmeler)
   - `AGENT.md` / `AGENTS.md` — ajan kuralları (mevcut, RSCP-aware güncelle)
   - `GOALS.md` — proje hedefleri (mevcut)
   - `README.md` — proje tanıtımı (mevcut)
   - `ROADMAP.md` — bu dosya (detaylı yeniden yazım)
   - `topic_design.md` — tüm topic'lerin, msg'lerin, QoS ayarlarının dokümantasyonu
   - `stm_protocol.md` — H723 ASCII protokolü, telemetri formatı, komut seti
   - `rscp_bridge.md` — RSCP bridge tasarım dokümanı

2. **`earendil_interfaces` paketi — eksik msg'leri ekle:**
   - `RscpCommand.msg` — RSCP'den gelen komutun internal temsili
   - `RscpStatus.msg` — RSCP bridge durumu
   - `MissionStage.msg` — mevcut stage bilgisi (stage numarası + isim)
   - `JetsonArucoDetections.msg` — ArUco tag tespit sonuçları
   - `StmStatus.msg` — STM durum bilgisi (mode, uptime, link quality)
   - `StmFaultFlags.msg` — F411 14-fault flag seti
   - Mevcut msg'leri koru: `VehicleState`, `SafetyStatus`, `HealthDiag`, `MissionStatus`, `Waypoint`

3. **Config dosyaları oluştur/güncelle:**
   - `vehicle.yaml` — fiziksel spec:
     ```yaml
     wheel_base_m: 0.825
     track_width_m: 1.10          # TBD – merkez-merkez mi dıştan-dışa mı?
     wheel_radius_m: 0.125
     wheel_circumference_m: 0.785  # ~2π×0.125
     effective_track_width_m: 0.0  # TBD – M11 saha kalibrasyonu
     angular_correction_gain: 0.0  # TBD
     left_motor_gain: 1.0          # TBD
     right_motor_gain: 1.0         # TBD
     hall_pulses_per_motor_rev: 90 # POLE_PAIRS=15×6
     gear_ratio: 1.0               # TBD – hub varsayım
     motor_pole_pairs: 15
     max_motor_rpm: 0              # TBD
     ```
   - `safety_params.yaml` — **mevcut HATALI** (topic adları yanlış, değerler eski):
     ```yaml
     # Mevcut hatalı: input=/cmd_vel_pre_safety, output=/cmd_vel, wheel_base=0.38
     # Doğrusu:
     safety_mux:
       ros__parameters:
         input_topics: ["/cmd_vel_nav", "/cmd_vel_manual"]
         output_topic: "/cmd_vel_safe"
         estop_topic: "/e_stop"
         deadman_topic: "/deadman"
         watchdog_timeout_ms: 400
         command_timeout_ms: 300
         max_linear_speed: 0.5
         max_angular_speed: 1.0
     ```
   - `control_params.yaml` — **mevcut HATALI** (wheel_base=0.38, wheel_radius=0.0625):
     ```yaml
     # Doğrusu:
     stm_bridge:
       ros__parameters:
         cmd_vel_topic: "/cmd_vel_safe"
         serial_port: "/dev/earendil_h7"
         serial_baud: 115200
         ack_timeout_ms: 500
         max_retries: 3
         link_loss_timeout_ms: 3000
         wheel_base: 0.825
         wheel_radius: 0.125
         wheel_circumference: 0.785
         hall_pulses_per_motor_rev: 90
         gear_ratio: 1.0
         use_hardware: false
     ```
   - `sensor_params.yaml` — **mevcut HATALI** (wheel_base=0.38, wheel_radius=0.0625):
     ```yaml
     # Doğrusu: wheel_base: 0.825, wheel_radius: 0.125
     ```
   - `nav2_params.yaml` — Nav2 yapılandırması (M7'de detaylandırılacak)
   - `mission_params.yaml` — mission manager parametreleri

4. **Launch taslakları:**
   - `minimal.launch.py` — en az paketle çalıştırma
   - `vehicle.launch.py` — tam araç launch

5. **RSCP proto doğrulama:**
   - `rscp_protobuf` pip paketi import edilebilir mi?
   - `RequestEnvelope` ve `ResponseEnvelope` sınıfları mevcut mu?
   - `WhichOneof('request')` doğru çalışıyor mu?

6. **İskelet build:**
   - `colcon build --symlink-install` temiz
   - `ros2 interface list` yeni msg/srv/action'ları gösteriyor

**ROS node'ları:** Yok — sadece iskelet ve dokümantasyon.

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_interfaces/msg/RscpCommand.msg` | Yeni |
| `src/earendil_interfaces/msg/RscpStatus.msg` | Yeni |
| `src/earendil_interfaces/msg/MissionStage.msg` | Yeni |
| `src/earendil_interfaces/msg/JetsonArucoDetections.msg` | Yeni |
| `src/earendil_interfaces/msg/StmStatus.msg` | Yeni |
| `src/earendil_interfaces/msg/StmFaultFlags.msg` | Yeni |
| `src/earendil_control/config/control_params.yaml` | Düzelt (wheel_base, wheel_radius) |
| `src/earendil_sensors/config/sensor_params.yaml` | Düzelt (wheel_base, wheel_radius) |
| `src/earendil_safety/config/safety_params.yaml` | Düzelt (topic adları) |
| `docs/topic_design.md` | Yeni |
| `docs/stm_protocol.md` | Yeni |
| `docs/rscp_bridge.md` | Yeni |

**Test planı:**
- `colcon build --symlink-install` → temiz derleme
- `ros2 interface list | grep earendil` → yeni arayüzler görünüyor
- `python3 -c "import rscp_protobuf; print(rscp_protobuf.RequestEnvelope)"` → import başarılı
- Launch dosyaları `ros2 launch` ile açılıp kapanıyor (node yok, sadece yapılandırma)

**Kabul kriterleri:**
- [ ] `colcon build` temiz
- [ ] `ros2 interface list` tüm yeni msg/srv/action'ları gösteriyor
- [ ] `rscp_protobuf` ve `cobs` import edilebilir
- [ ] `vehicle.yaml` TBD notlu
- [ ] Hatalı config dosyaları düzeltildi (wheel_base, topic adları)
- [ ] Dokümantasyon tamam
- [ ] `earendil.py` hala `tools/` karantinasında
- [ ] `eski proje/` kopyalanmadı

**Bağımlılıklar:** Yok.

**Riskler:** Yok — dokümantasyon ve iskelet.

**Definition of Done:** Mimari kontrat donmuş, 11-paket iskeleti derleniyor, config dosyaları doğru değerlerle hazır.

---

### M2 — RSCP Bridge (COBS + Protobuf + Request/Response)

**Amaç:** `earendil_rscp_bridge` paketini oluştur. Seri bağlantı, COBS framing, `RequestEnvelope` parse + `WhichOneof('request')` dağıtım, `ResponseEnvelope` üretimi. RSCP bridge motor komutu üretmez.

**Ön koşullar:** M1 tamamlanmış olmalı (paket iskeleti, interfaces, `rscp_protobuf` import).

**Yapılacak işler:**

1. **Paket yapısı:**
   - `src/earendil_rscp_bridge/` — ROS 2 Python paketi
   - `earendil_rscp_bridge/rscp_bridge_node.py` — ana node
   - `earendil_rscp_bridge/rscp_parser.py` — COBS + protobuf parse/produce
   - `config/rscp_bridge_params.yaml` — seri port, baud, timeout
   - `launch/rscp_bridge.launch.py`
   - `docs/rscp_bridge.md`

2. **Seri port okuma:**
   - `pyserial` ile seri port aç (`/dev/earendil_rscp`, baud parametre)
   - Byte-byte oku, `0x00` sınırlayıcı algıla
   - Partial frame buffer yönetimi (timeout ile frame temizleme)
   - Seri port kapanma/yeniden bağlanma recovery

3. **COBS decode:**
   - `0x00` gelince buffer'ı al
   - `cobs.cobs.decode(buffer)` → decoded bytes
   - Decode hatası → frame at + log warning + devam (crash yok)

4. **Protobuf parse:**
   - `rscp_protobuf.RequestEnvelope().ParseFromString(decoded)`
   - Parse hatası → frame at + log warning + devam (crash yok)

5. **`WhichOneof('request')` ile dağıtım:**
   - `arm_disarm` → `/rscp/command` topic'e publish + arm gate sinyali
   - `set_stage` → `/rscp/current_stage` publish + `/rscp/command` publish
   - `navigate_to_gps` → `/mission/command` publish (tip: NAVIGATE_TO_GPS)
   - `search_area` → `/mission/command` publish (tip: SEARCH_AREA)
   - `start_exploration` → `/mission/command` publish (tip: START_EXPLORATION)
   - `None` / bilinmeyen tür → log warning + `Acknowledge` + opsiyonel `message` hatası; crash yok

6. **ArmDisarm güvenlik eşlemesi:**
   - `arm_disarm.value = true` → RSCP arm sinyali publish (`/rscp/command` tip: ARM)
   - `arm_disarm.value = false` → RSCP disarm sinyali publish (`/rscp/command` tip: DISARM)
   - `value_wrapper` oneof kontrolü: wrapper yoksa hata log
   - Arm/disarm sinyali safety_mux'a gider; H723 operating mode stm_bridge tarafından değiştirilir

7. **Acknowledge gönderimi:**
   - Her `RequestEnvelope` sonrası `Acknowledge` `ResponseEnvelope` oluştur
   - `SerializeToString()` → `cobs.cobs.encode()` → `+ b"\x00"` → seri.write()
   - Error durumunda `message` field'ı kullan (hata açıklaması)

8. **TaskFinished gönderimi:**
   - `/mission/result` topic'inden `task_finished` sinyali dinle
   - Sinyal geldiğinde `TaskFinished` `ResponseEnvelope` oluştur → COBS encode → seri

9. **GPSCoordinate gönderimi:**
   - Hedef konum bulunduğunda (mission manager bildirir) → `/gps/fix`'ten koordinat al
   - `GPSCoordinate` `ResponseEnvelope` oluştur (lat, lon, alt — EGM96)
   - RTK quality kontrolü: FIXED/FLOAT/DGPS → gönder; SPS/NO_FIX → gönderme + hata log

10. **distance gönderimi:**
    - StartExploration alt-görevi boyunca `/stm/wheel_odom` integrasyonu
    - Her update'de kümülatif mesafe hesapla
    - Keşif bittiğinde `distance` `ResponseEnvelope` oluştur → gönder

11. **RoverStatus periyodik gönderimi (≤1 Hz):**
    - 1 saniyede bir timer
    - Kaynak: `/rtk/status` + `/gps/fix` + `/stm/imu/data` (heading) + battery
    - `RoverState` eşleme: RSCP `RoverState` → internal state mapping
    - `BatteryState`: H723'den gelmiyor (GAP) → M2 için `battery_state.voltage=0, state_of_charge=NaN` + hata log
    - M3'te H723 battery/status eklendikten sonra kapatılacak

12. **Bilinmeyen request davranışı:**
    - `WhichOneof('request')` None dönerse → log error + `Acknowledge` + `message` ("unknown request type")
    - Crash yok, node yaşamaya devam eder

13. **Decode/parse error recovery:**
    - COBS decode exception → frame at + log + devam
    - Protobuf parse exception → frame at + log + devam
    - Seri port hatası → reconnect döngüsü + log
    - Hiçbir hata durumu node'un çökmesine neden olmaz

14. **Test planı:**
    - Fake RSCP module: `io.BytesIO` ile kayıtlı frame'ler
    - Her komut tipi için test: `set_stage(1)`, `arm_disarm(true)`, `navigate_to_gps(coord)`, `search_area(center,10)`, `start_exploration`
    - Bilinmeyen komut testi
    - COBS framing hatası testi (bozuk frame)
    - Serial kopma/yeniden bağlanma testi
    - RoverStatus ≤1 Hz doğrulama
    - **Motor bağlı değil**

**ROS node'ları:**
- `rscp_bridge_node` — ana bridge node

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/rscp/current_stage` | Publish | `std_msgs/UInt32` | Mevcut stage |
| `/rscp/command` | Publish | `earendil_interfaces/RscpCommand` | RSCP komutu (tip + payload) |
| `/rscp/status` | Publish | `earendil_interfaces/RscpStatus` | Bridge durumu |
| `/mission/command` | Publish | `earendil_interfaces/MissionCommand` | Mission manager'a görev komutu |
| `/mission/result` | Subscribe | `std_msgs/String` | Mission manager'dan sonuç |
| `/mission/status` | Subscribe | `earendil_interfaces/MissionStatus` | Görev durumu |
| `/gps/fix` | Subscribe | `sensor_msgs/NavSatFix` | GPS konumu |
| `/rtk/status` | Subscribe | `std_msgs/String` | RTK fix kalitesi |
| `/stm/imu/data` | Subscribe | `sensor_msgs/Imu` | Heading bilgisi |
| `/stm/wheel_odom` | Subscribe | `nav_msgs/Odometry` | Teker odom (mesafe hesabı) |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_rscp_bridge/earendil_rscp_bridge/__init__.py` | Yeni |
| `src/earendil_rscp_bridge/earendil_rscp_bridge/rscp_bridge_node.py` | Yeni |
| `src/earendil_rscp_bridge/earendil_rscp_bridge/rscp_parser.py` | Yeni |
| `src/earendil_rscp_bridge/config/rscp_bridge_params.yaml` | Yeni |
| `src/earendil_rscp_bridge/launch/rscp_bridge.launch.py` | Yeni |
| `src/earendil_rscp_bridge/package.xml` | Yeni |
| `src/earendil_rscp_bridge/setup.py` | Yeni |
| `src/earendil_rscp_bridge/setup.cfg` | Yeni |
| `src/earendil_rscp_bridge/docs/rscp_bridge.md` | Yeni |
| `test/test_rscp_parser.py` | Yeni |
| `test/rscp_fake_module.py` | Yeni |

**Config / parametreler:**
```yaml
rscp_bridge:
  ros__parameters:
    serial_port: /dev/earendil_rscp
    serial_baud: 115200
    serial_timeout_s: 1.0
    rover_status_period_s: 1.0     # ≤1 Hz
    frame_buffer_timeout_s: 2.0    # partial frame timeout
    reconnect_delay_s: 1.0
    log_level: INFO
```

**Test planı:**
1. Fake RSCP module ile tüm 5 komut tipi parse + dispatch
2. Bilinmeyen komut → crash yok, Acknowledge + message
3. COBS decode hatası → frame at, devam
4. Protobuf parse hatası → frame at, devam
5. Serial kopma → reconnect
6. RoverStatus periyodik ≤1 Hz
7. ArmDisarm arm/disarm sinyali doğru publish

**Kabul kriterleri:**
- [ ] Fake-recorded RSCP frame ile tüm komut türleri parse edilebilir
- [ ] Doğru `WhichOneof` dispatch
- [ ] Her komut sonrası `Acknowledge` gönderilir
- [ ] `RoverStatus` ≤1 Hz periyodik
- [ ] `TaskFinished` / `GPSCoordinate` / `distance` doğru durumlarda gönderilir
- [ ] Serial kopma recovery çalışır
- [ ] COBS decode hatası recovery çalışır
- [ ] Motor bağlı değil

**Bağımlılıklar:** M1 (paket iskeleti, interfaces, `rscp_protobuf`).

**Riskler:**
- Competition Module elimizde değil → fake module ile test, sahada gerçek gerekli
- `RoverState` ↔ `RoverMode_t` eşleme güvenlik kritik, stakeholder ile netleştirilmeli
- Battery telemetry gap (H723'den gelmiyor) → şimdilik 0/NaN
- Heading kaynağı: QMC5883P mag → `/stm/imu/data`'dan yaw (M3'te gelecek)
- EGM96 datum: `altitude` EGM96 geoid yüksekliği (GPS adapter'dan gelmeli)

**Definition of Done:** `earendil_rscp_bridge` paketi çalışıyor, fake RSCP module ile tüm komutlar parse edilebilir, motor bağlı değil.

---

### M3 — STM Bridge ve Güvenli Motor Zinciri

**Amaç:** `/cmd_vel_safe` → H723 ASCII komutu; H723 telemetri → ROS topic'leri; skid-steer kinematik; teker odom üretimi; heartbeat/timeout.

**Ön koşullar:** M1 (interfaces, config).

**Yapılacak işler:**

1. **stm_bridge node:**
   - `/cmd_vel_safe` topic'ini dinle (`geometry_msgs/Twist`)
   - Skid-steer kinematik:
     ```
     left_rpm  = (vx - ω × effective_track_width / 2) / wheel_circumference × 60
     right_rpm = (vx + ω × effective_track_width / 2) / wheel_circumference × 60
     ```
   - Motor mapping: FL=RL (sol), FR=RR (sağ)
   - RPM clamp: ±max_motor_rpm (TBD, M11'de belirlenecek; şimdilik ±300 varsayım)
   - ASCII komut formatı: `FL rpm X\r\n`, `FR rpm X\r\n`, `RL rpm X\r\n`, `RR rpm X\r\n`
   - H723 host komutu: `rpm <signed>` formatı

2. **H723 telemetri parse:**
   - H723'den gelen satırları parse et
   - MPU9250 verisi → `/stm/imu/data` (`sensor_msgs/Imu`)
     - ENU/NED dönüşümü (H723 hangi eksende gönderiyor?)
     - mg → SI birim dönüşümü (ivme: mg→m/s², gyro: mdps→rad/s)
     - Kovaryans ayarları (TBD)
   - QMC5883P verisi → `/stm/magnetic_field` (`sensor_msgs/MagneticField`)
   - Motor telemetri: `RPM:...,T:...,D:...,DIR:...,APP_PH:...,SP:...,BRAKE:...,FC:...,H:...,PWM_SET:...,PWM_ACT:...,QDROP:...,RXB:...`
   - H723 prefix: `FL|...` / `FR|...` / `RL|...` / `RR|...`
   - Durum bilgisi → `/stm/status` (`earendil_interfaces/StmStatus`)
   - Fault flag'leri → `/stm/fault_flags` (`earendil_interfaces/StmFaultFlags`)

3. **Teker odom üretimi:**
   - 4× F411 RPM telemetrisi → ortalama sol/sağ RPM
   - `v = (left_rpm + right_rpm) / 2 × wheel_circumference / 60`
   - `ω = (right_rpm - left_rpm) / wheel_circumference × wheel_circumference / (2 × effective_track_width)`
   - Publish: `/stm/wheel_odom` (`nav_msgs/Odometry`, frame: `odom` → `base_link`)
   - `hall_pulses=90`, `gear_ratio=1.0` kullan

4. **H723 heartbeat/timeout:**
   - H723'den belirli sürede telemetri gelmezse → link loss
   - `LINK_LOSS_TIMEOUT_MS = 3000` (mevcut; M4'te defense-in-depth olarak 300–500ms Pi-tarafı timeout eklenir)
   - Link loss → stm_bridge duruş komutu gönder (0 RPM tüm motorlar)
   - Log warning

5. **H723 firmware gap listesi (Pi-dışı, not amaçlı):**
   - Binary frame formatı yok (ASCII var)
   - CRC/seq/timestamp yok
   - Odometry/pose üretimi yok (Pi'de yapılacak)
   - Battery/status/env yok → RoverStatus battery_state boş kalacak
   - Timeout 3000ms ≠ hedef 300–500ms (Pi-tarafı defense-in-depth M4'te)
   - cmd_vel değil RPM/duty+direction+turn-ratio
   - `RoverState` ↔ `RoverMode_t` eşleme: DISARMED↔DISARM(0), AUTONOMOUS↔AUTONOMOUS(2), MANUAL↔MANUAL(1) — sıra farklı!

6. **Test planı:**
   - Loopback test: stm_bridge → fake H723 (Python script) → telemetri parse doğrulama
   - 4-motor RPM komutu: `/cmd_vel_safe` publish → ASCII komut çıkışı doğrulama
   - Teker odom: telemetri RPM → `/stm/wheel_odom` doğrulama
   - IMU parse: fake MPU9250 verisi → `/stm/imu/data` doğrulama
   - Link loss timeout: telemetri durdur → 3000ms sonra duruş
   - **Motor bağlı değil**

**ROS node'ları:**
- `stm_bridge_node` — cmd_vel→RPM + telemetri parse

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/cmd_vel_safe` | Subscribe | `geometry_msgs/Twist` | Güvenli komut (safety_mux'dan) |
| `/stm/imu/data` | Publish | `sensor_msgs/Imu` | STM IMU verisi |
| `/stm/magnetic_field` | Publish | `sensor_msgs/MagneticField` | STM manyetometre |
| `/stm/wheel_odom` | Publish | `nav_msgs/Odometry` | Teker odom |
| `/stm/status` | Publish | `earendil_interfaces/StmStatus` | STM durumu |
| `/stm/fault_flags` | Publish | `earendil_interfaces/StmFaultFlags` | F411 fault'ları |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_control/earendil_control/stm_bridge.py` | Yeni |
| `src/earendil_control/earendil_control/telemetry_parser.py` | Yeni |
| `src/earendil_control/earendil_control/wheel_odom.py` | Yeni |
| `src/earendil_control/config/vehicle.yaml` | Yeni |
| `src/earendil_control/launch/stm_bridge.launch.py` | Yeni |
| `test/test_telemetry_parser.py` | Yeni |
| `test/fake_h723.py` | Yeni |

**Config / parametreler:**
```yaml
# vehicle.yaml (ayrı dosya, birden fazla node tarafından okunur)
vehicle:
  wheel_base_m: 0.825
  track_width_m: 1.10
  wheel_radius_m: 0.125
  wheel_circumference_m: 0.785
  effective_track_width_m: 0.0  # TBD — M11
  angular_correction_gain: 0.0  # TBD — M11
  left_motor_gain: 1.0          # TBD — M11
  right_motor_gain: 1.0         # TBD — M11
  hall_pulses_per_motor_rev: 90
  gear_ratio: 1.0               # TBD — M11
  motor_pole_pairs: 15
  max_motor_rpm: 300            # TBD — M11
```

**Test planı:**
1. Fake H723 loopback: stm_bridge ↔ Python fake-H723
2. cmd_vel → RPM dönüşümü doğruluğu (1 m/s düz → beklenen RPM)
3. 4 motor mapping: FL=RL, FR=RR
4. Telemetri parse: her alan doğru
5. Teker odom: RPM → v, ω doğruluğu
6. Link loss: 3000ms timeout → duruş
7. **Motor bağlı değil**

**Kabul kriterleri:**
- [ ] Loopback/fake-H723 doğrulama başarılı
- [ ] Telemetri parse: IMU, mag, motor RPM, fault
- [ ] 4-motor RPM komutu: doğru ASCII format
- [ ] `/stm/wheel_odom` dolu ve doğru
- [ ] Link loss → duruş <3000ms
- [ ] Motor bağlı değil

**Bağımlılıklar:** M1 (interfaces, config, vehicle.yaml).

**Riskler:**
- H723 firmware sahipliği — mevcut protokolü anlama ve uyarlama
- Python vs C++ performansı — telemetri parse yüksek frekansta olmalı
- Hub `gear_ratio` (1.0 varsayım) doğrulanmalı
- IMU ENU/NED + mg/mdps birim dönüşümü hatalı olabilir
- CMD_WATCHDOG=800ms uyumu: H723 komut gönderme sıklığı buna uymalı

**Definition of Done:** stm_bridge çalışıyor, fake H723 ile cmd_vel→RPM→telemetri döngüsü doğrulanmış, `/stm/*` topic'leri dolu.

---

### M4 — Safety Mux ve Güvenli Komut Kapısı

**Amaç:** Tek güvenli çıkış `/cmd_vel_safe`; e-stop, deadman, watchdog, komut timeout 300–500ms, hız limitleri; tek-yazıcı denetimi. RSCP arm/disarm gate.

**Ön koşullar:** M3 (stm_bridge, `/cmd_vel_safe` tüketici).

**Yapılacak işler:**

1. **safety_mux node:**
   - Giriş topic'leri: `/cmd_vel_nav` (Nav2), `/cmd_vel_manual` (web teleop)
   - Çıkış topic: `/cmd_vel_safe` (tek güvenli çıkış)
   - Öncelik sırası: e-stop > deadman > watchdog > komut timeout
   - Her gate deterministik sıfır (Twist zero) üretir

2. **E-stop latching:**
   - `/e_stop` topic'inden e-stop sinyali (hardware GPIO + software)
   - E-stop aktifken tüm çıkış sıfır
   - Latching: e-stop kaldırılana kadar sıfırda kalır
   - E-stop kaldırıldıktan sonra komut gelene kadar sıfır

3. **Deadman timeout:**
   - `/deadman` topic'inden periyodik heartbeat
   - Belirli sürede heartbeat gelmezse → sıfır
   - Timeout parametre: `deadman_timeout_ms: 1000`

4. **Komut timeout (defense-in-depth):**
   - `/cmd_vel_nav` ve `/cmd_vel_manual`'dan belirli sürede komut gelmezse → sıfır
   - Timeout: 300–500ms (H723 LINK_LOSS 3000ms'e defense-in-depth)
   - Bu, Pi-tarafı ilk güvenlik katmanıdır

5. **Watchdog node:**
   - STM heartbeat age kontrolü: H723'den telemetri gelmezse → alarm
   - RSCP heartbeat age kontrolü: CM ile bağlantı koparsa → alarm
   - Herhangi biri timeout aşarsa → safety_mux'a sinyal → sıfır

6. **H723 operating mode uyumu:**
   - Boot'ta DISARM default
   - `arm_disarm(true)` → safety_mux arm gate aç → stm_bridge H723'e AUTONOMOUS komutu
   - `arm_disarm(false)` → safety_mux disarm → stm_bridge H723'e DISARM komutu → motor dur

7. **RSCP arm/disarm gate:**
   - `/rscp/command`'dan arm/disarm sinyali
   - Arm: safety_mux aktif modda çalışsın
   - Disarm: safety_mux tüm çıkışı sıfırlasın + stm_bridge disarm komutu

8. **Nav+manual öncelik:**
   - Deadman varsa `/cmd_vel_manual` öncelikli (operatör kontrolü)
   - Deadman yoksa `/cmd_vel_nav` aktif (otonom mod)
   - İkisi aynı anda → manual öncelikli

9. **Tek-yazıcı audit scripti:**
   - `/cmd_vel_safe`'e yalnız safety_mux yazıyor mu? → `ros2 topic info /cmd_vel_safe` kontrol
   - H723 seri portuna yalnız stm_bridge yazıyor mu?
   - RSCP seri portuna yalnız rscp_bridge yazıyor mu?
   - Jetson motor topic'lerine publish yolu yok mu?

**ROS node'ları:**
- `safety_mux_node` — komut multiplexer
- `watchdog_node` — heartbeat monitor

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/cmd_vel_nav` | Subscribe | `geometry_msgs/Twist` | Nav2 navigasyon komutu |
| `/cmd_vel_manual` | Subscribe | `geometry_msgs/Twist` | Web teleop komutu |
| `/cmd_vel_safe` | Publish | `geometry_msgs/Twist` | Tek güvenli çıkış |
| `/e_stop` | Subscribe | `std_msgs/Bool` | E-stop sinyali |
| `/deadman` | Subscribe | `std_msgs/Bool` | Deadman heartbeat |
| `/rscp/command` | Subscribe | `earendil_interfaces/RscpCommand` | Arm/disarm sinyali |
| `/stm/status` | Subscribe | `earendil_interfaces/StmStatus` | STM heartbeat |
| `/safety/status` | Publish | `earendil_interfaces/SafetyStatus` | Safety durumu |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_safety/earendil_safety/safety_mux.py` | Yeni (mevcut safety_node.py yeniden yazılacak) |
| `src/earendil_safety/earendil_safety/watchdog_node.py` | Güncellenecek |
| `src/earendil_safety/config/safety_params.yaml` | Düzelt (topic adları, değerler) |
| `src/earendil_safety/launch/safety.launch.py` | Güncellenecek |
| `test/test_safety_mux.py` | Yeni |
| `scripts/audit_single_writer.py` | Yeni |

**Config / parametreler:**
```yaml
safety_mux:
  ros__parameters:
    input_topics: ["/cmd_vel_nav", "/cmd_vel_manual"]
    output_topic: "/cmd_vel_safe"
    estop_topic: "/e_stop"
    deadman_topic: "/deadman"
    arm_gate_topic: "/rscp/command"
    watchdog_timeout_ms: 400
    command_timeout_ms: 300
    deadman_timeout_ms: 1000
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    estop_latching: true
```

**Test planı:**
1. E-stop aktif → tüm çıkış sıfır
2. Deadman timeout → sıfır
3. Komut timeout 300ms → sıfır
4. Nav+manual aynı anda → manual öncelikli
5. Node drop → watchdog keser
6. Arm/disarm gate: disarm → sıfır
7. Tek-yazıcı audit: `/cmd_vel_safe`'e yalnız safety_mux yazıyor
8. **Motor bağlı değil**

**Kabul kriterleri:**
- [ ] Her safety gate deterministik sıfır
- [ ] Öncelik testleri: e-stop > deadman > watchdog > timeout
- [ ] Node-drop'ta watchdog keser
- [ ] 4-motor kolektif stop < belirli mesafe
- [ ] Tek-yazıcı audit geçer
- [ ] Arm/disarm gate çalışıyor

**Bağımlılıklar:** M3 (stm_bridge `/cmd_vel_safe` tüketici).

**Riskler:**
- Nav+manual öncelik politikası: deadman tarayıcı/bağlantı koparsa ne olur?
- F411 non-latching fault: otomatik resume riski
- Latching policy: e-stop kaldırıldıktan sonra ne kadar beklenmeli?

**Definition of Done:** safety_mux çalışıyor, tüm gate'ler test edilmiş, tek-yazıcı audit geçmiş.

---

### M5 — Sensör Bringup

**Amaç:** RTK GPS, LiDAR, STM IMU, STM manyetometre, STM wheel odom Pi'de doğru frame/republish; TF; diagnostics.

**Ön koşullar:** M3 (STM topic'leri).

**Yapılacak işler:**

1. **RTK GPS adapter:**
   - Seri port → NMEA parse → `/gps/fix` (`sensor_msgs/NavSatFix`)
   - RTK fix kalitesi → `/rtk/status` (FIXED/FLOAT/DGPS/SPS/NO_FIX)
   - Frame: `gps_link`
   - Udev kuralı: `/dev/earendil_rtk`
   - RTK düzeltme kaynağı: NTRIP/hücresel (TBD)

2. **LiDAR adapter:**
   - Model-spesifik driver (TBD — LiDAR modeli belirsiz)
   - `/scan` (`sensor_msgs/LaserScan`)
   - Frame: `lidar_link`
   - Udev kuralı: `/dev/earendil_lidar`

3. **STM republish:**
   - `/stm/imu/data` → ENU dönüşümü + kovaryans ayarla → publish (frame: `imu_link`)
   - `/stm/magnetic_field` → SI birim kontrolü (frame: `imu_link`)
   - `/stm/wheel_odom` → frame_id kontrolü (frame: `odom` → `base_link`)

4. **TF ağacı:**
   - `odom` → `base_link` (stm_bridge'den wheel odom)
   - `map` → `odom` (M6 localization'dan)
   - `base_link` → `imu_link` (statik TF)
   - `base_link` → `lidar_link` (statik TF)
   - `base_link` → `gps_link` (statik TF)
   - `base_link` → `wheel_FL`, `wheel_FR`, `wheel_RL`, `wheel_RR` (statik TF)

5. **Diagnostics:**
   - H723 link durumu + heartbeat age
   - F411 14-fault durumu
   - GPS fix kalitesi
   - LiDAR sağlık
   - Battery (TBD — H723 gap)

6. **Bringup launch:**
   - Tüm sensörleri başlatan launch dosyası
   - `use_hardware` flag (fake/gerçek geçiş)

**ROS node'ları:**
- `gps_adapter_node` — RTK GPS
- `lidar_adapter_node` — LiDAR
- `imu_republish_node` — STM IMU republish (opsiyonel, stm_bridge doğrudan yapabilir)
- `diagnostics_node` — sistem sağlık

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/gps/fix` | Publish | `sensor_msgs/NavSatFix` | RTK GPS |
| `/rtk/status` | Publish | `std_msgs/String` | RTK fix kalitesi |
| `/scan` | Publish | `sensor_msgs/LaserScan` | LiDAR tarama |
| `/stm/imu/data` | Republish | `sensor_msgs/Imu` | STM IMU (ENU, kovaryans) |
| `/stm/magnetic_field` | Republish | `sensor_msgs/MagneticField` | STM mag |
| `/stm/wheel_odom` | Republish | `nav_msgs/Odometry` | STM teker odom |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_sensors/earendil_sensors/gps_adapter.py` | Güncellenecek |
| `src/earendil_sensors/earendil_sensors/lidar_adapter.py` | Güncellenecek |
| `src/earendil_sensors/earendil_sensors/imu_adapter.py` | Güncellenecek |
| `src/earendil_sensors/earendil_sensors/diagnostics_node.py` | Yeni |
| `src/earendil_sensors/config/sensor_params.yaml` | Düzelt (wheel_base, wheel_radius) |
| `deploy/99-earendil.rules` | Güncellenecek (udev) |
| `src/earendil_bringup/launch/sensors.launch.py` | Yeni |

**Config / parametreler:**
```yaml
gps_adapter:
  ros__parameters:
    serial_port: /dev/earendil_rtk
    serial_baud: 9600
    publish_rate_hz: 5.0
    fix_topic: /gps/fix
    rtk_status_topic: /rtk/status
    frame_id: gps_link

lidar_adapter:
  ros__parameters:
    scan_topic: /scan
    frame_id: lidar_link
    publish_rate_hz: 10.0

diagnostics:
  ros__parameters:
    stm_status_topic: /stm/status
    stm_fault_topic: /stm/fault_flags
    gps_topic: /gps/fix
    lidar_topic: /scan
    check_rate_hz: 5.0
```

**Test planı:**
1. `/gps/fix` RTK_FIXED ile dolu
2. `/rtk/status` doğru kalite raporluyor
3. `/scan` dolu ve doğru frame_id
4. TF ağacı `ros2 tf echo` ile doğrulanıyor
5. Diagnostics: tüm sensör sağlık topic'leri dolu
6. Udev kararlı isim

**Kabul kriterleri:**
- [ ] `/gps/fix` RTK_FIXED
- [ ] `/scan` dolu
- [ ] TF geçerli (REP-105)
- [ ] Diagnostics temiz
- [ ] `/stm/*` frame_id hatasız
- [ ] Udev kararlı

**Bağımlılıklar:** M3 (STM topic'leri).

**Riskler:**
- Jazzy/Ubuntu24 driver uyumsuzluğu
- RTK düzeltme kaynağı belirsiz (NTRIP/hücresel)
- LiDAR modeli belirsiz
- IMU ENU/NED + mg/mdps→SI dönüşümü
- `/scan` bant genişliği / CPU

**Definition of Done:** Tüm sensörler publish ediyor, TF geçerli, diagnostics çalışıyor, udev kararlı.

---

### M6 — Localization (RTK + STM odom + STM IMU)

**Amaç:** `robot_localization` ile odom/map füzyon; RTK global, STM odom+IMU yerel.

**Ön koşullar:** M5 (sensör topic'leri).

**Yapılacak işler:**

1. **Local EKF/UKF:**
   - Girdi: `/stm/wheel_odom` + `/stm/imu/data`
   - Çıktı: `odom` → `base_link` TF + `/odometry/filtered`
   - Dosya: `ukf_local.yaml`

2. **Global EKF/UKF:**
   - Girdi: local odometry + `/gps/fix` (navsat_transform ile)
   - Çıktı: `map` → `odom` TF + `/odometry/global`
   - Dosya: `ukf_global.yaml`

3. **navsat_transform:**
   - GPS → map koordinat dönüşümü
   - Datum: ilk RTK_FIXED konum veya sabit
   - Dosya: `navsat.yaml`

4. **RTK politikası:**
   - `FIXED` → güvenilir, global EKF GPS ağırlığı yüksek
   - `FLOAT` → 40cm tolerans, ağırlık orta
   - `DGPS` → 40cm tolerans, ağırlık orta
   - `SPS` / `NO_FIX` → GPS ağırlığı çok düşük veya yok
   - GPS hiç yoksa → otonom hareket başlatılmaz (M7 mission manager'da)

5. **Jetson visual odometry:**
   - Default kapalı yardımcı kaynak
   - M8'de Jetson hazır olduğunda opsiyonel açılabilir
   - Ağırlık düşük (yardımcı, birincil değil)

6. **Kovaryans ayarları:**
   - Wheel odom kovaryansı: TBD (saha kalibrasyonu)
   - IMU kovaryansı: TBD (saha kalibrasyonu)
   - GPS kovaryansı: RTK kalitesine göre dinamik (TBD)

**ROS node'ları:**
- `robot_localization` (navsat_transform_node + ekf_node × 2)

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/odometry/filtered` | Publish | `nav_msgs/Odometry` | Local odometry |
| `/odometry/global` | Publish | `nav_msgs/Odometry` | Global odometry |
| `/gps/fix` | Subscribe | `sensor_msgs/NavSatFix` | RTK GPS |
| `/rtk/status` | Subscribe | `std_msgs/String` | RTK kalitesi |
| `/stm/wheel_odom` | Subscribe | `nav_msgs/Odometry` | Teker odom |
| `/stm/imu/data` | Subscribe | `sensor_msgs/Imu` | STM IMU |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_bringup/config/ukf_local.yaml` | Güncellenecek |
| `src/earendil_bringup/config/ukf_global.yaml` | Güncellenecek |
| `src/earendil_bringup/config/navsat.yaml` | Güncellenecek |
| `src/earendil_navigation/earendil_navigation/gps_monitor.py` | Güncellenecek |
| `src/earendil_bringup/launch/localization.launch.py` | Yeni |

**Config / parametreler:**
```yaml
# ukf_local.yaml
ukf_local:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /stm/wheel_odom
    imu0: /stm/imu/data
    # ... kovaryans ayarları (TBD)

# ukf_global.yaml
ukf_global:
  ros__parameters:
    frequency: 10.0
    world_frame: map
    odom0: /odometry/filtered
    # ... GPS navsat_transform'dan gelecek

# navsat.yaml
navsat:
  ros__parameters:
    frequency: 5.0
    use_odometry_yaw: false
    wait_for_datum: false
    # datum: [lat, lon, yaw] veya ilk fix
```

**Test planı:**
1. Kısa mesafe drift testi: düz sürüş, drift < belirli eşik
2. RTK-FIXED: map↔GPS tutarlılığı
3. RTK kaybında: local-only modda sane davranış
4. Düşük hız: düzgün odometry
5. `/rtk/status` doğru raporlanıyor

**Kabul kriterleri:**
- [ ] Kısa mesafe drift sınırlı
- [ ] RTK-fixed map↔GPS tutarlı
- [ ] RTK kaybında sane
- [ ] Düşük hız düzgün
- [ ] rtk_status doğru

**Bağımlılıklar:** M5 (sensör topic'leri).

**Riskler:**
- RTK kayıpları politikası: FLOAT/SPS davranışı belirsiz
- Datum seçimi: sabit mi ilk fix mi?
- Tek vs çift EKF: hangisi daha kararlı?
- Kovaryans ayarları: saha kalibrasyonu gerekli
- Jetson/LiDAR odom ağırlığı belirsiz

**Definition of Done:** Localization çalışıyor, RTK-fixed tutarlı, local odom drift sınırlı.

---

### M7 — Nav2, Mission Manager ve Görev Akışı

**Amaç:** RSCP komutlarını otonom görev akışına dönüştür. Mission manager'ı RSCP stage-aware yap. Nav2 ile GPS navigasyonu. RTK-bilgili hedef onayı.

**Ön koşullar:** M2 (RSCP bridge), M3 (stm_bridge), M4 (safety), M6 (localization).

**Yapılacak işler:**

1. **Nav2 yapılandırması:**
   - `bt_navigator`: behavior tree
   - `controller`: DWB veya MPPI (skid-steer uyumlu)
   - `planner`: NavFn veya Smac
   - `costmap_2d`: global + local, `/scan` girdisi
   - Skid-steer tuning: `vx` + `wz` (differential drive modeli)
   - `nav2_params.yaml` detaylı yapılandırma

2. **Mission manager yeniden yazım:**
   - Mevcut `mission_manager.py` generic waypoint navigator → RSCP stage-aware yeniden yazılacak
   - Stage state machine (§7'deki tablolar)
   - RSCP komut dinleme: `/rscp/command`, `/rscp/current_stage`, `/mission/command`
   - Stage-aware görev planı:
     - `SetStage(v)` → stage değişikliği
     - `NavigateToGPS(coord)` → Nav2 goal gönder
     - `SearchArea(center,r)` → arama davranışı başlat (spiral/grid pattern)
     - `StartExploration` → keşif modu başlat
   - Görev sonucu publish: `/mission/result` (RSCP bridge dinler)
   - RTK quality check: hedefe varış onayı

3. **NavigateToGPS akışı:**
   - RSCP `navigate_to_gps(coordinate)` → `/mission/command`
   - Mission manager: GPS koordinatını map frame'e dönüştür (`navsat_transform` / `fromLL`)
   - Nav2 goal gönder
   - Varış kontrolü: GPS mesafesi < `arrival_tolerance` VE RTK kalitesi yeterli
   - RTK onayı: FIXED → güvenilir varış; FLOAT/DGPS → 40cm tolerans; SPS/NO_FIX → varış onayı yok
   - Varışta → `/mission/result` = `task_finished`

4. **SearchArea akışı:**
   - RSCP `search_area(center, radius)` → `/mission/command`
   - Mission manager: spiral/grid search pattern oluştur
   - Her waypoint'a Nav2 goal gönder
   - Hedef tespiti: Jetson ArUco, LiDAR anomali, veya operatör müdahalesi
   - Hedef bulundu → GPS koordinatını raporla → `/mission/result` = `gps_coordinate` + `task_finished`
   - Hedef bulunamadı → timeout → `/mission/result` = `task_finished` (hata)

5. **StartExploration akışı:**
   - RSCP `start_exploration` → `/mission/command`
   - Mission manager: keşif modu başlat
   - Mesafe ölçümü: `/stm/wheel_odom` integrasyonu (kümülatif)
   - ArUco tag j arama (Jetson)
   - Çıkış tag'i bulundu → mesafe raporla → `/mission/result` = `distance` + `task_finished`
   - Timeout → mevcut mesafe + `task_finished`

6. **ArmDisarm güvenlik eşleme:**
   - RSCP `arm_disarm(true)` → mission manager'a arm sinyali
   - Mission manager: H723 AUTONOMOUS moduna geçiş isteği (stm_bridge üzerinden)
   - RSCP `arm_disarm(false)` → disarm → H723 DISARM → motor dur
   - RoverState eşleme: DISARMED(0)↔DISARM, AUTONOMOUS(1)↔AUTONOMOUS, MANUAL(2)↔MANUAL

7. **Pause/cancel/retry/timeout:**
   - Pause: Nav2 goal iptal, mevcut konumda bekle
   - Cancel: tüm görev iptal, sıfır komut
   - Retry: mevcut waypoint/alt-görev tekrar
   - Timeout: belirli sürede tamamlanamazsa → güvenli duruş + hata raporu

8. **RSCP yanıtı gönderimi:**
   - `TaskFinished`: `/mission/result`'a `task_finished` sinyali → RSCP bridge → CM
   - `GPSCoordinate`: hedef konum bulunduğunda → RSCP bridge → CM
   - `distance`: mesafe ölçümü tamamlandığında → RSCP bridge → CM

9. **Mission logging:**
   - Başarılı/başarısız/süre
   - Waypoint hata/RMSE
   - Hız profili
   - RTK kalitesi log

10. **Defense-in-depth:**
    - E-stop'ta `/cmd_vel_nav`'e sıfır basma (safety_mux'dan bağımsız)

**ROS node'ları:**
- `mission_manager_node` — RSCP-aware mission state machine
- Nav2 stack (bt_navigator, controller, planner, costmap)

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/rscp/command` | Subscribe | `earendil_interfaces/RscpCommand` | RSCP komutları |
| `/rscp/current_stage` | Subscribe | `std_msgs/UInt32` | Mevcut stage |
| `/mission/command` | Subscribe | `earendil_interfaces/MissionCommand` | Mission komutu |
| `/mission/result` | Publish | `std_msgs/String` | Görev sonucu |
| `/mission/status` | Publish | `earendil_interfaces/MissionStatus` | Görev durumu |
| `/cmd_vel_nav` | Publish | `geometry_msgs/Twist` | Nav2 komutu |
| `/gps/fix` | Subscribe | `sensor_msgs/NavSatFix` | RTK GPS |
| `/rtk/status` | Subscribe | `std_msgs/String` | RTK kalitesi |
| `/jetson/aruco_detections` | Subscribe | `earendil_interfaces/JetsonArucoDetections` | ArUco tespitleri |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_navigation/earendil_navigation/mission_manager.py` | Yeniden yazım |
| `src/earendil_navigation/config/nav2_params.yaml` | Düzelt (skid-steer tuning) |
| `src/earendil_navigation/config/mission_params.yaml` | Yeni |
| `src/earendil_navigation/launch/mission.launch.py` | Yeni |
| `src/earendil_navigation/launch/nav2.launch.py` | Yeni |
| `test/test_mission_manager.py` | Yeni |
| `test/test_stage_state_machine.py` | Yeni |

**Config / parametreler:**
```yaml
mission_manager:
  ros__parameters:
    arrival_tolerance_m: 1.0
    rtk_fixed_tolerance_m: 0.3
    rtk_float_tolerance_m: 0.4
    rtk_no_fix_action: stop    # stop veya degrade
    search_pattern: spiral     # spiral veya grid
    search_speed_mps: 0.3
    exploration_timeout_s: 300
    waypoint_timeout_s: 120
    max_retry_count: 3
    log_mission: true
    log_path: /var/log/earendil/missions/
```

**Test planı:**
1. Stage state machine: her stage geçişi doğru
2. NavigateToGPS: fake GPS + Nav2 goal → varış
3. SearchArea: spiral pattern → hedef bulma
4. StartExploration: mesafe ölçümü + çıkış tag'i
5. RTK quality check: FIXED/FLOAT/SPS/NO_FIX davranışı
6. Pause/resume/cancel/retry
7. Timeout davranışı
8. RSCP fake module ile tam stage akışı

**Kabul kriterleri:**
- [ ] Mission çalışır: 4 stage fake module ile test edilebilir
- [ ] Pause/resume sağlam
- [ ] Cancel reset
- [ ] Retry çalışır
- [ ] Timeout → güvenli duruş
- [ ] Varış fix kalitesine bağlı
- [ ] Mission log dolar
- [ ] Engelde recover (Nav2 costmap)
- [ ] RSCP stage akışı: SetStage → SearchArea/NavigateToGPS → TaskFinished

**Bağımlılıklar:** M2 (RSCP bridge), M3 (stm_bridge), M4 (safety), M6 (localization).

**Riskler:**
- RTK bozulmasında tolerans politikası belirsiz
- BT/planner skid-steer seçimi: hangi controller en iyi?
- Costmap CPU tüketimi
- Retry yapısı: kaç deneme, ne zaman vazgeç?
- `RoverState` ↔ `RoverMode_t` eşleme güvenliği

**Definition of Done:** Mission manager RSCP stage-aware çalışıyor, Nav2 goal navigasyonu çalışır, fake module ile 4 stage test edilebilir.

---

### M8 — Jetson Stereo Vision, ArUco ve Derinlik

**Amaç:** Waveshare stereo kamera ile ArUco tespiti, derinlik/engel, opsiyonel visual odometry. Kamera 9-eksen IMU'su yalnız vision aux. Jetson motor komutu göndermez.

**Ön koşullar:** M5 (sensör bringup — TF, topic convention).

**Yapılacak işler:**

1. **Waveshare stereo kamera kurulumu:**
   - Jetson Nano'ya kamera bağlama
   - OpenCV + ArUco modülü
   - `cv2.aruco.DICT_ARUCO_ORIGINAL` sözlüğü (bkz. `examples/aruco_detection_example.py`)
   - `ArucoDetector` kullanımı

2. **ArUco tag tespiti:**
   - Tag i (Stage 3 giriş), tag j (Stage 3 çıkış), tag k (Stage 4 airlock)
   - Tespit → `/jetson/aruco_detections` (`earendil_interfaces/JetsonArucoDetections`)
   - Tag ID, konum (kamera frame), mesafe tahmini

3. **Derinlik/engel tespiti:**
   - Stereo derinlik haritası → engel tespiti
   - `/jetson/obstacles` — engel listesi
   - `/jetson/depth_summary` — özet derinlik bilgisi

4. **Visual odometry (opsiyonel):**
   - Stereo VO → `/jetson/visual_odom`
   - Default kapalı (M6'da yardımcı kaynak olarak açılabilir)
   - Ağırlık düşük, birincil localization değil

5. **Kamera IMU:**
   - Jetson-side kamera 9-eksen IMU → `/jetson/imu/data`
   - Bu **ana IMU değil**, yalnızca vision aux
   - Ana IMU STM MPU9250'dir

6. **Jetson → Pi iletişimi:**
   - Ethernet üzerinden ROS 2 DDS
   - İzole domain-id (DDS interference önlemi)
   - `earendil_jetson_bridge` node: Jetson'dan gelen topic'leri Pi'de relay/validate

7. **Jetson sağlık ve kopma:**
   - Jetson heartbeat monitor
   - Jetson koparsa → LiDAR-only fallback
   - Nav2 vision layer default kapalı (Jetson hazır olduğunda opsiyonel)

8. **CI audit:**
   - Jetson node'larının motor topic'lerine publish yolu yok
   - `scripts/audit_jetson_no_motor.py`

**ROS node'ları:**
- `jetson_bridge_node` — Jetson topic'lerini relay/validate
- Jetson-side: `aruco_detector_node`, `depth_processor_node`, `visual_odom_node`

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/jetson/aruco_detections` | Publish | `earendil_interfaces/JetsonArucoDetections` | ArUco tespitleri |
| `/jetson/obstacles` | Publish | custom | Engel listesi |
| `/jetson/depth_summary` | Publish | custom | Derinlik özeti |
| `/jetson/visual_odom` | Publish | `nav_msgs/Odometry` | Opsiyonel VO |
| `/jetson/imu/data` | Publish | `sensor_msgs/Imu` | Kamera IMU (aux) |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_jetson_bridge/earendil_jetson_bridge/__init__.py` | Yeni |
| `src/earendil_jetson_bridge/earendil_jetson_bridge/jetson_bridge_node.py` | Yeni |
| `src/earendil_jetson_bridge/config/jetson_bridge_params.yaml` | Yeni |
| `src/earendil_jetson_bridge/launch/jetson_bridge.launch.py` | Yeni |
| `src/earendil_jetson_bridge/package.xml` | Yeni |
| `src/earendil_jetson_bridge/setup.py` | Yeni |
| Jetson-side: `jetson_aruco/` | Yeni (Jetson'da çalışacak) |
| `scripts/audit_jetson_no_motor.py` | Yeni |

**Config / parametreler:**
```yaml
jetson_bridge:
  ros__parameters:
    aruco_topic: /jetson/aruco_detections
    obstacles_topic: /jetson/obstacles
    depth_summary_topic: /jetson/depth_summary
    visual_odom_topic: /jetson/visual_odom
    imu_topic: /jetson/imu/data
    heartbeat_timeout_s: 5.0
    fallback_mode: lidar_only
```

**Test planı:**
1. ArUco tag tespiti: bilinen tag → doğru ID + konum
2. Derinlik/engel: engel var → topic dolu
3. Jetson kopma → LiDAR-only fallback alert
4. Topic gecikme bütçesi: <100ms (Ethernet)
5. CI audit: Jetson → motor yolu yok
6. Kamera IMU: aux, ana IMU değil

**Kabul kriterleri:**
- [ ] ArUco tespit edilebilir (tag i/j/k)
- [ ] Özet topic'ler gecikme bütçesinde
- [ ] Jetson kopması alert + LiDAR-only
- [ ] CI audit: Jetson → motor yolu yok
- [ ] Kamera IMU aux

**Bağımlılıklar:** M5 (sensör bringup, TF).

**Riskler:**
- WiFi/Ethernet DDS interference
- Jetson saat senkronizasyonu (NTP/PTP)
- `depth_summary` şeması belirsiz
- Jetson Nano performansı: stereo + ArUco + derinlik aynı anda
- ArUco sözlüğü ve tag boyutu belirsiz
- Waveshare stereo modeli belirsiz

**Definition of Done:** ArUco tespit ediliyor, derinlik/engel topic'leri dolu, Jetson kopma fallback çalışıyor.

---

### M9 — Web Kontrol Paneli ve İzleme

**Amaç:** İzleme + kontrol paneli; zorunlu deadman teleop; e-stop; fault log.

**Ön koşullar:** M4 (safety), M7 (mission manager).

**Yapılacak işler:**

1. **Web server:**
   - RPIP5 üzerinde HTTP/WebSocket server
   - Araç durumu: e-stop, sensor sağlık, mod
   - RSCP stage + status gösterimi
   - RTK / STM / Jetson / LiDAR durumu
   - Aktif görev + waypoint haritası

2. **Deadman teleop:**
   - Web arayüzünden joystick/slider ile teleop
   - `/cmd_vel_manual` topic'ine publish
   - **Zorunlu deadman:** tarayıcı/bağlantı kopması → durur (M3 timeout katmanı)
   - Deadman heartbeat: periyodik tıklama/tutma

3. **E-stop:**
   - Web aracılığıyla software e-stop
   - `/e_stop` topic'ine publish
   - Safety_mux tetikler

4. **Durum gösterimi:**
   - H723 14-fault + link durumu
   - Battery (TBD — H723 gap)
   - GPS fix kalitesi
   - LiDAR sağlık
   - Jetson sağlık
   - Motor RPM'leri

5. **Fault log:**
   - Hata geçmişi gösterimi
   - Timestamp + hata açıklaması

6. **Güvenlik sınırları:**
   - Web yalnız `/cmd_vel_manual` yazar
   - `/cmd_vel_safe`'e **hiç** yazamaz (CI audit)
   - `earendil.py`'yi etkilemez (farklı paket, karantina)

**ROS node'ları:**
- `web_server_node` — HTTP/WebSocket server

**Topic / service / action yapısı:**
| Topic | Yön | Tip | Açıklama |
|---|---|---|---|
| `/cmd_vel_manual` | Publish | `geometry_msgs/Twist` | Deadman teleop |
| `/e_stop` | Publish | `std_msgs/Bool` | Web e-stop |
| `/deadman` | Publish | `std_msgs/Bool` | Deadman heartbeat |
| `/safety/status` | Subscribe | `earendil_interfaces/SafetyStatus` | Safety durumu |
| `/stm/status` | Subscribe | `earendil_interfaces/StmStatus` | STM durumu |
| `/mission/status` | Subscribe | `earendil_interfaces/MissionStatus` | Görev durumu |
| `/rtk/status` | Subscribe | `std_msgs/String` | RTK kalitesi |

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_web/earendil_web/web_server.py` | Güncellenecek |
| `src/earendil_web/earendil_web/templates/` | Yeni (HTML/CSS/JS) |
| `src/earendil_web/config/web_params.yaml` | Yeni |
| `src/earendil_web/launch/web.launch.py` | Yeni |
| `scripts/audit_web_no_safe.py` | Yeni |

**Config / parametreler:**
```yaml
web_server:
  ros__parameters:
    port: 8080
    cmd_vel_manual_topic: /cmd_vel_manual
    estop_topic: /e_stop
    deadman_topic: /deadman
    deadman_interval_ms: 500
    max_linear_speed: 0.3
    max_angular_speed: 0.5
```

**Test planı:**
1. Dashboard: tüm alt sistem durumu görünüyor
2. Deadman teleop: yalnız deadman'le çalışıyor
3. Tarayıcı kapanırsa → durur
4. Web e-stop → sıfırlar
5. Fault log dolar
6. CI audit: `/cmd_vel_safe`'e web yazma yok

**Kabul kriterleri:**
- [ ] Dashboard durum gösteriyor
- [ ] Teleop yalnız deadman'le
- [ ] Tarayıcı kapansa durur
- [ ] Web e-stop sıfırlar
- [ ] Fault log dolar
- [ ] `/cmd_vel_safe`'e web yazma CI audit'i

**Bağımlılıklar:** M4 (safety), M7 (mission manager).

**Riskler:**
- Mobil/tarayıcı uyumluluğu
- WebSocket vs HTTP polling
- Auth/TLS (opsiyonel)
- Server crash defense-in-depth

**Definition of Done:** Web panel çalışıyor, deadman teleop güvenli, e-stop çalışıyor.

---

### M10 — Deploy ve Systemd

**Amaç:** Tekrarlanabilir install/build/run; boot autostart; istikrar.

**Ön koşullar:** M7 (tüm stack çalışır).

**Yapılacak işler:**

1. **Install script:**
   - `deploy/install_rpi5_ubuntu24.sh`
   - ROS 2 Jazzy kurulumu (native, PPA)
   - `rscp_protobuf` pip kurulumu
   - `cobs` pip kurulumu
   - Bağımlılıklar: `pyserial`, `rclpy`, `nav2`, `robot_localization`
   - Udev kuralları

2. **Build script:**
   - `deploy/build.sh`
   - `colcon build --symlink-install`
   - Kaynak: `source /opt/ros/jazzy/setup.bash`

3. **Run script:**
   - `deploy/run_vehicle.sh`
   - Kaynak: `source install/setup.bash`
   - Launch: `ros2 launch earendil_bringup vehicle.launch.py`

4. **Systemd servisi:**
   - `deploy/earendil-vehicle.service`
   - `ExecStartPre`: build (opsiyonel)
   - `ExecStart`: run_vehicle.sh
   - `Restart=on-failure`
   - `WatchdogSec` (sd_notify)
   - `After=network-online.target`
   - `Journal` entegrasyonu

5. **Udev kuralları:**
   - `/dev/earendil_h7` — H723 STM32
   - `/dev/earendil_rtk` — RTK GPS
   - `/dev/earendil_lidar` — LiDAR
   - Jetson Ethernet

6. **`.env.example`:**
   - Seri port yolları
   - Baud rate'ler
   - Udev isimleri
   - Log dizini

7. **Docker:**
   - Opsiyonel, yalnız dev/test
   - Ana deploy Docker'a bağlı değil

**ROS node'ları:** Yok — deploy altyapısı.

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `deploy/install_rpi5_ubuntu24.sh` | Mevcut (güncellenecek) |
| `deploy/build.sh` | Mevcut (güncellenecek) |
| `deploy/run_vehicle.sh` | Mevcut (güncellenecek) |
| `deploy/earendil-vehicle.service` | Mevcut (güncellenecek) |
| `deploy/99-earendil.rules` | Mevcut (güncellenecek) |
| `.env.example` | Yeni |

**Test planı:**
1. Temiz Pi5'te install script çalışır
2. Build script temiz derler
3. Run script stack başlatır
4. Cold boot: systemd ile stack otomatik başlıyor
5. Crash: systemd restart
6. Serial kararlı
7. `journalctl -u earendil-vehicle` temiz
8. İkinci Pi'de tekrarlanabilir
9. `rscp_protobuf` import edilebilir

**Kabul kriterleri:**
- [ ] Cold boot stack başlatır
- [ ] Systemd crash'te restart
- [ ] Serial kararlı
- [ ] journalctl temiz
- [ ] İkinci Pi'de tekrarlanabilir
- [ ] `rscp_protobuf` import edilebilir

**Bağımlılıklar:** M7 (tüm stack çalışır).

**Riskler:**
- Pi güç/termal
- Jetson network ordering
- Systemd/ROS daemon ordering
- Multi-host DDS yapılandırması
- SD kart storage wear (log rotasyonu gerekli)

**Definition of Done:** Tekrarlanabilir deploy, boot autostart, systemd kararlı.

---

### M11 — ARC Saha Testleri ve Final Doğrulama

**Amaç:** Tüm milestone'ların entegre saha testi; `vehicle.yaml` TBD'lerini kalibre; ARC görev senaryoları tam akış.

**Ön koşullar:** M7 (mission manager), M10 (deploy). M8/M9 paralel çalışabilir.

**Yapılacak işler:**

1. **Fake RSCP module testi:**
   - Tüm komut türleri (SetStage, ArmDisarm, NavigateToGPS, SearchArea, StartExploration)
   - Doğru yanıt: Acknowledge, TaskFinished, GPSCoordinate, distance
   - COBS framing + protobuf parse + dispatch doğrulama

2. **Gerçek Competition Module testi:**
   - Gerçek CM ile seri bağlantı
   - Tüm komut akışları doğrulama
   - Bu test CM elimizde olduğunda mümkün

3. **Stage 1 tam akış:**
   ```
   SetStage(1) → Ack
   ArmDisarm(true) → Ack
   SearchArea(center,r) → Ack
   [otonom arama]
   GPSCoordinate(tepe) → gönderildi
   TaskFinished → gönderildi
   ```

4. **Stage 2 tam akış:**
   ```
   SetStage(2) → Ack
   SearchArea(center,r) → Ack
   [otonom arama]
   GPSCoordinate(kaya) → gönderildi
   TaskFinished → gönderildi
   ```

5. **Stage 3 tam akış:**
   ```
   SetStage(3) → Ack
   NavigateToGPS(giriş) → Ack
   [otonom navigasyon]
   TaskFinished → gönderildi (giriş ulaşıldı)
   StartExploration → Ack
   [keşif + mesafe ölçümü]
   distance → gönderildi
   TaskFinished → gönderildi
   ```

6. **Stage 4 tam akış:**
   ```
   SetStage(4) → Ack
   NavigateToGPS(airlock) → Ack
   [otonom navigasyon]
   TaskFinished → gönderildi (airlock ulaşıldı)
   ArmDisarm(false) → Ack
   [disarm]
   ```

7. **1m düz sürüş testi:**
   - Düz 1 metre sürüş
   - Hata: ±5cm (gain/circ kalibrasyonu)

8. **90° tank dönüş:**
   - 90 derece tank dönüşü
   - Hata: ±5° (angular_correction kalibrasyonu)

9. **360° tank dönüş:**
   - 360 derece tank dönüşü
   - Hata: ±10° (effective_track_width kalibrasyonu)

10. **Sol/sağ eşitleme:**
    - Düz sürüşte drift → motor gain kalibrasyonu

11. **Hall odom – RTK mesafe:**
    - Teker odom vs RTK mesafe karşılaştırma
    - Hata: <2% (wheel_radius/pulses/gear doğrulama)

12. **IMU yaw – tank dönüş:**
    - IMU yaw vs tank dönüş açısı
    - Yaw scaling + bias kalibrasyonu

13. **Düşük hız waypoint:**
    - Düşük hızda waypoint navigasyonu
    - arrival_tolerance kalibrasyonu

14. **RTK hedef onayı:**
    - FIXED → güvenilir varış
    - FLOAT/DGPS → 40cm tolerans
    - SPS/NO_FIX → onay yok

15. **ArUco tag testi:**
    - Tag i, j, k tespit edilebilir
    - Mesafe tahmini doğruluğu

16. **LiDAR engel testi:**
    - Engel algılama + önleme

17. **Jetson kopma testi:**
    - Jetson kapatıldı → LiDAR-only fallback

18. **E-stop testi:**
    - E-stop → tüm motor duruşu < belirli mesafe

19. **Watchdog testi:**
    - Pi timeout → H723 LINK_LOSS → F411 CMD_WATCHDOG → duruş

20. **RoverState ↔ RoverMode_t testi:**
    - Arm/disarm güvenli geçiş doğrulama

21. **Full mission rehearsal:**
    - 4 stage tam akış, baştan sona

**ROS node'ları:** Tümü — entegre test.

**Oluşturulacak veya güncellenecek dosyalar:**
| Dosya | Durum |
|---|---|
| `src/earendil_control/config/vehicle.yaml` | Kalibre (M11 TBD'leri) |
| `src/earendil_safety/config/safety_params.yaml` | Kalibre |
| `test/saha_test_proseduru.md` | Yeni |
| `test/kalibrasyon_formlari/` | Yeni |
| `test/arc_mission_scenarios/` | Yeni |

**Config / parametreler (kalibrasyon):**
```yaml
# vehicle.yaml — M11'de doldurulacak
effective_track_width_m: <ölçülen>    # 360° testinden
angular_correction_gain: <ölçülen>    # 90° testinden
left_motor_gain: <ölçülen>            # sol/sağ eşitleme
right_motor_gain: <ölçülen>           # sol/sağ eşitleme
gear_ratio: <doğrulanmış>             # hall odom testinden
max_motor_rpm: <test edilmiş>         # saha testinden
```

**Test planı:** Yukarıdaki 21 test maddesi.

**Kabul kriterleri:**
- [ ] Her test eşiği geçer
- [ ] Tekrarlanabilir
- [ ] Üç timeout katmanı (Pi, H723, F411) birlikte duruş
- [ ] F411 non-latching fault otomatik resume risk değerlendirme
- [ ] Tüm 4 stage senaryoları tamamlanabilir
- [ ] `vehicle.yaml` TBD'leri kalibre

**Bağımlılıklar:** M7 (mission manager), M10 (deploy). M8/M9 paralel.

**Riskler:**
- Yüzey değişimi (effective_track_width yüzeye bağımlı)
- Low-RPM motor deadband (<10 RPM)
- Termal: uzun çalışma → motor/ESC ısınması
- 300–400 RPM henüz test edilmedi
- RSCP Competition Module saha erişimi (fake ile test edilebilir ama gerçek gerekli)

**Definition of Done:** Tüm 4 stage tam akış testi geçmiş, saha kalibrasyonu tamamlanmış, ARC yarışmasına hazır.

---

## 10. Test Stratejisi

### 10.1 Test Katmanları

| Katman | Kapsam | Araç | Ne zaman |
|---|---|---|---|
| Unit test | Tek fonksiyon/sınıf | `pytest` | Her commit |
| Fake serial test | Seri iletişim simülasyonu | `io.BytesIO`, Python fake | M2, M3 |
| Fake RSCP module | COBS+protobuf framing + dispatch | Kayıtlı frame'ler | M2, M7 |
| Fake H723 test | cmd_vel→RPM + telemetri parse | Python fake-H723 | M3 |
| ROS launch test | Node başlatma/kapatma | `ros2 launch` + timeout | M1–M10 |
| Integration test | Birden fazla node | Fake donanım + ROS | M7 |
| Hardware-in-the-loop | Gerçek donanım, simülasyon komut | Gerçek STM + fake RSCP | M11 öncesi |
| Field test | Gerçek donanım + gerçek CM | Saha | M11 |
| Regression test | Milestone gerileme kontrolü | CI/CD | Her milestone |

### 10.2 Test Kapsamı

| Milestone | Unit | Fake serial | Fake RSCP | Fake H723 | Launch | Integration | HIL | Field |
|---|---|---|---|---|---|---|---|---|
| M1 | — | — | proto import | — | ✓ | — | — | — |
| M2 | parse | ✓ | ✓ | — | ✓ | — | — | — |
| M3 | parse | — | — | ✓ | ✓ | — | — | — |
| M4 | mux | — | — | — | ✓ | ✓ | — | — |
| M5 | adapter | — | — | — | ✓ | — | — | — |
| M6 | — | — | — | — | ✓ | ✓ | — | — |
| M7 | state machine | — | ✓ | — | ✓ | ✓ | — | — |
| M8 | aruco | — | — | — | ✓ | — | — | — |
| M9 | — | — | — | — | ✓ | — | — | — |
| M10 | — | — | — | — | ✓ | — | — | — |
| M11 | — | — | ✓ | ✓ | — | ✓ | ✓ | ✓ |

### 10.3 Test İlkeleri

- **Motor bağlı değil:** M2–M10 testlerinde motor bağlı değildir. Motor yalnız M11 saha testinde.
- **Fake her şey:** RSCP module, H723, GPS, LiDAR, Jetson — hepsi fake ile test edilebilir.
- **Safety testleri kritik:** E-stop, watchdog, timeout — her milestone'da tekrarlanmalı.
- **Tek-yazıcı audit:** `/cmd_vel_safe`, H723 serial, RSCP serial — tek üretici kontrolü.
- **Jetson motor audit:** Jetson → motor yolu yok kontrolü.
- **Regression:** Her milestone sonrası tüm önceki testler tekrar.

---

## 11. Saha Kalibrasyon Planı

| Parametre | Test | Beklenen | Tolerans | Milestone |
|---|---|---|---|---|
| `effective_track_width` | 360° tank dönüş | Ölçülen | ±10° | M11 |
| `angular_correction_gain` | 90° tank dönüş | Ölçülen | ±5° | M11 |
| `left_motor_gain` | Düz sürüş drift | 1.0 | ±5cm/10m | M11 |
| `right_motor_gain` | Düz sürüş drift | 1.0 | ±5cm/10m | M11 |
| `wheel_radius` | Hall odom vs RTK | 0.125 | <2% | M11 |
| `gear_ratio` | Hall odom vs RTK | 1.0 | <2% | M11 |
| `max_motor_rpm` | Maksimum güvenli hız | TBD | — | M11 |
| IMU yaw bias | Tank dönüş vs IMU | 0° | ±2° | M11 |
| IMU yaw scaling | 360° vs IMU | 1.0 | ±1% | M11 |
| RTK datum | Sabit nokta tekrar | — | ±2cm (FIXED) | M11 |
| Arrival tolerance | Düşük hız varış | TBD | — | M11 |
| Kovaryans (odom) | Drift karakteristiği | TBD | — | M11 |
| Kovaryans (IMU) | Noise karakteristiği | TBD | — | M11 |
| Kovaryans (GPS) | RTK kalitesi | TBD | — | M11 |

**Kural:** Kalibrasyon parametreleri `vehicle.yaml`'da saklanır, kodda hardcode edilmez.

---

## 12. Risk Registeri

| Risk | Etki | Olasılık | İlgili milestone | Azaltma planı | Açık karar |
|---|---|---|---|---|---|
| RSCP Competition Module geç gelirse | Fake module'a bağımlı kalma | Yüksek | M2, M11 | Fake RSCP module ile geliştirme; sahada gerçek gerekli | CM ne zaman elimizde olacak? |
| `TaskFinished` / `TaskCompleted` isim karışıklığı | Yanlış mesaj adı kullanımı | Orta | M2, M7 | Proto doğruluk kaynağı; `TaskFinished` kullanılacak | — |
| H723 firmware değişiklik yetkısı | Gap'ler kapatılamaz (battery, binary frame, timeout) | Yüksek | M3 | Pi-tarafı defense-in-depth; gap listesi not; stakeholder ile görüşme | H723 firmware sahibi kim? |
| Battery telemetry eksikliği | RoverStatus battery_state boş kalır | Yüksek | M2, M3 | Geçici 0/NaN + hata log; H723'den gelene kadar | H723 battery eklenecek mi? |
| STM timeout süresinin uzun kalması (3000ms ≠ 300–500ms) | Pi-tarafı timeout defense-in-depth gerekir | Orta | M4 | Safety_mux komut timeout 300–500ms | H723 timeout kısaltılacak mı? |
| RTK FLOAT/SPS davranışı | Hedef onayı belirsiz | Orta | M6, M7 | Politika tanımla: FIXED→güvenilir, FLOAT→40cm, SPS→onay yok | RTK düzeltme kaynağı? |
| Skid-steer odometri kayması | Teker kayması → drift | Yüksek | M6, M11 | Hall odom + IMU füzyon; saha kalibrasyonu | Yüzey değişimi nasıl ele alınacak? |
| Jetson Nano performansı | Stereo + ArUco + derinlik aynı anda yavaş | Orta | M8 | Benchmark testleri; gerekirse çözünürlük/fps düşür | Jetson modeli kesin mi? |
| Jetson/Pi zaman senkronizasyonu | Topic timestamp uyuşmazlığı | Düşük | M8 | NTP/PTP senkronizasyonu | — |
| ArUco sözlüğü ve tag boyutu belirsizliği | Yanlış sözlük → tespit yok | Orta | M8 | `DICT_ARUCO_ORIGINAL` varsayım; ARC dokümanından doğrula | ARC resmi tag spec? |
| LiDAR modeli belirsizliği | Driver uyumsuzluğu | Orta | M5 | Model belirlenince driver araştırması | LiDAR modeli? |
| İz açıklığı merkez-merkez mi dıştan-dışa mı? | effective_track_width hesaplama | Düşük | M11 | Saha ölçümü ile doğrula | Fiziksel ölçüm gerekli |
| F411 non-latching fault otomatik resume | Beklenmedik motor hareketi | Yüksek | M4, M11 | Risk değerlendirme; gerekirse policy değişikliği | Stakeholder kararı |
| Hub gear_ratio 1.0 varsayım | Odom hatası | Orta | M11 | Hall odom vs RTK ile doğrula | Fiziksel doğrulama |
| Multi-host DDS interference | Jetson↔Pi iletişim kopması | Düşük | M8 | İzole DDS domain-id | — |
| SD kart storage wear | Log birikimi → kart bozulması | Düşük | M10 | Log rotasyonu, tmpfs | — |

---

## 13. Açık Teknik Sorular

| # | Soru | İlgili milestone | Durum | Not |
|---|---|---|---|---|
| 1 | İz açıklığı merkez-merkez (1.10 m) mi dıştan-dışa (~0.95 m) mi? | M11 | Açık | Fiziksel ölçüm gerekli |
| 2 | H723 firmware sahipliği + saha erişimi (timeout, binary frame, odometry, battery/status) | M3 | Açık, Pi-dışı | En büyük bilinmeyen |
| 3 | `RoverState` (DISARMED/AUTONOMOUS/MANUAL) ↔ H723 `RoverMode_t` (DISARM/MANUAL/AUTONOMOUS) eşleme | M2, M7 | Açık | Sıra farklı, safety kritik |
| 4 | Hub `gear_ratio` (1.0 varsayım) ve `hall_pulses=90` doğrulama | M11 | Açık | Fiziksel doğrulama |
| 5 | RTK düzeltme kaynağı (NTRIP/hücresel) | M5 | Açık | Hangi kaynak? |
| 6 | LiDAR modeli | M5 | Açık | Model belirlenmeli |
| 7 | Waveshare stereo modeli | M8 | Açık | Model belirlenmeli |
| 8 | ArUco sözlüğü (DICT_ARUCO_ORIGINAL varsayım) | M8 | Açık | ARC dokümanından doğrula |
| 9 | Battery telemetry (H723 yok → RoverStatus.battery_state nasıl doldurulacak?) | M2, M3 | Açık | Geçici 0/NaN |
| 10 | Competition Module elimizde olacak mı? | M11 | Açık | Fake ile geliştirme, sahada gerçek gerekli |
| 11 | H723 LINK_LOSS timeout kısaltılacak mı? (3000ms → 300–500ms) | M3 | Açık | Firmware değişikliği gerekir |
| 12 | F411 non-latching fault: otomatik resume riski nasıl ele alınacak? | M4, M11 | Açık | Stakeholder kararı |
| 13 | Max motor RPM? | M11 | Açık | Saha testinde belirlenecek |
| 14 | Nav2 controller seçimi: DWB mü MPPI mi? (skid-steer uyumu) | M7 | Açık | Benchmark gerekli |
| 15 | RTK datum: sabit nokta mı ilk fix mi? | M6 | Açık | Karar gerekli |
| 16 | Stereo kamera kalibrasyonu ne zaman yapılacak? | M8 | Açık | Jetson kurulumu sonrası |

---

## 14. Definition of Done

Proje aşağıdaki kriterlerin **tamamı** karşılandığında "done" sayılır:

### RSCP Entegrasyonu
- [ ] RSCP komutları (SetStage, ArmDisarm, NavigateToGPS, SearchArea, StartExploration) alınabilir
- [ ] RSCP yanıt-ları (Acknowledge, TaskFinished, GPSCoordinate, distance, RoverStatus) doğru üretilebilir
- [ ] COBS framing + protobuf serialize/deserialize güvenilir
- [ ] Bilinmeyen komut → crash yok, hata log + Acknowledge
- [ ] `TaskFinished` (proto doğru adı) kullanılıyor, `TaskCompleted` değil

### Görev Yönetimi
- [ ] Arm/disarm durum yönetimi güvenli (H723 RoverMode_t ile eşleme)
- [ ] 4 stage görev akışı test edilebilir (fake module ile)
- [ ] Mission manager RSCP komutlarını Nav2 hedeflerine dönüştürebilir
- [ ] Pause/resume/cancel/retry/timeout çalışır

### Navigasyon ve Otonomi
- [ ] Otonom GPS hedef navigasyonu (Nav2) çalışır
- [ ] SearchArea spiral/grid arama başlatılabilir
- [ ] RTK kalitesine bağlı hedef onayı çalışır
- [ ] GPS hiç yoksa otonom hareket başlatılmaz

### Görüş
- [ ] ArUco tag tespiti (tag i, j, k)
- [ ] LiDAR engel algılama + önleme
- [ ] Stereo derinlik/engel tespiti (Jetson)

### Güvenlik
- [ ] E-stop / watchdog çalışır (H723 LINK_LOSS, F411 CMD_WATCHDOG, Pi komut timeout)
- [ ] Safety zinciri tek-yazıcı audit geçer
- [ ] Jetson → motor komutu yok (CI audit)
- [ ] STM görev kararı vermez

### Donanım
- [ ] STM bridge (H723 UART ASCII) güvenilir
- [ ] 4 BLDC motor güvenli kontrol
- [ ] RTK GPS + LiDAR + STM IMU/mag + STM wheel odom entegre

### Deploy
- [ ] Cold boot systemd ile stack başlıyor
- [ ] Tekrarlanabilir install/build/run
- [ ] `rscp_protobuf` ve `cobs` import edilebilir

### Saha
- [ ] 4 stage tam akış testi geçmiş
- [ ] Saha kalibrasyonu tamamlanmış (`vehicle.yaml` TBD'leri dolu)
- [ ] 1m düz, 90° ve 360° tank dönüş testleri geçmiş
- [ ] E-stop + watchdog + timeout üçlü duruş testi geçmiş

---

## 15. Kapanış

**Kritik yol:** M1 → M2 → (M3 ∥ M4) → M5 → M6 → M7 → M11.

**En büyük risk:** H723 firmware sahipliği + saha erişimi + `RoverState` ↔ `RoverMode_t` arm/disarm güvenliği.

**RSCP merkezlidir.** Bu proje genel bir GPS waypoint rover değildir. RSCP protokolü ARC 2026'nın resmi ve zorunlu iletişim protokolüdür.

**Proto doğruluk kaynağı:** `rscp protokol/rscp/proto/rscp.proto`. README'deki `TaskCompleted` ifadeleri yanlıştır; gerçek mesaj `TaskFinished`'dir.
