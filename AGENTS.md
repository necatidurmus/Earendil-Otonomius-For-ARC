# AGENT.md — Coding Agent Çalışma Kuralları (Earendil RPi5 Vehicle)

> `AGENTS.md` bu dosyanın birebir aynıdır (iki isim de desteklenir). Ajan bir göreve başlarken
> önce `CLAUDE.md` (özet) → bu dosya (detay) → `GOALS.md` (hedefler) → `ROADMAP.md` (milestone) okur.

## 0. Çalışma Kuralları
- Her görevde okuma sırası: `CLAUDE.md` → `AGENT.md` → `GOALS.md` → `ROADMAP.md` → ilgili milestone → `rscp protokol/` → `stm codes/` → ilgili paket dizini.
- **Bu proje ARC 2026 RSCP kontrollü rover'dır** (bkz. §2, `GOALS.md`). RSCP merkezdedir; waypoint/teleop ikincildir.
- Değişiklik tek konu/tek dosya odaklı. Refactor ile davranış değişikliğini KARIŞTIRMA.
- Safety kuralını yumuşatma; watchdog'u/TIM1'i/safety'i "geçici olarak" kapatma; tespit edilen sorunu gizleme.
- Çalışma bir STM firmware dosyasına değiyorsa: `stm codes/<proje>/AGENTS.md` + ilgili `docs/SAFETY.md`/`docs/TIM1_GATE_DRIVE.md` de okunur; o projenin kuralları geçerli.
- Kullanıcı bilgi/doküman istiyorsa yalnız oku/raporla; kod/değişiklik yapma.
- Build aracı yoksa (PlatformIO, colcon) uydurma komut; durumu raporla.

## 1. RSCP — Merkezi Protokol (önce bunu anla)
RSCP = Rover Serial Communication Protocol, ARC'ın resmi protokolü. Resmi repo: `https://github.com/anatolianroverchallenge/rscp`. Yerel klon: **`rscp protokol/rscp/`** (dizin adında boşluk var). Pi'deki `earendil_rscp_bridge` paketi Competition Module (CM) ile seri konuşur; resmi `.proto` + üretilmiş protobuf sınıfları kullanılır — **manuel serialize/deserialize YAZILMAZ**.

### 1.1 RSCP Repo İnceleme Prosedürü (RSCP bridge dokünmadan önce ZORUNLU)
1. `rscp protokol/rscp/proto/rscp.proto` — **bütünüyle oku**. Bu tek doğruluk kaynağıdır. Hangi mesajın hangi `oneof` alanına ait olduğunu buradan çıkar (aşağıdaki §1.2).
2. `rscp protokol/rscp/README.md` — frame formatı, COBS(`0x00` sınırlayıcı), 4 stage iletişim akışı (HM→CM→Rover). **Dikkat:** README'deki akış tabloları `\TaskCompleted` yazar ama proto'daki gerçek mesaj **`TaskFinished`**'tır. Daima proto'yu esas al; README'yi takma ad olarak not et.
3. `examples/python/receive_commands.py` + `receive_commands_noserial.py` — alma idiomu: byte byte oku, `0x00` gelince `cobs.cobs.decode(buffer)` → `RequestEnvelope().ParseFromString(decoded)` → `request.WhichOneof('request')`.
4. `examples/python/send_string_message.py` + `send_rover_status_message.py` — gönderme idiomu: `response.SerializeToString()` → `cobs.cobs.encode(bytes)` → `+ b"\x00"` → `serial_port.write(...)`. Alt-mesajlar için `.CopyFrom(...)`; skalar alt alanlar (`rover_status.battery_state.voltage = ...`) doğrudan atanabilir.
5. `examples/aruco_detection_example.py` — ArUco referansı: `cv2.aruco.DICT_ARUCO_ORIGINAL` + `ArucoDetector`. Bu Jetson tarafı içindir, bridge değil.
6. Release sınıfları: `pip3 install https://github.com/anatolianroverchallenge/rscp/releases/latest/download/rscp_protobuf.zip` → `import rscp_protobuf`. C/C++ için `cpp.zip`/`nanopb.zip`; MCU tarafı nanopb. Python kullanımı birincil.

### 1.2 Proto Doğrulama Listesi (bridge yazmadan doğrula)
`rscp.proto`'dan çıkarılan **kesin** yapı (değişirse proto'yu güncelle, dokümanı tahminle yazma):
- **RequestEnvelope** `oneof request`: `arm_disarm`(ArmDisarm), `set_stage`(SetStage), `navigate_to_gps`(NavigateToGPS), `search_area`(SearchArea), `start_exploration`(StartExploration).
- **ArmDisarm**: `oneof value_wrapper { bool value = 1; }` — `true`=arm, `false`=disarm. (value_wrapper sarmalayıcısına dikkat.) **Güvenlik kritik** → §4 ile nasıl kullanılacağını belirle.
- **SetStage**: `uint32 value` (1=Antenna, 2=Shackleton, 3=Lava Tube, 4=Airlock).
- **NavigateToGPS**: `GPSCoordinate coordinate`.
- **SearchArea**: `GPSCoordinate center_coordinate` + `float radius` (metre).
- **StartExploration**: `bool dummy_field` (payload yok, sade tetik).
- **ResponseEnvelope** `oneof response`: `acknowledge`(Acknowledge), `task_finished`(TaskFinished), `gps_coordinate`(GPSCoordinate), `distance`(double), `message`(string), `rover_status`(RoverStatus).
- **GPSCoordinate**: `double latitude`, `double longitude` (derece), `float altitude` (metre, **EGM96** geoid).
- **BatteryState**: `float voltage`, `float current`, `float state_of_charge` (0..1). → H723'de battery telemetry YOK (gap, bkz. §5); `rover_status.battery_state` dolmadan gönderilemez.
- **RoverStatus** (≤1 Hz): `RoverState state` + `GPSCoordinate coordinate` + `float heading` + `BatteryState battery_state`.
- **RoverState enum**: `DISARMED=0, AUTONOMOUS=1, MANUAL=2`. **Sıra H723 `RoverMode_t` (DISARM/MANUAL/AUTONOMOUS) ile farklı** → RSCP↔STM arası haritalama gerekli (§4).

## 2. RSCP Bridge Tasarım Kuralları
- **Tek sorumluluk:** `earendil_rscp_bridge` = seri + COBS + protobuf. Görev komutunu → `/rscp/command` + `/mission/command`'a çevirir; `/mission/status` + `/mission/result` + `/rtk/status` + `/gps/fix`'ten rover durumu toplar → `ResponseEnvelope` üretir.
- **Motor komutu ÜRETMEZ.** `/cmd_vel_*`'e hiçbir RSCP node'u yazmaz. Hareket: RSCP komutu → mission manager → Nav2 goal → `/cmd_vel_nav` → safety_mux → `/cmd_vel_safe` → stm_bridge.
- **Ser./deser. elle yazılmaz:** yalnız `rscp_protobuf` + `cobs` kullan. `.proto` değişirse resmi repo'dan yeni release çek; branch/patch ile sapma.
- **`WhichOneof('request')` ile dağıt**; `None`/bilinmeyen tür → `Acknowledge`/hata `message` ile cevap, çökme yok.
- **`ArmDisarm` güvenlik bağı:** `arm=true` H723'i AUTONOMOUS'a geçiş için sinyaller (safety_mux/M3 arm gate ile uyumlu); `arm=false` → DISARM + motor stop. H723'e bu geçiş yalnız stm_bridge üzerinden resmi komutla yapılır.
- **Test:** fake/recorded RSCP module (ör. `examples/` byte akışından) ile framing+parse+dispatch testi. **Motor bağlı değil.** Serial yoksa `receive_commands_noserial.py` tarzı `io.BytesIO` ile.
- COBS framing: `0x00` sınırlayıcı, partial frame buffer, decode hatası → frame at + log, crash yok.

### 2.1 RequestEnvelope → Mission/Olay Eşlemesi (tasarım kontratı)
| RSCP komut | Bridge aksiyonu | Sonraki RSCP yanıtı |
|---|---|---|
| `set_stage(v)` | `/rscp/current_stage` + `/rscp/command` → mission manager stage ayarı | `Acknowledge` |
| `arm_disarm(true/false)` | arm gate sinyali (safety_mux) + `/rscp/command` | `Acknowledge` (arm durumu `RoverStatus` ile de raporlanır) |
| `navigate_to_gps(coord)` | `/mission/command` (Nav2 goal, RTK onaylı) | `Acknowledge` → (varışta) `TaskFinished` |
| `search_area(center,r)` | `/mission/command` (arama alanı görevi) | `Acknowledge` → (bulunca) `GPSCoordinate` → `TaskFinished` |
| `start_exploration` | `/mission/command` (keşif modu; Jetson+LiDAR+odom) | `Acknowledge` → (ölçünce) `distance` → (çıkış tag)i `TaskFinished` |

### 2.2 ResponseEnvelope ← Rover Durumu Eşlemesi (tasarım kontratı)
| Yanıt | Kaynak | Ne zaman |
|---|---|---|
| `acknowledge` | komut alındı | her `RequestEnvelope` sonrası |
| `task_finished` | `/mission/result` | bir görev/alt-görev bitti (varış, arama, keşif, dock) |
| `gps_coordinate` | `/gps/fix` (RTK onaylı; §6 eşiklere uy) | target bulundu (antenna tepe, bazalt) |
| `distance` | `/stm/wheel_odom` integrasyonu (lav tüpü keşfi boyunca kat edilen) | StartExploration alt-görevi ölçümü |
| `message` | hata/durum metni | opsiyonel debug/hata |
| `rover_status` | `/rtk/status`+`/gps/fix`+`/stm/imu/data`(heading)+ battery | periyodik ≤1 Hz |

## 3. Safety Zincirini BOZMAMA Kuralları (RSCP + motor)
- `/cmd_vel_{nav,manual}` → safety_mux → `/cmd_vel_safe` → stm_bridge → H723 → F411 dışında motor komut yolu YOK.
- **RSCP bridge `/cmd_vel_safe`'e YAZAMAZ; `/cmd_vel_nav`'a da yazmaz** (Nav2 yazar). RSCP yalnız mission manager'a hedef verir. (CI audit.)
- Web node `/cmd_vel_manual` yazar; `/cmd_vel_safe`'e **YAZAMAZ**. Jetson node'larının motor topic'lerine publish/publish yolu **yok** (CI audit).
- E-stop/deadman/timeout/watchdog gate'lerini atlayan kestirme yol ekleme. Öncelik: e-stop > deadman > watchdog > komut timeout (300–500 ms).
- **GPS hiç yoksa otonom hareket başlatılmaz.** RTK onayı (CLAUDE §8): FIXED→güvenilir varış; FLOAT/DGPS→40 cm tolerans; SPS/NO_FIX→**varış onayı yok**. `TaskFinished`/`GPSCoordinate` kaliteye bağlı gönder.
- F411 TIM1 gate-drive hot-path (`motor_driver.c`: CCxE/CCxNE, dead-time, allOff): değişiklikte `docs/TIM1_GATE_DRIVE.md` güncelle + CCxE/CCxNE etkisini açıkla. F411 break'i (BKIN) fiziksel pin olmadan re-enable etme. Watchdog/safety'leri "test geçsin" diye disable etme.

## 4. STM'den Çıkarılacak Bilgiler (ince, kopyalama)
- **H723 (`earendil-mainfirmware/`):** 4 motor UART (USART2=FL, UART4=FR, UART7=RL, UART5=RR), host=USART3 ASCII `<cmd>\r\n`, `ACK_TIMEOUT_MS=500`, `MAX_RETRIES=3`, `LINK_LOSS_TIMEOUT_MS=3000`, MPU9250 + QMC5883P, mode DISARM/MANUAL/AUTONOMOUS, 14 motor fault, arc-turn `turn_ratio` permille. **GAP:** CRC/seq/timestamp yok; odometry/pose yok; battery/status/ENV yok (placeholder); timeout 3000 ms ≠ hedef 300–500; cmd_vel değil RPM/duty+direction+turn-ratio. Ayrıca: `RoverState`↔`RoverMode_t` haritalama ihtiyacı (RSCP arm/disarm yolunun güvenliği).
- **F411 (`earendilmotorcontroller/f411-motor-cube/`):** `POLE_PAIRS=15` ⇒ 90 Hall edge/devir; `CMD_WATCHDOG_MS`, `HOST_DISCONNECT_TIMEOUT_MS=2000`; faults non-latching; akım sensörü YOK; telemetri `RPM:...,T:...,D:...,DIR:...,APP_PH:...,SP:...,BRAKE:...,FC:...,H:...,PWM_SET:...,PWM_ACT:...,QDROP:...,RXB:...`. H7 forward `FL|.../FR|.../RL|.../RR|...` prefix.
- **`earendil.py`** = PC-side PySide6 debug GUI (H723'a USB-serial, ROS-dışı); `earendil_tools/` karantinasında; `earendil_web` değil.
- **`manipulation_roadmap.md`** = robot-kol UART8 planı; **scope dışı** (drive motorları ile karıştırma).

## 5. `eski proje/`den Fikirler (dondurulmuş referans)
Mission lifecycle (load/start/pause/resume/cancel/reset/retry/skip-on-stuck); waypoint yaşam döngüsü (`/ml_waypoint`/`/ml_mission`/`/ml_control`/`/ml_status`); parametre preset (Slow/Normal/Fast) + sweep + `ros2 param set`; mission report (path error/RMSE, hız profili, jerk); GPS↔SLAM mode geçişi (`/nav_mode`); RTK datum (`/fromLL`, EGM96); E-stop'ta `/cmd_vel_nav`'e sıfır basma (defense-in-depth); web teleop fikirleri; JSON status topic; waypoint güvenlik uyarısı. **MATLAB/Gazebo/Docker/X11/`tunnel_gps_spoofer` taşınmaz** (CLAUDE §11).

## 6. Kod Yazmadan Önce Kontrol Listesi
- [ ] İlgili milestone `ROADMAP.md`'de tanımlı; ajan-oluşturma/davranış-bü-yüme aşaması dışındaysa kullanıcıya doğrula.
- [ ] RSCP bridge ise: `rscp.proto` + python examples okundu, mesaj alanları doğrulandı (§1.2).
- [ ] Hedef dosya/paket tanımlı; değişiklik tek konu; arm/disarm/motor yolu safety zincirini esnetmiyor (§3).
- [ ] Kalibrasyon parametresi hardcode ETMİYORSUN (CLAUDE §9, §15).
- [ ] MATLAB/Gazebo/Docker bağımlılığı eklemiyorsun (CLAUDE §11).
- [ ] Topic adı `CLAUDE.md` §7 ile uyumlu.
- [ ] Çalışan kod gerekiyorsa: interface/yaml/launch iskeleti önce var mı?
- [ ] Firmware dokunuyorsan: `stm codes/<proje>/AGENTS.md` + `docs/SAFETY.md` okundu.

## 7. Dosya Değiştirme / Yeni Paket Kuralları
- Tek-yazıcı: yalnız `safety_mux` `/cmd_vel_safe`'e yazar; yalnız `stm_bridge` H723'a yazar; yalnız `earendil_rscp_bridge` CM'ye yazar.
- Parametreler `*_params.yaml`'da; kod `declare_parameter` + varsayılan + config. Hardcoded sayısal sabit YOK (firmware sabitleri hariç).
- `vehicle.yaml`: `wheel_base_m=0.825`, `track_width_m=1.10` (`# TBD – merkez/dış`), `wheel_radius_m=0.125`, `wheel_circumference_m=~0.785`, `effective_track_width_m` (TBD), `angular_correction_gain` (TBD), `left/right_motor_gain` (TBD), `hall_pulses_per_motor_rev=90`, `gear_ratio=1.0` (TBD), `motor_pole_pairs=15`, `max_motor_rpm` (TBD).
- Launch: `use_hardware`/`use_sim_time` flag'leri; native araçta `use_hardware:=true`, `use_sim_time:=false` default.
- Yeni paket yalnız 11-paket setinde (eksikse önce ROADMAP'e/kullanıcıya ekle). Her paket: `package.xml`, `CMakeLists.txt`/`setup.py`, `README.md`, `config/*_params.yaml`, `launch/`. `earendil_interfaces` `msg/`/`srv/`/`action/` + enum (MotorId, MissionStage).
- RSCP için yeni `*.proto` üretme; resmi release kullan. RSCP mesajları protobuf ile taşınır — ROS msg'ye gerek yok (RSCP bridge iç/ara topic'ler `earendil_interfaces`'ten).
- Build artifact commit ETME: `build/`, `install/`, `log/`, `.pio/`, `*.elf`, `*.bin`, `*.o`, `*.map`.

## 8. Kabul Kriterleri (job done)
- [ ] `colcon build --symlink-install` temiz. RSCP için `rscp_protobuf`/`cobs` import edilebilir.
- [ ] Değişen davranış birim testi/launch veya fake-module testi ile doğrulandı (mümkünse).
- [ ] Safety tek-yazıcı audit geçer (`/cmd_vel_safe` + H723 serial + RSCP serial tek üretici).
- [ ] RSCP tarafında manuel ser./deser. YOK; resmi `.proto`/sınıflar kullanıldı.
- [ ] Hiçbir kalibrasyon parametresi hardcode edilmedi.
- [ ] MATLAB/Gazebo/Docker bağımlılığı eklenmedi; Jetson→motor yolu yok; STM görev kararı vermiyor.
- [ ] Firmware değiştiyse `pio run -d <module>` temiz (PlatformIO yoksa rapor).
- [ ] Commit artifact'leri dahil değil; `ROADMAP.md` ile çelişen adım atılmadı.
- [ ] Kullanıcı bilgi/doküman istiyorsa: yalnız oku/raporla, değişiklik yok.

## 9. İlk Repo İnceleme Adımları (yeni ajan)
1. Kök: `ls`; `README.md`, `GOALS.md`, `ROADMAP.md`, `CLAUDE.md`, bu `AGENT.md`, `.env.example`.
2. `rscp protokol/rscp/` → §1.1 prosedürü: `proto/rscp.proto`, `README.md`, `examples/python/`, `examples/aruco_detection_example.py`.
3. `src/` → 11 paket var mı? (`earendil_rscp_bridge` dahil). Eksikse M1 iskelet açığı.
4. `deploy/` → `install_rpi5_ubuntu24.sh`, `build.sh`, `run_vehicle.sh`, `earendil-vehicle.service`, udev.
5. `stm codes/` → iki firmware: `earendil-mainfirmware` (H723) + `earendilmotorcontroller` (F411×4).
6. `eski proje/` → dondurulmuş MATLAB/Gazebo/Leo referansı. Hiçbir dosyasını yeni yola kopyalama.
7. `src/earendil_control/control_params.yaml` (varsa) → eski `wheel_base=0.38`/`wheel_radius=0.0625` HATALI; doğrusu `0.825`/`0.125` (CLAUDE §9). Açık işaretle.

## 10. YAPMAMA
- RSCP'yi opsiyonel/debug gibi gösterme; sistemi sadece waypoint projesi gibi anlatma.
- RSCP tarafında manuel serialize/deserialize yazma; `.proto` dışı mesaj uydurma.
- RSCP bridge → motor komutu üretme; safety zincirini bypass eden yol tanımlama.
- Jetson'u ana karar bilgisayarı / kamera-IMU'yu ana IMU / web teleop'u ana görev kontrolü gibi gösterme.
- MATLAB/Gazebo/Docker eski projeyi yeni ana mimariye taşıma.
- Saha kalibrasyon parametresini hardcode etme; Docker'ı ana deploy yapma.
- TIM1 gate-drive hot-path kurallarını esnetme; watchdog/safety'leri "test geçsin" diye devre dışı bırakma; sorunu gizleme.
- `earendil.py`'yi `tools/` karantinasından çıkarma.

> Özet kontrat: `CLAUDE.md`. Hedefler: `GOALS.md`. Milestone planı: `ROADMAP.md`.
