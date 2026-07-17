# GOALS.md — Earendil RPi5 Vehicle Proje Hedefleri

> Bu dosya projenin nihai hedeflerini, başarı kriterlerini ve kapsam dışı bırakılan unsurları tanımlar.
> Ajan bir göreve başlarken `CLAUDE.md` (özet) → `AGENT.md` (detay) → bu dosya (hedefler) → `ROADMAP.md` (milestone) okur.

## 1. Nihai Hedef

**Anatolian Rover Challenge (ARC) 2026** yarışmasında, 4 stage'li otonom görevi başarıyla tamamlayan, **RSCP protokolüyle komut alan**, Raspberry Pi 5 + ROS 2 Jazzy (native Ubuntu 24.04) üzerinde çalışan **gerçek rover üst seviye kontrol sistemi** geliştirmek.

Proje sadece genel bir GPS waypoint rover **değildir**. Ana hedef: ARC Competition Module'dan RSCP komutlarını güvenilir şekilde almak, görevleri otonom olarak yürütmek, ve sonuçları RSCP protokolüyle geri bildirmek.

---

## 2. Yarışma Bağlamı

**Anatolian Rover Challenge (ARC) 2026** — üniversite öğrencileri arası otonom rover yarışması.
- **Konum:** Türkiye
- **Protokol:** RSCP (Rover Serial Communication Protocol) — resmi, zorunlu
- **RSCP Kaynağı:** https://github.com/anatolianroverchallenge/rscp (`rscp_protobuf` + `cobs` Python paketi)
- **Görev:** 4 stage (Antenna Installation, Shackleton Crater, Lava Tube, Return to Airlock)
- **İletişim:** Rover ↔ Competition Module (CM) arası seri, COBS-framed protobuf

---

## 3. Başarı Kriterleri

Aşağıdaki kriterlerin **tamamı** karşılandığında proje başarılı sayılır:

### RSCP Entegrasyonu
- [ ] RSCP komutları (SetStage, ArmDisarm, NavigateToGPS, SearchArea, StartExploration) alınabilir
- [ ] RSCP yanıtları (Acknowledge, TaskFinished, GPSCoordinate, distance, RoverStatus) doğru üretilebilir
- [ ] COBS framing + protobuf serialize/deserialize güvenilir çalışır
- [ ] Bilinmeyen komut → crash yok, hata log + Acknowledge

### Görev Yönetimi
- [ ] Arm/disarm durum yönetimi güvenli (H723 RoverMode_t ile eşleme)
- [ ] Stage-based görev akışı (4 stage) test edilebilir
- [ ] Mission manager, RSCP komutlarını Nav2 hedeflerine dönüştürebilir

### Navigasyon ve Otonomi
- [ ] Otonom GPS hedef navigasyonu (Nav2) çalışır
- [ ] SearchArea görevi başlatılabilir (spiral/grid arama)
- [ ] RTK kalitesine bağlı hedef onayı (FIXED → güvenilir, FLOAT/DGPS → 40cm tolerans, SPS/NO_FIX → onay yok)
- [ ] GPS hiç yoksa otonom hareket başlatılmaz

### Görüş ve Engel Algılama
- [ ] ArUco tag tespiti (tag i, j, k — DICT_ARUCO_ORIGINAL)
- [ ] LiDAR engel algılama + önleme
- [ ] Stereo derinlik/engel tespiti (Jetson)

### Güvenlik
- [ ] E-stop / watchdog çalışır (H723 LINK_LOSS, F411 CMD_WATCHDOG, Pi komut timeout)
- [ ] Safety zinciri tek-yazıcı audit geçer
- [ ] Jetson → motor komutu yok (CI audit)
- [ ] STM görev kararı vermez (düşük seviye kontrolcü)

### Donanım Entegrasyonu
- [ ] STM bridge (H723 UART ASCII) güvenilir çalışır
- [ ] 4 BLDC motor güvenli kontrol (F411×4)
- [ ] RTK GPS + LiDAR + STM IMU/mag + STM wheel odom entegre

---

## 4. Kapsam Dışı Bırakılan Unsurlar

Aşağıdaki unsurlar **yeni mimariye taşınmaz**, yalnız eski referans olarak kalır:

| Unsur | Neden kapsam dışı |
|---|---|
| MATLAB + App Designer + ROS Toolbox | Gerçek araç bağımlılığı yok, simülasyon odaklı |
| Gazebo/Ignition (`leo_*`/`clearpath_*` SDF) | Simülasyon, gerçek donanım değil |
| Docker/Docker Compose ana deploy | Native systemd birincil, Docker yalnız dev/test |
| NVIDIA GPU + X11 | Jetson ayrı, Pi'de GPU gerektiren iş yok |
| `leo_robot`/`leo_simulator`/`leo_common` | Eski platform bağımlılığı |
| `tunnel_gps_spoofer.py`/`/gps_spoofer/control` | Fake GPS, gerçek sistemde yeri yok |
| `use_sim_time=True` default | Gerçek donanımda `false` |
| Gazebo DiffDrive `/odom` | Simülasyon topic'i |
| `earendil_sim` paketi | Opsiyonel simülasyon, ana sistemden bağımsız |
| Manuel RSCP serialize/deserialize | Resmi `rscp_protobuf` + `cobs` kullanılacak |

---

## 5. Ana Görev Senaryoları

### Senaryo 1: Stage 1 — Antenna Installation
```
CM → SetStage(1) → Rover Ack
CM → ArmDisarm(true) → Rover Ack (H723 AUTONOMOUS moduna geçer)
CM → SearchArea(lat1,lon1,radius1) → Rover Ack
Rover: alanda spiral/grid arama, zirveyi bulur
Rover → GPSCoordinate(lat2,lon2) [tepe konumu]
Rover → TaskFinished
```

### Senaryo 2: Stage 2 — Shackleton Crater
```
CM → SetStage(2) → Rover Ack
CM → SearchArea(lat3,lon3,radius3) → Rover Ack
Rover: alanda arama, koyu ilmenit-bazalt kaya bulur
Rover → GPSCoordinate(lat4,lon4) [kaya konumu]
Rover → TaskFinished
```

### Senaryo 3: Stage 3 — Lava Tube
```
CM → SetStage(3) → Rover Ack
CM → NavigateToGPS(lat5,lon5) [lav tüpü girişi] → Rover Ack
Rover: gider, ArUco tag i tespit eder
Rover → TaskFinished [giriş ulaşıldı]
CM → StartExploration → Rover Ack
Rover: lav tüpünü keşfeder, mesafe ölçer (wheel odom integrasyonu)
Rover → distance [ölçülen mesafe]
Rover: ArUco tag j (çıkış) tespit eder, tüpten çıkar
Rover → TaskFinished
```

### Senaryo 4: Stage 4 — Return to Airlock
```
CM → SetStage(4) → Rover Ack
CM → NavigateToGPS(lat6,lon6) [airlock] → Rover Ack
Rover: gider, ArUco tag k tespit eder, dock eder
Rover → TaskFinished [airlock ulaşıldı]
CM → ArmDisarm(false) → Rover Ack (H723 DISARM moduna geçer, motorlar durur)
Görev tamamlandı.
```

---

## 6. Kabul Kriterleri (Milestone Bazında)

| Milestone | Temel Kabul Kriteri |
|---|---|
| M1 | 11-paket iskeleti `colcon build` temiz; RSCP proto doğrulandı |
| M2 | Fake RSCP module ile tüm komut türleri parse+dispatch; motor bağlı değil |
| M3 | Loopback/fake-H723 ile 4-motor RPM komutu + telemetri parse |
| M4 | Her safety gate deterministik sıfır; tek-yazıcı audit geçer |
| M5 | `/gps/fix` RTK_FIXED; `/scan` dolu; TF geçerli |
| M6 | Kısa mesafe drift sınırlı; RTK map↔gps tutarlı |
| M7 | RSCP stage akışı fake module ile test edilebilir; Nav2 goal navigasyonu çalışır |
| M8 | ArUco tespit edilebilir; Jetson→motor yolu yok (audit) |
| M9 | Dashboard durum gösteriyor; deadman teleop çalışıyor; e-stop sıfırlıyor |
| M10 | Cold boot systemd ile stack başlıyor; `rscp_protobuf` import edilebilir |
| M11 | Tüm 4 stage tam akış testi; saha kalibrasyonu tamamlandı |

---

## 7. Açık Teknik Sorular

Aşağıdaki sorular henüz yanıtlanmamış olup, milestone'lar boyunca çözülecek:

| Soru | İlgili Milestone | Durum |
|---|---|---|
| İz açıklığı merkez-merkez (1.10 m) mi dıştan-dışa (~0.95 m) mi? | M11 | Açık |
| H723 firmware sahipliği + saha erişimi (timeout, binary frame, odometry, battery/status) | M3 | Açık, Pi-dışı |
| `RoverState` (DISARMED/AUTONOMOUS/MANUAL) ↔ H723 `RoverMode_t` eşleme | M2/M7 | Açık, safety kritik |
| Hub `gear_ratio` (1.0 varsayım) ve `hall_pulses=90` doğrulama | M11 | Açık |
| RTK düzeltme kaynağı (NTRIP/hücresel) | M5 | Açık |
| LiDAR modeli | M5 | Açık |
| Waveshare stereo modeli | M8 | Açık |
| ArUco sözlüğü (DICT_ARUCO_ORiginal varsayım) | M8 | Açık |
| Battery telemetry (H723 yok → RSCP `RoverStatus.battery_state` nasıl doldurulacak?) | M2/M3 | Açık |
| Competition Module elimizde olacak mı? (fake ile geliştirme, sahada gerçek gerekli) | M11 | Açık |

---

## 8. Proje Değerleri

- **Güvenlik her şeyden önce:** Safety zinciri hiçbir koşulda bypass edilmez. Watchdog/safety "test geçsin" diye devre dışı bırakılmaz. Sorun gizlenmez.
- **Gerçek donanım, gerçek protokol:** Simülasyon değil, RSCP değil opsiyonel — ARC 2026'ya gerçek araçla katılacağız.
- **Resmi kaynaklar:** RSCP'de manuel serialize/deserialize yok, resmi `.proto` + `rscp_protobuf`. STM'de `stm codes/` referans alınır, kopyalanmaz.
- **Tekrarlanabilir deploy:** Native ROS 2 Jazzy + systemd. Docker yalnız geliştirme/test.
- **Kalibrasyon hardcode edilmez:** Saha kalibrasyon parametreleri TBD olarak kalır, M11'de sahada doldurulur.
