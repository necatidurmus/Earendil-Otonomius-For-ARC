# Geliştirme Roadmap — Earendil Otonomius For ARC

**Son Güncelleme:** 2025  
**Platform:** ROS 2 Humble | Simulation-first

---

## Faz Genel Bakışı

```
Faz 0 ──► Faz 1 ──► Faz 2 ──► Faz 3 ──► Faz 4
 Altyapı   Core     ARC       Entegras-  Gerçek
 & İskelet Fonk.    Görevler  yon & Test Rover
(şimdi)   (1-2 ay) (2-3 ay)  (1-2 ay)  (ileride)
```

---

## Faz 0 — Altyapı ve İskelet ✅ (Mevcut)

**Hedef:** Sağlam repo yapısı, tüm paket iskeletleri, dokümantasyon.

**Çıktılar:**
- [x] Monorepo klasör yapısı kuruldu
- [x] Tüm ROS 2 paket iskeletleri oluşturuldu
- [x] `sim_config.yaml` — merkezi konfigürasyon
- [x] `Dockerfile` + `docker-compose.yml`
- [x] `docs/` — mimari, roadmap, test stratejisi
- [x] `.github/workflows/ci.yml` — CI iskelet
- [x] `earendil_msgs` — custom mesaj tanımları
- [x] Branch stratejisi ve commit convention belirlendi

**Baz Alınan:**
- Earendil-Otonomius v0.2 — Dual-UKF GPS+SLAM (9/9 waypoint ✅)
- `mission_manager.py`, `gps_monitor.py`, `tf_mode_relay.py`, `tunnel_gps_spoofer.py` taşındı ve yeniden yapılandırıldı

---

## Faz 1 — Core Fonksiyonlar (Öncelikli)

**Hedef:** Temel simülasyon çalışır, basit görev döngüsü tamamlanır.

### 1.1 Simülasyon Altyapısı
- [ ] `earendil_simulation`: ARC arena Gazebo dünyası (`arc_arena.sdf`)
- [ ] Tünel bölgesi olan dünya (`arc_tunnel.sdf`)
- [ ] GPS spoofer tünel desteği (mevcut `tunnel_gps_spoofer.py` taşıması)
- [ ] `sim_bringup.launch.py` — tek komutla tam simülasyon stack'i

### 1.2 Robot Tanımı
- [ ] `earendil_description`: URDF/xacro rover modeli
- [ ] LiDAR, IMU, GPS, kamera sensor tanımları
- [ ] RViz konfigürasyonu

### 1.3 Lokalizasyon
- [ ] `earendil_localization`: UKF Local + UKF Global (mevcut config'den)
- [ ] GPS Monitor node (mevcut `gps_monitor.py` modüler hale getirildi)
- [ ] TF Mode Relay node (mevcut `tf_mode_relay.py` modüler)
- [ ] SLAM Toolbox entegrasyonu
- [ ] `localization.launch.py`

### 1.4 Navigasyon
- [ ] `earendil_navigation`: Nav2 params (mevcut `nav2_params_hybrid.yaml` taşıması)
- [ ] Waypoint manager stub
- [ ] `navigation.launch.py`

### 1.5 Safety Supervisor (Temel)
- [ ] `earendil_safety`: cmd_vel watchdog
- [ ] E-stop topic desteği
- [ ] Tilt limiti (IMU'dan)

**Başarı Kriteri:**
> Simülasyonda rover GPS modunda 3 waypoint'e ulaşır, SLAM moduna geçer, tünel geçer, GPS moduna döner.

---

## Faz 2 — ARC Görev Mantığı

**Hedef:** ARC yarışmasına özgü görevler state machine ile çalışır.

### 2.1 Mission State Machine
- [ ] `earendil_mission`: tam state machine implementasyonu
- [ ] RSCP komut akışı (`START`, `STOP`, `PAUSE`, `E_STOP`)
- [ ] Stage-based mission orchestration
- [ ] Görev tamamlama kriterleri

### 2.2 Task Executor — ArUco
- [ ] `earendil_perception`: ArUco marker tespit node'u
- [ ] Marker'a yaklaşma + duruş mantığı
- [ ] Birden fazla marker arama stratejisi

### 2.3 Task Executor — Rock Detection
- [ ] Kaya/engel tespit stub → temel implementasyon
- [ ] LiDAR tabanlı engel sınıflandırma

### 2.4 Task Executor — Peak Search
- [ ] Elevation map tabanlı tepe arama (stub → implementasyon)
- [ ] Hepe noktasına navigasyon

### 2.5 Task Executor — Tunnel Traversal
- [ ] Tünel giriş/çıkış tespiti
- [ ] SLAM mod geçişi tetikleyici
- [ ] Dar geçit navigasyonu (costmap parametresi)

### 2.6 RSCP Bridge
- [ ] `earendil_rscp`: WebSocket tabanlı köprü
- [ ] Telemetri yayını (konum, görev durumu, batarya)
- [ ] Bağlantı kesilme güvenli durumu

**Başarı Kriteri:**
> Simülasyonda RSCP START komutu ile görev başlar: ArUco bulunur, tünel geçilir, tepe noktası belirlenir, COMPLETE ile biter.

---

## Faz 3 — Entegrasyon ve Test

**Hedef:** Tüm katmanlar birlikte çalışır, CI yeşil, test coverage %60+.

### 3.1 Tam Entegrasyon Testi
- [ ] `run_arc_full_test.sh` — tek komutla tam ARC görev simülasyonu
- [ ] Birden fazla ARC görevi sıralı testi
- [ ] Hata senaryoları testi (GPS kaybı, sensor timeout, e-stop)

### 3.2 Safety Supervisor Tamamlama
- [ ] Batarya simülasyonu ve düşük batarya testi
- [ ] Çoklu e-stop kaynağı (RSCP + yerel)
- [ ] Recovery davranışı

### 3.3 Teleop Tamamlama
- [ ] Joystick manual override
- [ ] Web teleop (opsiyonel)
- [ ] Otonom ↔ Manuel geçiş kesintisiz

### 3.4 Logging Sistemi
- [ ] Görev başında otomatik rosbag kaydı
- [ ] Event log (görev geçişleri, hatalar)
- [ ] Replay altyapısı testi

### 3.5 CI/CD
- [ ] `colcon build` CI'da çalışıyor
- [ ] Unit testler CI'da çalışıyor
- [ ] Lint (ament_flake8, ament_pep257) CI'da geçiyor

**Başarı Kriteri:**
> CI her PR'da yeşil. Tam ARC simülasyon testi < 20 dakika tamamlanıyor.

---

## Faz 4 — Gerçek Rover Taşınması (İleride)

**Hedef:** Aynı yazılım gerçek Jetson Orin Nano üzerinde çalışır.

### 4.1 Donanım Sürücüleri
- [ ] GPS sürücüsü (U-blox vb.)
- [ ] IMU sürücüsü (BNO055 / ICM-42688)
- [ ] LiDAR sürücüsü
- [ ] Motor kontrol sürücüsü
- [ ] Kamera sürücüsü

### 4.2 Real Robot Bringup
- [ ] `robot_bringup.launch.py` — gerçek donanım için
- [ ] Sensor kalibrasyon scripti
- [ ] URDF gerçek rover ile eşleştirme

### 4.3 Jetson Optimizasyonu
- [ ] Isaac ROS entegrasyonu (algılama için GPU hızlandırma)
- [ ] SLAM harita kaydetme/yükleme
- [ ] Düşük bant genişliği RSCP (UDP üzerinden)

### 4.4 Saha Testi
- [ ] ARC arena benzeri gerçek ortam testi
- [ ] GPS/SLAM geçiş gerçek koşullar testi
- [ ] Yarışma öncesi tam rehearsal

---

## Milestone Özeti

| Faz | Süre (tahmini) | Durum |
|-----|----------------|-------|
| Faz 0 — Altyapı | 1 hafta | ✅ Tamamlandı |
| Faz 1 — Core | 4–6 hafta | 🔄 Devam |
| Faz 2 — ARC Görevler | 8–10 hafta | ⏳ Bekliyor |
| Faz 3 — Entegrasyon | 4–6 hafta | ⏳ Bekliyor |
| Faz 4 — Gerçek Rover | TBD | ⏳ Bekliyor |

---

## Ertelenen / Şimdilik Stub Bırakılan Modüller

| Modül | Neden Ertelendi |
|-------|----------------|
| `rock_detector` (kamera) | Gerçek veri seti ve model gerekiyor |
| `peak_finder` (elevation) | Elevation map altyapısı önce kurulmalı |
| `matlab_adapter` | Gerçek MATLAB ihtiyacı ileride netleşecek |
| Motor sürücüleri | Gerçek rover henüz yok |
| Isaac ROS GPU algılama | Faz 4 için |
| Web teleop UI | Faz 2 sonunda değerlendirilecek |
