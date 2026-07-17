# Test Implementation Guide — Earendil RPi5 Vehicle

> Bu doküman ARC 2026 Earendil rover projesi için adım adım test ve deployment rehberidir.
> RPi5'e nasıl aktaracağın, STM kartlarıyla nasıl test edeceğin, ve hangi milestone'da neyi doğrulaman gerektiği anlatılır.

---

## İçindekiler

1. [Genel Bakış ve Sistem Topolojisi](#1-genel-bakış-ve-sistem-topolojisi)
2. [Gerekli Donanım ve Araçlar](#2-gerekli-donanım-ve-araçlar)
3. [RPi5 Kurulum (Sıfırdan)](#3-rpi5-kurulum-sıfırdan)
4. [PC'den RPi5'e Aktarma ve Build](#4-pcden-rpi5e-aktarma-ve-build)
5. [STM Firmware Testleri](#5-stm-firmware-testleri)
6. [ROS 2 Node Testleri (Motorsuz)](#6-ros-2-node-testleri-motorsuz)
7. [RPi5 ↔ STM Entegrasyon Testi](#7-rpi5--stm-entegrasyon-testi)
8. [RSCP Testleri (Fake Competition Module)](#8-rscp-testleri-fake-competition-module)
9. [Safety Zinciri Testleri](#9-safety-zinciri-testleri)
10. [Sensör Testleri](#10-sensör-testleri)
11. [Saha Testleri (Motor Bağlı)](#11-saha-testleri-motor-bağlı)
12. [Troubleshooting](#12-troubleshooting)
13. [Hızlı Referans Komutları](#13-hızlı-referans-komutları)

---

## 1. Genel Bakış ve Sistem Topolojisi

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         TEST KATMANLARI                                 │
│                                                                         │
│  Layer 0: STM F411 (×4) ── BLDC motor kontrol, Hall sensör             │
│  Layer 1: STM H723      ── Motor dağıtıcı, IMU+mag, watchdog           │
│  Layer 2: RPi5           ── ROS 2, Nav2, RSCP bridge, safety           │
│  Layer 3: Jetson Nano    ── ArUco, derinlik (opsiyonel)                │
│                                                                         │
│  FİZİKSEL BAĞLANTI:                                                     │
│  CM ──[USB-UART]──> /dev/earendil_rscp ──> RSCP bridge                │
│  RPi5 ──[USB-UART]──> /dev/earendil_h7 ──> H723 (USART3, 115200)     │
│  H723 ──[4×UART]──> F411×4 (FL/FR/RL/RR)                              │
│  RPi5 ──[Ethernet/DDS]──> Jetson Nano                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

**Test felsefesi:** Motor bağlı olmadan test et. Her katmanı ayrı ayrı, sonra entegre test et. Milestone sırasını takip et: M1 → M2 → M3 → M4 → M5 → M6 → M7.

---

## 2. Gerekli Donanım ve Araçlar

### 2.1 Donanım

| Donanım | Amaç | Zorunlu mu? |
|---------|------|-------------|
| Raspberry Pi 5 (4/8 GB) | Ana bilgisayar | Evet |
| MicroSD kart (32 GB+) | OS + workspace | Evet |
| USB-UART adaptör (CP2102/CH340) ×2 | H723 + RSCP seri bağlantı | Evet (test için) |
| STM32H723ZG kartı | Ara beyin | Evet |
| STM32F411 motor kartı ×4 | BLDC sürücü | Saha testinde |
| ST-Link V2 veya J-Link | Firmware programlama/debug | Evet |
| Multimetre, osiloskop | Donanım doğrulama | Önerilir |
| BLDC motorlar + güç kaynağı | Saha testi | Saha testinde |

### 2.2 Yazılım Araçları (PC)

```bash
# PC'ye kurulacak (geliştirme için)
pip3 install rscp_protobuf cobs pyserial
sudo apt install stlink-tools        # STM programlama
# veya
sudo apt install openocd             # alternatif debug

# STM32CubeIDE veya PlatformIO
# PlatformIO: https://platformio.org/install/ide?install=vscode
pio --version  # veya STM32CubeIDE
```

### 2.3 Yazılım Araçları (RPi5)

```bash
# deploy/install_rpi5_ubuntu24.sh bunları kurar:
# ROS 2 Jazzy, Nav2, robot_localization, tf2, diagnostic_updater
# pyserial, pyyaml, aiohttp, gpiod
pip3 install rscp_protobuf cobs       # RSCP için ek
```

---

## 3. RPi5 Kurulum (Sıfırdan)

### 3.1 Ubuntu 24.04 Kurulumu

1. Raspberry Pi Imager ile Ubuntu Server 24.04 LTS (64-bit) flash'la
2. İlk boot: SSH erişimi aç (`ssh ubuntu@<ip>`, varsayılan şifre: `ubuntu`)
3. Sistem güncelle:
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo reboot
   ```

### 3.2 ROS 2 + Proje Kurulumu

```bash
# 1. Proje deposunu klonla
cd ~
git clone <repo-url> earendil_rpi5_vehicle
cd earendil_rpi5_vehicle

# 2. Install scriptini çalıştır
bash deploy/install_rpi5_ubuntu24.sh

# 3. Yeniden başlat
sudo reboot

# 4. RSCP Python paketlerini kur
pip3 install --break-system-packages rscp_protobuf cobs

# 5. Kullanıcı grubu ayarları (seri port erişimi)
sudo usermod -aG dialout,gpio,tty $USER
# Yeniden giriş yap veya reboot
```

### 3.3 Seri Port Doğrulama

```bash
# USB-UART adaptörlerini tak, sonra:
ls -la /dev/ttyUSB* /dev/ttyACM*
# Udev kurallarını kontrol et:
cat /etc/udev/rules.d/99-earendil.rules
# Olması gereken:
# /dev/earendil_h7   → H723 STM32
# /dev/earendil_rscp → RSCP Competition Module
# /dev/earendil_rtk  → RTK GPS

# Udev kuralı yoksa elle ekle (vendor/product ID'leri kontrol et):
# sudo nano /etc/udev/rules.d/99-earendil.rules
# SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="earendil_h7"

sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/earendil_*
```

---

## 4. PC'den RPi5'e Aktarma ve Build

### 4.1 Aktarma Yöntemleri

**Yöntem A: Git (önerilen)**
```bash
# PC'de
git add -A && git commit -m "your changes"
git push origin main

# RPi5'te
cd ~/earendil_rpi5_vehicle
git pull origin main
```

**Yöntem B: rsync (hızlı deneme)**
```bash
# PC'den RPi5'e (WiFi/Ethernet)
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
    --exclude='.pio/' --exclude='eski proje/' \
    ./ ubuntu@<rpi5_ip>:~/earendil_rpi5_vehicle/
```

**Yöntem C: scp (tek dosya)**
```bash
scp src/earendil_control/earendil_control/stm_bridge.py \
    ubuntu@<rpi5_ip>:~/earendil_rpi5_vehicle/src/earendil_control/earendil_control/
```

### 4.2 Build

```bash
# RPi5'te
cd ~/earendil_rpi5_vehicle

# Method 1: Script ile
bash deploy/build.sh

# Method 2: Manuel
source /opt/ros/jazzy/setup.bash
colcon build --packages-select earendil_interfaces --symlink-install
colcon build --symlink-install

# Doğrula
source install/setup.bash
ros2 interface list | grep earendil
ros2 pkg list | grep earendil
```

### 4.3 Build Hataları İçin

```bash
# Temiz build
rm -rf build/ install/ log/
colcon build --symlink-install 2>&1 | tee build_log.txt

# Tek paket build (hızlı debug)
colcon build --packages-select earendil_control --symlink-install --event-handlers console_direct+
```

---

## 5. STM Firmware Testleri

### 5.1 H723 Firmware Yükleme

```bash
# ST-Link ile (SWD)
# Yöntem 1: STM32CubeIDE
#   - projeyi aç → Build → Debug → Run

# Yöntem 2: st-flash (open source)
cd "stm codes/earendil-mainfirmware"
# .elf veya .bin dosyası STM32CubeIDE'den export et
st-flash write Debug/earendil-mainfirmware.bin 0x08000000

# Yöntem 3: PlatformIO (eğer platformio.ini varsa)
cd "stm codes/earendil-mainfirmware"
pio run -t upload
```

### 5.2 F411 Firmware Yükleme (Her motor kartı için)

```bash
cd "stm codes/earendilmotorcontroller/f411-motor-cube"
pio run -d f411-motor-cube              # build
pio run -d f411-motor-cube -t upload    # yükle (ST-Link bağlı)
```

> **UYARI:** PlatformIO yoksa rapor et, uydurma komut kullanma. `docs/SAFETY.md` ve `docs/BRINGUP.md` oku.

### 5.3 H723 Seri Test (PC'den, motorsuz)

H723'ü USB-UART ile PC'ye bağla. Bu test motor olmadan haberleşme doğrular.

```bash
# PC'de seri terminal aç (115200 baud)
screen /dev/ttyUSB0 115200
# veya
minicom -D /dev/ttyUSB0 -b 115200

# Komut gönder (H723 host interface):
# RPM komutu
FL rpm 0
FR rpm 0
RL rpm 0
RR rpm 0

# Beklenen yanıt: ACK (ACK_TIMEOUT_MS=500ms içinde)
# Motor bağlı değilse telemetri hatası normal ama ACK gelmeli

# Mode komutu (eğer firmware destekliyorsa)
mode autonomous
mode disarm

# Çıkış: Ctrl+A → K (screen) veya Ctrl+A → X (minicom)
```

**Doğrulama:**
- [ ] H723 boot mesajı geldi
- [ ] `FL rpm 0` komutuna ACK geldi (< 500ms)
- [ ] Seri iletişim kararlı (kopma yok)

### 5.4 F411 Tek Motor Test (Tek kart, motorsuz)

F411'i USB-UART ile PC'ye bağla. Motor bağlı değil ama firmware tepkisini gör.

```bash
# F411 seri terminal
screen /dev/ttyUSB1 115200

# Komutlar (docs/PROTOCOL.md'den):
help            # Komut listesi
status          # Durum sorgula
rpm 0           # Sıfır RPM
stop            # Motor durdur
clrerr          # Hataları temizle

# Motor bağlı değilken beklenen:
# - Hall sensör verisi yok → RPM=0, H=0
# - Fault code ≠ 0 olabilir (motor disconnected)
# - Ama firmware crash yapmamalı
```

**Doğrulama:**
- [ ] F411 boot ve `help` çalışır
- [ ] `status` motor durumunu raporlar (motor olmasa bile)
- [ ] `rpm 0` komutu ACK alır
- [ ] `stop` çalışır
- [ ] Watchdog timeout davranışı: komut göndermeyi bırak → 800ms sonra motor durur

### 5.5 H723 ↔ F411 Entegrasyon (Motorsuz)

H723 ve F411 kartlarını UART hatları üzerinden bağla (FL=USART2, FR=UART4, RL=UART7, RR=UART5).

```bash
# PC'den H723'e komut gönder:
FL rpm 100

# H723 F411-FL'e "rpm 100" göndermeli
# F411 motor bağlı değil → telemetri: RPM:0, FC:fault_code
# H723 bu telemetriyi prefix ile PC'ye aktarır:
# FL|RPM:0,T:...,D:...,DIR:...,APP_PH:...,SP:100,BRAKE:0,FC:...,H:...
```

**Doğrulama:**
- [ ] H723 komutu F411'e iletir
- [ ] F411 telemetri satırı H723 prefix ile döner
- [ ] 4 motor hattı ayrı ayrı çalışır
- [ ] LINK_LOSS_TIMEOUT (3000ms): komut kesilince H723 safe state'e geçer

---

## 6. ROS 2 Node Testleri (Motorsuz)

Tüm testler `use_hardware:=false` modunda, motor bağlı olmadan.

### 6.1 Build ve Launch Doğrulama

```bash
cd ~/earendil_rpi5_vehicle
source install/setup.bash

# 1. Interface'ler doğru mu?
ros2 interface list | grep earendil
# Beklenen: RscpCommand, RscpStatus, MissionStage, MissionStatus,
#           StmStatus, StmFaultFlags, SafetyStatus, VehicleState, vb.

# 2. Her interface'in detayını kontrol et
ros2 interface show earendil_interfaces/msg/RscpCommand
ros2 interface show earendil_interfaces/msg/StmStatus

# 3. Paketler doğru mu?
ros2 pkg list | grep earendil
# Beklenen: earendil_interfaces, earendil_rscp_bridge, earendil_control,
#           earendil_safety, earendil_navigation, earendil_sensors, vb.

# 4. Launch dosyaları açılıyor mu?
ros2 launch earendil_bringup minimal.launch.py
# Ctrl+C ile kapat. Crash olmamalı.
```

### 6.2 STM Bridge Test (Simulation Mode)

```bash
# Terminal 1: stm_bridge simülasyon modunda başlat
ros2 run earendil_control stm_bridge --ros-args \
    -p use_hardware:=false \
    -p cmd_vel_topic:=/cmd_vel_safe

# Terminal 2: Test komutları gönder
# Düz ileri (1 m/s)
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Dur
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Status kontrol
ros2 topic echo /stm/status
# Beklenen: operating_mode=0 (DISARMED), link_active=false (sim mod)
```

**Doğrulama:**
- [ ] `use_hardware:=false` ile serial bağlantı denemiyor
- [ ] Status topic'i periyodik publish ediyor (1 Hz)
- [ ] Log'da "SIMULATION mode" mesajı görünüyor
- [ ] Crash yok

### 6.3 RSCP Bridge Test (Simülasyon, Seri Yok)

RSCP bridge seri port olmadan test edilemez (her zaman seri açmaya çalışır). Ama `rscp_parser.py` modülünü ayrı test edebilirsin.

```python
# Python testi — PC veya RPi5'te
python3 -c "
from earendil_rscp_bridge.rscp_parser import (
    FrameReader, decode_frame, encode_frame,
    parse_request, create_acknowledge, create_task_finished
)

# 1. Acknowledge encode/decode
ack_bytes = create_acknowledge()
encoded = encode_frame(ack_bytes)
print(f'Acknowledge frame: {encoded.hex()}')

# 2. COBS round-trip
import cobs
decoded = cobs.cobs.decode(encoded.rstrip(b'\x00'))
print(f'COBS decoded: {decoded.hex()}')

# 3. RequestEnvelope parse
import rscp_protobuf as rscp
env = rscp.RequestEnvelope()
env.set_stage.value = 1
data = env.SerializeToString()
parsed_env = rscp.RequestEnvelope()
parsed_env.ParseFromString(data)
print(f'Parsed field: {parsed_env.WhichOneof(\"request\")}')
print(f'Stage value: {parsed_env.set_stage.value}')

print('All RSCP parser tests passed!')
"
```

### 6.4 Safety Node Test

```bash
# Terminal 1: safety node başlat
ros2 run earendil_safety safety_node --ros-args \
    -p input_topic:=/cmd_vel_pre_safety \
    -p output_topic:=/cmd_vel

# Terminal 2: Test senaryoları

# 1. Deadman yokken komut gönder → çıkış sıfır olmalı
ros2 topic echo /cmd_vel &
ros2 topic pub --once /cmd_vel_pre_safety geometry_msgs/Twist \
    "{linear: {x: 0.5}}"

# 2. Deadman bas → komut geçmeli
ros2 topic pub -r 10 /deadman std_msgs/Bool "{data: true}" &
ros2 topic pub -r 10 /cmd_vel_pre_safety geometry_msgs/Twist \
    "{linear: {x: 0.3}}"
ros2 topic echo /cmd_vel  # 0.3 m/s görmeli

# 3. Deadman bırak → 1 saniye sonra sıfır
ros2 topic pub --once /deadman std_msgs/Bool "{data: false}"
# 1 saniye bekle, /cmd_vel sıfır olmalı

# 4. E-stop testi
ros2 service call /emergency_stop earendil_interfaces/srv/EmergencyStop \
    "{engage: true}"
# /cmd_vel sıfır olmalı

# 5. E-stop kaldır
ros2 service call /emergency_stop earendil_interfaces/srv/EmergencyStop \
    "{engage: false}"
```

**Doğrulama:**
- [ ] Deadman yokken → çıkış sıfır
- [ ] Deadman varken → komut geçiyor
- [ ] Deadman bırakınca → timeout ile sıfır
- [ ] E-stop → anında sıfır
- [ ] Speed limit → max_linear, max_angular aşılmıyor

---

## 7. RPi5 ↔ STM Entegrasyon Testi

Bu testte H723 USB-UART ile RPi5'e bağlı. Motorlar bağlı değil.

### 7.1 Fiziksel Bağlantı

```
RPi5 USB port ──[USB-UART adaptör]──> H723 USART3 (TX/RX/GND)
                                       │
                               H723 4×UART hatları
                                       │
                              (F411 kartlarına bağlı veya değil)
```

**ÖNEMLI:** GND bağlantısını unutma. TX↔RX çapraz bağlı olmalı.

### 7.2 Udev Test

```bash
# Adaptörü tak, sonra:
dmesg | tail -5          # hangi /dev/ttyUSB* olduğunu gör
ls -la /dev/earendil_*   # udev symlink oluşmuş mu?
udevadm info /dev/earendil_h7  # detaylı bilgi
```

### 7.3 stm_bridge Hardware Mode Test

```bash
# Terminal 1: stm_bridge hardware modunda
ros2 run earendil_control stm_bridge --ros-args \
    -p use_hardware:=true \
    -p serial_port:=/dev/earendil_h7

# Beklenen log:
# "stm_bridge started — port=/dev/earendil_h7, ... use_hardware=True"
# "STM serial opened: /dev/earendil_h7"
# (periyodik status: link_active=true veya false)

# Terminal 2: Komut gönder
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/Twist \
    "{linear: {x: 0.0}, angular: {x: 0.0}}"

# H723'e giden komutları görmek için:
# H723 seri terminali aç (başka bir USB-UART ile) veya
# stm_bridge log'unda "FL rpm 0" gibi mesajlar görmeli
```

**Doğrulama:**
- [ ] Seri port açıldı (log: "STM serial opened")
- [ ] Komut gönderimi: H723'e `FL rpm 0\r\n` gidiyor
- [ ] H723'den telemetri geliyor → `/stm/status` link_active=true
- [ ] Link loss: H723'ü kapat → 3 saniye sonra link_active=false

### 7.4 Telemetri Doğrulama

```bash
# H723'den telemetri gelirken:
ros2 topic echo /stm/status
ros2 topic echo /stm/imu/data
ros2 topic echo /stm/magnetic_field
ros2 topic echo /stm/wheel_odom

# Motor bağlı değilse:
# - /stm/imu/data: MPU9250 verisi gelmeli (H723 kartında IMU var)
# - /stm/magnetic_field: QMC5883P verisi gelmeli
# - /stm/wheel_odom: RPM=0 → v=0, ω=0
# - /stm/status: link_active=true, operating_mode=0 (DISARMED)
```

---

## 8. RSCP Testleri (Fake Competition Module)

Competition Module elimizde değilken fake module ile test ediyoruz.

### 8.1 Fake RSCP Module Scripti

```python
#!/usr/bin/env python3
"""
fake_rscp_module.py — Fake ARC Competition Module.
H723 ile aynı mantıkta USB-UART kullanır, RSCP frame'leri gönderir.

Kullanım:
    python3 fake_rscp_module.py /dev/ttyUSB0
"""
import sys
import time
import cobs
import rscp_protobuf as rscp


def create_set_stage(stage: int) -> bytes:
    env = rscp.RequestEnvelope()
    env.set_stage.value = stage
    return env.SerializeToString()


def create_arm(arm: bool) -> bytes:
    env = rscp.RequestEnvelope()
    env.arm_disarm.value = arm
    return env.SerializeToString()


def create_navigate(lat: float, lon: float, alt: float) -> bytes:
    env = rscp.RequestEnvelope()
    env.navigate_to_gps.coordinate.latitude = lat
    env.navigate_to_gps.coordinate.longitude = lon
    env.navigate_to_gps.coordinate.altitude = alt
    return env.SerializeToString()


def create_search(lat: float, lon: float, radius: float) -> bytes:
    env = rscp.RequestEnvelope()
    env.search_area.center_coordinate.latitude = lat
    env.search_area.center_coordinate.longitude = lon
    env.search_area.radius = radius
    return env.SerializeToString()


def create_explore() -> bytes:
    env = rscp.RequestEnvelope()
    env.start_exploration.dummy_field = True
    return env.SerializeToString()


def encode_and_send(serial_port, protobuf_bytes: bytes):
    """COBS encode + 0x00 delimiter + send."""
    encoded = cobs.cobs.encode(protobuf_bytes) + b'\x00'
    serial_port.write(encoded)
    print(f"  Sent {len(encoded)} bytes: {protobuf_bytes.hex()}")


def wait_for_response(serial_port, timeout=2.0):
    """Wait for a COBS response frame."""
    buffer = b''
    start = time.time()
    while time.time() - start < timeout:
        data = serial_port.read(1)
        if data:
            if data[0] == 0x00:
                if buffer:
                    try:
                        decoded = cobs.cobs.decode(buffer)
                        env = rscp.ResponseEnvelope()
                        env.ParseFromString(decoded)
                        field = env.WhichOneof('response')
                        print(f"  Response: {field}")
                        if field == 'message':
                            print(f"    message: {env.message}")
                        elif field == 'rover_status':
                            print(f"    state: {env.rover_status.state}")
                            print(f"    heading: {env.rover_status.heading}")
                        return field
                    except Exception as e:
                        print(f"  Parse error: {e}")
                        return None
                buffer = b''
            else:
                buffer += data
    print("  Timeout — no response")
    return None


def run_stage_1(port):
    """Stage 1: Antenna Installation test sequence."""
    print("\n=== Stage 1: Antenna Installation ===")

    print("1. SetStage(1)")
    encode_and_send(port, create_set_stage(1))
    resp = wait_for_response(port)
    assert resp == 'acknowledge', f"Expected acknowledge, got {resp}"

    print("2. ArmDisarm(true)")
    encode_and_send(port, create_arm(True))
    resp = wait_for_response(port)
    assert resp == 'acknowledge'

    print("3. SearchArea(center, 10m)")
    encode_and_send(port, create_search(39.92, 32.85, 10.0))
    resp = wait_for_response(port)
    assert resp == 'acknowledge'

    print("  (Waiting for rover to find antenna...)")
    # In real scenario, wait for GPSCoordinate + TaskFinished
    time.sleep(2)

    print("4. Simulate TaskFinished (from mission result)")
    # In real scenario, this comes from /mission/result topic
    print("  Stage 1 test complete (simulated)")


def run_all_commands(port):
    """Quick test: send all 5 command types."""
    print("\n=== Quick Command Test ===")

    commands = [
        ("SetStage(1)", create_set_stage(1)),
        ("ArmDisarm(true)", create_arm(True)),
        ("NavigateToGPS", create_navigate(39.92, 32.85, 900.0)),
        ("SearchArea(10m)", create_search(39.92, 32.85, 10.0)),
        ("StartExploration", create_explore()),
        ("ArmDisarm(false)", create_arm(False)),
    ]

    for name, data in commands:
        print(f"\n{name}:")
        encode_and_send(port, data)
        resp = wait_for_response(port)
        if resp:
            print(f"  OK: {resp}")
        else:
            print(f"  WARN: no response")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port> [stage1|all]")
        sys.exit(1)

    import serial
    port_name = sys.argv[1]
    mode = sys.argv[2] if len(sys.argv) > 2 else 'all'

    port = serial.Serial(port_name, 115200, timeout=0.1)
    print(f"Connected to {port_name}")

    if mode == 'stage1':
        run_stage_1(port)
    else:
        run_all_commands(port)

    port.close()
    print("\nDone!")
```

### 8.2 Fake RSCP ile Test

```bash
# Yöntem A: İki USB-UART adaptörü + loopback
# RPi5'te:
#   /dev/earendil_rscp → RSCP bridge'e bağlı
#   /dev/ttyUSB1 → fake module'a bağlı
# İkisi birbirine bağlı (TX↔RX çapraz)

# Terminal 1: RSCP bridge
ros2 run earendil_rscp_bridge rscp_bridge_node --ros-args \
    -p serial_port:=/dev/earendil_rscp

# Terminal 2: Fake module
python3 fake_rscp_module.py /dev/ttyUSB1 all

# Terminal 3: Topic'leri izle
ros2 topic echo /rscp/command
ros2 topic echo /rscp/current_stage
ros2 topic echo /rscp/status

# Yöntem B: Loopback (tek adaptör, TX→RX)
# H723 TX pinini RX pinine bağla (loopback jumper)
# Bu durumda bridge kendi gönderdiğini okur — tam test değil ama framing doğrular
```

### 8.3 RSCP Parser Unit Test

```python
#!/usr/bin/env python3
"""test_rscp_parser.py — RSCP parser unit testleri."""
import sys
sys.path.insert(0, 'src/earendil_rscp_bridge')

from earendil_rscp_bridge.rscp_parser import (
    FrameReader, decode_frame, encode_frame,
    parse_request, create_acknowledge, create_task_finished,
    create_gps_coordinate, create_distance, create_message, create_rover_status,
)

import rscp_protobuf as rscp


def test_cobs_roundtrip():
    """COBS encode → decode round trip."""
    ack = create_acknowledge()
    encoded = encode_frame(ack)
    # 0x00 delimiter'ı kaldır
    raw = encoded[:-1]
    decoded = decode_frame(raw)
    assert decoded == ack
    print("PASS: COBS round-trip")


def test_parse_set_stage():
    """SetStage parse testi."""
    env = rscp.RequestEnvelope()
    env.set_stage.value = 2
    data = env.SerializeToString()

    envelope, field, cmd = parse_request(data)
    assert envelope is not None
    assert field == 'set_stage'
    assert cmd.stage_value == 2
    print("PASS: parse SetStage")


def test_parse_arm():
    """ArmDisarm parse testi."""
    env = rscp.RequestEnvelope()
    env.arm_disarm.value = True
    data = env.SerializeToString()

    envelope, field, cmd = parse_request(data)
    assert field == 'arm_disarm'
    assert cmd.arm_value == True
    print("PASS: parse ArmDisarm")


def test_parse_navigate():
    """NavigateToGPS parse testi."""
    env = rscp.RequestEnvelope()
    env.navigate_to_gps.coordinate.latitude = 39.92
    env.navigate_to_gps.coordinate.longitude = 32.85
    env.navigate_to_gps.coordinate.altitude = 900.0
    data = env.SerializeToString()

    envelope, field, cmd = parse_request(data)
    assert field == 'navigate_to_gps'
    assert abs(cmd.latitude - 39.92) < 0.001
    assert abs(cmd.longitude - 32.85) < 0.001
    print("PASS: parse NavigateToGPS")


def test_parse_search():
    """SearchArea parse testi."""
    env = rscp.RequestEnvelope()
    env.search_area.center_coordinate.latitude = 39.92
    env.search_area.center_coordinate.longitude = 32.85
    env.search_area.radius = 10.0
    data = env.SerializeToString()

    envelope, field, cmd = parse_request(data)
    assert field == 'search_area'
    assert abs(cmd.search_radius - 10.0) < 0.01
    print("PASS: parse SearchArea")


def test_parse_explore():
    """StartExploration parse testi."""
    env = rscp.RequestEnvelope()
    env.start_exploration.dummy_field = True
    data = env.SerializeToString()

    envelope, field, cmd = parse_request(data)
    assert field == 'start_exploration'
    print("PASS: parse StartExploration")


def test_parse_unknown():
    """Bilinmeyen/boş mesaj parse testi."""
    data = b'\x00\x01\x02\x03'  # bozuk veri
    envelope, field, cmd = parse_request(data)
    assert envelope is None or field is None
    print("PASS: parse unknown (no crash)")


def test_frame_reader():
    """FrameReader byte-byte feed testi."""
    ack = create_acknowledge()
    encoded = encode_frame(ack)

    reader = FrameReader()
    for byte in encoded:
        result = reader.feed(byte)
        if byte == 0x00 and result is not None:
            # Frame bulundu
            decoded = decode_frame(result)
            assert decoded == ack
            print("PASS: FrameReader")
            return
    print("FAIL: FrameReader — no frame completed")


def test_create_responses():
    """Response oluşturma testleri."""
    # Acknowledge
    ack = create_acknowledge()
    assert len(ack) > 0
    print("PASS: create_acknowledge")

    # TaskFinished
    tf = create_task_finished()
    assert len(tf) > 0
    print("PASS: create_task_finished")

    # GPSCoordinate
    gps = create_gps_coordinate(39.92, 32.85, 900.0)
    assert len(gps) > 0
    env = rscp.ResponseEnvelope()
    env.ParseFromString(gps)
    assert env.WhichOneof('response') == 'gps_coordinate'
    assert abs(env.gps_coordinate.latitude - 39.92) < 0.001
    print("PASS: create_gps_coordinate")

    # Distance
    dist = create_distance(42.5)
    env2 = rscp.ResponseEnvelope()
    env2.ParseFromString(dist)
    assert env2.WhichOneof('response') == 'distance'
    assert abs(env2.distance - 42.5) < 0.01
    print("PASS: create_distance")

    # Message
    msg = create_message("test error")
    env3 = rscp.ResponseEnvelope()
    env3.ParseFromString(msg)
    assert env3.WhichOneof('response') == 'message'
    assert env3.message == "test error"
    print("PASS: create_message")

    # RoverStatus
    rs = create_rover_status(
        state=1, latitude=39.92, longitude=32.85, altitude=900.0,
        heading=180.0, battery_voltage=12.0, battery_current=1.5, battery_soc=0.8
    )
    env4 = rscp.ResponseEnvelope()
    env4.ParseFromString(rs)
    assert env4.WhichOneof('response') == 'rover_status'
    assert env4.rover_status.state == 1
    print("PASS: create_rover_status")


if __name__ == '__main__':
    test_cobs_roundtrip()
    test_parse_set_stage()
    test_parse_arm()
    test_parse_navigate()
    test_parse_search()
    test_parse_explore()
    test_parse_unknown()
    test_frame_reader()
    test_create_responses()
    print("\n=== ALL TESTS PASSED ===")
```

---

## 9. Safety Zinciri Testleri

### 9.1 Tek-Yazıcı Audit

Bu test `/cmd_vel_safe`'e yalnız safety_mux'un yazdığını doğrular.

```bash
# Tüm sistemi başlat (simulation mode)
ros2 launch earendil_bringup vehicle.launch.py use_hardware:=false

# Audit: /cmd_vel_safe publisher'ları
ros2 topic info /cmd_vel_safe
# Beklenen: Publisher count: 1 (safety_mux)

# Audit: H723 serial publisher'ı
# (stm_bridge'den başka kimse /dev/earendil_h7'ye yazmamalı)
# Bu elle doğrulanır veya launch dosyasında kontrol edilir

# Audit: Jetson motor topic'ine yazıyor mu?
# Jetsonağlı yoksa bu test atlanır ama mimari olarak Jetson motor komutu ÜRETMEZ
```

### 9.2 Safety Gate Fonksiyonel Test

```bash
# Başlat
ros2 run earendil_safety safety_node --ros-args \
    -p input_topic:=/cmd_vel_pre_safety \
    -p output_topic:=/cmd_vel_safe \
    -p watchdog_timeout_ms:=500 \
    -p max_linear_speed:=0.5

# Test 1: Komut yok → çıkış sıfır
ros2 topic echo /cmd_vel_safe
# İlk 500ms'den sonra Twist(0,0,0,0,0,0) görmeli

# Test 2: Watchdog timeout
ros2 topic pub -r 20 /cmd_vel_pre_safety geometry_msgs/Twist "{linear: {x: 0.3}}"
sleep 0.3
ros2 topic pub --once /cmd_vel_pre_safety geometry_msgs/Twist "{linear: {x: 0.0}}"
# 500ms bekle → çıkış sıfır olmalı

# Test 3: E-stop latching
ros2 service call /emergency_stop earendil_interfaces/srv/EmergencyStop "{engage: true}"
ros2 topic pub -r 20 /cmd_vel_pre_safety geometry_msgs/Twist "{linear: {x: 0.5}}"
ros2 topic echo /cmd_vel_safe  # sıfır olmalı
```

### 9.3 Üç Katmanlı Timeout Doğrulama

```
Katman 1 — Pi safety_mux: 300-500ms komut timeout → sıfır
Katman 2 — H723: LINK_LOSS_TIMEOUT_MS=3000ms → safe state
Katman 3 — F411: CMD_WATCHDOG_MS=800ms → motor durdur
```

Test:
1. Komut göndermeyi kes
2. 300-500ms: Pi safety_mux sıfır gönderir
3. 800ms: F411 motor durdurur (motor bağlıyken)
4. 3000ms: H723 safe state'e geçer

---

## 10. Sensör Testleri

### 10.1 RTK GPS Testi

```bash
# GPS adaptörünü tak
ls -la /dev/earendil_rtk

# NMEA verisini kontrol et
screen /dev/earendil_rtk 9600
# $GPGGA, $GPRMC, $GNGGA gibi NMEA cümleleri görmeli
# Ctrl+A, K ile çık

# GPS adapter node başlat (varsa)
ros2 run earendil_sensors gps_adapter
ros2 topic echo /gps/fix
ros2 topic echo /rtk/status
```

### 10.2 LiDAR Testi

```bash
# LiDAR modeline göre driver
# Örnek: RPLidar
ros2 launch rplidar_ros rplidar_a1_launch.py
ros2 topic echo /scan
# LaserScan mesajı görmeli, 360 derece tarama

# TF kontrolü
ros2 run tf2_ros tf2_echo base_link lidar_link
```

### 10.3 STM IMU/Mag Testi

```bash
# stm_bridge çalışırken:
ros2 topic echo /stm/imu/data
# header.frame_id = "imu_link"
# linear_acceleration: ~9.8 z'de (yerçekimi)
# angular_velocity: ~0 (durağan)

ros2 topic echo /stm/magnetic_field
# Manyetometre verisi
```

### 10.4 TF Ağacı Doğrulama

```bash
# TF ağacını görselleştir
ros2 run tf2_tools view_frames
# frames.pdf aç → TF ağacı doğru mu?

# Beklenen TF:
# map → odom → base_link → imu_link
#                         → lidar_link
#                         → gps_link

# Elle kontrol
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

---

## 11. Saha Testleri (Motor Bağlı)

> **UYARI:** Bu testlerde motorlar bağlı ve enerjili. E-stop erişilebilir olmalı. Güvenlik kurallarına u.

### 11.1 İlk Motor Test (Yerde, Tekerlekler Havada)

```bash
# 1. E-stop basılı tut
# 2. stm_bridge başlat (hardware mode)
ros2 run earendil_control stm_bridge --ros-args \
    -p use_hardware:=true \
    -p serial_port:=/dev/earendil_h7

# 3. ARM et
ros2 topic pub --once /rscp/command earendil_interfaces/msg/RscpCommand \
    "{command_type: 1}"  # CMD_ARM

# 4. Çok düşük hız gönder
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/Twist \
    "{linear: {x: 0.1}}"  # 0.1 m/s

# 5. Motorlar dönmeli, tekerlekler havada
# 6. E-stop bırak → motor durmalı
```

**Doğrulama:**
- [ ] 4 motor aynı yönde dönüyor
- [ ] Hız artınca RPM artıyor
- [ ] E-stop → anında duruş
- [ ] Ters yön testi: `linear.x = -0.1`

### 11.2 Düz Sürüş Testi (1 metre)

```bash
# Zeminde test
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.2}}"
# 5 saniye (≈1 metre @ 0.2 m/s)
sleep 5
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}}"

# Ölç: başlangıç ve bitiş noktası arası mesafe
# Beklenen: ~1 metre, hata ±10cm (kalibrasyon öncesi)
```

### 11.3 Tank Dönüş Testi (90°)

```bash
# 90 derece tank dönüşü
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/Twist \
    "{angular: {z: 0.5}}"  # 0.5 rad/s

# 90° = π/2 rad → süre: (π/2) / 0.5 ≈ 3.14 saniye
sleep 3.14
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}}"

# Ölç: dönüş açısını kontrol et
# Beklenen: ~90°, hata ±15° (kalibrasyon öncesi)
```

### 11.4 Otonom Navigasyon Testi (GPS waypoint)

```bash
# Nav2 + GPS ile otonom navigasyon
ros2 launch earendil_bringup vehicle.launch.py \
    use_hardware:=true \
    use_gps:=true \
    use_navigation:=true

# Mission manager'a waypoint gönder
ros2 topic pub --once /mission/command earendil_interfaces/msg/RscpCommand \
    "{command_type: 3, latitude: 39.9200, longitude: 32.8500, altitude: 900.0}"

# Rover gitmeli, GPS fix izle
ros2 topic echo /gps/fix
ros2 topic echo /rtk/status
```

### 11.5 Tam RSCP Stage Akışı

```bash
# Fake RSCP module ile 4 stage tam akış testi

# Stage 1: Antenna Installation
python3 fake_rscp_module.py /dev/ttyUSB1 stage1
# Rover SearchArea'da dönmeli, GPSCoordinate raporlamalı, TaskFinished göndermeli

# Stage 2-4: ROADMAP.md §7'deki state machine tablolarına göre
```

---

## 12. Troubleshooting

### 12.1 Seri Port Sorunları

| Sorun | Çözüm |
|-------|-------|
| `/dev/earendil_*` görünmüyor | `dmesg \| tail`, USB adaptörü kontrol et, udev kuralları |
| Permission denied | `sudo usermod -aG dialout $USER`, yeniden giriş yap |
| Port meşgul | `lsof /dev/earendil_h7`, başka process kapat |
| Bağlantı kopuyor | USB kablosu/kalitesi, powered hub kullan |
| Yanlış port | `udevadm info /dev/ttyUSB0`, vendor/product ID eşleştir |

### 12.2 Build Sorunları

| Sorun | Çözüm |
|-------|-------|
| `rscp_protobuf` import hatası | `pip3 install --break-system-packages rscp_protobuf` |
| `cobs` bulunamıyor | `pip3 install --break-system-packages cobs` |
| `colcon build` hata | `rm -rf build/ install/ log/`, temiz build |
| Interface bulunamıyor | `colcon build --packages-select earendil_interfaces` önce |
| Launch hata | `ros2 launch --show-args <package> <launch.py>` argümanları kontrol et |

### 12.3 STM Sorunları

| Sorun | Çözüm |
|-------|-------|
| H723'e yükleme yapılamıyor | ST-Link bağlantısı, BOOT0 pin, `st-flash` |
| F411'e yükleme yapılamıyor | PlatformIO + ST-Link, `docs/BRINGUP.md` |
| ACK gelmiyor | Baud rate (115200), TX↔RX çapraz, GND bağlantısı |
| Motor dönmüyor | H723 mode: DISARM'da → ARM et, güç kaynağı |
| Motor titriyor | Hall sensör kablo sırası, `hall map` komutu |
| F411 fault | `clrerr` gönder, güç kaynağını kontrol et |

### 12.4 ROS Sorunları

| Sorun | Çözüm |
|-------|-------|
| Topic yayınlanmıyor | `ros2 topic list`, publisher var mı? |
| QoS uyumsuz | `ros2 topic info -v /topic`, QoS eşleştir |
| Node çöküyor | `ros2 run --prefix 'gdb -ex run' package node` |
| TF hatası | `ros2 run tf2_tools view_frames`, eksik TF |
| DDS interference | `export ROS_DOMAIN_ID=<unique>`, Jetson ile farklı |

---

## 13. Hızlı Referans Komutları

### 13.1 Build ve Run

```bash
# Tam build
cd ~/earendil_rpi5_vehicle
bash deploy/build.sh

# Çalıştır
bash deploy/run_vehicle.sh

# Systemd ile
sudo systemctl start earendil-vehicle
sudo systemctl status earendil-vehicle
journalctl -u earendil-vehicle -f
```

### 13.2 Node Tek Tek Başlatma

```bash
source install/setup.bash

# STM bridge
ros2 run earendil_control stm_bridge --ros-args -p use_hardware:=false

# Safety
ros2 run earendil_safety safety_node

# RSCP bridge
ros2 run earendil_rscp_bridge rscp_bridge_node

# Navigation
ros2 launch earendil_navigation mission.launch.py

# Full vehicle
ros2 launch earendil_bringup vehicle.launch.py use_hardware:=false
```

### 13.3 Monitoring

```bash
# Topic listesi
ros2 topic list

# Topic hızı
ros2 topic hz /stm/imu/data
ros2 topic hz /stm/wheel_odom
ros2 topic hz /cmd_vel_safe

# Topic detay
ros2 topic info -v /cmd_vel_safe

# Node listesi
ros2 node list

# Node graph
ros2 run rqt_graph rqt_graph

# TF ağacı
ros2 run tf2_tools view_frames

# STM durumu
ros2 topic echo /stm/status
ros2 topic echo /stm/fault_flags

# Safety durumu
ros2 topic echo /safety_status

# RSCP durumu
ros2 topic echo /rscp/status
```

### 13.4 STM Debug Komutları (Seri Terminal)

```bash
# H723'e bağlan
screen /dev/earendil_h7 115200

# Motor komutları
FL rpm 100    # Front-left, 100 RPM
FR rpm -100   # Front-right, geri
RL rpm 0      # Rear-left, dur
RR rpm 0      # Rear-right, dur

# Mode değiştir
mode autonomous
mode manual
mode disarm

# Durum sorgula
status

# Çıkış: Ctrl+A, sonra K
```

### 13.5 Test Senaryoları Hızlı Kartı

| Test | Komut | Beklenen |
|------|-------|----------|
| Build temiz | `colcon build --symlink-install` | 0 error |
| Interface list | `ros2 interface list \| grep earendil` | Tüm msg/srv/action |
| RSCP import | `python3 -c "import rscp_protobuf"` | Import OK |
| COBS import | `python3 -c "import cobs"` | Import OK |
| sim mode launch | `ros2 launch earendil_bringup minimal.launch.py` | Crash yok |
| STM sim | `ros2 run earendil_control stm_bridge -p use_hardware:=false` | SIMULATION log |
| Status Hz | `ros2 topic hz /stm/status` | ~1 Hz |
| Safety zero | e-stop aktif + komut gönder | `/cmd_vel_safe` = 0 |
| Single writer | `ros2 topic info /cmd_vel_safe` | 1 publisher |

---

## Milestone Test Sırası

```
M1  → §4.2 (build), §6.1 (interface list)
M2  → §8 (RSCP parser test, fake module)
M3  → §5 (STM test), §6.2 (stm_bridge sim), §7 (H723 entegrasyon)
M4  → §9 (safety zinciri)
M5  → §10 (sensör testleri)
M6  → TF doğrulama, localization (bu doc'ta yok, ROADMAP §9 M6'ya bak)
M7  → §8.2 (fake RSCP ile tam stage akışı)
M11 → §11 (saha testleri)
```

> Her milestone sonrası bir önceki milestone'ın testlerini tekrar çalıştır (regression).

---

*Bu doküman ROADMAP.md ve CLAUDE.md referans alınarak hazırlanmıştır. Sorular için: `docs/topic_design.md`, `docs/stm_protocol.md`, `docs/rscp_bridge.md`.*
