# STM Protocol — H723 ↔ Pi Communication

> Reference: `stm codes/earendil-mainfirmware/` (H723), `stm codes/earendilmotorcontroller/` (F411×4)
> AGENT.md §4 for detailed extraction rules.

## Physical Layer

- **Interface:** USART3 on H723, USB-UART adapter on Pi
- **Baud:** 115200
- **Format:** ASCII text, `<command>\r\n` terminated
- **Udev:** `/dev/earendil_h7`

## Pi → H723 Commands

| Command | Format | Description |
|---|---|---|
| RPM | `FL rpm <signed>\r\n` | Front-left motor RPM |
| RPM | `FR rpm <signed>\r\n` | Front-right motor RPM |
| RPM | `RL rpm <signed>\r\n` | Rear-left motor RPM |
| RPM | `RR rpm <signed>\r\n` | Rear-right motor RPM |

- Positive RPM = forward, negative = reverse
- H723 forwards to F411 via 4× UART (USART2=FL, UART4=FR, UART7=RL, UART5=RR)

## H723 → Pi Telemetry

### Motor Telemetry (per motor)

Format: `FL|RPM:...,T:...,D:...,DIR:...,APP_PH:...,SP:...,BRAKE:...,FC:...,H:...,PWM_SET:...,PWM_ACT:...,QDROP:...,RXB:...\r\n`

| Field | Description |
|---|---|
| `RPM` | Measured RPM (from Hall sensors) |
| `T` | Temperature (raw) |
| `D` | Duty cycle |
| `DIR` | Direction |
| `APP_PH` | Applied phase |
| `SP` | Setpoint |
| `BRAKE` | Brake state |
| `FC` | Fault code |
| `H` | Hall state |
| `PWM_SET` | PWM setpoint |
| `PWM_ACT` | PWM actual |
| `QDROP` | Queue drop count |
| `RXB` | RX buffer |

Prefix: `FL|`, `FR|`, `RL|`, `RR|`

### IMU Data

- **MPU9250** (main IMU): accelerometer + gyroscope
  - Units: mg (accel), mdps/°s (gyro) — needs SI conversion on Pi side
  - Frame: TBD (ENU vs NED — needs H723 firmware reading)
- **QMC5883P** (magnetometer): magnetic field
  - Units: TBD

### Status/Mode

H723 operating mode:
- `DISARM` (0) — motors disabled, boot default
- `MANUAL` (1) — direct operator control
- `AUTONOMOUS` (2) — Pi-controlled

## ACK Protocol

- Pi sends command → H723 sends ACK within `ACK_TIMEOUT_MS=500ms`
- Max retries: `MAX_RETRIES=3`
- No ACK after 3 retries → command failed

## Link-Loss Detection

- H723: `LINK_LOSS_TIMEOUT_MS=3000ms` — if no command from Pi within 3s → safe state
- F411: `CMD_WATCHDOG_MS=800ms`, `HOST_DISCONNECT_TIMEOUT_MS=2000ms`

## Known Gaps (Pi-documented, firmware-side changes needed)

| Gap | Current | Target | Notes |
|---|---|---|---|
| Frame format | ASCII | Binary with CRC/seq/timestamp | Firmware change needed |
| Timeout | 3000ms | 300-500ms | Pi-side defense-in-depth in M4 |
| Odometry | None (Pi computes from RPM) | H723-side pose optional | Pi does this currently |
| Battery/Status | Placeholder | Real battery telemetry | Firmware change needed |
| Command format | RPM/duty/direction | cmd_vel (vx, wz) | Pi does conversion in stm_bridge |

## RoverMode ↔ RoverState Mapping

**CRITICAL — Safety-relevant, sequence differs!**

| RSCP RoverState | Value | H723 RoverMode_t | Value |
|---|---|---|---|
| DISARMED | 0 | DISARM | 0 |
| AUTONOMOUS | 1 | AUTONOMOUS | 2 |
| MANUAL | 2 | MANUAL | 1 |

Mapping function needed in stm_bridge: `rscp_state → h723_mode`

## F411 Motor Controller

- **MCU:** STM32F411 (4 units: FL, FR, RL, RR)
- **Motor:** BLDC 6-step commutation
- **Hall sensors:** POLE_PAIRS=15 → 90 edges/motor revolution
- **Gear ratio:** 1.0 (hub motor assumption — verify in M11)
- **Faults:** Non-latching (auto-resume risk)
- **No current sensor**
- **TIM1 gate-drive:** CCxE/CCxNE, dead-time, allOff — see `stm codes/.../docs/TIM1_GATE_DRIVE.md`
