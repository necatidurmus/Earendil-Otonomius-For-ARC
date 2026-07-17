# RSCP Bridge Design Document

> Reference: `rscp protokol/rscp/proto/rscp.proto` (single source of truth),
> `rscp protokol/rscp/examples/python/` (idioms), AGENT.md §1-§2.

## Overview

The `earendil_rscp_bridge` package provides serial communication between the RPi5 and the ARC Competition Module (CM) using the RSCP (Rover Serial Communication Protocol).

**Single responsibility:** Serial + COBS + protobuf. Parse `RequestEnvelope`, dispatch to internal topics. Produce `ResponseEnvelope` from rover state.

**Does NOT:** Generate motor commands. Motor path: RSCP → mission_manager → Nav2 → `/cmd_vel_nav` → safety_mux → `/cmd_vel_safe` → stm_bridge.

## Protocol Stack

```
Physical:  USB-UART serial (/dev/earendil_rscp, 115200 baud)
Framing:   COBS encoding, 0x00 delimiter
Payload:   Protobuf (rscp_protobuf package)
Message:   RequestEnvelope / ResponseEnvelope
```

## Dependencies

- `rscp_protobuf` — official Python package from ARC GitHub releases
  - Install: `pip3 install https://github.com/anatolianroverchallenge/rscp/releases/latest/download/rscp_protobuf.zip`
- `cobs` — COBS encoding/decoding
  - Install: `pip3 install cobs`
- `pyserial` — serial port communication
  - Install: `pip3 install pyserial`

**Manual serialize/deserialize is NOT used.** Only official `.proto` + generated classes.

## Receive Flow (CM → Rover)

```
Serial byte-byte read
  → 0x00 detected → buffer collected
  → cobs.cobs.decode(buffer) → decoded bytes
  → rscp_protobuf.RequestEnvelope().ParseFromString(decoded)
  → request.WhichOneof('request')
  → dispatch:
      'arm_disarm'         → /rscp/command (CMD_ARM or CMD_DISARM)
      'set_stage'          → /rscp/current_stage + /rscp/command
      'navigate_to_gps'    → /mission/command (CMD_NAVIGATE_TO_GPS)
      'search_area'        → /mission/command (CMD_SEARCH_AREA)
      'start_exploration'  → /mission/command (CMD_START_EXPLORATION)
      None/unknown         → log warning + Acknowledge + error message
  → Acknowledge response sent back
```

## Transmit Flow (Rover → CM)

```
/mission/result, /gps/fix, /rtk/status, /stm/imu/data, /stm/wheel_odom
  → Build ResponseEnvelope:
      'acknowledge'    — after every RequestEnvelope
      'task_finished'  — from /mission/result signal
      'gps_coordinate' — from /gps/fix (RTK quality checked)
      'distance'       — from /stm/wheel_odom integration
      'rover_status'   — periodic ≤1 Hz (timer)
      'message'        — error/status text
  → envelope.SerializeToString()
  → cobs.cobs.encode(bytes) + b"\x00"
  → serial.write()
```

## Command Dispatch Table

| RSCP Command | Bridge Action | Internal Topic | Response |
|---|---|---|---|
| `set_stage(v)` | Set stage | `/rscp/current_stage` + `/rscp/command` | `Acknowledge` |
| `arm_disarm(true)` | Arm signal | `/rscp/command` (CMD_ARM) | `Acknowledge` |
| `arm_disarm(false)` | Disarm signal | `/rscp/command` (CMD_DISARM) | `Acknowledge` |
| `navigate_to_gps(coord)` | Nav2 goal | `/mission/command` | `Acknowledge` → `TaskFinished` |
| `search_area(center,r)` | Search task | `/mission/command` | `Acknowledge` → `GPSCoordinate` → `TaskFinished` |
| `start_exploration` | Explore mode | `/mission/command` | `Acknowledge` → `distance` → `TaskFinished` |

## ArmDisarm Safety

- `arm_disarm.value = true` → `/rscp/command` CMD_ARM → safety_mux arm gate opens → stm_bridge sends AUTONOMOUS to H723
- `arm_disarm.value = false` → `/rscp/command` CMD_DISARM → safety_mux disarm → stm_bridge sends DISARM to H723 → motors stop
- `value_wrapper` oneof must be checked: no wrapper → error log

## Error Recovery

| Error | Action |
|---|---|
| COBS decode exception | Drop frame + log warning + continue |
| Protobuf parse exception | Drop frame + log warning + continue |
| Serial port error | Reconnect loop + log |
| Unknown request type | Log warning + Acknowledge + error message |
| Partial frame timeout | Clear buffer + log |

**No error causes node crash.**

## RoverStatus (≤1 Hz)

Sources:
- `/rtk/status` + `/gps/fix` → coordinate + fix quality
- `/stm/imu/data` → heading (yaw)
- Battery: **H723 gap** — no battery telemetry available. Temporary: `voltage=0, state_of_charge=NaN` + error log until H723 firmware adds battery support.

## Testing

- Fake RSCP module: `io.BytesIO` with recorded frames from `examples/python/`
- Each command type tested: set_stage(1), arm_disarm(true), navigate_to_gps, search_area, start_exploration
- Unknown command test
- COBS framing error test (corrupted frame)
- Serial disconnect/reconnect test
- **Motor NOT connected**

## Files

```
src/earendil_rscp_bridge/
├── earendil_rscp_bridge/
│   ├── __init__.py
│   ├── rscp_bridge_node.py    # Main ROS node
│   └── rscp_parser.py         # COBS + protobuf parse/produce
├── config/
│   └── rscp_bridge_params.yaml
├── launch/
│   └── rscp_bridge.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```
