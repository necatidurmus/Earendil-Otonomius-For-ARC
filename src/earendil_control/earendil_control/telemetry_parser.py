#!/usr/bin/env python3
"""
Telemetry Parser — parse H723 ASCII telemetry lines into ROS messages.

H723 sends telemetry over USART3 in ASCII format:
  Motor: FL|RPM:...,T:...,D:...,DIR:...,APP_PH:...,SP:...,BRAKE:...,FC:...,H:...,PWM_SET:...,PWM_ACT:...,QDROP:...,RXB:...\r\n
  IMU:   IMU|AX:...,AY:...,AZ:...,GX:...,GY:...,GZ:...\r\n
  MAG:   MAG|MX:...,MY:...,MZ:...\r\n
  Mode:  MODE:DISARM|MANUAL|AUTONOMOUS\r\n
  ACK:   ACK\r\n
  NACK:  NACK\r\n

Units (from firmware):
  - Accelerometer (MPU9250): mg → convert to m/s² (÷1000×9.80665)
  - Gyroscope (MPU9250): mdps or dps → convert to rad/s
  - Magnetometer (QMC5883P): raw → convert to µT

Reference: stm codes/earendil-mainfirmware/, AGENT.md §4
"""

import re
import math
import logging
from dataclasses import dataclass, field
from typing import Optional, Dict

logger = logging.getLogger(__name__)

# ── Constants ───────────────────────────────────────────────────────────────

MG_TO_MS2 = 9.80665 / 1000.0  # mg → m/s²

# Motor prefixes
MOTOR_PREFIXES = {'FL': 0, 'FR': 1, 'RL': 2, 'RR': 3}
MOTOR_NAMES = ['FL', 'FR', 'RL', 'RR']

# Operating modes
MODE_DISARMED = 0
MODE_MANUAL = 1
MODE_AUTONOMOUS = 2

MODE_MAP = {
    'DISARM': MODE_DISARMED,
    'MANUAL': MODE_MANUAL,
    'AUTONOMOUS': MODE_AUTONOMOUS,
}


# ── Data classes ────────────────────────────────────────────────────────────

@dataclass
class MotorTelemetry:
    """Parsed motor telemetry from F411 via H723."""
    motor_id: int  # 0=FL, 1=FR, 2=RL, 3=RR
    rpm: float = 0.0
    temperature: float = 0.0
    duty: float = 0.0
    direction: int = 0
    applied_phase: int = 0
    setpoint: float = 0.0
    brake: int = 0
    fault_code: int = 0
    hall_state: int = 0
    pwm_set: float = 0.0
    pwm_actual: float = 0.0
    queue_drop: int = 0
    rx_buffer: int = 0


@dataclass
class ImuData:
    """Parsed IMU data from MPU9250 via H723."""
    accel_x: float = 0.0  # m/s² (ENU)
    accel_y: float = 0.0  # m/s²
    accel_z: float = 0.0  # m/s²
    gyro_x: float = 0.0   # rad/s (ENU)
    gyro_y: float = 0.0   # rad/s
    gyro_z: float = 0.0   # rad/s


@dataclass
class MagData:
    """Parsed magnetometer data from QMC5883P via H723."""
    mag_x: float = 0.0  # µT
    mag_y: float = 0.0  # µT
    mag_z: float = 0.0  # µT


@dataclass
class TelemetryFrame:
    """A complete parsed telemetry frame from H723."""
    motors: Dict[int, MotorTelemetry] = field(default_factory=dict)
    imu: Optional[ImuData] = None
    mag: Optional[MagData] = None
    mode: Optional[int] = None  # MODE_* constant
    ack: bool = False
    nack: bool = False
    raw_line: str = ''


# ── Parser ──────────────────────────────────────────────────────────────────

# Motor telemetry pattern: FL|RPM:123,T:45,D:67,DIR:1,APP_PH:0,SP:123,BRAKE:0,FC:0,H:5,PWM_SET:50,PWM_ACT:48,QDROP:0,RXB:128
MOTOR_PATTERN = re.compile(
    r'^(FL|FR|RL|RR)\|'
    r'RPM:([^,]+),T:([^,]+),D:([^,]+),DIR:([^,]+),APP_PH:([^,]+),'
    r'SP:([^,]+),BRAKE:([^,]+),FC:([^,]+),H:([^,]+),'
    r'PWM_SET:([^,]+),PWM_ACT:([^,]+),QDROP:([^,]+),RXB:([^,]+)$'
)

# IMU pattern: IMU|AX:1234,AY:5678,AZ:9012,GX:123,GY:456,GZ:789
IMU_PATTERN = re.compile(
    r'^IMU\|'
    r'AX:([^,]+),AY:([^,]+),AZ:([^,]+),'
    r'GX:([^,]+),GY:([^,]+),GZ:([^,]+)$'
)

# MAG pattern: MAG|MX:123,MY:456,MZ:789
MAG_PATTERN = re.compile(
    r'^MAG\|'
    r'MX:([^,]+),MY:([^,]+),MZ:([^,]+)$'
)

# Mode pattern: MODE:DISARM or MODE:MANUAL or MODE:AUTONOMOUS
MODE_PATTERN = re.compile(r'^MODE:(DISARM|MANUAL|AUTONOMOUS)$')


def parse_line(line: str) -> TelemetryFrame:
    """Parse a single telemetry line from H723.

    Args:
        line: Raw ASCII line (stripped of \r\n)

    Returns:
        TelemetryFrame with parsed data
    """
    frame = TelemetryFrame(raw_line=line)
    line = line.strip()

    if not line:
        return frame

    # ACK
    if line == 'ACK':
        frame.ack = True
        return frame

    # NACK
    if line == 'NACK':
        frame.nack = True
        return frame

    # Mode
    m = MODE_PATTERN.match(line)
    if m:
        frame.mode = MODE_MAP.get(m.group(1), MODE_DISARMED)
        return frame

    # IMU
    m = IMU_PATTERN.match(line)
    if m:
        try:
            # H723 sends mg for accel, dps or mdps for gyro
            # Convert to SI: mg→m/s², dps→rad/s (or mdps→rad/s)
            ax_mg = float(m.group(1))
            ay_mg = float(m.group(2))
            az_mg = float(m.group(3))
            gx_raw = float(m.group(4))
            gy_raw = float(m.group(5))
            gz_raw = float(m.group(6))

            frame.imu = ImuData(
                accel_x=ax_mg * MG_TO_MS2,
                accel_y=ay_mg * MG_TO_MS2,
                accel_z=az_mg * MG_TO_MS2,
                # TODO: Confirm if gyro is dps or mdps from H723 firmware
                # Assuming dps for now (common MPU9250 setting)
                gyro_x=math.radians(gx_raw),
                gyro_y=math.radians(gy_raw),
                gyro_z=math.radians(gz_raw),
            )
        except (ValueError, IndexError) as e:
            logger.warning("IMU parse error: %s — line: %s", e, line)
        return frame

    # Magnetometer
    m = MAG_PATTERN.match(line)
    if m:
        try:
            frame.mag = MagData(
                mag_x=float(m.group(1)),
                mag_y=float(m.group(2)),
                mag_z=float(m.group(3)),
            )
        except (ValueError, IndexError) as e:
            logger.warning("MAG parse error: %s — line: %s", e, line)
        return frame

    # Motor telemetry
    m = MOTOR_PATTERN.match(line)
    if m:
        try:
            prefix = m.group(1)
            motor_id = MOTOR_PREFIXES[prefix]
            mt = MotorTelemetry(motor_id=motor_id)
            mt.rpm = float(m.group(2))
            mt.temperature = float(m.group(3))
            mt.duty = float(m.group(4))
            mt.direction = int(m.group(5))
            mt.applied_phase = int(m.group(6))
            mt.setpoint = float(m.group(7))
            mt.brake = int(m.group(8))
            mt.fault_code = int(m.group(9))
            mt.hall_state = int(m.group(10))
            mt.pwm_set = float(m.group(11))
            mt.pwm_actual = float(m.group(12))
            mt.queue_drop = int(m.group(13))
            mt.rx_buffer = int(m.group(14))
            frame.motors[motor_id] = mt
        except (ValueError, IndexError, KeyError) as e:
            logger.warning("Motor telemetry parse error: %s — line: %s", e, line)
        return frame

    # Unknown line
    logger.debug("Unknown telemetry line: %s", line)
    return frame


def parse_buffer(buffer: str) -> list:
    """Parse a buffer of multiple telemetry lines.

    Args:
        buffer: Raw ASCII buffer with \\r\\n separated lines

    Returns:
        List of TelemetryFrame objects
    """
    frames = []
    for line in buffer.split('\n'):
        line = line.strip()
        if line:
            frames.append(parse_line(line))
    return frames
