#!/usr/bin/env python3
"""
RSCP Parser — COBS decode/encode + protobuf parse/produce for RSCP protocol.

Uses official rscp_protobuf + cobs packages.
Manual serialize/deserialize is NOT used — only official proto + generated classes.

Reference: rscp protokol/rscp/proto/rscp.proto (single source of truth)
NOTE: README says TaskCompleted, proto says TaskFinished — always use proto.
"""

import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cobs.cobs
import rscp_protobuf

logger = logging.getLogger(__name__)

# ── Command type constants (match RscpCommand.msg) ──────────────────────────

CMD_SET_STAGE = 0
CMD_ARM = 1
CMD_DISARM = 2
CMD_NAVIGATE_TO_GPS = 3
CMD_SEARCH_AREA = 4
CMD_START_EXPLORATION = 5

# ── RoverState mapping ──────────────────────────────────────────────────────
# RSCP RoverState enum: DISARMED=0, AUTONOMOUS=1, MANUAL=2
# H723 RoverMode_t:     DISARM=0,   MANUAL=1,      AUTONOMOUS=2  (DIFFERENT!)
# This mapping is used by stm_bridge, not here, but defined for reference.

ROVER_STATE_DISARMED = 0
ROVER_STATE_AUTONOMOUS = 1
ROVER_STATE_MANUAL = 2


@dataclass
class ParsedCommand:
    """Result of parsing a RequestEnvelope."""
    command_type: int  # CMD_* constant
    stage_value: int = 0
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    search_radius: float = 0.0
    arm_value: Optional[bool] = None  # None if not arm_disarm


# ── COBS framing ────────────────────────────────────────────────────────────

def decode_frame(raw_bytes: bytes) -> bytes:
    """Decode a COBS-encoded frame (0x00 delimiter already stripped).

    Args:
        raw_bytes: COBS-encoded bytes (without trailing 0x00)

    Returns:
        Decoded protobuf bytes

    Raises:
        cobs.DecodeError: if COBS decoding fails
    """
    return cobs.cobs.decode(raw_bytes)


def encode_frame(protobuf_bytes: bytes) -> bytes:
    """Encode protobuf bytes into COBS frame with 0x00 delimiter.

    Args:
        protobuf_bytes: Serialized protobuf message

    Returns:
        COBS-encoded bytes + 0x00 delimiter
    """
    return cobs.cobs.encode(protobuf_bytes) + b"\x00"


# ── Request parsing ─────────────────────────────────────────────────────────

def parse_request(decoded_bytes: bytes) -> Tuple[Optional[object], Optional[str], Optional[ParsedCommand]]:
    """Parse decoded bytes into a RequestEnvelope and extract command data.

    Args:
        decoded_bytes: Decoded protobuf bytes (after COBS decode)

    Returns:
        Tuple of (envelope, field_name, parsed_command)
        - envelope: the raw RequestEnvelope (for inspection)
        - field_name: one of 'arm_disarm', 'set_stage', 'navigate_to_gps',
          'search_area', 'start_exploration', or None if unknown
        - parsed_command: ParsedCommand with extracted data, or None on error
    """
    try:
        envelope = rscp_protobuf.RequestEnvelope()
        envelope.ParseFromString(decoded_bytes)
    except Exception as e:
        logger.error("Protobuf parse error: %s", e)
        return None, None, None

    field_name = envelope.WhichOneof('request')

    if field_name is None:
        logger.warning("RequestEnvelope has no request field set")
        return envelope, None, None

    try:
        cmd = _extract_command(envelope, field_name)
        return envelope, field_name, cmd
    except Exception as e:
        logger.error("Command extraction error for '%s': %s", field_name, e)
        return envelope, field_name, None


def _extract_command(envelope, field_name: str) -> ParsedCommand:
    """Extract command data from a parsed RequestEnvelope."""
    if field_name == 'arm_disarm':
        arm_msg = envelope.arm_disarm
        # ArmDisarm has oneof value_wrapper { bool value = 1 }
        which = arm_msg.WhichOneof('value_wrapper')
        if which is None:
            logger.warning("ArmDisarm: value_wrapper not set")
            return ParsedCommand(command_type=CMD_ARM, arm_value=None)
        arm_value = arm_msg.value
        cmd_type = CMD_ARM if arm_value else CMD_DISARM
        return ParsedCommand(command_type=cmd_type, arm_value=arm_value)

    elif field_name == 'set_stage':
        return ParsedCommand(
            command_type=CMD_SET_STAGE,
            stage_value=envelope.set_stage.value,
        )

    elif field_name == 'navigate_to_gps':
        coord = envelope.navigate_to_gps.coordinate
        return ParsedCommand(
            command_type=CMD_NAVIGATE_TO_GPS,
            latitude=coord.latitude,
            longitude=coord.longitude,
            altitude=coord.altitude,
        )

    elif field_name == 'search_area':
        sa = envelope.search_area
        return ParsedCommand(
            command_type=CMD_SEARCH_AREA,
            latitude=sa.center_coordinate.latitude,
            longitude=sa.center_coordinate.longitude,
            altitude=sa.center_coordinate.altitude,
            search_radius=sa.radius,
        )

    elif field_name == 'start_exploration':
        return ParsedCommand(command_type=CMD_START_EXPLORATION)

    else:
        logger.warning("Unknown request field: '%s'", field_name)
        return ParsedCommand(command_type=-1)


# ── Response production ─────────────────────────────────────────────────────

def create_acknowledge() -> bytes:
    """Create an Acknowledge ResponseEnvelope.

    NOTE: 'acknowledge' and 'message' are in the same oneof — cannot combine.
    For error messages, send Acknowledge first, then a separate message response.

    Returns:
        Serialized bytes ready for COBS encoding
    """
    resp = rscp_protobuf.ResponseEnvelope()
    resp.acknowledge.CopyFrom(rscp_protobuf.Acknowledge())
    return resp.SerializeToString()


def create_task_finished() -> bytes:
    """Create a TaskFinished ResponseEnvelope.

    Returns:
        Serialized bytes ready for COBS encoding
    """
    resp = rscp_protobuf.ResponseEnvelope()
    resp.task_finished.CopyFrom(rscp_protobuf.TaskFinished())
    return resp.SerializeToString()


def create_gps_coordinate(latitude: float, longitude: float, altitude: float) -> bytes:
    """Create a GPSCoordinate ResponseEnvelope.

    Args:
        latitude: degrees
        longitude: degrees
        altitude: metres (EGM96 geoid)

    Returns:
        Serialized bytes ready for COBS encoding
    """
    resp = rscp_protobuf.ResponseEnvelope()
    resp.gps_coordinate.latitude = latitude
    resp.gps_coordinate.longitude = longitude
    resp.gps_coordinate.altitude = altitude
    return resp.SerializeToString()


def create_distance(distance_m: float) -> bytes:
    """Create a distance ResponseEnvelope.

    Args:
        distance_m: measured distance in metres

    Returns:
        Serialized bytes ready for COBS encoding
    """
    resp = rscp_protobuf.ResponseEnvelope()
    resp.distance = distance_m
    return resp.SerializeToString()


def create_message(text: str) -> bytes:
    """Create a message ResponseEnvelope.

    Args:
        text: message string

    Returns:
        Serialized bytes ready for COBS encoding
    """
    resp = rscp_protobuf.ResponseEnvelope()
    resp.message = text
    return resp.SerializeToString()


def create_rover_status(
    state: int,
    latitude: float = 0.0,
    longitude: float = 0.0,
    altitude: float = 0.0,
    heading: float = 0.0,
    battery_voltage: float = 0.0,
    battery_current: float = 0.0,
    battery_soc: float = 0.0,
) -> bytes:
    """Create a RoverStatus ResponseEnvelope (≤1 Hz).

    Args:
        state: RoverState enum (0=DISARMED, 1=AUTONOMOUS, 2=MANUAL)
        latitude, longitude, altitude: GPS position
        heading: compass heading in degrees
        battery_voltage: volts (0 if unknown — H723 gap)
        battery_current: amps (0 if unknown)
        battery_soc: state of charge 0..1 (0 if unknown)

    Returns:
        Serialized bytes ready for COBS encoding
    """
    resp = rscp_protobuf.ResponseEnvelope()
    status = rscp_protobuf.RoverStatus()

    # Map state value to RoverState enum
    if state == 0:
        status.state = rscp_protobuf.RoverState.DISARMED
    elif state == 1:
        status.state = rscp_protobuf.RoverState.AUTONOMOUS
    elif state == 2:
        status.state = rscp_protobuf.RoverState.MANUAL
    else:
        logger.warning("Unknown RoverState value: %d, defaulting to DISARMED", state)
        status.state = rscp_protobuf.RoverState.DISARMED

    status.coordinate.latitude = latitude
    status.coordinate.longitude = longitude
    status.coordinate.altitude = altitude
    status.heading = heading

    status.battery_state.voltage = battery_voltage
    status.battery_state.current = battery_current
    status.battery_state.state_of_charge = battery_soc

    resp.rover_status.CopyFrom(status)
    return resp.SerializeToString()


# ── Frame reader (byte-by-byte with 0x00 delimiter) ─────────────────────────

class FrameReader:
    """Reads COBS frames byte-by-byte from a serial stream.

    Usage:
        reader = FrameReader()
        for byte in serial.read(1):
            frame = reader.feed(byte)
            if frame is not None:
                # Process complete frame
    """

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, byte: int) -> Optional[bytes]:
        """Feed a single byte. Returns complete frame bytes when 0x00 is received.

        Args:
            byte: single byte (0-255)

        Returns:
            Complete COBS-encoded frame (without 0x00 delimiter), or None if frame incomplete
        """
        if byte == 0x00:
            if len(self._buffer) == 0:
                # Empty frame (consecutive 0x00), skip
                return None
            frame = bytes(self._buffer)
            self._buffer.clear()
            return frame
        else:
            self._buffer.append(byte)
            return None

    def reset(self):
        """Clear the buffer (e.g., on timeout or error)."""
        self._buffer.clear()

    @property
    def buffer_size(self) -> int:
        return len(self._buffer)
