#!/usr/bin/env python3
"""
Fake RSCP Module — simulates ARC Competition Module for testing.

Generates COBS+protobuf frames for all 5 command types, plus edge cases.
Can be used with io.BytesIO (no serial) or with a real serial port for HIL testing.

Usage:
  # Standalone test (no serial):
  python3 test/rscp_fake_module.py

  # With rscp_bridge via loopback serial (e.g., socat):
  # socat -d -d pty,raw,echo=0,link=/tmp/rscp_fake pty,raw,echo=0,link=/tmp/rscp_real
  # Then: rscp_bridge serial_port=/tmp/rscp_real
  # And:  python3 test/rscp_fake_module.py --serial /tmp/rscp_fake
"""

import sys
import os
import io
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_rscp_bridge'))

import cobs.cobs
import rscp_protobuf
from earendil_rscp_bridge.rscp_parser import FrameReader, decode_frame, parse_request


def create_set_stage(stage: int) -> bytes:
    """Create a SetStage RequestEnvelope."""
    req = rscp_protobuf.RequestEnvelope()
    req.set_stage.value = stage
    return cobs.cobs.encode(req.SerializeToString()) + b'\x00'


def create_arm_disarm(arm: bool) -> bytes:
    """Create an ArmDisarm RequestEnvelope."""
    req = rscp_protobuf.RequestEnvelope()
    req.arm_disarm.value = arm
    return cobs.cobs.encode(req.SerializeToString()) + b'\x00'


def create_navigate_to_gps(lat: float, lon: float, alt: float) -> bytes:
    """Create a NavigateToGPS RequestEnvelope."""
    req = rscp_protobuf.RequestEnvelope()
    req.navigate_to_gps.coordinate.latitude = lat
    req.navigate_to_gps.coordinate.longitude = lon
    req.navigate_to_gps.coordinate.altitude = alt
    return cobs.cobs.encode(req.SerializeToString()) + b'\x00'


def create_search_area(lat: float, lon: float, radius: float) -> bytes:
    """Create a SearchArea RequestEnvelope."""
    req = rscp_protobuf.RequestEnvelope()
    req.search_area.center_coordinate.latitude = lat
    req.search_area.center_coordinate.longitude = lon
    req.search_area.center_coordinate.altitude = 0.0
    req.search_area.radius = radius
    return cobs.cobs.encode(req.SerializeToString()) + b'\x00'


def create_start_exploration() -> bytes:
    """Create a StartExploration RequestEnvelope."""
    req = rscp_protobuf.RequestEnvelope()
    req.start_exploration.dummy_field = True
    return cobs.cobs.encode(req.SerializeToString()) + b'\x00'


def create_empty_request() -> bytes:
    """Create an empty RequestEnvelope (no field set — test edge case)."""
    req = rscp_protobuf.RequestEnvelope()
    return cobs.cobs.encode(req.SerializeToString()) + b'\x00'


def create_corrupted_frame() -> bytes:
    """Create a corrupted COBS frame (invalid COBS encoding)."""
    return b'\xff\xff\xff\x00'


# ── ARC Stage 1 Test Sequence ───────────────────────────────────────────────

def stage1_sequence():
    """Generate the full Stage 1 (Antenna Installation) command sequence."""
    return [
        ('SetStage(1)', create_set_stage(1)),
        ('ArmDisarm(true)', create_arm_disarm(True)),
        ('SearchArea(39.92, 32.85, 50)', create_search_area(39.92, 32.85, 50.0)),
    ]


def stage2_sequence():
    """Generate the full Stage 2 (Shackleton Crater) command sequence."""
    return [
        ('SetStage(2)', create_set_stage(2)),
        ('SearchArea(39.93, 32.86, 30)', create_search_area(39.93, 32.86, 30.0)),
    ]


def stage3_sequence():
    """Generate the full Stage 3 (Lava Tube) command sequence."""
    return [
        ('SetStage(3)', create_set_stage(3)),
        ('NavigateToGPS(39.94, 32.87, 890)', create_navigate_to_gps(39.94, 32.87, 890.0)),
        ('StartExploration', create_start_exploration()),
    ]


def stage4_sequence():
    """Generate the full Stage 4 (Return to Airlock) command sequence."""
    return [
        ('SetStage(4)', create_set_stage(4)),
        ('NavigateToGPS(39.95, 32.88, 880)', create_navigate_to_gps(39.95, 32.88, 880.0)),
        ('ArmDisarm(false)', create_arm_disarm(False)),
    ]


def edge_cases():
    """Edge case test frames."""
    return [
        ('Empty request', create_empty_request()),
        ('Corrupted COBS', create_corrupted_frame()),
        ('SetStage(0)', create_set_stage(0)),  # stage 0 = idle
        ('SetStage(255)', create_set_stage(255)),  # out of range
    ]


def all_commands():
    """All 5 command types + edge cases."""
    return [
        ('SetStage(1)', create_set_stage(1)),
        ('ArmDisarm(true)', create_arm_disarm(True)),
        ('NavigateToGPS(39.92, 32.85, 890)', create_navigate_to_gps(39.92, 32.85, 890.0)),
        ('SearchArea(39.93, 32.86, 50)', create_search_area(39.93, 32.86, 50.0)),
        ('StartExploration', create_start_exploration()),
    ] + edge_cases()


def parse_response(data: bytes):
    """Parse a ResponseEnvelope from bytes."""
    resp = rscp_protobuf.ResponseEnvelope()
    resp.ParseFromString(data)
    field = resp.WhichOneof('response')
    if field == 'acknowledge':
        return 'Acknowledge'
    elif field == 'task_finished':
        return 'TaskFinished'
    elif field == 'gps_coordinate':
        c = resp.gps_coordinate
        return f'GPSCoordinate({c.latitude:.6f}, {c.longitude:.6f}, {c.altitude:.1f})'
    elif field == 'distance':
        return f'Distance({resp.distance:.2f}m)'
    elif field == 'message':
        return f'Message("{resp.message}")'
    elif field == 'rover_status':
        s = resp.rover_status
        return f'RoverStatus(state={s.state}, lat={s.coordinate.latitude:.4f}, heading={s.heading:.1f})'
    else:
        return f'Unknown({field})'


def test_with_bytesio():
    """Test fake module by processing frames through FrameReader (simulates serial)."""
    print('=== Testing with io.BytesIO (no serial) ===\n')

    all_frames = all_commands()

    for name, frame_bytes in all_frames:
        print(f'  Sending: {name}')

        # Feed byte-by-byte through FrameReader
        reader = FrameReader()
        received_frame = None
        for b in frame_bytes:
            result = reader.feed(b)
            if result is not None:
                received_frame = result

        if received_frame is None:
            print(f'    ERROR: No frame received (corrupted: {name == "Corrupted COBS"})')
            if name == 'Corrupted COBS':
                print(f'    EXPECTED: corrupted frame should not produce valid output')
            continue

        # COBS decode
        try:
            decoded = decode_frame(received_frame)
        except Exception as e:
            if 'Corrupted' in name:
                print(f'    EXPECTED: COBS decode error for corrupted frame')
            else:
                print(f'    ERROR: COBS decode failed: {e}')
            continue

        # Protobuf parse
        _, field_name, cmd = parse_request(decoded)

        if field_name is None:
            if 'Empty' in name:
                print(f'    EXPECTED: empty request has no field set')
            else:
                print(f'    ERROR: no field set')
            continue

        if cmd is None:
            print(f'    ERROR: command extraction failed')
            continue

        print(f'    Received: {field_name} → command_type={cmd.command_type}')

    print('\n=== Stage Sequences ===\n')

    sequences = [
        ('Stage 1 — Antenna Installation', stage1_sequence()),
        ('Stage 2 — Shackleton Crater', stage2_sequence()),
        ('Stage 3 — Lava Tube', stage3_sequence()),
        ('Stage 4 — Return to Airlock', stage4_sequence()),
    ]

    for seq_name, frames in sequences:
        print(f'  {seq_name}:')
        for name, frame_bytes in frames:
            reader = FrameReader()
            for b in frame_bytes:
                result = reader.feed(b)
                if result is not None:
                    decoded = decode_frame(result)
                    _, fn, cmd = parse_request(decoded)
                    print(f'    {name} → {fn}')
        print()

    print('=== Fake module test complete ===')


def test_with_serial(port: str, baud: int = 115200):
    """Test fake module by sending frames over a real serial port."""
    import serial
    import time

    print(f'=== Testing with serial port: {port} (baud={baud}) ===\n')

    all_frames = all_commands()

    with serial.Serial(port, baud, timeout=1) as ser:
        for name, frame_bytes in all_frames:
            print(f'  Sending: {name}')
            ser.write(frame_bytes)
            time.sleep(0.1)

            # Try to read response
            response_data = b''
            start = time.time()
            while time.time() - start < 1.0:
                byte = ser.read(1)
                if byte == b'\x00':
                    break
                response_data += byte

            if response_data:
                try:
                    decoded = cobs.cobs.decode(response_data)
                    print(f'    Response: {parse_response(decoded)}')
                except Exception as e:
                    print(f'    Response decode error: {e}')
            else:
                print(f'    No response (bridge may not be running)')

        # Send stage sequences
        print('\n  Stage sequences:')
        for seq_name, frames in [('S1', stage1_sequence()), ('S2', stage2_sequence()),
                                  ('S3', stage3_sequence()), ('S4', stage4_sequence())]:
            print(f'    {seq_name}:')
            for name, frame_bytes in frames:
                ser.write(frame_bytes)
                time.sleep(0.1)
                response_data = b''
                start = time.time()
                while time.time() - start < 1.0:
                    byte = ser.read(1)
                    if byte == b'\x00':
                        break
                    response_data += byte
                if response_data:
                    try:
                        decoded = cobs.cobs.decode(response_data)
                        print(f'      {name} → {parse_response(decoded)}')
                    except Exception:
                        print(f'      {name} → (decode error)')
                else:
                    print(f'      {name} → (no response)')

    print('\n=== Serial test complete ===')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fake RSCP Module')
    parser.add_argument('--serial', type=str, help='Serial port for HIL testing')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    if args.serial:
        test_with_serial(args.serial, args.baud)
    else:
        test_with_bytesio()
