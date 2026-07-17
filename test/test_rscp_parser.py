#!/usr/bin/env python3
"""
Tests for earendil_rscp_bridge.rscp_parser — COBS + protobuf parsing.

Run: python3 test/test_rscp_parser.py
Requires: rscp_protobuf, cobs
"""

import sys
import os

# Add package to path for testing without colcon install
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_rscp_bridge'))

import cobs.cobs
import rscp_protobuf
from earendil_rscp_bridge.rscp_parser import (
    FrameReader,
    decode_frame,
    encode_frame,
    parse_request,
    create_acknowledge,
    create_task_finished,
    create_gps_coordinate,
    create_distance,
    create_message,
    create_rover_status,
    CMD_SET_STAGE,
    CMD_ARM,
    CMD_DISARM,
    CMD_NAVIGATE_TO_GPS,
    CMD_SEARCH_AREA,
    CMD_START_EXPLORATION,
)

passed = 0
failed = 0


def test(name, condition):
    global passed, failed
    if condition:
        passed += 1
        print(f'  PASS: {name}')
    else:
        failed += 1
        print(f'  FAIL: {name}')


def run_tests():
    global passed, failed

    print('=== COBS Framing ===')

    # COBS round-trip
    data = b'hello rscp protocol'
    encoded = encode_frame(data)
    test('encode adds 0x00 delimiter', encoded[-1:] == b'\x00')
    test('decode round-trip', decode_frame(encoded[:-1]) == data)

    # Empty data
    empty_encoded = encode_frame(b'')
    test('empty data encode/decode', decode_frame(empty_encoded[:-1]) == b'')

    # Data with 0x00 inside
    data_with_zero = b'hello\x00world'
    encoded_z = encode_frame(data_with_zero)
    test('data with 0x00 round-trip', decode_frame(encoded_z[:-1]) == data_with_zero)

    # Corrupted COBS
    try:
        decode_frame(b'\xff\xff\xff')
        test('corrupted COBS raises', False)
    except Exception:
        test('corrupted COBS raises', True)

    print('\n=== Response Production ===')

    # Acknowledge
    ack = create_acknowledge()
    da = rscp_protobuf.ResponseEnvelope()
    da.ParseFromString(ack)
    test('Acknowledge type', da.WhichOneof('response') == 'acknowledge')

    # TaskFinished
    tf = create_task_finished()
    dtf = rscp_protobuf.ResponseEnvelope()
    dtf.ParseFromString(tf)
    test('TaskFinished type', dtf.WhichOneof('response') == 'task_finished')

    # GPSCoordinate
    gps = create_gps_coordinate(39.123, 32.456, 950.0)
    dg = rscp_protobuf.ResponseEnvelope()
    dg.ParseFromString(gps)
    test('GPSCoordinate type', dg.WhichOneof('response') == 'gps_coordinate')
    test('GPSCoordinate lat', abs(dg.gps_coordinate.latitude - 39.123) < 1e-6)
    test('GPSCoordinate lon', abs(dg.gps_coordinate.longitude - 32.456) < 1e-6)
    test('GPSCoordinate alt', abs(dg.gps_coordinate.altitude - 950.0) < 1e-3)

    # Distance
    d = create_distance(42.5)
    dd = rscp_protobuf.ResponseEnvelope()
    dd.ParseFromString(d)
    test('Distance type', dd.WhichOneof('response') == 'distance')
    test('Distance value', abs(dd.distance - 42.5) < 1e-6)

    # Message
    msg = create_message('test error text')
    dm = rscp_protobuf.ResponseEnvelope()
    dm.ParseFromString(msg)
    test('Message type', dm.WhichOneof('response') == 'message')
    test('Message text', dm.message == 'test error text')

    # RoverStatus — AUTONOMOUS
    rs = create_rover_status(1, 39.0, 32.0, 100.0, 45.0, 12.6, 1.5, 0.85)
    drs = rscp_protobuf.ResponseEnvelope()
    drs.ParseFromString(rs)
    test('RoverStatus type', drs.WhichOneof('response') == 'rover_status')
    test('RoverStatus state AUTONOMOUS', drs.rover_status.state == 1)
    test('RoverStatus coordinate', abs(drs.rover_status.coordinate.latitude - 39.0) < 1e-6)
    test('RoverStatus heading', abs(drs.rover_status.heading - 45.0) < 1e-3)
    test('RoverStatus battery', abs(drs.rover_status.battery_state.voltage - 12.6) < 1e-3)

    # RoverStatus — DISARMED
    rs_d = create_rover_status(0)
    drs_d = rscp_protobuf.ResponseEnvelope()
    drs_d.ParseFromString(rs_d)
    test('RoverStatus DISARMED', drs_d.rover_status.state == 0)

    print('\n=== Request Parsing ===')

    # SetStage
    req = rscp_protobuf.RequestEnvelope()
    req.set_stage.value = 2
    _, fn, cmd = parse_request(req.SerializeToString())
    test('SetStage field name', fn == 'set_stage')
    test('SetStage command type', cmd.command_type == CMD_SET_STAGE)
    test('SetStage value', cmd.stage_value == 2)

    # ArmDisarm(true)
    req2 = rscp_protobuf.RequestEnvelope()
    req2.arm_disarm.value = True
    _, fn2, cmd2 = parse_request(req2.SerializeToString())
    test('ArmDisarm(true) field name', fn2 == 'arm_disarm')
    test('ArmDisarm(true) command type', cmd2.command_type == CMD_ARM)
    test('ArmDisarm(true) value', cmd2.arm_value == True)

    # ArmDisarm(false)
    req2b = rscp_protobuf.RequestEnvelope()
    req2b.arm_disarm.value = False
    _, fn2b, cmd2b = parse_request(req2b.SerializeToString())
    test('ArmDisarm(false) command type', cmd2b.command_type == CMD_DISARM)
    test('ArmDisarm(false) value', cmd2b.arm_value == False)

    # NavigateToGPS
    req3 = rscp_protobuf.RequestEnvelope()
    req3.navigate_to_gps.coordinate.latitude = 40.0
    req3.navigate_to_gps.coordinate.longitude = 30.0
    req3.navigate_to_gps.coordinate.altitude = 500.0
    _, fn3, cmd3 = parse_request(req3.SerializeToString())
    test('NavigateToGPS field name', fn3 == 'navigate_to_gps')
    test('NavigateToGPS command type', cmd3.command_type == CMD_NAVIGATE_TO_GPS)
    test('NavigateToGPS lat', abs(cmd3.latitude - 40.0) < 1e-6)
    test('NavigateToGPS lon', abs(cmd3.longitude - 30.0) < 1e-6)
    test('NavigateToGPS alt', abs(cmd3.altitude - 500.0) < 1e-3)

    # SearchArea
    req4 = rscp_protobuf.RequestEnvelope()
    req4.search_area.center_coordinate.latitude = 41.0
    req4.search_area.center_coordinate.longitude = 29.0
    req4.search_area.radius = 50.0
    _, fn4, cmd4 = parse_request(req4.SerializeToString())
    test('SearchArea field name', fn4 == 'search_area')
    test('SearchArea command type', cmd4.command_type == CMD_SEARCH_AREA)
    test('SearchArea lat', abs(cmd4.latitude - 41.0) < 1e-6)
    test('SearchArea radius', abs(cmd4.search_radius - 50.0) < 1e-6)

    # StartExploration
    req5 = rscp_protobuf.RequestEnvelope()
    req5.start_exploration.dummy_field = True
    _, fn5, cmd5 = parse_request(req5.SerializeToString())
    test('StartExploration field name', fn5 == 'start_exploration')
    test('StartExploration command type', cmd5.command_type == CMD_START_EXPLORATION)

    # Empty request
    req6 = rscp_protobuf.RequestEnvelope()
    _, fn6, cmd6 = parse_request(req6.SerializeToString())
    test('Empty request field name', fn6 is None)

    print('\n=== FrameReader ===')

    reader = FrameReader()
    test('FrameReader feed non-zero', reader.feed(0x01) is None)
    test('FrameReader feed non-zero', reader.feed(0x02) is None)
    test('FrameReader feed 0x00', reader.feed(0x00) == b'\x01\x02')
    test('FrameReader consecutive 0x00', reader.feed(0x00) is None)
    test('FrameReader buffer empty', reader.buffer_size == 0)

    # FrameReader reset
    reader.feed(0xAA)
    reader.feed(0xBB)
    reader.reset()
    test('FrameReader reset', reader.buffer_size == 0)

    print('\n=== Full COBS+Protobuf Round-Trip ===')

    # Simulate full pipeline: create request → serialize → COBS encode → byte-by-byte → decode → parse
    for stage_val in [1, 2, 3, 4]:
        req_full = rscp_protobuf.RequestEnvelope()
        req_full.set_stage.value = stage_val
        serialized = req_full.SerializeToString()
        cobs_encoded = cobs.cobs.encode(serialized) + b'\x00'

        reader2 = FrameReader()
        frame_out = None
        for b in cobs_encoded:
            result = reader2.feed(b)
            if result is not None:
                frame_out = result

        test(f'Full round-trip stage={stage_val} frame received', frame_out is not None)
        decoded = decode_frame(frame_out)
        _, fn, cmd = parse_request(decoded)
        test(f'Full round-trip stage={stage_val} parsed', fn == 'set_stage' and cmd.stage_value == stage_val)

    # ArmDisarm full round-trip
    req_arm = rscp_protobuf.RequestEnvelope()
    req_arm.arm_disarm.value = True
    cobs_arm = cobs.cobs.encode(req_arm.SerializeToString()) + b'\x00'
    reader3 = FrameReader()
    frame_arm = None
    for b in cobs_arm:
        result = reader3.feed(b)
        if result is not None:
            frame_arm = result
    _, fn_arm, cmd_arm = parse_request(decode_frame(frame_arm))
    test('Full round-trip arm_disarm', fn_arm == 'arm_disarm' and cmd_arm.command_type == CMD_ARM)

    # NavigateToGPS full round-trip
    req_nav = rscp_protobuf.RequestEnvelope()
    req_nav.navigate_to_gps.coordinate.latitude = 39.92
    req_nav.navigate_to_gps.coordinate.longitude = 32.85
    req_nav.navigate_to_gps.coordinate.altitude = 890.0
    cobs_nav = cobs.cobs.encode(req_nav.SerializeToString()) + b'\x00'
    reader4 = FrameReader()
    frame_nav = None
    for b in cobs_nav:
        result = reader4.feed(b)
        if result is not None:
            frame_nav = result
    _, fn_nav, cmd_nav = parse_request(decode_frame(frame_nav))
    test('Full round-trip navigate_to_gps', fn_nav == 'navigate_to_gps' and abs(cmd_nav.latitude - 39.92) < 1e-6)

    print(f'\n{"="*40}')
    print(f'Results: {passed} passed, {failed} failed, {passed + failed} total')

    if failed > 0:
        print('SOME TESTS FAILED!')
        sys.exit(1)
    else:
        print('ALL TESTS PASSED!')
        sys.exit(0)


if __name__ == '__main__':
    run_tests()
