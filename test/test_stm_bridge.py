#!/usr/bin/env python3
"""
Test stm_bridge skid-steer kinematics and telemetry integration.

Tests kinematics logic without ROS (extracted math) and verifies
telemetry parser integration with stm_bridge's expected patterns.

Usage:
  python3 test/test_stm_bridge.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_rscp_bridge'))

# ── Import telemetry parser ──────────────────────────────────────────────────
from earendil_control.telemetry_parser import (
    parse_line, TelemetryFrame, MODE_AUTONOMOUS,
)


# ── Extracted kinematics (same logic as stm_bridge._on_cmd_vel) ──────────────

def skid_steer_to_rpms(vx: float, wz: float,
                       track_width: float = 1.10,
                       wheel_circumference: float = 0.785,
                       max_rpm: float = 300):
    """Convert cmd_vel to motor RPMs — matches stm_bridge._on_cmd_vel exactly."""
    half_track = track_width / 2.0
    left_v = vx - wz * half_track
    right_v = vx + wz * half_track

    left_rpm = (left_v / wheel_circumference) * 60.0
    right_rpm = (right_v / wheel_circumference) * 60.0

    # Clamp
    left_rpm = max(-max_rpm, min(max_rpm, left_rpm))
    right_rpm = max(-max_rpm, min(max_rpm, right_rpm))

    return [left_rpm, right_rpm, left_rpm, right_rpm]  # FL, FR, RL, RR


def rpms_to_wheel_odom(rpms, dt, track_width=1.10, wheel_circumference=0.785):
    """Compute wheel odometry from motor RPMs — matches stm_bridge._update_wheel_odom."""
    left_rpm = (rpms[0] + rpms[2]) / 2.0   # FL + RL
    right_rpm = (rpms[1] + rpms[3]) / 2.0   # FR + RR

    left_v = left_rpm * wheel_circumference / 60.0
    right_v = right_rpm * wheel_circumference / 60.0

    v = (left_v + right_v) / 2.0
    omega = (right_v - left_v) / track_width
    return v, omega


# ── Tests ────────────────────────────────────────────────────────────────────

passed = 0
failed = 0


def check(desc, condition):
    global passed, failed
    if condition:
        passed += 1
        print(f'  ✓ {desc}')
    else:
        failed += 1
        print(f'  ✗ {desc}')


def test_stop():
    """Test: zero cmd_vel → zero RPM."""
    rpms = skid_steer_to_rpms(0.0, 0.0)
    print('Test: Zero cmd_vel → zero RPM')
    check('FL=0', rpms[0] == 0.0)
    check('FR=0', rpms[1] == 0.0)
    check('RL=0', rpms[2] == 0.0)
    check('RR=0', rpms[3] == 0.0)


def test_straight_forward():
    """Test: forward only → all motors same RPM."""
    rpms = skid_steer_to_rpms(1.0, 0.0)  # 1 m/s forward
    print('Test: Straight forward (1 m/s) → all equal RPM')
    expected = (1.0 / 0.785) * 60.0  # ~76.4 RPM
    check(f'FL≈expected ({rpms[0]:.1f} vs {expected:.1f})', abs(rpms[0] - expected) < 0.1)
    check(f'FR≈expected ({rpms[1]:.1f} vs {expected:.1f})', abs(rpms[1] - expected) < 0.1)
    check('FL==FR', rpms[0] == rpms[1])
    check('RL==RR', rpms[2] == rpms[3])
    check('FL==RL', rpms[0] == rpms[2])


def test_straight_backward():
    """Test: backward only → all motors negative."""
    rpms = skid_steer_to_rpms(-1.0, 0.0)
    print('Test: Straight backward (−1 m/s) → all negative RPM')
    check('FL<0', rpms[0] < 0)
    check('FR<0', rpms[1] < 0)
    check('FL==FR', rpms[0] == rpms[1])


def test_pivot_left():
    """Test: pure rotation left → left motors negative, right positive."""
    rpms = skid_steer_to_rpms(0.0, 1.0)  # 1 rad/s left turn
    print('Test: Pivot left (wz=1 rad/s) → left−, right+')
    check('FL<0', rpms[0] < 0)
    check('FR>0', rpms[1] > 0)
    check('RL<0', rpms[2] < 0)
    check('RR>0', rpms[3] > 0)
    check('FL==RL', rpms[0] == rpms[2])
    check('FR==RR', rpms[1] == rpms[3])
    check('|FL|==|FR|', abs(rpms[0]) == abs(rpms[1]))


def test_pivot_right():
    """Test: pure rotation right → left motors positive, right negative."""
    rpms = skid_steer_to_rpms(0.0, -1.0)
    print('Test: Pivot right (wz=−1 rad/s) → left+, right−')
    check('FL>0', rpms[0] > 0)
    check('FR<0', rpms[1] < 0)


def test_arc_forward_left():
    """Test: forward + left turn → all positive, left < right."""
    rpms = skid_steer_to_rpms(1.0, 0.5)
    print('Test: Arc forward-left (vx=1, wz=0.5) → left < right')
    check('FL>0', rpms[0] > 0)
    check('FR>0', rpms[1] > 0)
    check('FL < FR', rpms[0] < rpms[1])
    check('RL < RR', rpms[2] < rpms[3])


def test_clamp():
    """Test: high speed request clamped to max_rpm."""
    rpms = skid_steer_to_rpms(10.0, 0.0, max_rpm=300)
    print('Test: High speed clamped to max_rpm=300')
    check('FL<=300', rpms[0] <= 300.0)
    check('FR<=300', rpms[1] <= 300.0)
    check('FL=300', abs(rpms[0] - 300.0) < 0.01)
    check('FR=300', abs(rpms[1] - 300.0) < 0.01)


def test_negative_clamp():
    """Test: high reverse speed clamped to -max_rpm."""
    rpms = skid_steer_to_rpms(-10.0, 0.0, max_rpm=300)
    print('Test: High reverse clamped to -300')
    check('FL>=-300', rpms[0] >= -300.0)
    check('FL=-300', abs(rpms[0] - (-300.0)) < 0.01)


def test_wheel_odom_straight():
    """Test: straight motion → v=vx, omega=0."""
    rpms = skid_steer_to_rpms(1.0, 0.0)
    v, omega = rpms_to_wheel_odom(rpms, 1.0)
    print('Test: Wheel odom — straight at 1 m/s')
    check(f'v≈1.0 ({v:.3f})', abs(v - 1.0) < 0.01)
    check(f'omega≈0.0 ({omega:.3f})', abs(omega) < 0.001)


def test_wheel_odom_pivot():
    """Test: pure rotation → v=0, omega>0."""
    rpms = skid_steer_to_rpms(0.0, 1.0)
    v, omega = rpms_to_wheel_odom(rpms, 1.0)
    print('Test: Wheel odom — pivot (wz=1 rad/s)')
    check(f'v≈0 ({v:.3f})', abs(v) < 0.001)
    check(f'omega≈1.0 ({omega:.3f})', abs(omega - 1.0) < 0.01)


def test_wheel_odom_integration():
    """Test: forward for 1 second → x≈1, y≈0."""
    rpms = skid_steer_to_rpms(1.0, 0.0)
    v, omega = rpms_to_wheel_odom(rpms, 1.0)

    x, y, theta = 0.0, 0.0, 0.0
    dt = 0.01
    for _ in range(100):
        theta += omega * dt
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt

    print('Test: Wheel odom integration — forward 1s')
    check(f'x≈1.0 ({x:.3f})', abs(x - 1.0) < 0.01)
    check(f'y≈0.0 ({y:.3f})', abs(y) < 0.01)
    check(f'theta≈0 ({theta:.3f})', abs(theta) < 0.001)


def test_wheel_odom_circle():
    """Test: constant vx+wz for 2π radians → circle, returns to start."""
    vx, wz = 0.5, 1.0
    rpms = skid_steer_to_rpms(vx, wz)
    v, omega = rpms_to_wheel_odom(rpms, 1.0)

    x, y, theta = 0.0, 0.0, 0.0
    dt = 0.001
    t = 0.0
    target_theta = 2.0 * math.pi

    while theta < target_theta and t < 20.0:
        theta += omega * dt
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        t += dt

    R = v / omega  # expected radius
    print(f'Test: Circle — vx={vx}, wz={wz}, R={R:.2f}m, final theta={theta:.2f}')
    check(f'theta≈2π ({theta:.3f})', abs(theta - target_theta) < 0.1)
    check(f'x≈0 ({x:.3f})', abs(x) < 0.15)
    check(f'y≈0 ({y:.3f})', abs(y) < 0.15)


def test_telemetry_parser_integration():
    """Test: telemetry parser correctly parses fake H723 data patterns."""
    print('Test: Telemetry parser integration with stm_bridge patterns')

    # Motor telemetry line (exact format stm_bridge expects)
    frame = parse_line('FL|RPM:100,T:45,D:50,DIR:1,APP_PH:0,SP:100,BRAKE:0,FC:0,H:5,PWM_SET:50,PWM_ACT:48,QDROP:0,RXB:128')
    check('Motor frame parsed', frame is not None)
    check('Has motors', len(frame.motors) == 1)
    check('FL motor present', 0 in frame.motors)  # FL = motor_id 0
    check('RPM=100', frame.motors[0].rpm == 100.0)
    check('Temp=45', frame.motors[0].temperature == 45.0)
    check('Duty=50', frame.motors[0].duty == 50.0)
    check('Hall=5', frame.motors[0].hall_state == 5)

    # IMU telemetry line
    frame = parse_line('IMU|AX:10,AY:20,AZ:1000,GX:1.0,GY:0.5,GZ:0.2')
    check('IMU frame parsed', frame is not None)
    check('Has imu', frame.imu is not None)
    check('AX≈0.098 m/s²', abs(frame.imu.accel_x - 10 * 9.80665e-3) < 0.01)
    check('AZ≈9.807 m/s²', abs(frame.imu.accel_z - 1000 * 9.80665e-3) < 0.01)

    # MAG telemetry line
    frame = parse_line('MAG|MX:25.0,MY:-10.0,MZ:40.0')
    check('MAG frame parsed', frame is not None)
    check('Has mag', frame.mag is not None)
    check('MX=25.0', abs(frame.mag.mag_x - 25.0) < 0.01)

    # Mode telemetry
    frame = parse_line('MODE:AUTONOMOUS')
    check('Mode frame parsed', frame is not None)
    check('Mode=AUTONOMOUS', frame.mode == MODE_AUTONOMOUS)

    # ACK
    frame = parse_line('ACK')
    check('ACK detected', frame.ack is True)

    # NACK
    frame = parse_line('NACK')
    check('NACK detected', frame.nack is True)


def test_rpm_format():
    """Test: RPM command format matches H723 expected."""
    print('Test: RPM command format matches H723')

    def format_rpm_cmd(name, rpm):
        return f'{name} rpm {int(round(rpm))}\r\n'

    check('FL 100', format_rpm_cmd('FL', 100) == 'FL rpm 100\r\n')
    check('FR -50', format_rpm_cmd('FR', -50) == 'FR rpm -50\r\n')
    check('RL 0', format_rpm_cmd('RL', 0) == 'RL rpm 0\r\n')
    check('RR 250', format_rpm_cmd('RR', 250) == 'RR rpm 250\r\n')
    check('RPM rounding', format_rpm_cmd('FL', 76.4) == 'FL rpm 76\r\n')
    check('RPM rounding .5', format_rpm_cmd('FL', 76.6) == 'FL rpm 77\r\n')


def test_wheel_odom_360_turn():
    """Test: in-place 360° turn → back to origin."""
    rpms = skid_steer_to_rpms(0.0, 2.0)  # ~2 rad/s pivot
    v, omega = rpms_to_wheel_odom(rpms, 1.0)

    x, y, theta = 0.0, 0.0, 0.0
    dt = 0.001
    while theta < 2 * math.pi:
        theta += omega * dt
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt

    print('Test: 360° pivot → back to origin')
    check(f'x≈0 ({x:.4f})', abs(x) < 0.01)
    check(f'y≈0 ({y:.4f})', abs(y) < 0.01)
    check(f'v=0 ({v:.4f})', abs(v) < 0.001)


if __name__ == '__main__':
    print('=' * 60)
    print('STM Bridge Tests — Kinematics + Telemetry')
    print('=' * 60)
    print()

    test_stop()
    print()
    test_straight_forward()
    print()
    test_straight_backward()
    print()
    test_pivot_left()
    print()
    test_pivot_right()
    print()
    test_arc_forward_left()
    print()
    test_clamp()
    print()
    test_negative_clamp()
    print()
    test_wheel_odom_straight()
    print()
    test_wheel_odom_pivot()
    print()
    test_wheel_odom_integration()
    print()
    test_wheel_odom_circle()
    print()
    test_wheel_odom_360_turn()
    print()
    test_telemetry_parser_integration()
    print()
    test_rpm_format()

    print()
    print('=' * 60)
    print(f'Results: {passed} passed, {failed} failed')
    print('=' * 60)
    sys.exit(1 if failed > 0 else 0)
