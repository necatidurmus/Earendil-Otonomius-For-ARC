#!/usr/bin/env python3
"""
Fake H723 — simulates STM32H723 telemetry for testing stm_bridge.

Usage:
  # With loopback serial (socat):
  socat -d -d pty,raw,echo=0,link=/tmp/h723_fake pty,raw,echo=0,link=/tmp/h723_real
  python3 test/fake_h723.py --serial /tmp/h723_fake
  # Then stm_bridge uses /tmp/h723_real

  # Standalone test (no serial):
  python3 test/fake_h723.py

Generates motor telemetry, IMU data, and magnetometer data at realistic rates.
"""

import sys
import os
import time
import math
import argparse
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_control'))


def generate_motor_telemetry(motor_name: str, rpm: float = 0.0) -> str:
    """Generate a motor telemetry line in H723 format."""
    return (
        f'{motor_name}|RPM:{rpm:.0f},T:45,D:50,DIR:1,APP_PH:0,'
        f'SP:{rpm:.0f},BRAKE:0,FC:0,H:5,'
        f'PWM_SET:50,PWM_ACT:48,QDROP:0,RXB:128'
    )


def generate_imu_telemetry(t: float) -> str:
    """Generate IMU telemetry line (MPU9250)."""
    # Simulate gravity on Z axis (mg)
    ax = 10 * math.sin(t * 0.5)  # small noise
    ay = 10 * math.cos(t * 0.3)
    az = 1000  # ~1g in mg
    # Simulate gyro (dps)
    gx = 1.0 * math.sin(t * 0.1)
    gy = 0.5 * math.cos(t * 0.2)
    gz = 0.2 * math.sin(t * 0.05)
    return f'IMU|AX:{ax:.0f},AY:{ay:.0f},AZ:{az:.0f},GX:{gx:.2f},GY:{gy:.2f},GZ:{gz:.2f}'


def generate_mag_telemetry(t: float) -> str:
    """Generate magnetometer telemetry line (QMC5883P)."""
    mx = 25.0 + 5.0 * math.sin(t * 0.01)
    my = -10.0 + 3.0 * math.cos(t * 0.01)
    mz = 40.0 + 2.0 * math.sin(t * 0.02)
    return f'MAG|MX:{mx:.1f},MY:{my:.1f},MZ:{mz:.1f}'


def generate_mode_telemetry(mode: str = 'AUTONOMOUS') -> str:
    """Generate mode telemetry line."""
    return f'MODE:{mode}'


class FakeH723:
    """Simulates H723 telemetry output."""

    def __init__(self, serial_port=None, baud=115200):
        self._serial_port = serial_port
        self._baud = baud
        self._serial = None
        self._running = True
        self._rpm_targets = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self._mode = 'AUTONOMOUS'
        self._start_time = time.time()

    def start(self):
        """Start the fake H723."""
        if self._serial_port:
            self._open_serial()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

        self._telemetry_loop()

    def _open_serial(self):
        import serial
        self._serial = serial.Serial(self._serial_port, self._baud, timeout=0.1)
        print(f'Fake H723: opened {self._serial_port}')

    def _read_loop(self):
        """Read commands from stm_bridge."""
        buffer = ''
        while self._running:
            try:
                data = self._serial.read(256)
                if data:
                    buffer += data.decode('ascii', errors='replace')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._process_command(line)
            except Exception as e:
                if self._running:
                    print(f'Fake H723 read error: {e}')
                break

    def _process_command(self, line: str):
        """Process incoming command from stm_bridge."""
        # Expected: FL rpm 100\r\n
        parts = line.strip().split()
        if len(parts) == 3 and parts[1] == 'rpm':
            motor = parts[0]
            rpm = int(parts[2])
            motor_map = {'FL': 0, 'FR': 1, 'RL': 2, 'RR': 3}
            if motor in motor_map:
                self._rpm_targets[motor_map[motor]] = rpm
                # Send ACK
                self._send('ACK')
                print(f'Fake H723: {motor} rpm {rpm} → ACK')

    def _send(self, data: str):
        """Send data over serial."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.write((data + '\r\n').encode('ascii'))
            except Exception:
                pass

    def _telemetry_loop(self):
        """Generate telemetry at realistic rates."""
        print('Fake H723: starting telemetry loop')
        print('  Motor telemetry: 10 Hz')
        print('  IMU: 100 Hz (simulated at 10 Hz for testing)')
        print('  MAG: 10 Hz')
        print('  Mode: 1 Hz')
        print()

        motor_count = 0
        imu_count = 0
        mag_count = 0
        mode_count = 0

        while self._running:
            t = time.time() - self._start_time

            # Motor telemetry at ~10 Hz
            if motor_count < t * 10:
                for i, name in enumerate(['FL', 'FR', 'RL', 'RR']):
                    self._send(generate_motor_telemetry(name, self._rpm_targets[i]))
                motor_count += 1

            # IMU at ~10 Hz (simulating lower rate for testing)
            if imu_count < t * 10:
                self._send(generate_imu_telemetry(t))
                imu_count += 1

            # MAG at ~10 Hz
            if mag_count < t * 10:
                self._send(generate_mag_telemetry(t))
                mag_count += 1

            # Mode at ~1 Hz
            if mode_count < t:
                self._send(generate_mode_telemetry(self._mode))
                mode_count += 1

            time.sleep(0.01)  # 100 Hz loop

    def stop(self):
        self._running = False
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass


def test_without_serial():
    """Test fake H723 data generation without serial."""
    print('=== Fake H723 — Standalone Test ===\n')

    t = 0.0
    for i in range(5):
        print(f't={t:.1f}s:')
        for name in ['FL', 'FR', 'RL', 'RR']:
            print(f'  {generate_motor_telemetry(name, 100 + i * 10)}')
        print(f'  {generate_imu_telemetry(t)}')
        print(f'  {generate_mag_telemetry(t)}')
        print(f'  {generate_mode_telemetry("AUTONOMOUS")}')
        print(f'  ACK')
        t += 0.1
        print()

    print('=== Test complete — use --serial for HIL testing ===')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fake H723 STM32')
    parser.add_argument('--serial', type=str, help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    if args.serial:
        fake = FakeH723(args.serial, args.baud)
        try:
            fake.start()
        except KeyboardInterrupt:
            fake.stop()
    else:
        test_without_serial()
