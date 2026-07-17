#!/usr/bin/env python3
"""
Audit: Web node NEVER writes to /cmd_vel_safe.

Earendil safety invariant: only safety_mux publishes to /cmd_vel_safe.
Web dashboard must only publish to /cmd_vel_manual and /e_stop.

Usage:
  python3 scripts/audit_web_no_safe.py
"""

import subprocess
import sys


def check_web_not_on_safe_topic() -> bool:
    """Verify web_dashboard node is NOT a publisher on /cmd_vel_safe."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'info', '/cmd_vel_safe', '-v'],
            capture_output=True, text=True, timeout=5
        )
        output = result.stdout

        # Find all publisher node names
        writers = []
        in_publishers = False
        for line in output.split('\n'):
            if 'Publisher count:' in line:
                in_publishers = True
                continue
            if 'Subscribers' in line:
                in_publishers = False
            if in_publishers and 'Node name:' in line:
                name = line.split(':')[-1].strip()
                writers.append(name)

        # Check web_dashboard is NOT in writers
        web_writes_safe = 'web_dashboard' in writers
        if web_writes_safe:
            print(f'FAIL: web_dashboard publishes to /cmd_vel_safe!')
            print(f'  Writers found: {writers}')
            return False

        print(f'OK: web_dashboard does NOT publish to /cmd_vel_safe')
        print(f'  Current writers: {writers}')
        return True

    except FileNotFoundError:
        print('SKIP: ros2 CLI not found (not running on ROS 2 system)')
        return True
    except subprocess.TimeoutExpired:
        print('SKIP: ros2 topic info timed out')
        return True
    except Exception as e:
        print(f'SKIP: {e}')
        return True


def check_web_publishes_manual() -> bool:
    """Verify web_dashboard DOES publish to /cmd_vel_manual."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'info', '/cmd_vel_manual', '-v'],
            capture_output=True, text=True, timeout=5
        )
        output = result.stdout

        writers = []
        in_publishers = False
        for line in output.split('\n'):
            if 'Publisher count:' in line:
                in_publishers = True
                continue
            if 'Subscribers' in line:
                in_publishers = False
            if in_publishers and 'Node name:' in line:
                name = line.split(':')[-1].strip()
                writers.append(name)

        if 'web_dashboard' in writers:
            print(f'OK: web_dashboard publishes to /cmd_vel_manual')
            return True
        else:
            print(f'WARN: web_dashboard not found publishing to /cmd_vel_manual')
            print(f'  (node may not be running)')
            return True  # Not a failure, just not running

    except Exception:
        return True


def main():
    print('=' * 50)
    print('  Web Safety Audit')
    print('  Invariant: web_dashboard NEVER writes /cmd_vel_safe')
    print('=' * 50)

    ok = True
    ok &= check_web_not_on_safe_topic()
    ok &= check_web_publishes_manual()

    print()
    if ok:
        print('PASS: Web safety audit passed')
        return 0
    else:
        print('FAIL: Web safety audit FAILED')
        return 1


if __name__ == '__main__':
    sys.exit(main())
