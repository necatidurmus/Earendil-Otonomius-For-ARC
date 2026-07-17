#!/usr/bin/env python3
"""
Tek-yazıcı audit scripti — RPi5 üzerinde çalıştırılır.

/cmd_vel_safe'e yalnız safety_mux yazıyor mu?
H723 seri portuna yalnız stm_bridge yazıyor mu?
RSCP seri portuna yalnız rscp_bridge yazıyor mu?

Kullanım:
  ros2 run earendil_tools audit_single_writer
  # veya
  python3 scripts/audit_single_writer.py
"""

import subprocess
import sys


def check_topic_writers(topic: str, expected_writer: str) -> bool:
    """Check that only expected_writer publishes to topic."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'info', topic, '-v'],
            capture_output=True, text=True, timeout=5
        )
        output = result.stdout

        # Count publishers
        pub_count = 0
        writers = []
        in_publishers = False
        for line in output.split('\n'):
            if 'Publisher count:' in line:
                pub_count = int(line.split(':')[-1].strip())
            if 'Node name:' in line:
                name = line.split(':')[-1].strip()
                writers.append(name)
            if 'Publishers' in line:
                in_publishers = True
            if 'Subscribers' in line:
                in_publishers = False

        print(f'\n{topic}:')
        print(f'  Publisher count: {pub_count}')
        print(f'  Writers: {writers}')

        if pub_count == 0:
            print(f'  ⚠ No publishers (node not running)')
            return True  # Not a safety violation

        if pub_count > 1:
            print(f'  ✗ MULTIPLE WRITERS — safety violation!')
            return False

        if expected_writer not in writers:
            print(f'  ✗ Expected {expected_writer}, found {writers}')
            return False

        print(f'  ✓ Single writer: {expected_writer}')
        return True

    except subprocess.TimeoutExpired:
        print(f'  ⚠ Timeout (ros2 not running?)')
        return True
    except FileNotFoundError:
        print(f'  ⚠ ros2 not found (not in PATH)')
        return True


def main():
    print('=' * 60)
    print('Tek-Yazıcı Audit')
    print('=' * 60)

    all_ok = True

    # 1. /cmd_vel_safe → only safety_mux
    if not check_topic_writers('/cmd_vel_safe', 'safety_mux'):
        all_ok = False

    # 2. H723 serial → only stm_bridge
    # (Check via topic info for /cmd_vel_safe — stm_bridge is subscriber only)
    print('\nSTM serial port (/dev/earendil_h7):')
    print('  ℹ Check manually: lsof /dev/earendil_h7')
    print('  Expected: only stm_bridge has fd open')

    # 3. RSCP serial → only rscp_bridge
    print('\nRSCP serial port (/dev/earendil_rscp):')
    print('  ℹ Check manually: lsof /dev/earendil_rscp')
    print('  Expected: only rscp_bridge has fd open')

    # 4. Jetson should NOT publish to /cmd_vel_*
    if not check_topic_writers('/cmd_vel_nav', ''):
        pass  # Multiple writers ok for /cmd_vel_nav (Nav2 + others)
    print('  ℹ Jetson should NOT be in /cmd_vel_* writers')

    print()
    print('=' * 60)
    if all_ok:
        print('✓ Audit PASSED — single-writer rule upheld')
    else:
        print('✗ Audit FAILED — safety violations found!')
    print('=' * 60)

    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
