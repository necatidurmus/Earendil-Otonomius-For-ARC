#!/usr/bin/env python3
"""
Test mission report metrics + CSV/preset/sweep saf fonksiyonlar (ROS bağımsız).

Kapsar:
  - compute_metrics: path length, RMSE, hız profili, jerk, RTK dağılım
  - build_mission_report: report dict yapısı
  - import_waypoints_csv / export_waypoints_csv: CSV girdi/çıktı
  - load_preset_yaml: preset parametre okuma (yaml veya fallback parser)
  - parse_sweep_config: sweep config JSON validate

Usage:
  python3 test/test_mission_report.py
"""

import sys
import os
import math
import json
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_navigation'))

from earendil_navigation.mission_report import (
    PathSample, WaypointRecord,
    compute_metrics, build_mission_report,
    import_waypoints_csv, export_waypoints_csv,
    load_preset_yaml, parse_sweep_config,
)


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


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ── compute_metrics ───────────────────────────────────────────────────────────

def test_metrics_empty():
    print('\n[metrics] empty input')
    m = compute_metrics([], [])
    check('path_length 0', m['path_length_m'] == 0.0)
    check('rmse 0', m['rmse_m'] == 0.0)
    check('max_speed 0', m['max_speed_mps'] == 0.0)
    check('avg_speed 0', m['avg_speed_mps'] == 0.0)
    check('max_jerk 0', m['max_jerk_mps3'] == 0.0)
    check('rtk_samples empty', m['rtk_quality_samples'] == {})


def test_metrics_path_length():
    print('\n[metrics] path length')
    samples = [
        PathSample(t=0.0, x=0.0, y=0.0),
        PathSample(t=1.0, x=3.0, y=0.0),
        PathSample(t=2.0, x=3.0, y=4.0),
    ]
    m = compute_metrics(samples, [])
    # 3 + 4 = 7
    check('path_length = 7.0', approx(m['path_length_m'], 7.0))


def test_metrics_rmse():
    print('\n[metrics] rmse vs nearest waypoint')
    # hedef (0,0) — örnekler (3,0),(0,4) → mesafeler 3,4 → RMSE = sqrt((9+16)/2)
    wps = [WaypointRecord(idx=0, lat=0, lon=0, target_x=0.0, target_y=0.0)]
    samples = [
        PathSample(t=0.0, x=3.0, y=0.0),
        PathSample(t=1.0, x=0.0, y=4.0),
    ]
    m = compute_metrics(samples, wps)
    expected = math.sqrt((9.0 + 16.0) / 2.0)
    check(f'rmse = {expected:.4f}', approx(m['rmse_m'], expected))


def test_metrics_speed_profile():
    print('\n[metrics] speed profile')
    samples = [
        PathSample(t=0.0, x=0.0, y=0.0, vx=0.0, vy=0.0),
        PathSample(t=1.0, x=0.0, y=0.0, vx=1.0, vy=0.0),   # speed 1
        PathSample(t=2.0, x=0.0, y=0.0, vx=0.0, vy=3.0),   # speed 3
    ]
    m = compute_metrics(samples, [])
    check('max_speed = 3.0', approx(m['max_speed_mps'], 3.0))
    check('avg_speed = 4/3', approx(m['avg_speed_mps'], (0.0 + 1.0 + 3.0) / 3.0))


def test_metrics_jerk():
    print('\n[metrics] jerk (accel change / dt)')
    # dt=1, v: 0→1→3 → accel: 1, 2 → jerk = |2-1|/1 = 1.0
    samples = [
        PathSample(t=0.0, x=0, y=0, vx=0.0, vy=0.0),
        PathSample(t=1.0, x=0, y=0, vx=1.0, vy=0.0),
        PathSample(t=2.0, x=0, y=0, vx=3.0, vy=0.0),
    ]
    m = compute_metrics(samples, [])
    check('max_jerk = 1.0', approx(m['max_jerk_mps3'], 1.0))


def test_metrics_rtk_distribution():
    print('\n[metrics] rtk quality distribution')
    samples = [
        PathSample(t=0.0, x=0, y=0, rtk_status='FIXED'),
        PathSample(t=1.0, x=0, y=0, rtk_status='FIXED'),
        PathSample(t=2.0, x=0, y=0, rtk_status='FLOAT'),
        PathSample(t=3.0, x=0, y=0, rtk_status='NO_FIX'),
    ]
    m = compute_metrics(samples, [])
    check('FIXED count 2', m['rtk_quality_samples'].get('FIXED') == 2)
    check('FLOAT count 1', m['rtk_quality_samples'].get('FLOAT') == 1)
    check('NO_FIX count 1', m['rtk_quality_samples'].get('NO_FIX') == 1)


# ── build_mission_report ──────────────────────────────────────────────────────

def test_report_structure():
    print('\n[report] structure')
    wps = [WaypointRecord(idx=0, lat=39.0, lon=32.0, target_x=10.0, target_y=0.0,
                          reached=True, retries=1, skipped=False, elapsed_s=12.5)]
    samples = [PathSample(t=0.0, x=0.0, y=0.0, vx=0.5, vy=0.0, rtk_status='FIXED')]
    r = build_mission_report('mission_001', 2, 1000.0, 1010.0, 'task_finished',
                             wps, samples, ['task_finished'])
    check('mission_id', r['mission_id'] == 'mission_001')
    check('stage', r['stage'] == 2)
    check('start_time', r['start_time'] == 1000.0)
    check('end_time', r['end_time'] == 1010.0)
    check('duration_s = 10.0', approx(r['duration_s'], 10.0))
    check('end_reason', r['end_reason'] == 'task_finished')
    check('waypoints len 1', len(r['waypoints']) == 1)
    check('waypoint idx', r['waypoints'][0]['idx'] == 0)
    check('waypoint reached', r['waypoints'][0]['reached'] is True)
    check('waypoint retries', r['waypoints'][0]['retries'] == 1)
    check('path len 1', len(r['path']) == 1)
    check('path sample fields', r['path'][0]['rtk_status'] == 'FIXED')
    check('metrics present', 'rmse_m' in r['metrics'])
    check('result_messages', r['result_messages'] == ['task_finished'])


def test_report_json_serializable():
    print('\n[report] json serializable')
    wps = [WaypointRecord(idx=0, lat=39.0, lon=32.0)]
    samples = [PathSample(t=0.0, x=1.0, y=2.0)]
    r = build_mission_report('m', 1, 0.0, 5.0, 'done', wps, samples, [])
    s = json.dumps(r)  # exception fırlatırsa test patlar
    check('json.dumps OK', isinstance(s, str))


# ── CSV import/export ─────────────────────────────────────────────────────────

def test_csv_import_with_header():
    print('\n[csv] import with header')
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write('lat,lon,alt,stage\n')
        f.write('39.0,32.0,100.0,1\n')
        f.write('39.1,32.1,101.0,2\n')
        f.write('# comment line\n')
        f.write('bad,line\n')
        path = f.name
    try:
        wps = import_waypoints_csv(path)
        check('2 valid waypoints', len(wps) == 2)
        check('first lat', approx(wps[0][0], 39.0))
        check('first lon', approx(wps[0][1], 32.0))
        check('first alt', approx(wps[0][2], 100.0))
        check('second lat', approx(wps[1][0], 39.1))
    finally:
        os.unlink(path)


def test_csv_import_without_header():
    print('\n[csv] import without header (data first)')
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write('39.0,32.0,100.0\n')
        f.write('39.1,32.1,101.0\n')
        path = f.name
    try:
        wps = import_waypoints_csv(path)
        check('2 waypoints (no header)', len(wps) == 2)
        check('first lat', approx(wps[0][0], 39.0))
    finally:
        os.unlink(path)


def test_csv_import_missing_alt():
    print('\n[csv] import missing alt column → default 0.0')
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write('lat,lon\n')
        f.write('39.0,32.0\n')
        path = f.name
    try:
        wps = import_waypoints_csv(path)
        check('1 waypoint', len(wps) == 1)
        check('alt default 0.0', approx(wps[0][2], 0.0))
    finally:
        os.unlink(path)


def test_csv_import_empty():
    print('\n[csv] import empty file')
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write('')
        path = f.name
    try:
        wps = import_waypoints_csv(path)
        check('empty → 0 waypoints', wps == [])
    finally:
        os.unlink(path)


def test_csv_export_roundtrip():
    print('\n[csv] export content check')
    wps = [
        WaypointRecord(idx=0, lat=39.0, lon=32.0, reached=True, retries=2,
                       skipped=False, elapsed_s=10.0),
        WaypointRecord(idx=1, lat=39.1, lon=32.1, reached=False, retries=0,
                       skipped=True, elapsed_s=20.0),
    ]
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        path = f.name
    try:
        export_waypoints_csv(path, wps)
        import csv as _csv
        with open(path, 'r') as fh:
            rows = list(_csv.reader(fh))
        check('header row', rows[0][1] == 'lat' and rows[0][2] == 'lon')
        check('row1 lat', approx(float(rows[1][1]), 39.0))
        check('row1 reached', rows[1][3] == '1')
        check('row1 retries', rows[1][4] == '2')
        check('row2 skipped', rows[2][5] == '1')
    finally:
        os.unlink(path)


# ── load_preset_yaml ──────────────────────────────────────────────────────────

def test_preset_load():
    print('\n[preset] load conservative yaml')
    preset_path = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'earendil_navigation',
        'config', 'mission_presets', 'conservative.yaml')
    params = load_preset_yaml(preset_path)
    check('arrival_tolerance present', 'arrival_tolerance_m' in params)
    check('arrival_tolerance = 1.5', approx(float(params['arrival_tolerance_m']), 1.5))
    check('search_speed = 0.2', approx(float(params['search_speed_mps']), 0.2))
    check('waypoint_timeout = 180', int(params['waypoint_timeout_s']) == 180)
    check('max_retry = 5', int(params['max_retry_count']) == 5)


def test_preset_aggressive():
    print('\n[preset] aggressive yaml')
    preset_path = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'earendil_navigation',
        'config', 'mission_presets', 'aggressive.yaml')
    params = load_preset_yaml(preset_path)
    check('arrival_tolerance = 0.8', approx(float(params['arrival_tolerance_m']), 0.8))
    check('search_speed = 0.5', approx(float(params['search_speed_mps']), 0.5))
    check('waypoint_timeout = 90', int(params['waypoint_timeout_s']) == 90)


def test_preset_test():
    print('\n[preset] test yaml')
    preset_path = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'earendil_navigation',
        'config', 'mission_presets', 'test.yaml')
    params = load_preset_yaml(preset_path)
    check('arrival_tolerance = 1.0', approx(float(params['arrival_tolerance_m']), 1.0))
    check('exploration_timeout = 60', int(params['exploration_timeout_s']) == 60)
    check('max_retry = 1', int(params['max_retry_count']) == 1)


# ── parse_sweep_config ────────────────────────────────────────────────────────

def test_sweep_valid():
    print('\n[sweep] valid config')
    cfg = parse_sweep_config('{"param":"arrival_tolerance_m","values":[0.5,1.0,1.5]}')
    check('param', cfg['param'] == 'arrival_tolerance_m')
    check('values len 3', len(cfg['values']) == 3)
    check('values[0]', cfg['values'][0] == 0.5)


def test_sweep_missing_param():
    print('\n[sweep] missing param → ValueError')
    try:
        parse_sweep_config('{"values":[1.0]}')
        check('raised ValueError', False)
    except ValueError:
        check('raised ValueError', True)


def test_sweep_missing_values():
    print('\n[sweep] missing values → ValueError')
    try:
        parse_sweep_config('{"param":"x"}')
        check('raised ValueError', False)
    except ValueError:
        check('raised ValueError', True)


def test_sweep_empty_values():
    print('\n[sweep] empty values → ValueError')
    try:
        parse_sweep_config('{"param":"x","values":[]}')
        check('raised ValueError', False)
    except ValueError:
        check('raised ValueError', True)


def test_sweep_invalid_json():
    print('\n[sweep] invalid JSON → JSONDecodeError')
    import json as _json
    try:
        parse_sweep_config('{not json')
        check('raised', False)
    except (_json.JSONDecodeError, ValueError):
        check('raised JSONDecodeError/ValueError', True)


# ── run ───────────────────────────────────────────────────────────────────────

def main():
    print('═══ Mission Report Tests ═══')
    test_metrics_empty()
    test_metrics_path_length()
    test_metrics_rmse()
    test_metrics_speed_profile()
    test_metrics_jerk()
    test_metrics_rtk_distribution()
    test_report_structure()
    test_report_json_serializable()
    test_csv_import_with_header()
    test_csv_import_without_header()
    test_csv_import_missing_alt()
    test_csv_import_empty()
    test_csv_export_roundtrip()
    test_preset_load()
    test_preset_aggressive()
    test_preset_test()
    test_sweep_valid()
    test_sweep_missing_param()
    test_sweep_missing_values()
    test_sweep_empty_values()
    test_sweep_invalid_json()
    print(f'\n═══ {passed} passed, {failed} failed ═══')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
