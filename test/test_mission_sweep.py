#!/usr/bin/env python3
"""
Test mission sweep state machine + config (ROS bağımsız).

Kapsar (SweepLogic — ROS'suz extract):
  - Sweep arming: config parse → armed, idx=0, reports=[]
  - Run iteration: advance idx, apply param value, collect report id
  - Done detection: idx >= len(values) → done
  - Config validation (parse_sweep_config) — valid/invalid

Gerçek sweep koşumu Nav2+GPS gerektirir → M11 saha. Burada yalnız
state makinesi + config parse test edilir (mock koşum).

Usage:
  python3 test/test_mission_sweep.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_navigation'))

from earendil_navigation.mission_report import parse_sweep_config


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


# ── Sweep state machine logic extract ─────────────────────────────────────────

class SweepLogic:
    """Sweep state machine — ROS'suz, test edilebilir (mock koşum).

    mission_manager._handle_start_sweep + sweep iteration logic. Gerçek
    mission koşumu Nav2'ye bağlı; burada mock run ile state test edilir.
    """

    def __init__(self):
        self.sweep_config = None
        self.sweep_idx = 0
        self.sweep_reports = []  # list of (value, report_id)
        self.applied_values = []  # param values applied per run
        self.armed = False

    def arm(self, config_json):
        """_handle_start_sweep equivalent."""
        try:
            cfg = parse_sweep_config(config_json)
        except Exception as e:
            return False, str(e)
        if 'param' not in cfg or 'values' not in cfg:
            return False, 'needs param and values'
        self.sweep_config = cfg
        self.sweep_idx = 0
        self.sweep_reports = []
        self.applied_values = []
        self.armed = True
        return True, None

    def current_value(self):
        if not self.armed or self.sweep_idx >= len(self.sweep_config['values']):
            return None
        return self.sweep_config['values'][self.sweep_idx]

    def apply_param(self, value):
        """Mock: parametre uygula (gerçek set_parameters stub)."""
        self.applied_values.append(value)

    def run_iteration(self, report_id):
        """Bir run tamamlandı → report topla, idx ilerlet."""
        if not self.armed:
            return False
        val = self.current_value()
        if val is None:
            return False
        self.apply_param(val)
        self.sweep_reports.append((val, report_id))
        self.sweep_idx += 1
        return True

    def done(self):
        return self.armed and self.sweep_idx >= len(self.sweep_config['values'])

    def reset(self):
        self.sweep_config = None
        self.sweep_idx = 0
        self.sweep_reports = []
        self.applied_values = []
        self.armed = False


# ── Tests: config validation (parse_sweep_config) ─────────────────────────────

def test_config_valid():
    print('\n[config] valid')
    cfg = parse_sweep_config('{"param":"arrival_tolerance_m","values":[0.5,1.0,1.5]}')
    check('param', cfg['param'] == 'arrival_tolerance_m')
    check('values len 3', len(cfg['values']) == 3)


def test_config_single_value():
    print('\n[config] single value valid')
    cfg = parse_sweep_config('{"param":"max_retry_count","values":[1]}')
    check('one value', len(cfg['values']) == 1)


def test_config_missing_param():
    print('\n[config] missing param → ValueError')
    try:
        parse_sweep_config('{"values":[1.0]}')
        check('raised', False)
    except ValueError:
        check('raised ValueError', True)


def test_config_missing_values():
    print('\n[config] missing values → ValueError')
    try:
        parse_sweep_config('{"param":"x"}')
        check('raised', False)
    except ValueError:
        check('raised ValueError', True)


def test_config_empty_values():
    print('\n[config] empty values → ValueError')
    try:
        parse_sweep_config('{"param":"x","values":[]}')
        check('raised', False)
    except ValueError:
        check('raised ValueError', True)


def test_config_invalid_json():
    print('\n[config] invalid JSON')
    import json as _json
    try:
        parse_sweep_config('{bad')
        check('raised', False)
    except (_json.JSONDecodeError, ValueError):
        check('raised', True)


# ── Tests: sweep arming ───────────────────────────────────────────────────────

def test_arm_valid():
    print('\n[arm] valid config arms sweep')
    s = SweepLogic()
    ok, err = s.arm('{"param":"arrival_tolerance_m","values":[0.5,1.0,1.5]}')
    check('armed', ok is True and err is None)
    check('armed flag', s.armed is True)
    check('idx 0', s.sweep_idx == 0)
    check('reports empty', s.sweep_reports == [])
    check('config stored', s.sweep_config['param'] == 'arrival_tolerance_m')


def test_arm_invalid_config():
    print('\n[arm] invalid config → not armed')
    s = SweepLogic()
    ok, err = s.arm('{"values":[]}')
    check('not armed', ok is False)
    check('armed flag false', s.armed is False)
    check('error msg', err is not None)


def test_arm_bad_json():
    print('\n[arm] bad JSON → not armed')
    s = SweepLogic()
    ok, err = s.arm('{bad json')
    check('not armed', ok is False)
    check('armed flag false', s.armed is False)


# ── Tests: run iteration ──────────────────────────────────────────────────────

def test_run_iteration_advances():
    print('\n[run] iteration advances idx + collects report')
    s = SweepLogic()
    s.arm('{"param":"arrival_tolerance_m","values":[0.5,1.0,1.5]}')
    check('first value', s.current_value() == 0.5)
    ok1 = s.run_iteration('rpt_001')
    check('run 1 ok', ok1 is True)
    check('idx 1', s.sweep_idx == 1)
    check('report collected', s.sweep_reports == [(0.5, 'rpt_001')])
    check('value applied', s.applied_values == [0.5])
    check('not done', s.done() is False)


def test_run_full_sweep_done():
    print('\n[run] full sweep → done')
    s = SweepLogic()
    s.arm('{"param":"arrival_tolerance_m","values":[0.5,1.0,1.5]}')
    s.run_iteration('rpt_001')
    s.run_iteration('rpt_002')
    s.run_iteration('rpt_003')
    check('idx 3', s.sweep_idx == 3)
    check('done', s.done() is True)
    check('3 reports', len(s.sweep_reports) == 3)
    check('all values applied', s.applied_values == [0.5, 1.0, 1.5])
    check('current_value None', s.current_value() is None)


def test_run_after_done_noop():
    print('\n[run] iteration after done → no-op')
    s = SweepLogic()
    s.arm('{"param":"x","values":[1]}')
    s.run_iteration('rpt_001')
    check('done after 1', s.done() is True)
    ok = s.run_iteration('rpt_002')
    check('extra run rejected', ok is False)
    check('still 1 report', len(s.sweep_reports) == 1)


def test_run_not_armed():
    print('\n[run] not armed → no-op')
    s = SweepLogic()
    ok = s.run_iteration('rpt_001')
    check('run rejected', ok is False)
    check('no reports', s.sweep_reports == [])


# ── Tests: reset ──────────────────────────────────────────────────────────────

def test_reset():
    print('\n[reset] clears sweep state')
    s = SweepLogic()
    s.arm('{"param":"x","values":[1,2]}')
    s.run_iteration('rpt_001')
    s.reset()
    check('not armed', s.armed is False)
    check('config None', s.sweep_config is None)
    check('idx 0', s.sweep_idx == 0)
    check('reports cleared', s.sweep_reports == [])
    check('applied cleared', s.applied_values == [])


# ── run ───────────────────────────────────────────────────────────────────────

def main():
    print('═══ Mission Sweep Tests ═══')
    test_config_valid()
    test_config_single_value()
    test_config_missing_param()
    test_config_missing_values()
    test_config_empty_values()
    test_config_invalid_json()
    test_arm_valid()
    test_arm_invalid_config()
    test_arm_bad_json()
    test_run_iteration_advances()
    test_run_full_sweep_done()
    test_run_after_done_noop()
    test_run_not_armed()
    test_reset()
    print(f'\n═══ {passed} passed, {failed} failed ═══')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
