#!/usr/bin/env python3
"""
Mission report + metrics + CSV/preset/sweep saf fonksiyonlar (ROS bağımsız).

Bu modül rclpy import ETMEZ — dev box'ta (ROS yok) doğrudan import edilebilir
ve test edilebilir. mission_manager.py buradan import eder.

İçerik:
  - PathSample / WaypointRecord dataclass'lar
  - compute_metrics: path length, RMSE, hız profili, jerk
  - build_mission_report: mission report JSON dict
  - import_waypoints_csv / export_waypoints_csv: CSV girdi/çıktı
  - load_preset_yaml: preset parametre okuma
  - parse_sweep_config: sweep config JSON validate
"""

import math
import csv
import json
from dataclasses import dataclass
from typing import List, Dict, Tuple


# ── Report dataclasses ───────────────────────────────────────────────────────

@dataclass
class PathSample:
    """Path örneği (report_sample_hz ile örnekleme)."""
    t: float
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    rtk_status: str = 'NO_FIX'


@dataclass
class WaypointRecord:
    """Waypoint kaydı (mission report içinde)."""
    idx: int
    lat: float
    lon: float
    target_x: float = 0.0
    target_y: float = 0.0
    reached: bool = False
    retries: int = 0
    skipped: bool = False
    elapsed_s: float = 0.0


# ── Metrics ──────────────────────────────────────────────────────────────────

def compute_metrics(path_samples: List[PathSample],
                    waypoints: List[WaypointRecord]) -> dict:
    """Path length, RMSE, hız profili, jerk hesapla (saf fonksiyon).

    RMSE: her path örneği ile o anki en-yakın hedef waypoint arası mesafe RMS.
    Jerk: ivme değişimi / dt — örnekleme üstünden max mutlak.
    """
    if not path_samples:
        return {
            'path_length_m': 0.0, 'rmse_m': 0.0,
            'max_speed_mps': 0.0, 'avg_speed_mps': 0.0,
            'max_jerk_mps3': 0.0, 'rtk_quality_samples': {},
        }

    # Path length (kümülatif örnekleme mesafesi)
    length = 0.0
    for i in range(1, len(path_samples)):
        dx = path_samples[i].x - path_samples[i - 1].x
        dy = path_samples[i].y - path_samples[i - 1].y
        length += math.hypot(dx, dy)

    # RMSE — en yakın hedef waypoint (target_x/y; (0,0) geçerli hedef olabilir)
    targets = [(wp.target_x, wp.target_y) for wp in waypoints]
    sq_errors = []
    for s in path_samples:
        if not targets:
            break
        dmin = min(math.hypot(s.x - tx, s.y - ty) for tx, ty in targets)
        sq_errors.append(dmin * dmin)
    rmse = math.sqrt(sum(sq_errors) / len(sq_errors)) if sq_errors else 0.0

    # Hız profili
    speeds = [math.hypot(s.vx, s.vy) for s in path_samples]
    max_speed = max(speeds) if speeds else 0.0
    avg_speed = sum(speeds) / len(speeds) if speeds else 0.0

    # Jerk — ivme değişimi / dt (üç nokta üstünden)
    max_jerk = 0.0
    for i in range(2, len(path_samples)):
        dt1 = path_samples[i - 1].t - path_samples[i - 2].t
        dt2 = path_samples[i].t - path_samples[i - 1].t
        if dt1 <= 0 or dt2 <= 0:
            continue
        v_pp = math.hypot(path_samples[i - 2].vx, path_samples[i - 2].vy)
        v_prev = math.hypot(path_samples[i - 1].vx, path_samples[i - 1].vy)
        v_cur = math.hypot(path_samples[i].vx, path_samples[i].vy)
        accel_prev = (v_prev - v_pp) / dt1
        accel_cur = (v_cur - v_prev) / dt2
        jerk = abs((accel_cur - accel_prev) / dt2)
        max_jerk = max(max_jerk, jerk)

    # RTK kalite dağılımı
    rtk_counts: Dict[str, int] = {}
    for s in path_samples:
        rtk_counts[s.rtk_status] = rtk_counts.get(s.rtk_status, 0) + 1

    return {
        'path_length_m': length,
        'rmse_m': rmse,
        'max_speed_mps': max_speed,
        'avg_speed_mps': avg_speed,
        'max_jerk_mps3': max_jerk,
        'rtk_quality_samples': rtk_counts,
    }


def build_mission_report(mission_id: str, stage: int,
                         start_time: float, end_time: float,
                         end_reason: str,
                         waypoints: List[WaypointRecord],
                         path_samples: List[PathSample],
                         result_messages: List[str]) -> dict:
    """Mission report JSON dict (ROS bağımsız, test edilebilir)."""
    metrics = compute_metrics(path_samples, waypoints)
    return {
        'mission_id': mission_id,
        'stage': stage,
        'start_time': start_time,
        'end_time': end_time,
        'duration_s': end_time - start_time if start_time > 0 else 0.0,
        'end_reason': end_reason,
        'waypoints': [
            {
                'idx': wp.idx, 'lat': wp.lat, 'lon': wp.lon,
                'target_x': wp.target_x, 'target_y': wp.target_y,
                'reached': wp.reached, 'retries': wp.retries,
                'skipped': wp.skipped, 'elapsed_s': wp.elapsed_s,
            } for wp in waypoints
        ],
        'path': [
            {
                't': s.t, 'x': s.x, 'y': s.y,
                'vx': s.vx, 'vy': s.vy, 'rtk_status': s.rtk_status,
            } for s in path_samples
        ],
        'metrics': metrics,
        'result_messages': list(result_messages),
    }


# ── CSV import/export ────────────────────────────────────────────────────────

def import_waypoints_csv(path: str) -> List[Tuple[float, float, float]]:
    """CSV import: lat,lon,alt[,stage] başlık + satır → (lat, lon, alt) listesi.

    Başlık satırı (lat/latitude) otomatik atlanır. # yorum satırları atlanır.
    Hatalı satırlar atlanır (exception fırlatmaz).
    """
    wps: List[Tuple[float, float, float]] = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return wps
        h = [c.strip().lower() for c in header]
        if not (h and h[0] in ('lat', 'latitude')):
            # ilk satır veri — işle
            try:
                vals = [float(x) for x in header[:3]]
                wps.append((vals[0], vals[1], vals[2] if len(vals) > 2 else 0.0))
            except ValueError:
                pass  # gerçek başlık, atla
        for row in reader:
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                vals = [float(x) for x in row[:3]]
                wps.append((vals[0], vals[1], vals[2] if len(vals) > 2 else 0.0))
            except (ValueError, IndexError):
                continue
    return wps


def export_waypoints_csv(path: str, waypoints: List[WaypointRecord]):
    """CSV export: waypoint kayıtlarını dosyaya yaz."""
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['idx', 'lat', 'lon', 'reached', 'retries', 'skipped',
                    'elapsed_s'])
        for wp in waypoints:
            w.writerow([wp.idx, wp.lat, wp.lon, int(wp.reached), wp.retries,
                        int(wp.skipped), wp.elapsed_s])


# ── Preset / Sweep ───────────────────────────────────────────────────────────

def load_preset_yaml(path: str) -> Dict[str, object]:
    """Preset yaml oku → {param_name: value} dict.

    ROS param dict yapısı:
      mission_manager:
        ros__parameters:
          <key>: <value>

    PyYAML varsa kullanır; yoksa minimal indent-bazlı parser (key: value).
    """
    try:
        import yaml
    except ImportError:
        yaml = None

    if yaml is not None:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        node = data.get('mission_manager', {})
        return dict(node.get('ros__parameters', {}))

    # Minimal fallback parser (indent-bazlı)
    params: Dict[str, object] = {}
    in_params = False
    with open(path, 'r') as f:
        for line in f:
            stripped = line.rstrip()
            if not stripped or stripped.lstrip().startswith('#'):
                continue
            if 'ros__parameters' in stripped:
                in_params = True
                continue
            # indent 0 = yeni top-level key → ros__parameters bitti
            if in_params and stripped and not stripped.startswith(' ') \
                    and ':' in stripped:
                in_params = False
                continue
            if in_params and stripped.startswith(' '):
                kv = stripped.strip()
                if ':' in kv:
                    k, v = kv.split(':', 1)
                    k = k.strip()
                    v = v.strip().split('#')[0].strip()
                    try:
                        if '.' in v:
                            params[k] = float(v)
                        else:
                            params[k] = int(v)
                    except ValueError:
                        params[k] = v.strip("'\"")
    return params


def parse_sweep_config(config_json: str) -> dict:
    """Sweep config JSON parse + validate (test edilebilir).

    Format: {"param": "<param_name>", "values": [<v1>, <v2>, ...]}
    """
    cfg = json.loads(config_json)
    if 'param' not in cfg or not isinstance(cfg['param'], str):
        raise ValueError('sweep config: "param" (string) required')
    if 'values' not in cfg or not isinstance(cfg['values'], list):
        raise ValueError('sweep config: "values" (list) required')
    if not cfg['values']:
        raise ValueError('sweep config: "values" empty')
    return cfg
