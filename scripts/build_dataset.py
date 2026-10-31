"""데이터 수집 기록(data/raw/)으로 학습·평가용 데이터셋을 만듭니다 (설계: edit_analysis.md 3.3, 3.4)

입력: data/raw/manifest.csv 와 실행마다 <run_id>_sensor.csv, <run_id>_machine.csv (화면의 '데이터 수집'으로 기록)
출력 (data/dataset/):
  windows.csv     윈도우(1초, 0.25초 간격)마다 특징값·절삭 조건·판정 구간 여부·라벨
                  특징은 실시간 감지와 같은 src/features.py로 계산합니다.
  runs.csv        실행마다 대표 조건(판정 구간의 중앙값)과 채터 윈도우 비율
  ramp_limits.csv 경사 절입 시험(run_type=ramp)에서 채터가 처음 나타난 깊이 = 회전수별 안정 한계 추정값
                  → scripts/make_experiment_nc.py grid --limits 에 넣어 본 실험 깊이를 정합니다.
판정 구간: 가공 중(State=running), GRBL Run, Z ≤ -gate_depth, 스핀들 회전 (실시간 감지와 같은 규칙).
라벨: 윈도우가 라벨 구간과 overlap 이상 겹치면 그 라벨 (채터 > 전이 > 비가공), 아니면 판정 구간이면 stable.
"""
import argparse
import csv
import os
import sys

import numpy as np
import pandas as pd

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

import features  # noqa: E402
from data_recorder import read_manifest  # noqa: E402
from vibration_monitor import DETECTION_DEFAULTS  # noqa: E402

LABEL_PRIORITY = ['chatter', 'transition', 'air']
FEATURE_COLUMNS = (features.TIME_FEATURE_NAMES
                   + ['rms', 'saturation', 'fs', 'spectral', 'nonharmonic_ratio', 'peak_freq', 'peak_prominence',
                      'spectral_entropy'])


def _num(series):
    return pd.to_numeric(series, errors='coerce')


def load_sensor(path):
    """센서 기록 → [(acc, 측정 시각 t, t→서버 시각 변환 함수)] 연속 구간 목록과 시각 종류"""
    df = pd.read_csv(path)
    acc = df[['AccX', 'AccY', 'AccZ']].apply(pd.to_numeric, errors='coerce')
    host = _num(df['HostTime'])
    device = _num(df['DeviceTime']) if 'DeviceTime' in df.columns else None
    use_device = device is not None and device.notna().mean() > 0.9
    valid = acc.notna().all(axis=1) & host.notna() & (device.notna() if use_device else True)
    acc = acc[valid].to_numpy(float)
    host = host[valid].to_numpy(float)
    t = device[valid].to_numpy(float) if use_device else host
    breaks = np.flatnonzero((np.diff(t) < 0) | (np.diff(t) > 1.0)) + 1
    segments = []
    for a, b in zip(np.r_[0, breaks], np.r_[breaks, len(t)]):
        if b - a < 20:
            continue
        if use_device:
            # 장치 시각 → 서버 시각: 센서 클럭 오차·수신 지연을 직선으로 맞춤
            slope, intercept = np.polyfit(t[a:b], host[a:b], 1)
            to_host = (lambda s, i: (lambda x: s * x + i))(slope, intercept)
        else:
            to_host = (lambda x: x)
        segments.append((acc[a:b], t[a:b], to_host))
    return segments, 'device' if use_device else 'host'


def machine_lookup(machine, config):
    """가공 로그 → 시각별 (판정 구간, 조건 변경 후 경과 시간, 행) 조회 함수"""
    machine = machine.sort_values('HostTime').reset_index(drop=True)
    times = _num(machine['HostTime']).to_numpy(float)
    z = _num(machine['Z']).to_numpy(float)
    n = _num(machine['SpindleRPM']).to_numpy(float)
    F = _num(machine['FeedRate']).to_numpy(float)
    spindle_rt = _num(machine['SpindleRT']).to_numpy(float) if 'SpindleRT' in machine.columns else np.full(len(times), np.nan)
    spinning = np.where(np.isnan(spindle_rt), np.nan_to_num(n) > 0, spindle_rt > 0)
    gate = ((machine['State'].astype(str) == 'running') & (machine['GrblState'].astype(str) == 'Run')).to_numpy() \
        & (z <= -config['gate_depth_mm']) & spinning
    signature = list(zip(gate, np.round(np.nan_to_num(n)), np.round(np.nan_to_num(F), 1)))
    changed_at = np.empty(len(times))
    for i in range(len(times)):
        changed_at[i] = times[i] if i == 0 or signature[i] != signature[i - 1] else changed_at[i - 1]

    def at(host_time):
        i = int(np.searchsorted(times, host_time, side='right')) - 1
        if i < 0:
            return None
        return {'gate': bool(gate[i]), 'since_change': host_time - changed_at[i], 'n': n[i], 'F': F[i], 'z': z[i],
                'x': float(_num(machine['X']).iloc[i])}
    return at


def window_label(labels, start, end, overlap):
    for name in LABEL_PRIORITY:
        for seg in labels:
            if seg.get('label') != name or seg.get('end') is None:
                continue
            shared = min(end, seg['end']) - max(start, seg['start'])
            if shared >= overlap * (end - start):
                return name
    return None


def build_run(raw_dir, run, config, overlap):
    segments, timing = load_sensor(os.path.join(raw_dir, run['sensor_file']))
    machine = pd.read_csv(os.path.join(raw_dir, run['machine_file']))
    if machine.empty or not segments:
        return [], timing
    lookup = machine_lookup(machine, config)
    feat_config = features.feature_config(dict(config, accel_range_g=float(run.get('sensor_range_g') or config['accel_range_g'])))
    run_start = min(_num(machine['HostTime']).min(), min(seg[2](seg[1][0]) for seg in segments))
    rows = []
    for acc, t, to_host in segments:
        nominal_fs = features.estimate_fs(t)
        for i0, i1, t_end in features.window_ranges(t, feat_config):
            if not features.window_is_usable(t[i0:i1], feat_config, nominal_fs):
                continue
            host_end = to_host(t_end)
            state = lookup(host_end)
            if state is None:
                continue
            feats = features.compute_features(acc[i0:i1], t[i0:i1], feat_config,
                                              state['n'] if np.isfinite(state['n']) else None)
            label = window_label(run['labels'], host_end - feat_config['window_sec'], host_end, overlap)
            if label is None:
                label = 'stable' if state['gate'] else 'noncut'
            row = {'run_id': run['run_id'], 'run_type': run.get('run_type', ''), 't_host': round(host_end, 4),
                   't_rel': round(host_end - run_start, 3), 'gate': int(state['gate']),
                   'transient': int(state['since_change'] < config['transient_sec']), 'label': label,
                   'n': state['n'], 'F': state['F'], 'ap': max(0.0, -state['z']) if np.isfinite(state['z']) else None,
                   'ae': _num(pd.Series([run.get('ae_mm')])).iloc[0], 'flutes': _num(pd.Series([run.get('flutes')])).iloc[0],
                   'x': state['x'], 'timing': timing}
            row.update({name: feats.get(name) for name in FEATURE_COLUMNS})
            rows.append(row)
    return rows, timing


def main():
    parser = argparse.ArgumentParser(description="데이터 수집 기록으로 학습·평가용 데이터셋 생성")
    parser.add_argument('--raw', default=os.path.join(BASE_DIR, 'data', 'raw'))
    parser.add_argument('--out', default=os.path.join(BASE_DIR, 'data', 'dataset'))
    parser.add_argument('--overlap', type=float, default=0.5, help="라벨 구간과 이 비율 이상 겹치면 그 라벨")
    args = parser.parse_args()

    runs = read_manifest(args.raw)
    if not runs:
        sys.exit(f"{args.raw}/manifest.csv가 없습니다. 화면의 '데이터 수집'으로 먼저 기록하세요.")
    config = dict(DETECTION_DEFAULTS, **features.DEFAULT_FEATURE_CONFIG)
    all_rows, run_rows, ramp_rows = [], [], []
    for run in runs:
        try:
            rows, timing = build_run(args.raw, run, config, args.overlap)
        except (OSError, KeyError, ValueError) as e:
            print(f"  [건너뜀] {run['run_id']}: {e}")
            continue
        all_rows.extend(rows)
        cut = [r for r in rows if r['gate'] and not r['transient'] and r['label'] in ('stable', 'chatter', 'transition')]
        chatter = [r for r in cut if r['label'] == 'chatter']
        summary = {'run_id': run['run_id'], 'run_type': run.get('run_type', ''), 'timing': timing,
                   'windows': len(rows), 'cut_windows': len(cut), 'chatter_windows': len(chatter),
                   'chatter_fraction': round(len(chatter) / len(cut), 3) if cut else None,
                   'n': np.nanmedian([r['n'] for r in cut]) if cut else None,
                   'F': np.nanmedian([r['F'] for r in cut]) if cut else None,
                   'ap': np.nanmedian([r['ap'] for r in cut if r['ap'] is not None]) if cut else None,
                   'ae': run.get('ae_mm'), 'flutes': run.get('flutes')}
        run_rows.append(summary)
        if run.get('run_type') == 'ramp' and cut:
            onset = next((r for r in cut if r['label'] in ('chatter', 'transition')), None)
            ramp_rows.append({'run_id': run['run_id'], 'n': summary['n'], 'F': summary['F'],
                              'onset_ap': round(onset['ap'], 3) if onset else '',
                              'max_ap': round(max(r['ap'] for r in cut), 3),
                              'note': '' if onset else '채터 없음 (최대 깊이까지 안정)'})
        print(f"  {run['run_id']}: 윈도우 {len(rows)}개 (판정 구간 {len(cut)}, 채터 {len(chatter)}), 시각={timing}")

    os.makedirs(args.out, exist_ok=True)
    pd.DataFrame(all_rows).to_csv(os.path.join(args.out, 'windows.csv'), index=False, encoding='utf-8-sig')
    pd.DataFrame(run_rows).to_csv(os.path.join(args.out, 'runs.csv'), index=False, encoding='utf-8-sig')
    if ramp_rows:
        with open(os.path.join(args.out, 'ramp_limits.csv'), 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=list(ramp_rows[0]))
            writer.writeheader()
            writer.writerows(ramp_rows)
        limits = ",".join(f"{int(r['n'])}={r['onset_ap']}" for r in ramp_rows if r['onset_ap'] != '' and r['n'])
        if limits:
            print(f"안정 한계 추정 (grid --limits 입력값): {limits}")
    print(f"데이터셋 저장: {args.out} (실행 {len(run_rows)}개, 윈도우 {len(all_rows)}개)")


if __name__ == '__main__':
    main()
