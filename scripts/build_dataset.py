"""데이터 수집 기록(data/raw/)으로 학습·평가용 데이터셋을 만듭니다 (설계: edit_analysis.md 3.3, 3.4)

입력: data/raw/manifest.csv 와 실행마다 <run_id>_sensor.csv, <run_id>_machine.csv (화면의 '데이터 수집'으로 기록)
출력 (data/dataset/):
  windows.csv     윈도우마다 특징값·절삭 조건·판정 구간 여부·라벨·창 설정(profile)
                  창은 실행의 센서 샘플링으로 정합니다: 400 Hz 이상이면 0.2초 창·0.05초 간격('fast'),
                  아니면 1초·0.25초('legacy'). 특징은 실시간 감지와 같은 src/features.py로 계산합니다.
  runs.csv        실행마다 대표 조건(판정 구간의 중앙값)과 채터 윈도우 비율
  labels.csv      라벨 구간 원본 (실행, 라벨, 시작·끝 서버 시각) → evaluate_detector.py가 감지 지연을 구간 시작부터 잼
  ramp_limits.csv 경사 절입 시험(run_type=ramp)에서 채터가 시작된 깊이 = 회전수별 안정 한계 추정값 (edit_analysis2.md P10)
                  onset_ap: 라벨 앞 3초 안에서 신호로 다시 찾은 시작 깊이 (작업자 반응 지연 보정, refine_onset)
                  label_ap: 라벨 시각의 깊이, censored=1: 끝까지 채터가 없던 실행 (한계 ≥ max_ap, 중도절단)
                  → scripts/train_models.py가 안정 한계 모델(SLD)을 학습하고, 회전수별 최솟값은
                    scripts/make_experiment_nc.py grid --limits 에 넣어 본 실험 깊이를 정합니다.
판정 구간: 가공 중(State=running), GRBL Run, Z ≤ -gate_depth, 스핀들 회전 (실시간 감지와 같은 규칙).
라벨: 윈도우가 라벨 구간과 overlap 이상 겹치면 그 라벨 (채터 > 전이 > 비가공), 아니면 판정 구간이면 stable.
"""
import argparse
import csv
import math
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
ONSET_SEARCH_SEC = 3.0  # ramp: 라벨 앞 이 시간 안에서 채터가 시작된 점을 신호로 다시 찾음 (작업자 반응 지연 보정)
LABEL_COLUMNS = ['run_id', 'run_type', 'label', 'start', 'end', 'start_x', 'end_x', 'note']
FEATURE_COLUMNS = (features.TIME_FEATURE_NAMES
                   + ['rms', 'saturation', 'fs', 'spectral', 'nonharmonic_ratio', 'peak_freq', 'peak_prominence',
                      'spectral_entropy', 'rotation_hz', 'rotation_source', 'masked_fraction', 'block_rms'])


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


def onset_statistics(rows):
    """채터가 시작되면 커지는 윈도우 통계: 진동 크기(rms)와, 주파수 특징이 있으면 비고조파 진동 크기(rms·√비고조파 비율)"""
    stats = {'rms': np.array([r['rms'] for r in rows], dtype=float)}
    if all(r.get('nonharmonic_ratio') is not None and np.isfinite(r['nonharmonic_ratio']) for r in rows):
        stats['nonharmonic_rms'] = np.array([r['rms'] * math.sqrt(max(r['nonharmonic_ratio'], 0.0)) for r in rows])
    return stats


def knee_fit(t, x, search_sec=ONSET_SEARCH_SEC):
    """'직선 위에 꺾이는 점부터 더해지는 상승, 채터가 다 커지면 다시 평평'을 최소제곱으로 맞춥니다.
    꺾이는 점 후보는 t가 -search_sec..0인 윈도우, 다 커지는 점 후보는 그 뒤의 윈도우(끝까지 커지면 마지막)입니다.
    다 커지는 점이 없으면 깊이가 빨리 늘 때(짧은 ramp) 상승 뒤의 평평한 구간 때문에 꺾이는 점이 앞당겨집니다.
    반환: (꺾이는 점, 꺾임 없는 직선보다 줄어든 오차 비율). 상승이 양수인 후보가 없으면 (None, 0)."""
    line = np.column_stack([np.ones_like(t), t])
    line_sse = float(np.sum((x - line @ np.linalg.lstsq(line, x, rcond=None)[0]) ** 2))
    best = None
    for knee in t[(t >= -search_sec) & (t <= 0.0)]:
        for top in t[t > knee]:
            design = np.column_stack([np.ones_like(t), t, np.clip(t - knee, 0.0, top - knee)])
            coef = np.linalg.lstsq(design, x, rcond=None)[0]
            sse = float(np.sum((x - design @ coef) ** 2))
            if coef[2] > 0 and (best is None or sse < best[0]):
                best = (sse, knee)
    if best is None or line_sse <= 0:
        return None, 0.0
    return float(best[1]), 1.0 - best[0] / line_sse


def refine_onset(rows, label_start, window_sec):
    """ramp 실행에서 채터가 시작된 시각을 신호로 다시 찾습니다 (edit_analysis2.md P10, 분석 2.5).

    작업자는 채터를 듣고 늦게 라벨을 누르므로 라벨 시각의 깊이는 실제 한계보다 깊습니다(1–2초면 0.04–0.08 mm).
    라벨 앞 ONSET_SEARCH_SEC 안에서 통계(onset_statistics)마다 꺾이는 점을 찾고(knee_fit), 오차가 더 많이 준 통계의 점을
    씁니다. 깊이가 늘며 진동이 조금씩 커지는 것은 바탕 직선이 흡수합니다. 강한 채터가 회전 고조파 위치 근처에
    생기면 비고조파 비율이 오히려 떨어질 수 있어, 진동 크기(rms)도 함께 봅니다.
    오차가 30% 넘게 줄지 않으면 라벨 시각을 그대로 씁니다. 꺾이는 점은 윈도우 끝 시각이라 윈도우 길이의 절반을 뺍니다.
    rows: 판정 구간 윈도우(시간 순서), 반환: (시작 시각, 'signal (통계 이름)' 또는 'label')
    """
    span = [r for r in rows if label_start - ONSET_SEARCH_SEC - 1.0 <= r['t_host'] <= label_start + 1.0]
    if len(span) < 8:
        return label_start, 'label'
    t = np.array([r['t_host'] for r in span]) - label_start
    fits = {name: knee_fit(t, x) for name, x in onset_statistics(span).items()}
    name, (knee, reduction) = max(fits.items(), key=lambda item: item[1][1])
    if knee is None or reduction < 0.3:
        return label_start, 'label'
    return label_start + knee - window_sec / 2, f'signal ({name})'


def ramp_limit(run, cut):
    """ramp 실행 하나의 안정 한계 행 (ramp_limits.csv). 채터·전이 라벨이 없으면 중도절단(censored)."""
    t_cut = np.array([r['t_host'] for r in cut])
    ap_cut = np.array([r['ap'] for r in cut], dtype=float)
    labels = sorted((seg for seg in run['labels'] if seg.get('label') in ('chatter', 'transition') and seg.get('start') is not None),
                    key=lambda seg: seg['start'])
    row = {'run_id': run['run_id'], 'n': float(np.nanmedian([r['n'] for r in cut])),
           'F': float(np.nanmedian([r['F'] for r in cut])), 'flutes': run.get('flutes'), 'tool_id': run.get('tool_id'),
           'onset_ap': '', 'label_ap': '', 'onset_method': '', 'onset_shift_s': '', 'censored': 0,
           'max_ap': round(float(np.nanmax(ap_cut)), 3), 'note': ''}
    if not labels:
        row.update(censored=1, note='채터 없음 (최대 깊이까지 안정, 중도절단)')
        return row
    window_sec = features.WINDOW_PROFILES[cut[0]['profile']]['window_sec']
    start = labels[0]['start']
    onset, method = refine_onset(cut, start, window_sec)
    row.update(onset_ap=round(float(np.interp(onset, t_cut, ap_cut)), 3), label_ap=round(float(np.interp(start, t_cut, ap_cut)), 3),
               onset_method=method, onset_shift_s=round(start - onset, 2))
    if method == 'label':
        row['note'] = '신호에서 시작점을 찾지 못해 라벨 시각 사용'
    return row


def build_run(raw_dir, run, config, overlap):
    segments, timing = load_sensor(os.path.join(raw_dir, run['sensor_file']))
    machine = pd.read_csv(os.path.join(raw_dir, run['machine_file']))
    if machine.empty or not segments:
        return [], timing
    lookup = machine_lookup(machine, config)
    base = features.feature_config(dict(config, accel_range_g=float(run.get('sensor_range_g') or config['accel_range_g'])))
    rates = [fs for fs in (features.estimate_fs(seg[1]) for seg in segments) if fs]
    feat_config = features.profile_config(base, features.window_profile(np.median(rates) if rates else None, base))
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
            row = {'run_id': run['run_id'], 'run_type': run.get('run_type', ''), 'profile': feat_config['profile'],
                   't_host': round(host_end, 4),
                   't_rel': round(host_end - run_start, 3), 'gate': int(state['gate']),
                   'transient': int(state['since_change'] < config['transient_sec'] - 1e-6), 'label': label,
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
    all_rows, run_rows, ramp_rows, label_rows = [], [], [], []
    for run in runs:
        try:
            rows, timing = build_run(args.raw, run, config, args.overlap)
        except (OSError, KeyError, ValueError) as e:
            print(f"  [건너뜀] {run['run_id']}: {e}")
            continue
        all_rows.extend(rows)
        label_rows.extend(dict({key: seg.get(key) for key in LABEL_COLUMNS[2:]}, run_id=run['run_id'],
                               run_type=run.get('run_type', '')) for seg in run['labels'])
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
            ramp_rows.append(ramp_limit(run, cut))
        print(f"  {run['run_id']}: 윈도우 {len(rows)}개 (판정 구간 {len(cut)}, 채터 {len(chatter)}), 시각={timing}")

    os.makedirs(args.out, exist_ok=True)
    pd.DataFrame(all_rows).to_csv(os.path.join(args.out, 'windows.csv'), index=False, encoding='utf-8-sig')
    pd.DataFrame(run_rows).to_csv(os.path.join(args.out, 'runs.csv'), index=False, encoding='utf-8-sig')
    pd.DataFrame(label_rows, columns=LABEL_COLUMNS).to_csv(os.path.join(args.out, 'labels.csv'), index=False,
                                                           encoding='utf-8-sig', float_format='%.6f')
    if ramp_rows:
        with open(os.path.join(args.out, 'ramp_limits.csv'), 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=list(ramp_rows[0]))
            writer.writeheader()
            writer.writerows(ramp_rows)
        # 같은 회전수를 여러 번 시험했으면 가장 얕은 시작 깊이 (보수적으로)
        lowest = {}
        for r in ramp_rows:
            if r['onset_ap'] != '' and r['n']:
                lowest[int(round(r['n']))] = min(r['onset_ap'], lowest.get(int(round(r['n'])), math.inf))
        if lowest:
            print(f"안정 한계 추정 (grid --limits 입력값): {','.join(f'{n}={ap}' for n, ap in sorted(lowest.items()))}")
        shifts = [r['onset_shift_s'] for r in ramp_rows if r['onset_method'].startswith('signal')]
        if shifts:
            print(f"  ramp {len(ramp_rows)}개: 신호로 시작점을 다시 찾은 실행 {len(shifts)}개 (라벨보다 평균 {np.mean(shifts):.2f}초 앞), "
                  f"중도절단 {sum(r['censored'] for r in ramp_rows)}개")
    print(f"데이터셋 저장: {args.out} (실행 {len(run_rows)}개, 윈도우 {len(all_rows)}개)")


if __name__ == '__main__':
    main()
