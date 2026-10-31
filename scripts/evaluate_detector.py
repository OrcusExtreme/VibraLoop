"""이상 진동 감지기 평가 (설계: edit_analysis.md 3.4)

  python scripts/evaluate_detector.py              # data/dataset (데이터 수집 기록): 실행 단위 교차 검증
  python scripts/evaluate_detector.py --legacy     # data/학습 데이터 (예전 CSV): 파일 단위 교차 검증
  python scripts/evaluate_detector.py --set k=3 --set self_ratio=2.5   # 판정 설정을 바꿔 비교

지표
- 채터 사례별 감지율과 감지 지연(초): 라벨이 붙은 채터 구간마다 경보가 났는지, 구간 시작부터 몇 초 뒤인지
- 오경보: 채터 구간 근처(앞뒤 2초)가 아닌 곳의 경보 수, 안정 가공 1시간당 횟수
감지기는 실시간과 같은 코드(src/vibration_monitor.py의 fit_detector, DetectionCore)로 학습·판정합니다.
평가하는 실행·파일은 학습에서 빼므로(교차 검증) 처음 보는 가공에서의 성능을 추정합니다.
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
from sklearn.model_selection import GroupKFold

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

import features  # noqa: E402
import vibration_monitor as vm  # noqa: E402

SPECTRAL_KEYS = ('nonharmonic_ratio', 'peak_freq', 'peak_prominence', 'spectral_entropy')


def parse_overrides(items):
    overrides = {}
    for item in items or []:
        key, _, value = item.partition('=')
        if key not in vm.DETECTION_DEFAULTS:
            sys.exit(f"알 수 없는 설정: {key} (가능: {', '.join(vm.DETECTION_DEFAULTS)})")
        default = vm.DETECTION_DEFAULTS[key]
        if isinstance(default, bool):
            overrides[key] = value.lower() in ('1', 'true', 'yes')
        elif isinstance(default, int):
            overrides[key] = int(value)
        elif isinstance(default, float):
            overrides[key] = float(value)
        else:
            overrides[key] = value
    return overrides


def row_features(row):
    feats = {name: float(row[name]) for name in features.TIME_FEATURE_NAMES + ['rms']}
    feats['spectral'] = bool(row.get('spectral')) and str(row.get('spectral')).lower() not in ('false', '0', 'nan')
    for key in SPECTRAL_KEYS:
        value = row.get(key)
        feats[key] = None if value is None or pd.isna(value) else float(value)
    return feats


def simulate_run(core, run_rows):
    """실행 하나를 시간 순서대로 판정 → 경보 시작 시각 목록"""
    alarms = []
    in_cut = False
    for _, row in run_rows.iterrows():
        if not row['gate']:
            if in_cut:
                core.new_cut()
            in_cut = False
            continue
        if not in_cut:
            core.new_cut()
            in_cut = True
        if row['transient']:
            continue
        cond = {'n': row['n'], 'F': row['F'], 'ap': row['ap']}
        step = core.step(row_features(row), cond)
        if step['decision'] == 'start':
            alarms.append(row['t_host'])
    return alarms


def chatter_events(run_rows, window_sec, max_gap=0.6):
    """연속한 채터 윈도우 → (시작 시각, 끝 시각) 사례 목록"""
    events = []
    for _, row in run_rows[run_rows['label'] == 'chatter'].iterrows():
        start, end = row['t_host'] - window_sec, row['t_host']
        if events and start - events[-1][1] <= max_gap:
            events[-1][1] = end
        else:
            events.append([start, end])
    return events


def evaluate_dataset(dataset_dir, config, hop):
    path = os.path.join(dataset_dir, 'windows.csv')
    if not os.path.exists(path):
        sys.exit(f"{path}가 없습니다. 먼저 python scripts/build_dataset.py를 실행하세요.")
    df = pd.read_csv(path, encoding='utf-8-sig').sort_values(['run_id', 't_host'])
    run_ids = df['run_id'].unique()
    if len(run_ids) < 2:
        sys.exit("교차 검증에는 실행이 2개 이상 필요합니다.")
    window_sec = config.get('window_sec', features.DEFAULT_FEATURE_CONFIG['window_sec'])
    detected, latencies, events_total, false_alarms, stable_seconds = 0, [], 0, 0, 0.0
    folds = GroupKFold(n_splits=min(5, len(run_ids))).split(run_ids, groups=run_ids)
    for fold, (train_idx, test_idx) in enumerate(folds, start=1):
        train_runs, test_runs = set(run_ids[train_idx]), set(run_ids[test_idx])
        sessions = []
        for run_id in train_runs:
            stable = df[(df['run_id'] == run_id) & (df['gate'] == 1) & (df['transient'] == 0) & (df['label'] == 'stable')]
            sessions.append([row_features(row) for _, row in stable.iterrows()])
        try:
            model = vm.fit_detector(sessions, config)
        except ValueError as e:
            print(f"  fold {fold}: 학습 실패 ({e})")
            continue
        core = vm.DetectionCore(model, dict(vm.DETECTION_DEFAULTS, **config))
        for run_id in sorted(test_runs):
            rows = df[df['run_id'] == run_id]
            alarms = simulate_run(core, rows)
            events = chatter_events(rows, window_sec)
            matched = set()
            for start, end in events:
                hits = [a for a in alarms if start - 0.5 <= a <= end + 1.0]
                events_total += 1
                if hits:
                    detected += 1
                    latencies.append(hits[0] - start)
                    matched.update(hits)
            for alarm in alarms:
                if alarm in matched or any(start - 2.0 <= alarm <= end + 2.0 for start, end in events):
                    continue
                false_alarms += 1
            stable_seconds += ((rows['gate'] == 1) & (rows['label'] == 'stable')).sum() * hop
            print(f"  {run_id}: 채터 사례 {len(events)}개, 경보 {len(alarms)}회")
    print_summary(events_total, detected, latencies, false_alarms, stable_seconds)


def evaluate_legacy(config):
    """예전 학습 데이터: 정상 파일은 하나씩 빼고 학습해 오경보를, '이상' 파일은 전체 정상 파일로 학습해 감지를 봅니다.
    기계 상태 기록이 없어 진동 크기(idle_rms_g 이상)로 가공 구간을 판단합니다."""
    files = sorted(glob.glob(os.path.join(BASE_DIR, 'data', '학습 데이터', '*.csv')))
    normal = [f for f in files if '이상' not in os.path.basename(f)]
    abnormal = [f for f in files if '이상' in os.path.basename(f)]
    data = {f: features.load_sensor_csv(f) for f in files}
    full = dict(vm.DETECTION_DEFAULTS, **config)
    fc = features.feature_config(full)

    def run(model, d):
        core = vm.DetectionCore(model, full)
        alarms, active = 0, 0
        for acc, t in d['segments']:
            core.new_cut()
            for _, feats in features.session_windows(acc, t, fc, active_only=False):
                if feats['rms'] < fc['idle_rms_g']:
                    core.new_cut()
                    continue
                active += 1
                alarms += core.step(feats)['decision'] == 'start'
        return alarms, active

    false_alarms, seconds = 0, 0.0
    for f in normal:
        model = vm.train_detector([data[g] for g in normal if g != f], full)
        alarms, active = run(model, data[f])
        false_alarms += alarms
        seconds += active * fc['hop_sec']
        print(f"  정상 {os.path.basename(f)}: 가공 윈도우 {active}개, 경보 {alarms}회")
    model = vm.train_detector([data[f] for f in normal], full)
    for f in abnormal:
        alarms, active = run(model, data[f])
        print(f"  이상 {os.path.basename(f)}: 가공 윈도우 {active}개, 경보 {alarms}회")
    print(f"정상 오경보: {false_alarms}회 / 가공 {seconds / 60:.1f}분 ({false_alarms / max(seconds / 3600, 1e-9):.0f}회/시간)")
    print("주의: 예전 데이터는 이상 파일이 1개뿐이고 시각·조건 기록이 없어 참고용입니다.")


def print_summary(events_total, detected, latencies, false_alarms, stable_seconds):
    print("\n결과")
    if events_total:
        print(f"  채터 감지율: {detected}/{events_total} ({detected / events_total:.0%})")
    else:
        print("  채터 라벨이 없어 감지율을 계산하지 못했습니다.")
    if latencies:
        print(f"  감지 지연: 중앙값 {np.median(latencies):.2f}초, 최대 {np.max(latencies):.2f}초")
    hours = stable_seconds / 3600
    if hours > 0:
        print(f"  오경보: {false_alarms}회 / 안정 가공 {stable_seconds / 60:.1f}분 ({false_alarms / hours:.1f}회/시간)")


def main():
    parser = argparse.ArgumentParser(description="이상 진동 감지기 교차 검증")
    parser.add_argument('--dataset', default=os.path.join(BASE_DIR, 'data', 'dataset'))
    parser.add_argument('--legacy', action='store_true', help="data/학습 데이터(예전 CSV)로 평가")
    parser.add_argument('--set', action='append', metavar='KEY=VALUE', help="판정 설정 변경 (여러 번 사용 가능)")
    args = parser.parse_args()
    config = parse_overrides(args.set)
    print(f"판정 설정: {dict(vm.DETECTION_DEFAULTS, **config)}")
    if args.legacy:
        evaluate_legacy(config)
    else:
        evaluate_dataset(args.dataset, config, features.DEFAULT_FEATURE_CONFIG['hop_sec'])


if __name__ == '__main__':
    main()
