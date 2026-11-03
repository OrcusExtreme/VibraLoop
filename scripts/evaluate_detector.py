"""이상 진동 감지기 평가 (설계: edit_analysis.md 3.4, edit_analysis2.md P01)

  python scripts/evaluate_detector.py              # data/dataset (데이터 수집 기록): 실행 단위 교차 검증
  python scripts/evaluate_detector.py --legacy     # data/학습 데이터 (예전 CSV): 파일 단위 교차 검증
  python scripts/evaluate_detector.py --set confirm_sec=0.5 --set self_ratio=2.5   # 판정 설정을 바꿔 비교
  python scripts/evaluate_detector.py --set decision_mode=two_tier     # 2단계 판정(빠른 차단 + CUSUM)으로 평가
  python scripts/evaluate_detector.py --legacy --set pca_variance=0.95 --set t2_alarm=true   # 감지 모델(PCA) 설정 비교
  python scripts/evaluate_detector.py --condition-model models/condition_model.joblib   # 조건 모델(P09)을 넣어 평가
  python scripts/evaluate_detector.py --dataset data/synthetic/dataset --json result.json   # 결과를 JSON으로 저장
  python scripts/evaluate_detector.py --legacy --compare tests/baseline/v1.1_legacy.json   # 저장한 결과와 나란히 비교

지표
- 채터 사례별 감지율과 감지 지연(초): 라벨 구간마다 경보가 났는지, 구간 시작부터 몇 초 뒤인지 (중앙값·90%·최대).
  구간은 build_dataset.py의 labels.csv(라벨 시각 그대로)를 쓰고, 없으면 채터 윈도우를 이어 붙여 추정합니다.
  경보 시각은 경보를 낸 윈도우의 끝 시각이므로 지연은 윈도우 이동 간격 단위로 나옵니다.
- 오경보: 채터 구간 근처(앞뒤 2초)가 아닌 곳의 경보 수, 안정 가공 1시간당 횟수
- 실행 종류(run_type)별 집계, 경보 경로(판정 방식)별·경보 원인(경보 윈도우에서 가장 큰 증거 항목)별 감지·오경보 수
창 설정(센서 샘플링: 1초 창·0.25초 간격 / 0.2초 창·0.05초 간격)이 다른 실행은 따로 교차 검증합니다.
감지기는 실시간과 같은 코드(src/vibration_monitor.py의 fit_detector, DetectionCore)로 학습·판정하고, 학습 윈도우는
scripts/train_models.py가 감지 모델을 저장할 때와 같은 정상 윈도우입니다(normal_windows: 채터·전이 라벨 앞 3초 제외).
평가하는 실행·파일은 학습에서 빼므로(교차 검증) 처음 보는 가공에서의 성능을 추정합니다.
--set 값은 서버와 같은 규칙으로 검사합니다(vibration_monitor.setting_problem). 허용 범위 밖이면 평가하지 않습니다.
"""
import argparse
import glob
import json
import os
import sys
from collections import defaultdict

import numpy as np
import pandas as pd
from sklearn.model_selection import GroupKFold

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

import features  # noqa: E402
import vibration_monitor as vm  # noqa: E402
from train_models import LABEL_GUARD_SEC, label_starts, normal_windows, row_features  # noqa: E402

MATCH_BEFORE, MATCH_AFTER = 0.5, 1.0   # 채터 사례로 인정하는 경보 범위: 구간 시작 0.5초 전 ~ 끝 1초 뒤
FA_MARGIN = 2.0                        # 채터 구간 앞뒤 이 시간 안의 경보는 오경보로 세지 않음
BOOL_WORDS = {'1': True, 'true': True, 'yes': True, '0': False, 'false': False, 'no': False}


def parse_overrides(items):
    """--set KEY=VALUE 목록 → 판정 설정. 값의 형식은 기본값을 따르고, 서버와 같은 허용 범위로 검사합니다."""
    overrides = {}
    for item in items or []:
        key, _, value = item.partition('=')
        if key not in vm.DETECTION_DEFAULTS:
            sys.exit(f"알 수 없는 설정: {key} (가능: {', '.join(vm.DETECTION_DEFAULTS)})")
        default = vm.DETECTION_DEFAULTS[key]
        try:
            if isinstance(default, bool):
                parsed = BOOL_WORDS[value.lower()]
            elif isinstance(default, int):
                parsed = int(value)
            elif isinstance(default, float):
                parsed = float(value)
            else:
                parsed = value
        except (KeyError, ValueError):
            sys.exit(f"설정 값을 읽을 수 없습니다: {item}")
        problem = vm.setting_problem(key, parsed)
        if problem:
            sys.exit(f"쓸 수 없는 설정 값입니다: {problem}")
        overrides[key] = parsed
    return overrides


def simulate_run(core, run_rows):
    """실행 하나를 시간 순서대로 판정 → [(경보 시작 시각, 경보 경로, 실행 시작부터 초, 가장 큰 증거 항목)]
    실시간(VibrationMonitor._gate)과 같이 판정 구간 안에서 회전수·이송이 바뀌면 core.condition_changed()를 부릅니다."""
    alarms = []
    in_cut, last_condition = False, None
    for _, row in run_rows.iterrows():
        if not row['gate']:
            if in_cut:
                core.new_cut()
            in_cut = False
            continue
        condition = (round(row['n']) if pd.notna(row['n']) else 0, round(row['F'], 1) if pd.notna(row['F']) else 0)
        if not in_cut:
            core.new_cut()
            in_cut = True
        elif condition != last_condition:
            core.condition_changed()
        last_condition = condition
        if row['transient']:
            continue
        cond = {'n': row['n'], 'F': row['F'], 'ap': row['ap'], 'flutes': row.get('flutes'), 'ae': row.get('ae')}
        step = core.step(row_features(row), cond)
        if step['decision'] == 'start':
            alarms.append((row['t_host'], step.get('trip') or core.config['decision_mode'], row.get('t_rel'),
                           max(step['parts'], key=step['parts'].get)))
    return alarms


def chatter_events(run_rows, window_sec, max_gap=0.6):
    """연속한 채터 윈도우 → (시작 시각, 끝 시각) 사례 목록 (labels.csv가 없는 예전 데이터셋용)"""
    events = []
    for _, row in run_rows[run_rows['label'] == 'chatter'].iterrows():
        start, end = row['t_host'] - window_sec, row['t_host']
        if events and start - events[-1][1] <= max_gap:
            events[-1][1] = end
        else:
            events.append([start, end])
    return events


def load_label_events(dataset_dir):
    """labels.csv의 채터 라벨 구간 → {run_id: [[시작, 끝], ...]}. 파일이 없으면 None"""
    path = os.path.join(dataset_dir, 'labels.csv')
    if not os.path.exists(path):
        return None
    labels = pd.read_csv(path, encoding='utf-8-sig', dtype={'run_id': str})
    events = defaultdict(list)
    for row in labels[labels['label'] == 'chatter'].itertuples():
        if pd.notna(row.start) and pd.notna(row.end):
            events[row.run_id].append([float(row.start), float(row.end)])
    return {run_id: sorted(spans) for run_id, spans in events.items()}


def run_hop(rows):
    """실행의 윈도우 이동 간격(초) = 이웃한 윈도우 끝 시각 차이의 중앙값"""
    gaps = np.diff(np.sort(rows['t_host'].to_numpy(float)))
    gaps = gaps[gaps > 0]
    return float(np.median(gaps)) if len(gaps) else features.DEFAULT_FEATURE_CONFIG['hop_sec']


def _round(value):
    return round(float(value), 3)


class Tally:
    """감지율·지연·오경보 집계"""

    def __init__(self):
        self.runs = self.events = self.detected = self.false_alarms = 0
        self.latencies = []
        self.stable_seconds = 0.0

    def summary(self):
        lat, hours = self.latencies, self.stable_seconds / 3600
        return {
            'runs': self.runs, 'events': self.events, 'detected': self.detected,
            'detection_rate': round(self.detected / self.events, 3) if self.events else None,
            'latency_median_s': _round(np.median(lat)) if lat else None,
            'latency_p90_s': _round(np.percentile(lat, 90)) if lat else None,
            'latency_max_s': _round(np.max(lat)) if lat else None,
            'false_alarms': self.false_alarms,
            'stable_minutes': round(self.stable_seconds / 60, 2),
            'false_alarms_per_hour': round(self.false_alarms / hours, 2) if hours > 0 else None,
        }


def score_run(rows, alarms, events, tallies, trips, causes=None):
    """한 실행의 경보를 채터 사례와 맞춰 tallies에 더합니다. 반환: 감지한 사례의 지연 목록
    trips·causes: 경보 경로별·증거 항목별 감지·오경보 수 (causes는 경보 튜플의 네 번째 값, 없으면 세지 않음)"""
    matched, latencies = set(), []
    stable_seconds = ((rows['gate'] == 1) & (rows['label'] == 'stable')).sum() * run_hop(rows)
    for tally in tallies:
        tally.runs += 1
        tally.events += len(events)
        tally.stable_seconds += stable_seconds
    for start, end in events:
        hits = [a for a in alarms if start - MATCH_BEFORE <= a[0] <= end + MATCH_AFTER]
        if not hits:
            continue
        latencies.append(hits[0][0] - start)
        matched.update(hits)
        trips[hits[0][1]]['detections'] += 1
        if causes is not None:
            causes[hits[0][3]]['detections'] += 1
        for tally in tallies:
            tally.detected += 1
            tally.latencies.append(latencies[-1])
    for alarm in alarms:
        if alarm in matched or any(start - FA_MARGIN <= alarm[0] <= end + FA_MARGIN for start, end in events):
            continue
        trips[alarm[1]]['false_alarms'] += 1
        if causes is not None:
            causes[alarm[3]]['false_alarms'] += 1
        for tally in tallies:
            tally.false_alarms += 1
    return latencies


def evaluate_dataset(dataset_dir, config, condition_model=None):
    """condition_model: 조건 모델(vm.load_condition_model). 평가하는 실행과 다른 데이터(예: grid 시험)로 학습한 것을 씁니다."""
    path = os.path.join(dataset_dir, 'windows.csv')
    if not os.path.exists(path):
        sys.exit(f"{path}가 없습니다. 먼저 python scripts/build_dataset.py를 실행하세요.")
    df = pd.read_csv(path, encoding='utf-8-sig', dtype={'run_id': str}).sort_values(['run_id', 't_host'])
    if 'profile' not in df.columns:  # P04 이전 데이터셋은 모두 1초 창
        df['profile'] = 'legacy'
    full = dict(vm.DETECTION_DEFAULTS, **config)
    # 학습 윈도우: train_models가 감지 모델을 저장할 때와 같은 정상 윈도우 (늦은 라벨 앞 LABEL_GUARD_SEC 제외)
    normal = normal_windows(df, label_starts(dataset_dir, df))
    guarded = int(((df['gate'] == 1) & (df['transient'] == 0) & (df['label'] == 'stable')).sum()) - len(normal)
    print(f"  정상 학습 윈도우 {len(normal)}개 (채터·전이 라벨 시작 앞 {LABEL_GUARD_SEC:g}초 안의 {guarded}개 뺌)")
    label_events = load_label_events(dataset_dir)
    overall, by_type = Tally(), defaultdict(Tally)
    trips = defaultdict(lambda: {'detections': 0, 'false_alarms': 0})
    causes = defaultdict(lambda: {'detections': 0, 'false_alarms': 0})
    used_profiles = {}
    # 창 설정마다 따로 교차 검증합니다 (모델은 같은 창 설정의 윈도우로만 학습하므로)
    for profile in sorted(df['profile'].unique()):
        part = df[df['profile'] == profile]
        run_ids = part['run_id'].unique()
        label = vm.PROFILE_LABELS.get(profile, profile)
        if len(run_ids) < 2:
            print(f"  [{label}] 실행이 {len(run_ids)}개뿐이라 교차 검증하지 않습니다.")
            continue
        print(f"  [{label}] 실행 {len(run_ids)}개")
        used_profiles[profile] = dict(features.WINDOW_PROFILES[profile])
        window_sec = features.WINDOW_PROFILES[profile]['window_sec']
        folds = GroupKFold(n_splits=min(5, len(run_ids))).split(run_ids, groups=run_ids)
        for fold, (train_idx, test_idx) in enumerate(folds, start=1):
            train_runs, test_runs = set(run_ids[train_idx]), set(run_ids[test_idx])
            sessions = []
            for run_id in sorted(train_runs):
                stable = normal[(normal['profile'] == profile) & (normal['run_id'] == run_id)]
                sessions.append([row_features(row) for _, row in stable.iterrows()])
            try:
                model = vm.fit_detector(sessions, config, profile)
            except ValueError as e:
                print(f"  fold {fold}: 학습 실패 ({e})")
                continue
            core = vm.DetectionCore(model, full, condition_model)
            for run_id in sorted(test_runs):
                rows = part[part['run_id'] == run_id]
                alarms = simulate_run(core, rows)
                events = label_events.get(run_id, []) if label_events is not None else chatter_events(rows, window_sec)
                run_type = rows['run_type'].iloc[0] if 'run_type' in rows.columns else None
                run_type = str(run_type) if run_type is not None and pd.notna(run_type) and str(run_type) else '(없음)'
                latencies = score_run(rows, alarms, events, (overall, by_type[run_type]), trips, causes)
                when = (" (" + ", ".join(f"{_seconds(a[2])}초 {a[3]}" for a in alarms) + ")") if alarms else ""
                delay = (", 지연 " + ", ".join(f"{x:.2f}" for x in latencies) + "초") if latencies else ""
                print(f"  {run_id}: 채터 사례 {len(events)}개, 경보 {len(alarms)}회{when}{delay}")
    if not used_profiles:
        sys.exit("교차 검증에는 같은 창 설정(센서 샘플링)의 실행이 2개 이상 필요합니다.")
    result = {
        'mode': 'dataset', 'dataset': _display_path(dataset_dir),
        'events_source': 'labels' if label_events is not None else 'windows',
        'label_guard_sec': LABEL_GUARD_SEC, 'guarded_windows': guarded,
        'config': full, 'window_profiles': used_profiles,
        'overall': overall.summary(), 'by_type': {k: v.summary() for k, v in sorted(by_type.items())},
        'by_trip': {k: v for k, v in sorted(trips.items())}, 'by_cause': {k: v for k, v in sorted(causes.items())},
        'latencies_s': [_round(x) for x in overall.latencies],
        'condition_model': None if condition_model is None else {
            key: condition_model.get(key) for key in ('features', 'n_runs', 'n_rows', 'log_margin', 'coverage', 'trained_at')},
    }
    print_dataset_summary(result)
    return result


def evaluate_legacy(config):
    """예전 학습 데이터: 정상 파일은 하나씩 빼고 학습해 오경보를, '이상' 파일은 전체 정상 파일로 학습해 감지를 봅니다.
    기계 상태 기록이 없어 진동 크기(idle_rms_g 이상)로 가공 구간을 판단합니다."""
    files = sorted(glob.glob(os.path.join(BASE_DIR, 'data', '학습 데이터', '*.csv')))
    normal = [f for f in files if '이상' not in os.path.basename(f)]
    abnormal = [f for f in files if '이상' in os.path.basename(f)]
    data = {f: features.load_sensor_csv(f) for f in files}
    full = dict(vm.DETECTION_DEFAULTS, **config)
    base = features.feature_config(full)
    profiles = {f: features.window_profile(data[f]['fs'], base) for f in files}  # 파일마다 샘플링으로 창 설정

    def run(models, f):
        """파일 f를 그 창 설정의 모델로 판정 → (경보마다 가장 큰 증거 항목, 가공 윈도우 수, 평가 간격).
        그 설정의 모델이 없으면 None"""
        model = models.get(profiles[f])
        if model is None:
            return None
        fc = model['feature_config']
        core = vm.DetectionCore(model, full)
        alarms, active = [], 0
        for acc, t in data[f]['segments']:
            core.new_cut()
            for _, feats in features.session_windows(acc, t, fc, active_only=False):
                if feats['rms'] < fc['idle_rms_g']:
                    core.new_cut()
                    continue
                active += 1
                step = core.step(feats)
                if step['decision'] == 'start':
                    alarms.append(max(step['parts'], key=step['parts'].get))
        return alarms, active, fc['hop_sec']

    def counted(alarms):
        return {cause: alarms.count(cause) for cause in sorted(set(alarms))}

    false_alarms, seconds, per_file, causes = 0, 0.0, {}, []
    for f in normal:
        result = run(vm.train_detectors([data[g] for g in normal if g != f], full), f)
        if result is None:
            print(f"  정상 {os.path.basename(f)}: 같은 창 설정의 다른 학습 파일이 없어 건너뜀")
            continue
        alarms, active, hop = result
        false_alarms += len(alarms)
        causes += alarms
        seconds += active * hop
        per_file[os.path.basename(f)] = len(alarms)
        print(f"  정상 {os.path.basename(f)}: 가공 윈도우 {active}개, 경보 {len(alarms)}회"
              + (f" ({', '.join(alarms)})" if alarms else ""))
    models = vm.train_detectors([data[f] for f in normal], full)
    abnormal_result = {}
    for f in abnormal:
        result = run(models, f)
        if result is None:
            print(f"  이상 {os.path.basename(f)}: 같은 창 설정의 학습 파일이 없어 건너뜀")
            continue
        alarms, active, _ = result
        abnormal_result[os.path.basename(f)] = {'alarms': len(alarms), 'windows': active, 'causes': counted(alarms)}
        print(f"  이상 {os.path.basename(f)}: 가공 윈도우 {active}개, 경보 {len(alarms)}회"
              + (f" ({', '.join(alarms)})" if alarms else ""))
    per_hour = false_alarms / max(seconds / 3600, 1e-9)
    print(f"정상 오경보: {false_alarms}회 / 가공 {seconds / 60:.1f}분 ({per_hour:.0f}회/시간)")
    print("주의: 예전 데이터는 이상 파일이 1개뿐이고 시각·조건 기록이 없어 참고용입니다.")
    return {
        'mode': 'legacy', 'config': full,
        'window_profiles': {p: dict(features.WINDOW_PROFILES[p]) for p in sorted(set(profiles.values()))},
        'normal': {'files': len(normal), 'false_alarms': false_alarms, 'cut_minutes': round(seconds / 60, 2),
                   'false_alarms_per_hour': round(per_hour, 2), 'per_file': per_file, 'causes': counted(causes)},
        'abnormal': abnormal_result,
    }


def _display_path(path):
    """JSON에 남길 경로 (프로젝트 안이면 상대 경로)"""
    absolute = os.path.abspath(path)
    try:
        inside = os.path.commonpath([absolute, BASE_DIR]) == BASE_DIR
    except ValueError:  # 다른 드라이브
        inside = False
    return os.path.relpath(absolute, BASE_DIR).replace('\\', '/') if inside else absolute


def _seconds(value):
    return '-' if value is None else f"{value:.2f}"


def print_tally(s, indent='  '):
    if s['events']:
        print(f"{indent}채터 감지율: {s['detected']}/{s['events']} ({s['detection_rate']:.0%})")
    else:
        print(f"{indent}채터 라벨이 없어 감지율을 계산하지 못했습니다.")
    if s['latency_median_s'] is not None:
        print(f"{indent}감지 지연: 중앙값 {s['latency_median_s']:.2f}초, 90% {s['latency_p90_s']:.2f}초, "
              f"최대 {s['latency_max_s']:.2f}초")
    if s['false_alarms_per_hour'] is not None:
        print(f"{indent}오경보: {s['false_alarms']}회 / 안정 가공 {s['stable_minutes']:.1f}분 "
              f"({s['false_alarms_per_hour']:.1f}회/시간)")


def print_dataset_summary(result):
    source = '라벨 구간 시각' if result['events_source'] == 'labels' else '채터 윈도우로 추정한 구간'
    print(f"\n결과 (채터 사례 기준: {source})")
    print_tally(result['overall'])
    for key, title in (('by_trip', '경보 경로'), ('by_cause', '경보 원인(가장 큰 증거 항목)')):
        if result.get(key):
            print(f"  {title}: " + ", ".join(f"{name} 감지 {v['detections']}회·오경보 {v['false_alarms']}회"
                                           for name, v in result[key].items()))
    if len(result['by_type']) > 1:
        print("\n실행 종류별 (사례·감지·지연 중앙값/최대·오경보·안정 가공 분)")
        for name, s in result['by_type'].items():
            print(f"  {name:<14} 실행 {s['runs']:>2}  사례 {s['events']:>2}  감지 {s['detected']:>2}  "
                  f"지연 {_seconds(s['latency_median_s']):>5}/{_seconds(s['latency_max_s']):>5}초  "
                  f"오경보 {s['false_alarms']:>2}  안정 {s['stable_minutes']:>5.1f}분")


def _cell(value):
    if value is None:
        return '-'
    if isinstance(value, float):
        return f"{value:.2f}"
    return str(value)


def compare(result, baseline_path):
    """저장해 둔 결과와 주요 지표를 나란히 출력합니다 (다른 값에는 * 표시)."""
    with open(baseline_path, encoding='utf-8') as f:
        base = json.load(f)
    print(f"\n기준과 비교 (기준: {baseline_path})")
    if base.get('mode') != result['mode']:
        print(f"  평가 종류가 달라 비교하지 않습니다 (기준 {base.get('mode')}, 현재 {result['mode']}).")
        return
    changed = {k: (base['config'].get(k), v) for k, v in result['config'].items() if base['config'].get(k) != v}
    if changed:
        print("  판정 설정 차이: " + ", ".join(f"{k} {b}→{r}" for k, (b, r) in changed.items()))
    rows = []
    if result['mode'] == 'legacy':
        b, r = base['normal'], result['normal']
        rows += [('정상 파일 오경보(회)', b['false_alarms'], r['false_alarms']),
                 ('오경보(회/시간)', b['false_alarms_per_hour'], r['false_alarms_per_hour']),
                 ('가공 시간(분)', b['cut_minutes'], r['cut_minutes'])]
        for name in sorted(set(base['abnormal']) | set(result['abnormal'])):
            rows.append((f"이상 파일 경보(회) {name}", base['abnormal'].get(name, {}).get('alarms'),
                         result['abnormal'].get(name, {}).get('alarms')))
    else:
        scopes = [('전체', base['overall'], result['overall'])]
        scopes += [(name, base['by_type'].get(name), s) for name, s in result['by_type'].items()]
        for name, b, r in scopes:
            b = b or {}
            rows += [(f"{name} 감지", f"{b.get('detected', '-')}/{b.get('events', '-')}", f"{r['detected']}/{r['events']}"),
                     (f"{name} 지연 중앙값(초)", b.get('latency_median_s'), r['latency_median_s']),
                     (f"{name} 지연 최대(초)", b.get('latency_max_s'), r['latency_max_s']),
                     (f"{name} 오경보(회)", b.get('false_alarms'), r['false_alarms'])]
    width = max(len(name) for name, _, _ in rows) + 2
    print(f"  {'항목':<{width}}{'기준':>10}{'현재':>10}")
    for name, b, r in rows:
        mark = '  *' if _cell(b) != _cell(r) else ''
        print(f"  {name:<{width}}{_cell(b):>10}{_cell(r):>10}{mark}")


def main():
    parser = argparse.ArgumentParser(description="이상 진동 감지기 교차 검증")
    parser.add_argument('--dataset', default=os.path.join(BASE_DIR, 'data', 'dataset'))
    parser.add_argument('--legacy', action='store_true', help="data/학습 데이터(예전 CSV)로 평가")
    parser.add_argument('--set', action='append', metavar='KEY=VALUE', help="판정 설정 변경 (여러 번 사용 가능)")
    parser.add_argument('--json', metavar='PATH', help="결과를 JSON 파일로 저장")
    parser.add_argument('--compare', metavar='PATH', help="저장해 둔 결과(JSON)와 나란히 비교")
    parser.add_argument('--condition-model', metavar='PATH',
                        help="조건 모델(scripts/train_models.py)을 넣어 평가 (평가하는 실행과 다른 데이터로 학습한 것)")
    args = parser.parse_args()
    config = parse_overrides(args.set)
    condition_model = None
    if args.condition_model:
        condition_model = vm.load_condition_model(args.condition_model)
        if condition_model is None:
            sys.exit(f"조건 모델을 읽을 수 없습니다: {args.condition_model} (scripts/train_models.py로 학습한 파일인지 확인하세요)")
        coverage = condition_model.get('coverage')
        print(f"조건 모델: {args.condition_model} (실행 {condition_model['n_runs']}개로 학습"
              + (f", 상한 포함률 {coverage:.1%}" if coverage is not None else "") + ")")
    print(f"판정 설정: {dict(vm.DETECTION_DEFAULTS, **config)}")
    result = evaluate_legacy(config) if args.legacy else evaluate_dataset(args.dataset, config, condition_model)
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=1)
        print(f"\n결과 저장: {args.json}")
    if args.compare:
        compare(result, args.compare)


if __name__ == '__main__':
    main()
