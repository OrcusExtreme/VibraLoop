"""이상 진동(채터) 감지와 대응 (설계 근거: edit_analysis.md)

감지
- 특징은 src/features.py 하나로 계산합니다 (학습·실시간 동일, 스무딩 없음, 윈도우마다 평균 제거).
- 창 설정은 센서 샘플링으로 정합니다 (features.WINDOW_PROFILES): 400 Hz 이상이면 0.2초 창을 0.05초마다,
  기존 센서(약 100 Hz)는 1초 창을 0.25초마다 평가합니다. 감지 모델은 창 설정별로 따로 학습하고(train_detectors),
  센서에 맞는 설정의 모델이 없으면 다른 설정의 모델로 감지하면서 그 사실을 상태에 표시합니다.
- 평가 간격마다 최근 윈도우를 평가해 '증거값'을 만듭니다. 1 이상이면 기준 초과이고,
  아래 항목 중 가장 큰 값입니다.
  · lof: LOF 연속 점수 / 정상 데이터에서 세션 단위로 검증한 분위수 기준
         (학습 데이터 하위 10%보다 조용한 윈도우는 제외: 채터는 진동이 커지는 현상)
  · level: 진동 크기(AC RMS) / 정상 가공 윈도우의 분위수 기준 (절대 상한)
  · self: 기준선 대비 진동 크기 비율 / self_ratio. 기준선 = max(컷 처음 2초 기준선, 롤링 기준선)이고, 롤링 기준선은
          현재 윈도우와 겹치지 않는 직전 baseline_windows개(2초)에서 정상으로 보인 윈도우 RMS의 중앙값입니다.
          경보 중이거나 증거값이 release_level 이상인 윈도우는 넣지 않아 이상이 보이면 멈춥니다.
          경보가 없을 때 회전수·이송이 바뀌면 다시 쌓습니다 (경보 중에는 이상 전 기준선 유지).
  · condition: 조건 모델(models/condition_model.joblib)이 있으면 (회전수, 이송, 깊이)로 예측한 진동 크기 대비 z 점수 / condition_z
  · spectral: 주파수 특징을 쓸 수 있으면 회전 고조파가 아닌 에너지 비율 / nonharmonic_ratio_thr (봉우리가 뚜렷할 때)
    고조파는 접혀(앨리어싱) 보이는 위치까지 빼고, 회전 주파수는 스펙트럼에서 다시 찾습니다 (features.spectral_features)
- 판정: 최근 n개 중 k개 초과(k_of_n, 기본 4개 중 4개 = 1초 연속) 또는 CUSUM. 해제는 release_level 미만이
  release_windows번 연속일 때 (히스테리시스). k, n, release_windows, baseline_windows, cusum_threshold는
  0.25초 간격 기준 값이고, 0.05초 간격에서는 같은 시간이 되도록 늘려 씁니다 (DetectionCore.window_counts).
- 판정 구간: 가공 중이고 GRBL이 Run, 공구가 소재 안(Z ≤ -gate_depth_mm), 스핀들이 돌 때만.
  진입 직후와 조건(회전수·이송·오버라이드) 변경 직후 transient_sec는 제외합니다.
대응
- 먼저 오버라이드로 회전수·이송을 바꾸고(mitigation_enabled), mitigation_wait_sec 안에 해제되지 않으면
- 피드 홀드 → Hold:0 → 스핀들 정지 → 리셋 순서로 위치를 유지한 채 정지하고, 깊이를 나눈 재가공 파일을 만듭니다.
"""
import csv
import json
import logging
import math
import os
import re
import sys
import threading
import time
from collections import deque

import joblib
import numpy as np
from sklearn.neighbors import LocalOutlierFactor
from sklearn.preprocessing import StandardScaler

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import features  # noqa: E402
import rework  # noqa: E402
from optimizer import CuttingConditionOptimizer  # noqa: E402

TRAINING_DATA_DIR = os.path.join(BASE_DIR, "data", "학습 데이터")
MASTER_DATASET_NAME = "master_dataset.csv"
DETECTION_LOG_PATH = os.path.join(BASE_DIR, "data", "Vibration_detection.csv")
DETECTION_LOG_HEADER = ["DetectedAt", "X", "Y", "Z", "SpindleRPM", "FeedRate", "SpindleOverride", "FeedOverride",
                        "Evidence", "ChatterHz", "Action"]
SETTINGS_PATH = os.path.join(BASE_DIR, "data", "detector_settings.json")
CONDITION_MODEL_PATH = os.path.join(BASE_DIR, "models", "condition_model.joblib")

# 기본값은 data/학습 데이터(정상 19개, 이상 1개)를 파일 단위로 하나씩 빼고 검증해 정한 임시값입니다
# (정상 오경보 약 9분에 1회, 이상 파일 경보 4회). 라벨이 있는 새 데이터로 scripts/evaluate_detector.py를 돌려 다시 맞추세요.
DETECTION_DEFAULTS = {
    # 학습 기준: 정상 데이터에서 이 분위수를 넘으면 '기준 초과'
    'lof_quantile': 0.999,
    'level_quantile': 0.999,
    'lof_neighbors': 20,
    'min_training_windows': 30,
    # 판정
    'decision_mode': 'k_of_n',     # 'k_of_n' 또는 'cusum'
    # 아래 개수(k, n, release_windows, baseline_windows)와 cusum_threshold는 0.25초 간격 기준입니다.
    # 다른 간격(고속 0.05초)에서는 첫 윈도우부터 마지막 윈도우까지의 시간이 같도록 늘려 씁니다.
    'k': 4,                        # 최근 n개 윈도우 중 k개 초과 (0.25초 간격이면 4개 = 1초 연속)
    'n': 4,
    'cusum_drift': 0.0,            # CUSUM: 누적값 = max(0, 누적값 + ln(증거값) - drift)
    'cusum_threshold': 1.2,
    'release_level': 0.8,
    'release_windows': 4,
    # 직전 구간(롤링 기준선)과의 비교: 현재 윈도우와 겹치지 않는 직전 baseline_windows개(0.25초 기준 8개 = 2초)
    'baseline_windows': 8,
    'self_ratio': 3.0,
    # 조건 모델·주파수 특징
    'condition_z': 3.0,
    'nonharmonic_ratio_thr': 0.6,
    'chatter_prominence_min': 8.0,
    # 판정 구간
    'gate_depth_mm': 0.05,
    'transient_sec': 0.75,
    # 대응
    'mitigation_enabled': True,
    'mitigation_wait_sec': 4.0,
    'rework_min_passes': 2,
    'status_emit_interval': 0.5,
}
REFERENCE_HOP_SEC = 0.25   # DETECTION_DEFAULTS의 개수 설정을 정한 평가 간격
PROFILE_LABELS = {'legacy': '1초 창·0.25초 간격', 'fast': '0.2초 창·0.05초 간격'}
MAX_EVALS_PER_BATCH = 4    # 한 번에 받은 샘플로 평가할 윈도우 수 상한 (더 밀리면 최신 윈도우로 건너뜀)

# 화면(모델 관리 창)에서 바꿀 수 있고 data/detector_settings.json에 저장되는 설정
RUNTIME_SETTINGS = {
    'mitigation_enabled': lambda v: bool(v),
    'decision_mode': lambda v: v if v in ('k_of_n', 'cusum') else None,
}

logger = logging.getLogger('vibration')


def load_settings(path=SETTINGS_PATH):
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    settings = {}
    for key, parse in RUNTIME_SETTINGS.items():
        if key in data and parse(data[key]) is not None:
            settings[key] = parse(data[key])
    return settings


def save_settings(settings, path=SETTINGS_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({key: settings[key] for key in RUNTIME_SETTINGS if key in settings}, f, ensure_ascii=False, indent=2)


def load_condition_model(path=CONDITION_MODEL_PATH):
    """scripts/train_models.py가 만든 조건 모델 (없으면 None)."""
    if not os.path.exists(path):
        return None
    try:
        bundle = joblib.load(path)
        if not isinstance(bundle, dict) or 'model' not in bundle:
            raise ValueError("형식이 맞지 않습니다")
        return bundle
    except Exception as e:
        logger.warning(f"조건 모델을 읽을 수 없습니다 ({path}): {e}")
        return None


def append_detection_record(position, spindle, feedrate, spindle_override, feed_override, evidence=None,
                            chatter_freq=None, action='', path=DETECTION_LOG_PATH):
    """이상 진동 감지 기록을 CSV에 한 줄 추가합니다 (없거나 이전 형식이면 헤더부터 새로 작성).

    position: 감지 시점의 (X, Y, Z), spindle/feedrate: 오버라이드를 반영한 실제 절삭 조건 (모르면 None)
    action: 'mitigate'(오버라이드 대응) 또는 'stop'(정지 후 재가공)
    """
    if os.path.exists(path) and os.path.getsize(path) > 0:
        with open(path, encoding='utf-8-sig', errors='ignore') as f:
            first_line = f.readline().strip()
        if first_line != ",".join(DETECTION_LOG_HEADER):
            # 예전 형식 파일은 덮어쓰지 않고 보관
            os.replace(path, path + ".old")
    write_header = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(DETECTION_LOG_HEADER)
        writer.writerow([
            time.strftime('%Y-%m-%d %H:%M:%S'),
            f"{position[0]:.3f}", f"{position[1]:.3f}", f"{position[2]:.3f}",
            "" if spindle is None else f"{spindle:.0f}",
            "" if feedrate is None else f"{feedrate:.1f}",
            spindle_override,
            feed_override,
            "" if evidence is None else f"{evidence:.2f}",
            "" if chatter_freq is None else f"{chatter_freq:.0f}",
            action,
        ])


def _as_sessions(sensor_data, default_fs=100.0):
    """학습 입력을 세션 목록 [[(acc, t), ...], ...]으로 맞춥니다.

    sensor_data: features.load_sensor_csv() 결과(dict)들, (acc, t) 튜플들, 또는 (N, 3) 배열들(100 Hz로 가정)
    """
    sessions = []
    for item in sensor_data:
        if isinstance(item, dict):
            sessions.append(list(item['segments']))
        elif isinstance(item, tuple) and len(item) == 2:
            sessions.append([(np.asarray(item[0], dtype=float)[:, :3], np.asarray(item[1], dtype=float))])
        else:
            acc = np.asarray(item, dtype=float)[:, :3]
            sessions.append([(acc, np.arange(len(acc)) / default_fs)])
    return sessions


def _session_fs(session):
    """세션(연속 구간 목록)의 샘플링 (구간별 추정값의 중앙값, 모르면 None)"""
    values = [features.estimate_fs(t) for _, t in session]
    values = [fs for fs in values if fs]
    return float(np.median(values)) if values else None


def train_detectors(sensor_data, config=None):
    """정상 가공 센서 데이터로 창 설정별 감지 모델을 학습합니다 → {'legacy': 모델, 'fast': 모델}.

    파일(세션)마다 샘플링으로 창 설정을 정하고(features.window_profile), 같은 설정의 세션끼리 학습합니다.
    윈도우가 부족한 설정은 건너뛰고, 학습한 모델이 하나도 없으면 ValueError.
    """
    config = dict(DETECTION_DEFAULTS, **(config or {}))
    base = features.feature_config(config)
    grouped = {}
    for session in _as_sessions(sensor_data):
        profile = features.window_profile(_session_fs(session), base)
        feat_config = features.profile_config(base, profile)
        rows = []
        for acc, t in session:
            rows.extend(f for _, f in features.session_windows(acc, t, feat_config))
        if rows:
            grouped.setdefault(profile, []).append(rows)
    models, errors = {}, []
    for profile, sessions in grouped.items():
        try:
            models[profile] = fit_detector(sessions, config, profile)
        except ValueError as e:
            errors.append(f"{PROFILE_LABELS[profile]}: {e}")
    if not models:
        raise ValueError("; ".join(errors) or "학습할 가공 구간이 없습니다.")
    return models


def train_detector(sensor_data, config=None):
    """감지 모델 하나를 학습합니다 (창 설정이 섞인 데이터면 저속 'legacy' 모델). 윈도우가 부족하면 ValueError."""
    models = train_detectors(sensor_data, config)
    return models.get('legacy') or next(iter(models.values()))


def fit_detector(sessions, config=None, profile='legacy'):
    """세션별 윈도우 특징 목록(sessions: [[특징 dict, ...], ...])으로 감지 모델을 학습합니다.

    LOF 기준은 세션(파일·실행)을 하나씩 빼고 학습한 모델로 그 세션을 채점한 점수의 분위수입니다.
    같은 세션의 겹치는 윈도우로 기준을 정하면 실제보다 낙관적인 기준이 되기 때문입니다.
    profile: 윈도우를 만든 창 설정. 모델에 그 설정(feature_config)을 저장해 실시간도 같은 창으로 평가합니다.
    scripts/evaluate_detector.py는 데이터셋의 윈도우 특징으로 이 함수를 직접 부릅니다.
    """
    config = dict(DETECTION_DEFAULTS, **(config or {}))
    feat_config = features.profile_config(features.feature_config(config), profile)
    per_session = [(np.array([features.feature_vector(f) for f in rows]), np.array([f['rms'] for f in rows]))
                   for rows in sessions if rows]
    fs_values = [f['fs'] for rows in sessions for f in rows if f.get('fs')]
    total = sum(len(rms) for _, rms in per_session)
    if total < config['min_training_windows']:
        raise ValueError(f"학습할 가공 구간이 부족합니다 (윈도우 {total}개, 최소 {config['min_training_windows']}개).")

    def fit(X):
        scaler = StandardScaler().fit(X)
        lof = LocalOutlierFactor(n_neighbors=min(config['lof_neighbors'], len(X) - 1), novelty=True)
        lof.fit(scaler.transform(X))
        return scaler, lof

    X_all = np.vstack([X for X, _ in per_session])
    scaler, lof = fit(X_all)
    held_out = []
    if len(per_session) >= 3:
        for i, (X_i, _) in enumerate(per_session):
            X_rest = np.vstack([X for j, (X, _) in enumerate(per_session) if j != i])
            if len(X_rest) < config['min_training_windows']:
                continue
            sc, lo = fit(X_rest)
            held_out.append(-lo.score_samples(sc.transform(X_i)))
    if held_out:
        lof_threshold = float(np.quantile(np.concatenate(held_out), config['lof_quantile']))
        calibration = 'session_holdout'
    else:
        lof_threshold = float(np.quantile(-lof.negative_outlier_factor_, config['lof_quantile']) * 1.2)
        calibration = 'in_sample'
    rms_all = np.concatenate([rms for _, rms in per_session])
    return {
        'version': features.FEATURE_VERSION,
        'profile': profile,
        'feature_names': list(features.TIME_FEATURE_NAMES),
        'feature_config': feat_config,
        'scaler': scaler,
        'lof': lof,
        'lof_threshold': max(lof_threshold, 1.0),
        'lof_calibration': calibration,
        'rms_threshold': float(np.quantile(rms_all, config['level_quantile'])),
        'rms_median': float(np.median(rms_all)),
        'rms_low': float(np.quantile(rms_all, 0.1)),
        'sessions': len(per_session),
        'windows': int(total),
        'fs_median': float(np.median(fs_values)) if fs_values else None,
    }


class DetectionCore:
    """학습된 모델로 윈도우를 채점하고 판정 상태를 관리합니다 (실시간과 오프라인 평가가 함께 사용)."""

    def __init__(self, model, config, condition_model=None):
        self.model = model
        self.config = config
        self.condition_model = condition_model
        self.new_cut()

    def window_counts(self):
        """개수 설정(0.25초 간격 기준)을 이 모델의 평가 간격에 맞춥니다.

        k, n, release_windows는 첫 윈도우부터 마지막 윈도우까지의 시간이 같도록 (개수-1)×배율+1로,
        baseline_windows는 같은 시간을 덮도록 개수×배율로 늘립니다. CUSUM은 윈도우마다 더하므로 기준을 배율만큼 키웁니다.
        """
        feature_config = self.model.get('feature_config') or {}
        hop = feature_config.get('hop_sec') or REFERENCE_HOP_SEC
        window = feature_config.get('window_sec') or features.DEFAULT_FEATURE_CONFIG['window_sec']
        scale = REFERENCE_HOP_SEC / hop
        cfg = self.config

        def stretch(count):
            return int(round((int(count) - 1) * scale)) + 1

        return {'k': stretch(cfg['k']), 'n': stretch(cfg['n']), 'release_windows': stretch(cfg['release_windows']),
                'baseline_windows': max(1, int(round(cfg['baseline_windows'] * scale))),
                # 현재 윈도우와 겹치는 직전 윈도우 수 + 1: 이만큼 앞의 윈도우부터 기준선에 씁니다
                'baseline_gap': max(1, int(round(window / hop))),
                'cusum_threshold': cfg['cusum_threshold'] * scale, 'hop_sec': hop}

    def new_cut(self):
        """새 컷(판정 구간 시작)마다 기준 구간과 판정 상태를 초기화합니다."""
        self.counts = self.window_counts()
        self.recent = deque(maxlen=self.counts['n'])
        self.cusum = 0.0
        self.alarm = False
        self.release_count = 0
        self.reset_baseline()

    def reset_baseline(self):
        """기준선을 비우고 다시 쌓습니다 (새 컷, 또는 경보가 없을 때 회전수·이송이 바뀐 뒤)."""
        self.window_index = 0
        self.baseline_pool = deque()  # (윈도우 번호, RMS): 정상으로 보인 윈도우만
        self.anchor_values = []       # 컷 처음 baseline_windows개의 정상 윈도우 RMS
        self.anchor = None            # 컷 처음 기준선 (그 중앙값, 이후 고정)
        self.rolling = None           # 롤링 기준선
        self.baseline = None          # 실제로 쓰는 기준선 = max(컷 처음, 롤링)

    def score(self, feats, cond=None):
        """윈도우 특징 → (증거값, 항목별 값, 채터 주파수 또는 None)"""
        cond = cond or {}
        model = self.model
        parts = {}
        # LOF는 양쪽(너무 크거나 너무 작은 진동)을 모두 '드묾'으로 보므로, 학습 데이터 하위 10%보다 조용한
        # 윈도우는 제외합니다 (채터는 진동이 커지는 현상이고, 가벼운 절삭에서 오경보가 나지 않도록)
        if feats['rms'] >= model.get('rms_low', 0.0):
            x = features.feature_vector(feats, model['feature_names']).reshape(1, -1)
            parts['lof'] = float(-model['lof'].score_samples(model['scaler'].transform(x))[0] / model['lof_threshold'])
        parts['level'] = feats['rms'] / model['rms_threshold']
        if self.baseline:
            parts['self'] = feats['rms'] / self.baseline / self.config['self_ratio']
        cm = self.condition_model
        if cm and all(cond.get(name) is not None for name in cm['features']):
            X = np.array([[cond[name] for name in cm['features']]], dtype=float)
            if cm.get('scaler') is not None:
                X = cm['scaler'].transform(X)
            expected = float(cm['model'].predict(X)[0])
            z = (math.log(max(feats['rms'], 1e-6)) - expected) / max(cm.get('sigma', 0.3), 1e-3)
            parts['condition'] = z / self.config['condition_z']
        chatter_freq = None
        if feats.get('spectral') and feats.get('nonharmonic_ratio') is not None \
                and feats.get('peak_prominence', 0.0) >= self.config['chatter_prominence_min']:
            parts['spectral'] = feats['nonharmonic_ratio'] / self.config['nonharmonic_ratio_thr']
            if parts['spectral'] >= 1.0:
                chatter_freq = feats.get('peak_freq')
        evidence = max(parts.values())
        return evidence, parts, chatter_freq

    def _refresh_baseline(self):
        """지금 윈도우의 기준선을 정합니다 (edit_analysis2.md P05).

        롤링 기준선 = 이 윈도우와 겹치지 않는 직전 baseline_windows개(2초) 범위에서 정상으로 보인 윈도우 RMS의 중앙값.
        범위 안에 정상 윈도우가 절반보다 적으면(이상이 이어지는 중) 롤링 기준선을 바꾸지 않습니다.
        쓰는 기준선은 컷 처음 기준선과 롤링 기준선 중 큰 값입니다: 정상적으로 커진 진동(맞물림·가공 폭 증가)은 롤링
        기준선이 따라가 오경보를 막고, 가벼운 구간 뒤 정상 가공으로 돌아올 때는 컷 처음 기준선이 3배 급증으로 보지 않게 합니다.
        """
        counts = self.counts
        newest = self.window_index - counts['baseline_gap']
        oldest = newest - counts['baseline_windows'] + 1
        while self.baseline_pool and self.baseline_pool[0][0] < oldest:
            self.baseline_pool.popleft()
        values = [rms for index, rms in self.baseline_pool if index <= newest]
        if len(values) >= max(1, counts['baseline_windows'] // 2):
            self.rolling = float(np.median(values))
        candidates = [value for value in (self.anchor, self.rolling) if value]
        self.baseline = max(candidates) if candidates else None

    def _admit_baseline(self, feats, evidence):
        """정상으로 보인 윈도우만 기준선 후보로 넣습니다. 경보 중이거나 증거값이 release_level 이상이면 넣지 않으므로,
        이상이 보이기 시작하면 기준선이 멈춰 천천히 커지는 채터를 따라가지 않습니다."""
        if not self.alarm and evidence < self.config['release_level']:
            self.baseline_pool.append((self.window_index, feats['rms']))
            if self.anchor is None:
                self.anchor_values.append(feats['rms'])
                if len(self.anchor_values) >= self.counts['baseline_windows']:
                    self.anchor = float(np.median(self.anchor_values))

    def decide(self, evidence):
        """판정 상태 갱신 → 'start'(경보 시작), 'end'(해제), None"""
        cfg, counts = self.config, self.counts
        if cfg['decision_mode'] == 'cusum':
            self.cusum = max(0.0, self.cusum + math.log(max(evidence, 1e-6)) - cfg['cusum_drift'])
            triggered = self.cusum >= counts['cusum_threshold']
        else:
            self.recent.append(evidence >= 1.0)
            triggered = sum(self.recent) >= counts['k']
        if not self.alarm:
            if triggered:
                self.alarm = True
                self.release_count = 0
                return 'start'
            return None
        self.release_count = self.release_count + 1 if evidence < cfg['release_level'] else 0
        if self.release_count >= counts['release_windows']:
            self.alarm = False
            self.recent.clear()
            self.cusum = 0.0
            return 'end'
        return None

    def step(self, feats, cond=None):
        """판정 구간 안의 윈도우 하나를 처리합니다."""
        self._refresh_baseline()
        evidence, parts, chatter_freq = self.score(feats, cond)
        self._admit_baseline(feats, evidence)
        decision = self.decide(evidence)
        self.window_index += 1
        return {'evidence': evidence, 'parts': parts, 'chatter_freq': chatter_freq, 'fs': feats.get('fs'),
                'baseline': self.baseline, 'decision': decision, 'alarm': self.alarm}

    def condition_changed(self):
        """판정 구간 안에서 회전수·이송(오버라이드)이 바뀌었을 때. 경보가 없으면 기준선을 새 조건으로 다시 쌓고,
        경보 중(온라인 대응 중)이면 이상 전 기준선을 그대로 둬서 대응 뒤에도 같은 기준으로 진정 여부를 봅니다."""
        if not self.alarm:
            self.reset_baseline()


class VibrationMonitor:
    def __init__(self, socketio, controllers, gcode_lines=None, tolerance_value=0.0, initial_spindle_override=100):
        self.socketio = socketio
        self.controllers = controllers
        self.config = dict(DETECTION_DEFAULTS, **load_settings())
        self.feature_config = features.profile_config(features.feature_config(self.config), 'legacy')
        self.optimizer = CuttingConditionOptimizer()
        self.condition_model = load_condition_model()
        self.clock = time.time

        self.models = {}            # 창 설정별 감지 모델 {'legacy': ..., 'fast': ...}
        self.cores = {}
        self.model = None           # 지금 쓰는 모델 (센서 창 설정에 맞는 것, 없으면 다른 설정)
        self.core = None
        self.profile = None         # 지금 쓰는 모델의 창 설정
        self.stream_profile = None  # 센서 샘플링으로 정한 창 설정 (모르면 None)
        self.fallback = False       # 센서 창 설정의 모델이 없어 다른 설정의 모델을 쓰는 중
        self.range_g = None         # 센서 측정 범위 (헤더)
        self.is_trained = False
        self.training_files = []    # 현재 모델 학습에 사용한 파일 이름
        self.training_samples = 0   # 현재 모델 학습에 사용한 샘플 수
        self.training_windows = 0   # 현재 모델 학습에 사용한 가공 윈도우 수
        self._model_lock = threading.Lock()     # 학습 결과 교체와 실시간 평가가 섞이지 않도록 보호
        self._training_lock = threading.Lock()  # 모델 관리 창의 학습/초기화 요청이 겹치지 않도록 보호
        self._stream_lock = threading.Lock()
        self.sensor_info = {}
        self.spindle_override = initial_spindle_override
        self.is_running = False
        self.reset_stream()

        self.initialize_analyzer()

    # ------------------------------------------------------------------ 학습
    def train_model(self, sensor_data):
        """정상 가공 데이터로 창 설정별 모델을 학습합니다. 실패하면 기존 모델을 그대로 둡니다. 성공 여부를 반환합니다."""
        try:
            models = train_detectors(sensor_data, self.config)
        except Exception as e:
            logger.error(f"모델 학습 중 오류 발생: {e}")
            return False
        with self._model_lock:
            self.models = models
            self.cores = {profile: DetectionCore(model, self.config, self.condition_model)
                          for profile, model in models.items()}
            self.is_trained = True
            self.training_windows = sum(model['windows'] for model in models.values())
            self._select_model()
        for profile, model in models.items():
            logger.info(f"모델 학습 완료 ({PROFILE_LABELS[profile]}: 세션 {model['sessions']}개, 가공 윈도우 {model['windows']}개, "
                        f"LOF 기준 {model['lof_threshold']:.2f}, 진동 크기 기준 {model['rms_threshold']:.3f} g)")
        return True

    def _select_model(self):
        """센서 창 설정에 맞는 모델을 고릅니다. 그 설정의 모델이 없으면 저속 모델(없으면 있는 모델)을 씁니다.
        실시간 윈도우는 고른 모델의 창 설정으로 만듭니다. _model_lock을 잡은 상태에서 부릅니다."""
        wanted = self.stream_profile or 'legacy'
        if wanted in self.models:
            profile = wanted
        else:
            profile = 'legacy' if 'legacy' in self.models else next(iter(self.models), None)
        self.profile = profile
        self.model = self.models.get(profile)
        self.core = self.cores.get(profile)
        if self.model:
            config = dict(self.model['feature_config'])
        else:
            config = features.profile_config(features.feature_config(self.config), wanted)
        if self.range_g:
            config['accel_range_g'] = self.range_g
        self.feature_config = config
        self.fallback = self.model is not None and profile != wanted

    def _set_stream_profile(self, profile):
        """센서 창 설정을 정하고 모델을 다시 고릅니다. 쓰는 창 설정이 바뀌면 판정 상태와 평가 시각을 처음부터 맞춥니다."""
        changed = profile != self.stream_profile
        self.stream_profile = profile
        with self._model_lock:
            previous = self.profile
            self._select_model()
            switched = previous != self.profile
            if switched and self.core:
                self.core.new_cut()
        if switched:
            with self._stream_lock:
                self._next_eval = None
        if changed and profile and self.is_trained:
            fs = self.sensor_info.get('fs')
            source = f"센서 {fs:.0f} Hz" if fs else "센서 샘플링"
            if self.fallback:
                self._log(f"{source}에 맞는 {PROFILE_LABELS[profile]} 감지 모델이 없어 {PROFILE_LABELS[self.profile]} 모델로 감지합니다. "
                          "이 센서로 기록한 정상 가공 데이터를 학습 데이터에 추가하세요.", "orange")
            else:
                logger.info(f"{source} → {PROFILE_LABELS[self.profile]}으로 이상 진동을 판정합니다.")

    def _training_paths(self):
        master_path = os.path.join(TRAINING_DATA_DIR, MASTER_DATASET_NAME)
        if os.path.exists(master_path):
            return [master_path]
        if not os.path.isdir(TRAINING_DATA_DIR):
            return []
        return sorted(os.path.join(TRAINING_DATA_DIR, name) for name in os.listdir(TRAINING_DATA_DIR)
                      if name.lower().endswith('.csv') and '이상' not in name)

    def initialize_analyzer(self):
        """data/학습 데이터의 정상 가공 CSV(파일명에 '이상'이 없는 파일, master_dataset.csv가 있으면 그것만)로 학습합니다."""
        try:
            loaded, names = [], []
            for path in self._training_paths():
                try:
                    data = features.load_sensor_csv(path)
                except KeyError as e:
                    logger.warning(f"경고: {path} 파일에서 필요한 컬럼을 찾을 수 없습니다. 오류: {e}")
                    continue
                except Exception as e:
                    logger.warning(f"경고: {path} 파일을 읽을 수 없습니다. 오류: {e}")
                    continue
                if data['samples'] == 0:
                    continue
                loaded.append(data)
                names.append(os.path.basename(path))
            if not loaded:
                raise FileNotFoundError(f"정상 진동 데이터 파일을 찾을 수 없습니다: {TRAINING_DATA_DIR}")
            logger.info(f"총 {len(loaded)}개 파일, {sum(d['samples'] for d in loaded)}개의 정상 데이터로 학습 시작")
            if not self.train_model(loaded):
                return False
            self.training_files = names
            self.training_samples = sum(d['samples'] for d in loaded)
            logger.info("진동 분석기 초기화 및 모델 학습 완료")
            return True
        except Exception as e:
            logger.error(f"진동 분석기 초기화 실패: {e}")
            return False

    def get_status(self):
        """모델 관리 창에 표시할 학습 상태와 설정을 반환합니다 (모델 관련 값은 지금 쓰는 모델 기준)."""
        model = self.model or {}
        latest = dict(self.latest) if self.latest else {}
        counts = self.core.counts if self.core else {}
        return {
            'is_trained': self.is_trained,
            'is_training': self._training_lock.locked(),
            'training_dir': os.path.relpath(TRAINING_DATA_DIR, BASE_DIR).replace(os.sep, '/'),
            'files': list(self.training_files),
            'samples': self.training_samples,
            'windows': self.training_windows,
            'sessions': model.get('sessions'),
            'training_fs': model.get('fs_median'),
            'thresholds': {'lof': model.get('lof_threshold'), 'rms_g': model.get('rms_threshold'),
                           'calibration': model.get('lof_calibration')},
            'settings': {key: self.config[key] for key in ('mitigation_enabled', 'decision_mode', 'k', 'n',
                                                             'mitigation_wait_sec', 'rework_min_passes')},
            'window': {'profile': self.profile, 'label': PROFILE_LABELS.get(self.profile),
                       'stream_profile': self.stream_profile, 'fallback': self.fallback,
                       'window_sec': self.feature_config.get('window_sec'), 'hop_sec': self.feature_config.get('hop_sec'),
                       'k': counts.get('k'), 'n': counts.get('n')},
            'profiles': {profile: {'label': PROFILE_LABELS[profile], 'sessions': m.get('sessions'),
                                   'windows': m.get('windows'), 'fs': m.get('fs_median')}
                         for profile, m in self.models.items()},
            'condition_model': self.condition_model is not None,
            'optimizer': self.optimizer.describe(),
            'sensor': dict(self.sensor_info),
            'latest': {key: latest.get(key) for key in ('evidence', 'gate', 'alarm', 'stage', 'rms', 'fs', 'spectral')},
        }

    def update_settings(self, updates):
        """화면에서 바꾼 설정을 검증해 적용·저장합니다. 반환: (성공 여부, 메시지)"""
        applied = {}
        for key, value in (updates or {}).items():
            parse = RUNTIME_SETTINGS.get(key)
            if parse is None or parse(value) is None:
                return False, f"바꿀 수 없는 설정입니다: {key}={value}"
            applied[key] = parse(value)
        self.config.update(applied)
        save_settings(self.config)
        with self._model_lock:
            for core in self.cores.values():
                core.config = self.config
                core.new_cut()
        return True, "설정을 저장했습니다."

    def add_training_file(self, filename, content):
        """업로드한 정상 가공 센서 CSV를 학습 데이터 폴더에 저장하고 폴더 전체로 모델을 다시 학습합니다.

        filename: 업로드한 파일 이름, content: 파일 내용(bytes)
        반환값: (성공 여부, 메시지). 학습에 실패하면 저장한 파일을 지우고 기존 모델을 유지합니다.
        """
        name = os.path.basename(str(filename or '').replace('\\', '/')).strip()
        name = re.sub(r'[\x00-\x1f<>:"/\\|?*]', '_', name)
        stem, ext = os.path.splitext(name)
        if ext.lower() != '.csv':
            return False, "CSV 파일(.csv)만 학습에 사용할 수 있습니다."
        if '이상' in name:
            return False, "파일 이름에 '이상'이 들어간 파일은 이상 진동 데이터로 보고 학습에서 제외합니다. 정상 가공 데이터만 올려 주세요."
        if name.lower() == MASTER_DATASET_NAME:
            return False, f"{MASTER_DATASET_NAME}은(는) 폴더의 다른 학습 파일을 모두 대체하는 이름이라 업로드할 수 없습니다. 다른 이름으로 올려 주세요."
        if os.path.exists(os.path.join(TRAINING_DATA_DIR, MASTER_DATASET_NAME)):
            return False, (f"학습 데이터 폴더에 {MASTER_DATASET_NAME}이(가) 있어 그 파일만 학습에 쓰입니다. "
                           "추가 파일로 학습하려면 그 파일을 옮기거나 그 안에 데이터를 합치세요.")

        for encoding in ('utf-8-sig', 'cp949'):
            try:
                text = content.decode(encoding)
                break
            except UnicodeDecodeError:
                continue
        else:
            return False, "파일 인코딩을 읽을 수 없습니다. UTF-8 또는 CP949로 저장한 CSV를 올려 주세요."

        try:
            data = features.load_sensor_csv(text if '\n' in text else text + '\n')
        except KeyError:
            return False, "AccX, AccY, AccZ 열이 있는 센서 데이터 CSV가 필요합니다 (데이터 수집 기록의 *_sensor.csv 형식)."
        except Exception as e:
            return False, f"CSV 파일을 읽을 수 없습니다: {e}"
        if data['samples'] < 100:
            return False, f"유효한 AccX/AccY/AccZ 데이터가 {data['samples']}행뿐입니다. 최소 100행이 필요합니다."

        if not self._training_lock.acquire(blocking=False):
            return False, "이미 모델을 학습하는 중입니다. 끝난 뒤 다시 시도하세요."
        try:
            os.makedirs(TRAINING_DATA_DIR, exist_ok=True)
            if not stem.strip(' .'):
                stem = time.strftime('센서_데이터_%Y-%m-%d_%H-%M-%S')
            path = os.path.join(TRAINING_DATA_DIR, stem + ext)
            counter = 1
            while os.path.exists(path):  # 같은 이름의 기존 학습 파일은 덮어쓰지 않음
                path = os.path.join(TRAINING_DATA_DIR, f"{stem}_{counter}{ext}")
                counter += 1
            try:
                with open(path, 'w', encoding='utf-8', newline='') as f:
                    f.write(text)
            except OSError as e:
                return False, f"학습 데이터 폴더에 파일을 저장하지 못했습니다: {e}"

            saved_name = os.path.basename(path)
            # 학습이 실패하면 train_model이 기존 모델을 그대로 두고, 파일만 건너뛴 경우엔 기존 파일로 다시 학습된 상태
            if not self.initialize_analyzer() or saved_name not in self.training_files:
                try:
                    os.remove(path)
                except OSError:
                    pass
                return False, "업로드한 파일로 모델을 학습하지 못해 파일을 제거했습니다. 기존 학습 데이터로 감지를 계속합니다."
            self.reset_stream()
            logger.info(f"학습 데이터 추가: {saved_name} → 파일 {len(self.training_files)}개로 재학습")
            return True, (f"{saved_name}({data['samples']}행)을(를) 학습 데이터에 추가하고 모델을 다시 학습했습니다. "
                          f"(학습 파일 {len(self.training_files)}개)")
        finally:
            self._training_lock.release()

    def reset_analyzer(self):
        """학습 데이터 폴더의 파일로 모델을 다시 학습하고 감지 상태(버퍼·판정)를 초기화합니다.

        반환값: (성공 여부, 메시지). 학습에 실패하면 기존 모델을 유지합니다.
        """
        if not self._training_lock.acquire(blocking=False):
            return False, "이미 모델을 학습하는 중입니다. 끝난 뒤 다시 시도하세요."
        try:
            if not self.initialize_analyzer():
                return False, "학습 데이터 폴더의 파일로 모델을 학습하지 못했습니다. 기존 모델을 그대로 사용합니다."
            self.reset_stream()
            return True, f"학습 데이터 {len(self.training_files)}개 파일로 감지기를 초기화했습니다."
        finally:
            self._training_lock.release()

    # ------------------------------------------------------------------ 실시간 처리
    def reset_stream(self):
        """버퍼·판정 상태·진행 중인 대응을 비웁니다 (가공 시작, 재학습 후)."""
        with self._stream_lock:
            self._buf_t = deque()
            self._buf_a = deque()
            self._last_t = None
            self._next_eval = None
        self._signature = None
        self._changed_at = 0.0
        self._gate_was_on = False
        self._mitigated_in_cut = False  # 한 컷에서 온라인 대응은 한 번만 (반복해서 조건을 흔들지 않도록)
        self._last_emit = 0.0
        self.event = None
        self.latest = {}
        with self._model_lock:
            if self.core:
                self.core.new_cut()

    def set_sensor_info(self, info):
        """센서 스트림 정보(장치, 샘플링, 측정 범위)를 반영하고, 샘플링에 맞는 창 설정의 모델을 고릅니다.
        샘플링을 모르면(헤더 없는 기존 펌웨어) 받은 샘플 시각으로 추정합니다 (process_sensor_data)."""
        self.sensor_info = dict(info or {})
        self.range_g = float(info['range_g']) if info and info.get('range_g') else None
        fs = self.sensor_info.get('fs')
        self._set_stream_profile(features.window_profile(fs, self.feature_config) if fs else None)

    def process_sensor_data(self, samples, timestamps=None):
        """센서 샘플을 받아 평가 시각마다 윈도우를 평가합니다.

        평가 시각은 첫 윈도우 끝에서 hop_sec씩 늘어나는 격자라서 오프라인(features.window_ranges)과 같습니다.
        samples: [[ax, ay, az], ...] (g), timestamps: 샘플별 측정 시각(초, 장치 시각 권장). 없으면 받은 시각.
        """
        if not self.is_running or not self.is_trained:
            return
        now = self.clock()
        cfg = self.feature_config
        estimate = None
        with self._stream_lock:
            for i, sample in enumerate(samples):
                try:
                    a = (float(sample[0]), float(sample[1]), float(sample[2]))
                except (TypeError, ValueError, IndexError):
                    continue
                if not all(math.isfinite(v) and abs(v) <= cfg['accel_range_g'] * 1.05 for v in a):
                    continue  # 통신 오류로 깨진 값 (진동 자체는 거르지 않음)
                t = float(timestamps[i]) if timestamps is not None else now
                if self._last_t is not None and (t < self._last_t or t - self._last_t > cfg['max_gap_sec']):
                    self._buf_t.clear()
                    self._buf_a.clear()
                    self._next_eval = None
                self._buf_t.append(t)
                self._buf_a.append(a)
                self._last_t = t
            if not self._buf_t:
                return
            if self.stream_profile is None and self._buf_t[-1] - self._buf_t[0] >= 0.5 and len(self._buf_t) >= 10:
                estimate = features.estimate_fs(np.fromiter(self._buf_t, dtype=float, count=len(self._buf_t)))
        if estimate:
            self._set_stream_profile(features.window_profile(estimate, cfg))
            cfg = self.feature_config
        with self._stream_lock:
            if not self._buf_t:
                return
            t_last = self._buf_t[-1]
            while self._buf_t and self._buf_t[0] < t_last - cfg['window_sec'] - 0.5:
                self._buf_t.popleft()
                self._buf_a.popleft()
            if self._next_eval is None:
                self._next_eval = self._buf_t[0] + cfg['window_sec']
            ends = []
            while self._next_eval <= t_last and len(ends) < MAX_EVALS_PER_BATCH:
                ends.append(self._next_eval)
                self._next_eval += cfg['hop_sec']
            if self._next_eval <= t_last:  # 너무 밀렸으면 지난 윈도우는 건너뜀
                self._next_eval = t_last + cfg['hop_sec']
            if not ends:
                return
            t_arr = np.fromiter(self._buf_t, dtype=float, count=len(self._buf_t))
            a_arr = np.array(self._buf_a, dtype=float)
        nominal_fs = features.estimate_fs(t_arr)
        for end in ends:
            selected = (t_arr >= end - cfg['window_sec']) & (t_arr <= end)
            if features.window_is_usable(t_arr[selected], cfg, nominal_fs):
                self._evaluate(a_arr[selected], t_arr[selected], now)

    def _gate(self, now):
        """판정 구간 여부와 현재 절삭 조건. 컷이 새로 시작되면 판정 상태를 초기화합니다."""
        snapshot_fn = getattr(self.controllers, 'machine_snapshot', None)
        snap = snapshot_fn() if callable(snapshot_fn) else None
        if not snap:
            return False, {}
        z = snap.get('z')
        n, F = snap.get('n_eff'), snap.get('F_eff')
        spindle_rt = snap.get('spindle_rt')
        spindle_on = (spindle_rt if spindle_rt is not None else (n or 0)) > 0
        gate = bool(snap.get('state') == 'running' and snap.get('grbl_state') == 'Run' and z is not None
                    and z <= -self.config['gate_depth_mm'] and spindle_on)
        cond = {'n': n, 'F': F, 'ap': -z if z is not None and z < 0 else None, 'flutes': snap.get('flutes'),
                'n_cmd': snap.get('n_cmd'), 'F_cmd': snap.get('F_cmd'),
                'spindle_override': snap.get('spindle_override'), 'feed_override': snap.get('feed_override'),
                'x': snap.get('x'), 'y': snap.get('y'), 'z': z}
        signature = (gate, round(n or 0), round(F or 0, 1))
        if signature != self._signature:
            if gate and not self._gate_was_on:
                self._mitigated_in_cut = False
                with self._model_lock:
                    if self.core:
                        self.core.new_cut()
            elif gate:  # 판정 구간 안에서 회전수·이송(오버라이드)이 바뀜
                with self._model_lock:
                    if self.core:
                        self.core.condition_changed()
            self._signature = signature
            self._changed_at = now
        if not gate and self._gate_was_on:
            self._leave_cut()
        self._gate_was_on = gate
        # 경계(정확히 transient_sec 뒤)에서 부동소수점 오차로 판정이 흔들리지 않도록 아주 작은 여유를 둡니다
        cond['transient'] = now - self._changed_at < self.config['transient_sec'] - 1e-6
        return gate, cond

    def _leave_cut(self):
        """공구가 소재를 벗어나면 판정을 멈추고, 온라인 대응 중이던 사례는 종료합니다."""
        with self._model_lock:
            if self.core:
                self.core.new_cut()
        if self.event and self.event.get('stage') == 'mitigating':
            self._log("이상 진동 대응 중 공구가 소재를 벗어나 판정을 마칩니다.", "blue")
            self.event = None

    def _evaluate(self, acc, t, now):
        gate, cond = self._gate(now)
        feats = features.compute_features(acc, t, self.feature_config, cond.get('n'))
        result = {'time': now, 'gate': gate, 'rms': feats['rms'], 'fs': feats['fs'], 'spectral': feats['spectral'],
                  'evidence': None, 'parts': {}, 'alarm': False, 'chatter_freq': None,
                  'stage': self.event.get('stage') if self.event else None}
        if gate and not cond.get('transient'):
            with self._model_lock:
                core = self.core
                step = core.step(feats, cond) if core else None
            if step:
                result.update(evidence=step['evidence'], parts=step['parts'], alarm=step['alarm'],
                              chatter_freq=step['chatter_freq'])
                if step['decision'] == 'start':
                    self._on_alarm(cond, step, now)
                elif step['decision'] == 'end':
                    self._on_release()
        if self.event and self.event.get('stage') == 'mitigating' and now >= self.event['deadline']:
            if self.core and self.core.alarm:
                self._escalate()
        result['stage'] = self.event.get('stage') if self.event else None
        self.latest = result
        if now - self._last_emit >= self.config['status_emit_interval']:
            self._last_emit = now
            self.socketio.emit('vibration_status', {
                'evidence': None if result['evidence'] is None else round(result['evidence'], 3),
                'gate': gate, 'alarm': result['alarm'], 'stage': result['stage'], 'rms': round(feats['rms'], 4),
                'parts': {k: round(v, 3) for k, v in result['parts'].items()},
            })

    # ------------------------------------------------------------------ 대응
    def _log(self, message, color="orange"):
        logger.info(message)
        self.socketio.emit('log_update', {"message": message, "color": color})

    def _on_alarm(self, cond, step, now):
        mitigate = bool(self.config['mitigation_enabled'] and cond.get('n_cmd') and cond.get('F_cmd')
                        and not self._mitigated_in_cut)
        self.event = {'started': now, 'stage': 'detected', 'cond': dict(cond), 'evidence': step['evidence'],
                      'parts': dict(step['parts']), 'chatter_freq': step['chatter_freq'], 'fs': step.get('fs')}
        main_part = max(step['parts'], key=step['parts'].get)
        self._log(f"⚠️ 이상 진동 감지 (지표 {step['evidence']:.2f}, 주요 원인 {main_part})", "orange")
        self.socketio.emit('vibration_alert', {'show': True, 'evidence': step['evidence'],
                                               'message': f"⚠️ 이상 진동 감지 (지표 {step['evidence']:.2f})"})
        self._record(cond, step['evidence'], step['chatter_freq'], 'mitigate' if mitigate else 'stop')
        if mitigate:
            self._mitigate(now)
        else:
            self._stop_and_rework("이상 진동 감지로 정지합니다 (온라인 대응 꺼짐 또는 프로그램 S/F 없음).")

    def _record(self, cond, evidence, chatter_freq, action):
        position = tuple((cond.get(k) if cond.get(k) is not None else 0.0) for k in ('x', 'y', 'z'))
        try:
            append_detection_record(position, cond.get('n'), cond.get('F'), cond.get('spindle_override'),
                                    cond.get('feed_override'), evidence, chatter_freq, action)
        except Exception as e:
            logger.error(f"Vibration_detection.csv 기록 중 오류: {e}")

    def _mitigate(self, now):
        event = self.event
        cond = event['cond']
        suggestion = self.optimizer.suggest(cond.get('n'), cond.get('F'), cond.get('ap'), event['chatter_freq'],
                                            cond.get('flutes'), purpose='online', sample_rate=event.get('fs'))
        if not suggestion:
            self._stop_and_rework("현재 회전수·이송을 알 수 없어 온라인 대응 없이 정지합니다.")
            return
        spindle_target = int(round(100.0 * suggestion['n'] / cond['n_cmd']))
        feed_target = int(round(100.0 * suggestion['F'] / cond['F_cmd']))
        event.update(stage='mitigating', deadline=now + self.config['mitigation_wait_sec'], suggestion=suggestion)
        self._mitigated_in_cut = True
        self._log(f"온라인 대응: 스핀들 오버라이드 {cond.get('spindle_override')}% → {spindle_target}%, "
                  f"이송 오버라이드 {cond.get('feed_override')}% → {feed_target}% ({suggestion['reason']}). "
                  f"{self.config['mitigation_wait_sec']:g}초 안에 진정되지 않으면 정지합니다.", "orange")
        threading.Thread(target=self._apply_overrides, args=(spindle_target, feed_target), daemon=True).start()

    def _apply_overrides(self, spindle_target, feed_target):
        for target, value in (('spindle', spindle_target), ('feed', feed_target)):
            try:
                result = self.controllers.apply_override(target, value)
            except Exception as e:
                result = {'status': 'error', 'message': str(e)}
            if result.get('status') != 'success':
                self._log(f"온라인 대응 오버라이드 실패({target}): {result.get('message')}", "red")

    def _on_release(self):
        if self.event and self.event.get('stage') == 'mitigating':
            self._log("✅ 온라인 대응 후 진동이 안정되어 바꾼 조건으로 가공을 계속합니다.", "green")
            self.socketio.emit('vibration_alert', {'show': False})
        self.event = None

    def _escalate(self):
        self._stop_and_rework("온라인 대응 후에도 이상 진동이 계속되어 정지합니다.")

    def handle_abnormal_vibration(self):
        """확정된 이상 진동 처리: 위치를 유지한 채 정지하고 재가공을 준비합니다 (판정 없이 바로 정지)."""
        controllers = self.controllers
        if not (controllers.ser and controllers.ser.is_open and controllers.state == "running"):
            return
        _, cond = self._gate(self.clock())
        chatter_freq = self.latest.get('chatter_freq') if self.latest else None
        self.event = {'started': self.clock(), 'stage': 'detected', 'cond': dict(cond), 'evidence': None,
                      'parts': {}, 'chatter_freq': chatter_freq, 'fs': self.latest.get('fs') if self.latest else None}
        self.socketio.emit('vibration_alert', {'show': True, 'message': "⚠️ 이상 진동 감지"})
        self._record(cond, None, chatter_freq, 'stop')
        self._stop_and_rework("이상 진동 감지로 정지합니다.")

    def _stop_and_rework(self, reason):
        """피드 홀드 → Hold:0 → 스핀들 정지 → 리셋으로 정지하고, 별도 스레드에서 재가공 파일을 만듭니다."""
        controllers = self.controllers
        event = self.event or {'cond': {}, 'chatter_freq': None}
        if event.get('stage') == 'stopping':
            return
        event['stage'] = 'stopping'
        self.event = event
        try:
            controllers.safe_stop(reason)
            if controllers.vibration_detection_active:
                controllers.stop_vibration_detection()
            self._log(f"{reason} 위치를 유지한 채 정지한 뒤 재가공을 준비합니다.", "orange")
            self.socketio.emit('status_update', {"text": "이상 진동 감지: 정지", "color": "orange"})
            source_lines = list(controllers.rework_source_lines or controllers.gcode_lines)
        except Exception as e:
            logger.error(f"이상 진동 정지 처리 중 오류: {e}", exc_info=True)
            return
        threading.Thread(target=self._recover_and_create_rework, args=(dict(event), source_lines), daemon=True).start()

    def _recover_and_create_rework(self, event, source_lines):
        """정지 완료 확인 → 조건 제안(최적화기) → 깊이를 나눈 rework.nc 생성 (감시자가 자동 가공)"""
        controllers = self.controllers
        try:
            self.socketio.emit('log_update', {"message": "정지 완료를 기다린 뒤 컨트롤러 상태를 확인합니다...", "color": "blue"})
            if not controllers.recover_after_reset():
                self._log("⚠️ GRBL 컨트롤러 복구 실패: 상태를 확인한 뒤 수동으로 재가공하세요.", "red")
                return
            self.socketio.emit('status_update', {"text": "정지 완료: 재가공 준비 중", "color": "green"})

            cond = event.get('cond') or {}
            suggestion = self.optimizer.suggest(cond.get('n'), cond.get('F'), cond.get('ap'), event.get('chatter_freq'),
                                                cond.get('flutes'), purpose='rework', sample_rate=event.get('fs'))
            if suggestion is None:
                self._log("현재 회전수·이송을 알 수 없어 재가공 파일을 만들지 않습니다.", "red")
                return
            passes = max(int(suggestion['passes']), int(self.config['rework_min_passes']))
            self._log(f"재가공 조건: S={suggestion['n']:.0f} rpm, F={suggestion['F']:.1f} mm/min, 깊이 {passes}번 나눔 "
                      f"({suggestion['reason']})", "green")

            controllers.arm_rework()
            created = rework.create_rework_nc(
                cond.get('n') or suggestion['n'], cond.get('F') or suggestion['F'], suggestion['n'], suggestion['F'],
                cond.get('x') or 0.0, cond.get('y') or 0.0, cond.get('z') or 0.0,
                gcode_lines=source_lines,
                tolerance_value=controllers.tolerance_value,
                depth_passes=passes,
                note=f"rework {suggestion['method']}: S{suggestion['n']:.0f} F{suggestion['F']:.1f} depth passes {passes}",
            )
            if created:
                self._log("rework.nc 파일이 생성되었습니다.", "blue")
            else:
                controllers.disarm_rework()
                self._log("⚠️ rework.nc 파일 생성 실패", "red")
        except Exception as e:
            controllers.disarm_rework()
            logger.error(f"재가공 준비 중 오류: {e}", exc_info=True)
            self._log(f"⚠️ 재가공 준비 실패: {e}", "red")
        finally:
            self.event = None
