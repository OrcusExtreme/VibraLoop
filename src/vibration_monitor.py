"""이상 진동(채터) 감지와 대응 (설계 근거: edit_analysis.md)

감지
- 특징은 src/features.py 하나로 계산합니다 (학습·실시간 동일, 스무딩 없음, 윈도우마다 평균 제거).
- 창 설정은 센서 샘플링으로 정합니다 (features.WINDOW_PROFILES): 400 Hz 이상이면 0.2초 창을 0.05초마다,
  기존 센서(약 100 Hz)는 1초 창을 0.25초마다 평가합니다. 감지 모델은 창 설정별로 따로 학습하고(train_detectors),
  센서에 맞는 설정의 모델이 없으면 다른 설정의 모델로 감지하면서 그 사실을 상태에 표시합니다.
- scripts/train_models.py가 데이터셋으로 학습해 저장한 감지 모델(models/detector_model.joblib, edit_analysis2.md P12)이
  있으면 그 창 설정에는 그 모델을 쓰고, 나머지 창 설정만 학습 데이터 폴더의 CSV로 학습합니다. 데이터셋에는 가공 기록의
  회전수가 있어 고속 창 모델에 비고조파 비율이 들어갑니다(업로드 CSV에는 회전수가 없어 빠짐).
- 평가 간격마다 최근 윈도우를 평가해 '증거값'을 만듭니다. 1 이상이면 기준 초과이고,
  아래 항목 중 가장 큰 값입니다.
  · spe: PCA 잔차 제곱합(SPE) / 정상 데이터에서 세션 단위로 검증한 분위수 기준 (edit_analysis2.md P08)
         표준화한 특징으로 정상 가공의 상관 구조(주성분)를 배우고, 그 구조로 설명되지 않는 크기를 봅니다.
         입력은 진동 크기 특징 9개이고, 고속 창은 학습 데이터에 있으면 주파수 특징(스펙트럼 엔트로피, 비고조파 비율)을 더합니다.
         (학습 데이터 하위 10%보다 조용한 윈도우는 제외: 채터는 진동이 커지는 현상)
  · t2: 주성분 점수의 크기(Hotelling T²) / 같은 방식의 기준. 부하가 크면 함께 커지므로 기본은 경보에 쓰지 않고
        '학습 범위 밖' 표시로만 씁니다 (t2_alarm을 켜면 증거 항목에 포함).
  · level: 진동 크기(AC RMS) / 정상 가공 윈도우의 분위수 기준 (절대 상한)
  · self: 기준선 대비 진동 크기 비율 / self_ratio. 기준선 = max(컷 처음 2초 기준선, 롤링 기준선)이고, 롤링 기준선은
          현재 윈도우와 겹치지 않는 직전 baseline_sec(2초)에서 정상으로 보인 윈도우 RMS의 중앙값입니다.
          경보 중이거나 증거값이 release_level 이상인 윈도우는 넣지 않아 이상이 보이면 멈춥니다.
          경보가 없을 때 회전수·이송이 바뀌면 다시 쌓습니다 (경보 중에는 이상 전 기준선 유지).
  · condition: 진동 크기 / 조건 모델(models/condition_model.joblib, edit_analysis2.md P09)이 정한 이 조건의 정상 상한.
               상한 = 회전수·절입 깊이·칩 단면적(ap·F/(n·Z))으로 예측한 진동 크기 × 학습에서 뺀 실행의 99% 여유.
               날 수를 모르거나 학습한 조건 범위 밖이면 이 항목을 쓰지 않습니다. 이 항목이 있으면 level은 보조로 내립니다.
  · spectral: 주파수 특징을 쓸 수 있으면 회전 고조파가 아닌 에너지 비율 / nonharmonic_ratio_thr (봉우리가 뚜렷할 때)
    고조파는 접혀(앨리어싱) 보이는 위치까지 빼고, 회전 주파수는 스펙트럼에서 다시 찾습니다 (features.spectral_features)
- 판정 (DecisionRule, 시간 설정은 초 단위이고 평가 간격에 맞춰 윈도우 수로 바꿔 씀: DetectionCore.window_counts)
  · k_of_n: 증거값 ≥ 1인 윈도우가 confirm_sec(0.75초) 동안 이어지면 경보 (0.25초 간격이면 4개 = 1초 연속)
  · cusum: ln(증거값)을 시간으로 적분한 값이 cusum_threshold를 넘으면 경보 (큰 이상일수록 빨리)
  · two_tier (고속 센서, edit_analysis2.md P06): 빠른 차단 또는 CUSUM
      빠른 차단: 윈도우 끝 50 ms 블록의 RMS가 기준선의 fast_ratio배 이상인 상태가 fast_sec(블록 3개) 동안 이어짐.
                 블록은 겹치지 않아 짧은 충격 하나로는 발동하지 않고, 진동 크기만 봅니다.
      CUSUM: 증거값을 cusum_cap(2)으로 잘라 적분하므로 큰 충격 하나로 넘지 않고, 1–2 정도의 이상이 이어지면 0.4–1초에 경보.
  해제는 증거값(2단계면 블록 증거값도)이 release_level 미만인 상태가 release_sec 동안 이어질 때 (히스테리시스).
  shadow_two_tier: 고속 센서에서 대응은 decision_mode로 하고 2단계 판정은 기록만 합니다 (실데이터로 검증하기 전 비교용).
- 판정 구간: 가공 중이고 GRBL이 Run, 공구가 소재 안(Z ≤ -gate_depth_mm), 스핀들이 돌 때만.
  진입 직후와 조건(회전수·이송·오버라이드) 변경 직후 transient_sec는 제외합니다.
대응
- 먼저 오버라이드로 회전수·이송을 바꾸고(mitigation_enabled, 이송 먼저, 상태 보고로 반영 확인: cnc_app.apply_overrides),
  mitigation_wait_sec 안에 해제되지 않으면
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

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import features  # noqa: E402
import rework  # noqa: E402
from optimizer import MAX_PASSES, CuttingConditionOptimizer  # noqa: E402

TRAINING_DATA_DIR = os.path.join(BASE_DIR, "data", "학습 데이터")
MASTER_DATASET_NAME = "master_dataset.csv"
DETECTION_LOG_PATH = os.path.join(BASE_DIR, "data", "Vibration_detection.csv")
DETECTION_LOG_HEADER = ["DetectedAt", "X", "Y", "Z", "SpindleRPM", "FeedRate", "SpindleOverride", "FeedOverride",
                        "Evidence", "Cause", "ChatterHz", "Trip", "Action"]
ADDED_LOG_COLUMNS = {'Cause', 'Trip'}  # v1.1 이후 감지 기록에 더한 열 (이 열만 없는 예전 파일은 빈 열을 넣어 이어 씀)
TRIP_LABELS = {'k_of_n': '연속 초과', 'cusum': '누적합(CUSUM)', 'fast': '빠른 차단'}
SETTINGS_PATH = os.path.join(BASE_DIR, "data", "detector_settings.json")
CONDITION_MODEL_PATH = os.path.join(BASE_DIR, "models", "condition_model.joblib")
CONDITION_MODEL_KIND = 'monotone_hgbr'  # 조건 모델 형식 (P09, scripts/train_models.py). 예전 형식은 쓰지 않음
DETECTOR_MODEL_PATH = os.path.join(BASE_DIR, "models", "detector_model.joblib")
DETECTOR_MODEL_KIND = 'pca_spe'  # 데이터셋으로 학습한 감지 모델 묶음 형식 (P12, scripts/train_models.py)
T2_RECENT_SEC = 60.0  # 모델 관리 창의 '학습 범위 밖(T²) 비율'을 세는 최근 판정 시간 (윈도우 수 = 이 시간 / 평가 간격)

# 기본값은 data/학습 데이터(정상 19개, 이상 1개)를 파일 단위로 하나씩 빼고 검증해 정한 임시값입니다
# (정상 오경보 약 9분에 1회, 이상 파일 경보 4회). 라벨이 있는 새 데이터로 scripts/evaluate_detector.py를 돌려 다시 맞추세요.
DETECTION_DEFAULTS = {
    # 학습 기준: 정상 데이터에서 이 분위수를 넘으면 '기준 초과'
    'spe_quantile': 0.999,
    't2_quantile': 0.999,
    'level_quantile': 0.999,
    # PCA 주성분 수: 표준화한 특징 분산의 이 비율을 설명할 때까지. 0.95로는 이송 방향 등으로 축별 진동 비율이 다른
    # 정상 파일을 SPE가 이상으로 봤습니다(기존 데이터 오경보 2회). 0.99면 그런 차이는 주성분 안(T²)으로 들어갑니다.
    'pca_variance': 0.99,
    't2_alarm': False,             # T²(학습 범위 밖)도 증거 항목으로 경보에 쓸지 (기본: 표시만)
    'min_training_windows': 30,
    # 판정 (시간은 초 단위, 평가 간격에 맞춰 윈도우 수로 바꿔 씀)
    'decision_mode': 'k_of_n',     # 대응에 쓰는 판정: 'k_of_n', 'cusum', 'two_tier'(빠른 차단 + CUSUM, 고속 센서)
    'shadow_two_tier': True,       # 고속 센서에서 decision_mode가 two_tier가 아니면 2단계 판정을 기록만 함 (비교용)
    'confirm_sec': 0.75,           # k_of_n: 증거값 ≥ 1인 윈도우가 이 시간 동안 이어지면 경보 (0.25초 간격이면 4개 = 1초 연속)
    'confirm_fraction': 1.0,       # k_of_n: 그 시간 윈도우 중 이 비율 이상이 기준 초과면 (1.0 = 모두)
    'cusum_threshold': 0.3,        # CUSUM: ln(증거값) - drift를 시간(초)으로 적분한 기준 (증거값 2면 약 0.43초, 1.5면 0.74초)
    'cusum_drift': 0.0,
    'cusum_cap': 2.0,              # 2단계의 CUSUM은 증거값을 이 값으로 잘라 적분 (짧은 충격 하나로는 넘지 않게)
    'fast_ratio': 3.0,             # 빠른 차단: 윈도우 끝 50 ms 블록 RMS가 기준선의 이 배수 이상
    'fast_sec': 0.15,              # 빠른 차단: 그 상태가 이 시간(블록 3개) 동안 이어지면 경보
    'release_level': 0.8,
    'release_sec': 0.75,           # 해제: 증거값이 release_level 미만인 상태가 이 시간 동안 이어지면 (0.25초 간격이면 4개)
    # 직전 구간(롤링 기준선)과의 비교: 현재 윈도우와 겹치지 않는 직전 baseline_sec
    'baseline_sec': 2.0,
    'self_ratio': 3.0,
    # 조건 모델·주파수 특징
    'level_with_condition': 1.5,   # 조건 모델로 상한을 정한 윈도우에서는 절대 진동 크기 기준(level)을 이 배수로 올려 보조로 씀
    'nonharmonic_ratio_thr': 0.6,
    'chatter_prominence_min': 8.0,
    # 판정 구간
    'gate_depth_mm': 0.05,
    'transient_sec': 0.75,
    # 대응
    'mitigation_enabled': True,
    'mitigation_wait_sec': 4.0,
    'rework_min_passes': 2,        # 재가공 패스 수 하한 (최적화기가 안정 한계 모델의 패스 수와 비교할 때도 넣음)
    'status_emit_interval': 0.5,
}
PROFILE_LABELS = {'legacy': '1초 창·0.25초 간격', 'fast': '0.2초 창·0.05초 간격'}
MAX_EVALS_PER_BATCH = 4    # 한 번에 받은 샘플로 평가할 윈도우 수 상한 (더 밀리면 최신 윈도우로 건너뜀)
DECISION_MODES = ('k_of_n', 'cusum', 'two_tier')
# 감지 설정 숫자의 허용 범위 [최솟값, 최댓값] (None은 제한 없음, setting_problem). 이 밖의 값은 판정을 깨거나 뒤집습니다:
# 분위수·비율은 1 이하, 배수 기준은 1 이상, cusum_cap이 1 이하면 2단계의 CUSUM이 쌓이지 않고, 해제 수준이 1을 넘으면
# 기준을 넘은 채로 해제됩니다. 패스 수 하한은 최적화기의 최대 패스 수 이하입니다.
SETTING_RANGES = {
    'spe_quantile': (0.5, 1.0), 't2_quantile': (0.5, 1.0), 'level_quantile': (0.5, 1.0), 'pca_variance': (0.5, 1.0),
    'min_training_windows': (1, None), 'confirm_sec': (0.0, None), 'confirm_fraction': (0.01, 1.0),
    'cusum_threshold': (0.01, None), 'cusum_drift': (0.0, None), 'cusum_cap': (1.01, None), 'fast_ratio': (1.0, None),
    'fast_sec': (0.01, None), 'release_level': (0.01, 1.0), 'release_sec': (0.0, None), 'baseline_sec': (0.05, None),
    'self_ratio': (1.0, None), 'level_with_condition': (1.0, None), 'nonharmonic_ratio_thr': (0.01, 1.0),
    'chatter_prominence_min': (0.0, None), 'gate_depth_mm': (0.0, None), 'transient_sec': (0.0, None),
    'mitigation_wait_sec': (0.1, None), 'rework_min_passes': (1, MAX_PASSES), 'status_emit_interval': (0.05, None),
}

# 화면(모델 관리 창)에서 바꿀 수 있고 data/detector_settings.json에 저장되는 설정
RUNTIME_SETTINGS = ('mitigation_enabled', 'decision_mode', 'shadow_two_tier', 't2_alarm')

logger = logging.getLogger('vibration')


def setting_problem(key, value):
    """감지 설정 값 하나를 검사합니다 (설정 검증, edit_analysis2.md P12) → 문제 설명, 쓸 수 있는 값이면 None.
    형식은 기본값(DETECTION_DEFAULTS)과 같아야 하고(참·거짓, 정수, 숫자), 숫자는 SETTING_RANGES 안이어야 합니다."""
    if key not in DETECTION_DEFAULTS:
        return f"{key}: 알 수 없는 설정"
    default = DETECTION_DEFAULTS[key]
    if key == 'decision_mode':
        return None if value in DECISION_MODES else f"{key}={value!r}: {', '.join(DECISION_MODES)} 중 하나여야 함"
    if isinstance(default, bool):
        return None if isinstance(value, bool) else f"{key}={value!r}: true 또는 false여야 함"
    if isinstance(value, bool) or not isinstance(value, (int, float)) or (isinstance(default, int) and not isinstance(value, int)):
        return f"{key}={value!r}: {'정수' if isinstance(default, int) else '숫자'}여야 함"
    low, high = SETTING_RANGES.get(key, (None, None))
    if not math.isfinite(value) or (low is not None and value < low) or (high is not None and value > high):
        return f"{key}={value!r}: 허용 범위 {'-∞' if low is None else low}–{'∞' if high is None else high} 밖"
    return None


def load_settings(path=SETTINGS_PATH):
    """저장된 감지 설정 → (쓸 설정, 쓰지 않은 항목의 설명 목록). 화면에서 바꾸는 설정(RUNTIME_SETTINGS) 중 검사를 통과한
    값만 쓰고, 나머지(파일을 직접 고친 값 등)는 기본값을 씁니다. 파일이 없으면 ({}, [])."""
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
    except FileNotFoundError:
        return {}, []
    except (OSError, ValueError) as e:
        return {}, [f"{os.path.basename(path)}을(를) 읽을 수 없음 ({e})"]
    if not isinstance(data, dict):
        return {}, [f"{os.path.basename(path)}의 형식이 맞지 않음"]
    settings, problems = {}, []
    for key, value in data.items():
        problem = setting_problem(key, value) if key in RUNTIME_SETTINGS else f"{key}: 화면에서 바꾸는 설정이 아님"
        if problem:
            problems.append(problem)
        else:
            settings[key] = value
    return settings, problems


def save_settings(settings, path=SETTINGS_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({key: settings[key] for key in RUNTIME_SETTINGS if key in settings}, f, ensure_ascii=False, indent=2)


def load_condition_model(path=CONDITION_MODEL_PATH):
    """scripts/train_models.py가 만든 조건 모델 (없거나 P09 이전 형식이면 None)."""
    if not os.path.exists(path):
        return None
    try:
        bundle = joblib.load(path)
        if not isinstance(bundle, dict) or 'model' not in bundle:
            raise ValueError("형식이 맞지 않습니다")
        if bundle.get('kind') != CONDITION_MODEL_KIND:
            logger.warning(f"조건 모델({path})이 예전 형식이라 쓰지 않습니다. scripts/train_models.py로 다시 학습하세요.")
            return None
        return bundle
    except Exception as e:
        logger.warning(f"조건 모델을 읽을 수 없습니다 ({path}): {e}")
        return None


def load_detector_models(path=DETECTOR_MODEL_PATH):
    """scripts/train_models.py가 데이터셋으로 학습해 저장한 감지 모델 묶음 (edit_analysis2.md P12). 없거나 쓸 수 없으면 None.
    묶음의 'models'는 창 설정별 fit_detector 결과입니다. 특징 계산 방식(features.FEATURE_VERSION)이 바뀐 뒤에는 같은
    윈도우도 다른 값으로 채점하므로 쓰지 않습니다."""
    if not os.path.exists(path):
        return None
    try:
        bundle = joblib.load(path)
        if not isinstance(bundle, dict) or bundle.get('kind') != DETECTOR_MODEL_KIND:
            raise ValueError("형식이 맞지 않습니다")
        models = {profile: model for profile, model in (bundle.get('models') or {}).items()
                  if profile in PROFILE_LABELS and isinstance(model, dict) and 'pca' in model and 'feature_names' in model}
        if not models:
            raise ValueError("창 설정별 모델이 없습니다")
    except Exception as e:
        logger.warning(f"감지 모델을 읽을 수 없습니다 ({path}): {e}")
        return None
    if bundle.get('feature_version') != features.FEATURE_VERSION:
        logger.warning(f"감지 모델({path})은 특징 계산 방식이 지금과 달라 쓰지 않습니다. "
                       "scripts/build_dataset.py → scripts/train_models.py로 다시 학습하세요.")
        return None
    return dict(bundle, models=models)


def _upgrade_detection_log(path, header):
    """뒤에 더한 열(ADDED_LOG_COLUMNS)이 없던 형식(v1.1)의 감지 기록을 지금 형식으로 바꿉니다.
    열 이름으로 옮기고 없던 열은 빈칸으로 둡니다 (기록은 그대로 유지)."""
    with open(path, encoding='utf-8-sig', newline='') as f:
        rows = list(csv.reader(f))
    column = {name: i for i, name in enumerate(header)}
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(DETECTION_LOG_HEADER)
        writer.writerows([row[column[name]] if name in column and column[name] < len(row) else ''
                          for name in DETECTION_LOG_HEADER] for row in rows[1:] if row)


def append_detection_record(position, spindle, feedrate, spindle_override, feed_override, evidence=None,
                            chatter_freq=None, action='', path=DETECTION_LOG_PATH, trip='', cause=''):
    """이상 진동 감지 기록을 CSV에 한 줄 추가합니다 (없으면 헤더부터 새로 작성).

    Cause·Trip 열이 없던 예전 형식(v1.1)은 빈 열을 넣어 그대로 이어 쓰고, 그 밖의 예전 형식은 .old로 보관합니다.
    position: 감지 시점의 (X, Y, Z), spindle/feedrate: 오버라이드를 반영한 실제 절삭 조건 (모르면 None)
    cause: 가장 큰 증거 항목과, SPE면 기여도가 큰 특징 (describe_cause, 수동 정지면 빈칸)
    trip: 경보를 낸 판정 경로 ('k_of_n', 'cusum', 'fast', 수동 정지면 빈칸)
    action: 'mitigate'(오버라이드 대응), 'stop'(정지 후 재가공), 'shadow'(기록 전용 2단계 판정, 대응 없음)
    """
    if os.path.exists(path) and os.path.getsize(path) > 0:
        with open(path, encoding='utf-8-sig', errors='ignore') as f:
            header = next(csv.reader([f.readline().strip()]), [])
        if header != DETECTION_LOG_HEADER:
            missing = set(DETECTION_LOG_HEADER) - set(header)
            if header and set(header) < set(DETECTION_LOG_HEADER) and missing <= ADDED_LOG_COLUMNS:
                _upgrade_detection_log(path, header)
            else:
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
            cause or "",
            "" if chatter_freq is None else f"{chatter_freq:.0f}",
            trip or "",
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


def detector_feature_names(rows):
    """감지 모델 입력 특징: 진동 크기 특징 9개 + 학습 윈도우의 95% 이상에서 계산된 주파수 특징.
    주파수 특징은 고속 창(1 kHz, 장치 시각)에서만 계산되므로 저속 창 모델은 9개입니다."""
    names = list(features.TIME_FEATURE_NAMES)
    for name in features.SPECTRAL_FEATURE_NAMES:
        if rows and sum(f.get(name) is not None for f in rows) >= 0.95 * len(rows):
            names.append(name)
    return names


def _fit_pca(X, variance):
    """표준화 → PCA. 주성분은 설명 분산 비율이 variance 이상이 되는 최소 개수이고, SPE가 한 방향의 조합만 보지 않도록
    잔차가 두 방향 이상 남게 특징 수 - 2개 이하입니다."""
    mean = X.mean(axis=0)
    scale = X.std(axis=0)
    scale = np.where(scale > 1e-9, scale, 1.0)
    eigenvalues, vectors = np.linalg.eigh(np.cov((X - mean) / scale, rowvar=False))
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues, vectors = np.maximum(eigenvalues[order], 1e-9), vectors[:, order]
    explained = np.cumsum(eigenvalues) / eigenvalues.sum()
    k = max(1, min(int(np.searchsorted(explained, variance)) + 1, X.shape[1] - 2))
    return {'mean': mean, 'scale': scale, 'loadings': vectors[:, :k], 'eigenvalues': eigenvalues[:k],
            'explained': float(explained[k - 1])}


def pca_scores(pca, X):
    """특징 벡터(행마다 윈도우 하나) → (SPE, T², 특징별 SPE 기여도).

    표준화한 특징을 주성분으로 근사했을 때 SPE는 남는 잔차의 제곱합(상관 구조에서 벗어난 정도), T²는 주성분 점수를
    주성분 분산으로 나눈 제곱합(학습 범위의 중심에서 떨어진 정도)이고, 특징별 기여도는 그 특징의 잔차²입니다.
    없는 특징(NaN)은 잔차가 가장 작아지는 값으로 채우므로, SPE는 있는 특징만으로 계산한 값이 됩니다.
    """
    Z = (np.atleast_2d(np.asarray(X, dtype=float)) - pca['mean']) / pca['scale']
    P = pca['loadings']
    missing = np.isnan(Z)
    if missing.any():
        residual_space = np.eye(len(P)) - P @ P.T
        Z = Z.copy()
        for i in np.flatnonzero(missing.any(axis=1)):
            m = missing[i]
            Z[i, m] = -np.linalg.pinv(residual_space[np.ix_(m, m)]) @ residual_space[np.ix_(m, ~m)] @ Z[i, ~m]
    scores = Z @ P
    contributions = (Z - scores @ P.T) ** 2
    return contributions.sum(axis=1), (scores ** 2 / pca['eigenvalues']).sum(axis=1), contributions


def top_contributions(names, contributions, count=2):
    """SPE 기여도가 큰 특징 [(이름, SPE 중 비율), ...]"""
    total = float(np.sum(contributions))
    if total <= 0:
        return []
    return [(names[i], round(float(contributions[i]) / total, 3)) for i in np.argsort(contributions)[::-1][:count]]


def describe_cause(parts, contributions=None):
    """로그·감지 기록에 남길 주요 원인: 가장 큰 증거 항목, SPE면 기여도가 큰 특징까지 (예: 'spe: kurt_z 46%, rms_z 21%')"""
    if not parts:
        return ''
    main = max(parts, key=parts.get)
    if main == 'spe' and contributions:
        return "spe: " + ", ".join(f"{name} {share:.0%}" for name, share in contributions)
    return main


def fit_detector(sessions, config=None, profile='legacy'):
    """세션별 윈도우 특징 목록(sessions: [[특징 dict, ...], ...])으로 감지 모델(표준화 → PCA → SPE·T² 기준)을 학습합니다.

    SPE·T² 기준은 세션(파일·실행)을 하나씩 빼고 학습한 PCA로 그 세션을 채점한 값의 분위수입니다.
    같은 세션의 겹치는 윈도우로 기준을 정하면 실제보다 낙관적인 기준이 되기 때문입니다.
    채점하지 않는 조용한 윈도우(학습 데이터 진동 크기 하위 10% 미만)는 기준 계산에서도 뺍니다.
    profile: 윈도우를 만든 창 설정. 모델에 그 설정(feature_config)을 저장해 실시간도 같은 창으로 평가합니다.
    scripts/evaluate_detector.py는 데이터셋의 윈도우 특징으로 이 함수를 직접 부릅니다.
    """
    config = dict(DETECTION_DEFAULTS, **(config or {}))
    feat_config = features.profile_config(features.feature_config(config), profile)
    sessions = [rows for rows in sessions if rows]
    names = detector_feature_names([f for rows in sessions for f in rows])
    per_session = [(np.array([features.feature_vector(f, names) for f in rows]), np.array([f['rms'] for f in rows]))
                   for rows in sessions]
    fs_values = [f['fs'] for rows in sessions for f in rows if f.get('fs')]
    total = sum(len(rms) for _, rms in per_session)
    if total < config['min_training_windows']:
        raise ValueError(f"학습할 가공 구간이 부족합니다 (윈도우 {total}개, 최소 {config['min_training_windows']}개).")

    def fit(parts):
        """세션들로 PCA와 조용한 윈도우 기준(진동 크기 하위 10%)을 학습 (주파수 특징이 빠진 윈도우는 PCA에서 뺌)"""
        X = np.vstack([X for X, _ in parts])
        rms = np.concatenate([rms for _, rms in parts])
        return _fit_pca(X[~np.isnan(X).any(axis=1)], config['pca_variance']), float(np.quantile(rms, 0.1))

    def scored(pca, rms_low, parts):
        """채점하는 윈도우(rms_low 이상)의 (SPE, T²)"""
        X = np.vstack([X for X, _ in parts])
        rms = np.concatenate([rms for _, rms in parts])
        spe, t2, _ = pca_scores(pca, X[rms >= rms_low])
        return spe, t2

    pca, rms_low = fit(per_session)
    held_spe, held_t2 = [], []
    if len(per_session) >= 3:
        for i, session in enumerate(per_session):
            rest = [s for j, s in enumerate(per_session) if j != i]
            if sum(len(rms) for _, rms in rest) < config['min_training_windows']:
                continue
            spe, t2 = scored(*fit(rest), [session])
            held_spe.append(spe)
            held_t2.append(t2)
    if held_spe and sum(map(len, held_spe)):
        spe_values, t2_values, factor, calibration = np.concatenate(held_spe), np.concatenate(held_t2), 1.0, 'session_holdout'
    else:
        (spe_values, t2_values), factor, calibration = scored(pca, rms_low, per_session), 1.2, 'in_sample'
    rms_all = np.concatenate([rms for _, rms in per_session])
    return {
        'version': features.FEATURE_VERSION,
        'kind': 'pca',
        'profile': profile,
        'feature_names': names,
        'feature_config': feat_config,
        'pca': pca,
        'spe_threshold': max(float(np.quantile(spe_values, config['spe_quantile'])) * factor, 1e-6),
        't2_threshold': max(float(np.quantile(t2_values, config['t2_quantile'])) * factor, 1e-6),
        'calibration': calibration,
        'rms_threshold': float(np.quantile(rms_all, config['level_quantile'])),
        'rms_median': float(np.median(rms_all)),
        'rms_low': rms_low,
        'sessions': len(per_session),
        'windows': int(total),
        'fs_median': float(np.median(fs_values)) if fs_values else None,
    }


class DecisionRule:
    """판정 규칙 하나의 상태 (경보 시작·해제). mode: 'k_of_n', 'cusum', 'two_tier'

    update(증거값, 블록 증거값) → 'start'(경보 시작), 'end'(해제), None. 경보를 낸 경로는 trip에 남깁니다.
    블록 증거값 = 윈도우 끝 50 ms 블록 RMS / 기준선 / fast_ratio (고속 창 설정에서 기준선이 있을 때만, 아니면 None)
    """

    def __init__(self, mode, counts, config):
        self.mode, self.counts, self.config = mode, counts, config
        self.recent = deque(maxlen=counts['n'])
        self.cusum = 0.0
        self.fast_run = 0
        self.alarm = False
        self.trip = None
        self.release_count = 0

    def update(self, evidence, fast_evidence=None):
        cfg, counts = self.config, self.counts
        trip = None
        if self.mode == 'k_of_n':
            self.recent.append(evidence >= 1.0)
            if sum(self.recent) >= counts['k']:
                trip = 'k_of_n'
        else:
            # 2단계의 CUSUM은 증거값을 cusum_cap으로 잘라, 겹치는 윈도우 몇 개에 걸친 큰 충격 하나로는 넘지 않게 합니다
            value = min(evidence, cfg['cusum_cap']) if self.mode == 'two_tier' else evidence
            self.cusum = max(0.0, self.cusum + (math.log(max(value, 1e-6)) - cfg['cusum_drift']) * counts['hop_sec'])
            if self.cusum >= cfg['cusum_threshold']:
                trip = 'cusum'
            if self.mode == 'two_tier' and counts['fast_blocks']:
                self.fast_run = self.fast_run + 1 if fast_evidence is not None and fast_evidence >= 1.0 else 0
                if self.fast_run >= counts['fast_blocks']:
                    trip = 'fast'
        if not self.alarm:
            if trip:
                self.alarm, self.trip, self.release_count = True, trip, 0
                return 'start'
            return None
        level = max(evidence, fast_evidence or 0.0) if self.mode == 'two_tier' else evidence
        self.release_count = self.release_count + 1 if level < cfg['release_level'] else 0
        if self.release_count >= counts['release_windows']:
            self.alarm, self.trip = False, None
            self.recent.clear()
            self.cusum, self.fast_run = 0.0, 0
            return 'end'
        return None


class DetectionCore:
    """학습된 모델로 윈도우를 채점하고 판정 상태를 관리합니다 (실시간과 오프라인 평가가 함께 사용)."""

    def __init__(self, model, config, condition_model=None):
        self.model = model
        self.config = config
        self.condition_model = condition_model
        self.indicators = {}  # 마지막으로 채점한 윈도우의 PCA 표시값 (score)
        self._condition_key = self._condition_limit = None  # 조건 모델 상한을 마지막으로 계산한 조건과 그 값
        self.new_cut()

    @property
    def alarm(self):
        """대응에 쓰는 판정(decision_mode)의 경보 상태"""
        return self.rule.alarm

    def window_counts(self):
        """판정 시간 설정(초)을 이 모델의 평가 간격에서의 윈도우 수로 바꿉니다.

        confirm_sec·release_sec는 첫 윈도우 끝부터 마지막 윈도우 끝까지의 시간이라 (시간/간격)+1개,
        baseline_sec는 그 시간을 덮는 (시간/간격)개, fast_sec는 겹치지 않는 블록(block_sec) 개수입니다.
        """
        feature_config = self.model.get('feature_config') or {}
        hop = feature_config.get('hop_sec') or features.DEFAULT_FEATURE_CONFIG['hop_sec']
        window = feature_config.get('window_sec') or features.DEFAULT_FEATURE_CONFIG['window_sec']
        block = feature_config.get('block_sec')
        cfg = self.config
        n = int(round(cfg['confirm_sec'] / hop)) + 1
        return {'k': max(1, int(math.ceil(cfg['confirm_fraction'] * n - 1e-9))), 'n': n,
                'release_windows': int(round(cfg['release_sec'] / hop)) + 1,
                'baseline_windows': max(1, int(round(cfg['baseline_sec'] / hop))),
                # 현재 윈도우와 겹치는 직전 윈도우 수 + 1: 이만큼 앞의 윈도우부터 기준선에 씁니다
                'baseline_gap': max(1, int(round(window / hop))),
                'fast_blocks': max(1, int(round(cfg['fast_sec'] / block))) if block else 0,
                'hop_sec': hop}

    def new_cut(self):
        """새 컷(판정 구간 시작)마다 기준 구간과 판정 상태를 초기화합니다.
        고속 창 설정에서 대응 판정이 2단계가 아니고 shadow_two_tier면 2단계 판정을 기록 전용으로 함께 돌립니다."""
        self.counts = self.window_counts()
        mode = self.config['decision_mode']
        self.rule = DecisionRule(mode, self.counts, self.config)
        shadow = self.config.get('shadow_two_tier') and mode != 'two_tier' and self.counts['fast_blocks']
        self.shadow = DecisionRule('two_tier', self.counts, self.config) if shadow else None
        self.shadow_decision = None
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
        """윈도우 특징 → (증거값, 항목별 값, 채터 주파수 또는 None).
        경보에 쓰지 않는 PCA 표시값(T² / 기준, SPE 기여도가 큰 특징)은 self.indicators에 남깁니다."""
        cond = cond or {}
        model = self.model
        parts = {}
        self.indicators = {}
        # SPE·T²는 양쪽(너무 크거나 너무 작은 진동)을 모두 '드묾'으로 보므로, 학습 데이터 하위 10%보다 조용한
        # 윈도우는 제외합니다 (채터는 진동이 커지는 현상이고, 가벼운 절삭에서 오경보가 나지 않도록)
        if feats['rms'] >= model.get('rms_low', 0.0):
            spe, t2, contributions = pca_scores(model['pca'], features.feature_vector(feats, model['feature_names']))
            parts['spe'] = float(spe[0]) / model['spe_threshold']
            t2_ratio = float(t2[0]) / model['t2_threshold']
            if self.config['t2_alarm']:
                parts['t2'] = t2_ratio
            self.indicators = {'t2': t2_ratio, 'contributions': top_contributions(model['feature_names'], contributions[0])}
        limit = self.condition_limit(cond, feats.get('fs'))
        if limit:
            parts['condition'] = feats['rms'] / limit
        # 조건 모델이 이 조건의 정상 범위를 알면, 모든 조건을 섞은 절대 기준(level)은 보조(더 높은 기준)로 씀
        parts['level'] = feats['rms'] / (model['rms_threshold'] * (self.config['level_with_condition'] if limit else 1.0))
        if self.baseline:
            parts['self'] = feats['rms'] / self.baseline / self.config['self_ratio']
        chatter_freq = None
        if feats.get('spectral') and feats.get('nonharmonic_ratio') is not None \
                and feats.get('peak_prominence', 0.0) >= self.config['chatter_prominence_min']:
            parts['spectral'] = feats['nonharmonic_ratio'] / self.config['nonharmonic_ratio_thr']
            if parts['spectral'] >= 1.0:
                chatter_freq = feats.get('peak_freq')
        evidence = max(parts.values())
        return evidence, parts, chatter_freq

    def condition_limit(self, cond, fs=None):
        """조건 모델로 정한 지금 조건의 정상 진동 크기 상한 [g] (edit_analysis2.md P09).

        다음이면 None이고 조건 항목 없이 판정합니다.
        - 모델이 없거나, 날 수 등 조건을 모를 때
        - 센서 샘플링(fs)이 학습 데이터와 10% 넘게 다를 때: 샘플링·저역통과가 다르면 같은 진동도 크기가 다르게 잡힘
        - 학습한 조건 범위(양쪽 5% 여유) 밖일 때: 단조 GBDT가 경계의 값을 그대로 이어 써서 상한이 맞지 않음
        조건이 그대로면 다시 계산하지 않습니다.
        """
        cm = self.condition_model
        if not cm or (fs and cm.get('sensor_fs') and abs(fs / cm['sensor_fs'] - 1.0) > 0.1):
            return None
        physics = features.cutting_physics(cond.get('n'), cond.get('F'), cond.get('ap'), cond.get('flutes'), cond.get('ae'))
        if physics is None:
            return None
        x = tuple(physics[name] for name in cm['features'])
        if x != self._condition_key:
            ranges = [cm['training_range'][name] for name in cm['features']]
            inside = all(lo - 0.05 * (hi - lo) <= value <= hi + 0.05 * (hi - lo) for value, (lo, hi) in zip(x, ranges))
            self._condition_key = x
            self._condition_limit = math.exp(float(cm['model'].predict(np.array([x]))[0]) + cm['log_margin']) if inside else None
        return self._condition_limit

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

    def fast_evidence(self, feats):
        """블록 증거값 = 윈도우 끝 50 ms 블록 RMS / 기준선 / fast_ratio (블록 에너지나 기준선이 없으면 None)"""
        block = feats.get('block_rms')
        if block is None or not self.baseline:
            return None
        return block / self.baseline / self.config['fast_ratio']

    def _admit_baseline(self, feats, evidence, fast=None):
        """정상으로 보인 윈도우만 기준선 후보로 넣습니다. 경보 중이거나 증거값(블록 증거값 포함)이 release_level 이상이면
        넣지 않으므로, 이상이 보이기 시작하면 기준선이 멈춰 천천히 커지는 채터를 따라가지 않습니다."""
        level = evidence if fast is None else max(evidence, fast)
        if not self.alarm and level < self.config['release_level']:
            self.baseline_pool.append((self.window_index, feats['rms']))
            if self.anchor is None:
                self.anchor_values.append(feats['rms'])
                if len(self.anchor_values) >= self.counts['baseline_windows']:
                    self.anchor = float(np.median(self.anchor_values))

    def decide(self, evidence, fast_evidence=None):
        """판정 상태 갱신 → 'start'(경보 시작), 'end'(해제), None. 기록 전용 2단계 판정 결과는 shadow_decision에 남깁니다."""
        self.shadow_decision = self.shadow.update(evidence, fast_evidence) if self.shadow else None
        return self.rule.update(evidence, fast_evidence)

    def step(self, feats, cond=None):
        """판정 구간 안의 윈도우 하나를 처리합니다."""
        self._refresh_baseline()
        evidence, parts, chatter_freq = self.score(feats, cond)
        fast = self.fast_evidence(feats)
        self._admit_baseline(feats, evidence, fast)
        decision = self.decide(evidence, fast)
        self.window_index += 1
        return {'evidence': evidence, 'parts': parts, 'fast_evidence': fast, 'chatter_freq': chatter_freq,
                'fs': feats.get('fs'), 'baseline': self.baseline, 'decision': decision, 'alarm': self.alarm,
                'trip': self.rule.trip if decision == 'start' else None,
                'shadow_decision': self.shadow_decision,
                'shadow_trip': self.shadow.trip if self.shadow_decision == 'start' else None,
                't2': self.indicators.get('t2'), 'contributions': self.indicators.get('contributions', [])}

    def condition_changed(self):
        """판정 구간 안에서 회전수·이송(오버라이드)이 바뀌었을 때. 경보가 없으면 기준선을 새 조건으로 다시 쌓고,
        경보 중(온라인 대응 중)이면 이상 전 기준선을 그대로 둬서 대응 뒤에도 같은 기준으로 진정 여부를 봅니다."""
        if not self.alarm:
            self.reset_baseline()


class VibrationMonitor:
    def __init__(self, socketio, controllers, gcode_lines=None, tolerance_value=0.0, initial_spindle_override=100):
        self.socketio = socketio
        self.controllers = controllers
        saved, self.settings_problems = load_settings()  # 쓰지 않은 저장 설정은 모델 관리 창에도 표시 (setting_notes)
        for problem in self.settings_problems:
            logger.warning(f"저장된 감지 설정을 쓰지 않고 기본값을 씁니다: {problem}")
        self.config = dict(DETECTION_DEFAULTS, **saved)
        self.feature_config = features.profile_config(features.feature_config(self.config), 'legacy')
        self.clock = time.time
        self.load_saved_models()

        self.models = {}            # 창 설정별 감지 모델 {'legacy': ..., 'fast': ...}
        self.model_sources = {}     # 창 설정별 감지 모델의 출처: 'dataset'(models/detector_model.joblib) 또는 'folder'
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
        self.last_override = None   # 마지막 온라인 대응의 감지·전송·반영 확인 시각 (_apply_overrides)
        self._t2_recent = deque()   # 최근 판정 윈도우마다 T²가 기준 이상(학습 범위 밖)이었는지 (T2_RECENT_SEC 동안)
        self.reset_stream()

        self.initialize_analyzer()

    # ------------------------------------------------------------------ 학습
    def load_saved_models(self):
        """models/의 학습된 모델(scripts/train_models.py)을 읽습니다: 데이터셋 감지 모델, 조건 모델, 안정 한계 모델(최적화기).
        서버 시작과 '감지기 초기화'(reset_analyzer) 때 부르므로, 새로 학습한 모델을 서버를 다시 시작하지 않고 쓸 수 있습니다.
        감지 모델은 다음 train_model부터 쓰입니다."""
        self.saved_detector = load_detector_models()
        self.condition_model = load_condition_model()
        self.optimizer = CuttingConditionOptimizer()

    def train_model(self, sensor_data):
        """창 설정별 감지 모델을 준비합니다. 데이터셋 감지 모델(load_saved_models)이 있는 창 설정은 그 모델을 쓰고, 나머지는
        정상 가공 데이터(sensor_data: 학습 데이터 폴더의 CSV)로 학습합니다. 쓸 모델이 하나도 없으면 기존 모델을 그대로
        둡니다. 성공 여부를 반환합니다."""
        saved = dict((self.saved_detector or {}).get('models') or {})
        try:
            models = train_detectors(sensor_data, self.config)
        except Exception as e:
            if not saved:
                logger.error(f"모델 학습 중 오류 발생: {e}")
                return False
            if sensor_data:
                logger.warning(f"학습 데이터 폴더의 파일로 감지 모델을 학습하지 못했습니다: {e}")
            models = {}
        sources = {profile: 'folder' for profile in models}
        for profile, model in saved.items():  # 같은 창 설정이면 데이터셋 모델을 씀
            models[profile], sources[profile] = model, 'dataset'
        with self._model_lock:
            self.models = models
            self.model_sources = sources
            self.cores = {profile: DetectionCore(model, self.config, self.condition_model)
                          for profile, model in models.items()}
            self.is_trained = True
            self.training_windows = sum(model['windows'] for profile, model in models.items() if sources[profile] == 'folder')
            self._select_model()
        for profile, model in models.items():
            source = '데이터셋' if sources[profile] == 'dataset' else '학습 데이터 폴더'
            logger.info(f"감지 모델 준비 ({PROFILE_LABELS[profile]}, {source}: 세션 {model['sessions']}개, 가공 윈도우 {model['windows']}개, "
                        f"특징 {len(model['feature_names'])}개 → 주성분 {model['pca']['loadings'].shape[1]}개, "
                        f"SPE 기준 {model['spe_threshold']:.2f}, T² 기준 {model['t2_threshold']:.1f}, "
                        f"진동 크기 기준 {model['rms_threshold']:.3f} g)")
        return True

    def _select_model(self):
        """센서 창 설정에 맞는 모델을 고릅니다. 그 설정의 모델이 없으면 저속 모델(없으면 있는 모델)을 씁니다.
        실시간 윈도우는 고른 모델의 창 설정으로 만듭니다. _model_lock을 잡은 상태에서 부릅니다."""
        wanted = self.stream_profile or 'legacy'
        if wanted in self.models:
            profile = wanted
        else:
            profile = 'legacy' if 'legacy' in self.models else next(iter(self.models), None)
        if self.models.get(profile) is not self.model:  # 모델이 바뀌면 학습 범위 밖 비율을 새로 셈
            hop = (self.models[profile]['feature_config'].get('hop_sec') if profile else None) \
                or features.DEFAULT_FEATURE_CONFIG['hop_sec']
            self._t2_recent = deque(maxlen=max(1, int(round(T2_RECENT_SEC / hop))))
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
        """data/학습 데이터의 정상 가공 CSV(파일명에 '이상'이 없는 파일, master_dataset.csv가 있으면 그것만)로 학습합니다.
        데이터셋 감지 모델이 있는 창 설정은 그 모델을 쓰고(train_model), 폴더에 파일이 없어도 그 모델로 감지합니다."""
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
            if not loaded and not self.saved_detector:
                raise FileNotFoundError(f"정상 진동 데이터 파일을 찾을 수 없습니다: {TRAINING_DATA_DIR}")
            if loaded:
                logger.info(f"총 {len(loaded)}개 파일, {sum(d['samples'] for d in loaded)}개의 정상 데이터로 학습 시작")
            else:
                logger.info(f"{TRAINING_DATA_DIR}에 정상 진동 데이터 파일이 없어 데이터셋 감지 모델만 씁니다.")
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
        pca = model.get('pca') or {}
        latest = dict(self.latest) if self.latest else {}
        counts = self.core.counts if self.core else {}
        with self._model_lock:
            recent = list(self._t2_recent)
        hop = self.feature_config.get('hop_sec') or features.DEFAULT_FEATURE_CONFIG['hop_sec']
        saved, cm = self.saved_detector, self.condition_model
        return {
            'is_trained': self.is_trained,
            'is_training': self._training_lock.locked(),
            'training_dir': os.path.relpath(TRAINING_DATA_DIR, BASE_DIR).replace(os.sep, '/'),
            'files': list(self.training_files),
            'samples': self.training_samples,
            'windows': self.training_windows,
            'sessions': model.get('sessions'),
            'training_fs': model.get('fs_median'),
            'thresholds': {'spe': model.get('spe_threshold'), 't2': model.get('t2_threshold'),
                           'rms_g': model.get('rms_threshold'), 'calibration': model.get('calibration'),
                           'features': model.get('feature_names'),
                           'components': pca['loadings'].shape[1] if 'loadings' in pca else None,
                           'explained': pca.get('explained')},
            'settings': {key: self.config[key] for key in ('mitigation_enabled', 'decision_mode', 'shadow_two_tier', 't2_alarm',
                                                             'confirm_sec', 'release_sec', 'release_level', 'cusum_threshold',
                                                             'fast_ratio', 'fast_sec', 'mitigation_wait_sec', 'rework_min_passes')},
            'window': {'profile': self.profile, 'label': PROFILE_LABELS.get(self.profile),
                       'stream_profile': self.stream_profile, 'fallback': self.fallback,
                       'window_sec': self.feature_config.get('window_sec'), 'hop_sec': self.feature_config.get('hop_sec'),
                       'k': counts.get('k'), 'n': counts.get('n'), 'fast_blocks': counts.get('fast_blocks'),
                       'shadow': bool(self.core and self.core.shadow)},
            # 창 설정별 감지 모델: 출처('dataset' = models/detector_model.joblib, 'folder' = 학습 데이터 폴더), 특징 수, 비고조파 비율 포함 여부
            'profiles': {profile: {'label': PROFILE_LABELS[profile], 'source': self.model_sources.get(profile),
                                   'sessions': m.get('sessions'), 'windows': m.get('windows'), 'fs': m.get('fs_median'),
                                   'features': len(m.get('feature_names') or []),
                                   'nonharmonic': 'nonharmonic_ratio' in (m.get('feature_names') or [])}
                         for profile, m in self.models.items()},
            'detector_model': None if not saved else {
                'path': os.path.relpath(DETECTOR_MODEL_PATH, BASE_DIR).replace(os.sep, '/'), 'dataset': saved.get('dataset'),
                'trained_at': saved.get('trained_at'), 'profiles': sorted(saved['models'])},
            'condition_model': None if not cm else {
                'runs': cm.get('n_runs'), 'coverage': cm.get('coverage'), 'margin': math.exp(cm.get('log_margin', 0.0)),
                'sensor_fs': cm.get('sensor_fs'), 'trained_at': cm.get('trained_at')},
            # 최근 판정 윈도우 중 T²가 기준 이상(학습 범위 밖 조건)인 비율: 높으면 이 조건의 정상 데이터를 더 학습해야 함
            't2_recent': {'fraction': sum(recent) / len(recent), 'windows': len(recent), 'seconds': len(recent) * hop}
                         if recent else None,
            'notes': self.setting_notes(),
            'last_override': dict(self.last_override) if self.last_override else None,
            'optimizer': self.optimizer.describe(),
            'sensor': dict(self.sensor_info),
            'latest': {key: latest.get(key) for key in ('evidence', 'gate', 'alarm', 'stage', 'rms', 'fs', 'spectral', 't2')},
        }

    def setting_notes(self):
        """설정·센서·모델이 서로 맞지 않아 기대와 다르게 동작하는 점 (모델 관리 창에 표시, 설정 검증 P12)"""
        notes = [f"저장된 감지 설정을 쓰지 않고 기본값을 씀: {problem}" for problem in self.settings_problems]
        if self.config['decision_mode'] == 'two_tier' and self.core and not self.core.counts.get('fast_blocks'):
            notes.append(f"판정 방식이 2단계지만 지금 모델({PROFILE_LABELS[self.profile]})에는 50 ms 블록이 없어 빠른 차단 없이 "
                         "CUSUM으로만 판정합니다. 빠른 차단은 고속 센서(400 Hz 이상)에서 동작합니다.")
        fs = self.sensor_info.get('fs')
        model_fs = (self.model or {}).get('fs_median')
        if fs and model_fs and not self.fallback and abs(fs / model_fs - 1.0) > 0.1:
            notes.append(f"센서 샘플링({fs:.0f} Hz)이 감지 모델의 학습 데이터({model_fs:.0f} Hz)와 10% 넘게 다릅니다. "
                         "이 센서로 기록한 정상 가공 데이터로 다시 학습하세요.")
        cm = self.condition_model
        if fs and cm and cm.get('sensor_fs') and abs(fs / cm['sensor_fs'] - 1.0) > 0.1:
            notes.append(f"조건 모델은 {cm['sensor_fs']:.0f} Hz 센서로 학습되어 지금 센서({fs:.0f} Hz)에서는 쓰지 않습니다.")
        return notes

    def update_settings(self, updates):
        """화면에서 바꾼 설정을 검증(setting_problem)해 적용·저장합니다. 하나라도 틀리면 아무것도 바꾸지 않습니다.
        반환: (성공 여부, 메시지)"""
        applied = {}
        for key, value in (updates or {}).items():
            problem = setting_problem(key, value) if key in RUNTIME_SETTINGS else f"{key}: 화면에서 바꾸는 설정이 아님"
            if problem:
                return False, f"바꿀 수 없는 설정입니다: {problem}"
            applied[key] = value
        self.config.update(applied)
        save_settings(self.config)
        self.settings_problems = []  # 저장 파일을 검증한 값으로 다시 썼음
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
            profile = features.window_profile(data.get('fs'))
            kept = (f" {PROFILE_LABELS[profile]} 감지에는 데이터셋으로 학습한 감지 모델(models/detector_model.joblib)을 계속 씁니다."
                    if self.model_sources.get(profile) == 'dataset' else "")
            return True, (f"{saved_name}({data['samples']}행)을(를) 학습 데이터에 추가하고 모델을 다시 학습했습니다. "
                          f"(학습 파일 {len(self.training_files)}개){kept}")
        finally:
            self._training_lock.release()

    def reset_analyzer(self):
        """models/의 학습된 모델을 다시 읽고(load_saved_models), 학습 데이터 폴더의 파일로 모델을 다시 학습한 뒤 감지 상태
        (버퍼·판정)를 초기화합니다. scripts/train_models.py로 새로 학습한 모델도 서버를 다시 시작하지 않고 반영됩니다.

        반환값: (성공 여부, 메시지). 학습에 실패하면 기존 감지 모델을 유지합니다.
        """
        if not self._training_lock.acquire(blocking=False):
            return False, "이미 모델을 학습하는 중입니다. 끝난 뒤 다시 시도하세요."
        try:
            self.load_saved_models()
            if not self.initialize_analyzer():
                return False, "학습 데이터 폴더의 파일로 모델을 학습하지 못했습니다. 기존 모델을 그대로 사용합니다."
            self.reset_stream()
            dataset = [PROFILE_LABELS[p] for p, source in self.model_sources.items() if source == 'dataset']
            return True, (f"학습 데이터 {len(self.training_files)}개 파일로 감지기를 초기화했습니다."
                          + (f" 데이터셋 감지 모델: {', '.join(dataset)}." if dataset else ""))
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
                'tool_id': snap.get('tool_id'),
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
                  'evidence': None, 'parts': {}, 'alarm': False, 'chatter_freq': None, 't2': None, 'contributions': [],
                  'stage': self.event.get('stage') if self.event else None}
        if gate and not cond.get('transient'):
            with self._model_lock:
                core = self.core
                step = core.step(feats, cond) if core else None
                if step and step.get('t2') is not None:
                    self._t2_recent.append(step['t2'] >= 1.0)
            if step:
                result.update(evidence=step['evidence'], parts=step['parts'], alarm=step['alarm'],
                              chatter_freq=step['chatter_freq'], fast_evidence=step.get('fast_evidence'),
                              t2=step.get('t2'), contributions=step.get('contributions') or [])
                if step['decision'] == 'start':
                    self._on_alarm(cond, step, now)
                elif step['decision'] == 'end':
                    self._on_release()
                if step.get('shadow_decision') == 'start':
                    self._on_shadow_alarm(cond, step)
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
                't2': None if result['t2'] is None else round(result['t2'], 3),
                'contributions': result['contributions'],
            })

    # ------------------------------------------------------------------ 대응
    def _log(self, message, color="orange"):
        logger.info(message)
        self.socketio.emit('log_update', {"message": message, "color": color})

    def _on_alarm(self, cond, step, now):
        mitigate = bool(self.config['mitigation_enabled'] and cond.get('n_cmd') and cond.get('F_cmd')
                        and not self._mitigated_in_cut)
        trip = step.get('trip')
        cause = describe_cause(step['parts'], step.get('contributions'))
        self.event = {'started': now, 'detected_at': time.time(), 'stage': 'detected', 'cond': dict(cond),
                      'evidence': step['evidence'], 'parts': dict(step['parts']), 'chatter_freq': step['chatter_freq'],
                      'fs': step.get('fs'), 'trip': trip, 'cause': cause}
        self._log(f"⚠️ 이상 진동 감지 (지표 {step['evidence']:.2f}, 주요 원인 {cause}, "
                  f"판정 {TRIP_LABELS.get(trip, trip)})", "orange")
        self.socketio.emit('vibration_alert', {'show': True, 'evidence': step['evidence'], 'trip': trip,
                                               'message': f"⚠️ 이상 진동 감지 (지표 {step['evidence']:.2f})"})
        self._record(cond, step['evidence'], step['chatter_freq'], 'mitigate' if mitigate else 'stop', trip, cause)
        if mitigate:
            self._mitigate(now)
        else:
            self._stop_and_rework("이상 진동 감지로 정지합니다 (온라인 대응 꺼짐 또는 프로그램 S/F 없음).")

    def _on_shadow_alarm(self, cond, step):
        """기록 전용 2단계 판정의 경보: 로그와 감지 기록(Action=shadow)만 남기고 대응하지 않습니다."""
        trip = step.get('shadow_trip')
        fast = step.get('fast_evidence')
        detail = f", 블록 증거값 {fast:.2f}" if fast is not None else ""
        self._log(f"[기록 전용] 2단계 판정이 경보를 냈습니다 ({TRIP_LABELS.get(trip, trip)}, 지표 {step['evidence']:.2f}{detail}). "
                  "대응은 지금 판정 방식으로만 합니다.", "blue")
        self._record(cond, step['evidence'], step['chatter_freq'], 'shadow', trip,
                     describe_cause(step['parts'], step.get('contributions')))

    def _record(self, cond, evidence, chatter_freq, action, trip='', cause=''):
        position = tuple((cond.get(k) if cond.get(k) is not None else 0.0) for k in ('x', 'y', 'z'))
        try:
            append_detection_record(position, cond.get('n'), cond.get('F'), cond.get('spindle_override'),
                                    cond.get('feed_override'), evidence, chatter_freq, action, trip=trip, cause=cause)
        except Exception as e:
            logger.error(f"Vibration_detection.csv 기록 중 오류: {e}")

    def _mitigate(self, now):
        event = self.event
        cond = event['cond']
        suggestion = self.optimizer.suggest(cond.get('n'), cond.get('F'), cond.get('ap'), event['chatter_freq'],
                                            cond.get('flutes'), purpose='online', sample_rate=event.get('fs'),
                                            tool_id=cond.get('tool_id'))
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
        threading.Thread(target=self._apply_overrides, args=(spindle_target, feed_target, event), daemon=True).start()

    def _apply_overrides(self, spindle_target, feed_target, event=None):
        """이송을 먼저, 스핀들을 다음에 바꿉니다 (edit_analysis2.md P07).

        컨트롤러에 apply_overrides가 있으면 두 값을 한 번에 보내고 상태 보고(Ov:)로 반영을 확인합니다(cnc_app).
        감지 → 전송 → 반영 확인 시각을 사례(event['override'])와 last_override에 남기고 로그에 표시합니다.
        """
        targets = {'feed': feed_target, 'spindle': spindle_target}
        apply_both = getattr(self.controllers, 'apply_overrides', None)
        try:
            results = [apply_both(targets)] if callable(apply_both) else \
                [self.controllers.apply_override(target, value) for target, value in targets.items()]
        except Exception as e:
            results = [{'status': 'error', 'message': str(e)}]
        failed = [r for r in results if r.get('status') != 'success']
        for result in failed:
            self._log(f"온라인 대응 오버라이드 실패: {result.get('message')}", "red")
        result = results[0]
        if failed or not result.get('sent_at'):
            return
        detected = (event or {}).get('detected_at') or result['sent_at']
        timing = {'detected_at': detected, 'sent_at': result['sent_at'], 'confirmed_at': result.get('confirmed_at'),
                  'values': dict(result.get('values') or targets), 'commands': result.get('commands'),
                  'retries': result.get('retries', 0)}
        self.last_override = timing
        if event is not None:
            event['override'] = timing
        sent = f"감지 후 전송 {result['sent_at'] - detected:.2f}초"
        if timing['confirmed_at']:
            self._log(f"오버라이드 반영 확인: 이송 {timing['values']['feed']}%, 스핀들 {timing['values']['spindle']}% "
                      f"({sent}, 반영 확인 {timing['confirmed_at'] - detected:.2f}초"
                      + (f", 다시 보냄 {timing['retries']}회" if timing['retries'] else "") + ")", "blue")
        else:
            self._log(f"오버라이드를 보냈지만 상태 보고로 반영을 확인하지 못했습니다 ({sent}). "
                      f"{self.config['mitigation_wait_sec']:g}초 뒤 진동으로 판단합니다.", "orange")

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
                                                cond.get('flutes'), purpose='rework', sample_rate=event.get('fs'),
                                                tool_id=cond.get('tool_id'), min_passes=self.config['rework_min_passes'])
            if suggestion is None:
                self._log("현재 회전수·이송을 알 수 없어 재가공 파일을 만들지 않습니다.", "red")
                return
            passes = int(suggestion['passes'])  # 하한(rework_min_passes)은 최적화기가 넣음
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
