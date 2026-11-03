"""데이터셋(scripts/build_dataset.py 결과)으로 감지·재가공·온라인 대응에 쓸 모델을 한 번에 학습하고 보고합니다
(설계: edit_analysis2.md P08–P12)

정상 윈도우: 판정 구간(가공 중, 조건 변경 직후 제외)의 'stable' 윈도우 중, 같은 실행의 채터·전이 라벨 시작 앞
LABEL_GUARD_SEC(3초) 안에 끝나거나 라벨 시작에 걸친 윈도우를 뺀 것 (normal_windows). 작업자는 채터를 듣고 1–2초 늦게
라벨을 누르므로 그 앞의 윈도우에는 이미 채터가 섞였을 수 있습니다(분석 2.5). 감지 모델과 조건 모델이 이 윈도우로 학습하고,
scripts/evaluate_detector.py도 같은 윈도우로 학습해 평가합니다.

0. 감지 모델 → models/detector_model.joblib (src/vibration_monitor.py가 그 창 설정의 감지에 사용, edit_analysis2.md P12)
   - 창 설정(센서 샘플링: 1초 창 / 0.2초 창)마다 표준화 → PCA → SPE·T² 기준 (fit_detector, 실행 하나씩 빼고 정한 기준)
   - 데이터셋에는 가공 기록의 회전수가 있어 고속 창 모델에 비고조파 비율이 들어갑니다. 서버가 업로드 CSV(회전수 없음)로
     학습하는 모델에는 빠지는 특징입니다(P08).
1. 조건 모델 → models/condition_model.joblib (src/vibration_monitor.py가 감지에 사용, edit_analysis2.md P09)
   - 정상 가공 윈도우의 log(진동 크기)를 물리 변수(회전수 n, 절입 깊이 ap, 칩 단면적 A_chip = ap·F/(n·Z))로 예측
   - squared_error 단조 GBDT: ap·A_chip이 커지면 예측도 커지거나 같음 (분위수 손실은 단조 제약이 깨져서 쓰지 않음, 분석 2.4)
   - 상한 = 예측 + 학습에서 뺀 실행의 잔차 99% 분위수. 상한 포함률을 실행 단위 이중 교차 검증으로 확인해 출력합니다.
   - 날 수(flutes)를 모르는 실행은 A_chip을 계산할 수 없어 뺍니다.
   - 실데이터가 없을 때는 합성 grid 시험(make_synthetic_runs.py --grid)으로 같은 순서를 밟아 볼 수 있습니다.
2. 안정 한계 모델(SLD) → models/sld_gpr_model.joblib (edit_analysis2.md P10, src/optimizer.py가 재가공·온라인 대응에 사용)
   - build_dataset.py의 ramp_limits.csv(ramp 시험의 채터 시작 깊이)로, 날 주기 T = 60000/(n·Z) [ms]에 대한 한계 깊이를
     가우시안 프로세스(Constant × Matern ν=2.5 + White)로 학습합니다. 로브는 회전수가 아니라 날 주기에 대해 반복합니다(분석 2.5).
   - 끝까지 채터가 없던 실행(중도절단)은 시험한 최대 깊이로 넣습니다(실제 한계는 그보다 깊으므로 보수적).
   - 안전 깊이 = 평균 − κ·σ (--kappa, 기본 1.645 = 한쪽 95%). 회전수를 하나씩 빼고 예측해 오차와 포함률을 출력합니다.
   - 실데이터가 없을 때는 합성 ramp 시험(make_synthetic_runs.py --ramp)으로 같은 순서를 밟아 볼 수 있습니다.
모든 모델은 실행(SLD는 회전수) 단위 교차 검증(같은 실행의 윈도우가 학습·평가에 섞이지 않게) 결과를 출력합니다.

쓰지 않는 예전 형식의 모델(채터 확률 모델 GPC models/stability_model.joblib - P11에서 안정 한계 모델로 바꿈,
P09 이전 형식의 조건 모델)은 지우지 않고 models/legacy/로 옮깁니다(retire_model).
예전 RandomForest/LOF 모델(stft_dataset_3axis.csv 기반)은 현재 센서 특징과 단위가 달라 만들지 않습니다
(models/legacy/, edit_analysis.md 2.7).
학습한 모델은 서버를 다시 시작하거나 모델 관리 창의 '감지기 초기화'를 누르면 씁니다.

사용법: python scripts/train_models.py [--dataset data/dataset]
       합성 시험은 서버가 쓰는 기본 경로를 덮어쓰지 않도록 쓰지 않는 모델을 건너뛰고 다른 이름으로 저장합니다:
       python scripts/train_models.py --dataset data/synthetic/grid/dataset --skip-detector --skip-sld --condition-out models/condition_grid.joblib
       python scripts/train_models.py --dataset data/synthetic/ramp/dataset --skip-detector --skip-condition --sld-out models/sld_synthetic.joblib
"""
import argparse
import datetime
import math
import os
import shutil
import sys
import warnings

import joblib
import numpy as np
import pandas as pd
import sklearn
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.exceptions import ConvergenceWarning
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, Matern, WhiteKernel
from sklearn.model_selection import GroupKFold

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

import features  # noqa: E402
from optimizer import SLD_MODEL_KIND, SLD_MODEL_PATH, sld_safe_depth  # noqa: E402
from vibration_monitor import (CONDITION_MODEL_KIND, CONDITION_MODEL_PATH, DETECTION_DEFAULTS,  # noqa: E402
                               DETECTOR_MODEL_KIND, DETECTOR_MODEL_PATH, PROFILE_LABELS, fit_detector)

MODEL_DIR = os.path.join(BASE_DIR, 'models')
LEGACY_DIR = os.path.join(MODEL_DIR, 'legacy')
# 예전 모델 파일 → 설명 (쓰지 않으므로 학습할 때 models/legacy/로 옮김)
RETIRED_MODELS = {'stability_model.joblib': "예전 채터 확률 모델(GPC), P11에서 안정 한계 모델(sld_gpr_model.joblib)로 바꿈"}
# 채터·전이 라벨 시작 앞 이 시간 안의 윈도우는 정상 데이터로 쓰지 않음: 작업자의 반응 지연(분석 2.5)으로 이미 채터가
# 시작됐을 수 있는 구간 (build_dataset.ONSET_SEARCH_SEC, 채터 시작점을 다시 찾는 범위와 같음)
LABEL_GUARD_SEC = 3.0
# 창 설정·센서에 따라 없을 수 있는 윈도우 특징 (주파수 특징, 고속 창의 끝 블록 에너지, 샘플링)
OPTIONAL_KEYS = ('nonharmonic_ratio', 'peak_freq', 'peak_prominence', 'spectral_entropy', 'block_rms', 'fs')
DETECTOR_CONFIG_KEYS = ('spe_quantile', 't2_quantile', 'level_quantile', 'pca_variance', 'min_training_windows')

FEATURES = ['n', 'F', 'ap']
CONDITION_FEATURES = ['n', 'ap', 'a_chip']  # 회전수(제약 없음: 공진 등으로 오르내림), 절입 깊이, 칩 단면적
CONDITION_MONOTONIC = [0, 1, 1]             # ap·A_chip이 커지면 진동 크기 예측도 커지거나 같음
CONDITION_QUANTILE = 0.99                   # 상한 = 예측 + 학습에서 뺀 실행의 잔차 분위수 (log 단위)
SLD_KAPPA = 1.645   # 안전 깊이 = 평균 − κσ: 한쪽 95% 하한 (분석 2.5: 1.96은 한쪽 97.5%)
SLD_MIN_AP = 0.05   # 안전 깊이가 이보다 얕은 회전수는 '수동 확인' (판정 구간의 최소 깊이와 같은 값)


def load_cut_windows(dataset_dir):
    path = os.path.join(dataset_dir, 'windows.csv')
    if not os.path.exists(path):
        sys.exit(f"{path}가 없습니다. 먼저 python scripts/build_dataset.py를 실행하세요.")
    df = pd.read_csv(path, encoding='utf-8-sig', dtype={'run_id': str})
    cut = df[(df['gate'] == 1) & (df['transient'] == 0)].dropna(subset=FEATURES + ['rms'])
    return cut[cut['ap'] > 0]


def display_path(path):
    """출력·모델에 남길 경로 (프로젝트 안이면 상대 경로)"""
    absolute = os.path.abspath(path)
    try:
        inside = os.path.commonpath([absolute, BASE_DIR]) == BASE_DIR
    except ValueError:  # 다른 드라이브
        inside = False
    return os.path.relpath(absolute, BASE_DIR).replace('\\', '/') if inside else absolute


def grouped_folds(groups, max_splits=5):
    n_groups = len(set(groups))
    if n_groups < 2:
        return []
    return list(GroupKFold(n_splits=min(max_splits, n_groups)).split(np.zeros(len(groups)), groups=groups))


def row_features(row):
    """windows.csv의 한 행 → 감지기가 받는 윈도우 특징 (실시간 features.compute_features 결과와 같은 키)"""
    feats = {name: float(row[name]) for name in features.TIME_FEATURE_NAMES + ['rms']}
    feats['spectral'] = bool(row.get('spectral')) and str(row.get('spectral')).lower() not in ('false', '0', 'nan')
    for key in OPTIONAL_KEYS:
        value = row.get(key)
        feats[key] = None if value is None or pd.isna(value) else float(value)
    return feats


def window_lengths(windows):
    """윈도우마다 창 길이(초). 창 설정(profile) 열이 없는 P04 이전 데이터셋은 모두 1초 창"""
    profiles = windows['profile'] if 'profile' in windows.columns else pd.Series('legacy', index=windows.index)
    return profiles.map(lambda p: features.WINDOW_PROFILES.get(p, features.WINDOW_PROFILES['legacy'])['window_sec']).to_numpy(float)


def label_starts(dataset_dir, windows):
    """채터·전이 라벨이 시작한 서버 시각 {run_id: [시각, ...]}. labels.csv(라벨 시각 그대로)를 쓰고, 그 파일이 없는
    예전 데이터셋은 채터·전이 윈도우의 시작 시각(끝 시각 − 창 길이)을 씁니다."""
    path = os.path.join(dataset_dir, 'labels.csv')
    starts = {}
    if os.path.exists(path):
        labels = pd.read_csv(path, encoding='utf-8-sig', dtype={'run_id': str})
        marked = labels[labels['label'].isin(['chatter', 'transition']) & labels['start'].notna()]
        pairs = zip(marked['run_id'], marked['start'].astype(float))
    else:
        marked = windows[windows['label'].isin(['chatter', 'transition'])]
        pairs = zip(marked['run_id'], marked['t_host'].to_numpy(float) - window_lengths(marked))
    for run_id, start in pairs:
        starts.setdefault(run_id, []).append(float(start))
    return starts


def normal_windows(windows, starts, guard_sec=LABEL_GUARD_SEC):
    """정상 학습에 쓰는 윈도우: 판정 구간·조건 변경 직후 밖의 'stable' 윈도우 중, 같은 실행의 채터·전이 라벨 시작
    앞 guard_sec 구간에 걸치는 윈도우(그 구간 안에 끝나거나 라벨 시작에 걸친 윈도우)를 뺀 것.
    starts: label_starts 결과. 라벨 뒤의 'stable' 윈도우(채터가 끝난 뒤)는 그대로 씁니다."""
    normal = windows[(windows['gate'] == 1) & (windows['transient'] == 0) & (windows['label'] == 'stable')]
    if not starts or normal.empty:
        return normal
    t_end = normal['t_host'].to_numpy(float)
    t_start = t_end - window_lengths(normal)
    run_ids = normal['run_id'].to_numpy()
    keep = np.ones(len(normal), dtype=bool)
    for run_id, run_starts in starts.items():
        in_run = run_ids == run_id
        for start in run_starts:
            keep[in_run & (t_end > start - guard_sec) & (t_start < start)] = False
    return normal[keep]


def retire_model(path, reason):
    """쓰지 않는 모델 파일을 지우지 않고 models/legacy/로 옮깁니다 (같은 이름이 있으면 시각을 붙임)."""
    os.makedirs(LEGACY_DIR, exist_ok=True)
    stem, ext = os.path.splitext(os.path.basename(path))
    target = os.path.join(LEGACY_DIR, stem + ext)
    if os.path.exists(target):
        target = os.path.join(LEGACY_DIR, f"{stem}_{datetime.datetime.now():%Y%m%d_%H%M%S}{ext}")
    shutil.move(path, target)
    print(f"[예전 모델] {display_path(path)} → {display_path(target)} ({reason})")
    return target


def save_model(bundle, path):
    """모델을 저장합니다. 그 경로에 형식(kind)이 다른 예전 모델이 있으면 덮어쓰지 않고 models/legacy/로 옮깁니다."""
    if os.path.exists(path):
        try:
            old_kind = joblib.load(path).get('kind')
        except Exception:
            old_kind = None
        if old_kind != bundle['kind']:
            retire_model(path, f"예전 형식({old_kind or '형식 표시 없음'})이라 쓰지 않음")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    joblib.dump(bundle, path)


def train_detector(normal, dataset_dir, path=DETECTOR_MODEL_PATH):
    """정상 윈도우(normal_windows)로 창 설정(센서 샘플링)마다 감지 모델(PCA-SPE, fit_detector)을 학습해 한 파일에 저장합니다.
    세션 = 실행이라 SPE·T² 기준은 실행을 하나씩 빼고 채점한 값의 분위수입니다. 저장한 묶음(실패하면 None)을 반환합니다."""
    profiles = normal['profile'] if 'profile' in normal.columns else pd.Series('legacy', index=normal.index)
    models = {}
    for profile in sorted(set(profiles)):
        part = normal[profiles == profile]
        sessions = [[row_features(row) for _, row in rows.iterrows()] for _, rows in part.groupby('run_id', sort=True)]
        try:
            models[profile] = fit_detector(sessions, DETECTION_DEFAULTS, profile)
        except ValueError as e:
            print(f"[감지 모델] {PROFILE_LABELS.get(profile, profile)}: {e} 이 창 설정은 건너뜁니다.")
    if not models:
        print("[감지 모델] 학습할 정상 가공 윈도우가 없어 건너뜁니다.")
        return None
    bundle = {'kind': DETECTOR_MODEL_KIND, 'models': models, 'feature_version': features.FEATURE_VERSION,
              'dataset': display_path(dataset_dir), 'label_guard_sec': LABEL_GUARD_SEC,
              'config': {key: DETECTION_DEFAULTS[key] for key in DETECTOR_CONFIG_KEYS},
              'trained_at': datetime.datetime.now().isoformat(timespec='seconds'), 'numpy_version': np.__version__}
    save_model(bundle, path)
    print(f"[감지 모델] 저장: {display_path(path)} (창 설정 {len(models)}개, PCA-SPE)")
    for profile, model in models.items():
        pca, names = model['pca'], model['feature_names']
        sensor = f", 센서 약 {model['fs_median']:.0f} Hz" if model.get('fs_median') else ""
        calibration = ("실행 하나씩 빼고 정한 값" if model['calibration'] == 'session_holdout'
                       else "실행이 3개보다 적어 학습 데이터 분위수 × 1.2")
        print(f"  {PROFILE_LABELS[profile]}: 실행 {model['sessions']}개, 정상 윈도우 {model['windows']}개{sensor}, "
              f"특징 {len(names)}개 → 주성분 {pca['loadings'].shape[1]}개 (설명 분산 {pca['explained']:.1%})")
        print(f"    기준: SPE {model['spe_threshold']:.2f}, T² {model['t2_threshold']:.1f} ({calibration}), "
              f"진동 크기 {model['rms_threshold']:.3f} g")
        if profile == 'fast' and 'nonharmonic_ratio' not in names:
            print("    ⚠️ 비고조파 비율이 학습 윈도우의 95%보다 적게 계산되어 입력에서 빠졌습니다. "
                  "가공 기록의 회전수와 고조파 마스크(masked_fraction)를 확인하세요.")
    print("  감지율·지연·오경보는 scripts/evaluate_detector.py로 확인하세요 (같은 정상 윈도우로 실행 단위 교차 검증).")
    return bundle


def condition_rows(cut):
    """정상 가공(stable) 윈도우의 물리 변수(features.cutting_physics)와 log(진동 크기). 날 수를 모르는 윈도우는 뺍니다."""
    stable = cut[cut['label'] == 'stable']
    missing = pd.Series(np.nan, index=stable.index)
    rows = []
    for run_id, n, F, ap, flutes, ae, rms in zip(stable['run_id'], stable['n'], stable['F'], stable['ap'],
                                                 stable.get('flutes', missing), stable.get('ae', missing), stable['rms']):
        physics = features.cutting_physics(n, F, ap, flutes, ae)
        if physics:
            rows.append(dict(physics, run_id=run_id, log_rms=math.log(max(float(rms), 1e-6))))
    return pd.DataFrame(rows)


def make_condition_model():
    """log(진동 크기)를 예측하는 단조 GBDT. squared_error 손실이어야 단조 제약이 지켜집니다(분석 2.4)."""
    return HistGradientBoostingRegressor(loss='squared_error', monotonic_cst=CONDITION_MONOTONIC, max_iter=100,
                                         learning_rate=0.1, min_samples_leaf=20, random_state=0)


def held_out_residuals(X, y, groups):
    """실행 단위로 나눠 학습에서 뺀 실행의 잔차(log 실제 − log 예측)"""
    residuals = np.full(len(y), np.nan)
    for train_idx, test_idx in grouped_folds(groups):
        model = make_condition_model().fit(X[train_idx], y[train_idx])
        residuals[test_idx] = y[test_idx] - model.predict(X[test_idx])
    return residuals[~np.isnan(residuals)]


def train_condition(cut, path=CONDITION_MODEL_PATH):
    """정상 윈도우(normal_windows)로 조건 모델을 학습해 저장합니다. 저장한 묶음(건너뛰면 None)을 반환합니다."""
    rows = condition_rows(cut)
    n_runs = rows['run_id'].nunique() if not rows.empty else 0
    if n_runs < 3:
        print("[조건 모델] 날 수를 아는 정상 가공 실행이 3개 이상 필요합니다 (기록 화면의 날 수 또는 NC META). 건너뜁니다.")
        return None
    X = rows[CONDITION_FEATURES].to_numpy(float)
    y = rows['log_rms'].to_numpy(float)
    groups = rows['run_id'].to_numpy()
    margin = float(np.quantile(held_out_residuals(X, y, groups), CONDITION_QUANTILE))
    # 상한 포함률(이중 교차 검증): 바깥에서 뺀 실행을, 나머지 실행으로만 정한 모델과 상한(안쪽 교차 검증 분위수)으로 확인
    inside = []
    for train_idx, test_idx in grouped_folds(groups):
        inner = held_out_residuals(X[train_idx], y[train_idx], groups[train_idx])
        if len(inner):
            model = make_condition_model().fit(X[train_idx], y[train_idx])
            inside.append(y[test_idx] <= model.predict(X[test_idx]) + np.quantile(inner, CONDITION_QUANTILE))
    coverage = float(np.concatenate(inside).mean()) if inside else None
    model = make_condition_model().fit(X, y)
    stable_fs = cut.loc[cut['label'] == 'stable', 'fs'] if 'fs' in cut.columns else pd.Series(dtype=float)
    bundle = {'kind': CONDITION_MODEL_KIND, 'model': model, 'features': list(CONDITION_FEATURES),
              'sensor_fs': float(stable_fs.median()) if stable_fs.notna().any() else None,  # 샘플링이 다른 센서에는 쓰지 않음
              'monotonic': list(CONDITION_MONOTONIC), 'quantile': CONDITION_QUANTILE, 'log_margin': margin,
              'coverage': coverage,
              'training_range': {name: [float(rows[name].min()), float(rows[name].max())] for name in CONDITION_FEATURES},
              'trained_at': datetime.datetime.now().isoformat(timespec='seconds'),
              'sklearn_version': sklearn.__version__, 'n_runs': int(n_runs), 'n_rows': int(len(y))}
    save_model(bundle, path)
    print(f"[조건 모델] 저장: {display_path(path)} (실행 {n_runs}개, 정상 윈도우 {len(y)}개, 입력 {'·'.join(CONDITION_FEATURES)}, "
          f"ap·a_chip 증가 방향 단조)")
    print(f"  상한 = 예측 진동 크기 × {math.exp(margin):.2f} (학습에서 뺀 실행의 잔차 {CONDITION_QUANTILE:.0%} 분위수)")
    if coverage is not None:
        print(f"  실행 단위 이중 교차 검증: 학습에서 뺀 실행의 윈도우가 상한 안에 든 비율 {coverage:.1%} "
              f"(목표 {CONDITION_QUANTILE:.0%})")
        if coverage < CONDITION_QUANTILE - 0.02:
            print("  ⚠️ 상한 안에 드는 비율이 목표보다 낮습니다. 실행 사이의 차이가 커서 상한을 믿기 어렵습니다. "
                  "같은 조건을 반복 기록하거나 실행을 늘리세요.")
    return bundle


def sld_rows(dataset_dir):
    """ramp_limits.csv → 학습 행 (회전수 n, 날 수, 목표 깊이 target, 중도절단 여부). 파일이 없으면 None.
    중도절단 실행의 목표는 시험한 최대 깊이입니다(실제 한계는 그보다 깊음). P10 이전 파일은 시작 깊이가 비면 중도절단."""
    path = os.path.join(dataset_dir, 'ramp_limits.csv')
    if not os.path.exists(path):
        return None
    rows = pd.read_csv(path, encoding='utf-8-sig')
    for column in ('n', 'onset_ap', 'max_ap', 'flutes'):
        rows[column] = pd.to_numeric(rows[column], errors='coerce') if column in rows.columns else np.nan
    if 'censored' not in rows.columns:
        rows['censored'] = rows['onset_ap'].isna().astype(int)
    rows['censored'] = pd.to_numeric(rows['censored'], errors='coerce').fillna(0).astype(int)
    rows['tool_id'] = rows['tool_id'].fillna('').astype(str) if 'tool_id' in rows.columns else ''
    rows['target'] = np.where(rows['censored'] == 1, rows['max_ap'], rows['onset_ap'])
    return rows.dropna(subset=['n', 'target'])


def make_sld_model():
    """날 주기 T [ms] → 한계 깊이 [mm] 가우시안 프로세스: 상수 × Matern(ν=2.5) + 잡음 (분석 2.5)"""
    kernel = (ConstantKernel(1.0, (1e-2, 1e2)) * Matern(length_scale=0.3, length_scale_bounds=(0.02, 5.0), nu=2.5)
              + WhiteKernel(1e-3, (1e-6, 1e-1)))
    return GaussianProcessRegressor(kernel=kernel, normalize_y=True, n_restarts_optimizer=5, random_state=0)


def fit_sld(X, y):
    with warnings.catch_warnings():  # 길이 척도가 경계에 닿았다는 경고는 데이터가 적을 때 흔하므로 출력하지 않음
        warnings.simplefilter('ignore', ConvergenceWarning)
        return make_sld_model().fit(X, y)


def train_sld(dataset_dir, path=SLD_MODEL_PATH, kappa=SLD_KAPPA):
    """ramp 시험 결과로 안정 한계 모델을 학습해 저장합니다. 저장한 묶음(건너뛰면 None)을 반환합니다."""
    rows = sld_rows(dataset_dir)
    if rows is None or rows.empty:
        print("[안정 한계 모델] ramp 시험 결과(ramp_limits.csv)가 없어 건너뜁니다 (make_experiment_nc.py ramp로 기록).")
        return None
    rows = rows[rows['flutes'] > 0]
    if rows.empty:
        print("[안정 한계 모델] 날 수를 아는 ramp 시험이 없어 날 주기를 계산할 수 없습니다. 건너뜁니다.")
        return None
    tools = rows['tool_id'] + ' / ' + rows['flutes'].astype(int).astype(str) + '날'
    if tools.nunique() > 1:  # 한계는 공구마다 다르므로 기록이 가장 많은 공구만 씀
        print(f"[안정 한계 모델] 공구가 {tools.nunique()}개라 기록이 가장 많은 공구({tools.value_counts().idxmax()})만 씁니다.")
        rows = rows[tools == tools.value_counts().idxmax()]
    speeds = rows['n'].round().to_numpy()
    if len(set(speeds)) < 3:
        print("[안정 한계 모델] 회전수 3개 이상의 ramp 시험이 필요합니다. 건너뜁니다.")
        return None
    flutes = int(rows['flutes'].iloc[0])
    X = (60000.0 / (rows['n'].to_numpy(float) * flutes)).reshape(-1, 1)
    y = rows['target'].to_numpy(float)
    censored = rows['censored'].to_numpy(int) == 1
    max_tested = float(rows['max_ap'].max())
    # 회전수 하나씩 빼고 예측: 채터가 난 실행의 오차와, 실제 시작 깊이가 안전 깊이 이상인 비율(목표 약 95%)
    errors, safe = [], []
    for speed in sorted(set(speeds)):
        held = speeds == speed
        mean, std = fit_sld(X[~held], y[~held]).predict(X[held], return_std=True)
        mean = np.minimum(mean, max_tested)
        errors.extend((mean - y[held])[~censored[held]])
        safe.extend((y[held] >= mean - kappa * std)[~censored[held]])
    model = fit_sld(X, y)
    bundle = {'kind': SLD_MODEL_KIND, 'model': model, 'features': ['T'], 'flutes': flutes,
              'tool_id': rows['tool_id'].iloc[0] or None, 'kappa': kappa, 'min_ap': SLD_MIN_AP, 'max_tested_ap': max_tested,
              'speed_range': [float(speeds.min()), float(speeds.max())],
              'training_range': {'T': [float(X.min()), float(X.max())]}, 'kernel': str(model.kernel_),
              'cv': {'mae_mm': float(np.mean(np.abs(errors))) if errors else None,
                     'safe_fraction': float(np.mean(safe)) if safe else None, 'observed_runs': len(errors)},
              'trained_at': datetime.datetime.now().isoformat(timespec='seconds'), 'sklearn_version': sklearn.__version__,
              'n_runs': int(len(rows)), 'n_speeds': len(set(speeds)), 'n_censored': int(censored.sum())}
    save_model(bundle, path)
    print(f"[안정 한계 모델] 저장: {display_path(path)} (ramp {len(rows)}개, 회전수 {len(set(speeds))}개, 중도절단 {censored.sum()}개, "
          f"날 {flutes}개, κ={kappa:g})")
    print(f"  입력: 날 주기 T = 60000/(n·{flutes}) ms, 커널: {model.kernel_}")
    if errors:
        print(f"  회전수 하나씩 빼고 예측: 평균 절대 오차 {np.mean(np.abs(errors)):.3f} mm, "
              f"실제 시작 깊이가 안전 깊이 이상인 비율 {np.mean(safe):.0%} (목표 약 95%)")
        if np.mean(safe) < 0.9:
            print("  ⚠️ 시험하지 않은 회전수의 한계를 이웃 회전수로 맞히지 못합니다. 로브가 시험 간격보다 좁을 수 있습니다.\n"
                  "     안전 깊이는 시험한 회전수와 그 사이에서만 믿고, 쓰려는 회전수 근처는 더 촘촘히(예: 500 rpm 간격) 시험하세요.")
    for speed in sorted(set(speeds)):
        tested = ", ".join(f"≥{v:.3f}" if c else f"{v:.3f}" for v, c in zip(y[speeds == speed], censored[speeds == speed]))
        limit = sld_safe_depth(bundle, speed)
        print(f"  {speed:.0f} rpm: 시험 {tested} → 평균 {limit['mu']:.3f}, 안전 깊이 {limit['ap_safe']:.3f} mm"
              + (" (수동 확인)" if limit['manual_check'] else ""))
    return bundle


def main():
    parser = argparse.ArgumentParser(description="감지 모델, 조건 모델, 안정 한계 모델 학습")
    parser.add_argument('--dataset', default=os.path.join(BASE_DIR, 'data', 'dataset'))
    parser.add_argument('--skip-detector', action='store_true')
    parser.add_argument('--detector-out', default=DETECTOR_MODEL_PATH,
                        help="감지 모델을 저장할 경로 (기본: 서버가 쓰는 models/detector_model.joblib. 합성 시험은 건너뛰거나 다른 이름으로)")
    parser.add_argument('--skip-condition', action='store_true')
    parser.add_argument('--condition-out', default=CONDITION_MODEL_PATH,
                        help="조건 모델을 저장할 경로 (기본: 서버가 쓰는 models/condition_model.joblib. 합성 시험은 다른 이름으로)")
    parser.add_argument('--skip-sld', action='store_true')
    parser.add_argument('--sld-out', default=SLD_MODEL_PATH, help="안정 한계 모델을 저장할 경로 (기본: models/sld_gpr_model.joblib)")
    parser.add_argument('--kappa', type=float, default=SLD_KAPPA, help="안전 깊이 = 평균 − κσ (기본 1.645 = 한쪽 95%%)")
    args = parser.parse_args()
    for name, reason in RETIRED_MODELS.items():
        if os.path.exists(os.path.join(MODEL_DIR, name)):
            retire_model(os.path.join(MODEL_DIR, name), reason)
    cut = load_cut_windows(args.dataset)
    if cut.empty:
        sys.exit("판정 구간(가공 중) 윈도우가 없습니다. 기록과 데이터셋을 확인하세요.")
    normal = normal_windows(cut, label_starts(args.dataset, cut))
    guarded = int((cut['label'] == 'stable').sum()) - len(normal)
    print(f"정상 윈도우 {len(normal)}개 (실행 {normal['run_id'].nunique()}개). 채터·전이 라벨 시작 앞 {LABEL_GUARD_SEC:g}초 안의 "
          f"{guarded}개는 늦은 라벨 때문에 채터가 섞였을 수 있어 뺐습니다.")
    saved = []
    if not args.skip_detector and train_detector(normal, args.dataset, args.detector_out):
        saved.append(f"감지 모델({display_path(args.detector_out)})")
    if not args.skip_condition and train_condition(normal, args.condition_out):
        saved.append(f"조건 모델({display_path(args.condition_out)})")
    if not args.skip_sld and train_sld(args.dataset, args.sld_out, args.kappa):
        saved.append(f"안정 한계 모델({display_path(args.sld_out)})")
    if saved:
        print(f"저장한 모델: {', '.join(saved)}. 서버를 다시 시작하거나 모델 관리 창의 '감지기 초기화'를 누르면 새 모델을 씁니다.")
    else:
        print("저장한 모델이 없습니다.")


if __name__ == '__main__':
    main()
