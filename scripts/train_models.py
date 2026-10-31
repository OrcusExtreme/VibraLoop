"""데이터셋(scripts/build_dataset.py 결과)으로 재가공·온라인 대응에 쓸 모델을 학습합니다
(설계: edit_analysis.md 2.5, 2.7)

1. 채터 확률 모델 → models/stability_model.joblib (src/optimizer.py가 사용)
   - 입력 (회전수 n, 이송 F, 깊이 ap) → 채터 확률
   - 판정 구간 안의 윈도우를 실행·깊이(0.05 mm 단위)별로 묶고, 채터 윈도우가 절반 이상이면 채터(1)
   - 행이 400개 이하면 가우시안 프로세스 분류(데이터에서 멀면 확률이 0.5 쪽으로 가서 보수적), 많으면 2차 로지스틱 회귀
2. 조건 모델 → models/condition_model.joblib (src/vibration_monitor.py가 감지에 사용)
   - 안정 가공 윈도우의 log(진동 크기)를 (n, F, ap)로 예측, 예측 오차의 표준편차를 sigma로 저장
두 모델 모두 실행 단위 교차 검증(같은 실행의 윈도우가 학습·평가에 섞이지 않게) 결과를 출력합니다.

예전 RandomForest/LOF 모델(stft_dataset_3axis.csv 기반)은 현재 센서 특징과 단위가 달라 만들지 않습니다
(models/legacy/, edit_analysis.md 2.7).

사용법: python scripts/train_models.py [--dataset data/dataset] [--p-max 0.05]
"""
import argparse
import datetime
import os
import sys

import joblib
import numpy as np
import pandas as pd
import sklearn
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.gaussian_process import GaussianProcessClassifier, GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel, WhiteKernel
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import brier_score_loss, roc_auc_score
from sklearn.model_selection import GroupKFold
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

from optimizer import STABILITY_MODEL_PATH, save_stability_model  # noqa: E402
from vibration_monitor import CONDITION_MODEL_PATH  # noqa: E402

FEATURES = ['n', 'F', 'ap']
GP_MAX_ROWS = 400


def load_cut_windows(dataset_dir):
    path = os.path.join(dataset_dir, 'windows.csv')
    if not os.path.exists(path):
        sys.exit(f"{path}가 없습니다. 먼저 python scripts/build_dataset.py를 실행하세요.")
    df = pd.read_csv(path, encoding='utf-8-sig')
    cut = df[(df['gate'] == 1) & (df['transient'] == 0)].dropna(subset=FEATURES + ['rms'])
    return cut[cut['ap'] > 0]


def stability_rows(cut):
    """실행·깊이 구간별로 묶어 (n, F, ap, 채터 여부, 실행) 행을 만듭니다 (전이 구간은 제외)."""
    labeled = cut[cut['label'].isin(['stable', 'chatter'])].copy()
    labeled['ap_bin'] = (labeled['ap'] / 0.05).round() * 0.05
    grouped = labeled.groupby(['run_id', 'ap_bin'])
    rows = grouped.agg(n=('n', 'median'), F=('F', 'median'), ap=('ap', 'median'),
                       chatter=('label', lambda s: float((s == 'chatter').mean() >= 0.5))).reset_index()
    return rows


def grouped_folds(groups, max_splits=5):
    n_groups = len(set(groups))
    if n_groups < 2:
        return []
    return list(GroupKFold(n_splits=min(max_splits, n_groups)).split(np.zeros(len(groups)), groups=groups))


def varying_features(rows, label):
    """값이 하나뿐인 조건(예: 이송을 바꾸지 않은 실험)은 영향을 학습할 수 없으므로 입력에서 뺍니다."""
    keep = [name for name in FEATURES if rows[name].nunique() > 1]
    dropped = [name for name in FEATURES if name not in keep]
    if dropped:
        print(f"[{label}] {', '.join(dropped)}의 값이 하나뿐이라 입력에서 뺍니다 (그 조건의 영향은 학습하지 못함).")
    return keep


def gp_kernel():
    # 표준화한 입력에서 하나의 길이 척도(하한 0.3)를 쓰는 부드러운 커널: 데이터가 적을 때 축별 척도가 과적합되는 것을 막음
    return ConstantKernel(1.0) * RBF(length_scale=1.0, length_scale_bounds=(0.3, 10.0))


def make_classifier(n_rows):
    if n_rows <= GP_MAX_ROWS:
        return GaussianProcessClassifier(kernel=gp_kernel(), random_state=0)
    return make_pipeline(PolynomialFeatures(2), LogisticRegression(max_iter=2000))


def train_stability(cut, p_max):
    rows = stability_rows(cut)
    if rows.empty or rows['chatter'].nunique() < 2:
        print("[채터 확률 모델] 안정·채터 두 종류의 데이터가 모두 있어야 학습할 수 있습니다. 건너뜁니다.")
        return
    features = varying_features(rows, "채터 확률 모델")
    if not features:
        print("[채터 확률 모델] 조건이 모두 같아 학습할 수 없습니다. 건너뜁니다.")
        return
    X = rows[features].to_numpy(float)
    y = rows['chatter'].to_numpy(int)
    groups = rows['run_id'].to_numpy()
    oof = np.full(len(y), np.nan)
    for train_idx, test_idx in grouped_folds(groups):
        if len(set(y[train_idx])) < 2:
            continue
        scaler = StandardScaler().fit(X[train_idx])
        model = make_classifier(len(train_idx)).fit(scaler.transform(X[train_idx]), y[train_idx])
        oof[test_idx] = model.predict_proba(scaler.transform(X[test_idx]))[:, list(model.classes_).index(1)]
    cv = {}
    done = ~np.isnan(oof)
    if done.any():
        cv = {'accuracy': float(((oof[done] >= 0.5) == y[done]).mean()), 'brier': float(brier_score_loss(y[done], oof[done]))}
        if len(set(y[done])) == 2:
            cv['auc'] = float(roc_auc_score(y[done], oof[done]))
        safe = done & (oof < p_max)
        if safe.any():
            cv['chatter_rate_when_predicted_safe'] = float(y[safe].mean())
    scaler = StandardScaler().fit(X)
    model = make_classifier(len(y)).fit(scaler.transform(X), y)
    fitted = model.predict_proba(scaler.transform(X))[:, list(model.classes_).index(1)]
    save_stability_model(model, features, scaler=scaler, p_max=p_max, n_runs=int(len(set(groups))), n_rows=int(len(y)),
                         training_range={name: [float(rows[name].min()), float(rows[name].max())] for name in FEATURES},
                         cv=cv)
    print(f"[채터 확률 모델] 저장: {STABILITY_MODEL_PATH} (실행 {len(set(groups))}개, 행 {len(y)}개, 채터 비율 {y.mean():.0%})")
    print(f"  실행 단위 교차 검증: {cv or '실행이 부족해 계산하지 못함'}")
    print(f"  학습 조건에서 예측한 채터 확률 범위: {fitted.min():.1%} ~ {fitted.max():.1%}")
    if 'chatter_rate_when_predicted_safe' in cv:
        print(f"  → '안전'(채터 확률 < {p_max:.0%})으로 본 조건에서 실제 채터 비율: {cv['chatter_rate_when_predicted_safe']:.1%}")
    if fitted.min() >= p_max:
        # 가우시안 프로세스는 데이터가 적으면 확률을 0.5 쪽으로 보수적으로 냄 → 기준 미만 조건이 없으면 최적화기는 기본 규칙 사용
        print(f"  ⚠️ 채터 확률이 {p_max:.0%} 미만인 조건이 없어, 이대로는 최적화기가 이 모델 대신 채터 주파수·기본 규칙을 씁니다. "
              f"데이터(특히 안정 조건)를 늘리거나 --p-max를 높여 다시 학습하세요.")


def make_regressor(n_rows):
    if n_rows <= GP_MAX_ROWS:
        return GaussianProcessRegressor(kernel=gp_kernel() + WhiteKernel(0.1), normalize_y=True, random_state=0)
    return GradientBoostingRegressor(random_state=0)


def train_condition(cut):
    stable = cut[cut['label'] == 'stable'].copy()
    if stable['run_id'].nunique() < 3:
        print("[조건 모델] 안정 가공 실행이 3개 이상 필요합니다. 건너뜁니다.")
        return
    stable['log_rms'] = np.log(stable['rms'].clip(lower=1e-6))
    stable['ap_bin'] = (stable['ap'] / 0.05).round() * 0.05
    rows = stable.groupby(['run_id', 'ap_bin']).agg(n=('n', 'median'), F=('F', 'median'), ap=('ap', 'median'),
                                                     log_rms=('log_rms', 'median')).reset_index()
    features = varying_features(rows, "조건 모델")
    if not features:
        print("[조건 모델] 조건이 모두 같아 학습할 수 없습니다. 건너뜁니다.")
        return
    X = rows[features].to_numpy(float)
    y = rows['log_rms'].to_numpy(float)
    groups = rows['run_id'].to_numpy()
    residuals = []
    for train_idx, test_idx in grouped_folds(groups):
        scaler = StandardScaler().fit(X[train_idx])
        model = make_regressor(len(train_idx)).fit(scaler.transform(X[train_idx]), y[train_idx])
        test_runs = set(groups[test_idx])
        windows = stable[stable['run_id'].isin(test_runs)]
        predicted = model.predict(scaler.transform(windows[features].to_numpy(float)))
        residuals.append(windows['log_rms'].to_numpy(float) - predicted)  # 윈도우 단위 오차 (실행 안 흔들림 포함)
    sigma = float(np.std(np.concatenate(residuals))) if residuals else 0.3
    scaler = StandardScaler().fit(X)
    model = make_regressor(len(y)).fit(scaler.transform(X), y)
    bundle = {'model': model, 'scaler': scaler, 'features': features, 'sigma': max(sigma, 0.05),
              'trained_at': datetime.datetime.now().isoformat(timespec='seconds'),
              'sklearn_version': sklearn.__version__, 'n_runs': int(len(set(groups))), 'n_rows': int(len(y))}
    joblib.dump(bundle, CONDITION_MODEL_PATH)
    print(f"[조건 모델] 저장: {CONDITION_MODEL_PATH} (실행 {len(set(groups))}개, 예측 오차 표준편차 {sigma:.3f} (log 단위))")


def main():
    parser = argparse.ArgumentParser(description="채터 확률 모델과 조건 모델 학습")
    parser.add_argument('--dataset', default=os.path.join(BASE_DIR, 'data', 'dataset'))
    parser.add_argument('--p-max', type=float, default=0.05, help="최적화기가 '안전'으로 보는 채터 확률 상한")
    parser.add_argument('--skip-stability', action='store_true')
    parser.add_argument('--skip-condition', action='store_true')
    args = parser.parse_args()
    cut = load_cut_windows(args.dataset)
    if cut.empty:
        sys.exit("판정 구간(가공 중) 윈도우가 없습니다. 기록과 데이터셋을 확인하세요.")
    if not args.skip_stability:
        train_stability(cut, args.p_max)
    if not args.skip_condition:
        train_condition(cut)
    print("서버를 다시 시작하면 새 모델을 사용합니다.")


if __name__ == '__main__':
    main()
