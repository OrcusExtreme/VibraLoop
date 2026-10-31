import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestRegressor
from sklearn.multioutput import MultiOutputRegressor
from sklearn.neighbors import LocalOutlierFactor
import joblib
import os

# ---------------------------------------------------------
# 설정값
# ---------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STFT_DATASET_PATH = os.path.join(BASE_DIR, "data", "학습 데이터", "stft_dataset_3axis.csv")
N_RANGE = (16000, 24000)
F_RANGE = (50, 150)

# 모델 저장 경로
MODEL_DIR = os.path.join(BASE_DIR, "models")
REGR_MODEL_PATH = os.path.join(MODEL_DIR, "regr_model.joblib")
LOF_MODEL_PATH = os.path.join(MODEL_DIR, "lof_model.joblib")

def train_and_save_models():
    print(f"데이터셋 로드: {STFT_DATASET_PATH}")
    try:
        df = pd.read_csv(STFT_DATASET_PATH)
    except FileNotFoundError:
        print(f"오류: {STFT_DATASET_PATH} 파일을 찾을 수 없습니다. 경로를 확인해주세요.")
        return

    X = df[["n", "F"]].values
    Y_cols = [
        'mean_x', 'std_x', 'max_x', 'energy_x',
        'mean_y', 'std_y', 'max_y', 'energy_y',
        'mean_z', 'std_z', 'max_z', 'energy_z'
    ]
    Y = df[Y_cols].values

    # RandomForestRegressor 학습 및 저장
    print("RandomForestRegressor 학습 시작...")
    regr = MultiOutputRegressor(RandomForestRegressor(
        n_estimators=200,
        random_state=42,
        n_jobs=-1
    ))
    regr.fit(X, Y)
    joblib.dump(regr, REGR_MODEL_PATH)
    print(f"RandomForestRegressor 모델 저장 완료: {REGR_MODEL_PATH}")

    # LocalOutlierFactor 학습 및 저장
    print("LocalOutlierFactor 모델 학습 시작...")
    lof = LocalOutlierFactor(n_neighbors=20, novelty=True)
    lof.fit(Y) # LOF는 학습 데이터로 fit 해야 합니다.
    joblib.dump(lof, LOF_MODEL_PATH)
    print(f"LocalOutlierFactor 모델 저장 완료: {LOF_MODEL_PATH}")
    print("모든 모델 학습 및 저장 완료.")

if __name__ == "__main__":
    train_and_save_models()
