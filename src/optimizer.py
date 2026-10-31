import pandas as pd
import numpy as np
# from sklearn.ensemble import RandomForestRegressor
# from sklearn.multioutput import MultiOutputRegressor
# from sklearn.neighbors import LocalOutlierFactor
import joblib
import os
import logging
from scipy.signal import stft

logger = logging.getLogger('optimizer')

# ---------------------------------------------------------
# 설정값
# ---------------------------------------------------------
# STFT_DATASET_PATH = "stft_dataset_3axis.csv" # 더 이상 필요하지 않습니다.
N_RANGE = (16000, 24000)
F_RANGE = (50, 150)
TAU = 1.0

# 모델 로드 경로 (train_models.py에서 저장된 경로와 일치해야 합니다)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(BASE_DIR, "models")
REGR_MODEL_PATH = os.path.join(MODEL_DIR, "regr_model.joblib")
LOF_MODEL_PATH = os.path.join(MODEL_DIR, "lof_model.joblib")

class CuttingConditionOptimizer:
    def __init__(self):
        self.regr = None
        self.lof = None
        self.r = None
        self._load_and_train_models() # 이제 _load_models로 변경될 것입니다.

    def _load_and_train_models(self):
        print(f"모델 로드: {REGR_MODEL_PATH}, {LOF_MODEL_PATH}")
        try:
            self.regr = joblib.load(REGR_MODEL_PATH)
            self.lof = joblib.load(LOF_MODEL_PATH)
            print("모델 로드 완료.")
        except FileNotFoundError:
            print("오류: 사전 학습된 모델 파일을 찾을 수 없습니다. `train_models.py`를 실행하여 모델을 먼저 학습시키세요.")
            self.regr = None
            self.lof = None
            return
        except Exception as e:
            print(f"모델 로드 중 오류 발생: {e}")
            self.regr = None
            self.lof = None
            return

        # 잔차(r) 초기화는 모델 로드 후, 예측에 필요한 Y의 특징 개수에 맞게 설정
        # 여기서는 12개 특징을 가정하고 0으로 초기화합니다.
        self.r = np.zeros(12) 

    def g_predict(self, c):
        if self.regr is None:
            raise RuntimeError("회귀 모델이 로드되지 않았습니다.")
        n = float(c["n"])
        F = float(c["F"])
        x = np.array([[n, F]], float)
        return self.regr.predict(x)[0]

    def lof_score(self, v):
        if self.lof is None:
            raise RuntimeError("LOF 모델이 로드되지 않았습니다.")
        v = np.asarray(v).reshape(1,-1)
        return -self.lof.score_samples(v)[0]

    def is_safe(self, c):
        ph = self.g_predict(c)
        ph_adj = ph + self.r
        sc = self.lof_score(ph_adj)
        return sc >= TAU, sc

    def update_residual(self, realtime_stft_features, current_n, current_F):
        phi_now = realtime_stft_features
        c = {"n": current_n, "F": current_F}
        phi_hat = self.g_predict(c)
        self.r = phi_now - phi_hat
        print(f"잔차(r) 업데이트 완료: {self.r}")

    def time_proxy(self, F, S):
        fz = F / S
        return 1.0 / (fz * S)

    def clip_cand(self, n, F):
        n = float(np.clip(n, *N_RANGE))
        F = float(np.clip(F, *F_RANGE))
        return {"n": n, "F": F}

    def better(self, a, b):
        if b is None:
            return True
        return (a["time"] < b["time"]) or (np.isclose(a["time"], b["time"]) and a["score"] > b["score"])

    def optimize_staged(self, c0):
        # ... (기존 optimize_staged 로직 유지) ...
        best = None
        n0, F0 = c0["n"], c0["F"]

        # 1단계: ±3% 휴리스틱
        for k in range(1, 11):
            step = 0.03 * k
            for sN in (-1, +1):
                for sF in (-1, +1):
                    n = n0 * (1 + sN * step)
                    F = F0 * (1 + sF * step)
                    cand = self.clip_cand(n, F)
                    ok, sc = self.is_safe(cand)
                    if not ok:
                        continue
                    t = self.time_proxy(cand["F"], cand["n"])
                    rec = {"cand": cand, "score": sc, "time": t, "stage": "±3%"}
                    if self.better(rec, best):
                        best = rec
            if best is not None:
                return best

        # 2단계: ±5%
        for k in range(1, 11):
            step = 0.05 * k
            for sN in (-1, +1):
                for sF in (-1, +1):
                    n = n0 * (1 + sN * step)
                    F = F0 * (1 + sF * step)
                    cand = self.clip_cand(n, F)
                    ok, sc = self.is_safe(cand)
                    if not ok:
                        continue
                    t = self.time_proxy(cand["F"], cand["n"])
                    rec = {"cand": cand, "score": sc, "time": t, "stage": "±5%"}
                    if self.better(rec, best):
                        best = rec
            if best is not None:
                return best

        # 3단계: ±10% 3x3 그리드
        n_vals = np.linspace(n0*0.9, n0*1.1, 3)
        F_vals = np.linspace(F0*0.9, F0*1.1, 3)
        for n in n_vals:
            for F in F_vals:
                cand = self.clip_cand(n, F)
                ok, sc = self.is_safe(cand)
                if not ok:
                    continue
                t = self.time_proxy(F, n)
                rec = {"cand": cand, "score": sc, "time": t, "stage": "±10% 3x3"}
                if self.better(rec, best):
                    best = rec
        if best is not None:
            return best

        # 4단계: ±20% 5x5 그리드
        n_vals = np.linspace(n0*0.8, n0*1.2, 5)
        F_vals = np.linspace(F0*0.8, F0*1.2, 5)
        for n in n_vals:
            for F in F_vals:
                cand = self.clip_cand(n, F)
                ok, sc = self.is_safe(cand)
                if not ok:
                    continue
                t = self.time_proxy(F, n)
                rec = {"cand": cand, "score": sc, "time": t, "stage": "±20% 5x5"}
                if self.better(rec, best):
                    best = rec
        if best is not None:
            return best

        # 5단계: 전체 도메인 10x10
        n_vals = np.linspace(N_RANGE[0], N_RANGE[1], 10)
        F_vals = np.linspace(F_RANGE[0], F_RANGE[1], 10)
        for n in n_vals:
            for F in F_vals:
                cand = self.clip_cand(n, F)
                ok, sc = self.is_safe(cand)
                if not ok:
                    continue
                t = self.time_proxy(F, n)
                rec = {"cand": cand, "score": sc, "time": t, "stage": "10x10"}
                if self.better(rec, best):
                    best = rec

        return best

if __name__ == "__main__":
    # 이 부분은 더 이상 모델 학습을 수행하지 않고, 로드된 모델을 사용한 테스트 로직으로 변경될 수 있습니다.
    optimizer = CuttingConditionOptimizer()
    if optimizer.regr and optimizer.lof:
        start = {"n": 18000, "F": 80} # 예시로 중간값 사용
        res = optimizer.optimize_staged(start)

        if res is None:
            print("⚠️ 안전 후보가 없습니다.")
        else:
            c = res["cand"]
            print(f"[{res['stage']}]")
            print(f"S = {c['n']:.0f} rpm, F = {c['F']:.1f} mm/min")
            print(f"LOF = {res['score']:.3f}, time ≈ {res['time']:.6f}")

    else:
        print("모델 로드에 실패하여 최적화 기능을 실행할 수 없습니다.")
