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
# LOF 안전 임계값: lof_score(= -score_samples)는 정상일수록 1에 가깝고 이상할수록 커집니다.
# 모델의 offset_이 있으면 그 값(-offset_, contamination='auto'일 때 1.5)을 우선 사용합니다.
TAU = 1.5

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
        self.tau = TAU
        self._load_and_train_models() # 이제 _load_models로 변경될 것입니다.

    def is_ready(self):
        """회귀/LOF 모델이 모두 로드되었는지 여부"""
        return self.regr is not None and self.lof is not None

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

        # 후보 수~수십 개 단위 예측에서는 병렬 처리(n_jobs=-1) 준비 비용이 계산보다 커서 단일 스레드가 더 빠름
        for estimator in getattr(self.regr, "estimators_", []):
            if hasattr(estimator, "n_jobs"):
                estimator.n_jobs = 1

        # 학습된 LOF의 판정 경계(predict가 -1을 내는 기준)를 안전 임계값으로 사용
        offset = getattr(self.lof, "offset_", None)
        if offset is not None:
            self.tau = float(-offset)

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
        # LOF 점수가 임계값 이하여야 정상 진동 분포 안(안전)에 있는 조건입니다.
        return sc <= self.tau, sc

    def update_residual(self, realtime_stft_features, current_n, current_F):
        phi_now = realtime_stft_features
        c = {"n": current_n, "F": current_F}
        phi_hat = self.g_predict(c)
        self.r = phi_now - phi_hat
        print(f"잔차(r) 업데이트 완료: {self.r}")

    def time_proxy(self, F, S):
        # 가공 시간 = 경로 길이 / 이송 속도 F(mm/min)이므로 시간 대리변수는 1/F입니다.
        # (F = 날당 이송 × 날 수 × S 이므로 S는 F를 통해서만 시간에 영향을 주며,
        #  S 자체는 is_safe()의 LOF 안전성 판정에서 고려됩니다.)
        return 1.0 / F

    def clip_cand(self, n, F):
        n = float(np.clip(n, *N_RANGE))
        F = float(np.clip(F, *F_RANGE))
        return {"n": n, "F": F}

    def better(self, a, b):
        if b is None:
            return True
        # 가공 시간이 같으면 LOF 점수가 낮은(정상 분포에 더 가까운) 후보가 더 안전합니다.
        return (a["time"] < b["time"]) or (np.isclose(a["time"], b["time"]) and a["score"] < b["score"])

    def evaluate_candidates(self, cands):
        """후보 조건들을 한 번에 예측해 [(후보, 안전 여부, LOF 점수), ...]를 반환합니다.

        후보마다 따로 예측하면 RandomForest 호출 비용 때문에 전체 탐색(최대 214개 후보)에
        수십 초~100초가 걸려 재가공이 그만큼 늦어지므로 단계별로 묶어서 예측합니다.
        """
        X = np.array([[c["n"], c["F"]] for c in cands], float)
        ph_adj = self.regr.predict(X) + self.r
        scores = -self.lof.score_samples(ph_adj)
        return [(c, sc <= self.tau, float(sc)) for c, sc in zip(cands, scores)]

    def _best_of(self, evaluated, stage):
        best = None
        for cand, ok, sc in evaluated:
            if not ok:
                continue
            t = self.time_proxy(cand["F"], cand["n"])
            rec = {"cand": cand, "score": sc, "time": t, "stage": stage}
            if self.better(rec, best):
                best = rec
        return best

    def optimize_staged(self, c0):
        # 1·2단계는 변화폭(k)을 조금씩 키우다가 안전 후보가 처음 나온 k에서 멈춥니다.
        # (현재 조건에서 가장 적게 바꾼 안전 조건을 우선하는 휴리스틱)
        n0, F0 = c0["n"], c0["F"]

        # 1단계: ±3% 휴리스틱, 2단계: ±5% 휴리스틱 (k = 1..10, 후보는 k마다 n/F를 ± 조합한 4개)
        for unit, stage in ((0.03, "±3%"), (0.05, "±5%")):
            groups = [
                [self.clip_cand(n0 * (1 + sN * unit * k), F0 * (1 + sF * unit * k)) for sN in (-1, +1) for sF in (-1, +1)]
                for k in range(1, 11)
            ]
            evaluated = self.evaluate_candidates([cand for group in groups for cand in group])
            for i in range(len(groups)):
                best = self._best_of(evaluated[i * 4:(i + 1) * 4], stage)
                if best is not None:
                    return best

        # 3단계: ±10% 3x3, 4단계: ±20% 5x5, 5단계: 전체 도메인 10x10 그리드
        grids = (
            (np.linspace(n0*0.9, n0*1.1, 3), np.linspace(F0*0.9, F0*1.1, 3), "±10% 3x3"),
            (np.linspace(n0*0.8, n0*1.2, 5), np.linspace(F0*0.8, F0*1.2, 5), "±20% 5x5"),
            (np.linspace(N_RANGE[0], N_RANGE[1], 10), np.linspace(F_RANGE[0], F_RANGE[1], 10), "10x10"),
        )
        for n_vals, F_vals, stage in grids:
            best = self._best_of(self.evaluate_candidates([self.clip_cand(n, F) for n in n_vals for F in F_vals]), stage)
            if best is not None:
                return best

        return None

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
