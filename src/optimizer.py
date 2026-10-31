"""이상 진동(채터) 대응용 절삭 조건 제안

우선순위:
1. 채터 확률 모델 (models/stability_model.joblib, scripts/train_models.py로 라벨 데이터에서 학습)
   - 채터 확률이 p_max(기본 5%) 미만인 (회전수, 이송, 깊이) 조건 중 시간당 가공량(이송 / 패스 수)이 가장 큰 것
   - 현재 깊이로 안전한 조건이 없으면 깊이를 2·3번으로 나눈 경우를 찾습니다 (재가공일 때만)
2. 채터 주파수 f_c (주파수 특징을 계산할 수 있는 센서일 때)
   - 날 통과 주파수가 f_c의 정수분의 1이 되도록 n = 60 × f_c / (날 수 × k) 중 현재와 가장 가까운 회전수
   - 센서 샘플링이 날 통과 주파수의 2배를 넘을 때만 씁니다. 1 kHz 센서는 16000–24000 rpm 2날의 날 통과
     주파수(533–800 Hz)보다 나이퀴스트(500 Hz)가 낮아, 보통 그보다 높은 채터 주파수가 접혀 다른 값으로 보입니다
     (edit_analysis2.md 2.3). 이때 측정값으로 회전수를 계산하면 엉뚱한 회전수가 나옵니다.
3. 기본 규칙 (학습 모델도 채터 주파수도 없을 때): 날당 이송을 줄이는 방향
   - 온라인 대응: 회전수 +10%(상한이면 -10%), 이송 -20%
   - 재가공: 회전수 유지, 이송 -20%, 깊이 2번 나눔

어느 경우에도 이송은 현재보다 늘리지 않습니다 (allow_feed_increase=False).
예전 RandomForest/LOF 모델(models/legacy/)은 현재 센서 특징과 단위가 달라 쓰지 않습니다 (edit_analysis.md 2.7).
"""
import datetime
import logging
import os

import joblib
import numpy as np

logger = logging.getLogger('optimizer')

N_RANGE = (16000, 24000)   # 탐색할 스핀들 회전수 범위 (rpm)
F_RANGE = (50, 150)        # 탐색할 이송 속도 범위 (mm/min)
N_STEP, F_STEP = 250, 5
DEFAULT_P_MAX = 0.05

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(BASE_DIR, "models")
STABILITY_MODEL_PATH = os.path.join(MODEL_DIR, "stability_model.joblib")


def chatter_frequency_usable(n, flutes, sample_rate):
    """측정한 채터 주파수로 회전수를 계산해도 되는지: 센서 샘플링이 날 통과 주파수(n × 날 수 / 60)의 2배를 넘어야 합니다."""
    return bool(n and flutes and sample_rate) and sample_rate > 2.0 * n * flutes / 60.0


def load_stability_model(path=STABILITY_MODEL_PATH):
    """학습된 채터 확률 모델(dict)을 읽습니다. 없거나 읽을 수 없으면 None."""
    if not os.path.exists(path):
        return None
    try:
        bundle = joblib.load(path)
        if not isinstance(bundle, dict) or 'model' not in bundle or 'features' not in bundle:
            raise ValueError("형식이 맞지 않습니다")
        return bundle
    except Exception as e:
        logger.warning(f"채터 확률 모델을 읽을 수 없습니다 ({path}): {e}")
        return None


class CuttingConditionOptimizer:
    def __init__(self, model_path=STABILITY_MODEL_PATH, allow_feed_increase=False):
        self.model_path = model_path
        self.allow_feed_increase = allow_feed_increase
        self.bundle = load_stability_model(model_path)

    def is_ready(self):
        """기본 규칙이 있으므로 항상 제안할 수 있습니다."""
        return True

    def has_model(self):
        return self.bundle is not None

    def describe(self):
        if not self.bundle:
            return {'model': False, 'methods': ['chatter_frequency', 'fallback']}
        return {'model': True, 'methods': ['stability_model', 'chatter_frequency', 'fallback'],
                'p_max': self.bundle.get('p_max', DEFAULT_P_MAX), 'runs': self.bundle.get('n_runs'),
                'trained_at': self.bundle.get('trained_at')}

    # ------------------------------------------------------------------
    def suggest(self, n, F, ap=None, chatter_freq=None, flutes=None, purpose='rework', sample_rate=None):
        """현재 조건(n rpm, F mm/min, 깊이 ap mm)에서 바꿀 조건을 제안합니다.

        purpose: 'online'(가공 중 오버라이드로 바로 적용, 깊이는 못 바꿈) 또는 'rework'(재가공 파일 생성)
        sample_rate: 채터 주파수를 잰 센서의 샘플링 (Hz). 모르면 채터 주파수를 쓰지 않습니다.
        반환: {'n', 'F', 'passes', 'method', 'p_chatter', 'reason'} 또는 None(현재 조건을 모를 때)
        """
        if not n or not F or n <= 0 or F <= 0:
            return None
        max_passes = 3 if purpose == 'rework' else 1
        result, note = None, None
        if self.bundle is not None:
            result = self._from_model(n, F, ap, max_passes)
        if result is None and chatter_freq and flutes:
            if chatter_frequency_usable(n, flutes, sample_rate):
                result = self._from_chatter_frequency(n, F, chatter_freq, flutes)
            else:
                note = (f"측정한 채터 주파수 {chatter_freq:.0f} Hz는 센서 샘플링"
                        f"({'모름' if not sample_rate else f'{sample_rate:.0f} Hz'})이 날 통과 주파수의 2배보다 낮아 "
                        "접혀 보인 값일 수 있어 쓰지 않음")
        if result is None:
            result = self._fallback(n, F, purpose)
            if note:
                result['reason'] += f"; {note}"
        result['n'] = float(round(result['n']))
        result['F'] = float(round(result['F'], 1))
        return result

    def _feed_candidates(self, F):
        grid = np.arange(F_RANGE[0], F_RANGE[1] + F_STEP / 2, F_STEP, dtype=float)
        if not self.allow_feed_increase:
            grid = grid[grid <= F]
        return np.unique(np.append(grid, F))

    def _from_model(self, n, F, ap, max_passes):
        bundle = self.bundle
        features = bundle['features']
        if 'ap' in features and ap is None:
            return None
        model = bundle['model']
        classes = list(getattr(model, 'classes_', [0, 1]))
        if 1 not in classes:
            return None
        chatter_col = classes.index(1)
        p_max = float(bundle.get('p_max', DEFAULT_P_MAX))
        n_grid = np.arange(N_RANGE[0], N_RANGE[1] + N_STEP / 2, N_STEP, dtype=float)
        F_grid = self._feed_candidates(F)
        for passes in range(1, max_passes + 1):
            nn, ff = np.meshgrid(n_grid, F_grid, indexing='ij')
            columns = {'n': nn.ravel(), 'F': ff.ravel()}
            if 'ap' in features:
                columns['ap'] = np.full(nn.size, ap / passes)
            X = np.column_stack([columns[name] for name in features])
            if bundle.get('scaler') is not None:
                X = bundle['scaler'].transform(X)
            p = model.predict_proba(X)[:, chatter_col]
            feasible = np.flatnonzero(p < p_max)
            if len(feasible) == 0:
                continue
            throughput = columns['F'][feasible] / passes
            distance = np.abs(columns['n'][feasible] - n)
            order = np.lexsort((p[feasible], distance, -throughput))
            best = feasible[order[0]]
            return {'n': columns['n'][best], 'F': columns['F'][best], 'passes': passes, 'method': 'stability_model',
                    'p_chatter': float(p[best]),
                    'reason': f"채터 확률 모델: 예상 채터 확률 {p[best]:.1%} (기준 {p_max:.0%} 미만)"
                              + (f", 깊이를 {passes}번으로 나눔" if passes > 1 else "")}
        return None

    def _from_chatter_frequency(self, n, F, chatter_freq, flutes):
        candidates = []
        for k in range(1, 200):
            n_k = 60.0 * chatter_freq / (flutes * k)
            if n_k < N_RANGE[0]:
                break
            if n_k <= N_RANGE[1] and abs(n_k - n) / n >= 0.03:
                candidates.append((abs(n_k - n), n_k, k))
        if not candidates:
            return None
        _, n_new, k = min(candidates)
        F_new = F * min(1.0, n_new / n)  # 날당 이송(F / (n × 날 수))을 늘리지 않음
        return {'n': n_new, 'F': F_new, 'passes': 1, 'method': 'chatter_frequency', 'p_chatter': None,
                'reason': f"채터 주파수 {chatter_freq:.0f} Hz: 날 통과 주파수가 그 1/{k}이 되는 회전수"}

    def _fallback(self, n, F, purpose):
        if purpose == 'online':
            n_new = n * 1.1 if n * 1.1 <= N_RANGE[1] else n * 0.9
            return {'n': n_new, 'F': F * 0.8, 'passes': 1, 'method': 'fallback', 'p_chatter': None,
                    'reason': "학습된 채터 모델과 쓸 수 있는 채터 주파수가 없어 기본 규칙 적용 (회전수 ±10%, 이송 -20%로 날당 이송 감소)"}
        return {'n': n, 'F': F * 0.8, 'passes': 2, 'method': 'fallback', 'p_chatter': None,
                'reason': "학습된 채터 모델과 쓸 수 있는 채터 주파수가 없어 기본 규칙 적용 (이송 -20%, 깊이 2번 나눔)"}


def save_stability_model(model, features, path=STABILITY_MODEL_PATH, scaler=None, p_max=DEFAULT_P_MAX, **info):
    """scripts/train_models.py가 쓰는 저장 형식."""
    import sklearn
    bundle = {'model': model, 'scaler': scaler, 'features': list(features), 'p_max': p_max,
              'trained_at': datetime.datetime.now().isoformat(timespec='seconds'),
              'sklearn_version': sklearn.__version__}
    bundle.update(info)
    joblib.dump(bundle, path)
    return bundle
