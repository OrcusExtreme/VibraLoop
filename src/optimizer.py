"""이상 진동(채터) 대응용 절삭 조건 제안 (edit_analysis2.md P11)

우선순위:
1. 안정 한계 모델 (models/sld_gpr_model.joblib, scripts/train_models.py가 ramp 시험으로 학습, P10)
   - 회전수마다 안전 깊이 ap_safe(n) = 평균 − κσ (sld_safe_depth). 학습한 공구(날 수)와 같은 공구일 때만 씁니다.
   - 이송은 지금보다 늘리지 않고, 회전수를 낮추면 날당 이송이 늘지 않게 같은 비율로 줄입니다(채터 주파수 규칙과 같음).
   - 재가공: 회전수 후보마다 패스 수 N = ceil(깊이 / ap_safe) (N ≤ MAX_PASSES, 하한 min_passes). 가공 시간(N / 이송)이
     가장 짧은(가이드의 argmax F/N) 회전수를 고르고, 같으면 현재 회전수에 가까운 쪽(그래도 같으면 높은 쪽).
   - 온라인 대응: 가공 중에는 깊이를 바꿀 수 없으므로 ap_safe ≥ 지금 깊이인 회전수 중 지금과 가장 가까운 것
     (같으면 높은 쪽)으로 회전수를 바꿉니다. 지금 회전수(±N_STEP)는 이미 채터가 났으므로 뺍니다.
   - 안전 깊이가 min_ap보다 얕은 회전수('수동 확인')와 시험하지 않은 회전수는 후보에서 뺍니다.
2. 채터 주파수 f_c (주파수 특징을 계산할 수 있는 센서일 때)
   - 날 통과 주파수가 f_c의 정수분의 1이 되도록 n = 60 × f_c / (날 수 × k) 중 현재와 가장 가까운 회전수
   - 센서 샘플링이 날 통과 주파수의 2배를 넘을 때만 씁니다. 1 kHz 센서는 16000–24000 rpm 2날의 날 통과
     주파수(533–800 Hz)보다 나이퀴스트(500 Hz)가 낮아, 보통 그보다 높은 채터 주파수가 접혀 다른 값으로 보입니다
     (edit_analysis2.md 2.3). 이때 측정값으로 회전수를 계산하면 엉뚱한 회전수가 나옵니다.
3. 기본 규칙 (학습 모델도 채터 주파수도 없을 때): 날당 이송을 줄이는 방향
   - 온라인 대응: 회전수 +10%(상한이면 -10%), 이송 -20%
   - 재가공: 회전수 유지, 이송 -20%, 깊이 2번 나눔

어느 경우에도 이송은 현재보다 늘리지 않습니다.
예전 채터 확률 모델(GPC, models/stability_model.joblib)은 P11에서 안정 한계 모델로 바꾸며 지웠습니다.
예전 RandomForest/LOF 모델(models/legacy/)은 현재 센서 특징과 단위가 달라 쓰지 않습니다 (edit_analysis.md 2.7).
"""
import logging
import math
import os

import joblib
import numpy as np

logger = logging.getLogger('optimizer')

N_RANGE = (16000, 24000)   # 탐색할 스핀들 회전수 범위 (rpm)
N_STEP = 250               # 회전수 후보 간격 (rpm)
MAX_PASSES = 5             # 재가공에서 깊이를 나누는 최대 횟수

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(BASE_DIR, "models")
SLD_MODEL_PATH = os.path.join(MODEL_DIR, "sld_gpr_model.joblib")  # ramp 시험으로 학습한 안정 한계 (P10)
SLD_MODEL_KIND = 'sld_gpr'


def chatter_frequency_usable(n, flutes, sample_rate):
    """측정한 채터 주파수로 회전수를 계산해도 되는지: 센서 샘플링이 날 통과 주파수(n × 날 수 / 60)의 2배를 넘어야 합니다."""
    return bool(n and flutes and sample_rate) and sample_rate > 2.0 * n * flutes / 60.0


def load_sld_model(path=SLD_MODEL_PATH):
    """ramp 시험으로 학습한 안정 한계 모델(scripts/train_models.py, edit_analysis2.md P10). 없거나 읽을 수 없으면 None."""
    if not os.path.exists(path):
        return None
    try:
        bundle = joblib.load(path)
        if not isinstance(bundle, dict) or bundle.get('kind') != SLD_MODEL_KIND:
            raise ValueError("형식이 맞지 않습니다")
        return bundle
    except Exception as e:
        logger.warning(f"안정 한계 모델을 읽을 수 없습니다 ({path}): {e}")
        return None


def sld_safe_depth(model, rpm):
    """회전수 rpm의 안정 한계 예측 [mm] → {'mu', 'sigma', 'ap_safe', 'manual_check'}. 시험한 회전수 범위 밖이면 None.

    입력은 날 주기 T = 60000/(n·Z) [ms]입니다(분석 2.5: 로브는 회전수가 아니라 날 주기에 대해 반복).
    평균(mu)은 시험한 최대 깊이를 넘지 않게 자릅니다. 그보다 깊게는 시험하지 않아 안정하다고 말할 근거가 없기 때문입니다.
    ap_safe = mu − κ·σ (κ 기본 1.645 = 한쪽 95%)이고 0 이상 시험한 최대 깊이 이하입니다.
    manual_check: ap_safe가 최소 깊이(min_ap)보다 얕아 깊이를 나눠도 쓰기 어려운 회전수 (최적화기 후보에서 뺌)
    """
    low, high = model['speed_range']
    if not rpm or not low <= rpm <= high:
        return None
    mean, std = model['model'].predict(np.array([[60000.0 / (rpm * model['flutes'])]]), return_std=True)
    mu = min(float(mean[0]), model['max_tested_ap'])
    sigma = float(std[0])
    ap_safe = float(np.clip(mu - model['kappa'] * sigma, 0.0, model['max_tested_ap']))
    return {'mu': mu, 'sigma': sigma, 'ap_safe': ap_safe, 'manual_check': ap_safe < model['min_ap']}


class CuttingConditionOptimizer:
    def __init__(self, model_path=SLD_MODEL_PATH):
        self.model_path = model_path
        self.sld = load_sld_model(model_path)

    def is_ready(self):
        """기본 규칙이 있으므로 항상 제안할 수 있습니다."""
        return True

    def describe(self):
        """모델 관리 창에 표시할 조건 제안 방법 (안정 한계 모델이 있으면 그 범위와 공구)"""
        if not self.sld:
            return {'sld': False, 'methods': ['chatter_frequency', 'fallback']}
        return {'sld': True, 'methods': ['sld', 'chatter_frequency', 'fallback'], 'runs': self.sld.get('n_runs'),
                'speed_range': self.sld.get('speed_range'), 'flutes': self.sld.get('flutes'),
                'tool_id': self.sld.get('tool_id'), 'kappa': self.sld.get('kappa'), 'trained_at': self.sld.get('trained_at')}

    # ------------------------------------------------------------------
    def suggest(self, n, F, ap=None, chatter_freq=None, flutes=None, purpose='rework', sample_rate=None, tool_id=None,
                min_passes=1):
        """현재 조건(n rpm, F mm/min, 깊이 ap mm)에서 바꿀 조건을 제안합니다.

        purpose: 'online'(가공 중 오버라이드로 바로 적용, 깊이는 못 바꿈) 또는 'rework'(재가공 파일 생성)
        flutes·tool_id: 지금 공구의 날 수와 공구 번호. 안정 한계 모델은 학습한 공구와 같을 때만 씁니다.
        sample_rate: 채터 주파수를 잰 센서의 샘플링 (Hz). 모르면 채터 주파수를 쓰지 않습니다.
        min_passes: 재가공 패스 수 하한(감지 설정 rework_min_passes). 안정 한계 모델은 이 하한을 넣어 가공 시간을 비교합니다.
        반환: {'n', 'F', 'passes', 'method', 'reason'} (안정 한계 모델이면 'ap_safe'도) 또는 None(현재 조건을 모를 때)
        """
        if not n or not F or n <= 0 or F <= 0:
            return None
        result, notes = None, []
        if self.sld is not None:
            result, note = self._from_sld(n, F, ap, flutes, tool_id, purpose, max(1, int(min_passes)))
            notes += [note] if note else []
        if result is None and chatter_freq and flutes:
            if chatter_frequency_usable(n, flutes, sample_rate):
                result = self._from_chatter_frequency(n, F, chatter_freq, flutes)
            else:
                notes.append(f"측정한 채터 주파수 {chatter_freq:.0f} Hz는 센서 샘플링"
                             f"({'모름' if not sample_rate else f'{sample_rate:.0f} Hz'})이 날 통과 주파수의 2배보다 낮아 "
                             "접혀 보인 값일 수 있어 쓰지 않음")
        if result is None:
            result = self._fallback(n, F, purpose)
        if notes and result['method'] != 'sld':
            result['reason'] += "; " + "; ".join(notes)
        if purpose == 'rework':
            result['passes'] = max(int(result['passes']), int(min_passes))
        result['n'] = float(round(result['n']))
        result['F'] = math.floor(result['F'] * 10 + 1e-6) / 10  # 0.1 mm/min 단위로 내림 (반올림으로 이송이 늘지 않게)
        return result

    def _from_sld(self, n, F, ap, flutes, tool_id, purpose, min_passes=1):
        """안정 한계 모델로 조건을 고릅니다 → (결과 또는 None, 쓰지 못한 이유 또는 None). 규칙은 모듈 설명 1번."""
        model = self.sld

        def feed(speed):  # 이송은 지금 이하, 회전수를 낮추면 날당 이송(F / (n × 날 수))이 늘지 않게 같은 비율로 줄임
            return F * min(1.0, speed / n)

        def feed_note(speed):
            return "이송은 그대로" if speed >= n else f"날당 이송이 늘지 않게 이송 {feed(speed):.1f} mm/min"

        if not ap or ap <= 0:
            return None, "지금 깊이를 몰라 안정 한계 모델을 쓰지 않음"
        if not flutes:
            return None, "공구 날 수를 몰라 안정 한계 모델을 쓰지 않음 (공구 정보 입력 필요)"
        if int(flutes) != model['flutes'] or (tool_id and model.get('tool_id') and str(tool_id) != str(model['tool_id'])):
            return None, (f"안정 한계 모델은 다른 공구({model.get('tool_id') or '공구 번호 없음'}, {model['flutes']}날)로 "
                          "학습되어 쓰지 않음")
        candidates = []
        for speed in np.arange(N_RANGE[0], N_RANGE[1] + N_STEP / 2, N_STEP, dtype=float):
            limit = sld_safe_depth(model, speed)
            if limit and not limit['manual_check']:
                candidates.append((speed, limit['ap_safe']))
        if purpose == 'online':
            usable = [(abs(speed - n), -speed, speed, safe) for speed, safe in candidates
                      if safe >= ap and abs(speed - n) >= N_STEP]
            if not usable:
                return None, f"안정 한계 모델에 지금 깊이 {ap:.2f} mm가 안전한 다른 회전수가 없음"
            _, _, speed, safe = min(usable)
            return ({'n': speed, 'F': feed(speed), 'passes': 1, 'method': 'sld', 'ap_safe': safe,
                     'reason': f"안정 한계 모델: 지금 깊이 {ap:.2f} mm가 안전한 가장 가까운 회전수 "
                               f"(안전 깊이 {safe:.3f} mm), {feed_note(speed)}"}, None)
        usable = []
        for speed, safe in candidates:
            needed = math.ceil(ap / safe - 1e-9)
            if needed <= MAX_PASSES:
                passes = max(needed, min_passes)
                # 가공 시간 ∝ 패스 수 / 이송 (반올림: 이송을 줄이지 않는 회전수끼리는 같은 값이 되도록)
                usable.append((round(passes * F / feed(speed), 9), abs(speed - n), -speed, speed, safe, passes))
        if not usable:
            return None, f"안정 한계 모델로는 깊이 {ap:.2f} mm를 {MAX_PASSES}번 안에 나눌 수 있는 회전수가 없음"
        _, _, _, speed, safe, passes = min(usable)
        return ({'n': speed, 'F': feed(speed), 'passes': passes, 'method': 'sld', 'ap_safe': safe,
                 'reason': f"안정 한계 모델: 안전 깊이 {safe:.3f} mm → 깊이 {ap:.2f} mm를 {passes}번으로 나눔"
                           f" (가공 시간이 가장 짧은 회전수), {feed_note(speed)}"}, None)

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
        return {'n': n_new, 'F': F_new, 'passes': 1, 'method': 'chatter_frequency',
                'reason': f"채터 주파수 {chatter_freq:.0f} Hz: 날 통과 주파수가 그 1/{k}이 되는 회전수"}

    def _fallback(self, n, F, purpose):
        if purpose == 'online':
            n_new = n * 1.1 if n * 1.1 <= N_RANGE[1] else n * 0.9
            return {'n': n_new, 'F': F * 0.8, 'passes': 1, 'method': 'fallback',
                    'reason': "안정 한계 모델과 쓸 수 있는 채터 주파수가 없어 기본 규칙 적용 (회전수 ±10%, 이송 -20%로 날당 이송 감소)"}
        return {'n': n, 'F': F * 0.8, 'passes': 2, 'method': 'fallback',
                'reason': "안정 한계 모델과 쓸 수 있는 채터 주파수가 없어 기본 규칙 적용 (이송 -20%, 깊이 2번 나눔)"}
