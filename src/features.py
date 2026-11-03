"""진동 특징 계산: 학습, 실시간 감지, 데이터셋 생성이 모두 이 모듈만 사용합니다.

- 윈도우 길이와 간격은 샘플 개수가 아니라 시간(초)으로 정합니다 (샘플링이 달라도 같은 시간 축).
- 창 설정은 센서 샘플링으로 고릅니다 (WINDOW_PROFILES, edit_analysis2.md P04).
  fast_min_fs(400 Hz) 이상이면 'fast'(0.2초 창, 0.05초 간격, 끝 50 ms 블록 에너지), 아니면 'legacy'(1초, 0.25초).
  약 100 Hz의 기존 센서는 0.2초 창에 샘플이 18개 남짓이라 첨도·파고율이 의미가 없으므로 1초 창을 유지합니다.
- 모든 특징은 물리 단위(g) 신호에서 계산하고, 데이터에 따라 달라지는 정규화는 하지 않습니다.
- 윈도우마다 평균(중력·센서 자세 성분)을 빼고 진동 성분(AC)만 봅니다.
- 주파수 특징은 샘플링이 충분하고(spectral_min_fs 이상) 간격이 일정할 때만 계산합니다.
  약 100 Hz 샘플링에서는 날 통과·채터 주파수를 볼 수 없으므로 진동 크기 특징만 씁니다.
"""
import io
import math

import numpy as np
import pandas as pd

FEATURE_VERSION = 2

DEFAULT_FEATURE_CONFIG = {
    'window_sec': 1.0,             # 특징 윈도우 길이 (초) - 창 설정(profile_config)을 적용하면 그 값으로 바뀜
    'hop_sec': 0.25,               # 평가 간격 (초)
    'fast_min_fs': 400.0,          # 샘플링이 이 이상이면 고속 창 설정('fast')을 씀
    'min_fill': 0.6,               # 윈도우에 기대 샘플 수의 이 비율 이상이 있어야 사용
    'max_gap_sec': 0.25,           # 윈도우 안 샘플 간격이 이보다 길면 버림 (통신 끊김)
    'idle_rms_g': 0.02,            # 이보다 작은 진동은 가공하지 않는 구간으로 봄 (기계 상태가 없는 데이터용)
    'accel_range_g': 2.0,          # 센서 측정 범위 (포화 판단, 범위를 넘는 값은 통신 오류로 버림)
    'spectral_min_fs': 400.0,      # 주파수 특징을 계산할 최소 샘플링 (Hz)
    'harmonic_halfwidth_hz': 3.0,  # 회전 주파수 고조파로 보고 뺄 기본 폭 (± Hz, 주파수 칸 1.5개보다 좁으면 1.5칸)
    'harmonic_orders': 8,          # 회전 주파수 몇 차까지 뺄지 (나이퀴스트를 넘는 차수는 접혀 보이는 위치에서 뺌)
    'rotation_search_pct': 5.0,    # 실제 회전 주파수를 지령값 ±몇 % 안에서 스펙트럼으로 찾을지
    'rotation_tolerance_pct': 0.3,  # 찾은 회전 주파수의 남은 오차 (%) → 고조파 폭에 차수만큼 곱해 더함
    'rotation_min_prominence': 10.0,  # 회전 고조파 두 차수 이상이 스펙트럼 중앙값의 이 배수 이상일 때만 찾은 값을 씀
    'max_masked_fraction': 0.5,    # 고조파로 뺀 대역이 이 비율을 넘으면 비고조파 비율을 계산하지 않음
    'min_chatter_hz': 20.0,        # 이보다 낮은 주파수는 주파수 특징에서 제외 (자세 변화·이송 가감속)
}

# 창 설정: 센서 샘플링에 따라 하나를 고릅니다 (window_profile). block_sec는 윈도우 끝의 겹치지 않는 블록 길이로,
# 평가 간격과 같아서 이웃한 평가의 블록이 서로 겹치지 않습니다 (P06 Fast-Trip이 쓸 블록 에너지).
WINDOW_PROFILES = {
    'legacy': {'window_sec': 1.0, 'hop_sec': 0.25, 'block_sec': None},   # 기존 펌웨어 (약 100 Hz)
    'fast': {'window_sec': 0.2, 'hop_sec': 0.05, 'block_sec': 0.05},     # 고속 펌웨어 (1 kHz에서 창당 200샘플)
}

# 감지 모델(PCA) 입력: 진동 크기 특징 9개 (모두 로그 변환)
TIME_FEATURE_NAMES = ['rms_x', 'rms_y', 'rms_z', 'kurt_x', 'kurt_y', 'kurt_z', 'crest_x', 'crest_y', 'crest_z']
# 고속 창 설정에서 더하는 주파수 특징 (0–1 비율이라 로그 변환하지 않음). 학습 윈도우에 있을 때만 씁니다.
SPECTRAL_FEATURE_NAMES = ['spectral_entropy', 'nonharmonic_ratio']
_EPS = 1e-9


def feature_config(overrides=None):
    config = dict(DEFAULT_FEATURE_CONFIG)
    if overrides:
        config.update({k: v for k, v in overrides.items() if k in DEFAULT_FEATURE_CONFIG})
    return config


def window_profile(fs, config=None):
    """센서 샘플링(Hz) → 창 설정 이름 ('fast' 또는 'legacy'). 샘플링을 모르면 'legacy'."""
    config = config or DEFAULT_FEATURE_CONFIG
    return 'fast' if fs and fs >= config['fast_min_fs'] else 'legacy'


def profile_config(config, profile):
    """특징 설정에 창 설정(window_sec, hop_sec, block_sec)을 적용한 사본. 'profile' 키에 이름을 남깁니다."""
    return dict(config, profile=profile, **WINDOW_PROFILES[profile])


def valid_sample_mask(acc, range_g):
    """숫자가 아니거나 측정 범위를 크게 넘는 값(통신 오류)을 거르는 마스크. 진동 자체는 손대지 않습니다."""
    acc = np.asarray(acc, dtype=float)
    return np.isfinite(acc).all(axis=1) & (np.abs(acc) <= range_g * 1.05).all(axis=1)


def estimate_fs(t):
    """샘플 시각으로 평균 샘플링 주파수를 추정합니다."""
    t = np.asarray(t, dtype=float)
    if len(t) < 2 or t[-1] <= t[0]:
        return None
    return (len(t) - 1) / (t[-1] - t[0])


def is_uniform(t, tolerance=0.1):
    """샘플 간격이 일정한지 (장치 시각 기준 데이터). 주파수 특징은 이때만 계산합니다."""
    dt = np.diff(np.asarray(t, dtype=float))
    if len(dt) < 2 or dt.mean() <= 0:
        return False
    return dt.std() / dt.mean() < tolerance


def window_ranges(t, config):
    """시간 기준 윈도우의 (시작 인덱스, 끝 인덱스, 끝 시각) 목록. t는 오름차순."""
    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        return []
    window, hop = config['window_sec'], config['hop_sec']
    ranges = []
    end = t[0] + window
    while end <= t[-1] + 1e-9:
        i0 = int(np.searchsorted(t, end - window, side='left'))
        i1 = int(np.searchsorted(t, end, side='right'))
        ranges.append((i0, i1, end))
        end += hop
    return ranges


def window_is_usable(tw, config, nominal_fs):
    """샘플 수와 간격으로 윈도우를 쓸 수 있는지 판단합니다."""
    if len(tw) < 8:
        return False
    if nominal_fs and len(tw) < config['min_fill'] * config['window_sec'] * nominal_fs:
        return False
    return np.diff(tw).max(initial=0.0) <= config['max_gap_sec']


def time_features(acc, range_g):
    """진동 크기 특징 (축별 AC RMS, 첨도, 파고율, 포화 비율)."""
    acc = np.asarray(acc, dtype=float)
    ac = acc - acc.mean(axis=0)
    mean_square = (ac ** 2).mean(axis=0)
    rms = np.sqrt(mean_square)
    fourth = (ac ** 4).mean(axis=0)
    kurt = np.where(mean_square > 1e-12, fourth / np.maximum(mean_square ** 2, 1e-24), 3.0)
    crest = np.where(rms > 1e-6, np.abs(ac).max(axis=0) / np.maximum(rms, 1e-12), 1.0)
    feats = {'rms': float(np.sqrt(mean_square.mean())),
             'saturation': float((np.abs(acc) >= 0.98 * range_g).mean())}
    for i, axis in enumerate('xyz'):
        feats[f'rms_{axis}'] = float(rms[i])
        feats[f'kurt_{axis}'] = float(kurt[i])
        feats[f'crest_{axis}'] = float(crest[i])
        feats[f'mean_{axis}'] = float(acc[:, i].mean())
    return feats


def fold_frequency(freq, fs):
    """샘플링 fs에서 freq가 보이는 주파수 (0 ~ fs/2). 나이퀴스트(fs/2)를 넘는 성분은 접혀(앨리어싱) 보입니다."""
    return np.abs((np.asarray(freq, dtype=float) + fs / 2.0) % fs - fs / 2.0)


def estimate_rotation_hz(freqs, power, f_cmd, fs, config):
    """스펙트럼에서 실제 회전 주파수를 찾습니다 (지령값 ±rotation_search_pct %).

    후보마다 1–4차 고조파가 접혀 보이는 위치의 로그 크기를 더해 가장 큰 후보를 고릅니다. 여러 차수가 함께 맞아야
    하므로 큰 채터 봉우리 하나에 끌려가지 않습니다. 샘플링 오차도 모든 주파수를 같은 비율로 옮기므로 여기서 함께
    보정됩니다. 고른 후보에서 두 차수 이상(보통 1차 런아웃과 날 통과 차수)이 스펙트럼 중앙값의
    rotation_min_prominence배 이상으로 뚜렷하지 않으면 지령값을 씁니다.
    반환: (회전 주파수 Hz, 'spectrum' 또는 'command')
    """
    step = freqs[1] - freqs[0]
    span = f_cmd * config['rotation_search_pct'] / 100.0
    if span < step:
        return f_cmd, 'command'
    candidates = np.linspace(f_cmd - span, f_cmd + span, int(20 * span / step) + 1)  # 주파수 칸의 1/10 간격
    level = max(float(np.median(power)), _EPS)
    log_power = np.log(np.maximum(power, 0.1 * level))
    score = sum(np.interp(fold_frequency(order * candidates, fs), freqs, log_power) for order in range(1, 5))
    best = float(candidates[np.argmax(score)])
    peaks = np.interp(fold_frequency(best * np.arange(1, 5), fs), freqs, power)
    if (peaks >= config['rotation_min_prominence'] * level).sum() < 2:
        return f_cmd, 'command'
    return best, 'spectrum'


def spectral_features(acc, fs, config, spindle_rpm=None):
    """주파수 특징. 회전수를 알면 회전 주파수의 고조파를 뺀 나머지 에너지 비율과 그 봉우리를 계산합니다.

    채터는 회전·날 통과 주파수의 고조파가 아닌 곳에 큰 봉우리를 만듭니다. 1 kHz 센서에서는 날 통과 주파수
    (16000–24000 rpm 2날이면 533–800 Hz)가 나이퀴스트(500 Hz)를 넘어 다른 주파수로 접혀 보이므로, 고조파는
    접혀 보이는 위치에서 뺍니다. 회전 주파수는 지령값 근처에서 스펙트럼으로 다시 찾습니다 (실제 회전수·샘플링 오차).
    뺀 대역이 max_masked_fraction을 넘으면 남은 대역이 좁아 비고조파 비율과 봉우리를 계산하지 않습니다.
    """
    acc = np.asarray(acc, dtype=float)
    ac = acc - acc.mean(axis=0)
    n = len(ac)
    power = (np.abs(np.fft.rfft(ac * np.hanning(n)[:, None], axis=0)) ** 2).sum(axis=1)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    band = freqs >= config['min_chatter_hz']
    total = power[band].sum()
    if band.sum() < 4 or total <= 0:
        return {}
    p = power[band] / total
    feats = {'spectral_entropy': float(-(p * np.log(p + _EPS)).sum() / math.log(len(p)))}

    harmonic = np.zeros_like(band)
    if spindle_rpm and spindle_rpm > 0:
        f_rot, source = estimate_rotation_hz(freqs, power, spindle_rpm / 60.0, fs, config)
        halfwidth = max(config['harmonic_halfwidth_hz'], 1.5 * (freqs[1] - freqs[0]))
        tolerance = config['rotation_tolerance_pct'] / 100.0
        for order in range(1, int(config['harmonic_orders']) + 1):
            harmonic |= np.abs(freqs - fold_frequency(order * f_rot, fs)) <= halfwidth + tolerance * order * f_rot
        masked = float((band & harmonic).sum() / band.sum())
        feats.update(rotation_hz=round(f_rot, 2), rotation_source=source, masked_fraction=round(masked, 3))
        if masked > config['max_masked_fraction']:
            return feats
    candidates = band & ~harmonic
    if candidates.any():
        idx = np.flatnonzero(candidates)[np.argmax(power[candidates])]
        feats['peak_freq'] = float(freqs[idx])
        feats['peak_prominence'] = float(power[idx] / max(np.median(power[band]), _EPS))
    if spindle_rpm and spindle_rpm > 0:
        feats['nonharmonic_ratio'] = float(power[candidates].sum() / total)
    return feats


def block_rms(acc, t, block_sec):
    """윈도우 끝 block_sec 구간의 진동 크기 (AC RMS, 윈도우 평균을 뺌). 샘플이 4개 미만이면 None."""
    acc = np.asarray(acc, dtype=float)
    t = np.asarray(t, dtype=float)
    block = t > t[-1] - block_sec
    if block.sum() < 4:
        return None
    ac = acc[block] - acc.mean(axis=0)
    return float(np.sqrt((ac ** 2).mean()))


def compute_features(acc, t, config, spindle_rpm=None):
    """윈도우 하나의 특징 dict. 샘플링이 충분하고 간격이 일정하면 주파수 특징도 포함합니다.
    창 설정에 block_sec가 있으면 윈도우 끝 블록의 진동 크기(block_rms)도 계산합니다."""
    feats = time_features(acc, config['accel_range_g'])
    fs = estimate_fs(t)
    feats['fs'] = fs
    feats['n'] = len(acc)
    if config.get('block_sec'):
        feats['block_rms'] = block_rms(acc, t, config['block_sec'])
    feats['spectral'] = bool(fs and fs >= config['spectral_min_fs'] and is_uniform(t))
    if feats['spectral']:
        feats.update(spectral_features(acc, fs, config, spindle_rpm))
    return feats


def cutting_physics(n, F, ap, flutes, ae=None):
    """절삭 조건의 물리 변수 (edit_analysis2.md P09). 회전수·이송·깊이·날 수 중 하나라도 모르면 None.

    n: 회전수 [rpm], F: 이송 [mm/min], ap: 절입 깊이 [mm], flutes: 날 수 Z, ae: 절입 폭 [mm]
    fz = F / (n·Z) [mm/날], A_chip = ap·fz [mm², 날 하나가 깎는 칩 단면적]
    MRR = ap·ae·F [mm³/min]. ae를 모르면 ap·F [mm²/min]로 대신합니다.
    """
    try:
        n, F, ap, flutes = (float(v) for v in (n, F, ap, flutes))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) and v > 0 for v in (n, F, ap, flutes)):
        return None
    try:
        ae = float(ae)
    except (TypeError, ValueError):
        ae = math.nan
    fz = F / (n * flutes)
    return {'n': n, 'F': F, 'ap': ap, 'fz': fz, 'a_chip': ap * fz,
            'mrr': ap * (ae if math.isfinite(ae) and ae > 0 else 1.0) * F}


def feature_vector(feats, names=TIME_FEATURE_NAMES):
    """감지 모델 입력 벡터. 크기 특징은 분포가 한쪽으로 길어 로그 변환하고, 주파수 특징(0–1 비율)은 그대로 씁니다.
    윈도우에 없는 특징(예: 고조파 마스크가 넓어 계산하지 않은 비고조파 비율)은 NaN입니다."""
    values = []
    for name in names:
        value = feats.get(name)
        if value is None:
            values.append(np.nan)
        elif name in SPECTRAL_FEATURE_NAMES:
            values.append(float(value))
        else:
            values.append(math.log(max(float(value), 1e-6)))
    return np.array(values)


def session_windows(acc, t, config, spindle_rpm=None, active_only=True):
    """세션(연속 구간) 하나에서 쓸 수 있는 윈도우 특징 목록 [(끝 시각, 특징)]."""
    acc = np.asarray(acc, dtype=float)
    t = np.asarray(t, dtype=float)
    mask = valid_sample_mask(acc, config['accel_range_g'])
    acc, t = acc[mask], t[mask]
    nominal_fs = estimate_fs(t)
    windows = []
    for i0, i1, t_end in window_ranges(t, config):
        if not window_is_usable(t[i0:i1], config, nominal_fs):
            continue
        feats = compute_features(acc[i0:i1], t[i0:i1], config, spindle_rpm)
        if active_only and feats['rms'] < config['idle_rms_g']:
            continue
        windows.append((t_end, feats))
    return windows


def _split_segments(acc, t, max_gap=1.0):
    """시각이 거꾸로 가거나(장치 재시작) 오래 끊긴 곳에서 연속 구간을 나눕니다."""
    if len(t) == 0:
        return []
    breaks = np.flatnonzero((np.diff(t) < 0) | (np.diff(t) > max_gap)) + 1
    segments = []
    for a, b in zip(np.r_[0, breaks], np.r_[breaks, len(t)]):
        if b - a >= 2:
            seg_t = t[a:b]
            segments.append((acc[a:b], seg_t - seg_t[0]))
    return segments


def load_sensor_csv(source, default_fs=100.0):
    """센서 CSV를 읽습니다. source는 파일 경로 또는 CSV 텍스트.

    시각은 장치 시각(DeviceTime, 초) → 서버 수신 시각(HostTime) → 브라우저 시각(Timestamp, 예전 파일)
    → 없으면 default_fs 가정 순서로 씁니다.
    반환: {'segments': [(acc, t)], 'timing': ..., 'samples': N, 'fs': 추정 샘플링}
    열이 없으면 KeyError를 냅니다.
    """
    if isinstance(source, str) and '\n' in source:  # 경로에는 줄바꿈이 없으므로 CSV 텍스트로 판단
        df = pd.read_csv(io.StringIO(source))
    else:
        df = pd.read_csv(source, encoding='utf-8-sig')
    acc_df = df[['AccX', 'AccY', 'AccZ']].apply(pd.to_numeric, errors='coerce')

    timing = 'nominal'
    t = None
    for column, kind in (('DeviceTime', 'device'), ('HostTime', 'host')):
        if column in df.columns:
            values = pd.to_numeric(df[column], errors='coerce')
            if values.notna().mean() > 0.9:
                t, timing = values, kind
                break
    if t is None and 'Timestamp' in df.columns:
        stamps = pd.to_datetime(df['Timestamp'], utc=True, errors='coerce')
        if stamps.notna().mean() > 0.9:
            t, timing = (stamps - stamps.min()).dt.total_seconds(), 'browser'
    if t is None:
        t = pd.Series(np.arange(len(df)) / default_fs)

    valid = acc_df.notna().all(axis=1) & t.notna()
    acc = acc_df[valid].to_numpy(float)
    times = t[valid].to_numpy(float)
    if len(times):
        times = times - times[0]
    segments = _split_segments(acc, times)
    fs_values = [estimate_fs(seg_t) for _, seg_t in segments if estimate_fs(seg_t)]
    return {
        'segments': segments,
        'timing': timing,
        'samples': int(len(acc)),
        'fs': float(np.median(fs_values)) if fs_values else None,
    }
