"""진동 특징 계산: 학습, 실시간 감지, 데이터셋 생성이 모두 이 모듈만 사용합니다.

- 윈도우 길이와 간격은 샘플 개수가 아니라 시간(초)으로 정합니다 (샘플링이 달라도 같은 시간 축).
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
    'window_sec': 1.0,             # 특징 윈도우 길이 (초)
    'hop_sec': 0.25,               # 평가 간격 (초)
    'min_fill': 0.6,               # 윈도우에 기대 샘플 수의 이 비율 이상이 있어야 사용
    'max_gap_sec': 0.25,           # 윈도우 안 샘플 간격이 이보다 길면 버림 (통신 끊김)
    'idle_rms_g': 0.02,            # 이보다 작은 진동은 가공하지 않는 구간으로 봄 (기계 상태가 없는 데이터용)
    'accel_range_g': 2.0,          # 센서 측정 범위 (포화 판단, 범위를 넘는 값은 통신 오류로 버림)
    'spectral_min_fs': 400.0,      # 주파수 특징을 계산할 최소 샘플링 (Hz)
    'harmonic_halfwidth_hz': 3.0,  # 회전 주파수 고조파로 보고 뺄 폭 (± Hz, 고조파 차수에 비례해 조금씩 넓힘)
    'min_chatter_hz': 20.0,        # 이보다 낮은 주파수는 주파수 특징에서 제외 (자세 변화·이송 가감속)
}

# LOF 입력 순서 (모두 로그 변환)
TIME_FEATURE_NAMES = ['rms_x', 'rms_y', 'rms_z', 'kurt_x', 'kurt_y', 'kurt_z', 'crest_x', 'crest_y', 'crest_z']
_EPS = 1e-9


def feature_config(overrides=None):
    config = dict(DEFAULT_FEATURE_CONFIG)
    if overrides:
        config.update({k: v for k, v in overrides.items() if k in DEFAULT_FEATURE_CONFIG})
    return config


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


def spectral_features(acc, fs, config, spindle_rpm=None):
    """주파수 특징. 회전수를 알면 회전 주파수의 고조파를 뺀 나머지 에너지 비율과 그 봉우리를 계산합니다.

    채터는 회전·날 통과 주파수의 고조파가 아닌 곳에 큰 봉우리를 만듭니다.
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
        f_rot = spindle_rpm / 60.0
        for order in range(1, int(freqs[-1] / f_rot) + 1):
            harmonic |= np.abs(freqs - order * f_rot) <= config['harmonic_halfwidth_hz'] + 0.005 * order * f_rot
    candidates = band & ~harmonic
    if candidates.any():
        idx = np.flatnonzero(candidates)[np.argmax(power[candidates])]
        feats['peak_freq'] = float(freqs[idx])
        feats['peak_prominence'] = float(power[idx] / max(np.median(power[band]), _EPS))
    if spindle_rpm and spindle_rpm > 0:
        feats['nonharmonic_ratio'] = float(power[candidates].sum() / total)
    return feats


def compute_features(acc, t, config, spindle_rpm=None):
    """윈도우 하나의 특징 dict. 샘플링이 충분하고 간격이 일정하면 주파수 특징도 포함합니다."""
    feats = time_features(acc, config['accel_range_g'])
    fs = estimate_fs(t)
    feats['fs'] = fs
    feats['n'] = len(acc)
    feats['spectral'] = bool(fs and fs >= config['spectral_min_fs'] and is_uniform(t))
    if feats['spectral']:
        feats.update(spectral_features(acc, fs, config, spindle_rpm))
    return feats


def feature_vector(feats, names=TIME_FEATURE_NAMES):
    """LOF 입력 벡터 (크기 특징은 분포가 한쪽으로 길어 로그 변환)."""
    return np.array([math.log(max(float(feats[name]), 1e-6)) for name in names])


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
