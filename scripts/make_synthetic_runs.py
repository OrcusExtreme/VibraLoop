"""정답을 아는 합성 가공 기록 만들기 (설계: edit_analysis2.md P01)

실제 데이터 수집과 같은 형식(src/data_recorder.py: manifest.csv, <run_id>_sensor.csv, <run_id>_machine.csv)으로
채터가 언제 시작했는지 아는 가공 기록을 만듭니다. 감지기를 고칠 때마다 같은 데이터로 전후를 비교하는 데 씁니다.

  python scripts/make_synthetic_runs.py                     # data/synthetic/raw 에 기본 시나리오 28개 생성
  python scripts/build_dataset.py --raw data/synthetic/raw --out data/synthetic/dataset
  python scripts/evaluate_detector.py --dataset data/synthetic/dataset
  python scripts/make_synthetic_runs.py --grid --out data/synthetic/grid/raw   # 조건 모델(P09)용 grid 시험 108개
  python scripts/make_synthetic_runs.py --ramp --out data/synthetic/ramp/raw   # 안정 한계(P10)용 ramp 시험 18개

시나리오 (manifest의 run_type)
  syn_stable   정상 가공, 조건 10가지                                   → 오경보
  syn_abrupt   급성장 채터: 진폭 10%→90%가 약 0.13초 (로지스틱, τ=30 ms) → 감지율·지연
  syn_gradual  완만한 채터: 5초 동안 선형으로 커짐                         → 감지율·지연
  syn_impulse  정상 가공 중 짧은 충격(약 15 ms) 4번                       → 오경보
  syn_entry    짧은 홈 3개: 진입 순간 큰 진동, 1.2초 동안 맞물림이 늘어남  → 오경보
채터 라벨 구간은 채터 진폭이 최종값의 10%를 넘는 시각부터 다시 10% 밑으로 내려가는 시각까지입니다.
채터 크기는 같은 실행의 정상 진동 RMS(X축)에 대한 채터 성분 RMS의 비율로 정합니다 (실행마다 2–5배).
실행별 정답(채터 주파수·비율·라벨 시각·충격 시각)은 출력 폴더의 truth.csv에 함께 저장합니다.

신호 모델 (MPU6050 1 kHz 출력 흉내, 수치는 근사라 절대 성능이 아니라 버전 간 비교용)
- 8배 빠르게 만든 뒤 DLPF(2차 버터워스, 기본 184 Hz)를 거쳐 1 kHz로 솎아 냄 → 500 Hz를 넘는 날 통과 성분은
  실제 센서처럼 접혀(앨리어싱) 들어옴. 실제 DLPF 특성은 공개되지 않아 근사입니다.
- 강제 진동: 회전 주파수 1–3차(런아웃) + 날 통과 주파수 1–4차, 크기는 절입·이송에 비례
- 광대역 절삭 잡음, 센서 잡음 0.004 g, Y축 중력 1 g
- 채터: 회전 고조파와 그 접힌 위치에서 15 Hz 이상 떨어진 150–480 Hz 성분
- --rpm-error: 실제 스핀들 회전수가 지령(가공 기록의 회전수)과 다른 경우 (예: 0.02 = 2% 빠름)

grid 시험 (--grid, run_type syn_grid, edit_analysis2.md P09)
- 실제 계획(scripts/make_experiment_nc.py grid)처럼 회전수 16000–24000(1000 간격) × 이송 50·100·150 × 깊이 0.1–0.6 mm의 홈 가공.
  실제 grid 기록과 같은 형식이라 build_dataset → train_models(조건 모델)를 그대로 거치고, 실데이터가 생기면 그 기록으로 바꾸면 됩니다.
- 신호 모델은 위와 같고, 실행마다 정상 진동 크기에 로그 정규 편차(RUN_VARIATION, 공구·고정 상태의 차이)를 곱합니다.
- 합성 안정 한계(synthetic_limit)보다 깊은 조건은 가공 도중 급성장 채터가 생깁니다(라벨 있음). truth.csv에 한계를 남깁니다.

ramp 시험 (--ramp, run_type ramp, edit_analysis2.md P10)
- 실제 계획(scripts/make_experiment_nc.py ramp)처럼 회전수마다 홈 40 mm를 이송 100으로 가며 깊이를 0 → RAMP_MAX로 늘립니다.
  회전수 16000–24000(1000 간격) × 2회 = 18개. 실제 ramp 기록과 같은 형식이라 build_dataset → train_models(안정 한계)를 거칩니다.
- 깊이가 그 실행의 안정 한계(synthetic_limit + 반복마다 작은 편차)를 넘으면 채터가 커지기 시작합니다.
  작업자 라벨은 그보다 1–2초 늦게 시작합니다(반응 지연). 한계가 RAMP_MAX 이상인 회전수는 끝까지 채터가 없습니다(중도절단).
- truth.csv에 실행의 한계 깊이, 채터가 실제로 시작한 시각, 라벨 지연을 남깁니다.
"""
import argparse
import csv
import os
import sys

import numpy as np
from scipy import signal

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

from data_recorder import RunRecorder  # noqa: E402
from sensor_protocol import Sample  # noqa: E402

OVERSAMPLE = 8                  # 내부 생성 배율 (1 kHz 출력이면 8 kHz로 만듦)
LEAD_SEC = 1.0                  # 가공 전후 공중 이동 시간 (초)
MACHINE_DT = 0.1                # 가공 상태 기록 간격 (초)
SENSOR_NOISE_G = 0.004
AXIS_GAIN = np.array([1.0, 0.6, 0.3])
CHATTER_GAIN = np.array([1.0, 0.8, 0.5])
IMPULSE_GAIN = np.array([1.0, 0.7, 0.5])
GRAVITY = np.array([0.0, 1.0, 0.0])
HOST_T0 = 1_800_000_000.0       # 합성 기록의 서버 시각 기준 (실행마다 1000초씩 뒤)
TOOL_D = 6.0
OWN_FILES = ('manifest.csv', 'truth.csv')

# (회전수, 이송, 절입[, 채터 RMS 비율])
SCENARIOS = {
    'stable': [(S, F, ap) for S in (16000, 18000, 20000, 22000, 24000) for F, ap in ((80, 0.2), (120, 0.4))],
    'abrupt': [(16000, 100, 0.5, 2.0), (18000, 100, 0.4, 2.5), (20000, 120, 0.5, 3.0), (22000, 80, 0.6, 3.5),
               (24000, 100, 0.5, 4.0), (18000, 120, 0.6, 5.0)],
    'gradual': [(17000, 100, 0.5, 2.0), (19000, 120, 0.4, 3.0), (21000, 100, 0.6, 4.0), (23000, 80, 0.5, 5.0)],
    'impulse': [(16000, 100, 0.3), (19000, 80, 0.4), (21000, 120, 0.3), (24000, 100, 0.4)],
    'entry': [(17000, 100, 0.3), (20000, 80, 0.5), (22000, 120, 0.4), (24000, 100, 0.3)],
}
QUICK = {'stable': (0, 5), 'abrupt': (4, 5), 'gradual': (3,), 'impulse': (0,), 'entry': (0,)}
GRID = {'S': tuple(range(16000, 24001, 1000)), 'F': (50, 100, 150), 'ap': (0.1, 0.25, 0.4, 0.6)}
GRID_QUICK = {'S': (16000, 20000, 24000), 'F': (50, 150), 'ap': (0.1, 0.4)}
GRID_CUT_SEC = 5.0
RUN_VARIATION = 0.08            # grid 실행마다 정상 진동 크기에 곱하는 로그 정규 편차 (공구·고정 상태의 차이)
GRID_CHATTER_RATIO = 3.0        # 안정 한계보다 깊은 grid 실행의 채터 크기 (정상 RMS 대비)
RAMP_S = tuple(range(16000, 24001, 1000))
RAMP_QUICK_S = (16000, 18000, 20000, 22000)
RAMP_REPEATS = 2
RAMP_LENGTH_MM, RAMP_FEED = 40.0, 100  # 실제 ramp 계획의 기본 홈 길이·이송 (24초)
RAMP_MAX = 0.55                 # 최대 깊이 (mm): 한계 0.6 mm인 회전수(16000·20000·24000)는 끝까지 안정 → 중도절단
RAMP_LIMIT_SPREAD = 0.015       # 반복 실행마다 안정 한계에 더하는 정규 편차 (mm)
RAMP_CHATTER_SPAN = 0.03        # 깊이가 한계를 이만큼(mm) 넘으면 채터가 최대 크기
LABEL_DELAY = (1.0, 2.0)        # 채터가 시작된 뒤 작업자가 라벨을 누르기까지 (초)


def scenario_plan(quick=False):
    """시나리오 목록. quick은 테스트용 축소판 (실행 7개, 가공 10초). run_id는 전체판과 같은 번호를 씁니다."""
    plan = []
    for kind, conditions in SCENARIOS.items():
        for k, cond in enumerate(conditions):
            if quick and k not in QUICK[kind]:
                continue
            S, F, ap = cond[:3]
            plan.append({'kind': kind, 'run_id': f"{kind}_{k + 1:02d}", 'S': S, 'F': F, 'ap': ap,
                         'ratio': cond[3] if len(cond) > 3 else None, 'cut_sec': 10.0 if quick else 20.0})
    return plan


def synthetic_limit(S):
    """합성 안정 한계 깊이 [mm]: 회전수에 따라 0.3–0.6 mm 사이를 4000 rpm 주기로 오르내림 (안정 로브 흉내)"""
    return 0.45 + 0.15 * np.cos(2 * np.pi * (S - 16000) / 4000.0)


def grid_plan(quick=False):
    """grid 시험 목록 (회전수 × 이송 × 깊이). 합성 안정 한계보다 깊으면 가공 도중 급성장 채터가 생깁니다."""
    grid = GRID_QUICK if quick else GRID
    plan = []
    conditions = [(S, F, ap) for S in grid['S'] for F in grid['F'] for ap in grid['ap']]
    for k, (S, F, ap) in enumerate(conditions):
        limit = float(synthetic_limit(S))
        chatter = ap > limit + 1e-9
        plan.append({'kind': 'abrupt' if chatter else 'stable', 'run_type': 'syn_grid', 'run_id': f"grid_{k + 1:03d}",
                     'S': S, 'F': F, 'ap': ap, 'ratio': GRID_CHATTER_RATIO if chatter else None,
                     'cut_sec': 3.0 if quick else GRID_CUT_SEC, 'limit': limit})
    return plan


def ramp_plan(quick=False):
    """ramp 시험 목록: 회전수마다 깊이 0 → RAMP_MAX (quick은 회전수 4개 × 1회, 홈 길이 절반)"""
    speeds, repeats = (RAMP_QUICK_S, 1) if quick else (RAMP_S, RAMP_REPEATS)
    cut_sec = (RAMP_LENGTH_MM / 2 if quick else RAMP_LENGTH_MM) / RAMP_FEED * 60.0
    plan = []
    for S in speeds:
        for _ in range(repeats):
            plan.append({'kind': 'ramp', 'run_type': 'ramp', 'run_id': f"ramp_{len(plan) + 1:03d}", 'S': S, 'F': RAMP_FEED,
                         'ap': RAMP_MAX, 'ratio': GRID_CHATTER_RATIO, 'cut_sec': cut_sec, 'limit': float(synthetic_limit(S))})
    return plan


def depth_at(spec, cuts, t):
    """시각 t(초, 배열 가능)의 절입 깊이: ramp는 가공 구간 동안 0 → ap로 선형 증가, 나머지는 가공 중 ap (밖은 0)"""
    t = np.asarray(t, float)
    depth = np.zeros_like(t)
    for c0, c1 in cuts:
        inside = (t >= c0) & (t < c1)
        depth[inside] = spec['ap'] * (t[inside] - c0) / (c1 - c0) if spec['kind'] == 'ramp' else spec['ap']
    return depth


def fold(freq, fs):
    """샘플링 fs에서 freq가 접혀 보이는 주파수 (0 ~ fs/2)"""
    return np.abs((np.asarray(freq, float) + fs / 2) % fs - fs / 2)


def pick_chatter_hz(rng, f_rot, fs, low=150.0, high=480.0, clearance=15.0):
    """회전 고조파(접힌 위치 포함)에서 clearance 이상 떨어진 채터 주파수"""
    harmonics = fold(f_rot * np.arange(1, 13), fs)
    candidates = [f for f in np.arange(low, high, 1.0) if np.min(np.abs(harmonics - f)) >= clearance]
    return float(rng.choice(candidates))


def cut_spans(kind, cut_sec):
    """가공(공구가 소재 안에 있는) 구간 목록 (실행 시작부터 초)"""
    if kind == 'entry':
        each = (cut_sec - 4.0) / 3  # 홈 3개, 사이 2초 공중 이동
        return [(LEAD_SEC + i * (each + 2.0), LEAD_SEC + i * (each + 2.0) + each) for i in range(3)]
    return [(LEAD_SEC, LEAD_SEC + cut_sec)]


def synthesize(spec, flutes, fs, dlpf_hz, rng, rpm_error=0.0):
    """실행 하나의 센서 신호와 정답 → (t, acc[N×3], 가공 구간, 정답 dict). 실제 회전수 = 지령 × (1 + rpm_error)"""
    kind, S, F, ap = spec['kind'], spec['S'], spec['F'], spec['ap']
    fs_int = fs * OVERSAMPLE
    cuts = cut_spans(kind, spec['cut_sec'])
    duration = cuts[-1][1] + LEAD_SEC
    t = np.arange(int(round(duration * fs_int))) / fs_int
    f_rot = S * (1.0 + rpm_error) / 60.0
    f_tooth = f_rot * flutes
    depth = depth_at(spec, cuts, t) if kind == 'ramp' else ap
    load = (depth / 0.4) * (F / 100.0)
    engaged = np.zeros_like(t)
    for c0, c1 in cuts:
        inside = (t >= c0) & (t < c1)
        engaged[inside] = 0.3 + 0.7 * np.minimum(1.0, (t[inside] - c0) / 1.2) if kind == 'entry' else 1.0
    b, a = signal.butter(2, dlpf_hz, fs=fs_int)

    def sensor(x):
        return signal.lfilter(b, a, x, axis=0)[::OVERSAMPLE]

    def gain(freq):
        return float(abs(signal.freqz(b, a, worN=[freq], fs=fs_int)[1][0]))

    phases = rng.uniform(0, 2 * np.pi, (7, 3))
    forced = np.zeros((len(t), 3))
    for axis in range(3):
        runout = sum(amp * np.sin(2 * np.pi * h * f_rot * t + phases[h - 1, axis])
                     for h, amp in ((1, 0.10), (2, 0.04), (3, 0.015)))
        teeth = sum(amp * np.sin(2 * np.pi * h * f_tooth * t + phases[2 + h, axis])
                    for h, amp in ((1, 0.30), (2, 0.15), (3, 0.08), (4, 0.04)))
        noise = rng.normal(0.0, 0.05, len(t))
        forced[:, axis] = AXIS_GAIN[axis] * (runout * (0.5 + load * engaged) + (teeth + noise) * load * engaged)
    acc = sensor(forced * spec.get('gain', 1.0))  # gain: grid 실행마다 다른 공구·고정 상태 (기본 시나리오는 1)
    t_out = t[::OVERSAMPLE]
    rms_x = float(np.std(acc[engaged[::OVERSAMPLE] >= 0.999, 0]))
    truth = {'rms_x_g': rms_x, 'label': None, 'chatter_hz': None, 'impulses': []}

    extra = np.zeros((len(t), 3))
    if kind in ('abrupt', 'gradual'):
        c0, c1 = cuts[0]
        span = c1 - c0
        f_c = pick_chatter_hz(rng, f_rot, fs)
        if kind == 'abrupt':
            tau = 0.03
            onset = c0 + rng.uniform(0.35, 0.5) * span           # 진폭 10% 시각
            rise = 1.0 / (1.0 + np.exp(-(t - onset - tau * np.log(9)) / tau))
            off = onset + min(4.0, 0.3 * span)
        else:
            ramp = min(5.0, 0.3 * span)
            start = c0 + rng.uniform(0.25, 0.35) * span
            rise = np.clip((t - start) / ramp, 0.0, 1.0)
            onset = start + 0.1 * ramp
            off = start + ramp + min(2.0, 0.1 * span)
        envelope = rise * np.where(t > off, np.exp(-(t - off) / 0.15), 1.0)
        amp = spec['ratio'] * rms_x * np.sqrt(2) / gain(f_c)
        phase = rng.uniform(0, 2 * np.pi, 3)
        for axis in range(3):
            extra[:, axis] += CHATTER_GAIN[axis] * amp * envelope * np.sin(2 * np.pi * f_c * t + phase[axis])
        truth.update(label=(onset, off + 0.15 * np.log(10)), chatter_hz=f_c)
    elif kind == 'ramp' and spec['limit_run'] < spec['ap']:
        # 깊이가 이 실행의 안정 한계를 넘는 순간부터 채터가 커짐 (RAMP_CHATTER_SPAN만큼 더 깊어지면 최대), 소재를 벗어나면 멈춤
        c0, c1 = cuts[0]
        f_c = pick_chatter_hz(rng, f_rot, fs)
        onset = c0 + spec['limit_run'] / spec['ap'] * (c1 - c0)
        envelope = np.clip((depth - spec['limit_run']) / RAMP_CHATTER_SPAN, 0.0, 1.0)
        amp = spec['ratio'] * rms_x * np.sqrt(2) / gain(f_c)
        phase = rng.uniform(0, 2 * np.pi, 3)
        for axis in range(3):
            extra[:, axis] += CHATTER_GAIN[axis] * amp * envelope * np.sin(2 * np.pi * f_c * t + phase[axis])
        truth.update(label=(onset + spec['label_delay'], c1), chatter_hz=f_c, onset_s=onset)
    elif kind == 'impulse':
        c0, c1 = cuts[0]
        edges = np.linspace(c0 + 3.0, c1 - 1.0, 5)
        times = [float(rng.uniform(e0 + 0.25 * (e1 - e0), e1 - 0.25 * (e1 - e0))) for e0, e1 in zip(edges[:-1], edges[1:])]
        amp = 8.0 * rms_x / gain(150.0)
        for ti in times:
            k = (t >= ti) & (t < ti + 0.03)
            burst = amp * np.exp(-(t[k] - ti) / 0.004) * np.sin(2 * np.pi * 150.0 * (t[k] - ti))
            extra[k] += burst[:, None] * IMPULSE_GAIN
        truth['impulses'] = times
    elif kind == 'entry':
        amp = 2.5 * rms_x * np.sqrt(2) / gain(f_rot)
        for c0, c1 in cuts:
            k = (t >= c0) & (t < c1)
            burst = amp * np.exp(-(t[k] - c0) / 0.15) * np.sin(2 * np.pi * f_rot * (t[k] - c0))
            extra[k] += burst[:, None] * AXIS_GAIN
    acc = acc + sensor(extra) + GRAVITY + rng.normal(0.0, SENSOR_NOISE_G, (len(t_out), 3))
    return t_out, acc, cuts, truth


def machine_rows(spec, cuts, duration, host0):
    """가공 상태 기록 (cnc_app.machine_snapshot과 같은 키). 공구가 소재 안에 있을 때만 Z가 음수입니다."""
    rows = []
    for mt in np.arange(0.0, duration + 1e-9, MACHINE_DT):
        cut = next(((c0, c1) for c0, c1 in cuts if c0 <= mt < c1), None)
        rows.append({'time': host0 + mt, 'state': 'running', 'grbl_state': 'Run' if mt < duration - 1e-9 else 'Idle',
                     'x': spec['F'] / 60.0 * (mt - cut[0]) if cut else 0.0, 'y': 0.0,
                     'z': -float(depth_at(spec, cuts, [mt])[0]) if cut else 5.0,
                     'n_cmd': float(spec['S']), 'spindle_override': 100, 'n_eff': float(spec['S']),
                     'F_cmd': float(spec['F']), 'feed_override': 100, 'F_eff': float(spec['F']),
                     'spindle_rt': float(spec['S']), 'feed_rt': float(spec['F']), 'line': 5})
    return rows


def prepare_out(out, force):
    if os.path.isdir(out) and os.listdir(out):
        names = os.listdir(out)
        ours = [n for n in names if n in OWN_FILES or n.startswith('manifest.csv.')
                or n.endswith(('_sensor.csv', '_machine.csv'))]
        if not force:
            sys.exit(f"{out}에 이미 기록이 있습니다. 지우고 다시 만들려면 --force를 붙이세요.")
        if len(ours) != len(names):
            sys.exit(f"{out}에 합성 기록이 아닌 파일이 있어 지우지 않습니다: {sorted(set(names) - set(ours))[:5]}")
        for name in ours:
            os.remove(os.path.join(out, name))
    os.makedirs(out, exist_ok=True)


def main():
    parser = argparse.ArgumentParser(description="정답을 아는 합성 가공 기록 생성 (감지기 평가용)")
    parser.add_argument('--out', default=os.path.join(BASE_DIR, 'data', 'synthetic', 'raw'))
    parser.add_argument('--seed', type=int, default=1, help="난수 시드 (같으면 같은 기록)")
    parser.add_argument('--quick', action='store_true', help="테스트용 축소판 (실행 7개, 가공 10초 / grid는 12개, 가공 3초)")
    parser.add_argument('--grid', action='store_true',
                        help="기본 시나리오 대신 조건 모델(P09)용 grid 시험 (회전수 9 × 이송 3 × 깊이 4 = 108개)")
    parser.add_argument('--ramp', action='store_true',
                        help="기본 시나리오 대신 안정 한계(P10)용 ramp 시험 (회전수 9 × 2회 = 18개, quick은 4개)")
    parser.add_argument('--fs', type=float, default=1000.0, help="센서 출력 샘플링 (Hz)")
    parser.add_argument('--dlpf', type=float, default=184.0, help="센서 DLPF 대역폭 (Hz, MPU6050: 184 또는 260)")
    parser.add_argument('--flutes', type=int, default=2, help="공구 날 수")
    parser.add_argument('--rpm-error', type=float, default=0.0,
                        help="실제 회전수와 지령의 차이 비율 (예: 0.02 = 2%% 빠름, 가공 기록에는 지령값이 남음)")
    parser.add_argument('--force', action='store_true', help="출력 폴더의 예전 합성 기록을 지우고 다시 생성")
    args = parser.parse_args()
    prepare_out(args.out, args.force)

    recorder = RunRecorder(args.out)
    truth_rows = []
    if args.grid and args.ramp:
        sys.exit("--grid와 --ramp는 함께 쓸 수 없습니다.")
    plan = grid_plan(args.quick) if args.grid else ramp_plan(args.quick) if args.ramp else scenario_plan(args.quick)
    for index, spec in enumerate(plan):
        rng = np.random.default_rng([args.seed, index])
        if args.grid or args.ramp:  # 기본 시나리오의 난수 순서를 바꾸지 않도록 따로 뽑음
            spec['gain'] = float(np.exp(np.random.default_rng([args.seed, index, 1]).normal(0.0, RUN_VARIATION)))
        if args.ramp:
            extra_rng = np.random.default_rng([args.seed, index, 2])
            spec['limit_run'] = spec['limit'] + float(extra_rng.normal(0.0, RAMP_LIMIT_SPREAD))
            spec['label_delay'] = float(extra_rng.uniform(*LABEL_DELAY))
        t, acc, cuts, truth = synthesize(spec, args.flutes, args.fs, args.dlpf, rng, args.rpm_error)
        host0 = HOST_T0 + 1000.0 * index
        meta = {'run_id': spec['run_id'], 'run_type': spec.get('run_type') or f"syn_{spec['kind']}", 'S_rpm': spec['S'],
                'F_mm_min': spec['F'],
                'ap_mm': spec['ap'], 'ae_mm': TOOL_D, 'tool_id': 'SYN', 'tool_diameter_mm': TOOL_D,
                'flutes': args.flutes, 'material': 'synthetic',
                'notes': f"synthetic seed={args.seed} dlpf={args.dlpf:g}Hz rpm_error={args.rpm_error:g}"}
        ok, message, run_id = recorder.start(meta, {'device': 'synthetic', 'fs': args.fs, 'range_g': 8})
        if not ok:
            sys.exit(message)
        recorder.add_sensor([Sample(host0 + ti, float(ti), i, float(x), float(y), float(z), None, None, None)
                             for i, (ti, (x, y, z)) in enumerate(zip(t, acc))])
        for row in machine_rows(spec, cuts, t[-1], host0):
            recorder.add_machine(row)
        if truth['label']:
            start, end = truth['label']
            x_at = (lambda s: spec['F'] / 60.0 * (s - cuts[0][0]))
            recorder.run['labels'].append({'label': 'chatter', 'start': host0 + start, 'end': host0 + end,
                                           'start_x': round(x_at(start), 3), 'end_x': round(x_at(end), 3),
                                           'note': f"synthetic {truth['chatter_hz']:.0f} Hz x{spec['ratio']:g}"})
        recorder.stop()
        truth_rows.append({
            'run_id': run_id, 'run_type': meta['run_type'], 'S_rpm': spec['S'], 'F_mm_min': spec['F'], 'ap_mm': spec['ap'],
            'actual_rpm': round(spec['S'] * (1.0 + args.rpm_error), 1),
            'flutes': args.flutes, 'dlpf_hz': args.dlpf, 'seed': args.seed, 'rms_x_g': round(truth['rms_x_g'], 4),
            'chatter_ratio': spec['ratio'] or '', 'chatter_hz': '' if truth['chatter_hz'] is None else round(truth['chatter_hz'], 1),
            'label_start_s': round(truth['label'][0], 3) if truth['label'] else '',
            'label_end_s': round(truth['label'][1], 3) if truth['label'] else '',
            'impulse_times_s': ' '.join(f"{x:.3f}" for x in truth['impulses']),
            'cuts_s': ' '.join(f"{c0:.2f}-{c1:.2f}" for c0, c1 in cuts),
        })
        if args.grid:
            truth_rows[-1].update(limit_mm=round(spec['limit'], 3), run_gain=round(spec['gain'], 4))
        if args.ramp:  # limit_mm: 이 실행의 한계(반복 편차 포함), limit_curve_mm: 회전수별 한계 곡선 (안정 한계 모델의 정답)
            truth_rows[-1].update(limit_mm=round(spec['limit_run'], 4), limit_curve_mm=round(spec['limit'], 4),
                                  censored=int(spec['limit_run'] >= spec['ap']),
                                  onset_s=round(truth['onset_s'], 3) if 'onset_s' in truth else '',
                                  label_delay_s=round(spec['label_delay'], 3), run_gain=round(spec['gain'], 4))
        detail = (f", 채터 {spec['ratio']:g}배 @ {truth['chatter_hz']:.0f} Hz ({truth['label'][0]:.2f}–{truth['label'][1]:.2f}초)"
                  if truth['label'] else (f", 충격 {len(truth['impulses'])}번" if truth['impulses'] else ''))
        print(f"  {run_id}: {spec['S']} rpm, F{spec['F']}, ap {spec['ap']} mm, 정상 RMS {truth['rms_x_g']:.3f} g{detail}")

    with open(os.path.join(args.out, 'truth.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(truth_rows[0]))
        writer.writeheader()
        writer.writerows(truth_rows)
    print(f"합성 기록 {len(truth_rows)}개 저장: {args.out} (정답: truth.csv)")


if __name__ == '__main__':
    main()
