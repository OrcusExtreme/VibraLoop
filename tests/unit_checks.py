"""Unit/integration checks: features, sensor protocol, optimizer, rework, decision rules, monitor (fake controllers),
data recorder, experiment NC generator, synthetic runs -> dataset -> train -> evaluate.

Runs on a throw-away copy of the project made by run_tests.py:
  python tests/run_tests.py unit
"""
import csv
import filecmp
import glob
import json
import logging
import os
import subprocess
import sys
import threading
import time
import warnings
from collections import defaultdict

import numpy as np
import pandas as pd

if len(sys.argv) < 2 or not os.path.isdir(sys.argv[1]):
    sys.exit("usage: python tests/run_tests.py unit  (this script needs a project copy as its argument)")
warnings.filterwarnings("ignore")
logging.disable(logging.CRITICAL)
COPY = os.path.abspath(sys.argv[1])
sys.path.insert(0, os.path.join(COPY, "src"))

import features  # noqa: E402
import sensor_protocol  # noqa: E402
import optimizer  # noqa: E402
import rework  # noqa: E402
import data_recorder  # noqa: E402
import vibration_monitor as vm  # noqa: E402
from gcode_utils import ModalState, parse_words  # noqa: E402

results = []


def check(name, cond, detail=""):
    results.append(bool(cond))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""), flush=True)


def wait_for(pred, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


rng = np.random.default_rng(1)
cfg = features.feature_config()

print("== features ==")
t = np.arange(1000) / 1000.0
sig = np.column_stack([0.3 * np.sin(2 * np.pi * 50 * t) + 0.9, np.zeros_like(t) - 1.0, rng.normal(0, 1e-4, len(t))])
f = features.time_features(sig, 8.0)
check("AC RMS ignores gravity/DC (0.3 sine -> 0.212 g on X)", abs(f['rms_x'] - 0.3 / np.sqrt(2)) < 1e-3, f"{f['rms_x']:.4f}")
check("sine kurtosis 1.5, crest sqrt(2)", abs(f['kurt_x'] - 1.5) < 0.01 and abs(f['crest_x'] - np.sqrt(2)) < 0.01)
fs = 2000.0
t2 = np.arange(2000) / fs
rot = 18000 / 60.0
base = sum(0.1 / h * np.sin(2 * np.pi * h * rot * t2) for h in range(1, 4))
stable = np.column_stack([base, base * 0.5, base * 0.2]) + rng.normal(0, 0.005, (len(t2), 3))
chatter = stable + np.column_stack([0.4 * np.sin(2 * np.pi * 734 * t2)] * 3)
fs_stable = features.compute_features(stable, t2, cfg, spindle_rpm=18000)
fs_chatter = features.compute_features(chatter, t2, cfg, spindle_rpm=18000)
check("spectral features only with uniform, fast sampling", fs_stable['spectral'] and not features.compute_features(stable[::20], t2[::20], cfg)['spectral'])
check("stable cut: energy sits on spindle harmonics (non-harmonic ratio low)", fs_stable['nonharmonic_ratio'] < 0.2, f"{fs_stable['nonharmonic_ratio']:.2f}")
check("chatter: non-harmonic peak found at 734 Hz", fs_chatter['nonharmonic_ratio'] > 0.6 and abs(fs_chatter['peak_freq'] - 734) <= 2,
      f"ratio={fs_chatter['nonharmonic_ratio']:.2f} peak={fs_chatter['peak_freq']}")
tt = np.r_[np.arange(0, 2, 0.01), np.arange(2.5, 4, 0.01)]
ranges = features.window_ranges(tt, cfg)
usable = [features.window_is_usable(tt[a:b], cfg, 100.0) for a, b, _ in ranges]
check("windows spanning a 0.5 s gap are rejected", not all(usable) and any(usable))
mask = features.valid_sample_mask(np.array([[0, 0, 0], [9.9, 0, 0], [np.nan, 0, 0]]), 2.0)
check("out-of-range and NaN samples are dropped (not smoothed)", mask.tolist() == [True, False, False])
legacy = glob.glob(os.path.join(COPY, "data", "학습 데이터", "*13-23-07*.csv"))[0]
loaded = features.load_sensor_csv(legacy)
check("legacy browser-timestamped CSV loads with real (irregular) times", loaded['timing'] == 'browser' and 60 < loaded['fs'] < 90,
      f"fs={loaded['fs']:.1f}")
text = "DeviceTime,AccX,AccY,AccZ\n" + "".join(f"{i / 1000:.3f},0,1,0\n" for i in range(500)) + "".join(f"{i / 1000:.3f},0,1,0\n" for i in range(300))
loaded = features.load_sensor_csv(text)
check("device time reset splits segments", loaded['timing'] == 'device' and len(loaded['segments']) == 2)

print("== sensor protocol ==")
stream = sensor_protocol.SensorStream()
s, m = stream.parse_line("A,0,0,4096,0", 1.0)
check("fast samples before a header are held back with one notice", s == [] and "헤더" in m and stream.parse_line("A,1,0,4096,0", 1.0) == ([], None))
_, m = stream.parse_line("#H,device=MPU6050,fs=1000.0,range_g=8,dlpf_hz=184", 1.0)
check("header sets device/fs/range", stream.fs == 1000.0 and stream.range_g == 8.0 and "MPU6050" in m)
s1, _ = stream.parse_line("A,10,0,4096,-4096", 1.0)
s2, _ = stream.parse_line("A,11,409,4096,0", 1.0)
check("raw counts scaled to g for ±8 g (4096 = 1 g)", abs(s1[0].ay - 1.0) < 1e-9 and abs(s1[0].az + 1.0) < 1e-9 and abs(s2[0].ax - 0.0998535) < 1e-6)
check("device time advances 1/fs per sample", abs(s2[0].device_time - s1[0].device_time - 0.001) < 1e-12)
s3, _ = stream.parse_line("A,15,0,4096,0", 1.0)
check("seq gap counted as dropped and time jumps", stream.dropped == 3 and abs(s3[0].device_time - s2[0].device_time - 0.004) < 1e-12)
stream.parse_line("#O,100", 1.0)
s4, _ = stream.parse_line("A,100,0,4096,0", 1.0)
check("FIFO overflow: time skips the lost samples", stream.overflows == 1 and abs(s4[0].device_time - s3[0].device_time - 0.085) < 1e-9)
stream.parse_line("#H,device=MPU6050,fs=1010.0,range_g=8,dlpf_hz=184", 1.0)
s5, _ = stream.parse_line("A,101,0,4096,0", 1.0)
check("measured fs update does not make time jump", 0 < s5[0].device_time - s4[0].device_time < 0.0011)
legacy_stream = sensor_protocol.SensorStream()
s6, _ = legacy_stream.parse_line('{"seq":5,"t":1234,"accel":{"x":0.1,"y":0.98,"z":0.0}}', 2.0)
s7, _ = legacy_stream.parse_line('{"accel":{"x":0.1,"y":0.98,"z":0.0},"gyro":{"x":1,"y":2,"z":3}}', 2.0)
check("legacy JSON with seq/t uses device ms; without -> host time", s6[0].device_time == 1.234 and s7[0].device_time is None and s7[0].gx == 1.0)
check("unknown line is passed through as a message", legacy_stream.parse_line("MPU6050 초기화 중...", 2.0) == ([], "[센서] MPU6050 초기화 중..."))

print("== optimizer ==")
opt = optimizer.CuttingConditionOptimizer(model_path=os.path.join(COPY, "models", "none.joblib"))
online = opt.suggest(20000, 100, 0.3, purpose='online')
rew = opt.suggest(20000, 100, 0.3, purpose='rework')
check("fallback online: spindle +10%, feed -20%", online['method'] == 'fallback' and online['n'] == 22000 and online['F'] == 80)
check("fallback at the speed limit goes down instead", opt.suggest(23000, 100, purpose='online')['n'] == 20700)
check("fallback rework: same speed, feed -20%, depth in 2", rew['n'] == 20000 and rew['F'] == 80 and rew['passes'] == 2)
cf = opt.suggest(20000, 100, 0.3, chatter_freq=1500, flutes=2, purpose='online')
check("chatter frequency: tooth passing = f_c / k (1500 Hz, 2 flutes -> 22500 rpm), chip load not increased",
      cf['method'] == 'chatter_frequency' and cf['n'] == 22500 and cf['F'] == 100, str(cf))
check("unknown conditions -> no suggestion", opt.suggest(None, 100) is None)
from sklearn.gaussian_process import GaussianProcessClassifier  # noqa: E402
from sklearn.preprocessing import StandardScaler  # noqa: E402
X = np.array([[n, F, ap] for n in range(16000, 24001, 2000) for F in (50, 100, 150) for ap in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)], float)
limit = np.where(X[:, 0] >= 20000, 0.45, 0.25)
y = (X[:, 2] > limit).astype(int)
scaler = StandardScaler().fit(X)
gpc = GaussianProcessClassifier(random_state=0).fit(scaler.transform(X), y)
model_path = os.path.join(COPY, "models", "unit_stability.joblib")
optimizer.save_stability_model(gpc, ['n', 'F', 'ap'], path=model_path, scaler=scaler, p_max=0.2, n_runs=len(X))
opt_m = optimizer.CuttingConditionOptimizer(model_path=model_path)
sm = opt_m.suggest(17000, 100, 0.4, purpose='rework')
check("stability model: picks a safe (n, F, ap) without raising feed", sm['method'] == 'stability_model' and sm['F'] <= 100
      and sm['p_chatter'] < 0.2, str(sm))
check("stability model: moves to the stable high-speed region or splits depth", sm['n'] >= 20000 or sm['passes'] >= 2, str(sm))

print("== rework depth passes ==")
out = os.path.join(COPY, "nc", "unit_rework.nc")
program = ["G21", "G90", "M3 S20000", "G0 Z5", "G0 X1 Y2", "G1 Z-0.3 F100", "G1 X20 Z-0.3", "G91", "G1 Z-0.1", "G90", "G0 Z5", "M5"]
ok = rework.create_rework_nc(20000, 100, 22000, 80, 0, 0, 0, gcode_lines=program, tolerance_value=0.1, output_path=out,
                             depth_passes=3, note="unit (test)")
lines = [l.strip() for l in open(out, encoding="utf-8") if l.strip()]
plunges = [l for l in lines if l.startswith("G1 Z") and "F80" in l]
check("note comment first, parentheses kept safe", ok and lines[0] == "(unit [test])")
check("depth split -0.1 -> -0.2 -> -0.3, then tolerance pass -0.4", [l.split()[1] for l in plunges] == ["Z-0.100", "Z-0.200", "Z-0.300", "Z-0.400"], str(plunges))
check("safe heights untouched, relative (G91) Z not scaled", "G0 Z5" in lines and lines.count("G1 Z-0.1") == 4)
check("S/F replaced in every pass", all("S22000" in l for l in lines if l.startswith("M3 ")) and not any("F100" in l for l in lines)
      and sum(l.startswith("M3 ") for l in lines) == 4)

print("== decision rules ==")
dummy_model = {'feature_names': features.TIME_FEATURE_NAMES}
core = vm.DetectionCore(dummy_model, dict(vm.DETECTION_DEFAULTS))
decisions = [core.decide(e) for e in [1.2, 1.3, 0.5, 1.2, 1.2, 1.2, 1.2, 1.5, 0.9, 0.7, 0.7, 0.7, 0.7]]
check("k_of_n 4/4: start only after 4 consecutive exceedances", decisions.index('start') == 6, str(decisions))
check("hysteresis: release needs 4 windows below 0.8", decisions.index('end') == 12 and decisions[8] is None, str(decisions))
core2 = vm.DetectionCore(dummy_model, dict(vm.DETECTION_DEFAULTS, decision_mode='cusum'))
check("cusum: one very large window can trigger at once", core2.decide(4.0) == 'start')
core3 = vm.DetectionCore(dummy_model, dict(vm.DETECTION_DEFAULTS, decision_mode='cusum'))
check("cusum: small excess accumulates", [core3.decide(1.3) for _ in range(5)].count('start') == 1)

print("== monitor: gating, mitigation, escalation (fake controllers) ==")


class FakeSocket:
    def __init__(self):
        self.events = []

    def emit(self, name, data=None):
        self.events.append((name, data))


class FakeControllers:
    def __init__(self):
        self.snap = {'state': 'running', 'grbl_state': 'Run', 'z': -0.3, 'x': 1.0, 'y': 2.0, 'n_eff': 20000.0, 'F_eff': 100.0,
                     'n_cmd': 20000.0, 'F_cmd': 100.0, 'spindle_override': 100, 'feed_override': 100, 'spindle_rt': 20000.0,
                     'flutes': 2}
        self.overrides, self.stops = [], []
        self.ser = type('S', (), {'is_open': True})()
        self.state = 'running'
        self.vibration_detection_active = True
        self.rework_source_lines, self.gcode_lines = ["G1 Z-0.3 F100"], []
        self.tolerance_value = 0.0
        self.armed = False

    def machine_snapshot(self):
        return dict(self.snap, time=time.time())

    def apply_override(self, target, value):
        self.overrides.append((target, value))
        return {'status': 'success'}

    def safe_stop(self, reason=''):
        self.stops.append(reason)
        self.state = 'emergency'

    def stop_vibration_detection(self):
        self.vibration_detection_active = False

    def recover_after_reset(self):
        return True

    def arm_rework(self):
        self.armed = True

    def disarm_rework(self):
        self.armed = False


records, reworks = [], []
vm.append_detection_record = lambda *a, **k: records.append((a, k))
vm.rework.create_rework_nc = lambda *a, **k: reworks.append((a, k)) or True
sock, ctl = FakeSocket(), FakeControllers()
mon = vm.VibrationMonitor(sock, ctl)
normal_files = [f for f in glob.glob(os.path.join(COPY, "data", "학습 데이터", "*.csv")) if "이상" not in os.path.basename(f)]
check("monitor trains from data/학습 데이터 at start (normal files only)", mon.is_trained and len(mon.training_files) == len(normal_files),
      f"{len(mon.training_files)} of {len(normal_files)} files")
mon.is_running = True
normal = pd.read_csv(legacy, encoding="utf-8-sig")[["AccX", "AccY", "AccZ"]].to_numpy(float)
clock = [1000.0]
mon.clock = lambda: clock[0]
state = {'i': 0, 't': 50.0}


def feed(seconds, scale=1.0):
    """정상 데이터를 100 Hz로 공급. scale > 1이면 위치와 무관하게 큰 37 Hz 진동(채터 흉내)을 더함."""
    for _ in range(int(seconds * 100)):
        sample = normal[state['i'] % len(normal)]
        state['i'] += 1
        state['t'] += 0.01
        clock[0] += 0.01
        chatter_part = (scale - 1.0) * 0.5 * np.sin(2 * np.pi * 37 * state['t'])
        mon.process_sensor_data([(sample + chatter_part).tolist()], [state['t']])


feed(1.5)
check("first windows after entering the material are skipped (transient 0.75 s)", mon.latest.get('gate') is True and mon.latest.get('evidence') is None)
feed(0.5)
check("gate on: windows scored after the transient", mon.latest.get('gate') is True and mon.latest.get('evidence') is not None)
ctl.snap['z'] = 2.0
feed(0.6)
check("tool out of the material -> not scored (gate off)", mon.latest.get('gate') is False and mon.latest.get('evidence') is None)
ctl.snap['z'] = -0.3
feed(0.5)
check("re-entering the material starts a new cut (transient skipped)", mon.latest.get('evidence') is None)
feed(3.0)
check("baseline of the cut established from its first windows", mon.core.baseline is not None, f"baseline={mon.core.baseline}")
feed(2.0, scale=3.0)
check("chatter -> alarm -> online mitigation (no stop yet)", mon.event and mon.event['stage'] == 'mitigating' and not ctl.stops)
wait_for(lambda: len(ctl.overrides) == 2, 3)
check("mitigation overrides: spindle 110%, feed 80%", ctl.overrides == [('spindle', 110), ('feed', 80)], str(ctl.overrides))
check("detection logged as 'mitigate' with evidence", records and records[-1][0][-1] == 'mitigate' and records[-1][0][5] >= 1)
ctl.snap.update(n_eff=22000.0, F_eff=80.0, spindle_override=110, feed_override=80, spindle_rt=22000.0)
feed(5.0, scale=3.0)
check("still chattering after the wait -> escalate to position-keeping stop", ctl.stops and "온라인 대응 후에도" in ctl.stops[-1], str(ctl.stops))
wait_for(lambda: reworks, 5)
args, kwargs = reworks[-1] if reworks else ((), {})
check("rework uses the conditions at detection, fallback with 2 depth passes", reworks and kwargs.get('depth_passes') == 2
      and args[0] == 20000.0 and args[2] == 20000.0 and args[3] == 80.0 and ctl.armed, str(args[:4]))
check("status events carry live evidence", any(n == 'vibration_status' and d.get('evidence') is not None for n, d in sock.events))
check("get_status JSON-serialisable", json.dumps(mon.get_status()) is not None)
ok, msg = mon.update_settings({'mitigation_enabled': False})
check("settings saved to data/detector_settings.json", ok and json.load(open(vm.SETTINGS_PATH, encoding='utf-8'))['mitigation_enabled'] is False)
check("bad setting rejected", mon.update_settings({'k': 2})[0] is False)

print("== data recorder ==")
raw = os.path.join(COPY, "data", "raw_unit")
rec = data_recorder.RunRecorder(raw)
ok, msg, run_id = rec.start({'run_id': 'R/../1', 'S_rpm': 20000, 'flutes': 2, 'bogus': 'x'}, {'device': 'MPU6050', 'fs': 1000.0, 'range_g': 8})
check("run id sanitised", ok and run_id == "R_.._1")
rec.add_sensor([sensor_protocol.Sample(1.0 + i / 1000, i / 1000, i if i != 5 else 7, 0.0, 1.0, 0.0, None, None, None) for i in range(10)])
rec.add_machine({'time': 1.0, 'state': 'running', 'grbl_state': 'Run', 'x': 0.0, 'y': 0.0, 'z': -0.2, 'n_cmd': 20000.0,
                 'spindle_override': 100, 'n_eff': 20000.0, 'F_cmd': 100.0, 'feed_override': 100, 'F_eff': 100.0, 'line': 3})
check("label start/end and errors", rec.mark('chatter', 'start')[0] and not rec.mark('chatter', 'start')[0]
      and rec.mark('chatter', 'end')[0] and not rec.mark('bogus', 'start')[0])
rec.mark('air', 'start')
ok, msg, summary = rec.stop()
runs = data_recorder.read_manifest(raw)
check("manifest row with labels (open label closed at stop) and drop count", ok and runs[0]['run_id'] == 'R_.._1'
      and len(runs[0]['labels']) == 2 and runs[0]['sensor_dropped'] == '2' and runs[0]['S_rpm'] == '20000' and 'bogus' not in runs[0])
ok2, _, run_id2 = rec.start({'run_id': 'R/../1'})
rec.stop()
check("same run id is not overwritten", run_id2 == "R_.._1_1")

print("== experiment NC generator ==")
py = sys.executable
gen = os.path.join(COPY, "scripts", "make_experiment_nc.py")
out_dir = os.path.join(COPY, "nc", "experiments")
r1 = subprocess.run([py, gen, "ramp", "--tool-d", "6", "--flutes", "2", "--rpm", "16000:24000:2000"], capture_output=True, text=True, encoding="utf-8")
r2 = subprocess.run([py, gen, "grid", "--tool-d", "6", "--flutes", "2", "--rpm", "16000,20000", "--depths", "0.2,0.4",
                     "--feeds", "50,150", "--reference-every", "3"], capture_output=True, text=True, encoding="utf-8")
r3 = subprocess.run([py, gen, "grid", "--tool-d", "2", "--flutes", "2", "--depths", "1.5"], capture_output=True, text=True, encoding="utf-8")
r4 = subprocess.run([py, gen, "grid", "--name", "lim", "--tool-d", "6", "--flutes", "2", "--rpm", "16000,18000",
                     "--limits", "16000=0.5,18000=0.6", "--feeds", "100", "--reference", ""], capture_output=True, text=True, encoding="utf-8")
plan = pd.read_csv(os.path.join(out_dir, "grid", "plan.csv"), encoding="utf-8-sig")
check("ramp plan: one file per speed", r1.returncode == 0 and len(glob.glob(os.path.join(out_dir, "ramp", "ramp_*.nc"))) == 5, r1.stderr[-200:])
check("grid plan: 8 combinations + reference every 3 + final reference", r2.returncode == 0 and len(plan) == 8 + 3 + 1
      and (plan['type'] == 'reference').sum() == 4, r2.stderr[-200:])
check("slots do not overlap (Y pitch >= tool diameter)", plan.groupby('x0')['y'].apply(lambda s: s.sort_values().diff().dropna().min()).min() >= 6)
check("too-deep cut refused (tool breakage)", r3.returncode != 0 and "공구 파손" in r3.stderr, r3.stderr[-120:])
lim = pd.read_csv(os.path.join(out_dir, "lim", "plan.csv"), encoding="utf-8-sig")
check("--limits × factors set depths per speed", sorted(lim[lim['S_rpm'] == 16000]['ap_mm']) == [0.3, 0.45, 0.55, 0.7])
sys.path.insert(0, COPY)
import cnc_app  # noqa: E402
nc_text = open(os.path.join(out_dir, "ramp", "ramp_001.nc"), encoding="utf-8").read()
meta = cnc_app.parse_program_meta(nc_text)
check("META comment parsed (run_id, S, F, ap, flutes)", meta['run_id'] == 'ramp_001' and meta['S'] == '16000' and meta['flutes'] == '2' and meta['type'] == 'ramp')
res = cnc_app.load_nc_code(nc_text)
modal = ModalState()
moves = [modal.apply(l) for l in cnc_app.gcode_lines]
check("generated program loads (comment stripped) and ramps Z from 0 to -1.0", res['status'] == 'success'
      and not any('META' in l for l in cnc_app.gcode_lines) and modal.position[2] == 5.0
      and any(l.startswith("G1 X") and "Z-1.000" in l for l in cnc_app.gcode_lines))

print("== dataset -> train -> evaluate (synthetic recorded runs) ==")
raw_dir = os.path.join(COPY, "data", "raw")
recorder = data_recorder.RunRecorder(raw_dir)
fs = 1000.0
conditions = [(n, ap) for n in (16000, 18000, 20000, 22000, 24000) for ap in (0.2, 0.35, 0.5)]
for k, (n, ap) in enumerate(conditions):
    limit = 0.45 if n >= 20000 else 0.3
    chatter_run = ap > limit
    run_id = f"S{k:02d}"
    recorder.start({'run_id': run_id, 'run_type': 'grid', 'S_rpm': n, 'F_mm_min': 100, 'ap_mm': ap, 'ae_mm': 6, 'flutes': 2},
                   {'device': 'synthetic', 'fs': fs, 'range_g': 8})
    t0 = 1_800_000_000.0 + k * 100
    tt = np.arange(0, 12, 1 / fs)
    base = 0.05 * (ap / 0.2) * np.sin(2 * np.pi * n / 60 * tt)[:, None] * [1, 0.6, 0.3] + rng.normal(0, 0.01, (len(tt), 3))
    if chatter_run:
        on = (tt > 5) & (tt < 10)
        base[on] += 0.3 * np.sin(2 * np.pi * 437 * tt[on])[:, None] * [1, 0.8, 0.5]
    base[:, 1] += 1.0
    recorder.add_sensor([sensor_protocol.Sample(t0 + ti, ti, i, *row, None, None, None) for i, (ti, row) in enumerate(zip(tt, base))])
    for mt in np.arange(0, 12, 0.2):
        cutting = 1.0 <= mt <= 11.0
        recorder.add_machine({'time': t0 + mt, 'state': 'running', 'grbl_state': 'Run' if cutting else 'Idle',
                              'x': mt, 'y': 0.0, 'z': -ap if cutting else 5.0, 'n_cmd': float(n), 'spindle_override': 100,
                              'n_eff': float(n), 'F_cmd': 100.0, 'feed_override': 100, 'F_eff': 100.0,
                              'spindle_rt': float(n) if cutting else 0.0, 'feed_rt': 100.0, 'line': 5})
    run = recorder.run
    if chatter_run:
        run['labels'].append({'label': 'chatter', 'start': t0 + 5.0, 'end': t0 + 10.0, 'start_x': 5.0, 'end_x': 10.0, 'note': ''})
    recorder.stop()
bd = subprocess.run([py, os.path.join(COPY, "scripts", "build_dataset.py")], capture_output=True, text=True, encoding="utf-8")
win = pd.read_csv(os.path.join(COPY, "data", "dataset", "windows.csv"), encoding="utf-8-sig")
check("build_dataset: gated windows, chatter labels only in chatter runs, spectral features at 1 kHz", bd.returncode == 0
      and win['gate'].sum() > 300 and set(win[win['label'] == 'chatter']['run_id']) == {f"S{k:02d}" for k, (n, ap) in enumerate(conditions)
                                                                                         if ap > (0.45 if n >= 20000 else 0.3)}
      and win[win['gate'] == 1]['spectral'].astype(str).str.lower().eq('true').all(), (bd.stdout + bd.stderr)[-300:])
tm0 = subprocess.run([py, os.path.join(COPY, "scripts", "train_models.py"), "--skip-condition"], capture_output=True, text=True, encoding="utf-8")
check("15 runs + default p_max 5%: warns that no condition is below 5% (GP is conservative)", tm0.returncode == 0 and "⚠️" in tm0.stdout,
      tm0.stdout[-300:])
tm = subprocess.run([py, os.path.join(COPY, "scripts", "train_models.py"), "--p-max", "0.25"], capture_output=True, text=True, encoding="utf-8")
check("train_models: stability and condition models saved with CV report", tm.returncode == 0
      and os.path.isfile(os.path.join(COPY, "models", "stability_model.joblib"))
      and os.path.isfile(os.path.join(COPY, "models", "condition_model.joblib")) and "교차 검증" in tm.stdout, (tm.stdout + tm.stderr)[-400:])
print("    " + "\n    ".join(l for l in tm.stdout.splitlines() if l.strip()))
trained = optimizer.CuttingConditionOptimizer(model_path=os.path.join(COPY, "models", "stability_model.joblib"))
sug = trained.suggest(16000, 100, 0.5, purpose='rework')
check("trained optimizer avoids the chatter region (16000 rpm, 0.5 mm)", sug['method'] == 'stability_model'
      and (sug['n'] >= 19000 or sug['passes'] >= 2), str(sug))
ev = subprocess.run([py, os.path.join(COPY, "scripts", "evaluate_detector.py")], capture_output=True, text=True, encoding="utf-8")
check("evaluate_detector: run-level CV detects synthetic chatter", ev.returncode == 0 and "채터 감지율" in ev.stdout, (ev.stdout + ev.stderr)[-300:])
print("    " + "\n    ".join(l for l in ev.stdout.splitlines() if "감지율" in l or "지연" in l or "오경보" in l))
ce = subprocess.run([py, os.path.join(COPY, "scripts", "check_env.py")], capture_output=True, text=True, encoding="utf-8")
check("check_env loads the trained models", "채터 확률 모델(models/stability_model.joblib) 로드·예측 정상" in ce.stdout
      and "조건 모델(models/condition_model.joblib) 로드·예측 정상" in ce.stdout, ce.stdout[-400:])
cm = vm.load_condition_model(os.path.join(COPY, "models", "condition_model.joblib"))
core_c = vm.DetectionCore(mon.model, dict(vm.DETECTION_DEFAULTS), condition_model=cm)
feats_loud = dict(features.compute_features(base, tt, cfg)[:0] if False else {}, **{name: 0.3 for name in features.TIME_FEATURE_NAMES}, rms=1.0, spectral=False)
_, parts, _ = core_c.score(feats_loud, {'n': 20000.0, 'F': 100.0, 'ap': 0.2})
check("condition model adds a 'condition' part when conditions are known", 'condition' in parts and parts['condition'] > 1, str(parts))

print("== evaluation base (P01): synthetic scenarios -> dataset -> evaluation from exact label times ==")
sys.path.insert(0, os.path.join(COPY, "scripts"))
import evaluate_detector as ed  # noqa: E402
score_rows = pd.DataFrame({'t_host': np.arange(0, 30, 0.25), 'gate': 1, 'label': 'stable'})
tally, trips = ed.Tally(), defaultdict(lambda: {'detections': 0, 'false_alarms': 0})
lat = ed.score_run(score_rows, [(10.25, 'k_of_n', None), (16.5, 'k_of_n', None), (20.0, 'cusum', None)], [[10.0, 15.0]],
                   (tally,), trips)
summary = tally.summary()
check("scoring: latency from the label start; alarms within 2 s of an event are not false, later ones are",
      lat == [0.25] and summary['detected'] == 1 and summary['false_alarms'] == 1 and trips['cusum']['false_alarms'] == 1
      and abs(summary['stable_minutes'] - 0.5) < 1e-9, str(summary))
syn_raw = os.path.join(COPY, "data", "synthetic", "raw")
syn_ds = os.path.join(COPY, "data", "synthetic", "dataset")
make_syn = os.path.join(COPY, "scripts", "make_synthetic_runs.py")
gen1 = subprocess.run([py, make_syn, "--quick", "--out", syn_raw], capture_output=True, text=True, encoding="utf-8")
syn_runs = data_recorder.read_manifest(syn_raw)
kinds = {r['run_type'] for r in syn_runs}
check("synthetic generator: recorder-format runs for all five scenarios", gen1.returncode == 0
      and kinds == {'syn_stable', 'syn_abrupt', 'syn_gradual', 'syn_impulse', 'syn_entry'}, (gen1.stdout + gen1.stderr)[-300:])
chatter_runs = [r for r in syn_runs if r['run_type'] in ('syn_abrupt', 'syn_gradual')]
check("only the chatter scenarios carry a label, one 'chatter' segment each",
      chatter_runs and all(len(r['labels']) == 1 and r['labels'][0]['label'] == 'chatter' for r in chatter_runs)
      and not any(r['labels'] for r in syn_runs if r not in chatter_runs))
stable16 = next(r for r in syn_runs if r['run_type'] == 'syn_stable' and r['S_rpm'] == '16000')
sd = pd.read_csv(os.path.join(syn_raw, stable16['sensor_file']))
xs = sd['AccX'].to_numpy()[((sd['DeviceTime'] > 2) & (sd['DeviceTime'] < 4)).to_numpy()]
power = np.abs(np.fft.rfft((xs - xs.mean()) * np.hanning(len(xs)))) ** 2
freqs = np.fft.rfftfreq(len(xs), 0.001)
share_467 = power[np.abs(freqs - 467) <= 3].sum() / power.sum()
check("1 kHz device time; tooth passing 533 Hz at 16000 rpm appears folded at 467 Hz (aliasing like the real sensor)",
      np.allclose(np.diff(sd['DeviceTime']), 0.001, atol=1e-6) and share_467 > 0.05, f"power share at 467 Hz = {share_467:.3f}")
gen2 = subprocess.run([py, make_syn, "--quick", "--out", syn_raw + "_again"], capture_output=True, text=True, encoding="utf-8")
check("same seed -> identical sensor data", gen2.returncode == 0 and filecmp.cmp(
    os.path.join(syn_raw, stable16['sensor_file']), os.path.join(syn_raw + "_again", stable16['sensor_file']), shallow=False))
gen3 = subprocess.run([py, make_syn, "--quick", "--out", syn_raw], capture_output=True, text=True, encoding="utf-8")
check("existing synthetic output is not overwritten without --force", gen3.returncode != 0 and "--force" in gen3.stderr,
      gen3.stderr[-150:])
bd2 = subprocess.run([py, os.path.join(COPY, "scripts", "build_dataset.py"), "--raw", syn_raw, "--out", syn_ds],
                     capture_output=True, text=True, encoding="utf-8")
labels_df = pd.read_csv(os.path.join(syn_ds, "labels.csv"), encoding="utf-8-sig") if bd2.returncode == 0 else None
check("build_dataset writes labels.csv with the original label times", labels_df is not None and len(labels_df) == len(chatter_runs)
      and all(abs(labels_df.set_index('run_id').loc[r['run_id'], 'start'] - r['labels'][0]['start']) < 1e-3 for r in chatter_runs),
      (bd2.stdout + bd2.stderr)[-300:])
syn_json = os.path.join(COPY, "data", "synthetic", "quick.json")
ev2 = subprocess.run([py, os.path.join(COPY, "scripts", "evaluate_detector.py"), "--dataset", syn_ds, "--json", syn_json],
                     capture_output=True, text=True, encoding="utf-8")
res = json.load(open(syn_json, encoding="utf-8")) if ev2.returncode == 0 else {}
check("evaluate_detector: events from label times, summaries per scenario and per alarm path saved as JSON",
      ev2.returncode == 0 and res['events_source'] == 'labels' and res['overall']['events'] == len(chatter_runs)
      and set(res['by_type']) == kinds and 'by_trip' in res and res['by_type']['syn_impulse']['stable_minutes'] > 0
      and res['dataset'] == 'data/synthetic/dataset', (ev2.stdout + ev2.stderr)[-300:])
ev3 = subprocess.run([py, os.path.join(COPY, "scripts", "evaluate_detector.py"), "--dataset", syn_ds, "--compare", syn_json],
                     capture_output=True, text=True, encoding="utf-8")
check("--compare prints the stored result side by side (same code -> no changed metric)", ev3.returncode == 0
      and "기준과 비교" in ev3.stdout and "  *" not in ev3.stdout.split("기준과 비교")[1], ev3.stdout[-300:])

print(f"\nunit checks: {sum(results)}/{len(results)} passed", flush=True)
os._exit(0 if all(results) else 1)
