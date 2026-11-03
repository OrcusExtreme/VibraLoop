"""Unit/integration checks: features, sensor protocol, optimizer, rework, decision rules, monitor (fake controllers),
data recorder, experiment NC generator, synthetic runs -> dataset -> train -> evaluate.

Runs on a throw-away copy of the project made by run_tests.py:
  python tests/run_tests.py unit
"""
import contextlib
import csv
import filecmp
import glob
import io
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

print("== alias masking and rotation estimate (P03) ==")
from scipy import signal as sps  # noqa: E402


def sensor_signal(rpm, tone=None, rpm_error=0.0, fs_error=0.0, dlpf=260.0, seconds=1.0, seed=0):
    """1 kHz sensor output of spindle harmonics 1-6 made at 8 kHz -> DLPF -> decimation (folded like the real sensor).
    tone: optional (Hz, amplitude) non-harmonic component. rpm_error: spindle runs faster than commanded.
    fs_error: the device reports a sampling rate that is off by this fraction (timestamps use the reported rate)."""
    r = np.random.default_rng(seed)
    t8 = np.arange(int(seconds * 8000)) / 8000.0
    f_rot = rpm * (1 + rpm_error) / 60.0
    x = np.zeros((len(t8), 3))
    for k, amp in ((1, 0.10), (2, 0.15), (3, 0.04), (4, 0.06), (5, 0.02), (6, 0.03)):
        for axis, gain in enumerate((1.0, 0.6, 0.3)):
            x[:, axis] += amp * gain * np.sin(2 * np.pi * k * f_rot * t8 + r.uniform(0, 2 * np.pi))
    if tone:
        x += tone[1] * np.sin(2 * np.pi * tone[0] * t8)[:, None] * [1.0, 0.8, 0.5]
    b, a = sps.butter(2, dlpf, fs=8000.0)
    y = sps.lfilter(b, a, x, axis=0)[::8] + r.normal(0, 0.004, (len(t8) // 8, 3))
    return y, np.arange(len(y)) / (1000.0 * (1 + fs_error))


cfg8 = features.feature_config({'accel_range_g': 8.0})
worst, wrong_rotation = [], []
for rpm in (16000, 18000, 20000, 24000):
    for rpm_err, fs_err in ((0.0, 0.0), (0.02, 0.0), (-0.02, 0.0), (0.0, 0.02)):
        y, ty = sensor_signal(rpm, rpm_error=rpm_err, fs_error=fs_err, seed=rpm)
        f = features.compute_features(y, ty, cfg8, spindle_rpm=rpm)
        worst.append((f.get('nonharmonic_ratio', 1.0), rpm, rpm_err, fs_err))
        apparent = rpm * (1 + rpm_err) / 60.0 * (1 + fs_err)  # what the spectrum shows with the reported fs
        if f.get('rotation_source') != 'spectrum' or abs(f['rotation_hz'] - apparent) > 1.0:
            wrong_rotation.append((rpm, rpm_err, fs_err, f.get('rotation_hz'), round(apparent, 1)))
check("harmonics only (folded tooth passing, DLPF 260 Hz): non-harmonic ratio stays low at 16000-24000 rpm, "
      "also with 2% spindle-speed or sampling-rate error", max(worst)[0] < 0.2, f"worst {max(worst)}")
check("rotation frequency recovered from the spectrum (within 1 Hz of what the sensor shows)", not wrong_rotation,
      str(wrong_rotation))
y, ty = sensor_signal(18000, rpm_error=0.02)
legacy_cfg = dict(cfg8, harmonic_orders=1, rotation_search_pct=0.0)  # v1.1: commanded speed, 1st order only
legacy_ratio = features.compute_features(y, ty, legacy_cfg, spindle_rpm=18000)['nonharmonic_ratio']
check("v1.1 masking (commanded speed, no folded orders) would have flagged the same signal", legacy_ratio > 0.6,
      f"v1.1-style ratio {legacy_ratio:.2f}")
missed = []
for rpm, tone_hz in ((16000, 230.0), (18000, 250.0), (20000, 180.0), (24000, 300.0)):
    y, ty = sensor_signal(rpm, tone=(tone_hz, 0.3), rpm_error=0.02, seed=rpm + 1)
    f = features.compute_features(y, ty, cfg8, spindle_rpm=rpm)
    if not (f.get('nonharmonic_ratio', 0) > 0.6 and abs(f.get('peak_freq', 0) - tone_hz) <= 2.0):
        missed.append((rpm, tone_hz, f.get('nonharmonic_ratio'), f.get('peak_freq')))
check("non-harmonic tone (chatter) still found: ratio > 0.6 and peak at the tone", not missed, str(missed))
y, ty = sensor_signal(18000, tone=(311.0, 0.5), seed=7)
f = features.compute_features(y, ty, cfg8, spindle_rpm=18000)
check("a strong tone 11 Hz from the 1st order does not capture the rotation estimate (several orders must agree)",
      abs(f['rotation_hz'] - 300.0) < 1.0 and abs(f['peak_freq'] - 311.0) <= 2.0 and f['nonharmonic_ratio'] > 0.6,
      f"rotation {f['rotation_hz']} peak {f.get('peak_freq')} ratio {f.get('nonharmonic_ratio')}")
noise = np.random.default_rng(3).normal(0, 0.01, (1000, 3))
f_noise = features.compute_features(noise, np.arange(1000) / 1000.0, cfg8, spindle_rpm=18000)
f_wide = features.compute_features(*sensor_signal(16000), dict(cfg8, harmonic_orders=40, harmonic_halfwidth_hz=15.0),
                                   spindle_rpm=16000)
check("no visible rotation -> commanded speed; mask covering > 50% of the band -> no non-harmonic ratio",
      f_noise['rotation_source'] == 'command' and f_noise['rotation_hz'] == 300.0
      and f_wide['masked_fraction'] > 0.5 and 'nonharmonic_ratio' not in f_wide and 'peak_freq' not in f_wide,
      f"noise {f_noise.get('rotation_source')}, wide mask {f_wide.get('masked_fraction')}")

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
cf = opt.suggest(20000, 100, 0.3, chatter_freq=1500, flutes=2, purpose='online', sample_rate=4000)
check("chatter frequency: tooth passing = f_c / k (1500 Hz, 2 flutes -> 22500 rpm), chip load not increased",
      cf['method'] == 'chatter_frequency' and cf['n'] == 22500 and cf['F'] == 100, str(cf))
cf_1k = opt.suggest(20000, 100, 0.3, chatter_freq=437, flutes=2, purpose='online', sample_rate=1000)
cf_unknown = opt.suggest(20000, 100, 0.3, chatter_freq=437, flutes=2, purpose='online')
check("chatter frequency not used when the sensor cannot resolve tooth passing (1 kHz < 2 x 667 Hz) or fs is unknown",
      cf_1k['method'] == 'fallback' and "접혀" in cf_1k['reason'] and cf_unknown['method'] == 'fallback'
      and optimizer.chatter_frequency_usable(20000, 2, 1400) and not optimizer.chatter_frequency_usable(20000, 2, 1300),
      cf_1k['reason'])
check("unknown conditions -> no suggestion", opt.suggest(None, 100) is None)
check("without a stability limit model: chatter frequency, then the default rule",
      opt.describe() == {'sld': False, 'methods': ['chatter_frequency', 'fallback']})

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
check("mitigation overrides: feed 80% first, then spindle 110%", ctl.overrides == [('feed', 80), ('spindle', 110)], str(ctl.overrides))
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
check("no ramp tests in the dataset -> the stability limit model is skipped with a message", tm0.returncode == 0
      and "ramp_limits.csv" in tm0.stdout and "건너뜁니다" in tm0.stdout, tm0.stdout[-300:])
tm = subprocess.run([py, os.path.join(COPY, "scripts", "train_models.py")], capture_output=True, text=True, encoding="utf-8")
check("train_models: detector (P12) and condition models saved with CV reports; the old chatter-probability model is no longer "
      "made (P11)", tm.returncode == 0 and os.path.isfile(os.path.join(COPY, "models", "condition_model.joblib"))
      and os.path.isfile(os.path.join(COPY, "models", "detector_model.joblib")) and "[감지 모델] 저장" in tm.stdout
      and "교차 검증" in tm.stdout and not os.path.exists(os.path.join(COPY, "models", "stability_model.joblib"))
      and "채터 확률" not in tm.stdout, (tm.stdout + tm.stderr)[-400:])
print("    " + "\n    ".join(l for l in tm.stdout.splitlines() if l.strip()))
ev = subprocess.run([py, os.path.join(COPY, "scripts", "evaluate_detector.py")], capture_output=True, text=True, encoding="utf-8")
check("evaluate_detector: run-level CV detects synthetic chatter", ev.returncode == 0 and "채터 감지율" in ev.stdout, (ev.stdout + ev.stderr)[-300:])
print("    " + "\n    ".join(l for l in ev.stdout.splitlines() if "감지율" in l or "지연" in l or "오경보" in l))
ce = subprocess.run([py, os.path.join(COPY, "scripts", "check_env.py")], capture_output=True, text=True, encoding="utf-8")
check("check_env loads the trained detector and condition models and says the stability limit model is missing",
      "조건 모델(models/condition_model.joblib) 로드·예측 정상" in ce.stdout and "안정 한계 모델이 없어" in ce.stdout
      and "데이터셋 감지 모델(models/detector_model.joblib) 로드·채점 정상" in ce.stdout and "채터 확률" not in ce.stdout,
      ce.stdout[-400:])
# 뒤의 감지기 시험(P04–P11)은 학습 데이터 폴더로 학습한 모델을 전제로 하므로, 데이터셋 감지 모델은 P12 시험에서 다시 씀
detector_unit = os.path.join(COPY, "data", "detector_model_unit.joblib")
os.replace(os.path.join(COPY, "models", "detector_model.joblib"), detector_unit)
cm = vm.load_condition_model(os.path.join(COPY, "models", "condition_model.joblib"))
core_c = vm.DetectionCore(mon.model, dict(vm.DETECTION_DEFAULTS), condition_model=cm)
feats_loud = dict(features.compute_features(base, tt, cfg)[:0] if False else {}, **{name: 0.3 for name in features.TIME_FEATURE_NAMES}, rms=1.0, spectral=False)
_, parts, _ = core_c.score(feats_loud, {'n': 20000.0, 'F': 100.0, 'ap': 0.2, 'flutes': 2})
check("condition model adds a 'condition' part when conditions (incl. flutes) are known", 'condition' in parts and parts['condition'] > 1,
      str(parts))

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
check("evaluate_detector: events from label times, summaries per scenario, alarm path and evidence part saved as JSON",
      ev2.returncode == 0 and res['events_source'] == 'labels' and res['overall']['events'] == len(chatter_runs)
      and set(res['by_type']) == kinds and 'by_trip' in res and 'by_cause' in res
      and res['by_type']['syn_impulse']['stable_minutes'] > 0
      and res['dataset'] == 'data/synthetic/dataset', (ev2.stdout + ev2.stderr)[-300:])
ev3 = subprocess.run([py, os.path.join(COPY, "scripts", "evaluate_detector.py"), "--dataset", syn_ds, "--compare", syn_json],
                     capture_output=True, text=True, encoding="utf-8")
check("--compare prints the stored result side by side (same code -> no changed metric)", ev3.returncode == 0
      and "기준과 비교" in ev3.stdout and "  *" not in ev3.stdout.split("기준과 비교")[1], ev3.stdout[-300:])

print("== window settings per sensor (P04) ==")
fast_cfg = features.profile_config(cfg8, 'fast')
check("window setting from the sampling rate: ~90 Hz -> 1 s / 0.25 s, 1 kHz -> 0.2 s / 0.05 s with 50 ms blocks",
      features.window_profile(90.0) == 'legacy' and features.window_profile(1000.0) == 'fast'
      and features.window_profile(None) == 'legacy' and (fast_cfg['window_sec'], fast_cfg['hop_sec'], fast_cfg['block_sec']) == (0.2, 0.05, 0.05)
      and features.profile_config(fast_cfg, 'legacy')['window_sec'] == 1.0)
tg = np.arange(2000) / 1000.0
ranges_fast = features.window_ranges(tg, fast_cfg)
sizes = [b - a for a, b, _ in ranges_fast]
check("2 s at 1 kHz -> windows every 50 ms with 200-201 samples (same grid offline and live)",
      len(ranges_fast) == 36 and min(sizes) >= 200 and max(sizes) <= 201
      and np.allclose(np.diff([e for _, _, e in ranges_fast]), 0.05), f"{len(ranges_fast)} windows, samples {min(sizes)}-{max(sizes)}")
block_signal = np.zeros((200, 3))
block_signal[150:, 0] = np.sin(2 * np.pi * 100 * np.arange(50) / 1000.0)
fb = features.compute_features(block_signal, np.arange(200) / 1000.0, fast_cfg)
check("block energy = RMS of the last 50 ms only (window RMS is diluted by the quiet 150 ms before it)",
      abs(fb['block_rms'] - np.sqrt(0.5 / 3)) < 0.01 and fb['block_rms'] > 1.9 * fb['rms']
      and 'block_rms' not in features.compute_features(block_signal, np.arange(200) / 1000.0, cfg8),
      f"block {fb['block_rms']:.3f} window {fb['rms']:.3f}")
fast_model_stub = {'feature_names': features.TIME_FEATURE_NAMES, 'feature_config': fast_cfg}
fast_core = vm.DetectionCore(fast_model_stub, dict(vm.DETECTION_DEFAULTS))
fast_decisions = [fast_core.decide(1.2) for _ in range(16)]
check("time settings in seconds give the same times on the 0.05 s grid: 0.75 s -> 16/16 windows, release 16, baseline 40 (2 s), "
      "Fast-Trip 3 blocks",
      (fast_core.counts['k'], fast_core.counts['n'], fast_core.counts['release_windows'], fast_core.counts['baseline_windows'],
       fast_core.counts['fast_blocks']) == (16, 16, 16, 40, 3) and fast_decisions.index('start') == 15, str(fast_core.counts))
impulse_core = vm.DetectionCore(fast_model_stub, dict(vm.DETECTION_DEFAULTS))
check("one impulse (4 overlapping 0.2 s windows above threshold) does not trip on the 0.05 s grid",
      'start' not in [impulse_core.decide(e) for e in [2.0] * 4 + [0.3] * 20])
legacy_loaded = [features.load_sensor_csv(p) for p in sorted(glob.glob(os.path.join(COPY, "data", "학습 데이터", "*.csv")))
                 if "이상" not in os.path.basename(p)][:4]
fast_loaded = [features.load_sensor_csv(os.path.join(syn_raw, r['sensor_file'])) for r in syn_runs if r['run_type'] == 'syn_stable']
mixed = vm.train_detectors(legacy_loaded + fast_loaded)
check("mixed training data -> one model per window setting, each remembering its windows",
      set(mixed) == {'legacy', 'fast'} and mixed['legacy']['feature_config']['hop_sec'] == 0.25
      and mixed['fast']['feature_config']['window_sec'] == 0.2 and mixed['fast']['profile'] == 'fast'
      and vm.train_detector(legacy_loaded + fast_loaded)['profile'] == 'legacy',
      str({p: (m['sessions'], m['windows']) for p, m in mixed.items()}))
mon2 = vm.VibrationMonitor(FakeSocket(), FakeControllers())
mon2.set_sensor_info({'device': 'MPU6050', 'fs': 1000.0, 'range_g': 8})
st2 = mon2.get_status()['window']
check("1 kHz sensor without 1 kHz training data -> keeps the 1 s model, flagged as fallback and the operator is told",
      st2['stream_profile'] == 'fast' and st2['profile'] == 'legacy' and st2['fallback']
      and any(n == 'log_update' and '모델이 없어' in d.get('message', '') for n, d in mon2.socketio.events), str(st2))
mon2.train_model(legacy_loaded + fast_loaded)
status2 = mon2.get_status()
st2 = status2['window']
check("after adding 1 kHz normal data the 0.2 s / 0.05 s model is used; status lists both models; sensor range kept",
      st2['profile'] == 'fast' and not st2['fallback'] and st2['hop_sec'] == 0.05 and st2['k'] == 16
      and set(status2['profiles']) == {'legacy', 'fast'} and mon2.feature_config['accel_range_g'] == 8.0
      and json.dumps(status2) is not None, str(st2))
mon2.is_running = True
clock2 = [5000.0]
mon2.clock = lambda: clock2[0]
evaluated = []
real_evaluate = mon2._evaluate


def counting_evaluate(acc, t, now):
    evaluated.append((float(t[-1]), len(t)))
    real_evaluate(acc, t, now)


mon2._evaluate = counting_evaluate
stream_run = next(r for r in syn_runs if r['run_type'] == 'syn_stable')
# the fake controller must report the speed/feed the stream was recorded at (the spectrum masks the commanded speed)
mon2.controllers.snap.update(n_eff=float(stream_run['S_rpm']), n_cmd=float(stream_run['S_rpm']),
                             F_eff=float(stream_run['F_mm_min']), F_cmd=float(stream_run['F_mm_min']))
stream_df = pd.read_csv(os.path.join(syn_raw, stream_run['sensor_file']))
xyz, ts = stream_df[['AccX', 'AccY', 'AccZ']].to_numpy(), stream_df['DeviceTime'].to_numpy()
first = int(np.searchsorted(ts, 2.0))
spent = 0.0
for i in range(first, first + 3000, 20):  # 3 s of 1 kHz data in 20 ms batches
    clock2[0] += 0.02
    started = time.perf_counter()
    mon2.process_sensor_data(xyz[i:i + 20].tolist(), ts[i:i + 20].tolist())
    spent += time.perf_counter() - started
gaps = np.diff([e for e, _ in evaluated])
per_eval_ms = 1000.0 * spent / max(len(evaluated), 1)
check("1 kHz stream evaluated every 50 ms with 0.2 s windows; processing per evaluation well under the 50 ms budget",
      len(evaluated) >= 55 and np.allclose(gaps, 0.05, atol=0.0015) and min(n for _, n in evaluated) >= 199
      and mon2.latest.get('evidence') is not None and mon2.latest['evidence'] < 1.0 and per_eval_ms < 20,
      f"{len(evaluated)} evaluations, {per_eval_ms:.2f} ms each (incl. features, PCA, decision)")
mon2.set_sensor_info({'device': 'legacy-json'})  # 헤더 없는 센서: 받은 시각으로 샘플링을 추정
before_estimate = mon2.profile
for i in range(first + 3000, first + 4000, 20):
    clock2[0] += 0.02
    mon2.process_sensor_data(xyz[i:i + 20].tolist(), ts[i:i + 20].tolist())
check("no header: falls back to 1 s until the sampling rate is estimated from the timestamps, then 0.2 s",
      before_estimate == 'legacy' and mon2.stream_profile == 'fast' and mon2.profile == 'fast', f"{before_estimate} -> {mon2.profile}")

print("== rolling baseline (P05) ==")


class SelfOnlyCore(vm.DetectionCore):
    """Evidence = max('self' part, an external 'other' part) so that only the baseline logic is exercised."""

    def score(self, feats, cond=None):
        parts = {'other': feats.get('other', 0.0)}
        if self.baseline:
            parts['self'] = feats['rms'] / self.baseline / self.config['self_ratio']
        return max(parts.values()), parts, None


def run_core(core, rms_values, other=0.0):
    return [core.step({'rms': float(r), 'other': other}) for r in rms_values]


legacy_stub = {'feature_names': features.TIME_FEATURE_NAMES}  # 1 s windows every 0.25 s
core5 = SelfOnlyCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
steps = run_core(core5, [0.1] * 20 + [0.15] * 12)
first_self = next(i for i, s in enumerate(steps) if 'self' in s['parts'])
check("baseline starts after half of the 2 s range is filled (window 8, like v1.1) and skips the 3 overlapping windows",
      first_self == 7 and steps[23]['baseline'] == 0.1 and steps[31]['baseline'] == 0.15,
      f"first self at {first_self}, baseline at 23: {steps[23]['baseline']}, at 31: {steps[31]['baseline']}")
ramp = list(np.linspace(0.1, 0.35, 24))
core5 = SelfOnlyCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
steps = run_core(core5, [0.1] * 12 + ramp + [0.35] * 20)
fixed_self = max(r / 0.1 / 3.0 for r in ramp + [0.35])
check("slow legitimate increase x3.5 over 6 s (e.g. wider engagement): rolling baseline follows, no alarm "
      "(the v1.1 fixed cut-start baseline would have reached self 1.17)",
      not any(s['decision'] == 'start' for s in steps) and max(s['parts'].get('self', 0) for s in steps) < 0.8
      and fixed_self > 1.0 and abs(steps[-1]['baseline'] - 0.35) < 1e-9,
      f"max self {max(s['parts'].get('self', 0) for s in steps):.2f}, fixed would be {fixed_self:.2f}")
core5 = SelfOnlyCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
steps = run_core(core5, [0.15] * 12 + [0.04] * 12 + [0.15] * 12)
check("back to normal cutting after a quiet stretch is not a jump: the cut-start baseline covers the dip "
      "(rolling alone would have given self 1.25, as in 3 legacy files)",
      not any(s['decision'] == 'start' for s in steps) and max(s['parts'].get('self', 0) for s in steps[24:]) < 0.8
      and core5.anchor == 0.15, f"max self after the dip {max(s['parts'].get('self', 0) for s in steps[24:]):.2f}")
core5 = SelfOnlyCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
steps = run_core(core5, [0.1] * 12 + [0.5] * 40)
check("abrupt x5 jump: alarm after 4 windows and the baseline stays frozen for the whole 10 s anomaly",
      [s['decision'] for s in steps].index('start') == 15 and core5.alarm and steps[-1]['baseline'] == 0.1,
      f"start at {[s['decision'] for s in steps].index('start')}, baseline {steps[-1]['baseline']}")
core5 = SelfOnlyCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
steps = run_core(core5, [0.1] * 12) + run_core(core5, ramp + [0.35] * 8, other=0.85)
check("slow growth while another part already warns (>= 0.8, e.g. spectral): baseline frozen -> self part grows -> alarm",
      'start' in [s['decision'] for s in steps] and steps[-1]['baseline'] == 0.1)
core5 = SelfOnlyCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
run_core(core5, [0.1] * 12)
core5.condition_changed()
rebuilt = run_core(core5, [0.2] * 12)
core5b = SelfOnlyCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
run_core(core5b, [0.1] * 12 + [0.5] * 6)
core5b.condition_changed()  # online mitigation changed the overrides during the alarm
kept = run_core(core5b, [0.45] * 4)
check("speed/feed change without alarm -> baseline rebuilt at the new level; during an alarm (mitigation) it is kept",
      rebuilt[0]['baseline'] is None and rebuilt[-1]['baseline'] == 0.2 and core5b.alarm
      and all(s['baseline'] == 0.1 for s in kept), f"kept {[s['baseline'] for s in kept]}")
core5f = SelfOnlyCore(fast_model_stub, dict(vm.DETECTION_DEFAULTS))
steps = run_core(core5f, [0.1] * 60)
first_self = next(i for i, s in enumerate(steps) if 'self' in s['parts'])
check("0.05 s grid: same times (40-window = 2 s range, 4 overlapping windows skipped, starts after 1 s of normal windows)",
      core5f.counts['baseline_gap'] == 4 and core5f.counts['baseline_windows'] == 40 and first_self == 23,
      f"gap {core5f.counts['baseline_gap']}, range {core5f.counts['baseline_windows']}, first self at {first_self}")


class RecordingCore:
    def __init__(self):
        self.calls, self.config, self.alarm = [], {'decision_mode': 'k_of_n'}, False

    def new_cut(self):
        self.calls.append('new_cut')

    def condition_changed(self):
        self.calls.append('changed')

    def step(self, feats, cond=None):
        self.calls.append('step')
        return {'decision': None}


sim_rows = pd.DataFrame({'gate': [1, 1, 1, 1, 0], 'transient': [0, 0, 1, 0, 0], 'n': [20000, 20000, 20000, 20000, 0],
                         'F': [100, 100, 80, 80, 0], 'ap': 0.3, 't_host': range(5), 't_rel': range(5), 'rms': 0.1,
                         'spectral': False, **{name: 0.1 for name in features.TIME_FEATURE_NAMES}})
recording = RecordingCore()
ed.simulate_run(recording, sim_rows)
check("offline evaluation mirrors the live gate: feed change inside a cut -> condition_changed before the transient skip",
      recording.calls == ['new_cut', 'step', 'step', 'changed', 'step', 'new_cut'], str(recording.calls))

print("== two-tier decision: Fast-Trip + CUSUM, record-only mode (P06) ==")
cfg6 = dict(vm.DETECTION_DEFAULTS)
counts6 = vm.DetectionCore(fast_model_stub, cfg6).counts


def feed_rule(rule, evidences, fasts=None):
    return [rule.update(e, f) for e, f in zip(evidences, fasts or [None] * len(evidences))]


rule6 = vm.DecisionRule('two_tier', counts6, cfg6)
d6 = feed_rule(rule6, [0.5] * 5, [0.2, 1.2, 1.3, 1.4, 1.5])
check("Fast-Trip: last-50 ms block energy >= 3x baseline in 3 consecutive blocks -> alarm at the 3rd block (0.15 s)",
      d6.index('start') == 3 and rule6.trip == 'fast', str(d6))
rule6 = vm.DecisionRule('two_tier', counts6, cfg6)
check("one or two loud blocks never trip the fast tier",
      'start' not in feed_rule(rule6, [0.5] * 10, [0.2, 3.0, 0.3, 0.2, 2.0, 2.0, 0.3, 0.2, 0.2, 0.2]))
rule6 = vm.DecisionRule('two_tier', counts6, cfg6)
check("CUSUM tier caps evidence at 2: an impulse in 4 overlapping windows (evidence 5) does not trip",
      'start' not in feed_rule(rule6, [5.0] * 4 + [0.3] * 20))
rule6 = vm.DecisionRule('two_tier', counts6, cfg6)
d6 = feed_rule(rule6, [1.5] * 30)
rule6k = vm.DecisionRule('k_of_n', counts6, cfg6)
check("CUSUM tier: evidence 1.5 trips after ~0.74 s (15 windows); evidence 2 after ~0.45 s (k_of_n needs 0.75 s either way)",
      d6.index('start') == 14 and rule6.trip == 'cusum'
      and feed_rule(vm.DecisionRule('two_tier', counts6, cfg6), [2.0] * 20).index('start') == 8
      and feed_rule(rule6k, [2.0] * 20).index('start') == 15, str(d6.index('start')))
rule6 = vm.DecisionRule('two_tier', counts6, cfg6)
feed_rule(rule6, [0.5] * 3, [1.5] * 3)
d6 = feed_rule(rule6, [0.5] * 25, [0.9] * 5 + [0.5] * 20)
check("release needs window AND block evidence below 0.8 for 0.75 s (16 windows)", d6.index('end') == 20, str(d6.index('end')))
core6 = vm.DetectionCore(fast_model_stub, dict(vm.DETECTION_DEFAULTS))
core6_legacy = vm.DetectionCore(legacy_stub, dict(vm.DETECTION_DEFAULTS))
check("default: response stays on k_of_n; fast sensors also run the two-tier rule record-only; legacy sensors have no blocks",
      core6.rule.mode == 'k_of_n' and core6.shadow is not None and core6.shadow.mode == 'two_tier'
      and core6_legacy.shadow is None and core6_legacy.counts['fast_blocks'] == 0)
for mode in ('k_of_n', 'two_tier'):
    core6 = SelfOnlyCore(fast_model_stub, dict(vm.DETECTION_DEFAULTS, decision_mode=mode))
    steps6 = [core6.step({'rms': 0.1, 'block_rms': 0.1}) for _ in range(30)]
    steps6 += [core6.step({'rms': 0.1, 'block_rms': 0.5}) for _ in range(3)]
    if mode == 'k_of_n':
        check("record-only: a 5x block burst raises a 'fast' shadow alarm while the k_of_n response stays quiet",
              steps6[-1]['shadow_decision'] == 'start' and steps6[-1]['shadow_trip'] == 'fast'
              and not any(s['decision'] for s in steps6), str(steps6[-1]['fast_evidence']))
    else:
        check("decision_mode two_tier: the same burst is a real alarm via the fast tier (no shadow rule)",
              steps6[-1]['decision'] == 'start' and steps6[-1]['trip'] == 'fast' and core6.shadow is None)

# live monitor (1 kHz model from the P04 section): record-only alarm is logged as Action=shadow without any response
burst_start = first + 6000
for i in range(first + 4000, burst_start, 20):  # 2 s of normal cutting so the rolling baseline is in place
    clock2[0] += 0.02
    mon2.process_sensor_data(xyz[i:i + 20].tolist(), ts[i:i + 20].tolist())
records_before, overrides_before = len(records), len(mon2.controllers.overrides)
mean6 = xyz[first:first + 4000].mean(axis=0)
for i in range(burst_start, burst_start + 300, 20):  # 0.3 s burst: vibration x5
    clock2[0] += 0.02
    burst = (xyz[i:i + 20] - mean6) * 5 + mean6
    mon2.process_sensor_data(burst.tolist(), ts[i:i + 20].tolist())
new_records = records[records_before:]
check("live record-only two-tier: 0.3 s x5 burst -> '[기록 전용]' log and a Trip=fast, Action=shadow record; no override, no alarm",
      any(a[-1] == 'shadow' and k.get('trip') == 'fast' for a, k in new_records)
      and any(n == 'log_update' and '[기록 전용]' in d.get('message', '') for n, d in mon2.socketio.events)
      and len(mon2.controllers.overrides) == overrides_before and mon2.event is None,
      str([(a[-1], k.get('trip')) for a, k in new_records]))
ok6, _ = mon2.update_settings({'decision_mode': 'two_tier'})
for i in range(burst_start + 300, burst_start + 2300, 20):
    clock2[0] += 0.02
    mon2.process_sensor_data(xyz[i:i + 20].tolist(), ts[i:i + 20].tolist())
records_before = len(records)
for i in range(burst_start + 2300, burst_start + 2600, 20):
    clock2[0] += 0.02
    burst = (xyz[i:i + 20] - mean6) * 5 + mean6
    mon2.process_sensor_data(burst.tolist(), ts[i:i + 20].tolist())
new_records = records[records_before:]
# (the rework thread clears mon2.event as soon as the fake controllers finish, so check the log and the record instead)
check("live two_tier: the same burst is a real alarm through the fast tier, logged with Trip=fast",
      ok6 and any(n == 'log_update' and '판정 빠른 차단' in d.get('message', '') for n, d in mon2.socketio.events)
      and any(k.get('trip') == 'fast' and a[-1] in ('mitigate', 'stop') for a, k in new_records),
      str([(a[-1], k.get('trip')) for a, k in new_records]))

print("== PCA-SPE detector (P08) ==")
from sklearn.decomposition import PCA  # noqa: E402

rng8 = np.random.default_rng(8)
X8 = rng8.normal(size=(400, 3)) @ rng8.normal(size=(3, 9)) + 0.05 * rng8.normal(size=(400, 9))
pca8 = vm._fit_pca(X8, 0.99)
Z8 = (X8 - pca8['mean']) / pca8['scale']
k8 = pca8['loadings'].shape[1]
ref8 = PCA(n_components=k8).fit(Z8)
spe8, t28, contrib8 = vm.pca_scores(pca8, X8)
check("PCA keeps components up to 99% of the standardized variance (3 hidden factors -> 3); SPE and T² equal scikit-learn's PCA",
      k8 == 3 and np.allclose(spe8, ((Z8 - ref8.inverse_transform(ref8.transform(Z8))) ** 2).sum(axis=1), atol=1e-9)
      and np.allclose(t28, (ref8.transform(Z8) ** 2 / ref8.explained_variance_).sum(axis=1)) and np.allclose(contrib8.sum(axis=1), spe8),
      f"k={k8}, explained {pca8['explained']:.3f}")
check("at least two residual directions are left for SPE even when asked for 100% of the variance",
      vm._fit_pca(X8, 1.0)['loadings'].shape[1] == 7)
x8 = X8[0].copy()
x8[4] = np.nan
spe_missing, _, contrib_missing = vm.pca_scores(pca8, x8)
grid8 = np.repeat(X8[:1], 4001, axis=0)
grid8[:, 4] = np.linspace(X8[:, 4].min() - 5, X8[:, 4].max() + 5, 4001)
check("a missing feature (e.g. no non-harmonic ratio) is filled so that SPE uses only the features present (= minimum over its value)",
      abs(spe_missing[0] - vm.pca_scores(pca8, grid8)[0].min()) < 1e-5 and contrib_missing[0][4] < 1e-12)
base8 = {name: 0.1 for name in features.TIME_FEATURE_NAMES}
with_rpm = [dict(base8, spectral_entropy=0.6, nonharmonic_ratio=0.2)] * 100
without_rpm = [dict(base8, spectral_entropy=0.6, nonharmonic_ratio=None)] * 100
names8 = features.TIME_FEATURE_NAMES + ['spectral_entropy', 'nonharmonic_ratio']
vector8 = features.feature_vector(dict(base8, spectral_entropy=0.6), names8)
check("inputs: 9 vibration features (log), plus spectral entropy / non-harmonic ratio (as is) when >= 95% of the training windows "
      "have them (uploaded CSVs carry no spindle speed -> no non-harmonic ratio); missing -> NaN",
      vm.detector_feature_names(with_rpm) == names8 and vm.detector_feature_names(without_rpm) == names8[:-1]
      and vm.detector_feature_names(with_rpm[:90] + without_rpm[:10]) == names8[:-1]
      and vm.detector_feature_names([base8] * 50) == features.TIME_FEATURE_NAMES
      and np.isclose(vector8[0], np.log(0.1)) and vector8[9] == 0.6 and np.isnan(vector8[10]))


def pca_window(load, z_share=0.3):
    """X:Y:Z = 1 : 0.6 : z_share vibration at the given load, steady kurtosis and crest factor, 3 % noise"""
    noise = rng8.normal(0, 0.03, 9)
    window = {'rms_x': load * np.exp(noise[0]), 'rms_y': 0.6 * load * np.exp(noise[1]), 'rms_z': z_share * load * np.exp(noise[2])}
    window.update({f'kurt_{a}': 3.0 * np.exp(noise[3 + i]) for i, a in enumerate('xyz')})
    window.update({f'crest_{a}': 3.5 * np.exp(noise[6 + i]) for i, a in enumerate('xyz')})
    window['rms'] = float(np.sqrt((window['rms_x'] ** 2 + window['rms_y'] ** 2 + window['rms_z'] ** 2) / 3))
    return window


sessions8 = [[pca_window(rng8.uniform(0.1, 0.3)) for _ in range(80)] for _ in range(5)]
model8 = vm.fit_detector(sessions8, None, 'legacy')
core8 = vm.DetectionCore(model8, dict(vm.DETECTION_DEFAULTS))
check("thresholds from leave-one-session-out scores (>= 3 sessions); with 2 sessions in-sample x1.2",
      model8['kind'] == 'pca' and model8['calibration'] == 'session_holdout'
      and vm.fit_detector(sessions8[:2], None, 'legacy')['calibration'] == 'in_sample')
_, parts_load, _ = core8.score(pca_window(1.2))
t2_load = core8.indicators['t2']
check("4x the heaviest training load with the same X:Y:Z pattern -> T² beyond the training range (shown, not an alarm part), SPE < 1",
      t2_load > 1 and parts_load['spe'] < 1 and 't2' not in parts_load, f"T²/limit {t2_load:.2f}, spe {parts_load['spe']:.2f}")
_, parts_break, _ = core8.score(pca_window(0.2, z_share=1.2))
contrib_break = core8.indicators['contributions']
check("Z growing alone breaks the learned correlation -> SPE part >= 1, Z vibration named as the largest contribution",
      parts_break['spe'] > 1 and contrib_break[0][0] == 'rms_z'
      and vm.describe_cause(parts_break, contrib_break).startswith('spe: rms_z '), f"spe {parts_break['spe']:.1f}, {contrib_break}")
_, parts_quiet, _ = core8.score(pca_window(0.05))
ev_t2, parts_t2, _ = vm.DetectionCore(model8, dict(vm.DETECTION_DEFAULTS, t2_alarm=True)).score(pca_window(1.2))
check("windows quieter than the lowest 10% of training get no SPE/T² (chatter makes vibration larger); t2_alarm adds T² as a part",
      'spe' not in parts_quiet and core8.indicators == {} and 't2' in parts_t2 and ev_t2 >= parts_t2['t2'] > 1)
check("cause text for the log and the detection record: largest part, for SPE with its two largest feature contributions",
      vm.describe_cause({'spe': 1.4, 'level': 0.6}, [('kurt_z', 0.46), ('rms_z', 0.21)]) == 'spe: kurt_z 46%, rms_z 21%'
      and vm.describe_cause({'level': 1.2, 'spe': 0.4}, [('kurt_z', 0.5)]) == 'level' and vm.describe_cause({}) == '')
alarm_record = next((k for a, k in records if a[-1] == 'mitigate'), {})
check("live monitor: alarm log and detection record carry the cause; status events carry T²",
      alarm_record.get('cause') and any(n == 'log_update' and '주요 원인 ' in d.get('message', '') for n, d in sock.events)
      and any(n == 'vibration_status' and 't2' in d and 'contributions' in d for n, d in sock.events), str(alarm_record))

print("== override commands: fewer GRBL cycles, feed first, confirmation times (P07) ==")


def run_plan(current, plan):
    """GRBL 1.1: per processing cycle reset first, then +-10 / +-1 once each; the value is clamped at the end of the cycle"""
    value, path = current, []
    for batch in plan:
        if len(set(batch)) != len(batch):
            return None, path  # the same command twice in one cycle would count once
        value = 100 if 'reset' in batch else value
        value += sum(int(step) for step in batch if step != 'reset')
        path.append(value)
        value = max(10, min(200, value))
    return value, path


wrong = []
for current in [None] + list(range(10, 201)):
    for target in range(10, 201):
        # an unknown value is modelled as an impossible 1000 %, so only plans that start with a reset can reach the target
        final, path = run_plan(1000 if current is None else current, cnc_app.override_plan(current, target))
        if final != target or any(not 10 <= value <= 200 for value in path):
            wrong.append((current, target))
check("every change between 10% and 200% (and from an unknown value) reaches the target without leaving the range mid-way",
      not wrong, str(wrong[:5]))
check("typical plans: +10 in one cycle, -20 in two, 110->80 via reset in two (not three), 15->10 by -1 steps, 100->117 in three",
      cnc_app.override_plan(100, 110) == [['+10']] and cnc_app.override_plan(100, 80) == [['-10'], ['-10']]
      and cnc_app.override_plan(110, 80) == [['reset', '-10'], ['-10']] and cnc_app.override_plan(15, 10) == [['-1']] * 5
      and len(cnc_app.override_plan(100, 117)) == 3 and cnc_app.override_plan(None, 100) == [['reset']]
      and cnc_app.override_plan(80, 80) == [])


class OverrideControllers(FakeControllers):
    def apply_overrides(self, targets):
        self.overrides.append(dict(targets))
        now = time.time()
        return {'status': 'success', 'values': dict(targets), 'sent_at': now, 'confirmed_at': now + 0.2, 'commands': 3,
                'retries': 0}


sock7 = FakeSocket()
mon7 = vm.VibrationMonitor(sock7, OverrideControllers())
event7 = {'detected_at': time.time() - 0.01}
mon7._apply_overrides(110, 80, event7)
check("monitor sends both overrides in one call (feed first) and logs detection -> send -> confirmation times",
      mon7.controllers.overrides == [{'feed': 80, 'spindle': 110}] and list(mon7.controllers.overrides[0]) == ['feed', 'spindle']
      and event7['override']['confirmed_at'] - event7['override']['detected_at'] > 0.2 and mon7.get_status()['last_override']
      and any(n == 'log_update' and '반영 확인' in d['message'] and '감지 후 전송' in d['message'] for n, d in sock7.events))

print("== physical variables and the monotone condition model (P09) ==")
import joblib  # noqa: E402
import train_models  # noqa: E402

phys9 = features.cutting_physics(20000, 100, 0.4, 2, 6)
check("physics: fz = F/(n*Z), A_chip = ap*fz, MRR = ap*ae*F (ap*F without ae); unknown flutes or zero depth -> None",
      abs(phys9['fz'] - 0.0025) < 1e-12 and abs(phys9['a_chip'] - 0.001) < 1e-12 and abs(phys9['mrr'] - 240.0) < 1e-9
      and abs(features.cutting_physics(20000, 100, 0.4, 2)['mrr'] - 40.0) < 1e-9
      and features.cutting_physics(20000, 100, 0.4, None) is None and features.cutting_physics(20000, 100, 0.0, 2) is None)
rows9 = pd.DataFrame({'run_id': ['a', 'a', 'b', 'c'], 'label': ['stable', 'chatter', 'stable', 'stable'], 'n': 20000.0,
                      'F': 100.0, 'ap': 0.3, 'flutes': [2, 2, 2, np.nan], 'ae': 6.0, 'rms': 0.1})
training9 = train_models.condition_rows(rows9)
check("training rows: stable windows with a known flute count only (chatter and unknown-flute windows are left out)",
      list(training9['run_id']) == ['a', 'b'] and np.allclose(training9['a_chip'], 0.3 * 100 / (20000 * 2)))
grid_raw = os.path.join(COPY, "data", "synthetic_grid", "raw")
grid_ds = os.path.join(COPY, "data", "synthetic_grid", "dataset")
cond_path = os.path.join(COPY, "models", "condition_grid_unit.joblib")
g1 = subprocess.run([py, make_syn, "--grid", "--quick", "--out", grid_raw], capture_output=True, text=True, encoding="utf-8")
g2 = subprocess.run([py, os.path.join(COPY, "scripts", "build_dataset.py"), "--raw", grid_raw, "--out", grid_ds],
                    capture_output=True, text=True, encoding="utf-8")
g3 = subprocess.run([py, os.path.join(COPY, "scripts", "train_models.py"), "--dataset", grid_ds, "--skip-detector", "--skip-sld",
                     "--condition-out", cond_path], capture_output=True, text=True, encoding="utf-8")
grid_runs = data_recorder.read_manifest(grid_raw)
cm9 = vm.load_condition_model(cond_path)
check("synthetic grid test in the recorder format (12 runs, syn_grid) -> dataset -> condition model with a held-out coverage "
      "report; a coverage short of the 99% target (this sparse quick grid) comes with a warning",
      g1.returncode == 0 and len(grid_runs) == 12 and {r['run_type'] for r in grid_runs} == {'syn_grid'} and g2.returncode == 0
      and g3.returncode == 0 and cm9 is not None and "상한 안에 든 비율" in g3.stdout and cm9['log_margin'] > 0
      and (cm9['coverage'] >= 0.97 or "⚠️" in g3.stdout) and abs(cm9['sensor_fs'] - 1000.0) < 5, (g3.stdout + g3.stderr)[-300:])
print("    " + "\n    ".join(line for line in g3.stdout.splitlines() if line.startswith('  ')))
rng9 = np.random.default_rng(9)
base9 = np.column_stack([rng9.uniform(16000, 24000, 300), rng9.uniform(0.1, 0.4, 300), rng9.uniform(2e-4, 2e-3, 300)])
monotone9 = True
for column in (1, 2):  # ap, a_chip
    for factor in (1.05, 1.5, 3.0):
        bumped = base9.copy()
        bumped[:, column] *= factor
        monotone9 = monotone9 and bool(np.all(cm9['model'].predict(bumped) >= cm9['model'].predict(base9) - 1e-12))
check("condition model never predicts less vibration for a deeper cut or a larger chip (monotone in ap and A_chip)", monotone9)
core9 = vm.DetectionCore(mon2.models['fast'], dict(vm.DETECTION_DEFAULTS), condition_model=cm9)
cond9 = {'n': 20000.0, 'F': 150.0, 'ap': 0.3, 'flutes': 2}
limit9 = core9.condition_limit(cond9, 1000.0)
window9 = dict({name: 0.1 for name in features.TIME_FEATURE_NAMES}, rms=0.5 * limit9, fs=1000.0, spectral=False)
_, parts9, _ = core9.score(window9, cond9)
_, parts9_no_flutes, _ = core9.score(window9, dict(cond9, flutes=None))
level_threshold = mon2.models['fast']['rms_threshold']
check("condition part = RMS / (predicted x held-out 99% margin); with it, the absolute level part is relaxed x1.5",
      abs(parts9['condition'] - 0.5) < 1e-9 and abs(parts9['level'] - window9['rms'] / level_threshold / 1.5) < 1e-9
      and 'condition' not in parts9_no_flutes and abs(parts9_no_flutes['level'] - window9['rms'] / level_threshold) < 1e-9,
      f"limit {limit9:.4f} g, {parts9}")
check("no condition part outside the trained conditions or for a sensor with another sampling rate; deeper cut -> higher limit",
      core9.condition_limit(dict(cond9, n=30000.0), 1000.0) is None and core9.condition_limit(cond9, 100.0) is None
      and core9.condition_limit(dict(cond9, ap=0.2), 1000.0) <= limit9 <= core9.condition_limit(dict(cond9, ap=0.4), 1000.0))
old_path = os.path.join(COPY, "models", "condition_old_format.joblib")
joblib.dump({'model': None, 'features': ['n', 'F', 'ap'], 'sigma': 0.3}, old_path)
check("a condition model saved before P09 (z-score format) is not used (retrain message)", vm.load_condition_model(old_path) is None)
ev9 = subprocess.run([py, os.path.join(COPY, "scripts", "evaluate_detector.py"), "--dataset", syn_ds, "--condition-model", cond_path,
                      "--json", os.path.join(COPY, "data", "synthetic", "quick_condition.json")],
                     capture_output=True, text=True, encoding="utf-8")
res9 = json.load(open(os.path.join(COPY, "data", "synthetic", "quick_condition.json"), encoding="utf-8")) if ev9.returncode == 0 else {}
check("evaluate_detector --condition-model: evaluation with the condition model, model summary kept in the JSON",
      ev9.returncode == 0 and (res9.get('condition_model') or {}).get('n_runs') == 12, (ev9.stdout + ev9.stderr)[-300:])

print("== ramp onset correction, censoring and the stability limit model (P10) ==")
import build_dataset  # noqa: E402

rng10 = np.random.default_rng(10)
t10 = np.arange(-4.0, 1.0001, 0.05)
rise10 = 0.05 + 0.002 * t10 + 0.04 * np.clip(t10 + 1.5, 0.0, 0.7)  # rises from 1.5 s before the label, levels off 0.7 s later
rows10 = [{'t_host': 100.0 + ti, 'rms': xi + rng10.normal(0, 0.0005), 'nonharmonic_ratio': None} for ti, xi in zip(t10, rise10)]
onset10, method10 = build_dataset.refine_onset(rows10, 100.0, 0.2)
flat10 = [dict(r, rms=0.05 + rng10.normal(0, 0.001)) for r in rows10]
check("onset re-detection: a rise that starts 1.5 s before the label and levels off is found (minus half a window); "
      "no rise -> the label time is kept",
      method10.startswith('signal') and abs(onset10 - (100.0 - 1.5 - 0.1)) < 0.06
      and build_dataset.refine_onset(flat10, 100.0, 0.2) == (100.0, 'label'), f"{onset10 - 100.0:+.2f} s, {method10}")
ramp_raw = os.path.join(COPY, "data", "synthetic_ramp", "raw")
ramp_ds = os.path.join(COPY, "data", "synthetic_ramp", "dataset")
r1 = subprocess.run([py, make_syn, "--ramp", "--quick", "--out", ramp_raw], capture_output=True, text=True, encoding="utf-8")
r2 = subprocess.run([py, os.path.join(COPY, "scripts", "build_dataset.py"), "--raw", ramp_raw, "--out", ramp_ds],
                    capture_output=True, text=True, encoding="utf-8")
limits10 = pd.read_csv(os.path.join(ramp_ds, "ramp_limits.csv"), encoding="utf-8-sig").set_index('run_id') if r2.returncode == 0 \
    else pd.DataFrame()
truth10 = pd.read_csv(os.path.join(ramp_raw, "truth.csv"), encoding="utf-8-sig").set_index('run_id') if r1.returncode == 0 \
    else pd.DataFrame()
merged10 = limits10.join(truth10[['limit_mm', 'censored']].rename(columns={'censored': 'truth_censored'})) if len(limits10) else limits10
chatter10 = merged10[merged10['censored'] == 0] if len(merged10) else merged10
check("synthetic ramp (16000-22000 rpm): runs whose limit (0.6 mm) is beyond the deepest cut (0.55 mm) are censored; for the "
      "others the corrected onset depth is within 0.01 mm of the true limit, the label depth is 0.04 mm or more too deep",
      r1.returncode == 0 and r2.returncode == 0 and list(merged10['censored']) == list(merged10['truth_censored'])
      and set(merged10[merged10['censored'] == 1]['n'].round()) == {16000.0, 20000.0} and len(chatter10) == 2
      and (chatter10['onset_ap'] - chatter10['limit_mm']).abs().max() < 0.01
      and (chatter10['label_ap'] - chatter10['limit_mm']).min() > 0.04,
      merged10[['n', 'limit_mm', 'label_ap', 'onset_ap', 'censored']].to_string() if len(merged10) else (r1.stderr + r2.stderr)[-300:])
sld_path = os.path.join(COPY, "models", "sld_unit.joblib")
r3 = subprocess.run([py, os.path.join(COPY, "scripts", "train_models.py"), "--dataset", ramp_ds, "--skip-detector",
                     "--skip-condition", "--sld-out", sld_path], capture_output=True, text=True, encoding="utf-8")
sld10 = optimizer.load_sld_model(sld_path)
check("stability limit model: GPR on the tooth period, censored runs entered at the deepest tested depth, leave-one-speed-out report",
      r3.returncode == 0 and sld10 is not None and sld10['flutes'] == 2 and sld10['n_censored'] == 2
      and sld10['speed_range'] == [16000.0, 22000.0] and abs(sld10['max_tested_ap'] - 0.55) < 0.01
      and "회전수 하나씩 빼고 예측" in r3.stdout, (r3.stdout + r3.stderr)[-400:])
safe10 = {rpm: optimizer.sld_safe_depth(sld10, rpm) for rpm in (15000, 16000, 18000, 22000, 23000)} if sld10 else {}
check("safe depth: none outside the tested speeds; ap_safe <= mean <= deepest tested depth; a censored speed stays near the "
      "tested maximum, a chattering speed near its onset depth",
      safe10 and safe10[15000] is None and safe10[23000] is None
      and all(v['ap_safe'] <= v['mu'] <= sld10['max_tested_ap'] + 1e-9 for v in safe10.values() if v)
      and safe10[16000]['mu'] > 0.45 and safe10[18000]['mu'] < 0.36,
      str({rpm: v and {k: round(x, 3) if isinstance(x, float) else x for k, x in v.items()} for rpm, v in safe10.items()}))
strict10 = optimizer.sld_safe_depth(dict(sld10, kappa=50.0), 18000) if sld10 else None
check("a speed whose safe depth falls below the minimum depth is flagged for a manual check",
      strict10 is not None and strict10['ap_safe'] == 0.0 and strict10['manual_check'])
old_ramp = os.path.join(COPY, "data", "old_ramp_dataset")
os.makedirs(old_ramp, exist_ok=True)
pd.DataFrame({'run_id': ['a', 'b'], 'n': [18000, 20000], 'F': [100, 100], 'onset_ap': [0.31, None], 'max_ap': [0.6, 0.6],
              'note': ['', '채터 없음 (최대 깊이까지 안정)']}).to_csv(os.path.join(old_ramp, 'ramp_limits.csv'), index=False,
                                                              encoding='utf-8-sig')
old_rows10 = train_models.sld_rows(old_ramp)
check("ramp_limits.csv written before P10 (no censored column): an empty onset counts as censored at the deepest tested depth",
      list(old_rows10['censored']) == [0, 1] and list(old_rows10['target']) == [0.31, 0.6], old_rows10.to_string())

print("== optimizer with the stability limit model (P11) ==")
sld11_ds = os.path.join(COPY, "data", "sld_unit_dataset")
os.makedirs(sld11_ds, exist_ok=True)
speeds11 = list(range(16000, 24001, 500))  # trough at 19000-20500 rpm, stable lobe from 22500 rpm
limits11 = [0.30, 0.30, 0.28, 0.24, 0.18, 0.14, 0.12, 0.12, 0.12, 0.12, 0.14, 0.20, 0.30, 0.40, 0.48, 0.50, 0.50]
pd.DataFrame({'run_id': [f"R{s}" for s in speeds11], 'n': speeds11, 'F': 100.0, 'onset_ap': limits11, 'max_ap': 0.55,
              'censored': 0, 'flutes': 2, 'tool_id': 'T1'}).to_csv(os.path.join(sld11_ds, 'ramp_limits.csv'), index=False,
                                                                     encoding='utf-8-sig')
sld11_path = os.path.join(COPY, "models", "sld_p11_unit.joblib")
with contextlib.redirect_stdout(io.StringIO()):
    train_models.train_sld(sld11_ds, sld11_path)
opt11 = optimizer.CuttingConditionOptimizer(model_path=sld11_path)
safe11 = {s: optimizer.sld_safe_depth(opt11.sld, s)['ap_safe'] for s in range(16000, 24001, 250)} if opt11.sld else {}


def rework_time11(speed, n, ap, min_passes):
    """rework time in units of one pass at the current feed (passes / feed ratio); None if more than 5 passes are needed"""
    needed = int(np.ceil(ap / safe11[speed] - 1e-9))
    return max(needed, min_passes) / min(1.0, speed / n) if needed <= 5 else None


def fastest11(times, n, chosen):
    """the chosen speed has the shortest time, and no speed with the same time is closer to the current speed"""
    best = min(t for t in times.values() if t is not None)
    return abs(times[chosen] - best) < 1e-9 and all(abs(s - n) >= abs(chosen - n) for s, t in times.items()
                                                    if t is not None and abs(t - best) < 1e-9)


describe11 = opt11.describe()
check("model window info: stability limit model with its ramp count, speed range and tool",
      describe11['sld'] and describe11['methods'][0] == 'sld' and describe11['runs'] == 17
      and describe11['speed_range'] == [16000.0, 24000.0] and describe11['flutes'] == 2 and describe11['tool_id'] == 'T1',
      str(describe11))
rw11 = opt11.suggest(20000, 100, 0.3, flutes=2, tool_id='T1', purpose='rework')
times11 = {s: rework_time11(s, 20000, 0.3, 1) for s in safe11}
check("rework at the trough (20000 rpm, 0.3 mm): the speed with the shortest rework time (passes / feed), here one pass on "
      "the stable lobe; feed unchanged", rw11['method'] == 'sld' and rw11['passes'] == 1 and rw11['n'] >= 22000
      and rw11['F'] == 100.0 and rw11['ap_safe'] >= 0.3 and fastest11(times11, 20000, rw11['n']), str(rw11))
rw11_min2 = opt11.suggest(20000, 100, 0.3, flutes=2, tool_id='T1', purpose='rework', min_passes=2)
times11_min2 = {s: rework_time11(s, 20000, 0.3, 2) for s in safe11}
check("with the monitor's 2-pass minimum the comparison changes: the nearest speed where 2 passes are safe",
      rw11_min2['method'] == 'sld' and rw11_min2['passes'] == 2 and 20000 < rw11_min2['n'] < rw11['n']
      and fastest11(times11_min2, 20000, rw11_min2['n']), str(rw11_min2))
on11 = opt11.suggest(20000, 100, 0.3, flutes=2, tool_id='T1', purpose='online')
closer11 = [s for s, v in safe11.items() if v >= 0.3 and 250 <= abs(s - 20000) < abs(on11['n'] - 20000)]
check("online: the nearest speed whose safe depth covers the current depth (never the chattering speed), feed unchanged",
      on11['method'] == 'sld' and on11['passes'] == 1 and on11['F'] == 100.0 and safe11[on11['n']] >= 0.3
      and abs(on11['n'] - 20000) >= 250 and not closer11, str(on11))
on11_low = opt11.suggest(19000, 100, 0.2, flutes=2, tool_id='T1', purpose='online')
check("a lower speed lowers the feed by the same ratio, so the feed per tooth does not grow",
      on11_low['method'] == 'sld' and on11_low['n'] < 19000 and abs(on11_low['F'] - 100 * on11_low['n'] / 19000) < 0.1
      and on11_low['F'] / on11_low['n'] <= 100 / 19000 and "날당 이송" in on11_low['reason'], str(on11_low))
other11 = [opt11.suggest(20000, 100, 0.3, flutes=3, tool_id='T1'), opt11.suggest(20000, 100, 0.3, flutes=2, tool_id='T9')]
check("another tool (flute count or tool number differs) -> the model is not used and the reason says so; "
      "an unknown tool number with the same flute count still uses it",
      all(r['method'] == 'fallback' and "다른 공구(T1, 2날)" in r['reason'] for r in other11)
      and opt11.suggest(20000, 100, 0.3, flutes=2)['method'] == 'sld', other11[0]['reason'])
unknown11 = [opt11.suggest(20000, 100, 0.3), opt11.suggest(20000, 100, None, flutes=2, purpose='online')]
check("unknown flute count or depth -> default rule, with the reason",
      all(r['method'] == 'fallback' for r in unknown11) and "날 수를 몰라" in unknown11[0]['reason']
      and "깊이를 몰라" in unknown11[1]['reason'], f"{unknown11[0]['reason']} / {unknown11[1]['reason']}")
deep11 = opt11.suggest(20000, 100, 3.0, flutes=2, purpose='rework')
deep11_online = opt11.suggest(20000, 100, 0.6, chatter_freq=1500, flutes=2, purpose='online', sample_rate=4000)
check("no speed can take the depth (rework: more than 5 passes; online: deeper than every safe depth) -> the next method, "
      "with the reason", deep11['method'] == 'fallback' and "5번 안에" in deep11['reason']
      and deep11_online['method'] == 'chatter_frequency' and "안전한 다른 회전수가 없음" in deep11_online['reason'],
      f"{deep11['reason']} / {deep11_online['reason']}")
check("the pass minimum applies to every method (default-rule rework with a 3-pass minimum -> 3 passes)",
      opt.suggest(20000, 100, 0.3, purpose='rework', min_passes=3)['passes'] == 3)
grows11 = []
for n11 in range(16000, 24001, 500):
    for ap11 in (0.1, 0.2, 0.3, 0.45, 0.6):
        for purpose11 in ('online', 'rework'):
            r = opt11.suggest(n11, 100, ap11, flutes=2, purpose=purpose11, min_passes=2)
            if r['F'] > 100 or r['F'] / r['n'] > 100 / n11:
                grows11.append((n11, ap11, purpose11, r['n'], r['F']))
check("feed and feed per tooth never grow (every speed x depth x purpose, 170 cases)", not grows11, str(grows11[:3]))
old_gpc11 = os.path.join(COPY, "models", "stability_model.joblib")
joblib.dump({'model': None, 'features': ['n', 'F', 'ap'], 'p_max': 0.05}, old_gpc11)
ce11 = subprocess.run([py, os.path.join(COPY, "scripts", "check_env.py")], capture_output=True, text=True, encoding="utf-8")
check("an old chatter-probability model file is never read as a stability limit model; check_env says it can be deleted",
      optimizer.CuttingConditionOptimizer(model_path=old_gpc11).sld is None and "예전 채터 확률 모델" in ce11.stdout,
      ce11.stdout[-300:])

print("== dataset detector model, settings validation, model window (P12) ==")
t12 = 1000.0 + np.arange(1, 81) * 0.25  # 1 s windows every 0.25 s, ends 1000.25 .. 1020.0
w12 = pd.DataFrame({'run_id': 'r1', 't_host': t12, 'gate': 1, 'transient': 0, 'label': 'stable', 'profile': 'legacy'})
w12.loc[(w12['t_host'] > 1010.5) & (w12['t_host'] <= 1014.5), 'label'] = 'chatter'
w12 = pd.concat([w12, w12.assign(run_id='r2', label='stable')], ignore_index=True)
kept12 = train_models.normal_windows(w12, {'r1': [1010.0]})
stable12 = set(w12[(w12['run_id'] == 'r1') & (w12['label'] == 'stable')]['t_host'])
dropped12 = stable12 - set(kept12[kept12['run_id'] == 'r1']['t_host'])
fallback12 = train_models.normal_windows(w12, train_models.label_starts(os.path.join(COPY, "data", "no_labels_here"), w12))
dropped_fb12 = stable12 - set(fallback12[fallback12['run_id'] == 'r1']['t_host'])
check("normal windows: stable windows overlapping the 3 s before a chatter label (late label) or its start are left out; after the "
      "label and other runs are kept; without labels.csv the start of the first chatter window is used (a window earlier)",
      sorted(dropped12) == [t for t in t12 if 1007.0 < t < 1011.0 and not 1010.5 < t <= 1014.5]
      and (kept12['run_id'] == 'r2').sum() == 80 and kept12[kept12['run_id'] == 'r1']['t_host'].max() == 1020.0
      and sorted(dropped_fb12 - dropped12) == [1007.0] and max(dropped_fb12) < 1011.0,
      f"dropped {sorted(dropped12)}, without labels.csv {sorted(dropped_fb12)}")
check("evaluate_detector trains on the same normal windows (guard recorded in the JSON)",
      res.get('label_guard_sec') == 3.0 and res.get('guarded_windows', 0) > 0, str({k: res.get(k) for k in ('label_guard_sec', 'guarded_windows')}))
os.replace(detector_unit, os.path.join(COPY, "models", "detector_model.joblib"))
bundle12 = vm.load_detector_models()
check("dataset detector model: one PCA-SPE model per window setting, with the non-harmonic ratio (the dataset has the spindle speed)",
      bundle12 is not None and set(bundle12['models']) == {'fast'} and bundle12['dataset'] == 'data/dataset'
      and 'nonharmonic_ratio' in bundle12['models']['fast']['feature_names'] and bundle12['label_guard_sec'] == 3.0,
      str(bundle12 and {p: m['feature_names'] for p, m in bundle12['models'].items()}))
stale12 = os.path.join(COPY, "data", "detector_stale.joblib")
joblib.dump(dict(bundle12, feature_version=bundle12['feature_version'] - 1), stale12)
joblib.dump({'kind': 'lof', 'models': bundle12['models']}, stale12 + ".kind")
check("a detector model from another feature version or format is not used",
      vm.load_detector_models(stale12) is None and vm.load_detector_models(stale12 + ".kind") is None)
mon12 = vm.VibrationMonitor(FakeSocket(), FakeControllers())
mon12.set_sensor_info({'device': 'MPU6050', 'fs': 1000.0, 'range_g': 8})
st12 = mon12.get_status()
check("server: the dataset model serves its window setting (no fallback), the folder CSVs train the other one; status says which",
      st12['window']['profile'] == 'fast' and not st12['window']['fallback'] and st12['profiles']['fast']['source'] == 'dataset'
      and st12['profiles']['fast']['nonharmonic'] and st12['profiles']['legacy']['source'] == 'folder'
      and st12['detector_model']['profiles'] == ['fast'] and st12['t2_recent'] is None and json.dumps(st12) is not None,
      str({p: (v['source'], v['features']) for p, v in st12['profiles'].items()}))
mon12.is_running = True
clock12 = [9000.0]
mon12.clock = lambda: clock12[0]
mon12._on_alarm = lambda *a, **k: None  # 이 시험은 학습 범위 밖 비율만 봄 (대응은 앞의 시험에서 확인)
mon12._on_shadow_alarm = lambda *a, **k: None
s07 = pd.read_csv(os.path.join(raw_dir, "S07_sensor.csv"))  # 데이터셋의 안정 실행 (20000 rpm, 0.35 mm)
xyz07, ts07 = s07[['AccX', 'AccY', 'AccZ']].to_numpy(), s07['DeviceTime'].to_numpy()
mon12.controllers.snap.update(n_eff=20000.0, n_cmd=20000.0, F_eff=100.0, F_cmd=100.0, z=-0.35)
start07 = int(np.searchsorted(ts07, 2.0))
for i in range(start07, start07 + 3000, 20):
    clock12[0] += 0.02
    mon12.process_sensor_data(xyz07[i:i + 20].tolist(), ts07[i:i + 20].tolist())
recent12 = mon12.get_status()['t2_recent']
check("model window: share of recent judged windows outside the training range (T² >= threshold), counted per model; "
      "a cut like the training runs stays inside",
      recent12 is not None and recent12['windows'] >= 20 and recent12['fraction'] <= 0.2
      and abs(recent12['seconds'] - recent12['windows'] * 0.05) < 1e-9 and mon12._t2_recent.maxlen == 1200, str(recent12))
os.replace(os.path.join(COPY, "models", "detector_model.joblib"), detector_unit)
ok12r, msg12r = mon12.reset_analyzer()
st12r = mon12.get_status()
os.replace(detector_unit, os.path.join(COPY, "models", "detector_model.joblib"))
ok12s, msg12s = mon12.reset_analyzer()
check("'reset' re-reads models/: without the dataset model the 1 kHz sensor falls back to the 1 s folder model; with it, the "
      "dataset model is used again and the message says so",
      ok12r and st12r['detector_model'] is None and st12r['window']['fallback'] and ok12s
      and mon12.get_status()['profiles']['fast']['source'] == 'dataset' and "데이터셋 감지 모델" in msg12s, f"{msg12r} / {msg12s}")
with open(os.path.join(syn_raw, stream_run['sensor_file']), "rb") as f:
    ok12u, msg12u = mon12.add_training_file("fast_p12.csv", f.read())
check("uploading a 1 kHz CSV retrains the folder models, and the message says the dataset model still serves that window setting",
      ok12u and "models/detector_model.joblib" in msg12u and mon12.get_status()['profiles']['fast']['source'] == 'dataset', msg12u)
os.remove(os.path.join(COPY, "data", "학습 데이터", "fast_p12.csv"))

check("settings validation: every default passes; wrong types, out-of-range and unknown values are named",
      all(vm.setting_problem(k, v) is None for k, v in vm.DETECTION_DEFAULTS.items())
      and all(vm.setting_problem(k, v) for k, v in (('cusum_cap', 1.0), ('rework_min_passes', 6), ('rework_min_passes', 2.0),
                                                     ('spe_quantile', 1.5), ('mitigation_enabled', 'false'), ('bogus', 1),
                                                     ('confirm_sec', float('nan')), ('decision_mode', 'bogus'))))
with open(vm.SETTINGS_PATH, "w", encoding="utf-8") as f:
    json.dump({'decision_mode': 'bogus', 'mitigation_enabled': 'false', 't2_alarm': True, 'confirm_sec': 0.5}, f)
loaded12, problems12 = vm.load_settings()
ce12 = subprocess.run([py, os.path.join(COPY, "scripts", "check_env.py")], capture_output=True, text=True, encoding="utf-8")
check("a hand-edited detector_settings.json: only valid screen settings are used, the rest is listed; check_env flags it",
      loaded12 == {'t2_alarm': True} and len(problems12) == 3 and any('confirm_sec' in p for p in problems12)
      and "detector_settings.json에 쓰지 않는 값" in ce12.stdout and "decision_mode='bogus'" in ce12.stdout, str(problems12))
mon12b = vm.VibrationMonitor(FakeSocket(), FakeControllers())
notes12b = mon12b.get_status()['notes']
ok12b, _ = mon12b.update_settings({'decision_mode': 'two_tier'})
notes12c = mon12b.get_status()['notes']
check("the monitor starts with defaults for bad saved values and lists them; two-tier on a 1 s model is noted (no Fast-Trip); "
      "saving rewrites the file with checked values only",
      mon12b.config['decision_mode'] == 'two_tier' and any('decision_mode' in n for n in notes12b) and ok12b
      and any('빠른 차단 없이' in n for n in notes12c) and not any('bogus' in n for n in notes12c)
      and json.load(open(vm.SETTINGS_PATH, encoding="utf-8")).get('decision_mode') == 'two_tier', str(notes12c))
check("screen settings must be true/false or a known mode (no truthy strings)",
      not mon12b.update_settings({'mitigation_enabled': 'false'})[0] and not mon12b.update_settings({'confirm_sec': 0.5})[0])
mon12b.update_settings({'decision_mode': 'k_of_n'})
ev12 = subprocess.run([py, os.path.join(COPY, "scripts", "evaluate_detector.py"), "--dataset", syn_ds, "--set", "cusum_cap=0.5"],
                      capture_output=True, text=True, encoding="utf-8")
check("evaluate_detector --set: a value outside the allowed range is refused before evaluating",
      ev12.returncode != 0 and "cusum_cap=0.5" in (ev12.stdout + ev12.stderr), (ev12.stdout + ev12.stderr)[-200:])
old_cond12 = os.path.join(COPY, "models", "condition_p12.joblib")
joblib.dump({'model': None, 'features': ['n', 'F', 'ap'], 'sigma': 0.3}, old_cond12)
tm12 = subprocess.run([py, os.path.join(COPY, "scripts", "train_models.py"), "--skip-detector", "--skip-sld",
                       "--condition-out", old_cond12], capture_output=True, text=True, encoding="utf-8")
check("train_models moves unused old models to models/legacy/ instead of deleting or overwriting them "
      "(chatter-probability model, old-format condition model at the output path)",
      tm12.returncode == 0 and not os.path.exists(old_gpc11) and "[예전 모델]" in tm12.stdout
      and os.path.isfile(os.path.join(COPY, "models", "legacy", "stability_model.joblib"))
      and os.path.isfile(os.path.join(COPY, "models", "legacy", "condition_p12.joblib"))
      and vm.load_condition_model(old_cond12) is not None, (tm12.stdout + tm12.stderr)[-300:])

print(f"\nunit checks: {sum(results)}/{len(results)} passed", flush=True)
os._exit(0 if all(results) else 1)
