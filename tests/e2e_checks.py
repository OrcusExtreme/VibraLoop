"""End-to-end checks of cnc_app (socket handlers + streaming thread + background threads + watcher)
against the GRBL simulator (tests/grbl_sim.py).

Runs on a throw-away copy of the project made by run_tests.py, so nc/rework.nc, data/ and logs/ are untouched:
  python tests/run_tests.py e2e
Fixtures used from the copy: nc/O0527.txt, nc/O0807.txt, nc/O1001.nc, data/학습 데이터/*.csv, data/Vibration_detection.csv
"""
import csv
import glob
import io
import json
import subprocess
import logging
import os
import sys
import threading
import time
import warnings

import numpy as np
import pandas as pd

if len(sys.argv) < 2 or not os.path.isdir(sys.argv[1]):
    sys.exit("usage: python tests/run_tests.py e2e  (this script needs a project copy as its argument)")
warnings.filterwarnings("ignore")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
COPY = os.path.abspath(sys.argv[1])
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, COPY)
with open(os.path.join(COPY, "data", "Vibration_detection.csv"), encoding="utf-8", newline="") as f:
    INITIAL_DETECTION_ROWS = list(csv.reader(f))
TRAIN_CSVS = glob.glob(os.path.join(COPY, "data", "학습 데이터", "*.csv"))
N_NORMAL_FILES = sum("이상" not in os.path.basename(f) for f in TRAIN_CSVS)
logging.basicConfig(level=logging.WARNING)

import grbl_sim  # noqa: E402
import cnc_app  # noqa: E402
import rework  # noqa: E402  (src/ is on sys.path after importing cnc_app)
import features  # noqa: E402
import vibration_monitor  # noqa: E402
from sensor_protocol import SensorStream  # noqa: E402

logging.getLogger().setLevel(logging.WARNING)
for name in ("vibration", "watcher", "cnc_app", "__main__", "werkzeug", "engineio", "socketio"):
    logging.getLogger(name).setLevel(logging.WARNING)

results = []


def check(name, cond, detail=""):
    results.append(bool(cond))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""), flush=True)


def wait_for(pred, timeout=30.0, interval=0.05):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(interval)
    return pred()


SIMS = []


def fake_serial(*args, **kwargs):
    sim = grbl_sim.FakeGrbl(port=kwargs.get("port", "SIM"), time_scale=20.0)
    SIMS.append(sim)
    return sim


cnc_app.serial.Serial = fake_serial

client = cnc_app.socketio.test_client(cnc_app.app)
http = cnc_app.app.test_client()
events = []
events_lock = threading.Lock()


def drain():
    with events_lock:
        for packet in client.get_received():
            events.append((time.time(), packet["name"], packet["args"][0] if packet["args"] else None))
    return events


def emit(event, data=None):
    result = client.emit(event, data if data is not None else {}, callback=True)
    drain()
    return result


def logs_since(t0):
    return [d.get("message", "") for (t, n, d) in drain() if n == "log_update" and t >= t0 and isinstance(d, dict)]


def nc(name):
    with open(os.path.join(COPY, "nc", name), encoding="utf-8") as f:
        return f.read()


# background services exactly as in cnc_app.__main__
monitor = cnc_app.get_vibration_monitor()
check("old check_alarm/check_connection threads merged into monitor_controller",
      hasattr(cnc_app, "monitor_controller") and not hasattr(cnc_app, "check_alarm") and not hasattr(cnc_app, "check_connection"))
threading.Thread(target=cnc_app.monitor_controller, daemon=True).start()
cnc_app.start_nc_file_watcher()

print("== setup ==")
check("vibration analyzer trained from data/학습 데이터 (was disabled before)", monitor.is_trained and monitor.optimizer.is_ready())
r = emit("connect_port", {"port": "SIM"})
sim = SIMS[-1]
check("connect_port to simulated GRBL", r["status"] == "success" and r.get("alarm_status") == "none", str(r))
check("state alarm_unlocked after connect", cnc_app.state == "alarm_unlocked", cnc_app.state)
drain()


def run_program(label, content, timeout=120, during=None):
    """Load + start + wait for completion. Returns (start_line_index, job_events)."""
    r = emit("load_nc_file", {"content": content, "fileName": label})
    assert r["status"] == "success", r
    wait_for(lambda: cnc_app.state in ("idle", "alarm_unlocked"), 10)
    start_idx = len(sim.received_lines)
    t0 = time.time()
    r = emit("start_gcode")
    check(f"{label}: start_gcode accepted", r["status"] == "success", str(r))
    if during:
        during()
    finished = wait_for(lambda: not cnc_app.job_active, timeout)
    machine_idle_at_finish = sim.current_block is None and not sim.planner
    check(f"{label}: job finished", finished and cnc_app.state in ("idle", "alarm_unlocked"), cnc_app.state)
    check(f"{label}: completion reported only after the machine stopped moving", machine_idle_at_finish)
    sent = sim.received_lines[start_idx:]
    check(f"{label}: every line sent exactly once, in order (no resend / no restart)", sent == cnc_app.gcode_lines,
          f"sent {len(sent)} vs program {len(cnc_app.gcode_lines)}")
    check(f"{label}: GRBL RX buffer never overflowed", sim.overflows == 0, f"overflows={sim.overflows}")
    job_events = [(t, n, d) for (t, n, d) in drain() if t >= t0]
    return start_idx, job_events


print("== O0527.txt (modal 'X30 F70' move) ==")
sim.time_scale = 4.0
_, ev = run_program("O0527.txt", nc("O0527.txt"))
check("path follows the modal X30 move (sampled at 5Hz)", max(cnc_app.x_vals) > 29.0, f"max x={max(cnc_app.x_vals):.3f}")
check("final position back at origin", abs(cnc_app.x_vals[-1]) < 1e-3 and abs(cnc_app.z_vals[-1]) < 1e-3)
done_logs = [d["message"] for (_, n, d) in ev if n == "log_update" and "성공적으로 완료" in d.get("message", "")]
check("'가공 완료' emitted once", len(done_logs) == 1)
pos = [d for (_, n, d) in ev if n == "position_update"]
incr = [len(d["path_x"]) for d in pos if d.get("path_start", 0) > 0]
check("position_update sends only new path points", incr and max(incr) <= 5, f"updates={len(pos)}, max points/update={max(incr) if incr else None}")
logs = sorted(glob.glob(os.path.join(COPY, "logs", "cnc_log_*.csv")))
df = pd.read_csv(logs[-1]) if logs else None
check("CSV log written with real coordinates", df is not None and len(df) > 5 and df["X"].max() > 29.0, f"rows={0 if df is None else len(df)}, max X={None if df is None else df['X'].max()}")
states = [d["state"] for (_, n, d) in ev if n == "state_update"]
check("state_update events running -> idle", "running" in states and states[-1] in ("idle", "alarm_unlocked"), str(states))

print("== O1001.nc (CAM program: modal XY, G3 arcs, G28 conversion) ==")
sim.time_scale = 8.0
moves_before = len(sim.executed_moves)
_, ev = run_program("O1001.nc", nc("O1001.nc"), timeout=300)
check("arc/contour path visualised (Y spans the pocket)", min(cnc_app.y_vals) < -15 and max(cnc_app.y_vals) > 14,
      f"y range {min(cnc_app.y_vals):.2f}..{max(cnc_app.y_vals):.2f}")
check("cutting depth Z-0.5 reached", min(cnc_app.z_vals) < -0.49, f"min z={min(cnc_app.z_vals):.3f}")
first_move = next(m for m in sim.executed_moves[moves_before:] if (m[1][0], m[1][1]) != (m[2][0], m[2][1]))
check("first XY move happens at safe height (G28 -> G0 Z5, not G0 X0 Y0 Z0)", first_move[1][2] >= 4.99,
      f"first XY move '{first_move[0]}' at Z={first_move[1][2]:.2f}")

print("== interference during a job (alarm-status polling, overrides, jog, console) ==")
sim.time_scale = 4.0


def poke_during_job():
    wait_for(lambda: cnc_app.state == "running" and len(cnc_app.x_vals) > 3, 10)
    for _ in range(3):
        r1 = emit("check_alarm_status")
        emit("spindle_override_command", {"cmd": "0x9B"})
        time.sleep(0.2)
    check("check_alarm_status during job answers from state (no serial read)", r1["status"] == "success" and r1["state"] == "running", str(r1))
    r2 = emit("jog", {"dx": 1})
    check("jog rejected while machining", r2["status"] == "error", str(r2))
    t0 = time.time()
    emit("grbl_command", {"command": "G91"})
    check("raw GRBL console command rejected while machining", any("직접 보낼 수 없습니다" in m for m in logs_since(t0)))
    r3 = emit("set_feed_override", {"value": 70})
    wait_for(lambda: sim.feed_ov == 70, 2)
    check("feed override 70% applied via realtime bytes (apply_override existed? now yes)", r3["status"] == "success" and sim.feed_ov == 70,
          f"sim feed_ov={sim.feed_ov}")
    check("spindle override -10% x3 used 0x9B", sim.spindle_ov == 70, f"sim spindle_ov={sim.spindle_ov}")


run_program("O0807.txt", nc("O0807.txt"), during=poke_during_job)
emit("set_feed_override", {"value": 100})
emit("spindle_override_command", {"cmd": "0x99"})
wait_for(lambda: sim.feed_ov == 100 and sim.spindle_ov == 100, 2)

print("== pause / resume continues from where it stopped ==")
sim.time_scale = 2.0
emit("load_nc_file", {"content": nc("O0807.txt"), "fileName": "O0807.txt"})
start_idx = len(sim.received_lines)
emit("start_gcode")
wait_for(lambda: cnc_app.y_vals[-1] > 5, 20)
r = emit("toggle_pause")
check("pause accepted", r["status"] == "success" and cnc_app.state == "paused", str(r))
time.sleep(0.6)
y_paused = sim.mpos[1]
time.sleep(1.0)
check("machine holds position while paused", abs(sim.mpos[1] - y_paused) < 1e-6 and sim.hold, f"{y_paused:.3f} -> {sim.mpos[1]:.3f}")
check("streaming thread kept alive during pause", cnc_app.job_active)
r = emit("start_gcode")
check("resume accepted as resume (not a restart)", r["status"] == "success" and r.get("resumed"), str(r))
wait_for(lambda: not cnc_app.job_active, 60)
check("job completed after resume", cnc_app.state in ("idle", "alarm_unlocked") and max(cnc_app.y_vals) > 68.0, f"state={cnc_app.state} max y={max(cnc_app.y_vals):.2f}")
check("no line re-sent after resume (old code restarted from line 0)", sim.received_lines[start_idx:] == cnc_app.gcode_lines,
      f"{len(sim.received_lines[start_idx:])} sent vs {len(cnc_app.gcode_lines)}")

print("== GRBL 'error:' response pauses the job instead of resending ==")
sim.time_scale = 20.0
bad = "G21\nG90\nM3 S20000\nG1 Z-0.2 F200\nG43 H1\nG1 Y20 F200\nM5\nG0 Z5\nG0 X0 Y0 Z0\n"
emit("load_nc_file", {"content": bad, "fileName": "bad.nc"})
start_idx = len(sim.received_lines)
t0 = time.time()
emit("start_gcode")
check("error:20 -> job paused + feed hold", wait_for(lambda: cnc_app.state == "paused", 10) and sim.hold, cnc_app.state)
check("operator told which line failed", any("G43 H1" in m and "error:20" in m for m in logs_since(t0)))
emit("start_gcode")
wait_for(lambda: not cnc_app.job_active, 30)
sent = sim.received_lines[start_idx:]
check("job continues after resume; failing line sent once", sent == cnc_app.gcode_lines and sent.count("G43 H1") == 1, str(sent))

print("== emergency stop ==")
sim.time_scale = 2.0
emit("load_nc_file", {"content": nc("O0807.txt"), "fileName": "O0807.txt"})
emit("start_gcode")
wait_for(lambda: cnc_app.vibration_detection_active, 10)
wait_for(lambda: cnc_app.y_vals[-1] > 3, 20)
r = emit("emergency_stop")
check("emergency accepted", r["status"] == "success" and cnc_app.state == "emergency", str(r))
check("streaming thread exits", wait_for(lambda: not cnc_app.job_active, 5))
count_after = len(sim.received_lines)
time.sleep(1.0)
check("no G-code sent after the soft reset", len(sim.received_lines) == count_after)
check("GRBL raised ALARM:3 (reset while moving)", sim.alarm)
r = emit("emergency_stop")
check("emergency release detects alarm from status report", r["status"] == "success" and cnc_app.state == "alarm", f"{r} state={cnc_app.state}")
r = emit("unlock_alarm")
check("unlock -> alarm_unlocked", r["status"] == "success" and cnc_app.state == "alarm_unlocked" and not sim.alarm, f"{r} state={cnc_app.state}")

print("== tolerance validation ==")
resp = http.post("/set_tolerance", data={"tolerance": "10"}).get_json()
check("10mm tolerance rejected", resp["status"] == "error" and cnc_app.tolerance_value == 0.0, str(resp))
resp = http.post("/set_tolerance", data={"tolerance": "nan"}).get_json()
check("NaN tolerance rejected", resp["status"] == "error")
resp = http.post("/set_tolerance", data={"tolerance": "0.2"}).get_json()
check("0.2mm tolerance accepted", resp["status"] == "success" and cnc_app.tolerance_value == 0.2, str(resp))

print("== request_status (F5 reconnect) ==")
before = len(drain())
emit("request_status")
new = events[before:]
full = [d for (_, n, d) in new if n == "position_update" and d.get("path_start") == 0]
check("full path resent once on reconnect", full and len(full[-1]["path_x"]) == len(cnc_app.x_vals))
check("state_update / tolerance_update sent on reconnect",
      any(n == "state_update" for (_, n, _) in new) and any(n == "tolerance_update" and d["tolerance"] == 0.2 for (_, n, d) in new))

print("== controller monitor thread (alarm + connection checks in one thread) ==")
cnc_app.alarm_check_interval = 0.3
wait_for(lambda: not cnc_app.job_active and cnc_app.state in ("idle", "alarm_unlocked"), 10)
t0 = time.time()
sim._status_report = lambda: None  # controller stops answering '?'
check("3 unanswered polls -> connection_warning", wait_for(lambda: cnc_app.state == "connection_warning", 15), cnc_app.state)
check("no serial error/disconnect while merely silent", cnc_app.ser is not None)
del sim._status_report
check("answers again -> warning cleared (alarm_unlocked)", wait_for(lambda: cnc_app.state == "alarm_unlocked", 15), cnc_app.state)
check("operator told the controller recovered", any("정상으로 돌아왔습니다" in m for m in logs_since(t0)))
with sim.lock:
    sim.alarm = True
t0 = time.time()
check("alarm raised while idle is detected from the status report", wait_for(lambda: cnc_app.state == "alarm", 15), cnc_app.state)
check("alarm_update emitted to the UI", any(n == "alarm_update" and t >= t0 for (t, n, _) in drain()))
r = emit("unlock_alarm")
check("unlock clears the idle alarm", r["status"] == "success" and cnc_app.state == "alarm_unlocked" and not sim.alarm, f"{r} state={cnc_app.state}")
cnc_app.alarm_check_interval = 3.0

print("== confirmed chatter -> position-keeping stop (hold -> Hold:0 -> spindle stop -> reset) -> fallback rework ==")
sim.time_scale = 2.0
normal_file = [f for f in sorted(glob.glob(os.path.join(COPY, "data", "학습 데이터", "*.csv"))) if "13-23-07" in f][0]
normal = pd.read_csv(normal_file, encoding="utf-8-sig")[["AccX", "AccY", "AccZ"]].to_numpy(float)
emit("load_nc_file", {"content": nc("O0807.txt"), "fileName": "O0807.txt"})
source_program = list(cnc_app.gcode_lines)
resets_before, stops_before = sim.resets, sim.spindle_stops
rework_path = os.path.join(COPY, "nc", "rework.nc")
mtime_before = os.path.getmtime(rework_path)
t0 = time.time()
emit("start_gcode")
active = wait_for(lambda: cnc_app.vibration_detection_active and monitor.is_running, 10)
wait_for(lambda: cnc_app.y_vals[-1] > 3, 20)
check("vibration detection active during job", active)
lines_before_stop = len(sim.received_lines)
monitor.handle_abnormal_vibration()
check("stop requested -> state 'stopping'/'emergency'", cnc_app.state in ("stopping", "emergency"), cnc_app.state)
stopped = wait_for(lambda: cnc_app.state == "emergency" and not cnc_app.job_active, 10)
check("feed hold -> Hold:0 -> spindle stop (0x9E) -> reset",
      stopped and sim.resets == resets_before + 1 and sim.spindle_stops == stops_before + 1,
      f"state={cnc_app.state} resets={sim.resets - resets_before} spindle_stops={sim.spindle_stops - stops_before}")
check("no ALARM:3 (reset came after the hold completed, position kept)", not sim.alarm)
check("no G-code lines sent after the stop request", len(sim.received_lines) - lines_before_stop <= 1,
      f"{len(sim.received_lines) - lines_before_stop} lines")
pos_after = list(sim.mpos)
time.sleep(0.5)
check("machine does not move after the stop", sim.mpos == pos_after)
with open(os.path.join(COPY, "data", "Vibration_detection.csv"), encoding="utf-8", newline="") as f:
    det_rows = list(csv.reader(f))
check("Vibration_detection.csv: 11 columns, earlier records kept, new 'stop' record appended",
      det_rows[0] == vibration_monitor.DETECTION_LOG_HEADER and det_rows[:len(INITIAL_DETECTION_ROWS)] == INITIAL_DETECTION_ROWS
      and det_rows[-1][-1] == "stop" and float(det_rows[-1][2]) > 3 and det_rows[-1][4] != "", str(det_rows[-1]))
created = wait_for(lambda: os.path.getmtime(rework_path) > mtime_before, 30)
msgs = logs_since(t0)
check("controller checked ($X ok) before rework.nc was written", any("알람이 성공적으로 해제" in m for m in msgs))
check("rework from the fallback rule (no trained model): feed -20%, depth split in 2",
      created and any("재가공 조건" in m and "깊이 2번 나눔" in m for m in msgs),
      next((m for m in msgs if "재가공 조건" in m), "no rework message"))
started = wait_for(lambda: cnc_app.job_active and cnc_app.state == "running", 15)
check("watcher started the rework job automatically", started, cnc_app.state)
check("rework job uses the original program as source", cnc_app.rework_source_lines == source_program)
check("vibration detection re-enabled for the rework job", wait_for(lambda: cnc_app.vibration_detection_active, 10))
sim.time_scale = 20.0
states = []
while cnc_app.job_active:
    states.append(cnc_app.state)
    time.sleep(0.05)
check("state stayed 'running' for the whole rework", set(states) == {"running"}, str(set(states)))
check("rework finished", cnc_app.state in ("idle", "alarm_unlocked") and not sim.alarm)
with open(rework_path, encoding="utf-8") as f:
    rlines = [l.strip() for l in f if l.strip()]
check("rework.nc: note, safe approach, depth -0.1 -> -0.2, tolerance pass -0.4",
      rlines[0].startswith("(rework fallback") and rlines[3:5] == ["G0 Z5", "G0 X0.000 Y0.000"]
      and any(l.startswith("G1 Z-0.100") for l in rlines) and any(l.startswith("G1 Z-0.200") for l in rlines)
      and any(l.startswith("G1 Z-0.400") for l in rlines), str(rlines[:7]))

print("== rework.nc changed by someone else must NOT start the machine ==")
t0 = time.time()
with open(rework_path, "a", encoding="utf-8") as f:
    f.write("\n")
time.sleep(2.5)
check("no job started by an unarmed rework.nc change", not cnc_app.job_active)
check("operator informed that the change was ignored", any("자동 가공하지 않습니다" in m for m in logs_since(t0)))


class Feeder:
    """Pushes sensor samples into the monitor in real time (100 Hz) with device-like timestamps."""

    def __init__(self, data):
        self.data, self.mean, self.i, self.t = data, data.mean(axis=0), 0, 1000.0

    def run(self, seconds, scale=1.0):
        end = time.time() + seconds
        while time.time() < end:
            batch = []
            for _ in range(2):
                sample = self.data[self.i % len(self.data)]
                self.i += 1
                batch.append(((sample - self.mean) * scale + self.mean).tolist())
            stamps = [self.t + 0.01 * (k + 1) for k in range(len(batch))]
            self.t = stamps[-1]
            monitor.process_sensor_data(batch, stamps)
            time.sleep(0.02)


def start_long_cut(length=40):
    program = f"G21\nG90\nM3 S20000\nG0 Z5\nG0 X0 Y0\nG1 Z-0.2 F100\nG1 X{length} F100\nG0 Z5\nM5\n"
    emit("load_nc_file", {"content": program, "fileName": "long_cut.nc"})
    emit("start_gcode")
    return wait_for(lambda: monitor.is_running and (cnc_app.last_grbl_status or {}).get('state') == 'Run'
                    and cnc_app.machine_snapshot()['z'] <= -0.1, 20)


print("== online mitigation: chatter in a cut -> override change -> vibration settles -> job continues ==")
sim.time_scale = 1.0
wait_for(lambda: cnc_app.state in ("idle", "alarm_unlocked"), 10)
check("cut started (Run, Z<0, spindle on)", start_long_cut())
feeder = Feeder(normal)
feeder.run(3.0)
check("gate on and windows scored while cutting", monitor.latest.get('gate') is True and monitor.latest.get('evidence') is not None,
      str({k: monitor.latest.get(k) for k in ('gate', 'evidence', 'rms')}))
quiet_evidence = monitor.latest.get('evidence')
t0 = time.time()
feeder.run(2.5, scale=3.0)
check("alarm -> online mitigation, machine keeps cutting",
      monitor.event is not None and monitor.event.get('stage') == 'mitigating' and cnc_app.state == "running",
      f"event={monitor.event and monitor.event.get('stage')} state={cnc_app.state} evidence={monitor.latest.get('evidence')}")
check("overrides applied: spindle +10% (110%), feed -20% (80%)",
      wait_for(lambda: sim.spindle_ov == 110 and sim.feed_ov == 80, 5), f"spindle_ov={sim.spindle_ov} feed_ov={sim.feed_ov}")
feeder.run(3.0)
msgs = logs_since(t0)
check("vibration settles -> mitigation succeeds, job continues", any("안정되어" in m for m in msgs) and cnc_app.job_active
      and cnc_app.state == "running" and monitor.event is None, f"quiet evidence={quiet_evidence}")
with open(os.path.join(COPY, "data", "Vibration_detection.csv"), encoding="utf-8", newline="") as f:
    det_rows = list(csv.reader(f))
check("detection log records the mitigation", det_rows[-1][-1] == "mitigate" and det_rows[-1][8] != "", str(det_rows[-1]))
sim.time_scale = 20.0
wait_for(lambda: not cnc_app.job_active, 30)

print("== persistent chatter: mitigation fails -> position-keeping stop -> rework ==")
sim.time_scale = 1.0
wait_for(lambda: cnc_app.state in ("idle", "alarm_unlocked"), 10)
mtime_before = os.path.getmtime(rework_path)
resets_before = sim.resets
check("cut started", start_long_cut())
feeder.run(3.0)
t0 = time.time()
feeder.run(7.0, scale=3.0)
stopped = wait_for(lambda: cnc_app.state == "emergency" and not cnc_app.job_active, 10)
msgs = logs_since(t0)
check("mitigation tried first, then escalated to a stop",
      any("온라인 대응:" in m for m in msgs) and any("온라인 대응 후에도" in m for m in msgs) and stopped, cnc_app.state)
check("escalation stop kept position (no ALARM:3)", sim.resets == resets_before + 1 and not sim.alarm)
check("rework.nc created and started automatically",
      wait_for(lambda: os.path.getmtime(rework_path) > mtime_before, 30) and wait_for(lambda: cnc_app.job_active, 15))
sim.time_scale = 20.0
wait_for(lambda: not cnc_app.job_active, 60)
check("rework after escalation finished", cnc_app.state in ("idle", "alarm_unlocked") and not sim.alarm, cnc_app.state)

print("== detector settings API (온라인 대응·판정 방식) ==")
r = http.post("/api/vibration/settings", json={'mitigation_enabled': False}).get_json()
check("mitigation can be switched off and is saved", r["status"] == "success" and r["model"]["settings"]["mitigation_enabled"] is False
      and os.path.isfile(os.path.join(COPY, "data", "detector_settings.json")), r["message"])
r = http.post("/api/vibration/settings", json={'decision_mode': 'bogus'}).get_json()
check("invalid decision mode rejected", r["status"] == "error", r["message"])
r = http.post("/api/vibration/settings", json={'mitigation_enabled': True, 'decision_mode': 'cusum'}).get_json()
check("cusum mode accepted", r["status"] == "success" and monitor.config['decision_mode'] == 'cusum')
http.post("/api/vibration/settings", json={'decision_mode': 'k_of_n'})


class FakeFastSensor:
    """Fast-firmware serial stream (#H header + 'A,seq,ax,ay,az' at fs Hz), generated in real time."""

    def __init__(self, fs=1000):
        self.fs, self.is_open, self.seq = fs, True, 0
        self.buf = bytearray(b"#H,device=MPU6050,fs=1000.0,range_g=8,dlpf_hz=184\n")
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        start = time.time()
        rng = np.random.default_rng(0)
        while self.is_open:
            target = int((time.time() - start) * self.fs)
            lines = []
            while self.seq < target:
                ax, ay, az = (rng.normal(0, 300, 3) + [0, 4096, 0]).astype(int)
                lines.append(f"A,{self.seq},{ax},{ay},{az}\n")
                self.seq += 1
            with self.lock:
                self.buf.extend("".join(lines).encode())
            time.sleep(0.005)

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.buf)

    def read(self, n=1):
        with self.lock:
            out = bytes(self.buf[:n])
            del self.buf[:n]
            return out

    def close(self):
        self.is_open = False


print("== data collection: fast sensor stream + machine log + labels -> data/raw -> build_dataset ==")
sim.time_scale = 1.0
wait_for(lambda: cnc_app.state in ("idle", "alarm_unlocked"), 10)
fake_sensor = FakeFastSensor()
cnc_app.sensor_stream = SensorStream()
cnc_app.arduino_serial = fake_sensor
captured = []
real_process = monitor.process_sensor_data
monitor.process_sensor_data = lambda samples, stamps=None: captured.append(list(stamps or []))
threading.Thread(target=cnc_app.read_arduino_serial, daemon=True).start()
experiment = ("(META run_id=E2E_001 type=grid S=20000 F=100 ap=0.2 ae=6 tool=T1 tool_d=6 flutes=2 material=AL6061)\n"
              "G21 G90 G94\nG0 Z5\nM3 S20000\nG4 P1\nG0 X0 Y0\nG1 Z-0.2 F100\nG1 X8 F100\nG0 Z5\nM5\n")
emit("load_nc_file", {"content": experiment, "fileName": "E2E_001.nc"})
status = http.get("/api/recording/status").get_json()["recording"]
check("META comment of the loaded program is offered as defaults", status["program_meta"].get("run_id") == "E2E_001"
      and status["program_meta"].get("flutes") == "2", str(status["program_meta"]))
r = http.post("/api/recording/start", json={'notes': 'e2e run', 'stickout_mm': '20'}).get_json()
check("recording started with run_id from META", r["status"] == "success" and r["recording"]["run_id"] == "E2E_001", r["message"])
check("tool flutes from META used by the detector", cnc_app.tool_flutes == 2)
t_events = time.time()
emit("start_gcode")
wait_for(lambda: (cnc_app.last_grbl_status or {}).get('state') == 'Run' and cnc_app.machine_snapshot()['z'] <= -0.1, 20)
time.sleep(1.0)
r1 = http.post("/api/recording/mark", json={'label': 'chatter', 'edge': 'start'}).get_json()
time.sleep(1.5)
r2 = http.post("/api/recording/mark", json={'label': 'chatter', 'edge': 'end', 'note': 'test'}).get_json()
check("chatter segment marked", r1["status"] == "success" and r2["status"] == "success", f"{r1['message']} / {r2['message']}")
wait_for(lambda: not cnc_app.job_active, 30)
r = http.post("/api/recording/stop").get_json()
ui_events = [d for (t, n, d) in drain() if n == "arduino_data" and t >= t_events]
elapsed = time.time() - t_events
check("recording stopped and summarised", r["status"] == "success" and r["recording"]["active"] is False, r["message"])
raw_dir = os.path.join(COPY, "data", "raw")
sensor_df = pd.read_csv(os.path.join(raw_dir, "E2E_001_sensor.csv"))
machine_df = pd.read_csv(os.path.join(raw_dir, "E2E_001_machine.csv"))
with open(os.path.join(raw_dir, "manifest.csv"), encoding="utf-8-sig", newline="") as f:
    manifest = list(csv.DictReader(f))
dt = np.diff(sensor_df["DeviceTime"].to_numpy(float))
check("sensor file: ~1 kHz device timestamps, continuous Seq, values scaled to g (±8 g range)",
      len(sensor_df) > 800 * (sensor_df["HostTime"].iloc[-1] - sensor_df["HostTime"].iloc[0])
      and np.allclose(dt, 0.001, atol=1e-6) and (np.diff(sensor_df["Seq"]) == 1).all()
      and abs(sensor_df["AccY"].mean() - 1.0) < 0.05, f"rows={len(sensor_df)}")
check("machine log: GRBL Run rows at Z=-0.2 with spindle/feed", ((machine_df["GrblState"] == "Run") & (machine_df["Z"] < -0.1)).any()
      and machine_df["SpindleRPM"].max() >= 19000)
check("manifest row: conditions, tool, sensor, labels",
      manifest[-1]["run_id"] == "E2E_001" and manifest[-1]["run_type"] == "grid" and manifest[-1]["S_rpm"] == "20000"
      and manifest[-1]["flutes"] == "2" and manifest[-1]["stickout_mm"] == "20" and manifest[-1]["sensor_fs_hz"].startswith("1000")
      and json.loads(manifest[-1]["labels"])[0]["label"] == "chatter", str({k: manifest[-1][k] for k in ('run_id', 'run_type', 'S_rpm', 'sensor_fs_hz')}))
flat = [t for batch in captured for t in batch]
check("detector received device timestamps (monotonic, 1 ms apart)", len(flat) > 1000 and np.allclose(np.diff(flat[:2000]), 0.001, atol=1e-6))
check("detector knows the stream (1000 Hz, ±8 g)", monitor.sensor_info.get('fs') == 1000.0 and monitor.sensor_info.get('range_g') == 8.0)
check("screen updates are rate-limited (~20/s, not 1000/s)", 0 < len(ui_events) <= 25 * elapsed + 5, f"{len(ui_events)} events in {elapsed:.1f}s")
monitor.process_sensor_data = real_process
fake_sensor.close()
cnc_app.arduino_serial = None
proc = subprocess.run([sys.executable, os.path.join(COPY, "scripts", "build_dataset.py")], capture_output=True, text=True, encoding="utf-8")
windows_df = pd.read_csv(os.path.join(COPY, "data", "dataset", "windows.csv"), encoding="utf-8-sig") if proc.returncode == 0 else None
check("build_dataset: windows with gate/labels from the machine log and marks", windows_df is not None
      and windows_df["gate"].sum() > 5 and (windows_df["label"] == "chatter").sum() >= 2
      and windows_df[windows_df["gate"] == 1]["n"].median() >= 19000, (proc.stdout + proc.stderr)[-300:])

print("== old-format Vibration_detection.csv is kept as .old and a CSV log starts ==")
legacy = os.path.join(COPY, "data", "legacy_detection.csv")
with open(legacy, "w", encoding="utf-8") as f:
    f.write("Abnormal vibration detected at 2025-11-28 18:05:07\nCurrent position: X=0.000, Y=70.000, Z=0.840\nSpindle speed: 100%\n")
vibration_monitor.append_detection_record((1, 2, 3), None, None, 100, 100, path=legacy)
with open(legacy, encoding="utf-8", newline="") as f:
    rows = list(csv.reader(f))
check("legacy text moved to .old, new file has header + 1 row (unknown RPM/F left empty)",
      os.path.isfile(legacy + ".old") and len(rows) == 2 and rows[1][1:8] == ["1.000", "2.000", "3.000", "", "", "100", "100"], str(rows))

print("== vibration model management API (진동 감지 모델 창) ==")
train_dir = os.path.join(COPY, "data", "학습 데이터")
resp = http.get("/api/vibration/model").get_json()
model = resp.get("model", {})
n_files = len(model.get("files", []))
check("status: trained on the normal files only", resp["status"] == "success" and model["is_trained"] and n_files == N_NORMAL_FILES
      and not any("이상" in name for name in model["files"]) and model["windows"] > 0 and model["thresholds"]["rms_g"] > 0
      and model["thresholds"]["calibration"] == "session_holdout",
      str({k: v for k, v in model.items() if k != "files"}))
with open(sorted(glob.glob(os.path.join(train_dir, "*.csv")))[2], "rb") as f:
    sample_bytes = f.read()


def upload(name, content):
    return http.post("/api/vibration/train", data={"train_file": (io.BytesIO(content), name)},
                     content_type="multipart/form-data").get_json()


cnc_app.vibration_detection_active = True
r1 = upload("during_job.csv", sample_bytes)
r2 = http.post("/api/vibration/reset").get_json()
cnc_app.vibration_detection_active = False
check("training/reset refused while detection is active (job running)",
      r1["status"] == "error" and r2["status"] == "error" and "가공 중" in r1["message"], r1["message"])
for name, content, label in [
    ("notes.txt", sample_bytes, "non-CSV"),
    ("센서_이상.csv", sample_bytes, "'이상' (abnormal) file"),
    ("master_dataset.csv", sample_bytes, "reserved master_dataset.csv"),
    ("bad.csv", b"a,b,c\n1,2,3\n", "CSV without AccX/AccY/AccZ"),
    ("short.csv", ("AccX,AccY,AccZ\n" + "0.1,0.2,0.3\n" * 50).encode(), "fewer rows than one window"),
]:
    r = upload(name, content)
    check(f"{label} rejected", r["status"] == "error", r["message"])
check("rejected uploads left the folder untouched", len([n for n in os.listdir(train_dir) if n.endswith(".csv")]) == len(TRAIN_CSVS))
t_train = time.time()
r = upload("../../추가_정상.csv", sample_bytes)
t_train = time.time() - t_train
check("valid upload stored only inside the training folder and model retrained",
      r["status"] == "success" and os.path.isfile(os.path.join(train_dir, "추가_정상.csv"))
      and not os.path.exists(os.path.join(COPY, "추가_정상.csv")) and len(r["model"]["files"]) == n_files + 1,
      f"{r['message']} ({t_train:.1f}s)")
r = upload("추가_정상.csv", sample_bytes)
check("same name is not overwritten (stored as _1)", r["status"] == "success" and os.path.isfile(os.path.join(train_dir, "추가_정상_1.csv")), r["message"])
cp949_csv = ("시간,AccX,AccY,AccZ\n" + "".join(f"가,{x},{y},{z}\n" for x, y, z in normal[:400])).encode("cp949")
r = upload("cp949_정상.csv", cp949_csv)
with open(os.path.join(train_dir, "cp949_정상.csv"), encoding="utf-8") as f:
    first = f.readline().strip()
check("CP949 CSV (Excel on Korean Windows) accepted and stored as UTF-8", r["status"] == "success" and first == "시간,AccX,AccY,AccZ", r["message"])
recorded = os.path.join(COPY, "data", "raw", "E2E_001_sensor.csv")
with open(recorded, "rb") as f:
    r = upload("E2E_001_sensor.csv", f.read())
check("a data-collection sensor file (1 kHz, DeviceTime) can be added for training", r["status"] == "success", r["message"])
monitor._buf_t.extend([1.0, 1.01])
monitor._buf_a.extend([(0.0, 0.0, 0.0)] * 2)
r = http.post("/api/vibration/reset").get_json()
check("reset retrains from the folder and clears the detection buffer",
      r["status"] == "success" and len(r["model"]["files"]) == n_files + 4 and len(monitor._buf_t) == 0, r["message"])
windows_normal = features.session_windows(normal, np.arange(len(normal)) / 100.0, monitor.model['feature_config'])
evidence = monitor.core.score(windows_normal[5][1])[0]
check("scoring works with the retrained model", np.isfinite(evidence), f"evidence={evidence:.2f}")
print(f"    one retrain took {t_train:.1f}s")

print("== controller unplugged while idle -> port closed after repeated serial errors ==")
cnc_app.alarm_check_interval = 0.3


def unplugged(data):
    raise cnc_app.serial.SerialException("device disconnected")


sim.write = unplugged
t0 = time.time()
check("port closed after repeated serial errors", wait_for(lambda: cnc_app.ser is None, 15))
check("UI told the controller is disconnected",
      any(n == "connection_status" and isinstance(d, dict) and d.get("connected") is False and t >= t0 for (t, n, d) in drain()))

print(f"\ne2e checks: {sum(results)}/{len(results)} passed", flush=True)
os._exit(0 if all(results) else 1)
