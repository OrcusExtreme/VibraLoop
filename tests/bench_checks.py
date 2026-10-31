"""Benchmark: evaluate the detector on the synthetic scenarios and on the legacy training data, and print the
results next to the stored baseline (tests/baseline/<tag>_synthetic.json, <tag>_legacy.json).

Runs on a throw-away copy of the project made by run_tests.py:
  python tests/run_tests.py bench
Numbers that differ from the baseline are not failures (changing the detector is supposed to change them); this suite
only fails when the evaluation pipeline itself breaks. Changed metrics are marked with * in the comparison table.
The synthetic runs use a fixed seed, so the same code gives the same numbers.
To store a new baseline: run with --keep, then copy <copy>/bench/*.json to tests/baseline/ under a new tag.
"""
import json
import os
import subprocess
import sys

if len(sys.argv) < 2 or not os.path.isdir(sys.argv[1]):
    sys.exit("usage: python tests/run_tests.py bench  (this script needs a project copy as its argument)")
COPY = os.path.abspath(sys.argv[1])
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
BASELINE_TAG = 'v1.1'
OUT = os.path.join(COPY, 'bench')

results = []


def check(name, cond, detail=""):
    results.append(bool(cond))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""), flush=True)


def script(name, *args):
    proc = subprocess.run([sys.executable, os.path.join(COPY, 'scripts', name), *args], cwd=COPY, capture_output=True,
                          text=True, encoding='utf-8', errors='replace')
    return proc


def evaluate(kind, args):
    out_json = os.path.join(OUT, f"{kind}.json")
    baseline = os.path.join(TESTS_DIR, 'baseline', f"{BASELINE_TAG}_{kind}.json")
    extra = ['--compare', baseline] if os.path.exists(baseline) else []
    proc = script('evaluate_detector.py', *args, '--json', out_json, *extra)
    print(proc.stdout, flush=True)
    if proc.returncode != 0:
        print(proc.stderr[-2000:], flush=True)
    result = json.load(open(out_json, encoding='utf-8')) if os.path.exists(out_json) else None
    return proc, result, bool(extra)


raw = os.path.join(COPY, 'data', 'synthetic', 'raw')
dataset = os.path.join(COPY, 'data', 'synthetic', 'dataset')
print("== synthetic scenarios (fixed seed) ==", flush=True)
gen = script('make_synthetic_runs.py', '--out', raw)
check("synthetic runs generated", gen.returncode == 0, (gen.stdout + gen.stderr).strip().splitlines()[-1] if gen.stdout or gen.stderr else '')
build = script('build_dataset.py', '--raw', raw, '--out', dataset)
check("dataset built", build.returncode == 0, (build.stdout + build.stderr).strip().splitlines()[-1] if build.stdout or build.stderr else '')
proc, synthetic, compared = evaluate('synthetic', ['--dataset', dataset])
check("synthetic evaluation finished with label-based events", proc.returncode == 0 and synthetic is not None
      and synthetic['events_source'] == 'labels' and synthetic['overall']['events'] > 0)
check(f"compared with baseline {BASELINE_TAG}_synthetic.json", compared, "" if compared else "baseline file not found")

print("== legacy training data (data/학습 데이터) ==", flush=True)
proc, legacy, compared = evaluate('legacy', ['--legacy'])
check("legacy evaluation finished", proc.returncode == 0 and legacy is not None and legacy['normal']['files'] > 0)
check(f"compared with baseline {BASELINE_TAG}_legacy.json", compared, "" if compared else "baseline file not found")

print(f"\nbench checks: {sum(results)}/{len(results)} passed", flush=True)
sys.exit(0 if all(results) else 1)
