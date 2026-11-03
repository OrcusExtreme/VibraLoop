"""Benchmark: evaluate the detector on the synthetic scenarios and on the legacy training data, and print the
results next to the stored baseline (tests/baseline/<tag>_synthetic.json, <tag>_stress.json, <tag>_legacy.json).

Synthetic sets (same 28 scenarios, seed 1):
  synthetic  default sensor model: DLPF 184 Hz, spindle turns at the commanded speed
  stress     DLPF 260 Hz (more aliasing) and a spindle that turns 2 % faster than commanded
Each synthetic set is evaluated with the default decision (k_of_n) and with decision_mode=two_tier
(Fast-Trip + CUSUM), and both are printed next to the same v1.1 baseline.
Condition model (P09): for each set a synthetic grid test (make_synthetic_runs.py --grid, same sensor options) is
recorded, turned into a dataset and used to train a condition model (train_models.py) - the same steps a real grid
experiment goes through. Both decisions are then evaluated again with that model (--condition-model). A condition model
belongs to its sensor setup, so each set uses the grid recorded with its own DLPF / spindle-speed error.
Stability limit (P10): 18 synthetic ramp tests (make_synthetic_runs.py --ramp, known limit per run) go through
build_dataset (onset re-detection, censoring) and train_models (SLD GPR). The corrected onset depths and the safe depth on a
250 rpm grid are compared with the synthetic truth (bench/ramp.json).
Optimizer (P11): for every chattering condition (depth above the true limit; 250 rpm x 0.05 mm grid) the online and rework
choices made with that model are checked against the true limit curve and compared with the default rule (bench/optimizer.json).

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


def evaluate(kind, args, name=None):
    """Evaluate and compare with the baseline of `kind`; the result is stored as bench/<name or kind>.json."""
    out_json = os.path.join(OUT, f"{name or kind}.json")
    baseline = os.path.join(TESTS_DIR, 'baseline', f"{BASELINE_TAG}_{kind}.json")
    extra = ['--compare', baseline] if os.path.exists(baseline) else []
    proc = script('evaluate_detector.py', *args, '--json', out_json, *extra)
    print(proc.stdout, flush=True)
    if proc.returncode != 0:
        print(proc.stderr[-2000:], flush=True)
    result = json.load(open(out_json, encoding='utf-8')) if os.path.exists(out_json) else None
    return proc, result, bool(extra)


def last_line(proc):
    lines = (proc.stdout + proc.stderr).strip().splitlines()
    return lines[-1] if lines else ''


SETS = [('synthetic', os.path.join(COPY, 'data', 'synthetic'), []),
        ('stress', os.path.join(COPY, 'data', 'synthetic', 'stress'), ['--dlpf', '260', '--rpm-error', '0.02'])]
for kind, folder, options in SETS:
    raw, dataset = os.path.join(folder, 'raw'), os.path.join(folder, 'dataset')
    print(f"== {kind} scenarios (seed 1{', ' + ' '.join(options) if options else ''}) ==", flush=True)
    gen = script('make_synthetic_runs.py', '--out', raw, *options)
    check(f"{kind}: runs generated", gen.returncode == 0, last_line(gen))
    build = script('build_dataset.py', '--raw', raw, '--out', dataset)
    check(f"{kind}: dataset built", build.returncode == 0, last_line(build))
    proc, result, compared = evaluate(kind, ['--dataset', dataset])
    check(f"{kind}: evaluation finished with label-based events", proc.returncode == 0 and result is not None
          and result['events_source'] == 'labels' and result['overall']['events'] > 0)
    check(f"{kind}: compared with baseline {BASELINE_TAG}_{kind}.json", compared, "" if compared else "baseline file not found")
    print(f"-- {kind}: decision_mode=two_tier (Fast-Trip + CUSUM) --", flush=True)
    proc, result, _ = evaluate(kind, ['--dataset', dataset, '--set', 'decision_mode=two_tier'], f"{kind}_two_tier")
    check(f"{kind}: two-tier evaluation finished", proc.returncode == 0 and result is not None
          and result['config']['decision_mode'] == 'two_tier')

    print(f"-- {kind}: synthetic grid test -> condition model (P09) --", flush=True)
    grid_raw, grid_dataset = os.path.join(folder, 'grid', 'raw'), os.path.join(folder, 'grid', 'dataset')
    condition_model = os.path.join(OUT, f"{kind}_condition_model.joblib")
    gen = script('make_synthetic_runs.py', '--grid', '--out', grid_raw, *options)
    build = script('build_dataset.py', '--raw', grid_raw, '--out', grid_dataset)
    train = script('train_models.py', '--dataset', grid_dataset, '--skip-detector', '--skip-sld', '--condition-out', condition_model)
    print("    " + "\n    ".join(line for line in train.stdout.splitlines() if line.startswith(('[', '  '))), flush=True)
    check(f"{kind}: grid runs recorded, dataset built, condition model trained with a held-out coverage report",
          gen.returncode == 0 and build.returncode == 0 and train.returncode == 0 and os.path.isfile(condition_model)
          and "상한 안에 든 비율" in train.stdout, last_line(train))
    for decision, name in (([], f"{kind}_condition"), (['--set', 'decision_mode=two_tier'], f"{kind}_two_tier_condition")):
        print(f"-- {kind}: {'two_tier' if decision else 'k_of_n'} with the condition model --", flush=True)
        proc, result, _ = evaluate(kind, ['--dataset', dataset, *decision, '--condition-model', condition_model], name)
        check(f"{name}: evaluation with the condition model finished", proc.returncode == 0 and result is not None
              and result['condition_model'] is not None)

print("== synthetic ramp tests -> onset correction -> stability limit model (P10) ==", flush=True)
ramp_raw = os.path.join(COPY, 'data', 'synthetic', 'ramp', 'raw')
ramp_dataset = os.path.join(COPY, 'data', 'synthetic', 'ramp', 'dataset')
sld_path = os.path.join(OUT, 'sld_model.joblib')
gen = script('make_synthetic_runs.py', '--ramp', '--out', ramp_raw)
build = script('build_dataset.py', '--raw', ramp_raw, '--out', ramp_dataset)
train = script('train_models.py', '--dataset', ramp_dataset, '--skip-detector', '--skip-condition', '--sld-out', sld_path)
print("    " + "\n    ".join(line for line in build.stdout.splitlines() if '안정 한계' in line or 'ramp ' in line), flush=True)
print("    " + "\n    ".join(line for line in train.stdout.splitlines() if line.startswith(('[', '  '))), flush=True)
check("ramp: 18 runs recorded, dataset built, stability limit model trained", gen.returncode == 0 and build.returncode == 0
      and train.returncode == 0 and os.path.isfile(sld_path), last_line(train))
if os.path.isfile(sld_path):
    # 합성 데이터라서 정답(실행마다의 한계, 회전수별 한계 곡선)과 비교할 수 있습니다
    import numpy as np
    import pandas as pd
    sys.path[:0] = [os.path.join(COPY, 'src'), os.path.join(COPY, 'scripts')]
    import optimizer
    from make_synthetic_runs import synthetic_limit
    limits = pd.read_csv(os.path.join(ramp_dataset, 'ramp_limits.csv'), encoding='utf-8-sig')
    truth = pd.read_csv(os.path.join(ramp_raw, 'truth.csv'), encoding='utf-8-sig')[['run_id', 'limit_mm', 'censored']]
    rows = limits.merge(truth, on='run_id', suffixes=('', '_truth'))
    chatter = rows[rows['censored'] == 0]
    label_error = chatter['label_ap'] - chatter['limit_mm']
    onset_error = chatter['onset_ap'] - chatter['limit_mm']
    model = optimizer.load_sld_model(sld_path)
    speeds = np.arange(16000, 24001, 250)
    safe = np.array([optimizer.sld_safe_depth(model, s)['ap_safe'] for s in speeds])
    true_limit = np.array([synthetic_limit(s) for s in speeds])
    print(f"  onset depth vs true limit ({len(chatter)} chattering runs): label {label_error.mean():+.3f} mm (max {label_error.abs().max():.3f}), "
          f"corrected {onset_error.mean():+.3f} mm (max {onset_error.abs().max():.3f}), "
          f"censored {int(rows['censored'].sum())}/{len(rows)} (truth {int(rows['censored_truth'].sum())})", flush=True)
    print(f"  safe depth vs true limit curve, every 250 rpm: below the limit at {np.mean(safe <= true_limit):.0%} of speeds, "
          f"mean margin {np.mean(true_limit - safe):.3f} mm, closest {np.min(true_limit - safe):+.3f} mm", flush=True)
    for s in range(16000, 24001, 1000):
        limit = optimizer.sld_safe_depth(model, s)
        print(f"    {s} rpm: true {synthetic_limit(s):.3f}, mean {limit['mu']:.3f}, safe {limit['ap_safe']:.3f} mm", flush=True)
    summary = {'label_error_mm': {'mean': float(label_error.mean()), 'max_abs': float(label_error.abs().max())},
               'onset_error_mm': {'mean': float(onset_error.mean()), 'max_abs': float(onset_error.abs().max())},
               'censored': int(rows['censored'].sum()), 'grid_safe_fraction': float(np.mean(safe <= true_limit)),
               'grid_mean_margin_mm': float(np.mean(true_limit - safe)), 'cv': model['cv']}
    with open(os.path.join(OUT, 'ramp.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    check("ramp: onset correction and safe depth compared with the synthetic truth (bench/ramp.json)",
          len(chatter) > 0 and rows['censored'].equals(rows['censored_truth']))

    print("== optimizer choices from the stability limit model vs the synthetic truth (P11) ==", flush=True)
    # 채터가 나는 조건(깊이 > 참 한계)마다 고른 조건이 참 한계 곡선에서 안정한지(재가공은 패스당 깊이) 보고 기본 규칙과 비교합니다
    sld_opt = optimizer.CuttingConditionOptimizer(model_path=sld_path)
    rule_opt = optimizer.CuttingConditionOptimizer(model_path=os.path.join(OUT, 'no_model.joblib'))
    tool = {'flutes': model['flutes'], 'tool_id': model.get('tool_id')}
    cases = [(n, round(float(ap), 2)) for n in range(16000, 24001, 250) for ap in np.arange(0.15, 0.551, 0.05)
             if ap > synthetic_limit(n) + 1e-9]
    choices = {}
    for name, opt, purpose, min_passes, extra in (
            ('sld_online', sld_opt, 'online', 1, tool), ('rule_online', rule_opt, 'online', 1, {}),
            ('sld_rework', sld_opt, 'rework', 2, tool), ('rule_rework', rule_opt, 'rework', 2, {}),
            ('sld_rework_min1', sld_opt, 'rework', 1, tool)):
        picks = [(ap, opt.suggest(n, 100, ap, purpose=purpose, min_passes=min_passes, **extra)) for n, ap in cases]
        stable = [ap / (r['passes'] if purpose == 'rework' else 1) <= synthetic_limit(r['n']) + 1e-9 for ap, r in picks]
        used = [r['method'] == 'sld' for _, r in picks]
        result = {'cases': len(cases), 'min_passes': min_passes, 'stable': int(sum(stable)), 'sld_used': int(sum(used)),
                  'stable_when_sld': int(sum(s for s, u in zip(stable, used) if u)),
                  'mean_passes': float(np.mean([r['passes'] for _, r in picks])),
                  'mean_time': float(np.mean([r['passes'] * 100 / r['F'] for _, r in picks]))}
        choices[name] = result
        print(f"  {name}: truly stable {result['stable']}/{result['cases']}"
              + (f" (model used {result['sld_used']}, stable {result['stable_when_sld']})" if name.startswith('sld') else "")
              + f", mean passes {result['mean_passes']:.2f}, mean time {result['mean_time']:.2f} "
                f"(one pass at the current feed = 1)", flush=True)
    with open(os.path.join(OUT, 'optimizer.json'), 'w', encoding='utf-8') as f:
        json.dump(choices, f, ensure_ascii=False, indent=1)
    check("optimizer: online and rework choices from the stability limit model compared with the synthetic truth "
          "(bench/optimizer.json)", choices['sld_online']['sld_used'] > 0 and choices['sld_rework']['sld_used'] > 0)

print("== legacy training data (data/학습 데이터) ==", flush=True)
proc, legacy, compared = evaluate('legacy', ['--legacy'])
check("legacy evaluation finished", proc.returncode == 0 and legacy is not None and legacy['normal']['files'] > 0)
check(f"compared with baseline {BASELINE_TAG}_legacy.json", compared, "" if compared else "baseline file not found")

print(f"\nbench checks: {sum(results)}/{len(results)} passed", flush=True)
sys.exit(0 if all(results) else 1)
