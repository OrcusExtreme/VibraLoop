"""데이터 수집 실험용 NC 파일 생성 (설계: edit_analysis.md 3.2)

계획 종류
  ramp : 회전수마다 홈을 깊이 0 → --ramp-max로 점점 깊게 한 번씩 (채터가 시작되는 깊이 = 안정 한계 탐색)
  grid : 회전수 × 깊이 × 이송 조합을 무작위 순서로, --reference 조건을 --reference-every컷마다 반복
  air  : 회전수마다 소재 위(Z+1)에서 같은 경로를 이동 (스핀들 자체 진동 기준값)

각 파일 첫머리에 (META run_id=... type=... S=... F=... ap=... ae=... tool=... tool_d=... flutes=...) 주석이 있어,
화면에서 이 파일을 불러오면 데이터 수집 창의 조건 칸이 자동으로 채워집니다. 실행 순서는 plan.csv에 남깁니다.

예)
  python scripts/make_experiment_nc.py ramp --tool-d 6 --flutes 2 --ramp-max 1.0
  python scripts/make_experiment_nc.py grid --tool-d 6 --flutes 2 --depths 0.2,0.4,0.6,0.8 --feeds 50,100,150
  python scripts/make_experiment_nc.py grid --limits 16000=0.5,18000=0.7 --depth-factors 0.6,0.9,1.1,1.4 ...
홈은 Y 방향으로 (공구 지름 + 여유)만큼 띄워 겹치지 않게 배치하고, --stock-y를 넘으면 X 방향 다음 열로 넘깁니다.
작업 좌표 원점(X0 Y0 Z0)은 소재 모서리 윗면으로 맞춘 뒤 사용하세요. 공구 파손을 막기 위해 깊이는 공구 지름의
--max-depth-ratio배(기본 0.5)까지만 허용합니다.
"""
import argparse
import csv
import os
import random
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def number_list(text, cast=float):
    """'16000:24000:1000' 또는 '16000,18000' → 숫자 목록"""
    text = str(text).strip()
    if ':' in text:
        start, stop, step = (float(v) for v in text.split(':'))
        values, value = [], start
        while value <= stop + 1e-9:
            values.append(cast(round(value, 6)))
            value += step
        return values
    return [cast(v) for v in text.split(',') if v.strip()]


def parse_limits(text):
    """'16000=0.5,18000=0.7' → {16000: 0.5, 18000: 0.7}"""
    limits = {}
    for item in (text or '').split(','):
        if '=' in item:
            rpm, depth = item.split('=', 1)
            limits[int(float(rpm))] = float(depth)
    return limits


def meta_comment(meta):
    return "(META " + " ".join(f"{k}={v}" for k, v in meta.items() if v not in (None, '')) + ")"


def slot_program(meta, x0, y, length, depth, feed, plunge_feed, safe_z, rpm, ramp=False, air=False):
    """한 실행의 G-code (안전 높이 → 스핀들 가동·대기 → 진입 → 절삭 → 복귀)"""
    lines = [meta_comment(meta), "G21 G90 G94", f"G0 Z{safe_z:g}", f"M3 S{int(rpm)}",
             "G4 P2", f"G0 X{x0:.3f} Y{y:.3f}"]
    if air:
        lines += ["G1 Z1.000 F{0}".format(int(plunge_feed)), f"G1 X{x0 + length:.3f} F{int(feed)}"]
    elif ramp:
        # 소재 윗면(Z0)에서 시작해 X 방향으로 가며 깊이가 0 → depth로 선형 증가
        lines += [f"G1 Z0.000 F{int(plunge_feed)}", f"G1 X{x0 + length:.3f} Z{-depth:.3f} F{int(feed)}"]
    else:
        lines += [f"G1 Z{-depth:.3f} F{int(plunge_feed)}", f"G1 X{x0 + length:.3f} F{int(feed)}"]
    lines += [f"G0 Z{safe_z:g}", "M5"]
    return lines


def main():
    parser = argparse.ArgumentParser(description="데이터 수집 실험용 NC 파일 생성")
    parser.add_argument('plan', choices=['ramp', 'grid', 'air'])
    parser.add_argument('--name', help="계획 이름 (출력 폴더 이름, 기본: 계획 종류)")
    parser.add_argument('--out', default=os.path.join(BASE_DIR, 'nc', 'experiments'))
    parser.add_argument('--rpm', default='16000:24000:1000', help="회전수 목록 (예: 16000:24000:1000 또는 16000,20000)")
    parser.add_argument('--feeds', default='50,100,150', help="이송 목록 (mm/min)")
    parser.add_argument('--depths', default='', help="grid: 깊이 목록 (mm)")
    parser.add_argument('--limits', default='', help="grid: ramp로 찾은 회전수별 안정 한계 (예: 16000=0.5,18000=0.7)")
    parser.add_argument('--depth-factors', default='0.6,0.9,1.1,1.4', help="grid: --limits에 곱할 비율")
    parser.add_argument('--ramp-max', type=float, default=1.0, help="ramp: 최대 깊이 (mm)")
    parser.add_argument('--ramp-feed', type=float, default=100.0)
    parser.add_argument('--air-feed', type=float, default=100.0)
    parser.add_argument('--tool-d', type=float, required=True, help="공구 지름 (mm)")
    parser.add_argument('--flutes', type=int, required=True, help="공구 날 수")
    parser.add_argument('--tool-id', default='T1')
    parser.add_argument('--material', default='')
    parser.add_argument('--length', type=float, default=40.0, help="홈 길이 (mm)")
    parser.add_argument('--x0', type=float, default=5.0)
    parser.add_argument('--y0', type=float, default=5.0)
    parser.add_argument('--gap', type=float, default=2.0, help="홈 사이 여유 (mm)")
    parser.add_argument('--stock-x', type=float, default=150.0)
    parser.add_argument('--stock-y', type=float, default=100.0)
    parser.add_argument('--plunge-feed', type=float, default=50.0)
    parser.add_argument('--safe-z', type=float, default=5.0)
    parser.add_argument('--reference', default='20000,100,0.2', help="grid: 기준 조건 '회전수,이송,깊이' (빈 값이면 생략)")
    parser.add_argument('--reference-every', type=int, default=10)
    parser.add_argument('--seed', type=int, default=1, help="grid 순서를 섞는 난수 시드")
    parser.add_argument('--max-depth-ratio', type=float, default=0.5)
    parser.add_argument('--force', action='store_true', help="깊이 제한을 무시 (공구 파손 위험)")
    args = parser.parse_args()

    rpms = number_list(args.rpm, int)
    feeds = number_list(args.feeds)
    max_depth = args.tool_d * args.max_depth_ratio
    ae = args.tool_d  # 홈 가공(전체 폭)

    runs = []
    if args.plan == 'ramp':
        for rpm in rpms:
            runs.append({'type': 'ramp', 'S': rpm, 'F': args.ramp_feed, 'ap': args.ramp_max})
    elif args.plan == 'air':
        for rpm in rpms:
            runs.append({'type': 'air', 'S': rpm, 'F': args.air_feed, 'ap': 0.0})
    else:
        limits = parse_limits(args.limits)
        combos = []
        for rpm in rpms:
            if limits:
                if rpm not in limits:
                    sys.exit(f"--limits에 {rpm} rpm의 안정 한계가 없습니다.")
                depths = [round(limits[rpm] * f, 3) for f in number_list(args.depth_factors)]
            else:
                depths = number_list(args.depths) if args.depths else []
                if not depths:
                    sys.exit("grid 계획에는 --depths 또는 --limits가 필요합니다.")
            combos += [{'type': 'grid', 'S': rpm, 'F': feed, 'ap': depth} for depth in depths for feed in feeds]
        random.Random(args.seed).shuffle(combos)
        reference = None
        if args.reference:
            rpm, feed, depth = (float(v) for v in args.reference.split(','))
            reference = {'type': 'reference', 'S': int(rpm), 'F': feed, 'ap': depth}
        for i, combo in enumerate(combos):
            if reference and i % max(1, args.reference_every) == 0:
                runs.append(dict(reference))
            runs.append(combo)
        if reference:
            runs.append(dict(reference))

    too_deep = [r for r in runs if r['ap'] > max_depth]
    if too_deep and not args.force:
        sys.exit(f"깊이 {max(r['ap'] for r in too_deep):g} mm가 공구 지름의 {args.max_depth_ratio:g}배({max_depth:g} mm)를 넘습니다. "
                 "공구 파손 위험이 있어 생성하지 않습니다 (필요하면 --force).")

    name = args.name or args.plan
    out_dir = os.path.join(args.out, name)
    os.makedirs(out_dir, exist_ok=True)
    pitch = args.tool_d + args.gap
    rows_per_column = max(1, int((args.stock_y - args.y0 - args.tool_d / 2) // pitch) + 1)
    plan_rows = []
    for order, run in enumerate(runs, start=1):
        column, row = divmod(order - 1, rows_per_column)
        x0 = args.x0 + column * (args.length + 10.0)
        y = args.y0 + row * pitch
        if x0 + args.length > args.stock_x:
            sys.exit(f"소재(X {args.stock_x:g} × Y {args.stock_y:g} mm)에 {len(runs)}개 홈이 들어가지 않습니다. "
                     "--stock-x/--stock-y/--length를 조정하거나 계획을 나누세요.")
        run_id = f"{name}_{order:03d}"
        meta = {'run_id': run_id, 'type': run['type'], 'S': run['S'], 'F': f"{run['F']:g}", 'ap': f"{run['ap']:g}",
                'ae': f"{ae:g}", 'tool': args.tool_id, 'tool_d': f"{args.tool_d:g}", 'flutes': args.flutes,
                'material': args.material.replace(' ', '_')}
        lines = slot_program(meta, x0, y, args.length, run['ap'], run['F'], args.plunge_feed, args.safe_z, run['S'],
                             ramp=run['type'] == 'ramp', air=run['type'] == 'air')
        file_name = f"{run_id}.nc"
        with open(os.path.join(out_dir, file_name), 'w', encoding='utf-8', newline='\n') as f:
            f.write("\n".join(lines) + "\n")
        plan_rows.append({'order': order, 'run_id': run_id, 'type': run['type'], 'S_rpm': run['S'],
                          'F_mm_min': run['F'], 'ap_mm': run['ap'], 'ae_mm': ae, 'x0': round(x0, 3), 'y': round(y, 3),
                          'length': args.length, 'file': file_name})

    with open(os.path.join(out_dir, 'plan.csv'), 'w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(plan_rows[0]))
        writer.writeheader()
        writer.writerows(plan_rows)
    print(f"{len(plan_rows)}개 실행 생성: {out_dir}")
    print("화면에서 순서대로 불러와 '데이터 수집 → 기록 시작 → 가공 시작 → (채터가 들리면 구간 표시) → 기록 종료'로 진행하세요.")


if __name__ == '__main__':
    main()
