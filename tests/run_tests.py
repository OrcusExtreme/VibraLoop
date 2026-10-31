"""저장소 테스트 실행기

  .venv\\Scripts\\python tests\\run_tests.py              # unit, ui, e2e (기본)
  .venv\\Scripts\\python tests\\run_tests.py unit ui      # 골라서 실행
  .venv\\Scripts\\python tests\\run_tests.py bench        # 합성·예전 데이터 벤치마크 (v1.1 기준값과 비교)
  .venv\\Scripts\\python tests\\run_tests.py e2e --keep   # 임시 복사본을 지우지 않음 (실패 원인 확인용)

파이썬 테스트는 프로젝트를 임시 폴더에 복사해 그 안에서 실행합니다 (학습된 모델·수집 기록·감지 설정은 빼고 복사).
그래서 저장소의 nc/, data/, logs/, models/는 바뀌지 않습니다. 화면 테스트(ui)는 Node.js가 있어야 하며, 없으면 건너뜁니다.
"""
import argparse
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import time

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)

# 이름: (실행기, 파일들, 제한 시간(초))
SUITES = {
    'unit': ('python', ['unit_checks.py'], 900),
    'ui': ('node', ['ui_events_checks.js', 'ui_vibration_checks.js'], 300),
    'e2e': ('python', ['e2e_checks.py'], 1800),
    'bench': ('python', ['bench_checks.py'], 1800),
}
DEFAULT_SUITES = ['unit', 'ui', 'e2e']
IGNORED_NAMES = {'.git', '.venv', 'venv', 'wheels', 'analysis', '__pycache__', 'tests', 'raw', 'dataset', 'synthetic',
                 'detector_settings.json'}


def _ignore(directory, names):
    """복사에서 뺄 것: 실행 환경, 수집 기록·데이터셋, 실행 로그, PC별 설정, 학습된 모델 (models/legacy는 유지)"""
    ignored = {n for n in names if n in IGNORED_NAMES or n.startswith(('cnc_log_', 'upload_'))}
    if os.path.basename(directory) == 'models':
        ignored |= {n for n in names if n.endswith('.joblib')}
    return ignored


def make_project_copy(name):
    holder = tempfile.mkdtemp(prefix=f'projectlab_{name}_')
    copy = os.path.join(holder, 'ProjectLab_System')
    shutil.copytree(ROOT, copy, ignore=_ignore)
    return holder, copy


def remove_project_copy(holder):
    """임시 복사본 삭제. 복사본은 원본 폴더의 읽기 전용 속성까지 이어받으므로(Windows) 속성을 풀고 다시 지웁니다."""
    def clear_readonly(func, path, _exc):
        os.chmod(path, stat.S_IWRITE)
        func(path)
    try:
        shutil.rmtree(holder, onexc=clear_readonly)
    except OSError as e:
        print(f"임시 복사본을 지우지 못했습니다 (직접 지우세요): {holder} ({e})", flush=True)


def run_command(command, timeout):
    """명령을 실행하며 출력을 그대로 보여 주고, (종료 코드, 마지막 'passed' 줄)을 돌려줍니다."""
    env = dict(os.environ, PYTHONUTF8='1', PYTHONIOENCODING='utf-8')
    summary = ''
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                            text=True, encoding='utf-8', errors='replace')
    deadline = time.time() + timeout
    for line in proc.stdout:
        print(line, end='', flush=True)
        if 'passed' in line:
            summary = line.strip()
        if time.time() > deadline:
            proc.kill()
            print(f"\n[시간 초과] {timeout}초를 넘어 중단했습니다.", flush=True)
            return 1, summary or '시간 초과'
    return proc.wait(), summary


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')
    parser = argparse.ArgumentParser(description="ProjectLab_System 테스트 실행")
    parser.add_argument('suites', nargs='*', help=f"실행할 묶음 ({', '.join(SUITES)}; 기본: {' '.join(DEFAULT_SUITES)})")
    parser.add_argument('--keep', action='store_true', help="임시 프로젝트 복사본을 지우지 않음")
    args = parser.parse_args()
    names = args.suites or DEFAULT_SUITES
    unknown = [n for n in names if n not in SUITES]
    if unknown:
        parser.error(f"알 수 없는 묶음: {', '.join(unknown)}")

    results = []
    for name in names:
        runner, files, timeout = SUITES[name]
        for file_name in files:
            path = os.path.join(TESTS_DIR, file_name)
            print(f"\n===== {name}: {file_name} =====", flush=True)
            started = time.time()
            if runner == 'node':
                node = shutil.which('node')
                if not node:
                    print("Node.js가 없어 건너뜁니다 (https://nodejs.org).", flush=True)
                    results.append((file_name, None, '건너뜀', 0.0))
                    continue
                code, summary = run_command([node, path, os.path.join(ROOT, 'static', 'script.js')], timeout)
            else:
                holder, copy = make_project_copy(name)
                try:
                    code, summary = run_command([sys.executable, path, copy], timeout)
                finally:
                    if args.keep:
                        print(f"임시 복사본 유지: {copy}", flush=True)
                    else:
                        remove_project_copy(holder)
            results.append((file_name, code == 0, summary, time.time() - started))

    print("\n===== 요약 =====")
    for file_name, ok, summary, seconds in results:
        status = '건너뜀' if ok is None else ('통과' if ok else '실패')
        print(f"  [{status}] {file_name} ({seconds:.0f}초) {summary}")
    sys.exit(0 if all(ok is not False for _, ok, _, _ in results) else 1)


if __name__ == '__main__':
    main()
