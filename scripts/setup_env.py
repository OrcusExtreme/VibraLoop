"""가상환경(.venv)을 만들고 requirements.txt의 고정 버전 패키지를 설치한 뒤 실행 환경을 점검합니다.

setup.bat(Windows) / setup.sh(Linux·macOS)가 지원 버전(64비트 3.12 ~ 3.14)의 파이썬을 찾아 이 스크립트를 실행합니다.
아직 패키지가 없는 파이썬으로 실행되므로 표준 라이브러리만 사용합니다.

가상환경은 만든 PC·경로의 파이썬을 가리키므로 다른 PC로 복사하면 동작하지 않습니다.
이 스크립트는 그런 .venv를 찾아 지우고 이 PC에서 새로 만듭니다.
"""
import argparse
import os
import platform
import shutil
import stat
import subprocess
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENV_DIR = os.path.join(BASE_DIR, '.venv')
WHEEL_DIR = os.path.join(BASE_DIR, 'wheels')
REQUIREMENTS = os.path.join(BASE_DIR, 'requirements.txt')
SUPPORTED_PYTHON = ((3, 12), (3, 14))
# --download-wheels가 Windows에서 내려받는 대상 (오프라인 PC에 이 중 어느 버전이 설치돼 있어도 되도록)
WINDOWS_WHEEL_PYTHONS = ('3.12', '3.13', '3.14')

VENV_CHECK = (
    "import os, sys; "
    "ok = (3, 12) <= sys.version_info[:2] <= (3, 14) and sys.maxsize > 2**32 "
    "and os.path.normcase(os.path.realpath(sys.prefix)) == os.path.normcase(os.path.realpath(sys.argv[1])); "
    "sys.exit(0 if ok else 1)"
)


def venv_python():
    if os.name == 'nt':
        return os.path.join(VENV_DIR, 'Scripts', 'python.exe')
    return os.path.join(VENV_DIR, 'bin', 'python')


def run(command):
    print('>', subprocess.list2cmdline(command))
    sys.stdout.flush()
    return subprocess.run(command).returncode


def venv_is_usable():
    """이 폴더에 이 PC에서 만든, 지원 버전의 64비트 가상환경이 있는지 확인"""
    python = venv_python()
    if not os.path.isfile(python):
        return False
    try:
        result = subprocess.run([python, '-c', VENV_CHECK, VENV_DIR], capture_output=True, timeout=120)
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0


def remove_venv():
    def make_writable_and_retry(function, path, _error):
        os.chmod(path, stat.S_IWRITE)
        function(path)

    try:
        shutil.rmtree(VENV_DIR, onexc=make_writable_and_retry)
    except OSError as e:
        raise SystemExit(f"[실패] 기존 .venv를 지우지 못했습니다: {e}\n"
                         "실행 중인 서버나 .venv를 사용하는 편집기(VS Code 등)를 닫고 다시 실행하세요.")


def create_venv(recreate):
    if os.path.isdir(VENV_DIR):
        if not recreate and venv_is_usable():
            print(f"기존 가상환경을 사용합니다: {VENV_DIR}")
            return
        reason = "--recreate 옵션" if recreate else "다른 PC·경로에서 복사됐거나 지원하지 않는 파이썬 버전"
        print(f"기존 .venv를 지우고 새로 만듭니다 ({reason}).")
        remove_venv()
    print(f"가상환경 생성: {VENV_DIR} (Python {platform.python_version()})")
    if run([sys.executable, '-m', 'venv', VENV_DIR]) != 0:
        hint = "" if os.name == 'nt' else " Debian/Ubuntu는 python3-venv 패키지가 필요합니다 (예: sudo apt install python3.12-venv)."
        raise SystemExit(f"[실패] 가상환경을 만들지 못했습니다.{hint}")


def has_wheelhouse():
    return os.path.isdir(WHEEL_DIR) and any(name.endswith('.whl') for name in os.listdir(WHEEL_DIR))


def install_packages():
    python = venv_python()
    if has_wheelhouse():
        print("wheels 폴더의 설치 파일로 설치합니다 (인터넷 사용 안 함).")
        command = [python, '-m', 'pip', 'install', '--no-index', '--find-links', WHEEL_DIR,
                   '--only-binary=:all:', '-r', REQUIREMENTS]
    else:
        # 파이썬에 포함된 pip가 오래됐을 수 있어 먼저 업데이트 (실패해도 기존 pip로 계속 진행)
        run([python, '-m', 'pip', 'install', '--upgrade', 'pip'])
        command = [python, '-m', 'pip', 'install', '--only-binary=:all:', '-r', REQUIREMENTS]
    if run(command) != 0:
        raise SystemExit("[실패] 패키지 설치에 실패했습니다. 인터넷 연결 또는 wheels 폴더의 설치 파일을 확인하세요.")


def download_wheels():
    """인터넷이 없는 PC에 옮길 수 있도록 wheels 폴더에 설치 파일을 내려받습니다."""
    python = venv_python()
    os.makedirs(WHEEL_DIR, exist_ok=True)
    if os.name == 'nt':
        targets = [['--platform', 'win_amd64', '--python-version', version] for version in WINDOWS_WHEEL_PYTHONS]
        label = f"Windows 64비트 Python {', '.join(WINDOWS_WHEEL_PYTHONS)}"
    else:
        targets = [[]]
        label = f"이 PC와 같은 운영체제·Python {platform.python_version()}"
    print(f"wheels 폴더에 설치 파일 내려받기: {label}")
    for target in targets:
        command = [python, '-m', 'pip', 'download', '--only-binary=:all:', '--dest', WHEEL_DIR, *target, '-r', REQUIREMENTS]
        if run(command) != 0:
            raise SystemExit("[실패] 설치 파일을 내려받지 못했습니다. 인터넷 연결을 확인하세요.")
    print("완료: 프로젝트 폴더를 wheels 폴더와 함께 옮기면 다른 PC의 setup이 인터넷 없이 설치합니다.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--recreate', action='store_true', help="기존 .venv를 지우고 새로 만듭니다")
    parser.add_argument('--download-wheels', action='store_true',
                        help="인터넷이 없는 PC용 설치 파일을 wheels 폴더에 내려받습니다")
    parser.add_argument('--no-pause', action='store_true', help="setup.bat 전용 옵션 (여기서는 무시)")
    args = parser.parse_args()

    os.environ['PYTHONUTF8'] = '1'
    os.environ['PIP_DISABLE_PIP_VERSION_CHECK'] = '1'
    version = sys.version_info[:2]
    low, high = SUPPORTED_PYTHON
    if not (low <= version <= high) or sys.maxsize <= 2**32:
        raise SystemExit(f"[실패] 64비트 Python {low[0]}.{low[1]} ~ {high[0]}.{high[1]}이 필요합니다 "
                         f"(현재 {platform.python_version()}, {sys.executable}).")

    print(f"[1/3] 가상환경 준비 ({sys.executable})")
    create_venv(args.recreate)
    print("\n[2/3] 패키지 설치 (requirements.txt)")
    install_packages()
    if args.download_wheels:
        print()
        download_wheels()
    print("\n[3/3] 실행 환경 점검")
    if run([venv_python(), os.path.join(BASE_DIR, 'scripts', 'check_env.py')]) != 0:
        return 1
    start = "run.bat" if os.name == 'nt' else "bash run.sh"
    print(f"\n설치 완료: {start}로 서버를 시작한 뒤 브라우저에서 http://localhost:5000 을 여세요.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
