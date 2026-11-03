"""실행 환경 점검: 이 PC에서 프로그램을 실행할 준비가 됐는지 확인합니다.

setup.bat / setup.sh가 설치 후 자동으로 실행하며, 직접 실행할 때는 가상환경의 파이썬을 사용합니다.
  Windows:      .venv\\Scripts\\python.exe scripts\\check_env.py
  Linux/macOS:  .venv/bin/python scripts/check_env.py
문제가 있으면 [실패] 항목을 출력하고 종료 코드 1을 반환합니다.
"""
import os
import re
import sys
import warnings

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# requirements.txt의 고정 버전(numpy 2.5, scipy 1.18 등)이 설치 파일(휠)을 제공하는 파이썬 범위
SUPPORTED_PYTHON = ((3, 12), (3, 14))

failures = []
cautions = []


def ok(message):
    print(f"[정상] {message}")


def fail(message):
    failures.append(message)
    print(f"[실패] {message}")


def caution(message):
    cautions.append(message)
    print(f"[주의] {message}")


def check_python():
    version = sys.version_info[:2]
    bits = 64 if sys.maxsize > 2**32 else 32
    label = f"Python {sys.version.split()[0]} {bits}비트 ({sys.executable})"
    low, high = SUPPORTED_PYTHON
    if bits != 64:
        fail(f"{label}: 64비트 파이썬이 필요합니다.")
    elif not (low <= version <= high):
        fail(f"{label}: 지원 버전은 {low[0]}.{low[1]} ~ {high[0]}.{high[1]}입니다.")
    else:
        ok(label)


def check_packages():
    from importlib import metadata

    try:
        with open(os.path.join(BASE_DIR, 'requirements.txt'), encoding='ascii') as f:
            lines = f.readlines()
    except UnicodeDecodeError:
        fail("requirements.txt에 ASCII가 아닌 문자가 있습니다. 한글 Windows의 pip가 읽지 못하므로 영문만 사용하세요.")
        return

    pins = {}
    for line in lines:
        match = re.fullmatch(r'([A-Za-z0-9_.\-]+)==(\S+)', line.split('#', 1)[0].strip())
        if match:
            pins[match.group(1)] = match.group(2)

    problems = []
    for name, wanted in pins.items():
        try:
            installed = metadata.version(name)
        except metadata.PackageNotFoundError:
            problems.append(f"{name} 미설치(필요 {wanted})")
            continue
        if installed != wanted:
            problems.append(f"{name} {installed}(필요 {wanted})")
    if problems:
        fail("패키지 버전 불일치: " + ", ".join(problems) + " → setup 스크립트를 다시 실행하세요.")
    else:
        ok(f"requirements.txt의 패키지 {len(pins)}개가 모두 지정 버전으로 설치됨")


def check_detector_model(bundle, name, label):
    """데이터셋 감지 모델 묶음(scripts/train_models.py, P12): 형식·특징 계산 방식을 확인하고, 창 설정마다 학습 데이터
    중심의 윈도우를 채점해 SPE·T²가 유한한지 봅니다."""
    import numpy as np
    import features
    from vibration_monitor import DETECTOR_MODEL_KIND, PROFILE_LABELS, pca_scores

    if not isinstance(bundle, dict) or bundle.get('kind') != DETECTOR_MODEL_KIND or not bundle.get('models'):
        fail(f"models/{name}의 형식이 맞지 않아 쓰지 않습니다. scripts/train_models.py로 다시 학습하세요.")
        return
    if bundle.get('feature_version') != features.FEATURE_VERSION:
        fail(f"models/{name}은 특징 계산 방식이 지금과 달라 쓰지 않습니다. "
             "scripts/build_dataset.py → scripts/train_models.py로 다시 학습하세요.")
        return
    parts = []
    for profile, model in sorted(bundle['models'].items()):
        spe, t2, _ = pca_scores(model['pca'], model['pca']['mean'])
        if not (np.isfinite(spe[0]) and np.isfinite(t2[0])):
            fail(f"models/{name}의 {PROFILE_LABELS.get(profile, profile)} 모델 채점 결과가 올바르지 않습니다: SPE {spe[0]}, T² {t2[0]}")
            return
        parts.append(f"{PROFILE_LABELS.get(profile, profile)}: 실행 {model['sessions']}개, 특징 {len(model['feature_names'])}개")
    ok(f"{label}(models/{name}) 로드·채점 정상 ({'; '.join(parts)}, {bundle.get('dataset')}, {bundle.get('trained_at')})")


def check_models():
    """scripts/train_models.py로 학습한 모델(있으면)을 불러와 버전 경고와 예측을 확인합니다."""
    import joblib
    import numpy as np
    from sklearn.exceptions import InconsistentVersionWarning
    if os.path.join(BASE_DIR, 'src') not in sys.path:
        sys.path.insert(0, os.path.join(BASE_DIR, 'src'))
    from vibration_monitor import CONDITION_MODEL_KIND

    model_dir = os.path.join(BASE_DIR, 'models')
    missing = []
    if os.path.exists(os.path.join(model_dir, 'stability_model.joblib')):
        caution("models/stability_model.joblib(예전 채터 확률 모델)은 P11부터 쓰지 않습니다. scripts/train_models.py를 "
                "실행하면 models/legacy/로 옮기며, 직접 옮기거나 지워도 됩니다. "
                "재가공·온라인 대응 조건은 안정 한계 모델(models/sld_gpr_model.joblib)로 정합니다.")
    for name, label, without in (('detector_model.joblib', '데이터셋 감지 모델',
                                  "서버 시작 때 학습 데이터 폴더의 CSV로만 감지 모델을 학습하고(고속 센서는 비고조파 비율 없이)"),
                                 ('condition_model.joblib', '조건 모델', "감지에 조건 항목을 쓰지 않고"),
                                 ('sld_gpr_model.joblib', '안정 한계 모델',
                                  "재가공·온라인 대응 조건은 채터 주파수 또는 기본 규칙으로 정하고")):
        path = os.path.join(model_dir, name)
        if not os.path.exists(path):
            missing.append(f"{label}이 없어 {without}")
            continue
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            bundle = joblib.load(path)
        version_warnings = [w for w in caught if issubclass(w.category, InconsistentVersionWarning)]
        if version_warnings:
            fail(f"models/{name}이 다른 scikit-learn 버전으로 저장되어 결과를 믿을 수 없습니다. "
                 f"scripts/train_models.py로 다시 학습하세요: {version_warnings[0].message}")
            continue
        if name == 'detector_model.joblib':
            check_detector_model(bundle, name, label)
            continue
        if name == 'condition_model.joblib' and bundle.get('kind') != CONDITION_MODEL_KIND:
            fail(f"models/{name}이 예전 형식(P09 이전)이라 감지에 쓰지 않습니다. scripts/train_models.py로 다시 학습하세요 "
                 "(예전 파일은 models/legacy/로 옮깁니다).")
            continue
        # 학습 범위의 가운데 조건으로 예측해 봄 (범위를 모르면 20000 rpm, 100 mm/min, 0.3 mm)
        example = {'n': 20000.0, 'F': 100.0, 'ap': 0.3}
        example.update({key: (low + high) / 2 for key, (low, high) in (bundle.get('training_range') or {}).items()})
        X = np.array([[example[name] for name in bundle['features']]])
        value = bundle['model'].predict(X)[0]
        coverage = bundle.get('coverage')
        detail = f", 상한 포함률 {coverage:.1%}" if coverage is not None else ""
        if not np.isfinite(value):
            fail(f"models/{name} 예측 결과가 올바르지 않습니다: {value}")
        else:
            ok(f"{label}(models/{name}) 로드·예측 정상 (실행 {bundle.get('n_runs')}개로 학습{detail}, {bundle.get('trained_at')})")
    if missing:
        caution(", ".join(missing) + " 있습니다. "
                "데이터를 수집한 뒤 scripts/build_dataset.py → scripts/train_models.py로 학습하세요.")


def check_settings():
    """모델 관리 창에서 저장한 감지 설정(data/detector_settings.json)을 서버와 같은 규칙(setting_problem)으로 검사합니다."""
    if os.path.join(BASE_DIR, 'src') not in sys.path:
        sys.path.insert(0, os.path.join(BASE_DIR, 'src'))
    from vibration_monitor import SETTINGS_PATH, load_settings

    if not os.path.exists(SETTINGS_PATH):
        ok("저장된 감지 설정 없음: 기본값으로 감지합니다 (data/detector_settings.json)")
        return
    settings, problems = load_settings(SETTINGS_PATH)
    if problems:
        caution("data/detector_settings.json에 쓰지 않는 값이 있어 그 항목은 기본값을 씁니다: " + "; ".join(problems)
                + " (모델 관리 창에서 설정을 다시 저장하면 고쳐집니다)")
    else:
        ok("저장된 감지 설정 정상: " + ", ".join(f"{key}={value}" for key, value in settings.items()))


def check_training_data():
    data_dir = os.path.join(BASE_DIR, 'data', '학습 데이터')
    names = os.listdir(data_dir) if os.path.isdir(data_dir) else []
    usable = []
    for name in names:
        if not name.lower().endswith('.csv') or '이상' in name:
            continue
        with open(os.path.join(data_dir, name), encoding='utf-8-sig', errors='replace') as f:
            header = {column.strip() for column in f.readline().split(',')}
        if {'AccX', 'AccY', 'AccZ'} <= header:
            usable.append(name)
    if usable:
        ok(f"이상 진동 감지용 정상 학습 데이터 {len(usable)}개 (data/학습 데이터)")
    else:
        caution("data/학습 데이터에 정상 진동 CSV(AccX, AccY, AccZ 열)가 없어 이상 진동 감지가 꺼진 채로 실행됩니다.")


def check_web_assets():
    with open(os.path.join(BASE_DIR, 'templates', 'index.html'), encoding='utf-8') as f:
        html = f.read()
    local = re.findall(r'(?:src|href)="/static/([^"]+)"', html)
    external = re.findall(r'<script[^>]+src="(https?://[^"]+)"', html)
    missing = [path for path in local if not os.path.isfile(os.path.join(BASE_DIR, 'static', *path.split('/')))]
    if missing:
        fail(f"웹 화면 파일이 없습니다: {', '.join('static/' + path for path in missing)}")
    if external:
        caution(f"인터넷이 필요한 외부 스크립트가 있습니다: {', '.join(external)}")
    if not missing and not external:
        ok(f"웹 화면 파일 {len(local)}개가 모두 static 폴더에 있음 (인터넷 없이 동작)")


def check_serial():
    from serial.tools import list_ports

    ports = [port.device for port in list_ports.comports()]
    ok(f"시리얼 포트 조회 정상: {', '.join(ports) if ports else '연결된 포트 없음'}")


def check_app():
    if BASE_DIR not in sys.path:
        sys.path.insert(0, BASE_DIR)
    import cnc_app  # Flask 앱 생성과 라우트 등록까지 확인 (서버는 시작하지 않음)

    rules = {rule.rule for rule in cnc_app.app.url_map.iter_rules()}
    missing = [rule for rule in ('/', '/api/vibration/model', '/api/recording/status', '/set_tolerance') if rule not in rules]
    if missing:
        fail(f"웹 서버 경로가 등록되지 않았습니다: {', '.join(missing)}")
    else:
        ok(f"cnc_app 불러오기 정상 (웹 경로 {len(rules)}개 등록)")


def main():
    try:
        sys.stdout.reconfigure(errors='replace')  # 한글을 표시할 수 없는 콘솔에서도 점검은 계속
    except (AttributeError, ValueError):
        pass
    print(f"ProjectLab_System 실행 환경 점검: {BASE_DIR}")
    check_python()
    for check in (check_packages, check_models, check_settings, check_training_data, check_web_assets, check_serial, check_app):
        try:
            check()
        except Exception as e:
            fail(f"{check.__name__} 점검 중 오류: {type(e).__name__}: {e}")

    print()
    if failures:
        print(f"점검 결과: 실패 {len(failures)}건. 위의 [실패] 항목을 해결한 뒤 다시 점검하세요.")
        return 1
    suffix = f" (주의 {len(cautions)}건)" if cautions else ""
    print(f"점검 결과: 실행 준비 완료{suffix}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
