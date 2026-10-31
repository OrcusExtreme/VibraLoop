# 테스트

코드를 고칠 때마다 돌리는 회귀 테스트와, 감지기 성능을 수정 전후로 비교하는 벤치마크입니다.
GRBL 컨트롤러와 센서 없이 돌아갑니다.

## 실행

```bat
.venv\Scripts\python tests\run_tests.py              :: unit, ui, e2e (약 3분)
.venv\Scripts\python tests\run_tests.py unit         :: 묶음 하나만
.venv\Scripts\python tests\run_tests.py bench        :: 벤치마크 (약 1분)
.venv\Scripts\python tests\run_tests.py e2e --keep   :: 임시 복사본을 남김 (실패 원인 확인용)
```

Linux/macOS에서는 `.venv/bin/python tests/run_tests.py`로 실행합니다.

- 파이썬 테스트는 프로젝트를 임시 폴더에 복사해 그 안에서 실행합니다. 그래서 저장소 파일은 바뀌지 않습니다.
- 학습된 모델(`models/*.joblib`), 수집 기록(`data/raw`, `data/dataset`, `data/synthetic`), 감지 설정(`data/detector_settings.json`), 실행 로그는 복사하지 않습니다. 그래서 PC마다 같은 조건에서 시작합니다.
- `ui` 묶음은 Node.js가 필요하며, 없으면 건너뜁니다.

## 묶음

| 묶음 | 파일 | 내용 |
| :--- | :--- | :--- |
| `unit` | `unit_checks.py` | 특징 계산, 센서 프로토콜, 최적화기, 재가공 깊이 분할, 판정 규칙, 감지기(가짜 컨트롤러로 진입·대응·정지), 데이터 수집 기록, 실험 NC 생성, 데이터셋→학습→평가, 합성 기록과 평가 기반(P01) |
| `ui` | `ui_events_checks.js`, `ui_vibration_checks.js` | `static/script.js`의 소켓 이벤트 처리, 진동 감지 모델 창·데이터 수집 창 (가짜 DOM) |
| `e2e` | `e2e_checks.py` | GRBL 시뮬레이터(`grbl_sim.py`)와 실제 `cnc_app`: 스트리밍, 일시정지·재개, 오류 응답, 비상정지, 감시 스레드, 채터 → 온라인 대응 → 위치 유지 정지 → 재가공, 데이터 수집, 모델 관리 API |
| `bench` | `bench_checks.py` | 합성 시나리오(고정 시드)와 예전 학습 데이터로 감지기를 평가하고 `baseline/`의 기준값과 비교 |

`grbl_sim.py`는 GRBL 1.1의 128바이트 수신 버퍼, 16블록 플래너, 실시간 명령(`?`, `!`, `~`, 0x18, 오버라이드 0x90–0x9D, 0x9E)을 흉내 냅니다. 피드 홀드(Hold:1 → Hold:0)와 이동 중 리셋 알람(ALARM:3)도 재현합니다. 오버라이드 명령은 실제 GRBL처럼 처리 주기마다 플래그로 모아 반영하므로, 같은 명령을 너무 빨리 반복하면 한 번만 적용됩니다.

## 고정 자료

테스트는 저장소의 다음 파일을 그대로 씁니다. 이 파일들을 바꾸면 `unit`·`e2e` 결과가 달라질 수 있습니다.

- `nc/O0527.txt`, `nc/O0807.txt`, `nc/O1001.nc`, `nc/rework.nc`
- `data/학습 데이터/*.csv`: 이름에 '이상'이 들어간 파일은 이상 데이터로 다룸
- `data/Vibration_detection.csv`: 기존 기록이 보존되는지 확인

## 벤치마크와 기준값

- `baseline/v1.1_synthetic.json`과 `baseline/v1.1_legacy.json`은 v1.1 감지기의 결과입니다. 2026-10-31 P01 시점, 평가 방식도 P01 기준입니다.
- `bench`는 같은 시드로 합성 기록을 다시 만들어 평가하고, 결과를 기준값과 나란히 출력합니다. 바뀐 지표에는 `*`가 붙습니다.
- 숫자가 달라도 실패로 보지 않습니다. 평가 과정이 끝까지 도는지만 확인합니다.
- 새 기준값이 필요할 때의 순서:
  1. `run_tests.py bench --keep`으로 실행합니다.
  2. 복사본의 `bench/*.json`을 새 이름(예: `v1.2_*.json`)으로 `baseline/`에 넣습니다.
  3. `bench_checks.py`의 `BASELINE_TAG`를 바꿉니다.

합성 기록만 따로 만들어 볼 때는 다음을 실행합니다. `data/synthetic/`은 커밋되지 않습니다.

```bat
python scripts\make_synthetic_runs.py
python scripts\build_dataset.py --raw data\synthetic\raw --out data\synthetic\dataset
python scripts\evaluate_detector.py --dataset data\synthetic\dataset --compare tests\baseline\v1.1_synthetic.json
```
