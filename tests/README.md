# 테스트

코드를 고칠 때마다 돌리는 회귀 테스트와, 감지기 성능을 수정 전후로 비교하는 벤치마크입니다.
GRBL 컨트롤러와 센서 없이 돌아갑니다.

## 실행

```bat
.venv\Scripts\python tests\run_tests.py              :: unit, ui, e2e (약 3분)
.venv\Scripts\python tests\run_tests.py unit         :: 묶음 하나만
.venv\Scripts\python tests\run_tests.py bench        :: 벤치마크 (약 5분)
.venv\Scripts\python tests\run_tests.py e2e --keep   :: 임시 복사본을 남김 (실패 원인 확인용)
```

Linux/macOS에서는 `.venv/bin/python tests/run_tests.py`로 실행합니다.

- 파이썬 테스트는 프로젝트를 임시 폴더에 복사해 그 안에서 실행합니다. 그래서 저장소 파일은 바뀌지 않습니다.
- 학습된 모델(`models/*.joblib`), 수집 기록(`data/raw`, `data/dataset`, `data/synthetic`), 감지 설정(`data/detector_settings.json`), 실행 로그는 복사하지 않습니다. 그래서 PC마다 같은 조건에서 시작합니다.
- `ui` 묶음은 Node.js가 필요하며, 없으면 건너뜁니다.

## 묶음

| 묶음 | 파일 | 내용 |
| :--- | :--- | :--- |
| `unit` | `unit_checks.py` | 특징 계산, 센서 프로토콜, 최적화기, 재가공 깊이 분할, 판정 규칙, 감지기(가짜 컨트롤러로 진입·대응·정지), 데이터 수집 기록, 실험 NC 생성, 데이터셋→학습→평가, 합성 기록과 평가 기반(P01), 앨리어싱 마스킹·회전 주파수 추정·채터 주파수 사용 조건(P03), 센서별 창 설정·설정별 모델·실시간 평가 격자와 처리 시간(P04), 롤링 기준선의 시작·멈춤·조건 변경(P05), 2단계 판정(빠른 차단·CUSUM)과 기록 전용 모드(P06), PCA-SPE 감지기(scikit-learn PCA와 같은 값, 빠진 특징 처리, SPE·T² 역할, 원인 기록)(P08), 오버라이드 명령 계획(10–200% 전체 경우, 이송 먼저, 시각 기록)(P07), 물리 변수·합성 grid 시험·단조 조건 모델·조건 항목(P09), ramp 채터 시작점 재검출·중도절단·안정 한계 모델과 안전 깊이(P10), 안정 한계 모델로 고르는 재가공 패스 수·회전수와 온라인 회전수(가공 시간 최소·가까운 회전수, 패스 하한, 이송·날당 이송이 늘지 않음, 다른 공구·날 수/깊이 모름·불가능한 깊이에서 다음 방법), 예전 채터 확률 모델 파일 무시(P11), 늦은 라벨 앞 3초를 뺀 정상 윈도우, 데이터셋 감지 모델의 저장·특징 계산 방식 확인·창 설정별 사용과 '감지기 초기화' 때 다시 읽기, 학습 범위 밖(T²) 비율, 설정 검증(저장 파일·화면·`--set`·`check_env`), 예전 모델을 `models/legacy/`로 옮기기(P12) |
| `ui` | `ui_events_checks.js`, `ui_vibration_checks.js` | `static/script.js`의 소켓 이벤트 처리, 진동 감지 모델 창(창 설정별 감지 모델의 출처, 판정 기준, 학습 범위 밖 비율, 조건 모델, 설정 안내)·데이터 수집 창 (가짜 DOM) |
| `e2e` | `e2e_checks.py` | GRBL 시뮬레이터(`grbl_sim.py`)와 실제 `cnc_app`: 스트리밍, 일시정지·재개, 오류 응답, 비상정지, 감시 스레드, 채터 → 온라인 대응(보낸 명령·순서·시간, `Ov:` 확인, 잃어버린 명령 재전송) → 위치 유지 정지 → 재가공, 안정 한계 모델로 정한 온라인 회전수와 재가공 조건(`rework.nc`의 회전수·패스 수)(P11), 데이터 수집, 모델 관리 API |
| `bench` | `bench_checks.py` | 합성 시나리오(고정 시드)와 예전 학습 데이터로 감지기를 평가하고 `baseline/`의 기준값과 비교. 합성 grid 시험으로 학습한 조건 모델을 넣어서도 평가. 합성 ramp로 학습한 안정 한계 모델과 그 모델로 고른 대응 조건을 정답 한계와 비교 |

`grbl_sim.py`는 GRBL 1.1의 128바이트 수신 버퍼, 16블록 플래너, 실시간 명령(`?`, `!`, `~`, 0x18, 오버라이드 0x90–0x9D, 0x9E)을 흉내 냅니다. 피드 홀드(Hold:1 → Hold:0)와 이동 중 리셋 알람(ALARM:3)도 재현합니다. 오버라이드 명령은 실제 GRBL처럼 처리 주기마다 플래그로 모아 반영하므로, 같은 명령을 너무 빨리 반복하면 한 번만 적용됩니다. 실시간 바이트마다 받은 시각을 남기고(`realtime_times`), 오버라이드 바이트를 일부러 잃어버리게 할 수 있습니다(`drop_override_bytes`).

## 고정 자료

테스트는 저장소의 다음 파일을 그대로 씁니다. 이 파일들을 바꾸면 `unit`·`e2e` 결과가 달라질 수 있습니다.

- `nc/O0527.txt`, `nc/O0807.txt`, `nc/O1001.nc`, `nc/rework.nc`
- `data/학습 데이터/*.csv`: 이름에 '이상'이 들어간 파일은 이상 데이터로 다룸
- `data/Vibration_detection.csv`: 기존 기록이 보존되는지 확인

## 벤치마크와 기준값

- `baseline/`의 `v1.1_*.json`은 v1.1 감지기의 결과입니다. 평가 방식은 P01 기준입니다.
  - `v1.1_synthetic.json`: 합성 28개 (센서 DLPF 184 Hz, 스핀들이 지령 회전수대로 돎)
  - `v1.1_stress.json`: 같은 28개를 DLPF 260 Hz(앨리어싱이 더 큼), 실제 회전수 +2%로 만든 세트. P03 직전 코드로 계산했습니다.
  - `v1.1_legacy.json`: `data/학습 데이터`의 파일 단위 교차 검증
- `bench`는 같은 시드로 합성 기록을 다시 만들어 평가하고, 결과를 기준값과 나란히 출력합니다. 바뀐 지표에는 `*`가 붙습니다.
  - 합성 세트는 기본 판정(`k_of_n`)과 2단계 판정(`--set decision_mode=two_tier`)으로 한 번씩 평가하고, 둘 다 같은 v1.1 기준값과 비교합니다.
  - 평가 출력의 "경보 원인"은 경보마다 가장 큰 증거 항목(`spe`, `level`, `self`, `spectral` 등)을 센 것입니다. 오경보가 어느 항목에서 나는지 볼 때 씁니다.
  - 조건 모델(P09): 세트마다 같은 센서 옵션으로 합성 grid 시험 108개를 만들어 `build_dataset` → `train_models`로 학습하고, 두 판정을 그 모델로 다시 평가합니다(`<세트>_condition.json`, `<세트>_two_tier_condition.json`). 실제 grid 시험과 같은 순서입니다.
  - 조건 모델은 센서 설정에 묶여 있어, 세트마다 자기 DLPF·회전수 오차로 만든 grid를 씁니다.
  - 안정 한계(P10): 정답 한계를 아는 합성 ramp 시험 18개를 `build_dataset`(시작점 재검출·중도절단) → `train_models`(SLD)로 처리하고, 보정한 시작 깊이와 250 rpm마다의 안전 깊이를 정답과 비교합니다(`bench/ramp.json`).
  - 최적화기(P11): 같은 모델로, 채터가 나는 조건(깊이 > 정답 한계, 250 rpm × 0.05 mm 격자)마다 온라인 대응과 재가공에서 고른 조건이 정답 한계 안에 드는지(재가공은 패스당 깊이) 보고 기본 규칙과 비교합니다. 재가공은 감지기 기본값인 패스 하한 2번과 하한 없음 두 가지로 봅니다(`bench/optimizer.json`).
  - 평가와 `train_models`는 같은 정상 윈도우로 감지기를 학습합니다(P12: 채터·전이 라벨 시작 앞 3초 제외). 조건 모델·안정 한계 모델만 학습할 때는 `--skip-detector`로 감지 모델을 만들지 않습니다.
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

스트레스 세트는 생성할 때 `--dlpf 260 --rpm-error 0.02`를 붙이고, `v1.1_stress.json`과 비교합니다.
