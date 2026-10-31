# `cnc_system` vs `ProjectLab_System` 비교 분석 및 `ProjectLab_System` 수정 가이드 (`analyze2.md`)

> **문서 목적**: 실제 CNC 장비와 연동하여 구동된 이력이 있는 [`cnc_system`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system) 폴더와 현재 작업 대상인 [`ProjectLab_System`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System) 폴더의 모든 파일·바이트코드·실행 로그를 전수 비교하고, 실제 CNC 구동 로그(`2026-08-13`)에서 드러난 실증적 문제점과 이를 해결하기 위한 [`ProjectLab_System`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System)의 구체적 수정 방안을 정리합니다.

---

## 1. `cnc_system`과 `ProjectLab_System`의 전수 비교 결과

### 1.1 핵심 결론 요약
1. **모든 소스 코드(`.py`, `.js`, `.css`, `.html`, `.ino`), 학습 모델(`.joblib`), NC 데이터(`.nc`, `.txt`), 학습 데이터(`학습 데이터/*.csv`) 등 총 47개 핵심 파일은 두 폴더 간 바이트 단위로 100% 동일(MD5 해시 일치)**합니다.
2. 즉, [`cnc_system`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system)에 별도로 수정된 최신 소스 코드가 들어 있는 것이 아니라, **`2026-07-09` 시점의 동일한 코드베이스를 CNC 제어용 PC의 `C:\cnc_system` 경로에 복사하여 `2026-08-13`에 실제 CNC와 연결해 구동한 폴더**입니다.
3. 그러나 [`cnc_system`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system)에만 남아 있는 **실행 아티팩트([`구동방법.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/구동방법.txt), [`current_position.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/current_position.txt), `2026-08-13` 가공 로그 22개, `__pycache__` 컴파일 이력)**를 분석한 결과, **실제 CNC 환경에서 어떻게 구동했는지**와 **실제 장비 연동 시 어떤 버그들이 실제로 발생했는지**가 명확히 규명되었습니다.

---

### 1.2 파일별 MD5 해시 및 상태 비교표

#### (1) 100% 동일한 파일 (47개 파일, MD5 해시 완전 일치)
| 구분 | 파일 목록 | 파일 크기 | MD5 해시 (`cnc_system` = `ProjectLab_System`) |
| :--- | :--- | :---: | :--- |
| **백엔드 Python** | `cnc_app.py` | 72,227 B | `D60D83FB596F6EB4D9A3D8A517A8D949` |
| | `routes.py` | 53,324 B | `64898914B478E83CFE8473BF774463E8` |
| | `vibration_monitor.py` | 34,758 B | `9AB1FD9407F3AC748DA3A23DC0D3DBC4` |
| | `arduino_sensor.py` | 11,668 B | `21BF2F268B6E297C789CCAEBDD520125` |
| | `optimizer.py` | 7,566 B | `C355AD994A63082AC1E638D04522C852` |
| | `watcher.py` | 6,369 B | `0512488B8E319A652D152EA5734AFC03` |
| | `rework.py` | 5,972 B | `6F6F259F330C0E84D023DACC4241AAA5` |
| | `test_arduino_sensor.py` | 2,576 B | `D61F112D83F8F380D877B55B3273BA94` |
| | `train_models.py` | 2,007 B | `314C06861AE75B5C33BFDF79F8036EB0` |
| **프론트엔드** | `static/script.js` | 112,764 B | `76BFF9E5F0C3780BC4F68027983B6488` |
| | `static/style.css` | 41,840 B | `2FD83A4BFCF5A1667AC996FB68647E06` |
| | `templates/index.html` | 18,265 B | `F7C8D313A1ABBC5437AEC67C938582E1` |
| | `static/alert.txt`, `static/favicon.ico` | 110 B / 0 B | `6B83B3B6...` / `D41D8CD9...` |
| **모델 및 데이터** | `regr_model.joblib` | 4,727,228 B | `33AFA4AF76495D977E4B19CA94E1A939` |
| | `lof_model.joblib` | 44,298 B | `4644BBEE41EA9728783DCCFDA3083713` |
| | `O0527.txt`, `O0807.txt`, `O1001.nc`, `rework.nc` | 121~5,371 B | 모두 100% 동일 |
| | `Vibration_detection.csv` | 119 B | `CF52EED439B4A362F75184DACF7058D1` (`2025-11-26` 기록) |
| | `학습 데이터/*.csv` (20개 파일) | 93 KB~382 KB | 20개 파일 전체 100% 동일 |
| **아두이노/설정** | `MPU6050.ino`, `MPU6050/MPU6050.ino`, `sensor_example.ino` | 2,609~2,843 B | 모두 100% 동일 |
| | `requirements.txt`, `README.md`, `README_ARDUINO.md`, `venv/pyvenv.cfg` | - | 모두 100% 동일 |

---

#### (2) 두 폴더 간 차이가 존재하는 항목 (총 5개 영역)

| 항목 | `ProjectLab_System` | `cnc_system` (실제 CNC 구동 폴더) | 분석적 의미 |
| :--- | :--- | :--- | :--- |
| **1. [`구동방법.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/구동방법.txt)** | 없음 | **존재** (`2026-08-13 17:30` 생성)<br>`conda activate cnc_system`<br>`cd C:\cnc_system`<br>`python cnc_app.py` | 프로젝트 내부의 깨진 `venv/`를 쓰지 않고, **Anaconda `cnc_system` 가상환경(Python 3.12)**에서 `C:\cnc_system` 경로로 구동함. (단, `watcher.py` 실행 명령이 누락되어 있음) |
| **2. [`current_position.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/current_position.txt)** | `0.0` (3 B, `2026-07-09`) | **`0.76`** (4 B, `2026-08-13 19:06:02`) | [`cnc_app.py:980`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L974-L981)이 가공 시작 시 GRBL에 `?\n`을 보내 응답받은 `MPos`의 Z축 기계좌표(`0.76`)를 기록한 값. |
| **3. `logs/` 내 가공 로그** | 986개 (`2025-11-26`까지) | **1,008개** (`2026-08-13` 로그 **22개 추가**) | `2026-08-13 17:05:14` ~ `19:06:02` 약 2시간 동안 실제 CNC로 `O0527.txt`, `O0807.txt`, `O1001.nc`를 22회 실행한 궤적 기록. |
| **4. `__pycache__/` 바이트코드** | 구버전 PC(`c:\Users\USER\Desktop\...`)의 `.cpython-312.pyc` 잔존 (`rework` 5,190 B, `vibration_monitor` 20,951 B) | **`2026-08-13 13:03`에 `C:\cnc_system\*.py` 기준으로 재컴파일된 `.cpython-312.pyc`** (`optimizer`, `rework`, `routes`, `vibration_monitor`, `watcher`) | `ProjectLab_System`의 구버전 `.pyc` 역공학 결과, 과거 6인자 `create_rework_nc` 시그니처가 [`routes.py:971`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L971)에 미수정 상태로 방치된 원인이 확인됨. |
| **5. 기타 파일/폴더** | [`full.md`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/full.md) 존재 | 빈 폴더 `시스템/` 존재 (`2026-08-13 13:02`) | 실행 로직에는 영향 없음. |

---

## 2. 실제 CNC 구동 기록(`2026-08-13` 22개 로그) 기반 실증 결함 분석

[`cnc_system/logs/`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs)에 기록된 `2026-08-13`의 22개 로그 파일(`cnc_log_20260813_170514.csv` ~ `190602.csv`)과 파일 타임스탬프를 역추적하면, 실제 CNC 연동 시 **무엇이 동작했고 무엇이 실패했는지** 정확히 드러납니다.

### 2.1 정상 동작했던 부분 (실제 CNC 연동 확인 항목)
* **GRBL 시리얼 연결 및 기본 상태 쿼리(`?`)**: `115200 bps` 시리얼 연결 후 `MPos:x,y,z` 파싱 및 [`current_position.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/current_position.txt)(`0.76`) 저장 정상 동작.
* **단순 명시적 `G0`/`G1` 단일 라인 스트리밍**: `G1 Z-0.2 F50`, `G1 X10 F50`, `G1 Y70 F200`처럼 매 줄마다 `G0` 또는 `G1`이 명시된 직선 절삭 명령의 GRBL 전송 및 `ok` 수신 동작.

---

### 2.2 실제 CNC 테스트(`2026-08-13`)에서 발생한 4대 치명적 결함 실증

#### 실증 1: 모달(Modal) G-code 미인식으로 인한 **가공 중 스핀들 정지(`M5`) 및 강제 복귀 사고**
* **증거 로그**: [`cnc_log_20260813_170514.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_170514.csv), [`170738.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_170738.csv), [`170841.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_170841.csv), [`171233.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_171233.csv), [`185903.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_185903.csv), [`190431.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_190431.csv) 등 총 15회 실행 기록.
* **원인 분석**:
  [`O0527.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/O0527.txt#L6-L13)의 코드는 다음과 같습니다:
  ```gcode
  G1 Z-0.2 F50
  G1 X10 F50
  M3 S10000
  X30 F70       <-- G1이 생략된 모달(Modal) 직선 보간 명령 (20mm 이동, 약 17.1초 소요)
  M5            <-- 스핀들 정지!
  G0 Z5
  G0 X0 Y0 Z0
  ```
  1. [`cnc_app.py:1141`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1141)의 `if line.startswith('G0') or line.startswith('G1'):` 조건문은 `G1 X10 F50`까지는 인식하여 1mm씩 10단계(12초 동안) 대기하며 보간했습니다(`ProcessTime = 1 ~ 12`).
  2. 그러나 9행 `X30 F70`은 `G1`로 시작하지 않으므로, 백엔드(`send_gcode`)는 **이동 시간(17.14초)을 전혀 대기하지 않고 0초 만에 통과**하며 좌표도 `(10.0000, 0.0000, -0.2000, 12)`로 동결시켰습니다.
  3. 그 결과, **실제 CNC는 아직 `X=10`에서 `X=30`으로 소재를 깎으며 이동하는 도중인데 백엔드가 곧바로 `M5`(스핀들 회전 정지), `G0 Z5`, `G0 X0 Y0 Z0`를 GRBL 버퍼로 전송**해 버렸습니다.
  4. [`170738.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_170738.csv)에서 `(10.0000, 0.0000, -0.2000, 12)`가 4줄 연속 찍히며 종료된 것과, 이후 여러 로그에서 `X=8.0`~`10.0` 부근에서 반복적으로 수동 정지된 이유가 바로 이 버그 때문입니다.

#### 실증 2: 복합 G-code([`O1001.nc`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/O1001.nc)) 실행 시 **GRBL 시리얼 버퍼 폭주 및 좌표 고착 현상**
* **증거 로그**: [`cnc_log_20260813_171051.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_171051.csv)(309행), [`172144.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_172144.csv)(67행), [`172204.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_172204.csv)(148행).
* **원인 분석**:
  1. [`O1001.nc`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/O1001.nc#L15-L35)는 CAM에서 생성된 표준 G-code로, 22행 `G1 Y-17.543` 이후 23~315행 전체가 `X-15.087 Y-16.589` 형태의 모달 좌표 명령 및 `G3`(원호 보간), `Z15`/`Z2.1`/`Z-0.5` 단독 축 이동으로 이루어져 있습니다.
  2. [`cnc_log_20260813_171051.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/logs/cnc_log_20260813_171051.csv#L52-L310)를 보면 **52행부터 310행까지 258줄 내내 경과 시간이 `ProcessTime = 2`초로 고정**되어 있고 좌표도 `(-15.4700, -17.5430, 1.5000)`에서 단 1mm도 변하지 않았습니다.
  3. 백엔드가 모달 이동 시간을 계산하지 않고 수백 줄의 G-code를 2초 만에 GRBL로 쏟아부으면서 GRBL의 플래너/수신 버퍼가 포화되어 `ok` 응답이 지연되었고, 이때 [`cnc_app.py:1106`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1106)의 타임아웃 재전송 버그(`start_resp_time`이 `for attempt in range(3):` 루프 밖에 위치)까지 겹치며 중복 명령이 전송되어 3번 모두 도중에 가공이 중단되었습니다.

#### 실증 3: `2026-08-13` 실제 구동 내내 **이상 진동 감지 및 자동 재가공이 단 한 번도 동작하지 않음**
* **증거 파일**: [`cnc_system/Vibration_detection.csv`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/Vibration_detection.csv) 및 [`cnc_system/rework.nc`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/rework.nc)의 마지막 수정 시각이 `2026-08-13`이 아닌 `2026-07-09`(내부 기록 시각 `2025-11-26 15:14:21`)로 남아 있음.
* **원인 분석**:
  1. [`vibration_monitor.py:298`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L298)에 `base_path = "C:\\Users\\ace\\Desktop"`이 하드코딩되어 있어, `C:\cnc_system`에서 실행된 서버는 `master_dataset.csv`를 찾지 못해 `FileNotFoundError`를 발생시키고 `self.is_trained = False`가 되었습니다.
  2. 그 결과 [`analyze_realtime_data`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L188-L190)가 매번 `"모델이 학습되지 않았습니다"` 경고를 내며 즉시 `return False`를 반환하여, 실제 CNC 구동 당일 이상 진동 감지가 완전히 무력화되었습니다.
  3. 설령 학습이 되었더라도 [`구동방법.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/구동방법.txt)에 `python watcher.py` 실행 절차가 빠져 있고 [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py)가 `watcher.py`를 자동 실행하지 않으므로 자동 재가공(`load_rework_file`)이 트리거될 수 없었습니다.

#### 실증 4: `__pycache__` 바이트코드 역공학으로 확인된 **구버전/신버전 재가공 아키텍처 충돌**
* **증거 파일**: `ProjectLab_System/__pycache__/rework.cpython-312.pyc` vs `cnc_system/__pycache__/rework.cpython-312.pyc`
* **원인 분석**:
  1. `ProjectLab_System/__pycache__/rework.cpython-312.pyc`(구버전)의 바이트코드 심볼을 추출하면 함수 인자가 6개(`gcode_lines, tolerance_value, feed, spindle, cond_csv_path, output_path`)였습니다.
  2. 이후 [`rework.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L5)가 `CuttingConditionOptimizer`와 연동되면서 10개 인자(`current_n, current_F, optimal_n, optimal_F, x_pos, y_pos, z_pos, gcode_lines=None, tolerance_value=0.0, output_path="rework.nc"`)로 변경되었으나, [`routes.py:971`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L971)의 `create_rework_file()`은 여전히 구버전 4인자 방식(`spindle=23000` 고정)으로 호출하고 있습니다.
  3. 게다가 [`vibration_monitor.py:472`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L472)가 `Vibration_detection.csv`를 생성하면 [`watcher.py:65`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py#L65-L96)가 이를 감지해 `make_rework` 이벤트로 구버전 [`routes.py:971`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L971)을 호출(`TypeError` 발생)하는 동시에, [`vibration_monitor.py:524`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L524)도 직접 `rework.create_rework_nc()`를 호출하는 **이중 충돌 구조**가 남아 있습니다.

---

## 3. `ProjectLab_System`을 실제 CNC와 완벽히 연동하기 위한 수정 가이드

[`ProjectLab_System`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System)이 실제 CNC 장비에서 단순 이동뿐만 아니라 **복합 G-code 가공 + 실시간 이상 진동 감지 + 최적 조건 도출 + 안전한 자동 재가공**까지 끊김 없이 수행하도록 하기 위해 아래 **5단계 수정 사항**을 적용해야 합니다.

---

### Step 1. 경로 하드코딩 제거 및 사전 학습 모델/베이스라인 로드 수정

#### 1) [`vibration_monitor.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L290-L328)의 `initialize_analyzer()` 수정
* **문제점**: `C:\Users\ace\Desktop\master_dataset.csv` 절대 경로가 없으면 `is_trained = False`가 되어 진동 감지가 영구 불능 상태가 됩니다.
* **수정 방안**: 프로젝트 루트(`os.path.dirname(os.path.abspath(__file__))`) 아래의 `학습 데이터/*.csv` 파일들(정상 데이터)을 자동으로 읽어 `StandardScaler`, `LOF`, `RMS Baseline`을 초기화하거나, 이미 존재하는 [`lof_model.joblib`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/lof_model.joblib)을 활용하도록 변경합니다.

```python
# vibration_monitor.py:290 수정 예시
    def initialize_analyzer(self):
        """진동 분석기 초기화 및 모델 학습 (상대 경로 및 학습 데이터 폴더 폴백 지원)"""
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            candidate_files = [
                os.path.join(base_dir, "master_dataset.csv"),
            ]
            # master_dataset.csv가 없으면 '학습 데이터' 폴더 내 정상 CSV 파일들을 자동 탐색
            train_data_dir = os.path.join(base_dir, "학습 데이터")
            if os.path.isdir(train_data_dir):
                for fn in sorted(os.listdir(train_data_dir)):
                    if fn.endswith(".csv") and "이상" not in fn:
                        candidate_files.append(os.path.join(train_data_dir, fn))

            all_normal_data = []
            for file_path in candidate_files:
                if not os.path.exists(file_path):
                    continue
                try:
                    df = pd.read_csv(file_path)
                    if {'AccX', 'AccY', 'AccZ'}.issubset(df.columns):
                        vals = df[['AccX', 'AccY', 'AccZ']].dropna().values
                    elif {'X', 'Y', 'Z'}.issubset(df.columns):
                        vals = df[['X', 'Y', 'Z']].dropna().values
                    else:
                        continue
                    all_normal_data.extend(vals)
                    logger.info(f"{os.path.basename(file_path)} 로드 완료 ({len(vals)}건)")
                except Exception as e:
                    logger.warning(f"{file_path} 로드 중 오류: {e}")

            if not all_normal_data:
                raise FileNotFoundError("학습 가능한 정상 진동 CSV 데이터를 찾을 수 없습니다.")

            normal_data = np.array(all_normal_data, dtype=float)
            self.train_model(normal_data)
            logger.info(f"진동 분석기 초기화 완료 (총 {len(normal_data)}개 샘플 학습)")
            return True
        except Exception as e:
            logger.error(f"진동 분석기 초기화 실패: {e}")
            return False
```

#### 2) [`optimizer.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L22-L24)의 모델 경로를 스크립트 기준 절대 경로로 고정
* 실행 위치(`cwd`)가 달라도 항상 [`ProjectLab_System`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System) 내부의 [`regr_model.joblib`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/regr_model.joblib), [`lof_model.joblib`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/lof_model.joblib)을 로드하도록 수정합니다:
  ```python
  MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
  REGR_MODEL_PATH = os.path.join(MODEL_DIR, "regr_model.joblib")
  LOF_MODEL_PATH = os.path.join(MODEL_DIR, "lof_model.joblib")
  ```

---

### Step 2. 실제 CNC(GRBL) 스트리밍 동기화 및 모달 G-code 처리 수정 ([`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py))

`2026-08-13` 실제 CNC 로그에서 확인된 **가공 중 스핀들 조기 정지(`M5`)**, **모달 좌표 누락**, **버퍼 오버플로우/중복 재전송**을 해결하기 위한 핵심 수정입니다.

#### 1) 모달(Modal) G-code 상태 추적 및 `M5`/`M30` 전 물리적 이동 완료 대기 (`Idle` 동기화)
* **문제점**: [`cnc_app.py:1141`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1141)은 `line.startswith('G0') or line.startswith('G1')`일 때만 좌표 이동 및 이송 대기 시간을 계산합니다. [`O0527.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/O0527.txt#L9)의 `X30 F70`이나 [`O1001.nc`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/O1001.nc#L16-L315)의 `Z15`, `X-15.087 Y-16.589` 같은 모달 이동 명령을 만나면 대기 시간 없이 다음 줄(`M5` 등)을 즉시 전송합니다.
* **수정 방안**:
  1. `active_motion_mode`(`'G0'`, `'G1'`, `'G2'`, `'G3'`) 변수를 유지하여, 행에 `G0`/`G1`/`G2`/`G3`가 없더라도 `X`, `Y`, `Z` 좌표가 존재하면 직전 모달 모드로 이동 거리와 대기 시간을 계산합니다.
  2. `M5`(스핀들 정지)나 `M30`(프로그램 종료)을 전송하기 직전에는 GRBL에 `?` 상태 질의를 폴링하여 **기계 상태가 `<Idle|...>`이 될 때까지(물리적 절삭 이동이 완전히 끝날 때까지) 대기**한 후 `M5`를 보내도록 수정합니다.

#### 2) [`send_gcode`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1106-L1108) 내 응답 타임아웃 재시도 루프 수정 및 시리얼 락(`ser_lock`) 도입
* [`cnc_app.py:1106`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1106)의 `start_resp_time = time.time()`을 `for attempt in range(3):` 루프 **내부**로 이동시켜 2·3차 재시도 시에도 정상적으로 3초간 응답을 대기하도록 수정합니다.
* 전역 `ser_lock = threading.RLock()`을 선언하여 [`send_gcode`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L924), [`check_alarm`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1277), [`check_connection`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1326), [`handle_check_alarm_status`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L483)가 동시에 `ser.reset_input_buffer()`나 `ser.readline()`을 호출해 GRBL의 `ok` 응답을 가로채지 못하도록 보호합니다.

#### 3) 일시정지 후 재개 시 처음부터 다시 가공하는 버그 수정
* [`routes.py:269`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L269) 및 [`cnc_app.py:938-941`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L938-L941)에서 `state == "paused"` 또는 `pause == True`일 때는 `gcode_current_idx = 0`으로 초기화하지 않고 중단된 인덱스부터 이어서 전송하도록 수정합니다.

---

### Step 3. 이상 진동 긴급 정지 $\rightarrow$ 최적 조건 도출 $\rightarrow$ 자동 재가공 파이프라인 수정

#### 1) [`vibration_monitor.py:524-557`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L524-L557): GRBL 리셋/알람 해제와 `rework.nc` 생성 순서 역전 수정
* **문제점**: 현재 코드는 `rework.create_rework_nc()`를 **먼저** 호출해 파일을 생성한 뒤, `time.sleep(2.0)` 동안 대기하면서 GRBL에 `\x18`(소프트 리셋)과 `$X`(알람 해제)를 보냅니다. 이 때문에 [`watcher.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py#L128)가 파일 생성 즉시 재가공을 시작해 버리고, 재가공이 한창 진행 중일 때 2초 뒤 날아온 `\x18`이 GRBL을 강제 리셋시키는 레이스 컨디션이 발생합니다.
* **수정 방안**:
  1. GRBL 소프트 리셋(`\x18`) $\rightarrow$ `2.0초` 대기 $\rightarrow$ 알람 해제(`$X\n`) $\rightarrow$ `1.0초` 대기 $\rightarrow$ `self.controllers.state = "alarm_unlocked"` 설정을 **먼저 완료**합니다.
  2. GRBL이 완전히 준비된 **직후에** `rework.create_rework_nc(...)`를 호출하여 파일을 생성합니다.

#### 2) [`rework.py:91-128`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L91-L128): 재가공 진입 시 공구/소재 측면 충돌 방지
* **문제점**: `rework.py`가 `before_z_lines_cutting`(시작 X, Y 위치로의 급속 이동 등)보다 `G1 Z{original_z_value:.3f}`(Z축 하강)를 먼저 배치하여, 현재 위치에서 곧바로 Z축을 파고든 뒤 수평 이동하는 위험한 경로를 생성합니다.
* **수정 방안**:
  1. 재가공 시작 시 안전 높이 확보(`G0 Z5.0`) 및 시작점(`G0 X0 Y0` 또는 `before_z_lines_cutting`) 수평 이동을 **먼저** 수행한 후 `G1 Z...`로 하강하도록 순서를 교정합니다.

```python
# rework.py:91-128 수정 예시
    # --- 첫 번째 가공 경로 생성 (원본 Z 사용) ---
    rework_gcode.extend([
        "G21",
        "G90",
        "G0 Z5.0",
        "G0 X0 Y0",
        f"M3 S{int(optimal_n)}",
    ])
    for line in before_z_lines_cutting:
        rework_gcode.append(_apply_optimal_params(line, optimal_n, optimal_F))
    rework_gcode.append(f"G1 Z{original_z_value:.3f} F{int(optimal_F)}")
    for line in after_z_lines_cutting:
        rework_gcode.append(_apply_optimal_params(line, optimal_n, optimal_F))
```

#### 3) [`watcher.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py) 및 [`routes.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py)의 레거시 충돌 제거 및 자동 실행 연동
* [`watcher.py:61-101`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py#L61-L101): `Vibration_detection.csv` 생성 시 구버전 `make_rework` 이벤트를 호출하는 `on_created` 블록을 비활성화하고, `rework.nc` 생성/수정 시 `reset_path` 후 `load_rework_file`을 호출하도록 일원화합니다.
* [`routes.py:971`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L971): 수동 `/make_rework` 호출 시에도 에러가 나지 않도록 [`create_rework_nc`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L5)의 10인자 시그니처에 맞춰 호출 인자를 수정합니다.
* [`routes.py:1055`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L1055): `handle_load_rework_file()`에서 `controllers.send_gcode()` 호출 전에 `controllers.state = "running"`을 미리 설정하는 줄을 제거합니다(그래야 `send_gcode()` 내부의 `if state != "running":` 블록이 정상 실행되어 재가공 중에도 진동 감지와 CSV 로깅이 켜집니다).
* [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py): 별도로 `python watcher.py`를 켜는 것을 잊지 않도록 서버 시작 시 백그라운드 데몬 스레드로 `watcher`를 함께 기동하거나 시작 스크립트에 통합합니다.

#### 4) [`optimizer.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L62-L94)의 수식 결함 교정
* [`time_proxy`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L81-L83)에서 `fz = F / S` 후 `1.0 / (fz * S)`를 계산하면 `S`가 약분되어 스핀들 회전수가 최적화 시간 지표에서 완전히 사라집니다. 또한 [`lof_score`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L62-L72)에서 `-self.lof.score_samples(v)[0]`는 값이 **작을수록(1.0에 가까울수록) 정상(Inlier)**이고 클수록 이상치(Outlier)이므로, `sc <= TAU_MAX`(또는 `better()`에서 `a["score"] < b["score"]`) 방향으로 부등호를 교정해야 실제로 진동이 작은 안전 조건이 선택됩니다.

---

### Step 4. 프론트엔드 안전성(초기 공차 10mm 급강하 방지) 및 미구현 메서드 수정

1. **초기 공차 `10mm` 자동 덮어쓰기 방지 (최우선 안전 조치)**:
   * [`templates/index.html:80`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/templates/index.html#L80)의 `value="10"`을 `value="0.2"`(0.2mm)로 수정하고, [`static/script.js:32`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L32)의 `let toleranceValue = 10;`을 `let toleranceValue = 0.2;`로 수정합니다.
   * 수정하지 않을 경우 웹 페이지 접속 즉시 `/set_tolerance`로 `10.0mm`가 전송되어 2차 재가공 시 Z축이 `-10.2mm`로 급강하해 엔드밀 파손 및 스핀들 충돌이 발생합니다.
2. **미정의 함수/메서드 호출 수정**:
   * [`static/script.js:2449`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2449): 존재하지 않는 `requestPortList()` 호출을 `updateSerialPorts()`로 변경하여 알람 상태 체크 타이머가 중단되지 않게 합니다.
   * [`static/script.js:2693-2860`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2693-L2860): 3중으로 등록된 `DOMContentLoaded` 이벤트 리스너를 하나로 통합하여 버튼 이벤트 중복 바인딩(클릭 1회에 요청 2~4회 전송)을 제거합니다.
   * [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py): [`routes.py:923`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L923)에서 호출하는 `apply_override(override_type, value)` 함수를 [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py)에 구현하고, [`cnc_app.py:1515`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1515)의 `vibration_monitor.set_fixed_spindle` 호출을 [`VibrationMonitor`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L21) 메서드로 추가합니다.

---

### Step 5. 아두이노 센서 통신 규격 및 실행 스크립트 정비

1. **아두이노 보레이트 및 샘플링 주기 일치**:
   * [`MPU6050.ino`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/MPU6050.ino#L30)는 `115200 bps`로 `AccX,AccY,AccZ` CSV를 전송하는 반면, [`cnc_app.py:62`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L62)는 기본 보레이트가 `1000000`으로 설정되어 있습니다. 실제 아두이노 펌웨어에 맞춰 기본 보레이트를 통일하고, [`MPU6050.ino:64`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/MPU6050.ino#L64)의 `delay(500)`(2Hz)을 고속 샘플링 주기(`delay(1)` 등)로 수정해야 `BUFFER_SIZE = 1000` 버퍼가 제때 채워져 실시간 진동 감지가 동작합니다.
2. **구동 환경 및 [`requirements.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/requirements.txt) 업데이트**:
   * [`cnc_system/구동방법.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/cnc_system/구동방법.txt)에서 확인된 바와 같이 Conda 환경(`cnc_system`, Python 3.12) 또는 신규 `venv`에서 바로 패키지를 설치할 수 있도록 [`requirements.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/requirements.txt)에 누락된 `scikit-learn`, `scipy`, `pandas`, `joblib`, `watchdog`, `requests`, `python-socketio[client]`를 명시합니다.

---

## 4. 수정 우선순위 요약 체크리스트

| 우선순위 | 대상 파일 | 핵심 수정 내용 | 해결되는 실제 문제 |
| :---: | :--- | :--- | :--- |
| **P0 (즉시)** | [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py) | 모달 G-code(`X30 F70`, `Z15` 등) 이동 시간 계산 및 `M5`/`M30` 전 GRBL `Idle` 대기, `start_resp_time` 루프 내부 이동, `ser_lock` 추가 | `2026-08-13` 실증 문제: 가공 도중 스핀들이 멈추거나(`M5`) `O1001.nc` 실행 시 `t=2s`에서 버퍼 폭주로 멈추는 현상 해결 |
| **P0 (즉시)** | [`vibration_monitor.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py) | `C:\Users\ace\Desktop` 제거 후 `학습 데이터/*.csv` 자동 로드, GRBL 리셋/알람 해제 완료 후 `rework.nc` 생성 | `2026-08-13` 실증 문제: 모델 미학습으로 진동 감지가 전혀 작동하지 않던 문제 및 재가공 중 리셋 충돌 해결 |
| **P0 (즉시)** | [`index.html`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/templates/index.html), [`script.js`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js), [`rework.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py) | 초기 공차 `10` $\rightarrow$ `0.2` 변경, 재가공 시 `(X, Y)` 수평 이동 후 `Z` 하강 순서 교정 | 재가공 시 Z축 `-10.2mm` 급강하 및 측면 충돌로 인한 공구 파손 방지 |
| **P1 (높음)** | [`routes.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py), [`watcher.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py) | `handle_load_rework_file`의 `state="running"` 선행 설정 제거, 구버전 `make_rework` 충돌 제거, 일시정지 재개 시 인덱스 보존 | 자동 재가공 트리거 정상화 및 일시정지 후 이어하기 정상화 |
| **P2 (보통)** | [`optimizer.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py), [`MPU6050.ino`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/MPU6050.ino), [`requirements.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/requirements.txt) | `lof_score` 부호 및 `time_proxy` 수식 수정, 아두이노 보레이트/샘플링 주기 통일, 의존성 명시 | 최적 절삭 조건 탐색 정확도 확보 및 실시간 센서 수집 안정화 |

