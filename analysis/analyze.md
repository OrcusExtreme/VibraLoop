# ProjectLab 2026 전체 파일 구조 및 코드 종합 분석 보고서 (`analyze.md`)

- **분석 대상 루트**: `D:/Project_Lab/02_공모전_및_경진대회/ProjectLab 2026`
- **핵심 시스템 경로**: `D:/Project_Lab/02_공모전_및_경진대회/ProjectLab 2026/ProjectLab_System`
- **문서 경로**: `D:/Project_Lab/02_공모전_및_경진대회/ProjectLab 2026/프로젝트 LAB 문서/프로젝트 랩`

---

## 목차
1. [프로젝트 전체 파일 구조 (전수 인벤토리)](#0-프로젝트-전체-파일-구조-전수-인벤토리)
2. [실행 흐름 및 아키텍처 요약](#1-실행-흐름-및-아키텍처-요약)
3. [잠재적 버그, 예외 처리 누락, 논리적 결함 식별](#2-잠재적-버그-예외-처리-누락-논리적-결함-식별)
4. [모듈 간 의존성 및 환경 설정(스크립트, 라이브러리, 문서)의 불일치 문제점](#3-모듈-간-의존성-및-환경-설정스크립트-라이브러리-문서의-불일치-문제점)
5. [개선 및 수정을 위한 우선순위 가이드](#4-개선-및-수정을-위한-우선순위-가이드)

---

## 0. 프로젝트 전체 파일 구조 (전수 인벤토리)

루트 디렉토리(`ProjectLab 2026`) 하위에는 크게 **아카이브 압축 파일**, **특허/발표 문서 디렉토리**, **핵심 소프트웨어 디렉토리(`ProjectLab_System`)**가 존재합니다.

```text
ProjectLab 2026/
├── analyze.md                                  # (본 문서) 프로젝트 전체 종합 분석 보고서
├── CNC시스템.zip                                # 시스템 소스코드 백업 아카이브
├── 프로젝트 LAB 문서.zip                         # 프로젝트 산출물 문서 백업 아카이브
├── 프로젝트 LAB 문서/
│   └── 프로젝트 랩/
│       ├── (P2026-002) 발명신고서_윤주성_CNC 가공공정에 대한 진동 데이터 학습 기반 실시간 이상탐지 및 자율제어 시스템_서명.hwpx
│       ├── (YP26-22)_경남대학교 산학협력단_명세서 초안본_CNC 가공 공정에 대한 진동 데이터 학습 기반 실시간 이상탐지 및 자율제어 시스템(26.05.08).hwp
│       ├── 경남대학교 산학협력단_명세서 초안본_CNC 가공 공정에 대한 진동 데이터 학습 기반.pdf  (48p 특허 명세서)
│       ├── 발명신고서_윤주성_CNC 가공공정에 대한 진동 데이터 학습 기반 실시간 이상탐지 및 자율제어 시스템_서명.pdf (16p 발명신고서)
│       └── 발표자료_프로젝트 LAB.pdf             (20p 최종 발표자료)
└── ProjectLab_System/                          # 메인 소프트웨어 시스템 루트
    ├── cnc_app.py                              # [Backend Core] Flask/SocketIO 메인 서버, GRBL/Arduino 시리얼 통신, G-code 스트리밍 및 상태 머신 (1,586줄)
    ├── routes.py                               # [Backend API] HTTP REST 엔드포인트 및 Socket.IO 이벤트 핸들러 (1,069줄)
    ├── vibration_monitor.py                    # [AI/Signal] 실시간 진동 필터링(Median/Hampel), STFT 특징 추출, LOF/RMS 이상 감지 및 복구 트리거 (629줄)
    ├── optimizer.py                            # [AI/Optimizer] RandomForest 회귀 예측, 실시간 잔차 보정, LOF 안정성 평가 및 5단계 최적 절삭 조건 탐색 (200줄)
    ├── rework.py                               # [CAM/G-code] 최적 절삭 조건(S, F) 및 허용 공차(Z)를 반영한 2-Pass 재가공 G-code 생성기 (147줄)
    ├── watcher.py                              # [File Watcher] watchdog 기반 파일 감시(Vibration_detection.csv, rework.nc) 및 소켓 트리거 (167줄)
    ├── train_models.py                         # [ML Training] STFT 3축 데이터셋 기반 RandomForest 회귀 및 LOF 모델 오프라인 학습/저장 스크립트 (57줄)
    ├── arduino_sensor.py                       # [Simulator/CLI] 독립형 아두이노 시리얼 리더 및 사인파 기반 가상 진동 데이터 시뮬레이터 (253줄)
    ├── test_arduino_sensor.py                  # [Test] arduino_sensor.py의 포트 미지정/오류 시 시뮬레이션 모드 전환 검증 스크립트 (73줄)
    ├── regr_model.joblib                       # [Model] 사전 학습된 MultiOutputRegressor(RandomForestRegressor) 모델 바이너리
    ├── lof_model.joblib                        # [Model] 사전 학습된 LocalOutlierFactor(novelty=True) 모델 바이너리
    ├── Vibration_detection.csv                 # [State/Log] 이상 진동 감지 시점, 위치(X,Y,Z), 스핀들 오버라이드를 기록한 트리거 파일
    ├── current_position.txt                    # [State] 가공 시작 시 GRBL MPos Z축 좌표를 기록하는 상태 파일
    ├── O0527.txt / O0807.txt / O1001.nc        # [NC Data] 직선 가공 및 3D 포켓/윤곽 가공용 테스트 G-code 파일
    ├── rework.nc                               # [NC Data] 이상 감지 후 자동 생성된 2-Pass 후가공 G-code 파일
    ├── requirements.txt                        # [Config] Python 패키지 의존성 명세서
    ├── README.md / README_ARDUINO.md           # [Docs] 시스템 실행 가이드 및 아두이노 센서 연결 매뉴얼
    ├── full.md                                 # [Docs] 기존 작성된 시스템 분석 문서
    ├── MPU6050.ino                             # [Firmware] 아두이노 MPU6050 I2C 센서 펌웨어 (115200 bps, 500ms 주기, 중첩 JSON)
    ├── MPU6050/
    │   └── MPU6050.ino                         # [Firmware] 아두이노 MPU6050 I2C 센서 펌웨어 (115200 bps, 100ms 주기, 중첩 JSON)
    ├── arduino_example/
    │   └── sensor_example.ino                  # [Firmware] ADXL335 + LM35 아날로그 센서 예제 펌웨어 (9600 bps, 100ms 주기, 플랫 JSON)
    ├── templates/
    │   └── index.html                          # [Frontend] 메인 웹 대시보드 UI 템플릿 (337줄)
    ├── static/
    │   ├── script.js                           # [Frontend] Socket.IO 통신, Chart.js 경로 시각화, Jog/Override 제어, CSV 내보내기 로직 (2,825줄)
    │   ├── style.css                           # [Frontend] 다크 테마 UI 레이아웃, 반응형 스타일, 모달/알림 애니메이션 (1,984줄)
    │   ├── favicon.ico                         # [Asset] 웹 브라우저 파비콘
    │   └── alert.txt                           # [Asset Note] UTF-16LE 인코딩 텍스트 ("static/alert.mp3 경고음 파일 추가 필요" 안내문)
    ├── 학습 데이터/                              # [Dataset] 3축 가속도 및 자이로 센서 수집 데이터셋 (총 19개 CSV 파일)
    │   └── 센서_데이터_2025-05-31T17-04-14.csv 외 18개
    ├── logs/                                   # [Runtime Logs] 가공 좌표 및 경과시간 기록 로그 (총 986개 CSV 파일)
    │   ├── cnc_log_20250508_145631.csv ~ cnc_log_20251128_180527.csv (984개)
    │   └── upload_20250509_173142.csv, upload_20250509_173344.csv (2개)
    └── venv/                                   # [Env] Python 3.13.5 가상환경 (타 PC 경로 C:\Users\ace\anaconda3 하드코딩됨)
```

---

## 1. 실행 흐름 및 아키텍처 요약

### 1.1 시스템 아키텍처 개요
본 프로젝트는 **GRBL v1.1 기반 3축 CNC 밀링 머신**과 **3축 가속도 센서(MPU6050 등)**를 결합하여, 웹 대시보드에서 실시간으로 가공 경로 및 장비 상태를 모니터링/제어하고, 가공 중 발생하는 **이상 진동(채터 등)을 비지도 학습(LOF) 및 RMS 기반으로 실시간 탐지**하여 **즉시 가공 정지 $\rightarrow$ 최적 절삭 조건($S, F$) 탐색 $\rightarrow$ 후가공 G-code(`rework.nc`) 자동 재생성 및 재가공**을 수행하는 자율제어 시스템입니다.

```mermaid
flowchart TB
    subgraph Frontend ["클라이언트 계층 (Browser UI)"]
        UI["templates/index.html\nstatic/style.css"]
        JS["static/script.js\n(Socket.IO Client, Chart.js 2D/Z-Axis)"]
    end

    subgraph Backend ["웹/제어 서버 계층 (Flask + Socket.IO)"]
        App["cnc_app.py\n- 전역 상태 머신 (state, ser, gcode_lines)\n- G-code 버퍼 스트리밍 (send_gcode)\n- 시리얼/알람/센서 백그라운드 스레드"]
        Routes["routes.py\n- REST API (/set_tolerance, /api/set-zero 등)\n- Socket.IO 핸들러 (connect, jog, start, pause 등)"]
    end

    subgraph Intelligence ["AI 이상 탐지 및 자율 후가공 계층"]
        VMon["vibration_monitor.py (VibrationMonitor)\n- Median + Hampel 스무딩\n- 슬라이딩 윈도우 & STFT (12차원 특징)\n- LOF 이상치 + RMS 베이스라인 판정"]
        Opt["optimizer.py (CuttingConditionOptimizer)\n- RandomForest 회귀 예측 (regr_model.joblib)\n- 실시간 잔차 보정 (r = phi_now - phi_hat)\n- LOF 점수 평가 (lof_model.joblib) & 5단계 탐색"]
        Rework["rework.py (create_rework_nc)\n- 최적 S, F 치환 및 허용 공차(Z-tol) 반영\n- 2-Pass 후가공 rework.nc 생성"]
        Watcher["watcher.py (FileHandler / watchdog)\n- Vibration_detection.csv 감지 -> reset_path / make_rework\n- rework.nc 감지 -> load_rework_file"]
    end

    subgraph Hardware ["하드웨어 및 데이터 계층"]
        GRBL["CNC 컨트롤러 (GRBL 115200 bps)\nG-code 실행, 상태 쿼리(?), 소프트 리셋(0x18), 알람 해제($X)"]
        Sensor["아두이노 진동 센서 (MPU6050.ino 115200 bps)\n3축 가속도(accel) 및 자이로(gyro) JSON 스트리밍"]
        Files["로컬 파일 시스템\n- 학습 데이터/*.csv\n- logs/cnc_log_*.csv\n- rework.nc / Vibration_detection.csv"]
    end

    UI --- JS
    JS <==>|WebSocket / HTTP| Routes
    Routes <--> App
    App <==>|Serial (ser)| GRBL
    App <==>|Serial (arduino_serial)| Sensor
    App --> VMon
    VMon --> Opt
    VMon --> Rework
    Rework --> Files
    Files -.->|FileSystemEvent| Watcher
    Watcher -.->|Socket.IO Client (localhost:5000)| Routes
```

### 1.2 모듈별 상세 역할
1. **[cnc_app.py](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py)**:
   - 엔트리 포인트(`__main__`)로서 Flask 앱과 SocketIO 서버를 생성하고, `controllers = sys.modules[__name__]`를 통해 모듈 자체를 상태 컨테이너로 [`init_routes()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L19)에 주입합니다.
   - 4개의 상시 데몬 스레드([`check_alarm`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L698), [`check_connection`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1288), [`read_arduino_serial`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L292), [`start_nc_file_watcher`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1526))와 가공 시 생성되는 [`send_gcode`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L897) 스레드를 관리합니다.
2. **[routes.py](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py)**:
   - 웹 페이지 렌더링(`/`), 원점 설정(`/api/set-zero`), 허용 공차 설정(`/set_tolerance`), 후가공 파일 생성(`/make_rework`), 로그 다운로드(`/download_logs`) 등의 HTTP 라우트와 포트 연결, NC 파일 로드, 가공 시작/일시정지/긴급정지/알람해제, 조그(Jog) 이동, 오버라이드(Override), 상태 동기화(`request_status`) 등 Socket.IO 이벤트를 처리합니다.
3. **[vibration_monitor.py](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py)**:
   - [`VibrationMonitor`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L21) 클래스가 아두이노로부터 수신된 3축 가속도 데이터를 3-샘플 Median 및 Hampel 필터로 정제한 후 `sensor_data_buffer`(크기 1,000)에 적재합니다.
   - 버퍼가 차면 [`analyze_realtime_data()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L185)를 통해 축별 `StandardScaler` 정규화 $\rightarrow$ 슬라이딩 윈도우(크기 100, 스텝 25) 분할 $\rightarrow$ STFT(`nperseg=64`) 기반 축별 4개 특징(Mean, Std, Max, Energy 합, 총 12차원) 추출 $\rightarrow$ LOF 이상치 예측 및 윈도우별 RMS 임계값 초과 비율을 결합하여 이상 여부를 판단합니다.
   - 1초(`MIN_ABNORMAL_DURATION`) 이상 이상 진동 지속 시 [`handle_abnormal_vibration()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L436)을 호출하여 GRBL에 즉시 소프트 리셋(`0x18`)을 보내고, 최적 절삭 조건 도출 및 `rework.nc` 생성을 수행합니다.
4. **[optimizer.py](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py) & [train_models.py](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/train_models.py)**:
   - [`train_models.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/train_models.py#L21)에서 절삭 조건 $X=(n, F)$로 12차원 STFT 특징 $Y$를 예측하는 `MultiOutputRegressor(RandomForestRegressor)`와 정상 STFT 분포를 학습한 `LocalOutlierFactor(novelty=True)`를 각각 `regr_model.joblib`, `lof_model.joblib`으로 저장합니다.
   - [`CuttingConditionOptimizer`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L26)는 이상 진동 발생 시점의 실측 STFT 특징($\phi_{\text{now}}$)과 회귀 모델 예측값($\hat{\phi}$)의 차이인 잔차 $r = \phi_{\text{now}} - \hat{\phi}$를 업데이트하고([`update_residual`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L74)), 5단계 탐색([`optimize_staged`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L95): $\pm3\%$ 휴리스틱 $\rightarrow$ $\pm5\%$ 휴리스틱 $\rightarrow$ $\pm10\%$ 3×3 그리드 $\rightarrow$ $\pm20\%$ 5×5 그리드 $\rightarrow$ 전체 도메인 10×10 그리드)을 통해 안전 기준을 만족하면서 가공 시간 대리변수(`time_proxy`)가 최소인 최적 $(n^*, F^*)$를 반환합니다.
5. **[rework.py](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py) & [watcher.py](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py)**:
   - [`create_rework_nc()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L5)는 원본 G-code에서 핵심 절삭 경로를 추출한 뒤, 최적 조건 $(n^*, F^*)$로 원본 $Z$ 깊이를 가공하는 1차 패스와 허용 공차만큼 깊이를 더한($Z_{\text{new}} = Z_{\text{orig}} - \text{tolerance}$) 2차 패스를 결합하여 `rework.nc`를 작성합니다.
   - [`FileHandler`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py#L19)는 파일 시스템 이벤트를 감시하여 `Vibration_detection.csv` 생성 시 경로 초기화/재가공 파일 생성을 요청하고, `rework.nc` 수정 시 서버에 `load_rework_file` 이벤트를 보내 자동 재가공을 시작합니다.

### 1.3 단계별 런타임 실행 흐름
1. **부팅 단계**: `python cnc_app.py` 실행 $\rightarrow$ `logs/` 폴더 생성 $\rightarrow$ 4개 백그라운드 데몬 스레드(`check_alarm`, `check_connection`, `read_arduino_serial`, `start_nc_file_watcher`) 기동 $\rightarrow$ `0.0.0.0:5000`에서 Flask-SocketIO 서버 대기.
2. **하드웨어 연결 및 준비 단계**: 웹 UI 접속 시 `get_ports`로 COM 포트 목록 조회 $\rightarrow$ CNC 컨트롤러 연결(`connect_port` $\rightarrow$ [`connect_serial`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L143), DTR 리셋 후 `?` 및 `$$` 명령으로 상태/설정 확인) $\rightarrow$ 알람 상태일 경우 `$X`로 해제(`unlock_alarm`) $\rightarrow$ 아두이노 센서 포트 연결(`connect_sensor` $\rightarrow$ [`connect_arduino`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L421)).
3. **NC 코드 로드 및 가공 스트리밍 단계**: 사용자가 `.nc`/`.txt` 파일 업로드(`load_nc_file` $\rightarrow$ [`load_nc_code`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L503)) $\rightarrow$ 비지원 M코드/주석 제거 및 예상 가공 시간([`calculate_estimated_time`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L545)) 산출 $\rightarrow$ `start_gcode` 이벤트로 [`send_gcode`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L897) 스레드 시작 $\rightarrow$ 진동 탐지 활성화([`start_vibration_detection`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L99)) 및 CSV 로깅 시작 $\rightarrow$ GRBL 128B 버퍼 한도 내에서 G-code 전송 후 `ok` 대기 $\rightarrow$ 이동 거리 1mm 단위 선형 보간 및 UI 차트 실시간 갱신([`update_ui`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1263)).
4. **이상 진동 감지 및 자율 복구 단계**: `read_arduino_serial`이 수신한 가속도 데이터를 `vibration_monitor.process_sensor_data()`에 전달 $\rightarrow$ 1,000샘플마다 LOF+RMS 분석 $\rightarrow$ 1초 이상 이상 진동 지속 시 `handle_abnormal_vibration()` 실행 $\rightarrow$ GRBL 긴급정지(`0x18`), `Vibration_detection.csv` 작성, `CuttingConditionOptimizer`로 최적 $(n^*, F^*)$ 도출 후 `rework.nc` 생성 $\rightarrow$ `watcher.py`가 `rework.nc`를 감지하여 `load_rework_file` 트리거 $\rightarrow$ `rework.nc` 자동 로드 및 재가공 `send_gcode` 스레드 시작.

---

## 2. 잠재적 버그, 예외 처리 누락, 논리적 결함 식별

코드 전수 분석 결과, 런타임 크래시를 유발하는 치명적 버그부터 공작기계 파손 위험이 있는 제어 결함까지 총 **25건**의 구체적인 결함이 식별되었습니다.

### 2.1 [치명적] 런타임 즉시 크래시 및 기능 불능 결함

#### ① `routes.py`의 `create_rework_nc()` 인자 개수 및 타입 불일치 (`TypeError`)
- **위치**: [`routes.py` L971-L976](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L971-L976) vs [`rework.py` L5](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L5)
- **상세 원인**:
  - [`rework.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L5)의 함수 시그니처는 `def create_rework_nc(current_n, current_F, optimal_n, optimal_F, x_pos, y_pos, z_pos, gcode_lines=None, tolerance_value=0.0, output_path="rework.nc"):`로서 **7개의 필수 위치 인자**를 요구합니다.
  - 반면 [`routes.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L971-L976)의 `create_rework_file()` 내부에서는 다음과 같이 4개의 인자만 전달하고, 첫 번째 인자(`current_n`) 자리에 리스트인 `controllers.gcode_lines`를 넘깁니다:
    ```python
    success = create_rework_nc(
        controllers.gcode_lines,
        controllers.tolerance_value,
        controllers.current_feedrate,
        23000  # 스핀들 속도를 23000으로 고정
    )
    ```
- **영향**: `/make_rework` HTTP POST 요청 또는 `watcher.py`가 발생시키는 `make_rework` 소켓 이벤트 호출 시 100% `TypeError: create_rework_nc() missing 3 required positional arguments: 'x_pos', 'y_pos', and 'z_pos'` 예외가 발생하여 실패합니다.

#### ② 타 PC 절대 경로(`C:\Users\ace\Desktop`) 하드코딩으로 인한 실시간 이상 탐지 영구 불능 (`FileNotFoundError`)
- **위치**: [`vibration_monitor.py` L293-L318](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L293-L318), [`train_models.py` L12](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/train_models.py#L12)
- **상세 원인**:
  - [`VibrationMonitor.initialize_analyzer()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L290)가 `base_path = "C:\\Users\\ace\\Desktop"`에서 `"master_dataset.csv"`를 찾도록 하드코딩되어 있습니다.
  - [`train_models.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/train_models.py#L12) 또한 `STFT_DATASET_PATH = "C:\\Users\\ace\\Desktop\\stft_dataset_3axis.csv"`로 하드코딩되어 있습니다.
- **영향**:
  - `master_dataset.csv` 로드 실패로 인해 `initialize_analyzer()`가 `FileNotFoundError("정상 진동 데이터 파일을 찾을 수 없습니다.")`를 던지고 `False`를 반환합니다.
  - 그 결과 `self.is_trained`가 영구적으로 `False`로 남게 되어, 가공 중 [`analyze_realtime_data()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L185)가 호출될 때마다 `"모델이 학습되지 않았습니다"` 경고를 남기고 `False`를 반환하므로 **실시간 이상 진동 감지 기능이 전혀 동작하지 않습니다.**

#### ③ 미구현 함수/메서드 호출로 인한 `AttributeError` 2건
- **위치 1**: [`routes.py` L923](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L923) (`handle_set_feed_override`)
  - `result = controllers.apply_override("feed", value)`를 호출하나, [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py)에는 `apply_override` 함수가 존재하지 않아 웹 UI에서 Feed Override 설정 버튼 클릭 시 즉시 `AttributeError`가 발생합니다.
- **위치 2**: [`cnc_app.py` L1515](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1515) (`check_vibration`)
  - `vibration_monitor.set_fixed_spindle(FIXED_SPINDLE)`를 호출하나, [`VibrationMonitor`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L21) 클래스에는 `set_fixed_spindle` 메서드가 정의되어 있지 않습니다.

#### ④ 프론트엔드 미정의 함수 `requestPortList()` 및 스코프 불일치 변수 `isConnected` 호출 (`ReferenceError`)
- **위치**: [`static/script.js` L2449, L690, L1116, L2460](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2449)
- **상세 원인**:
  - [`script.js` L2444-L2455](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2444-L2455)의 `DOMContentLoaded` 리스너에서 `requestPortList();`를 호출하지만, 파일 전체에 해당 함수가 정의되어 있지 않아 브라우저 콘솔에 `Uncaught ReferenceError: requestPortList is not defined`가 발생합니다. 이로 인해 바로 아랫줄인 `setInterval(checkAlarmStatus, 10000);`(L2452)은 실행조차 되지 않습니다.
  - 또한 `let isConnected = false;`는 [`initSocketListeners()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L690) 내부의 지역 변수로 선언되어 있는데, 함수 외부인 [`connectPort()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L1116)와 [`checkAlarmStatus()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2460)에서 전역 변수처럼 참조하고 있습니다.

---

### 2.2 [심각] 공작기계 파손 위험 및 가공 제어/복구 시퀀스 결함

#### ⑤ 페이지 로드 시 허용 공차 `10mm` 자동 강제 설정으로 인한 과절삭/공구 파손 위험
- **위치**: [`templates/index.html` L80](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/templates/index.html#L80), [`static/script.js` L32, L2821-L2825](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2821-L2825), [`rework.py` L115](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L115)
- **상세 원인**:
  - 서버 시작 시 [`cnc_app.py` L45, L1539](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L45)에서는 `tolerance_value = 0.0`으로 초기화합니다.
  - 그러나 프론트엔드 [`script.js` L32](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L32)에는 `let toleranceValue = 10;`이 선언되어 있고, 파일 맨 마지막 줄([`script.js` L2821-L2825](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2821-L2825))에 이벤트 핸들러로 감싸지지 않은 최상위 `fetch` 호출이 존재합니다:
    ```javascript
    fetch('/set_tolerance', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: 'tolerance=' + encodeURIComponent(toleranceValue)
    });
    ```
- **영향**: 사용자가 웹 브라우저로 대시보드에 접속(또는 F5 새로고침)하는 즉시 서버의 `tolerance_value`가 **10.0mm**로 덮어써집니다. 사용자가 이를 인지하지 못한 채 가공 중 이상 진동이 발생하면, [`rework.py` L115](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L115)의 `new_z_value = original_z_value - tolerance_value`에 의해 2차 패스 절삭 깊이가 원래 `-0.2mm`에서 **`-10.2mm`로 급증**하여 공구 파손 및 스핀들 충돌 사고를 유발합니다.

#### ⑥ `rework.py`의 Z축 선행 하강으로 인한 소재 측면 충돌 위험
- **위치**: [`rework.py` L92-L105, L116-L127](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L92-L105)
- **상세 원인**:
  - `create_rework_nc()`는 원본 G-code를 `before_z_lines_cutting`(Z 절삭 하강 이전의 XY 접근 이동)과 `after_z_lines_cutting`(Z 하강 이후의 절삭 이동)으로 분리합니다.
  - 그런데 재가공 코드를 조립할 때 `before_z_lines_cutting`보다 **`G1 Z{original_z_value:.3f}`(절삭 깊이 하강)를 먼저 배치**했습니다:
    ```python
    rework_gcode.extend([
        "G21", "G90",
        f"M3 S{int(optimal_n)}",
        f"G1 Z{original_z_value:.3f} F{int(optimal_F)}" # Z축 먼저 하강!
    ])
    for line in before_z_lines_cutting:                 # 하강한 상태로 시작 XY 위치로 이동!
        rework_gcode.append(_apply_optimal_params(line, optimal_n, optimal_F))
    ```
- **영향**: 공구가 원점 $(0, 0)$이나 이전 위치에 있을 때 안전 높이($Z>0$)에서 시작점 $(X, Y)$로 이동하지 않고, 먼저 소재 깊이($Z<0$)로 파고든 상태에서 시작점 $(X, Y)$로 가로질러 이동하므로 피삭재와 공구가 충돌합니다.

#### ⑦ 이상 진동 감지 시 `VibrationMonitor`와 `watcher.py` 간의 복구 타이밍 충돌 (재가공 강제 리셋 버그)
- **위치**: [`vibration_monitor.py` L472-L557](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L472-L557) vs [`watcher.py` L65-L137](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py#L65-L137) vs [`routes.py` L1027-L1064](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L1027-L1064)
- **상세 원인 (스레드 타임라인 역전)**:
  1. `T = 0.0s`: `VibrationMonitor.handle_abnormal_vibration()`이 GRBL에 `\x18`(긴급정지)을 보내고 `Vibration_detection.csv`를 생성한 뒤, 최적 조건을 계산하여 `rework.create_rework_nc()`로 `rework.nc` 파일 생성을 완료합니다([`vibration_monitor.py` L524](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L524)).
  2. `T = 0.05s`: 직후 `VibrationMonitor`는 GRBL 복구를 위해 `\x18`(소프트 리셋)을 다시 보내고 **`time.sleep(2.0)` 동안 대기**합니다([`vibration_monitor.py` L542-L544](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L542-L544)).
  3. `T = 0.1s`: 그 사이 백그라운드 `watcher.py`의 [`on_modified`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py#L102)가 방금 생성된 `rework.nc`를 감지하여 즉시 `load_rework_file` 소켓 이벤트를 서버로 전송합니다!
  4. `T = 0.15s`: [`routes.py`의 `handle_load_rework_file()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L1027)은 **GRBL이 아직 부팅 및 알람 해제(`$X`)도 되기 전인데** `controllers.state = "running"`으로 바꾸고 `send_gcode` 재가공 스레드를 시작해버립니다!
  5. `T = 2.05s ~ 3.05s`: `time.sleep(2.0)`에서 깨어난 `VibrationMonitor`가 `$X\n`을 전송하고 `time.sleep(1.0)` 후 **`self.controllers.state = "alarm_unlocked"`로 상태를 덮어써 버립니다**([`vibration_monitor.py` L548-L554](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L548-L554)).
- **영향**:
  - 재가공이 시작된 직후 알람 상태의 GRBL에 G-code가 전송되어 에러가 나거나, 재가공 도중 `state`가 `"running"`에서 `"alarm_unlocked"`로 바뀌어 가공 완료 처리 및 진동 모니터링이 비정상적으로 깨집니다.
  - 또한 [`handle_load_rework_file()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L1055)에서 `send_gcode` 호출 전에 `controllers.state = "running"`을 미리 설정해버리기 때문에, [`send_gcode()` L947](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L947)(`if state in ["idle", "emergency", "alarm_unlocked"]:`) 조건문을 통과하지 못하고 **L990의 일시정지 재개(`else:`) 블록으로 잘못 진입**하여 재가공 시에는 `start_vibration_detection()`이 호출되지 않습니다.

#### ⑧ 일시정지(Pause) 후 재개(Resume) 시 G-code가 0번 라인부터 다시 시작되는 논리 결함
- **위치**: [`routes.py` L269](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L269), [`cnc_app.py` L940](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L940)
- **상세 원인**:
  - 일시정지(`toggle_pause`) 시 [`cnc_app.py` L1073, L1190](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1073)에서는 현재 진행 중인 라인 번호를 `gcode_current_idx = idx` (또는 `sent_idx`)에 저장하고 함수를 종료합니다.
  - 그러나 일시정지 상태에서 다시 '가공 시작' 버튼을 누르면 [`handle_start_gcode()` L269](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L269)에서 `controllers.gcode_current_idx = 0 # 반드시 0으로!`를 실행하고, [`send_gcode()` L940](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L940)에서도 또다시 `gcode_current_idx = 0`으로 초기화합니다.
- **영향**: 일시정지 후 재개하면 중단된 지점부터 이어서 가공하지 못하고 **무조건 처음(0번 인덱스)부터 다시 G-code를 전송**하며, 일시정지 시 꺼졌던 진동 탐지(`start_vibration_detection`)도 재개 분기(L990 `else:`)에서 다시 켜지지 않습니다.

#### ⑨ 시리얼 포트(`ser`) 멀티스레드 동기화 락(`Lock`) 부재 및 가공 중 버퍼 초기화 충돌
- **위치**: [`cnc_app.py` L698(`check_alarm`), L897(`send_gcode`), L1288(`check_connection`)](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L698), [`routes.py` L613(`handle_check_alarm_status`)](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L613)
- **상세 원인**:
  - 공유 객체인 `ser`(`serial.Serial`)에 대해 `threading.Lock()`이 전혀 적용되어 있지 않습니다.
  - 대기 상태(`state != "running"`)에서는 `check_alarm`(3초 주기)과 `check_connection`(5초 주기)이 동시에 `ser.reset_input_buffer()`와 `ser.write(b"?\n")`, `ser.readline()`을 호출하며 서로의 응답을 가로챕니다.
  - 심지어 [`routes.py` L613의 `handle_check_alarm_status()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L613)는 **현재 가공 중(`state == "running"`)인지 확인하는 가드 조건이 없습니다.** 만약 클라이언트에서 이 이벤트가 호출되면 가공 도중 `controllers.ser.reset_input_buffer()`가 실행되어 `send_gcode()`가 기다리던 `ok` 응답을 날려버립니다.

#### ⑩ `send_gcode()`의 응답 대기 루프 타임아웃 변수 위치 버그 및 모달(Modal) 좌표 파싱 누락
- **위치**: [`cnc_app.py` L1106-L1173](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1106-L1173)
- **상세 원인**:
  1. **재시도 루프 타임아웃 미갱신**:
     ```python
     start_resp_time = time.time()          # <-- for 루프 바깥에 위치!
     resp_timeout = 5.0
     for attempt in range(3):
         while time.time() - start_resp_time < resp_timeout:
             ...
     ```
     1차 시도에서 5초 타임아웃이 경과하면, `attempt = 1`과 `attempt = 2`에서는 `while` 조건이 처음부터 `False`가 되어 **응답을 전혀 기다리지 않고 0.1초 간격으로 동일 G-code 라인을 GRBL에 2번 연속 중복 전송**합니다.
  2. **에러 발생 시에도 정상 이동한 것처럼 좌표 보간 수행**:
     GRBL이 `error:`를 반환했을 때 `pending_lines.insert(0, ...)`로 재삽입 후 `break`로 루프를 빠져나오는데, `continue`하지 않고 L1140 이하의 좌표 파싱 및 보간 루프를 그대로 실행하여 물리적 위치와 UI 좌표가 어긋납니다.
  3. **모달(Modal) 이동 명령 및 `G2`/`G3` 원호 보간 파싱 누락**:
     [`cnc_app.py` L1172](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1172)에서 `is_move = sent_line.strip().startswith(('G0', 'G1', 'G00', 'G01'))`로만 이동 명령을 판단하고, `else:` 분기(L1208)에서는 `cur_x, cur_y, cur_z`를 갱신하지 않습니다.
     그러나 프로젝트 내 샘플인 [`O0527.txt` L9](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/O0527.txt#L9)(`X30 F70`)와 [`O1001.nc` L16-L321](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/O1001.nc#L16-L321)(약 300여 줄의 `X... Y... Z...` 모달 좌표 및 `G3` 원호 보간)은 `G0`/`G1`로 시작하지 않으므로 **UI 그래프에 경로가 전혀 그려지지 않고 `(0, 0, 0)`에 머무르는 심각한 시각화 결함**이 발생합니다.

---

### 2.3 [중요] 알고리즘/수식 논리 결함 및 프론트엔드/리소스 관리 결함

#### ⑪ `optimizer.py`의 목적 함수(`time_proxy`) 회전수 상쇄 및 LOF 부호/임계값 모순
- **위치**: [`optimizer.py` L19, L62-L72, L81-L83, L101-L135](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L62-L83)
- **상세 원인**:
  1. **`time_proxy` 수식 내 스핀들 회전수(`S`) 완전 약분**:
     ```python
     def time_proxy(self, F, S):
         fz = F / S
         return 1.0 / (fz * S)  # (F / S) * S == F 이므로 결과는 항상 1.0 / F
     ```
     특허 명세서 및 발표자료에서는 날당 이송(또는 회전당 이송 $f_s$)과 회전수 $S$의 곱을 최대화한다고 정의했으나, 코드에서는 이송 속도 `F`(mm/min)를 `S`로 나눈 뒤 다시 `S`를 곱해버려 **스핀들 회전수 `S`가 목적 함수에서 완전히 소거**됩니다.
  2. **LOF 이상 점수 부호 및 비교 방향 역전**:
     scikit-learn의 `lof.score_samples(v)`는 정상 데이터일수록 `-1.0`에 가깝고 이상치일수록 작은 음수(예: `-3.0`, `-10.0`)를 반환합니다.
     따라서 `lof_score(v)`가 반환하는 `-self.lof.score_samples(v)[0]`는 **값이 클수록 이상치(위험)**이고 **`1.0`에 가까울수록 정상(안전)**입니다.
     그런데 [`is_safe()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L68)는 `sc >= TAU` (`TAU = 1.0`)를 안전 조건으로 검사하고, [`better()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L90)는 `a["score"] > b["score"]`를 더 우수한 후보로 판정하여 **오히려 이상치 점수가 더 높은(더 위험한) 절삭 조건을 우선 선택**하는 논리적 역전이 발생합니다.
  3. **1·2단계 휴리스틱 조기 종료**:
     [`optimize_staged()` L115, L133](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L115)에서 `if best is not None: return best`가 `for k in range(1, 11):` 루프 안쪽에 들여쓰기되어 있어, $k=1$에서 안전 후보가 하나라도 나오면 $k=2\sim10$의 더 빠른 이송 속도(`F`) 후보들을 탐색하지 않고 즉시 반환합니다.

#### ⑫ `VibrationMonitor.__init__` 변수 이중 덮어쓰기 및 샘플링 레이트 괴리
- **위치**: [`vibration_monitor.py` L28-L86](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L28-L86)
- **상세 원인**:
  - `__init__` 전반부(L28-35)에서 설정한 `self.window_size = 200`, `self.sampling_rate = 1000`, `self.sensor_data_buffer = queue.Queue(maxsize=200)`를 후반부(L52-75)에서 `self.window_size = 100`, `self.fs = 5000`, `self.sensor_data_buffer = []`, `self.BUFFER_SIZE = 1000`으로 덮어씁니다.
  - 특히 아두이노 펌웨어([`MPU6050.ino` L7](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/MPU6050.ino#L7))는 `500ms` 간격(2Hz)으로 데이터를 전송하는데, `VibrationMonitor`는 5000Hz 샘플링을 가정하고 `BUFFER_SIZE = 1000`개가 모일 때까지 분석을 보류하므로 **첫 진동 분석이 실행되기까지 500초(8분 20초)가 소요**됩니다.

#### ⑬ 프론트엔드 `DOMContentLoaded` 3중 등록으로 인한 Chart.js 충돌 및 버튼 이벤트 2~4중 실행
- **위치**: [`static/script.js` L83, L2444, L2713](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L83)
- **상세 원인**:
  - `script.js` 내에 `document.addEventListener('DOMContentLoaded', ...)`가 3번 등록되어 있으며, L83과 L2713에서 모두 `initPage()`를 호출합니다.
  - `initPage()`가 2번 실행되면서:
    1. `initCharts()`가 2번 호출되어 동일 `<canvas>`에 Chart.js 인스턴스를 중복 생성하다가 `"Canvas is already in use"` 예외를 발생시킵니다.
    2. `setTimeout(() => initEventListeners(), 500)`가 2번 예약되어 **모든 제어/조그/포트 버튼에 클릭 리스너가 2번씩 바인딩**됩니다. (예: Jog `X+` 버튼 1회 클릭 시 `jog` 이벤트가 2회 연속 전송되어 2배 거리 이동)
    3. 허용 공차 확인 버튼(`#tolerance-confirm-btn`)은 L1663(2회), L1700(1회), L2786(1회) 총 **4회 중복 바인딩**되어 클릭 시 `alert` 창이 4번 연속 팝업됩니다.

#### ⑭ 실시간 보간 루프의 전체 경로 배열(`path_x, path_y, path_z`) 매 스텝 브로드캐스팅 및 차트 리렌더링 부하
- **위치**: [`cnc_app.py` L1202, L1273-L1284](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1273-L1284), [`static/script.js` L365-L418](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L365-L418)
- **상세 원인**:
  - `send_gcode()`에서 1mm 보간 스텝마다 호출되는 `update_ui()`가 누적 리스트 `x_vals, y_vals, z_vals` 전체를 매번 웹소켓으로 전송하고, 클라이언트는 애니메이션 비활성화(`update('none')`) 없이 `mainChart.update()`와 `zChart.update()`를 매 스텝 호출합니다.
  - 가공 시간이 길어져 포인트가 수천 개를 넘어가면 네트워크 패킷 크기와 DOM/Canvas 렌더링 부하가 $O(N^2)$으로 증가하여 브라우저가 멈춥니다.

#### ⑮ 센서 로그 이중 출력, CSV 파일 핸들 누수, CSS 문법 파괴 오타 및 경고음 에셋 누락
- **센서 로그 이중 출력**: [`cnc_app.py` L331, L350](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L331)에서 센서 데이터 1건당 `arduino_data`와 `log_update`를 동시에 보내고, [`script.js` L758, L917](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L758)에서 두 이벤트 모두 `addSensorLog()`를 호출하여 센서 로그창에 동일 데이터가 2줄씩 찍힙니다.
- **CSV 파일 디스크립터 누수**: [`start_csv_logging()`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L629)이 기존 열린 `csv_file`을 닫지 않고 새 파일을 오픈하며, `process_start_time`이 가공 종료 시 `None`으로 초기화되지 않습니다.
- **CSS 문법 오타**: [`static/style.css` L1388](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/style.css#L1388)에 `sensor-log_text`라는 잘못된 토큰이 단독 줄로 삽입되어 있어 직후의 `.sensor-log-buttons .log-toggle` 스타일 규칙이 무효화됩니다.
- **미구현 모달 및 죽은 코드**: [`index.html` L270-L311](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/templates/index.html#L270-L311)의 `#vibration-settings-modal`, `#vibration-alert-modal` 내 버튼들(`#train-model-btn`, `#reset-vibration-analyzer`, `#execute-recovery-btn`)은 `script.js`에 이벤트 리스너가 전혀 없으며, [`static/alert.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/alert.txt)에 명시된 `static/alert.mp3` 파일도 누락되어 있습니다.

---

## 3. 모듈 간 의존성 및 환경 설정(스크립트, 라이브러리, 문서)의 불일치 문제점

프로젝트 내 여러 모듈, 펌웨어, 가상환경, 문서가 서로 다른 시점에 수정되면서 심각한 규격 불일치가 발생하고 있습니다.

### 3.1 아두이노 센서 통신 속도(Baudrate), 샘플링 주기, JSON 스키마 4중 불일치

| 구분 | 대상 파일 | Baudrate | 샘플링 주기 | JSON 출력/파싱 스키마 | 호환성 상태 |
| :--- | :--- | :---: | :---: | :--- | :--- |
| **백엔드 수신부** | [`cnc_app.py` (`read_arduino_serial`)](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L49) | **115,200 bps** | 0.5ms 폴링 | `{"accel": {"x","y","z"}, "gyro": {"x","y","z"}}` 또는 `{"x","y","z"}` 또는 텍스트 `x:.. y:.. z:..` | 기준 (단, `accelX` 미지원) |
| **루트 펌웨어** | [`MPU6050.ino`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/MPU6050.ino#L7) | **115,200 bps** | **500 ms (2 Hz)** | `{"accel": {"x","y","z"}, "gyro": {"x","y","z"}}` | 스키마 호환 / **주기 너무 느림** |
| **폴더 펌웨어** | [`MPU6050/MPU6050.ino`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/MPU6050/MPU6050.ino#L7) | **115,200 bps** | **100 ms (10 Hz)** | `{"accel": {"x","y","z"}, "gyro": {"x","y","z"}}` | 스키마 호환 / **주기 느림** |
| **예제 펌웨어** | [`arduino_example/sensor_example.ino`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/arduino_example/sensor_example.ino#L19) | **9,600 bps** | 100 ms (10 Hz) | `{"accelX":..,"accelY":..,"accelZ":..,"vibration":..,"temp":..}` | **통신 불가** (Baudrate 및 JSON 키 불일치) |
| **파이썬 시뮬레이터** | [`arduino_sensor.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/arduino_sensor.py#L26) | **1,000,000 bps** (help: 9600) | **2.0 s (0.5 Hz)** (주석: 100ms) | `{"accelX":..,"accelY":..,"accelZ":..,"vibration":..,"temp":..,"timestamp":..}` | **호환 불가** (`cnc_app.py`에서 `0,0,0`으로 읽힘) |
| **문서 가이드** | [`README_ARDUINO.md`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/README_ARDUINO.md#L47) | **9,600 bps** | 100 ms | `{"accelX":..,"accelY":..,"accelZ":..,"vibration":..,"temp":..}` | 실제 `cnc_app.py`와 완전 불일치 |
| **특허/발표자료** | [`발표자료_프로젝트 LAB.pdf` (5, 7, 10p)](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8%20LAB%20%EB%AC%B8%EC%84%9C/%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8%20%EB%9E%A9/%EB%B0%9C%ED%91%9C%EC%9E%90%EB%A3%8C_%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8%20LAB.pdf) | Teensy 보드 + ACCEL 14 CLICK | **1,000 Hz** | `ProcessTime, AccX, AccY, AccZ, Label` | `vibration_monitor.py`(`fs=5000`)와 불일치 |

### 3.2 가상환경(`venv`) 및 Python 패키지 의존성 불일치
1. **깨진 `venv` 인터프리터 경로**:
   - `ProjectLab_System/venv/pyvenv.cfg`에 `home = C:\Users\ace\anaconda3`, `version = 3.13.5`가 하드코딩되어 있어 현재 PC(Python 3.11.9)에서는 `venv` 내 실행 파일(`python.exe`, `pip.exe`)이 작동하지 않습니다.
2. **`requirements.txt` 누락 및 미사용 패키지 혼재**:
   - [`watcher.py` L29](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/watcher.py#L29)에서 `socketio.Client()`가 WebSocket 트랜스포트로 연결하려면 `websocket-client` 패키지가 필요하나 [`requirements.txt`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/requirements.txt)와 `venv` 모두에 누락되어 있습니다.
   - 반면 `requirements.txt`에 포함된 `matplotlib>=3.4.0`과 `requests>=2.25.0`은 런타임 코드에서 실제로 사용되지 않습니다(`requests`는 import만 되어 있고 호출부는 주석 처리됨).
3. **`joblib` 직렬화 모델의 버전 종속성**:
   - `regr_model.joblib`과 `lof_model.joblib`은 `venv`의 `scikit-learn 1.7.2` / `joblib 1.5.2` / `numpy 2.3.5` 환경에서 덤프되었습니다. 다른 버전의 `scikit-learn`이나 `numpy 1.x` 환경에서 로드할 경우 `InconsistentVersionWarning` 또는 역직렬화(`BitGenerator`) 오류가 발생할 수 있습니다.

### 3.3 GRBL 실시간 오버라이드(Override) 명령 코드 불일치
- GRBL v1.1 표준 실시간 명령 바이트는 `0x99`(Spindle 100% Reset), `0x9A`(Spindle +10%), **`0x9B`(Spindle -10%)**, `0x9C`(Spindle +1%), `0x9D`(Spindle -1%)입니다.
- 그러나 [`script.js` L1268](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L1268)에서는 `-10%` 버튼 클릭 시 `{ cmd: "0x9C" }`를 전송하고, [`routes.py` L874-L876](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L874-L876)에서는 `elif cmd == '0x9C': ser.write(b'\x9B')`로 변환하여 전송하면서도 로그 메시지는 `"Spindle Override -10% 명령(0x9C) 전송"`이라고 잘못 출력합니다.

### 3.4 파일 입출력 인코딩(`utf-8` vs `cp949`) 및 CSV 포맷 불일치
1. **인코딩 불일치**:
   - [`rework.py` L140](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L140)은 `rework.nc`를 `encoding='utf-8'`로 저장하지만, [`cnc_app.py` L1420](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1420)(`NCFileWatcher`)은 `encoding='cp949'`로 읽고, [`routes.py` L1032](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L1032)(`handle_load_rework_file`)와 [`cnc_app.py` L643](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L643)(`start_csv_logging`)은 `encoding` 인자를 생략하여 OS 로케일(Windows CP949)에 의존합니다.
2. **`Vibration_detection.csv` 확장자 및 포맷 불일치**:
   - 파일명은 `.csv`이지만 [`vibration_monitor.py` L472-L475](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L472-L475)에서 기록하는 실제 내용은 CSV 헤더가 없는 일반 텍스트 3줄입니다:
     ```text
     Abnormal vibration detected at 2025-11-28 18:05:07
     Current position: X=0.000, Y=70.000, Z=0.840
     Spindle speed: 100%
     ```
3. **문서(`README.md`)와 실제 엔트리 포인트 불일치**:
   - [`README.md` L23, L41](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/README.md#L23)에는 실행 파일을 `python app.py`로 안내하고 있으나 실제 파일명은 [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py)입니다.

---

## 4. 개선 및 수정을 위한 우선순위 가이드

시스템의 안전한 구동과 정상적인 자율제어 파이프라인 완성을 위해 **P0(즉시 수정: 크래시 및 장비 파손 방지) $\rightarrow$ P1(단기 수정: 제어/알고리즘 정합성 확보) $\rightarrow$ P2(중기 개선: 성능 최적화 및 코드 정리)** 순서로 수정을 진행해야 합니다.

### 4.1 [P0] 크래시 및 공작기계 충돌 방지 (즉시 수정 필수)

1. **[P0-1] 페이지 로드 시 허용 공차 `10mm` 자동 전송 제거 및 기본값 안전화**:
   - [`static/script.js` L2821-L2825](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L2821-L2825)의 최상위 `fetch('/set_tolerance', ...)` 구문을 즉시 삭제합니다.
   - [`templates/index.html` L80](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/templates/index.html#L80)의 `value="10"`, `step="1"`을 `value="0.1"`, `step="0.05"`로 변경하고 [`script.js` L32](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L32)의 `toleranceValue = 0.0`으로 수정하여 10mm 급강하 충돌 사고를 원천 차단합니다.
2. **[P0-2] `rework.py` 안전 높이(`Z5`) 선행 이동 후 XY 접근하도록 순서 교정**:
   - [`rework.py` L91-L128](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/rework.py#L91-L128)에서 `G1 Z{original_z_value:.3f}`를 실행하기 전에 반드시 `G0 Z5` $\rightarrow$ `before_z_lines_cutting`(XY 시작 위치 이동)을 먼저 실행한 후 $Z$축 절삭 깊이로 진입하도록 순서를 수정합니다.
3. **[P0-3] `vibration_monitor.py` 및 `train_models.py`의 `C:\Users\ace\Desktop` 경로를 프로젝트 상대 경로로 변경**:
   - [`vibration_monitor.py` L298](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/vibration_monitor.py#L298)의 `base_path`를 `os.path.join(os.path.dirname(os.path.abspath(__file__)), "학습 데이터")`로 변경하여 프로젝트 내 CSV 파일들로 LOF 및 RMS 베이스라인이 정상 학습되도록 수정합니다.
4. **[P0-4] `routes.py`의 `create_rework_nc()` 인자 수정 및 `apply_override()` 구현**:
   - [`routes.py` L971](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L971)에서 `create_rework_nc()` 호출 시 키워드 인자(`current_n`, `current_F`, `optimal_n`, `optimal_F`, `x_pos`, `y_pos`, `z_pos`, `gcode_lines`, `tolerance_value`)를 명확히 전달하도록 수정합니다.
   - [`cnc_app.py`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py)에 `apply_override(target, value)` 함수를 구현하고, [`cnc_app.py` L1515](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1515)의 미구현 `set_fixed_spindle` 호출을 제거하거나 구현합니다.
5. **[P0-5] 깨진 `venv` 재생성**:
   - 기존 `ProjectLab_System/venv`를 제거하고 현재 시스템의 Python으로 `python -m venv venv` 및 `pip install -r requirements.txt`를 재수행합니다.

### 4.2 [P1] 제어 시퀀스, 스레드 동기화 및 AI 알고리즘 정합성 확보 (단기 수정)

1. **[P1-1] 이상 진동 감지 후 복구 및 재가공 시퀀스 일원화**:
   - 현재 `VibrationMonitor.handle_abnormal_vibration()`과 `watcher.py`가 동시에 동작하며 발생하는 레이스 컨디션을 방지하기 위해, `VibrationMonitor`에서 **① 긴급정지(`\x18`) $\rightarrow$ ② 소프트 리셋 및 `$X` 알람 해제 완료 대기 $\rightarrow$ ③ `state = "alarm_unlocked"` 전환 $\rightarrow$ ④ 마지막에 `rework.create_rework_nc()` 파일 생성** 순서로 실행하도록 순서를 변경합니다.
   - 또한 [`routes.py` L1055](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L1055)(`handle_load_rework_file`)에서 `send_gcode` 호출 전에 `controllers.state = "running"`으로 미리 바꾸지 않고 `"alarm_unlocked"` 상태를 유지한 채 `send_gcode`를 호출하여, 재가공 시에도 [`send_gcode()` L947](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L947)의 정상 시작 분기(진동 탐지 재활성화 포함)를 타도록 수정합니다.
2. **[P1-2] 일시정지(Pause) 및 재개(Resume) 로직 복구**:
   - [`routes.py` L269](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L269)와 [`cnc_app.py` L940](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L940)에서 무조건 `gcode_current_idx = 0`으로 초기화하는 코드를 제거하고, `state in ["idle", "alarm_unlocked", "emergency"]`(신규 시작)일 때만 `gcode_current_idx = 0`으로 설정하도록 변경합니다.
   - 재개 분기([`cnc_app.py` L990](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L990))에서도 `start_vibration_detection()`이 다시 활성화되도록 추가합니다.
3. **[P1-3] 시리얼 포트 `ser_lock` 도입 및 중복 상태 폴링 통합**:
   - `cnc_app.py`에 `ser_lock = threading.Lock()`을 생성하여 `send_gcode`, `check_alarm`, `check_connection`, `handle_jog`, `handle_check_alarm_status` 등 모든 시리얼 접근부를 `with ser_lock:`으로 보호합니다.
   - [`routes.py` L613](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/routes.py#L613)의 `handle_check_alarm_status` 상단에 `if controllers.state == "running": return ...` 가드를 추가하여 가공 중 버퍼 초기화를 방지하고, 역할이 겹치는 `check_alarm`과 `check_connection` 스레드를 단일 헬스체크 스레드로 통합합니다.
4. **[P1-4] `send_gcode()` 타임아웃 갱신 및 모달(Modal) 좌표 파싱 보완**:
   - [`cnc_app.py` L1106](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L1106)의 `start_resp_time = time.time()`을 `for attempt in range(3):` 루프 내부 첫 줄로 이동합니다.
   - `G0`/`G1`이 명시되지 않은 모달 좌표 라인(`X... Y... Z...`)과 `G2`/`G3` 명령에 대해서도 `x !== None or y !== None or z !== None`이면 이동으로 간주하여 `cur_x, cur_y, cur_z` 및 UI 경로가 정상 업데이트되도록 개선합니다.
5. **[P1-5] `optimizer.py`의 `time_proxy` 및 LOF 안전 판정 수식 교정**:
   - [`optimizer.py` L81](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/optimizer.py#L81)의 `time_proxy`에서 회전수 `S`가 약분되지 않도록 실제 목적 함수(예: 이송 속도 $F$와 회전수 $S$의 가중 결합 또는 `lof.score_samples`의 원래 부호 기준 안전성 최대화)를 명확히 정의하고, 1·2단계 휴리스틱에서 `k=1..10` 전체 후보를 평가한 뒤 최적해를 반환하도록 `if best is not None: return best`를 `for k` 루프 바깥으로 이동합니다.
6. **[P1-6] 아두이노 센서 샘플링 주기 및 JSON 파서 호환성 확보**:
   - [`cnc_app.py` L311](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L311)에서 `accelX, accelY, accelZ` 플랫 JSON 포맷도 인식하도록 파서를 확장하고, `MPU6050.ino`의 샘플링 주기와 `VibrationMonitor`의 `fs`/`BUFFER_SIZE`를 실제 데이터 수신 속도(예: 100Hz 이상 또는 버퍼 크기 축소)에 맞게 동기화합니다.

### 4.3 [P2] 프론트엔드 이벤트 정리, 성능 최적화 및 유지보수성 개선 (중기 개선)

1. **[P2-1] `script.js`의 3중 `DOMContentLoaded` 및 중복 이벤트 리스너 단일화**:
   - [`script.js` L2, L83, L2444, L2713](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/script.js#L83)에 흩어진 초기화 코드를 단 하나의 `DOMContentLoaded` 블록으로 통합합니다.
   - 미정의 함수 `requestPortList()`를 `fetchPorts()`로 교체하고, `isConnected` 변수를 파일 상단 전역 스코프로 이동합니다.
   - 중복 등록된 `#tolerance-confirm-btn`, `#unlock-btn`, `vibration_alert` 리스너를 각각 1곳으로 정리합니다.
2. **[P2-2] `update_ui()` 웹소켓 페이로드 경량화 및 센서 이중 로깅 제거**:
   - 매 보간 스텝마다 전체 배열(`path_x, path_y, path_z`)을 전송하는 대신 최신 좌표 `(x, y, z)`만 전송하여 클라이언트에서 `push`하도록 변경하고, Chart.js 업데이트 시 `mainChart.update('none')`을 사용하여 렌더링 부하를 줄입니다.
   - [`cnc_app.py` L350](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/cnc_app.py#L350)에서 센서 데이터마다 `log_update`를 중복 emit하는 코드를 제거하고 `arduino_data` 이벤트로 일원화합니다.
3. **[P2-3] 파일 인코딩 `utf-8` 통일, CSS 오타 수정 및 문서 최신화**:
   - 모든 `open()` 호출에 `encoding='utf-8'`을 명시합니다.
   - [`static/style.css` L1388](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/static/style.css#L1388)의 `sensor-log_text` 오타를 삭제합니다.
   - [`README.md`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/README.md)의 실행 명령어를 `python cnc_app.py`로 수정하고, [`README_ARDUINO.md`](file:///d:/Project_Lab/02_공모전_및_경진대회/ProjectLab%202026/ProjectLab_System/README_ARDUINO.md)의 보레이트(115200 bps) 및 JSON 데이터 규격을 최신 코드와 일치시킵니다.

