import numpy as np
import pandas as pd
import time
from flask_socketio import SocketIO
import sys
from sklearn.preprocessing import StandardScaler
from scipy.signal import stft
from sklearn.neighbors import LocalOutlierFactor
import tkinter as tk
from tkinter import messagebox
import threading
import queue
import logging
import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from optimizer import CuttingConditionOptimizer # CuttingConditionOptimizer 임포트
import rework # rework 모듈 임포트

# 진동 모니터 로거 설정
logger = logging.getLogger('vibration')

class VibrationMonitor:
    def __init__(self, socketio, controllers, gcode_lines=None, tolerance_value=0.0, initial_spindle_override=100):
        self.socketio = socketio
        self.controllers = controllers
        self.gcode_lines = gcode_lines
        self.tolerance_value = tolerance_value

        self.window_size = 200 # STFT 계산을 위한 데이터 윈도우 크기 (예: 200ms 데이터)
        self.nperseg = 128 # STFT 세그먼트 당 샘플 수
        self.noverlap = self.nperseg // 2 # STFT 오버랩 샘플 수
        self.sampling_rate = 1000 # 1000Hz (1ms 마다 데이터 수신 가정)

        self.sensor_data_buffer = queue.Queue(maxsize=self.window_size)
        self.last_abnormal_time = None
        self.MIN_ABNORMAL_DURATION = 1.0 # 1초 이상 이상 진동 지속 시 처리
        self.is_abnormal = False

        # 현재 디렉토리에 stft_dataset_3axis.csv가 있다고 가정 (train_models.py와 동일)
        # self.optimizer = CuttingConditionOptimizer(stft_dataset_path) # 더 이상 경로를 전달할 필요 없음
        self.optimizer = CuttingConditionOptimizer()

        # self.scaler1 = StandardScaler()
        # self.scaler2 = StandardScaler()
        # self.scaler3 = StandardScaler()
        # self._fit_scalers_from_dummy_data()

        # VibrationAnalyzer의 __init__ 로직을 직접 통합
        self.lof_model = None
        self.scaler1 = StandardScaler()
        self.scaler2 = StandardScaler()
        self.scaler3 = StandardScaler()
        self.window_size = 100  # 윈도우 크기
        self.step_size = 25     # 스텝 크기
        self.fs = 5000  # 샘플링 주파수
        # RMS/베이스라인 관련
        self.baseline_mean = None
        self.baseline_std = None
        self.rms_threshold_multiplier = 1.5  # baseline 대비 몇 배일 때 이상으로 판단할지
        self.abnormal_count = 0  # 연속된 이상 진동 카운트
        self.threshold = 0.5     # 이상 진동 판단 임계값 (초 단위)
        self.min_abnormal_ratio = 0.7  # 이상 진동으로 판단하기 위한 최소 비율
        self.is_trained = False # 모델 학습 여부

        # 입력 레벨 스무딩 / 이상치 제거 옵션
        self.enable_input_smoothing = True
        self.smooth_window = 3  # median window size (샘플 단위)
        self.hampel_k = 3       # Hampel window half-size
        self.hampel_threshold = 4.0  # Hampel 변별값 (MAD multiplier)
        # per-axis temporary buffers for smoothing
        from collections import deque as _dq
        self._sm_x = _dq(maxlen=self.smooth_window)
        self._sm_y = _dq(maxlen=self.smooth_window)
        self._sm_z = _dq(maxlen=self.smooth_window)
        self.sensor_data_buffer = []
        self.BUFFER_SIZE = 1000  # 버퍼 크기를 1000으로 감소 (약 0.2초 분량의 데이터)
        self.abnormal_vibration_start_time = None
        self.normal_vibration_start_time = None  # 정상 진동 시작 시간
        self.MIN_ABNORMAL_DURATION = 1.0  # 이상 진동 최소 지속 시간 (초)
        self.MIN_NORMAL_DURATION = 1.0    # 정상 진동 최소 지속 시간 (초)
        self.last_spindle_adjustment_time = 0
        self.MIN_SPINDLE_ADJUSTMENT_INTERVAL = 2.0
        self.spindle_override = initial_spindle_override
        self.MAX_SPINDLE_OVERRIDE = 200
        
        self.gcode_lines = gcode_lines if gcode_lines is not None else []
        self.tolerance_value = tolerance_value
        self.is_running = False
        self.last_analysis_log_time = 0 # 분석 로그 마지막 출력 시간
        self.log_display_interval = 3.0 # 로그 출력 간격 (3초)
        
        self.initialize_analyzer() # initialize_analyzer는 이제 train_model을 직접 호출함

    def compute_rms(self, arr):
        """배열(1D) 또는 (N,) 형식의 신호에 대해 RMS를 계산합니다."""
        try:
            a = np.asarray(arr)
            return np.sqrt(np.mean(a.astype(float)**2))
        except Exception:
            return 0.0

    def calibrate_baseline_from_training(self, df):
        """학습에 사용한 정상 데이터프레임(df: AccX,AccY,AccZ)을 사용해 RMS baseline을 만듭니다."""
        try:
            # 각 샘플 별 RMS 계산
            rms_vals = np.sqrt((df[:,0].astype(float)**2 + df[:,1].astype(float)**2 + df[:,2].astype(float)**2)/3.0)
            logger.info(f"[Baseline Calc] RMS values (first 5 samples): {rms_vals[:5].tolist()}")
            self.baseline_mean = float(np.mean(rms_vals))
            self.baseline_std = float(np.std(rms_vals))
            logger.info(f"[Baseline Calc] RMS baseline 설정: mean={self.baseline_mean:.6f}, std={self.baseline_std:.6f}")
            logger.info(f"[Baseline Calc] RMS threshold multiplier: {self.rms_threshold_multiplier}")
            logger.info(f"[Baseline Calc] Abnormal threshold: {self.baseline_mean + self.rms_threshold_multiplier * self.baseline_std:.6f}")
        except Exception as e:
            logger.warning(f"baseline 계산 실패: {e}")

    def sliding_window(self, data):
        """슬라이딩 윈도우로 데이터 분할"""
        if len(data) < self.window_size:
            return np.array([data])  # 데이터가 윈도우 크기보다 작으면 전체 데이터를 하나의 세그먼트로
        return np.array([
            data[i:i + self.window_size] 
            for i in range(0, len(data) - self.window_size + 1, self.step_size)
        ])

    def extract_features(self, segment):
        """STFT 기반 특징 추출"""
        try:
            f, t, Zxx = stft(segment, fs=self.fs, nperseg=64)  # nperseg 감소
            Zxx_mag = np.abs(Zxx)
            return [
                np.mean(Zxx_mag),
                np.std(Zxx_mag),
                np.max(Zxx_mag),
                np.sum(Zxx_mag**2) # energy (sum of squares)만 반환
            ]
        except Exception as e:
            logger.error(f"특징 추출 중 오류: {e}")
            return [0] * 4  # 오류 발생 시 0으로 채움 (4개 특징)

    def train_model(self, sensor_data):
        """모델 학습"""
        try:
            # 데이터 전처리
            df = pd.DataFrame(sensor_data, columns=['AccX', 'AccY', 'AccZ'])
            
            # 0값 처리 개선
            df = df.replace([np.inf, -np.inf], np.nan)
            df = df.ffill()  # 앞의 값으로 채우기 (FutureWarning 해결)
            df = df.fillna(0)  # 남은 NaN값은 0으로 채움
            
            # 정규화
            df["sensor1_norm"] = self.scaler1.fit_transform(df["AccX"].values.reshape(-1, 1)).flatten()
            df["sensor2_norm"] = self.scaler2.fit_transform(df["AccY"].values.reshape(-1, 1)).flatten()
            df["sensor3_norm"] = self.scaler3.fit_transform(df["AccZ"].values.reshape(-1, 1)).flatten()
            
            # 슬라이딩 윈도우 적용
            segments_s1 = self.sliding_window(df["sensor1_norm"].values)
            segments_s2 = self.sliding_window(df["sensor2_norm"].values)
            segments_s3 = self.sliding_window(df["sensor3_norm"].values)
            
            # 특징 추출
            features = np.array([
                self.extract_features(s1) + self.extract_features(s2) + self.extract_features(s3)
                for s1, s2, s3 in zip(segments_s1, segments_s2, segments_s3)
            ])
            
            # LOF 모델 학습 (contamination 감소)
            self.lof_model = LocalOutlierFactor(n_neighbors=20, contamination=0.1, novelty=True)
            self.lof_model.fit(features)
            
            logger.info("모델 학습 완료")
            self.is_trained = True
            # 학습에 사용한 원본 데이터를 이용해 baseline RMS 계산
            try:
                arr = np.array(sensor_data)
                if arr.ndim == 2 and arr.shape[1] >= 3:
                    self.calibrate_baseline_from_training(arr[:, :3])
            except Exception as e:
                logger.warning(f"학습 후 baseline 계산 중 오류: {e}")
            return True
            
        except Exception as e:
            logger.error(f"모델 학습 중 오류 발생: {str(e)}")
            return False

    def analyze_realtime_data(self, data):
        """실시간 데이터 분석"""
        try:
            if not self.is_trained:
                logger.warning("모델이 학습되지 않았습니다")
                return False

            # 데이터 전처리
            df = pd.DataFrame(data, columns=['AccX', 'AccY', 'AccZ'])
            df = df.replace([np.inf, -np.inf], np.nan)
            df = df.ffill()
            df = df.fillna(0)
            
            # 정규화
            df["sensor1_norm"] = self.scaler1.transform(df["AccX"].values.reshape(-1, 1)).flatten()
            df["sensor2_norm"] = self.scaler2.transform(df["AccY"].values.reshape(-1, 1)).flatten()
            df["sensor3_norm"] = self.scaler3.transform(df["AccZ"].values.reshape(-1, 1)).flatten()
            
            # 슬라이딩 윈도우 적용
            segments_s1 = self.sliding_window(df["sensor1_norm"].values)
            segments_s2 = self.sliding_window(df["sensor2_norm"].values)
            segments_s3 = self.sliding_window(df["sensor3_norm"].values)

            if len(segments_s1) == 0:
                return False

            # 특징 추출
            features = np.array([
                self.extract_features(s1) + self.extract_features(s2) + self.extract_features(s3)
                for s1, s2, s3 in zip(segments_s1, segments_s2, segments_s3)
            ])
            
            # LOF 모델을 사용하여 이상치 예측
            predictions = self.lof_model.predict(features)
            is_abnormal = np.any(predictions == -1)

            # RMS 기반 보강 검사: 각 윈도우에 대해 AccX/Y/Z의 RMS 계산
            rms_list = []
            try:
                total_len = len(df)
                # 윈도우 시작 인덱스와 수는 sliding_window과 동일한 방식으로 계산
                starts = list(range(0, max(0, total_len - self.window_size + 1), self.step_size))
                for s in starts:
                    seg = df.iloc[s:s + self.window_size]
                    if len(seg) == 0:
                        continue
                    accx = seg['AccX'].values.astype(float)
                    accy = seg['AccY'].values.astype(float)
                    accz = seg['AccZ'].values.astype(float)
                    # 축별 평균 제곱값의 평균으로 RMS 계산
                    rms_val = np.sqrt(np.mean((accx**2 + accy**2 + accz**2) / 3.0))
                    rms_list.append(rms_val)
            except Exception as e:
                logger.debug(f"RMS 계산 중 오류: {e}")

            abnormal_rms_ratio = 0.0
            if rms_list and self.baseline_mean is not None and self.baseline_std is not None:
                rms_threshold = self.baseline_mean + self.rms_threshold_multiplier * self.baseline_std
                logger.info(f"[Realtime RMS] Calculated RMS list (first 5): {rms_list[:5]}")
                logger.info(f"[Realtime RMS] Baseline Mean: {self.baseline_mean:.6f}, Std: {self.baseline_std:.6f}, Threshold: {rms_threshold:.6f}")
                abnormal_rms_count = sum(1 for r in rms_list if r > rms_threshold)
                abnormal_rms_ratio = abnormal_rms_count / len(rms_list)
                logger.info(f"[Realtime RMS] Abnormal RMS Count: {abnormal_rms_count}, Total RMS Segments: {len(rms_list)}, Abnormal Ratio: {abnormal_rms_ratio:.2f}")

            # 최종 이상 판단: LOF 결과와 RMS 비율을 조합
            final_is_abnormal = False
            if is_abnormal and (abnormal_rms_ratio >= 0.2 or self.baseline_mean is None):
                final_is_abnormal = True
            elif abnormal_rms_ratio >= self.min_abnormal_ratio:
                final_is_abnormal = True
            else:
                final_is_abnormal = False
            
            # 결과를 터미널에 출력 (즉시 출력)
            current_time_str = time.strftime("%H:%M:%S")
            abnormal_ratio = np.sum(predictions == -1) / len(predictions) * 100
            
            # 로그 및 클라이언트 전송 (최종 판단 사용)
            if final_is_abnormal:
                logger.info(f"⚠️ [{current_time_str}] 이상 진동 감지! (LOF_ratio={abnormal_ratio:.1f}%, RMS_ratio={abnormal_rms_ratio:.2f})")
                logger.info(f"📊 현재 스핀들 속도: {self.spindle_override}%")
                self.socketio.emit('vibration_status', {
                    'status': 'abnormal',
                    'lof_ratio': abnormal_ratio,
                    'rms_ratio': abnormal_rms_ratio,
                    'spindle_speed': self.spindle_override,
                    'time': current_time_str
                })
            else:
                logger.info(f"✅ [{current_time_str}] 정상 진동 (LOF_ratio={abnormal_ratio:.1f}%, RMS_ratio={abnormal_rms_ratio:.2f})")
                logger.info(f"📊 현재 스핀들 속도: {self.spindle_override}%")
                self.socketio.emit('vibration_status', {
                    'status': 'normal',
                    'lof_ratio': abnormal_ratio,
                    'rms_ratio': abnormal_rms_ratio,
                    'spindle_speed': self.spindle_override,
                    'time': current_time_str
                })

            return final_is_abnormal

        except Exception as e:
            logger.error(f"데이터 분석 중 오류 발생: {str(e)}")
            return False

    def initialize_analyzer(self):
        """진동 분석기 초기화 및 모델 학습"""
        try:
            normal_data_files = [
                "master_dataset.csv"
            ]
            
            all_normal_data = []
            base_path = os.path.join(BASE_DIR, "data", "학습 데이터")
            
            for file_name in normal_data_files:
                file_path = os.path.join(base_path, file_name)
                try:
                    logger.info(f"{file_path} 파일 로드 중...")
                    df = pd.read_csv(file_path)
                    # 필요한 컬럼만 선택 (AccX, AccY, AccZ)
                    df = df[['AccX', 'AccY', 'AccZ']].values
                    all_normal_data.extend(df)
                    logger.info(f"{file_path} 파일 로드 완료 (데이터 수: {len(df)})")
                except FileNotFoundError:
                    logger.warning(f"경고: {file_path} 파일을 찾을 수 없습니다.")
                    continue
                except KeyError as e:
                    logger.warning(f"경고: {file_path} 파일에서 필요한 컬럼을 찾을 수 없습니다. 오류: {e}")
                    continue
            
            if not all_normal_data:
                raise FileNotFoundError("정상 진동 데이터 파일을 찾을 수 없습니다.")
            
            normal_data = np.array(all_normal_data)
            logger.info(f"총 {len(normal_data)}개의 정상 데이터로 학습 시작")
            logger.debug("initialize_analyzer: train_model 호출 직전")
            self.train_model(normal_data) # VibrationMonitor 내부의 train_model 호출
            logger.debug("initialize_analyzer: train_model 호출 직후 및 모델 학습 완료")
            logger.info("진동 분석기 초기화 및 모델 학습 완료")
            return True
        except Exception as e:
            logger.error(f"진동 분석기 초기화 실패: {e}")
            return False

    def process_sensor_data(self, sensor_data):
        """센서 데이터 처리"""
        try:
            if not self.is_running:
                return

            # 센서 데이터를 버퍼에 추가
            # 입력 레벨에서 이상치 제거 / 스무딩을 수행한 뒤 버퍼에 추가
            if self.enable_input_smoothing:
                filtered = []
                for sample in sensor_data:
                    try:
                        x, y, z = float(sample[0]), float(sample[1]), float(sample[2])
                    except Exception:
                        # skip malformed sample
                        continue

                    # append to running deques
                    self._sm_x.append(x)
                    self._sm_y.append(y)
                    self._sm_z.append(z)

                    # median smoothing
                    med_x = float(np.median(list(self._sm_x)))
                    med_y = float(np.median(list(self._sm_y)))
                    med_z = float(np.median(list(self._sm_z)))

                    # Hampel-like outlier rejection: if current sample deviates from median by > threshold*MAD, use median
                    # compute MAD
                    def mad(arr):
                        a = np.asarray(arr)
                        return np.median(np.abs(a - np.median(a))) if a.size else 0.0

                    mx = mad(self._sm_x)
                    my = mad(self._sm_y)
                    mz = mad(self._sm_z)

                    def maybe_replace(value, med, mval):
                        if mval <= 0:
                            return value
                        if abs(value - med) > self.hampel_threshold * mval:
                            return med
                        return value

                    sx = maybe_replace(x, med_x, mx)
                    sy = maybe_replace(y, med_y, my)
                    sz = maybe_replace(z, med_z, mz)

                    filtered.append([sx, sy, sz])

                # extend buffer with filtered samples
                self.sensor_data_buffer.extend(filtered)
            else:
                self.sensor_data_buffer.extend(sensor_data)
            
            # 버퍼가 충분히 쌓이면 분석
            if len(self.sensor_data_buffer) >= self.BUFFER_SIZE:
                # 진동 분석
                is_abnormal = self.analyze_realtime_data(self.sensor_data_buffer)
                
                # 분석 결과에 따른 처리
                if is_abnormal:
                    if self.abnormal_vibration_start_time is None:
                        self.abnormal_vibration_start_time = time.time()
                    
                    current_abnormal_duration = time.time() - self.abnormal_vibration_start_time

                    if current_abnormal_duration >= self.MIN_ABNORMAL_DURATION: # 최소 지속 시간 이상이면 처리
                        # 이상 진동 감지 즉시 알림은 1회만 (팝업은 handle_abnormal_vibration에서)
                        if self.abnormal_count == 0: # 첫 감지 시에만 로그
                            self.socketio.emit('vibration_alert', {
                                'show': True,
                                'override': self.spindle_override,
                                'message': f"⚠️ 이상 진동 감지! 스핀들 속도: {self.spindle_override}%"
                            })
                            logger.info(f"⚠️ 이상 진동 감지됨 - 모니터링 시작 (지속 시간: {current_abnormal_duration:.2f}초)")
                        
                        self.abnormal_count += 1
                        self.handle_abnormal_vibration()
                        self.abnormal_vibration_start_time = None # 처리 후 초기화
                        self.abnormal_count = 0 # 처리 후 카운트 초기화
                else:
                    self.abnormal_count = 0
                    if self.abnormal_vibration_start_time is not None:
                        logger.info(f"✅ 이상 진동 해소됨 (지속 시간: {time.time() - self.abnormal_vibration_start_time:.2f}초)")
                        self.abnormal_vibration_start_time = None
                    
                    # 정상 진동 지속 시간 체크
                    if self.normal_vibration_start_time is None:
                        self.normal_vibration_start_time = time.time()
                    
                    # 현재까지의 정상 진동 지속 시간 계산
                    normal_duration = time.time() - self.normal_vibration_start_time
                    
                    # 정상 진동이 최소 지속 시간 이상 유지되었는지 확인
                    if normal_duration >= self.MIN_NORMAL_DURATION:
                        logger.info(f"✅ {self.MIN_NORMAL_DURATION}초 이상 지속된 정상 진동 확인")
                        self.normal_vibration_start_time = None
                
                # 버퍼 초기화 (최근 데이터 일부 유지)
                self.sensor_data_buffer = self.sensor_data_buffer[-self.BUFFER_SIZE//2:]
                
        except Exception as e:
            logger.error(f"센서 데이터 처리 중 오류 발생: {str(e)}")
            self.sensor_data_buffer = []  # 오류 발생 시 버퍼 초기화

    def handle_abnormal_vibration(self):
        """이상 진동 발생 시 처리"""
        logger.info("이상 진동 감지됨")
        
        # 진동 알림 표시
        self.socketio.emit('vibration_alert', {
            'show': True,
            'override': self.spindle_override,
            'message': f"⚠️ 이상 진동 감지! 스핀들 속도: {self.spindle_override}%"
        })
        
        # 이상 진동 감지 시 직접 컨트롤러를 통해 일시정지 실행
        try:
            logger.info(f"[Vibration Handle] 컨트롤러 상태 확인: ser={self.controllers.ser is not None}, is_open={self.controllers.ser.is_open if self.controllers.ser else 'N/A'}, state={self.controllers.state}")
            if self.controllers.ser and self.controllers.ser.is_open and self.controllers.state == "running":
                logger.info("[Vibration Handle] 긴급 정지 명령 전송: GRBL 소프트 리셋 (Ctrl-X)")
                self.controllers.ser.write(b"\x18") # Ctrl-X: GRBL 소프트 리셋 (긴급 정지)
                self.controllers.ser.flush()
                time.sleep(0.1)

                logger.info("[Vibration Handle] 컨트롤러 상태 'emergency'로 변경")
                self.controllers.pause = False # 긴급 정지는 일시정지가 아님
                self.controllers.state = "emergency"
                
                # 진동 탐지 중지
                if self.controllers.vibration_detection_active:
                    self.controllers.stop_vibration_detection()
                    self.socketio.emit('log_update', {"message": "진동 탐지가 중지되었습니다.", "color": "blue"})

                # 일시정지 메시지 표시
                reason = "이상 진동 감지로 인한 일시정지"
                self.socketio.emit('log_update', {"message": reason, "color": "orange"})
                self.socketio.emit('status_update', {"text": "일시정지", "color": "orange"})
                
                # Vibration_detection.csv 파일 생성
                try:
                    with open(os.path.join(BASE_DIR, 'data', 'Vibration_detection.csv'), 'w') as f:
                        f.write(f"Abnormal vibration detected at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                        f.write(f"Current position: X={self.controllers.x_vals[-1]:.3f}, Y={self.controllers.y_vals[-1]:.3f}, Z={self.controllers.z_vals[-1]:.3f}\n")
                        f.write(f"Spindle speed: {self.spindle_override}%\n")
                    logger.info("Vibration_detection.csv 파일이 생성되었습니다.")
                except Exception as e:
                    logger.error(f"Vibration_detection.csv 파일 생성 중 오류: {str(e)}")
                
                # 최적 절삭 조건 계산 및 rework.nc 생성
                if self.optimizer and self.controllers.current_spindle is not None and self.controllers.current_feedrate is not None:
                    logger.info("최적 절삭 조건 계산 시작...")
                    current_n = self.controllers.current_spindle # 현재 스핀들 속도
                    current_F = self.controllers.current_feedrate # 현재 이송 속도

                    # 이상 진동 발생 시의 실시간 STFT 특징값 추출 (최근 버퍼 데이터 사용)
                    if len(self.sensor_data_buffer) >= self.window_size:
                        # 버퍼의 마지막 윈도우를 사용하여 특징 추출
                        segment = self.sensor_data_buffer[-self.window_size:]
                        # 데이터프레임으로 변환 후 정규화
                        df_segment = pd.DataFrame(segment, columns=['AccX', 'AccY', 'AccZ'])
                        df_segment = df_segment.replace([np.inf, -np.inf], np.nan).ffill().fillna(0)
                        
                        s1_norm = self.scaler1.transform(df_segment["AccX"].values.reshape(-1, 1)).flatten()
                        s2_norm = self.scaler2.transform(df_segment["AccY"].values.reshape(-1, 1)).flatten()
                        s3_norm = self.scaler3.transform(df_segment["AccZ"].values.reshape(-1, 1)).flatten()

                        # 각 축별 특징 추출 후 결합
                        realtime_stft_features = (
                            self.extract_features(s1_norm) +
                            self.extract_features(s2_norm) +
                            self.extract_features(s3_norm)
                        )
                        realtime_stft_features = np.array(realtime_stft_features)

                        # 잔차 업데이트
                        self.optimizer.update_residual(realtime_stft_features, current_n, current_F)

                        c0 = {"n": current_n, "F": current_F}
                        res = self.optimizer.optimize_staged(c0)

                        if res is None:
                            logger.warning("⚠️ 안전한 최적 절삭 조건을 찾을 수 없습니다.")
                            self.socketio.emit('log_update', {"message": "⚠️ 안전한 최적 절삭 조건을 찾을 수 없습니다.", "color": "red"})
                        else:
                            optimal_c = res["cand"]
                            optimal_n = optimal_c["n"]
                            optimal_F = optimal_c["F"]
                            logger.info(f"✅ 최적 절삭 조건 도출 완료: S={optimal_n:.0f} rpm, F={optimal_F:.1f} mm/min (LOF={res['score']:.3f}, Time Proxy≈{res['time']:.6f})")
                            self.socketio.emit('log_update', {"message": f"✅ 최적 절삭 조건 도출 완료: S={optimal_n:.0f} rpm, F={optimal_F:.1f} mm/min", "color": "green"})

                            # rework.nc 파일 생성 (기존 함수 재활용)
                            try:
                                rework.create_rework_nc(
                                    current_n,
                                    current_F,
                                    optimal_n,
                                    optimal_F,
                                    self.controllers.x_vals[-1],
                                    self.controllers.y_vals[-1],
                                    self.controllers.z_vals[-1],
                                    gcode_lines=self.controllers.gcode_lines, # 현재 가공 중인 G-code 라인 전달
                                    tolerance_value=self.controllers.tolerance_value # 공차 값 전달
                                )
                                logger.info("rework.nc 파일이 생성되었습니다.")
                                self.socketio.emit('log_update', {"message": "rework.nc 파일이 생성되었습니다.", "color": "blue"})

                                # GRBL 긴급 정지 해제 및 알람 해제 시도
                                if self.controllers.ser and self.controllers.ser.is_open:
                                    try:
                                        logger.info("[Vibration Handle] GRBL 소프트 리셋 (Ctrl-X) 명령 전송...")
                                        self.controllers.ser.write(b"\x18") # Ctrl-X: GRBL 소프트 리셋
                                        self.controllers.ser.flush()
                                        time.sleep(2.0) # 리셋 후 부팅 및 초기화 대기 (충분히 길게)
                                        self.socketio.emit('log_update', {"message": "GRBL 소프트 리셋 명령 전송 완료. 컨트롤러 초기화 대기 중...", "color": "blue"})

                                        logger.info("[Vibration Handle] GRBL 알람 해제 ($X) 명령 전송...")
                                        self.controllers.ser.write(b"$X\n") # $X: 알람 해제
                                        self.controllers.ser.flush()
                                        time.sleep(1.0) # 응답 대기
                                        self.socketio.emit('log_update', {"message": "GRBL 알람 해제 명령 전송 완료. 상태 확인 중...", "color": "blue"})

                                        # 컨트롤러 상태 업데이트 및 UI 반영
                                        self.controllers.state = "alarm_unlocked" # 알람 해제 상태로 변경
                                        self.socketio.emit('status_update', {"text": "알람 해제됨: 재가공 준비 완료", "color": "green"})
                                        logger.info("[Vibration Handle] GRBL 컨트롤러가 알람 해제 상태로 변경되었습니다.")

                                    except Exception as grbl_e:
                                        logger.error(f"GRBL 명령 전송 중 오류: {str(grbl_e)}", exc_info=True)
                                        self.socketio.emit('log_update', {"message": f"⚠️ GRBL 컨트롤러 복구 실패: {str(grbl_e)}", "color": "red"})
                                else:
                                    logger.warning("GRBL 시리얼 포트가 연결되지 않아 컨트롤러를 복구할 수 없습니다.")
                                    self.socketio.emit('log_update', {"message": "⚠️ GRBL 시리얼 포트 연결 없음: 컨트롤러 복구 실패", "color": "red"})

                            except Exception as e:
                                logger.error(f"rework.nc 파일 생성 중 오류: {str(e)}", exc_info=True)
                                self.socketio.emit('log_update', {"message": f"⚠️ rework.nc 파일 생성 실패: {str(e)}", "color": "red"})
                    else:
                        logger.warning("STFT 특징값 추출을 위한 충분한 센서 데이터가 없습니다.")
                        self.socketio.emit('log_update', {"message": "STFT 특징값 추출을 위한 충분한 센서 데이터가 없습니다.", "color": "orange"})
                else:
                    logger.warning("Optimizer가 초기화되지 않았거나 현재 스핀들/이송 속도 정보를 가져올 수 없습니다.")
                    self.socketio.emit('log_update', {"message": "Optimizer가 초기화되지 않았거나 현재 스핀들/이송 속도 정보를 가져올 수 없습니다.", "color": "red"})
                
                logger.info("이상 진동으로 인한 일시정지 처리 완료")
        except Exception as e:
            logger.error(f"이상 진동 처리 중 오류 발생: {str(e)}")

def show_alert():
    """경고창 표시"""
    root = tk.Tk()
    root.withdraw()  # 기본 창 숨기기
    messagebox.showwarning("경고", "이상 진동이 감지되었습니다!")
    root.destroy()

def monitor_vibration(sensor_queue, stop_event, tolerance_value):
    """진동 모니터링"""
    try:
        # VibrationMonitor 인스턴스는 cnc_app.py에서 전역으로 관리되어 전달되거나,
        # 여기서 필요하다면 다시 초기화하여 사용해야 합니다.
        # 이 함수는 주로 실시간 데이터를 처리하는 역할을 하므로, 모델 학습은 VibrationMonitor 클래스에서 이루어져야 합니다.
        
        # cnc_app.py에서 vibration_monitor 인스턴스를 전역으로 가져오는 것이 더 효율적입니다.
        # 임시로 여기서 VibrationMonitor를 생성하고 학습을 시도합니다.
        monitor = VibrationMonitor(None, None) # socketio는 None으로 설정
        
        # 주의: 이 코드는 monitor_vibration 함수가 VibrationMonitor 클래스와 별개로 작동할 때만 유효합니다.
        # 만약 cnc_app.py에서 VibrationMonitor 인스턴스를 이미 생성하고 학습했다면 이 부분은 중복입니다.
        # 여기서는 테스트 목적으로 최소한의 학습을 시뮬레이션합니다.
        # 실제 구현에서는 이미 학습된 monitor 인스턴스를 사용해야 합니다.
        # TODO: 실제 센서 데이터를 사용하여 모델 학습
        # 이 부분은 initialize_analyzer()에서 처리되므로 여기서는 제거합니다.
        
        logger.info("진동 모니터링을 시작합니다.")
        
        while not stop_event.is_set():
            try:
                # 센서 데이터 가져오기
                sensor_data = sensor_queue.get(timeout=1.0)
                
                # 데이터 분석
                # VibrationMonitor 인스턴스를 통해 분석 수행
                is_abnormal = monitor.analyze_realtime_data(sensor_data)
                
                if is_abnormal:
                    # 현재 Z축 위치 확인
                    # 이 부분은 cnc_app.py의 process_sensor_data에서 처리되므로 여기서는 제거합니다.
                    pass # 현재는 아무것도 하지 않음
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"데이터 처리 중 오류 발생: {str(e)}")
                continue
            
    except Exception as e:
        logger.error(f"모니터링 중 오류 발생: {str(e)}")
    finally:
        logger.info("진동 모니터링을 종료합니다.") 