import serial
import serial.tools.list_ports
import threading
import time
import os
import json
import sys
import csv
import datetime
import re
from collections import deque
from flask import Flask
from flask_socketio import SocketIO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if os.path.join(BASE_DIR, 'src') not in sys.path:
    sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

from routes import init_routes
from gcode_utils import ModalState, code_numbers, parse_words, strip_comments
from vibration_monitor import VibrationMonitor
import logging
from watcher import start_rework_watcher  # watcher.py의 rework.nc 감시 함수

# Flask 앱 설정
app = Flask(
    __name__,
    template_folder=os.path.join(BASE_DIR, 'templates'),
    static_folder=os.path.join(BASE_DIR, 'static')
)
socketio = SocketIO(app, cors_allowed_origins="*")

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,  # DEBUG에서 INFO로 변경
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# 진동 모니터 로거 설정
vibration_logger = logging.getLogger('vibration')
vibration_logger.setLevel(logging.INFO)  # 진동 모니터의 로깅 레벨을 INFO로 설정

# 전역 변수 선언
controllers = sys.modules[__name__]  # 현재 모듈을 controllers로 사용
tolerance_value = 0.0

# 아두이노 센서 데이터 관련 변수
ARDUINO_SERIAL_PORT = None  # 기본 포트 없음 - 자동 감지로 변경
ARDUINO_BAUD_RATE = 115200
arduino_serial = None
latest_arduino_data = "센서 데이터 없음"
arduino_connected = False

# 가공 상태 관련 변수
state = "idle"  # idle, running, paused, emergency, alarm_unlocked, alarm
pause = False
progress = 0

# 위치 및 경로 데이터 (가공 중에는 GRBL 상태 보고의 실제 작업 좌표)
x_vals, y_vals, z_vals = [0], [0], [0]
ui_sent_len = 0  # 경로 점 중 UI로 이미 보낸 개수 (update_ui는 새로 추가된 점만 보냄)

# 가공 파라미터
current_spindle = None
current_feedrate = None
gcode_lines = []
rework_source_lines = []  # 재가공(rework.nc) 생성 기준이 되는 원본 NC 코드 (rework.nc 자체가 아님)

# 시리얼 연결
ser = None
# 시리얼 포트는 여러 스레드가 공유하므로 쓰기/읽기 트랜잭션은 반드시 이 잠금 안에서 수행합니다.
# 가공 중(job_active)에는 스트리밍 스레드만 응답을 읽고, 다른 스레드는 실시간 명령(?, !, ~, Ctrl-X, 오버라이드)만 씁니다.
ser_lock = threading.RLock()

# 가공 작업(스트리밍 스레드) 관리
job_lock = threading.Lock()
job_active = False   # send_gcode 스레드 동작 여부 (running/paused 모두 포함)
job_generation = 0   # 새 작업 시작/긴급정지 시 증가 → 이전 스트리밍 스레드 종료 신호
gcode_current_idx = 0
GRBL_RX_BUFFER_SIZE = 127     # GRBL 수신 버퍼(128바이트) 기준 문자 수 흐름 제어 한도
STATUS_POLL_INTERVAL = 0.2    # 가공 중 GRBL 상태(?) 조회 주기 (초)
SAFE_Z_HEIGHT = 5.0           # G28/G30 변환 시 사용하는 안전 높이 (mm, NC 코드의 "G0 Z5" 관례와 동일)
RAPID_FEEDRATE = 5000         # 급속 이송(G0) 속도 추정값 (mm/min, 예상 가공 시간 계산용)

# 이상 진동 복구 과정에서 생성된 rework.nc만 자동 가공하도록 허용 (수동 편집·외부 변경으로 기계가 움직이지 않게)
rework_armed_at = None
REWORK_ARM_TIMEOUT = 30.0  # 초

# CSV 로깅 관련 변수
csv_file = None
csv_writer = None
process_start_time = None
last_job_elapsed = 0

# 알람 상태 관련 변수
last_alarm_check = 0
last_alarm_message = ""
alarm_check_interval = 3.0  # 알람 확인 간격(초)
alarm_retry_count = 0
max_alarm_retries = 3

# Override 관련 변수
spindle_override = 100  # 기본값 100%
feed_override = 100     # 기본값 100%
base_spindle = None     # M3/M4의 기본 S값
base_feedrate = None    # 최초 F값
OVERRIDE_RANGE = (10, 200)  # GRBL 오버라이드 허용 범위 (%)
FEED_OVERRIDE_CMDS = {'reset': b'\x90', '+10': b'\x91', '-10': b'\x92', '+1': b'\x93', '-1': b'\x94'}
SPINDLE_OVERRIDE_CMDS = {'reset': b'\x99', '+10': b'\x9A', '-10': b'\x9B', '+1': b'\x9C', '-1': b'\x9D'}

# 진동 분석기 관련 변수
vibration_monitor = None  # VibrationMonitor 인스턴스를 저장할 변수
vibration_detection_active = False  # 진동 탐지 활성화 상태 변수
_vibration_monitor_lock = threading.Lock()

_last_emitted_state = None
grbl_wco = (0.0, 0.0, 0.0)  # GRBL이 마지막으로 보고한 작업 좌표 오프셋(WCO)


def notify_state(force=False):
    """state가 바뀌었으면 클라이언트에 state_update 이벤트를 보냅니다."""
    global _last_emitted_state
    if force or state != _last_emitted_state:
        _last_emitted_state = state
        socketio.emit('state_update', {"state": state})

def get_vibration_monitor():
    """VibrationMonitor를 한 번만 생성합니다 (생성 시 정상 진동 데이터로 모델 학습)."""
    global vibration_monitor
    with _vibration_monitor_lock:
        if vibration_monitor is None:
            # controllers를 전달하여 VibrationMonitor 인스턴스 생성
            vibration_monitor = VibrationMonitor(socketio, controllers, gcode_lines, tolerance_value)
    return vibration_monitor

def start_vibration_detection(reset_buffer=True):
    """진동 탐지를 시작하는 함수 (reset_buffer=False이면 일시정지 전 데이터를 이어서 사용)"""
    global vibration_detection_active
    
    logger.info("[vibration] 진동 탐지 시작")
    vibration_detection_active = True
    
    # 진동 모니터 초기화
    monitor = get_vibration_monitor()
    if reset_buffer:
        monitor.sensor_data_buffer = []
        monitor.abnormal_vibration_start_time = None
        monitor.abnormal_count = 0
    
    if state == "running":
        monitor.is_running = True # 진동 모니터링 시작

    if not monitor.is_trained:
        socketio.emit('log_update', {"message": "진동 분석 모델이 학습되지 않아 이상 진동 감지가 동작하지 않습니다. data/학습 데이터 폴더를 확인하세요.", "color": "orange"})

    logging.info("\n[vibration] ⚙️ 가공 시작: 진동 분석 시스템 활성화")
    logging.info("[vibration] 센서 데이터 수집 및 분석 시작")
    logging.info("[vibration] 이상 진동 감지 대기 중...\n")
    
    socketio.emit('log_update', {"message": "진동 탐지가 시작되었습니다.", "color": "green"})

def stop_vibration_detection():
    """진동 탐지를 중지하는 함수"""
    global vibration_detection_active, vibration_monitor
    
    if vibration_detection_active:
        logger.info("[vibration] 진동 탐지 중지")
        vibration_detection_active = False
        if vibration_monitor is not None:
            vibration_monitor.is_running = False # 진동 모니터링 중지
        logging.info("\n[vibration] ⏹️ 가공 종료: 진동 분석 시스템 비활성화")
        logging.info("[vibration] 센서 데이터 분석 중지\n")
        socketio.emit('log_update', {"message": "진동 탐지가 중지되었습니다.", "color": "blue"})

# 시리얼 포트 검색
def list_serial_ports():
    """사용 가능한 시리얼 포트 목록을 반환합니다. 연결 시도 없이 목록만 반환합니다."""
    try:
        ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
        return ports
    except Exception as e:
        return []

# GRBL 상태 보고 파싱
STATE_RE = re.compile(r'^<([A-Za-z]+)')
POSITION_RES = {key: re.compile(key + r':([-\d.]+),([-\d.]+),([-\d.]+)') for key in ('MPos', 'WPos', 'WCO')}
FS_RE = re.compile(r'FS:([-\d.]+),([-\d.]+)')
OV_RE = re.compile(r'Ov:(\d+),(\d+),(\d+)')

def parse_status_report(line):
    """GRBL 상태 보고를 dict로 변환합니다. 예: <Run|MPos:1.000,2.000,0.000|FS:500,8000|WCO:0.000,0.000,0.000>

    반환: {'state': 'Run', 'work': (x, y, z), 'MPos': ..., 'feed': .., 'spindle': .., 'ov': (feed%, rapid%, spindle%)}
    """
    global grbl_wco
    match = STATE_RE.match(line.strip())
    if not match:
        return None
    status = {'state': match.group(1)}
    for key, regex in POSITION_RES.items():
        pos_match = regex.search(line)
        if pos_match:
            status[key] = tuple(float(v) for v in pos_match.groups())
    if 'WCO' in status:
        grbl_wco = status['WCO']
    # 작업 좌표 = WPos, 또는 MPos - WCO ($10 설정에 따라 GRBL이 둘 중 하나만 보고함)
    if 'WPos' in status:
        status['work'] = status['WPos']
    elif 'MPos' in status:
        status['work'] = tuple(m - o for m, o in zip(status['MPos'], grbl_wco))
    fs_match = FS_RE.search(line)
    if fs_match:
        status['feed'], status['spindle'] = float(fs_match.group(1)), float(fs_match.group(2))
    ov_match = OV_RE.search(line)
    if ov_match:
        status['ov'] = tuple(int(v) for v in ov_match.groups())
    return status

def query_status(timeout=1.0):
    """GRBL에 '?'를 보내 상태 보고를 받아 dict로 반환합니다. (가공 스트리밍 중에는 스트리밍 스레드만 사용)"""
    with ser_lock:
        if not ser or not ser.is_open:
            return None
        ser.reset_input_buffer()
        ser.write(b"?")  # 실시간 명령: 개행을 붙이면 GRBL이 빈 줄에 대한 'ok'를 추가로 보냄
        ser.flush()
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('<'):
                return parse_status_report(line)
    return None

# 포트 연결 (컨트롤러 연결)
def connect_serial(port):
    """GRBL 컨트롤러에 시리얼 포트 연결"""
    global ser, state
    
    if not port:
        return {"status": "error", "message": "포트를 선택해주세요"}
    if job_active:
        return {"status": "error", "message": "가공 중에는 컨트롤러 연결을 변경할 수 없습니다."}
    
    try:
        # 기존 연결 해제
        disconnect_serial()
        
        # 데이터 처리를 위한 변수 초기화
        connected = False
        alarm_status = "unknown"  # 기본적으로 상태 알 수 없음으로 설정
        
        # GRBL 컨트롤러에 연결 시도 (115200 baud로 시도)
        try:
            with ser_lock:
                ser = serial.Serial(
                    port=port,
                    baudrate=115200,  # GRBL 기본 통신 속도
                    timeout=1.0,
                    write_timeout=1.0,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
            
                # 연결 즉시 상태 업데이트 (연결은 됐지만 상태 확인 필요)
                state = "unknown"  # 초기 상태를 unknown으로 설정
            
                # DTR 신호 전송 (아두이노 리셋)
                ser.dtr = True
                time.sleep(0.5)  # 리셋 대기
                ser.dtr = False
                time.sleep(1.0)  # 부팅 대기
            
                # 시리얼 버퍼 초기화
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            
                # 상태 확인 명령 전송 (실시간 명령이므로 개행 없이 전송)
                ser.write(b"?")
                ser.flush()  # 버퍼 즉시 전송
                time.sleep(0.3)  # 응답 대기
                # 응답 여러 줄 모두 읽기 (최대 1초 대기)
                responses = []
                start_time = time.time()
                while time.time() - start_time < 1.0:
                    while ser.in_waiting > 0:
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            responses.append(response)
                    time.sleep(0.05)
                # 여러 줄 중 ALARM/Alarm이 있는지 확인
                alarm_found = any("ALARM" in r.upper() or "<ALARM" in r.upper() for r in responses)
                if alarm_found:
                    alarm_status = "alarm"
                    state = "alarm"  # 알람 상태 설정
                elif any("<" in r and ">" in r and ("Idle" in r or "idle" in r.lower()) for r in responses):
                    alarm_status = "none"
                    state = "alarm_unlocked"  # 알람이 없으면 바로 가공 가능 상태로 설정
                else:
                    alarm_status = "unknown"
            
                connected = True
            
                # GRBL 설정 확인 ($언어 명령)
                if connected:
                    socketio.emit('log_update', {"message": "GRBL 설정 정보 확인 중...", "color": "blue"})
                    ser.reset_input_buffer()
                    ser.write(b"$$\n")
                    ser.flush()
                
                    # 설정 정보 응답 대기
                    settings_responses = []
                    start_time = time.time()
                    while time.time() - start_time < 2.0:  # 설정 읽기 위해 더 오래 대기
                        while ser.in_waiting > 0:
                            setting_response = ser.readline().decode('utf-8', errors='ignore').strip()
                            if setting_response:
                                settings_responses.append(setting_response)
                        time.sleep(0.05)
                
                    # GRBL 설정 응답이 있는지 확인
                    if settings_responses:
                        # 설정 로그 출력 (처음 5개만 표시)
                        settings_count = len(settings_responses)
                        socketio.emit('log_update', {"message": f"GRBL 설정 {settings_count}개 확인됨", "color": "green"})
                    
                        # 중요 설정 정보 로그에 기록 (처음 5개 설정만)
                        for i, setting in enumerate(settings_responses[:5]):
                            socketio.emit('log_update', {"message": f"GRBL 설정: {setting}", "color": "blue"})
                    
                        # 설정이 많은 경우 생략 메시지 표시
                        if settings_count > 5:
                            socketio.emit('log_update', {"message": f"외 {settings_count - 5}개 설정 생략됨", "color": "blue"})
                    else:
                        socketio.emit('log_update', {"message": "GRBL 설정 정보를 가져올 수 없습니다", "color": "orange"})
            
            notify_state()
            # 연결 성공 시 응답 상태에 따라 결과 반환
            if connected:
                if alarm_status == "alarm":
                    # 알람 상태면 알람 해제 필요
                    return {
                        "status": "success", 
                        "message": f"포트 {port} 연결됨 (알람 상태)",
                        "alarm_status": "alarm"
                    }
                elif alarm_status == "none":
                    # 알람이 없으면 바로 가공 가능 상태
                    return {
                        "status": "success", 
                        "message": f"포트 {port} 연결됨 (가공 준비 완료)",
                        "alarm_status": "none"
                    }
                else:
                    return {
                        "status": "success", 
                        "message": f"포트 {port} 연결됨 (상태 확인 필요)",
                        "alarm_status": "unknown"
                    }
            else:
                return {"status": "error", "message": f"포트 {port} 연결 실패: 응답 없음"}
        except serial.SerialException as e:
            return {"status": "error", "message": f"포트 {port} 연결 실패: {str(e)}"}
    except Exception as e:
        error_msg = f"연결 중 오류 발생: {str(e)}"
        return {"status": "error", "message": error_msg}

# 컨트롤러 연결 해제
def disconnect_serial():
    """시리얼 연결을 종료하고 컨트롤러와의 통신을 중단합니다."""
    global ser, state
    if job_active:
        return {"status": "error", "message": "가공 중에는 컨트롤러 연결을 해제할 수 없습니다. 긴급정지 후 다시 시도하세요."}
    with ser_lock:
        if ser and hasattr(ser, 'is_open'):
            try:
                if ser.is_open:
                    ser.close()
                ser = None
                state = "idle"
                return {"status": "success", "message": "컨트롤러 연결 해제됨"}
            except Exception as e:
                ser = None
                state = "idle"
                return {"status": "warning", "message": "연결 해제 중 오류 발생"}
    return {"status": "info", "message": "연결된 컨트롤러가 없습니다"}

def _sensor_dict(ax, ay, az, gx=None, gy=None, gz=None):
    """센서 값을 {'accel': {...}, 'gyro': {...}} 표준 형식으로 변환합니다."""
    def num(value):
        return float(value) if value is not None else 0.0
    return {
        'accel': {'x': num(ax), 'y': num(ay), 'z': num(az)},
        'gyro': {'x': num(gx), 'y': num(gy), 'z': num(gz)}
    }

def parse_sensor_line(line):
    """센서 한 줄을 표준 형식으로 변환합니다. 해석할 수 없으면 None을 반환합니다.

    지원 형식:
      - {"accel": {"x","y","z"}, "gyro": {"x","y","z"}}   (arduino/MPU6050 펌웨어)
      - {"accelX", "accelY", "accelZ", ...}                (arduino_example/sensor_example.ino, src/arduino_sensor.py)
      - {"x", "y", "z"}                                    (단순 3축)
      - "x: 5319 y: 5824 z: 2801"                          (텍스트)
    """
    try:
        data = json.loads(line)
    except json.JSONDecodeError:
        data = None
    if isinstance(data, dict):
        gyro = data.get('gyro') if isinstance(data.get('gyro'), dict) else {}
        if isinstance(data.get('accel'), dict):
            accel = data['accel']
            return _sensor_dict(accel.get('x'), accel.get('y'), accel.get('z'), gyro.get('x'), gyro.get('y'), gyro.get('z'))
        if all(key in data for key in ('accelX', 'accelY', 'accelZ')):
            return _sensor_dict(data['accelX'], data['accelY'], data['accelZ'], data.get('gyroX'), data.get('gyroY'), data.get('gyroZ'))
        if all(key in data for key in ('x', 'y', 'z')):
            return _sensor_dict(data['x'], data['y'], data['z'], gyro.get('x'), gyro.get('y'), gyro.get('z'))
        return None
    # JSON이 아닌 경우 텍스트 형식 파싱 시도 (예: "x: 5319 y: 5824 z: 2801")
    text_match = re.match(r'x:\s*(-?\d+(?:\.\d+)?)\s+y:\s*(-?\d+(?:\.\d+)?)\s+z:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)
    if text_match:
        return _sensor_dict(*text_match.groups())
    return None

# 아두이노 센서 데이터 읽기 함수
def read_arduino_serial():
    """아두이노 시리얼 연결을 통해 센서 데이터를 주기적으로 읽는 함수"""
    global latest_arduino_data, arduino_serial, arduino_connected, ARDUINO_SERIAL_PORT

    while True:
        # 연결이 활성화된 경우에만 데이터 읽기 시도
        if arduino_serial and arduino_serial.is_open:
            try:
                # 데이터 읽기
                if arduino_serial.in_waiting > 0:
                    line = arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            data = parse_sensor_line(line)
                        except (TypeError, ValueError):
                            data = None

                        if data is not None:
                            latest_arduino_data = json.dumps(data)
                            # 클라이언트는 arduino_data 하나로 센서 로그 표시와 센서 데이터 저장을 모두 처리
                            socketio.emit('arduino_data', {'data': latest_arduino_data})
                            
                            # 진동 분석을 위한 데이터 처리
                            if vibration_monitor is not None:
                                acc = data['accel']
                                vibration_monitor.process_sensor_data([[acc['x'], acc['y'], acc['z']]])  # 2D 배열로 전달
                        else:
                            # 파싱할 수 없는 형식(펌웨어 초기화 메시지 등)은 그대로 표시
                            latest_arduino_data = line
                            socketio.emit('log_update', {"message": f"[센서] {line}", "color": "sensor"})
                
                # 짧은 대기 시간
                time.sleep(0.0005)
            except serial.SerialException as se:
                # 시리얼 통신 오류 처리
                error_msg = str(se)
                if "ClearCommError" in error_msg or "핸들이 잘못되었습니다" in error_msg:
                    socketio.emit('log_update', {"message": "시리얼 포트 핸들 오류: 연결이 끊어졌습니다.", "color": "red"})
                else:
                    socketio.emit('log_update', {"message": f"센서 연결 오류: {error_msg}", "color": "red"})
                
                # 연결 종료 처리
                arduino_connected = False
                try:
                    if arduino_serial and arduino_serial.is_open:
                        arduino_serial.close()
                except:
                    pass
                
                arduino_serial = None
                socketio.emit('arduino_status', {'connected': False, 'port': None})
                socketio.emit('log_update', {"message": "센서 연결이 종료되었습니다. 다시 연결하려면 포트를 선택하고 연결 버튼을 클릭하세요.", "color": "orange"})
                time.sleep(5)
            except Exception as e:
                # 기타 모든 예외 처리
                arduino_connected = False
                socketio.emit('log_update', {"message": f"센서 데이터 처리 오류: {str(e)}", "color": "red"})
                time.sleep(1)
        else:
            # 연결이 없는 경우에는 단순히 대기
            time.sleep(0.5)

# connect_arduino 함수 수정 - 특정 포트에 연결
def connect_arduino(port=None):
    """특정 아두이노 센서 포트에 연결합니다. 포트가 명시적으로 지정된 경우에만 연결을 시도합니다."""
    global arduino_serial, arduino_connected, ARDUINO_SERIAL_PORT
    
    # 포트가 지정되지 않은 경우 연결 시도하지 않음
    if not port:
        socketio.emit('log_update', {"message": "연결할 포트가 지정되지 않았습니다. 포트를 선택해주세요.", "color": "orange"})
        return False
    
    # 이미 연결되어 있으면 해제
    if arduino_serial and arduino_serial.is_open:
        try:
            arduino_serial.close()
        except:
            pass
        arduino_serial = None
        arduino_connected = False
        socketio.emit('log_update', {"message": "기존 센서 연결을 해제했습니다.", "color": "blue"})
    
    # 선택한 포트에 연결 시도
    try:
        socketio.emit('log_update', {"message": f"센서 포트 {port}에 연결 시도 중...", "color": "blue"})
        
        # 명시적으로 지정된 포트에만 시리얼 포트 연결 시도
        arduino_serial = serial.Serial(
            port=port,
            baudrate=ARDUINO_BAUD_RATE,
            timeout=1,
            write_timeout=1
        )
        
        # 포트 연결 성공하면 2초 대기
        time.sleep(2)  # 아두이노 리셋 시간 대기
        
        # 연결 성공 여부 한번 더 확인
        if not arduino_serial or not arduino_serial.is_open:
            raise serial.SerialException("포트가 정상적으로 열리지 않았습니다.")
        
        # 연결 성공 처리
        ARDUINO_SERIAL_PORT = port
        arduino_connected = True
        socketio.emit('arduino_status', {'connected': True, 'port': port})
        socketio.emit('log_update', {"message": f"센서가 {port}에 성공적으로 연결되었습니다.", "color": "green"})
        
        return True
        
    except serial.SerialException as se:
        error_msg = str(se)
        if "PermissionError" in error_msg or "Access is denied" in error_msg or "액세스가 거부되었습니다" in error_msg:
            socketio.emit('log_update', {"message": f"포트 {port} 액세스 거부됨: 다른 프로그램이 이미 사용 중입니다.", "color": "red"})
        elif "not found" in error_msg or "찾을 수 없습니다" in error_msg:
            socketio.emit('log_update', {"message": f"포트 {port}를 찾을 수 없습니다.", "color": "red"})
        else:
            socketio.emit('log_update', {"message": f"포트 {port} 연결 실패: {error_msg}", "color": "red"})
        
        # 다른 포트 자동 시도 없이 연결 실패 처리
        if arduino_serial:
            try:
                arduino_serial.close()
            except:
                pass
            arduino_serial = None
            arduino_connected = False
        
        return False
        
    except Exception as e:
        error_msg = str(e)
        socketio.emit('log_update', {"message": f"포트 {port} 연결 실패: {error_msg}", "color": "red"})
        
        # 연결이 실패했을 경우 정리
        if arduino_serial:
            try:
                arduino_serial.close()
            except:
                pass
            arduino_serial = None
            arduino_connected = False
        
        return False
    
# GRBL에서 지원하지 않는 M 코드 (필요시 추가)
UNSUPPORTED_M_CODES = {0, 1, 6, 7, 9, 30, 48, 49, 60}
M_WORD_RE = re.compile(r'M\s*0*(\d+)(?![\d.])', re.IGNORECASE)

# NC 코드 파일을 불러오는 함수
def load_nc_code(file_content, is_rework=False):
    """NC 코드 파일 내용을 파싱하고 가공 경로를 초기화합니다.

    is_rework=False(사용자가 불러온 NC 코드)이면 재가공 경로 생성의 기준 코드로도 저장합니다.
    """
    global gcode_lines, rework_source_lines
    try:
        raw_lines = file_content.split('\n')
        new_lines = []
        for line in raw_lines:
            # 주석 제거 (GRBL 실행에 영향이 없고, 긴 주석은 수신 버퍼 계산을 어렵게 함)
            cleaned_line = strip_comments(line).strip()
            # 빈 줄, 한글 포함, G-code 단어가 없는 라인('%' 등) 무시
            if not cleaned_line or re.search(r'[ㄱ-ㅎ가-힣]', cleaned_line):
                continue
            words = parse_words(cleaned_line)
            if not words:
                continue
            g_codes = code_numbers(words, 'G')
            # G28/G30 원점 복귀 → 안전 높이로 Z를 먼저 올린 뒤 XY 원점으로 이동
            # (G0 X0 Y0 Z0 한 줄로 바꾸면 Z0(소재 윗면)까지 내려간 채 XY가 이동해 충돌할 수 있음)
            if 28 in g_codes or 30 in g_codes:
                axes = {letter for letter, _ in words if letter in 'XYZ'}
                new_lines.append(f"G0 Z{SAFE_Z_HEIGHT:g}")
                if not axes or axes & {'X', 'Y'}:
                    new_lines.append("G0 X0 Y0")
                continue
            # 지원하지 않는 M 코드는 해당 단어만 제거 (M03/M05처럼 앞에 0이 붙은 코드는 그대로 유지)
            if any(code in UNSUPPORTED_M_CODES for code in code_numbers(words, 'M')):
                cleaned_line = M_WORD_RE.sub(
                    lambda m: '' if int(m.group(1)) in UNSUPPORTED_M_CODES else m.group(0), cleaned_line
                ).strip()
                if not parse_words(cleaned_line):
                    continue
            new_lines.append(cleaned_line)
        gcode_lines = new_lines
        if not is_rework:
            rework_source_lines = list(new_lines)
        # NC 코드 로드 관련 로그 기록 추가
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_message = f"[{current_time}] NC 코드 로드 완료: 총 {len(gcode_lines)}개 라인"
        socketio.emit('log_update', {"message": log_message, "color": "green"})
        # NC 코드 라인을 로그창에 표시
        if len(gcode_lines) > 0:
            sample_log = "NC 코드 내용:"
            socketio.emit('log_update', {"message": sample_log, "color": "blue"})
            for i, line in enumerate(gcode_lines):
                line_log = f"  {i+1}: {line}"
                socketio.emit('log_update', {"message": line_log, "color": "gray"})
        return {"status": "success", "lines": len(gcode_lines)}
    except Exception as e:
        error_log = f"NC 코드 파싱 오류: {str(e)}"
        socketio.emit('log_update', {"message": error_log, "color": "red"})
        return {"status": "error", "message": error_log}

# 예상 가공 시간 계산 함수
def calculate_estimated_time(gcode_lines):
    """NC 코드의 이동 거리와 피드 속도를 분석하여 예상 가공 시간을 계산합니다.

    G 코드가 생략된 모달 이동(예: 'X30 F70')과 원호(G2/G3) 길이도 반영합니다.
    """
    total_time_min = 0  # 총 예상 시간 (분)
    modal = ModalState()
    # F가 나오기 전 이동에는 NC 코드의 첫 F 값을 사용 (없으면 안전한 기본값)
    first_feedrate = next((value for line in gcode_lines for letter, value in parse_words(line) if letter == 'F' and value > 0), 500)
    
    for line in gcode_lines:
        move = modal.apply(line)
        if move is None or move.length == 0:
            continue
        # G0은 급속 이송, 그 외는 현재 이송 속도
        feedrate = RAPID_FEEDRATE if move.mode == 'G0' else (modal.feed or first_feedrate)
        total_time_min += move.length / feedrate
    
    # NC 코드의 라인 수에 따른 추가 시간 (명령어 처리 시간)
    command_time_min = len(gcode_lines) * 0.01
    
    # 총 예상 시간 (초 단위)
    total_seconds = int((total_time_min + command_time_min) * 60)
    
    # HH:MM:SS 형식으로 변환
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    time_formatted = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    
    return {
        "total_seconds": total_seconds,
        "text": time_formatted
    }

# CSV 로깅 시작
def start_csv_logging():
    """가공 데이터를 CSV 파일로 저장하기 위한 로깅을 시작합니다."""
    global csv_file, csv_writer

    # 이전 로그 파일이 열려 있으면 닫기
    stop_csv_logging()
    
    # 로그 디렉토리 생성
    log_dir = os.path.join(BASE_DIR, 'logs')
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # 파일명 생성 (현재 시간 기반)
    current_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = os.path.join(log_dir, f"cnc_log_{current_time}.csv")
    
    # CSV 파일 열기
    csv_file = open(filename, 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(csv_file)
    
    # 헤더 작성 - 클라이언트 저장 방식과 동일하게 설정
    csv_writer.writerow(['X', 'Y', 'Z', 'ProcessTime'])
    
    return filename

# CSV 로깅 종료
def stop_csv_logging():
    """CSV 로깅을 종료하고 파일을 닫습니다."""
    global csv_file, csv_writer
    if csv_file:
        csv_file.close()
        csv_file = None
        csv_writer = None

# 가공 데이터 로깅
def log_process_data():
    """현재 좌표와 가공 경과 시간(초)을 CSV에 기록합니다."""
    if not csv_writer:
        return

    elapsed_sec = time.time() - process_start_time if process_start_time else 0

    current_x = x_vals[-1] if x_vals else 0
    current_y = y_vals[-1] if y_vals else 0
    current_z = z_vals[-1] if z_vals else 0

    csv_writer.writerow([
        f"{current_x:.4f}",
        f"{current_y:.4f}",
        f"{current_z:.4f}",
        int(round(elapsed_sec))
    ])

# 알람 상태 체크 (주기적)
def check_alarm():
    """주기적으로 CNC 컨트롤러의 알람 상태를 확인합니다."""
    global ser, state, last_alarm_check, last_alarm_message, alarm_retry_count
    
    while True:
        # 현재 시간 기록
        current_time = time.time()
        
        # 시리얼 연결 상태와 현재 가공 상태 확인 (가공 중에는 스트리밍 스레드만 시리얼을 읽음)
        has_connection = ser and hasattr(ser, 'is_open') and ser.is_open
        
        # 일정 간격으로만 알람 확인 (너무 자주 확인하면 통신 부하 발생)
        if has_connection and not job_active and (current_time - last_alarm_check) >= alarm_check_interval:
            last_alarm_check = current_time
            
            try:
                with ser_lock:
                    if job_active or not ser or not ser.is_open:
                        raise RuntimeError("skip")
                    # 이전에 입력 버퍼에 남아있는 데이터가 있으면 읽어서 처리
                    if ser.in_waiting > 0:
                        # 버퍼 데이터 모두 읽기
                        while ser.in_waiting > 0:
                            buffer_data = ser.readline().decode('utf-8', errors='ignore').strip()
                            if buffer_data and ("ERROR" in buffer_data.upper() or "ALARM" in buffer_data.upper()):
                                # 버퍼에서 알람 상태 발견 (긴급정지 상태는 사용자가 해제할 때까지 유지)
                                if last_alarm_message != buffer_data:
                                    last_alarm_message = buffer_data
                                    if state != "emergency":
                                        state = "alarm"
                
                    # 버퍼 내용 처리 후 입력 버퍼 비우기 (중요: 상태 쿼리 전에 버퍼 정리)
                    ser.reset_input_buffer()
                
                    # 상태 확인 명령 전송 (실시간 명령이므로 개행 없이 전송)
                    ser.write(b"?")
                
                    # 응답 대기
                    start_time = time.time()
                    max_wait = 0.8  # 응답 대기 시간 (800ms로 충분)
                
                    # 응답 대기 (제한된 시간 내에)
                    response_received = False
                    while time.time() - start_time < max_wait:
                        if ser.in_waiting > 0:
                            # 한 줄씩 읽어서 처리
                            response = ser.readline().decode('utf-8', errors='ignore').strip()
                            if response:  # 빈 응답 무시
                                response_received = True
                            
                                # 알람 상태인 경우 (긴급정지 상태는 사용자가 해제할 때까지 유지)
                                if "ALARM" in response.upper() or "ERROR" in response.upper():
                                    if state not in ("alarm", "emergency"):
                                        state = "alarm"
                                    
                                        # 알람 상태 표시
                                        socketio.emit('alarm_update', {'text': f"알람 상태: {response[:50]}", 'color': 'red'})
                                # 정상 상태인 경우 (Idle)
                                elif "<" in response and ">" in response and ("Idle" in response or "idle" in response.lower()):
                                    # 알람이 없고 idle 상태이면 가공 가능 상태로 설정
                                    if state == "alarm":
                                        # 알람 상태는 유지 (해제 버튼을 눌러야만 해제됨)
                                        pass
                                    elif state not in ["running", "paused", "emergency"]:
                                        # 알람이 없는 상태일 때는 가공 가능 상태로 설정
                                        state = "alarm_unlocked"
                            
                                break
                        time.sleep(0.01)
                
                if response_received:
                    alarm_retry_count = 0
                # 응답이 없는 경우
                else:
                    alarm_retry_count += 1
                    
                    # 연속적으로 응답이 없을 때만 경고 표시 (로그만 표시하고 UI는 업데이트하지 않음)
                    if alarm_retry_count >= max_alarm_retries:
                        # 사용자 혼란 방지를 위해 로그는 한 번만 표시
                        if state != "connection_warning":
                            state = "connection_warning"
                
            except RuntimeError:
                pass
            except Exception as e:
                # 심각한 오류인 경우에만 알람 표시 (일시적인 오류는 무시)
                alarm_retry_count += 1
                if alarm_retry_count >= max_alarm_retries:
                    error_msg = str(e)
                    if "Device not configured" in error_msg or "Port is closed" in error_msg:
                        state = "idle"
                    else:
                        socketio.emit('log_update', {"message": f"통신 오류: {error_msg[:50]}", "color": "orange"})
                    alarm_retry_count = 0

        # 상태가 바뀌었으면 클라이언트에 알림
        notify_state()
        
        # 다음 체크까지 대기 (대기 시간 최적화)
        time.sleep(1.0)

# 알람 해제 함수
def unlock_alarm():
    """알람 상태를 해제합니다."""
    global state
    
    try:
        if not ser or not ser.is_open:
            socketio.emit('log_update', {"message": "시리얼 포트가 연결되어 있지 않습니다.", "color": "red"})
            return False
        if job_active:
            socketio.emit('log_update', {"message": "가공 중에는 알람을 해제할 수 없습니다.", "color": "red"})
            return False
            
        # 알람 해제 명령 전송
        socketio.emit('log_update', {"message": "알람 해제 명령 전송 중...", "color": "blue"})
        
        with ser_lock:
            # 버퍼 초기화 - 중요: 명확하게 이전 버퍼 데이터를 제거
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        
            # 상태 확인 명령 먼저 전송하여 현재 상태 확인
            ser.write(b"?")
            ser.flush()  # 즉시 전송 보장
            time.sleep(0.5)  # 응답 대기
        
            # 현재 상태 확인 응답 처리
            status_message = ""
            if ser.in_waiting > 0:
                status_message = ser.readline().decode('utf-8', errors='ignore').strip()
                socketio.emit('log_update', {"message": f"현재 상태: {status_message}", "color": "blue"})
        
            # 입력 버퍼 다시 초기화
            ser.reset_input_buffer()
        
            # $X 명령 전송 (알람 해제)
            ser.write(b"$X\n")
            ser.flush()  # 즉시 전송 보장
            time.sleep(1.0)  # 응답 대기 시간 증가
        
            # 응답 확인
            response = ""
            unlock_success = False
            
            while ser.in_waiting > 0:
                current_response = ser.readline().decode('utf-8', errors='ignore').strip()
                if current_response:
                    response = current_response
                    if "ok" in response.lower():
                        unlock_success = True
                    socketio.emit('log_update', {"message": f"알람 해제 응답: {response}", "color": "green"})
            
            # 응답이 없거나 성공 응답이 아닌 경우, 추가 확인
            if not unlock_success:
                # 두 번째 $X 명령 시도 (첫 번째가 실패한 경우)
                socketio.emit('log_update', {"message": "두 번째 알람 해제 명령 시도...", "color": "blue"})
                ser.reset_input_buffer()
                ser.write(b"$X\n")
                ser.flush()
                time.sleep(1.0)
                
                while ser.in_waiting > 0:
                    second_response = ser.readline().decode('utf-8', errors='ignore').strip()
                    if second_response:
                        response = second_response
                        if "ok" in response.lower():
                            unlock_success = True
                        socketio.emit('log_update', {"message": f"두 번째 알람 해제 응답: {response}", "color": "green"})

                # 여전히 성공하지 못한 경우 상태 확인
                if not unlock_success:
                    # 상태 확인으로 다시 시도
                    ser.reset_input_buffer()
                    ser.write(b"?")
                    ser.flush()
                    time.sleep(0.5)

                    if ser.in_waiting > 0:
                        status_response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if status_response:
                            socketio.emit('log_update', {"message": f"상태 확인 응답: {status_response}", "color": "blue"})
                            # <Idle> 상태면 알람이 해제된 것
                            if "Idle" in status_response or "idle" in status_response.lower():
                                unlock_success = True
        
        # 최종 상태 업데이트
        if unlock_success:
            state = "alarm_unlocked"
            notify_state()
            socketio.emit('log_update', {"message": "알람이 성공적으로 해제되었습니다.", "color": "green"})
            socketio.emit('alarm_update', {'text': "알람 해제됨", 'color': 'green'})
            socketio.emit('status_update', {"text": "알람 해제됨: 가공 준비 완료", "color": "green"})
            return True
        else:
            # 알람 해제 실패 시 추가 정보
            if response:
                socketio.emit('log_update', {"message": f"알람 해제 실패: {response}", "color": "orange"})
            else:
                socketio.emit('log_update', {"message": "알람 해제 명령에 응답이 없습니다. 컨트롤러 연결을 확인하세요.", "color": "orange"})
                
            socketio.emit('alarm_update', {'text': "알람 해제 실패", 'color': 'red'})
            socketio.emit('status_update', {"text": "알람 해제 실패: 재시도 필요", "color": "red"})
            return False
    except Exception as e:
        socketio.emit('log_update', {"message": f"알람 해제 중 오류: {str(e)}", "color": "red"})
        socketio.emit('alarm_update', {'text': f"오류: {str(e)[:30]}", 'color': 'red'})
        return False

# === 가공 작업 제어 ===
def reset_path(x=0.0, y=0.0, z=0.0):
    """가공 경로를 초기화합니다 (다음 update_ui에서 클라이언트가 전체 경로를 다시 받음)."""
    global ui_sent_len
    x_vals[:] = [x]
    y_vals[:] = [y]
    z_vals[:] = [z]
    ui_sent_len = 0

def start_job():
    """현재 gcode_lines로 새 가공 작업(스트리밍 스레드)을 시작합니다. (성공 여부, 메시지)를 반환합니다."""
    global job_active, job_generation, state, pause, progress, gcode_current_idx
    with job_lock:
        if job_active:
            return False, "이미 가공 작업이 진행 중입니다."
        if not ser or not ser.is_open:
            return False, "시리얼 포트가 연결되지 않았습니다."
        if not gcode_lines:
            return False, "NC 코드가 로드되지 않았습니다."
        job_generation += 1
        generation = job_generation
        job_active = True
        state = "running"
        pause = False
        progress = 0
        gcode_current_idx = 0
    notify_state()
    threading.Thread(target=send_gcode, args=(generation,), daemon=True).start()
    return True, "가공이 시작되었습니다."

def pause_job():
    """피드 홀드(!)로 가공을 일시정지합니다. 스트리밍 스레드는 응답을 계속 읽으며 대기합니다."""
    global state, pause
    if state != "running" or not job_active:
        return False, "가공 중이 아닐 때는 일시정지를 할 수 없습니다."
    with ser_lock:
        ser.write(b"!")  # 실시간 명령: 피드 홀드
        ser.flush()
    pause = True
    state = "paused"
    if vibration_detection_active:
        stop_vibration_detection()
    notify_state()
    return True, "가공이 일시정지되었습니다."

def resume_job():
    """사이클 시작(~)으로 일시정지된 가공을 중단된 지점부터 이어서 진행합니다."""
    global state, pause
    if state != "paused":
        return False, "일시정지 상태가 아닙니다."
    if not job_active:
        state = "idle"
        notify_state()
        return False, "재개할 가공 작업이 없습니다. 가공을 다시 시작하세요."
    with ser_lock:
        ser.write(b"~")  # 실시간 명령: 사이클 시작/재개
        ser.flush()
    pause = False
    state = "running"
    start_vibration_detection(reset_buffer=False)
    notify_state()
    return True, "가공이 재개되었습니다."

def emergency_halt():
    """가공을 즉시 중단합니다: 스트리밍 스레드 종료 신호 + GRBL 소프트 리셋(Ctrl-X).

    소프트 리셋은 GRBL 버퍼에 남은 이동과 스핀들/이송을 즉시 멈춥니다.
    """
    global job_generation, state, pause
    with job_lock:
        job_generation += 1  # 이후 스트리밍 스레드는 G-code를 더 보내지 않고 종료
    state = "emergency"
    pause = True
    with ser_lock:
        if ser and ser.is_open:
            ser.write(b"\x18")
            ser.flush()
    notify_state()

def recover_after_reset(timeout=5.0):
    """긴급정지(소프트 리셋) 후 스트리밍 스레드 종료를 기다린 뒤 GRBL 알람을 해제합니다. 성공 여부 반환."""
    deadline = time.time() + timeout
    while job_active and time.time() < deadline:
        time.sleep(0.05)
    if job_active:
        logger.warning("스트리밍 스레드가 종료되지 않아 GRBL 복구를 건너뜁니다.")
        return False
    time.sleep(2.0)  # 소프트 리셋 후 GRBL 초기화 대기
    return unlock_alarm()

def effective_cutting_conditions():
    """현재 실제 절삭 조건 (프로그램 S/F에 오버라이드 비율을 반영). 값이 없으면 None."""
    n = current_spindle * spindle_override / 100.0 if current_spindle else None
    F = current_feedrate * feed_override / 100.0 if current_feedrate else None
    return n, F

def arm_rework():
    """곧 생성될 rework.nc의 자동 가공을 1회 허용합니다 (이상 진동 복구 과정에서만 호출)."""
    global rework_armed_at
    rework_armed_at = time.time()

def disarm_rework():
    global rework_armed_at
    rework_armed_at = None

def on_rework_file_ready(path):
    """파일 감시자 콜백: nc/rework.nc가 생성·수정되었을 때 호출됩니다."""
    global rework_armed_at
    armed = rework_armed_at is not None and time.time() - rework_armed_at <= REWORK_ARM_TIMEOUT
    rework_armed_at = None
    if not armed:
        socketio.emit('log_update', {"message": "rework.nc 변경 감지: 이상 진동 복구로 생성된 파일이 아니므로 자동 가공하지 않습니다.", "color": "gray"})
        return
    logger.info("[watcher] rework.nc 감지 - 재가공 자동 시작")
    start_rework_job(path)

def start_rework_job(path=None):
    """rework.nc를 읽어 재가공을 시작합니다. (성공 여부, 메시지)를 반환합니다."""
    path = path or os.path.join(BASE_DIR, 'nc', 'rework.nc')
    if job_active:
        message = "이미 가공 작업이 진행 중이어서 rework.nc를 시작할 수 없습니다."
        socketio.emit('log_update', {"message": message, "color": "red"})
        return False, message
    if state not in ("idle", "alarm_unlocked"):
        message = f"현재 상태({state})에서는 재가공을 시작할 수 없습니다. 알람을 해제한 뒤 다시 시도하세요."
        socketio.emit('log_update', {"message": message, "color": "red"})
        return False, message

    try:
        with open(path, 'r', encoding='utf-8') as f:
            rework_content = f.read()
    except Exception as e:
        message = f"rework.nc 파일을 읽을 수 없습니다: {str(e)}"
        socketio.emit('log_update', {"message": message, "color": "red"})
        return False, message

    result = load_nc_code(rework_content, is_rework=True)
    if result.get('status') != 'success':
        message = f"rework.nc 로드 실패: {result.get('message')}"
        socketio.emit('log_update', {"message": message, "color": "red"})
        return False, message

    socketio.emit('log_update', {"message": "rework.nc 로드 완료, 가공을 시작합니다.", "color": "green"})
    ok, message = start_job()
    if ok:
        socketio.emit('status_update', {"text": "가공 시작", "color": "blue"})
        socketio.emit('disable_nc_upload', {})
    else:
        socketio.emit('log_update', {"message": message, "color": "red"})
    return ok, message

def apply_override(target, value):
    """GRBL 1.1 실시간 명령으로 Feed/Spindle 오버라이드를 value(%)로 설정합니다.

    GRBL에는 절대값 설정 명령이 없으므로 100%로 리셋한 뒤 ±10%, ±1% 단계 명령을 필요한 만큼 보냅니다.
    (같은 명령이 GRBL 메인 루프 한 주기 안에 겹치면 한 번으로 처리되므로 명령 사이에 간격을 둡니다)
    """
    global feed_override, spindle_override
    commands = {'feed': FEED_OVERRIDE_CMDS, 'spindle': SPINDLE_OVERRIDE_CMDS}.get(target)
    if commands is None:
        return {"status": "error", "message": f"알 수 없는 오버라이드 대상: {target}"}
    if not ser or not ser.is_open:
        return {"status": "error", "message": "시리얼 포트가 연결되지 않았습니다."}
    try:
        requested = int(round(float(value)))
    except (TypeError, ValueError):
        return {"status": "error", "message": f"잘못된 오버라이드 값: {value}"}

    clamped = max(OVERRIDE_RANGE[0], min(OVERRIDE_RANGE[1], requested))
    diff = clamped - 100
    sign = '+' if diff > 0 else '-'
    sequence = [commands['reset']] + [commands[sign + '10']] * (abs(diff) // 10) + [commands[sign + '1']] * (abs(diff) % 10)
    try:
        for command in sequence:
            with ser_lock:
                ser.write(command)
                ser.flush()
            time.sleep(0.03)
    except Exception as e:
        return {"status": "error", "message": f"오버라이드 명령 전송 실패: {str(e)}"}

    if target == 'feed':
        feed_override = clamped
    else:
        spindle_override = clamped
    socketio.emit('override_status', {'spindle': spindle_override, 'feed': feed_override})

    label = 'Feed' if target == 'feed' else 'Spindle'
    message = f"{label} Override {clamped}% 설정됨"
    if clamped != requested:
        message += f" (GRBL 허용 범위 {OVERRIDE_RANGE[0]}~{OVERRIDE_RANGE[1]}%로 조정)"
    return {"status": "success", "message": message, "value": clamped}

def _read_serial_lines(rx, wait=0.02):
    """도착한 데이터를 읽어 완성된 줄 목록을 반환합니다 (끝나지 않은 줄은 rx에 보관)."""
    with ser_lock:
        waiting = ser.in_waiting
        if waiting:
            rx.extend(ser.read(waiting))
    if not waiting:
        time.sleep(wait)
        return []
    lines = []
    while b'\n' in rx:
        raw, _, rest = rx.partition(b'\n')
        rx[:] = rest
        text = raw.decode('utf-8', errors='ignore').strip()
        if text:
            lines.append(text)
    return lines

def _apply_job_status(status):
    """가공 중 상태 보고로 실제 좌표 경로, CSV 로그, 오버라이드 표시를 갱신합니다."""
    global feed_override, spindle_override
    work = status.get('work')
    if work:
        x, y, z = (round(v, 4) for v in work)
        if (x, y, z) != (x_vals[-1], y_vals[-1], z_vals[-1]):
            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)
            log_process_data()
    ov = status.get('ov')
    if ov and (ov[0] != feed_override or ov[2] != spindle_override):
        feed_override, spindle_override = ov[0], ov[2]
        socketio.emit('override_status', {'spindle': spindle_override, 'feed': feed_override})
    update_ui()

# G코드 명령 전송 및 실행
def send_gcode(generation=None):
    """G코드를 GRBL로 스트리밍하는 함수 (start_job()이 별도 스레드로 실행)

    - 문자 수 기반 흐름 제어: GRBL 수신 버퍼(128바이트)에 들어간 바이트 수를 세고, 'ok'를 받을 때마다 다음 줄을 보냅니다.
      (응답 지연 시 같은 줄을 다시 보내지 않습니다 — 재전송은 중복 이동을 만듭니다)
    - 화면 경로와 CSV 로그는 G-code 계산값이 아니라 GRBL 상태 보고(?)의 실제 좌표를 사용하므로,
      'X30 F70'처럼 G 코드가 생략된 모달 이동이나 원호(G2/G3)도 그대로 반영됩니다.
    - 'error:' 응답을 받으면 피드 홀드 후 일시정지하고 사용자에게 알립니다.
    - 일시정지(!) 중에도 응답을 계속 읽어 버퍼 계산이 어긋나지 않게 하고, 재개(~)하면 이어서 전송합니다.
    - 모든 줄의 'ok'를 받은 뒤 GRBL이 Idle이 될 때(실제 이동이 끝날 때)까지 기다렸다가 완료 처리합니다.
    """
    global state, pause, progress, current_spindle, current_feedrate, base_spindle, base_feedrate
    global gcode_current_idx, job_active, process_start_time, last_job_elapsed, last_alarm_message

    if generation is None:
        generation = job_generation

    def aborted():
        return generation != job_generation or state in ("emergency", "stopped", "alarm") or not ser or not ser.is_open

    lines = list(gcode_lines)
    total = len(lines)
    idx = 0
    completed = False
    
    print("\n[DEBUG] send_gcode 함수 시작", file=sys.stderr)
    print(f"[DEBUG] 현재 상태: {state}", file=sys.stderr)
    
    try:
        if not ser or not ser.is_open:
            socketio.emit('log_update', {"message": "시리얼 포트가 연결되지 않았습니다.", "color": "red"})
            return
        if not lines:
            socketio.emit('log_update', {"message": "가공할 G코드가 없습니다.", "color": "red"})
            return

        # G-code에서 최초 스핀들 속도 및 이송 속도 파싱 또는 기본값 설정
        initial_spindle_found = False
        initial_feedrate_found = False
        for line in lines:
            for letter, value in parse_words(line):
                if letter == 'S' and not initial_spindle_found:
                    current_spindle = value
                    initial_spindle_found = True
                elif letter == 'F' and not initial_feedrate_found:
                    current_feedrate = value
                    initial_feedrate_found = True
            if initial_spindle_found and initial_feedrate_found:
                break

        # G-code에서 찾지 못하면 안전한 기본값으로 설정
        if not initial_spindle_found:
            current_spindle = 10000.0  # 안전한 기본 스핀들 속도
            socketio.emit('log_update', {"message": f"G-code에서 스핀들 속도를 찾을 수 없어 기본값 S{int(current_spindle)}으로 설정합니다.", "color": "orange"})
        if not initial_feedrate_found:
            current_feedrate = 200.0  # 안전한 기본 이송 속도
            socketio.emit('log_update', {"message": f"G-code에서 이송 속도를 찾을 수 없어 기본값 F{int(current_feedrate)}으로 설정합니다.", "color": "orange"})

        base_spindle = current_spindle # 최초 스핀들 속도를 base_spindle로 설정
        base_feedrate = current_feedrate # 최초 이송 속도를 base_feedrate로 설정

        # 시작 위치 확인 (실제 작업 좌표로 경로 시작, 기계 좌표 Z는 current_position.txt에 기록)
        status = query_status()
        start_pos = status.get('work', (0.0, 0.0, 0.0)) if status else (0.0, 0.0, 0.0)
        if status and 'MPos' in status:
            with open(os.path.join(BASE_DIR, 'data', 'current_position.txt'), 'w', encoding='utf-8') as f:
                f.write(str(status['MPos'][2]))
        reset_path(*(round(v, 4) for v in start_pos))

        # 진동 탐지 활성화 및 CSV 로깅 시작
        print("[DEBUG] 진동 탐지 활성화 시도", file=sys.stderr)
        start_vibration_detection()
        start_csv_logging()
        process_start_time = time.time()
        log_process_data()
        
        # 웹 인터페이스 업데이트
        socketio.emit('log_update', {"message": "가공이 시작되었습니다.", "color": "green"})
        socketio.emit('log_update', {"message": f"가공 시작: 총 {total}개 명령 실행 예정", "color": "blue"})
        socketio.emit('progress_update', {"progress": progress})
        notify_state(force=True)
        update_ui(full=True)

        modal = ModalState()  # 'ok'를 받은 줄 기준으로 현재 프로그램 S/F 추적
        pending = deque()     # GRBL 버퍼에 들어가 'ok'를 기다리는 줄: (인덱스, 내용, 바이트 수)
        buffer_used = 0
        acked = 0
        rx = bytearray()
        last_poll = 0.0
        last_ack_time = time.time()
        idle_streak = 0       # 마지막 'ok' 이후 연속으로 받은 Idle 상태 보고 수
        stall_warned = False

        while True:
            if aborted():
                break

            # 1) 가공 중이면 GRBL 수신 버퍼 여유만큼 다음 줄 전송
            if state == "running":
                while idx < total:
                    line = lines[idx]
                    line_len = len(line) + 1  # +1 for '\n'
                    if line_len > GRBL_RX_BUFFER_SIZE:
                        raise ValueError(f"{idx + 1}번째 줄이 GRBL 수신 버퍼보다 깁니다: {line[:40]}...")
                    if buffer_used + line_len > GRBL_RX_BUFFER_SIZE:
                        break
                    with ser_lock:
                        if aborted():
                            break
                        ser.write(f"{line}\n".encode())
                    pending.append((idx, line, line_len))
                    buffer_used += line_len
                    idx += 1
                gcode_current_idx = idx

            # 2) 주기적으로 실제 위치/상태 조회
            now = time.time()
            if now - last_poll >= STATUS_POLL_INTERVAL:
                with ser_lock:
                    if aborted():
                        break
                    ser.write(b"?")
                last_poll = now

            # 3) 응답 처리
            for response in _read_serial_lines(rx):
                if response == "ok":
                    if not pending:
                        continue
                    sent_idx, sent_line, sent_len = pending.popleft()
                    buffer_used -= sent_len
                    acked += 1
                    last_ack_time = time.time()
                    idle_streak = 0
                    stall_warned = False
                    modal.apply(sent_line)
                    if modal.feed:
                        current_feedrate = modal.feed
                    if modal.spindle:
                        current_spindle = modal.spindle
                    progress = int(acked / total * 100)

                    if sent_idx % 10 == 0 or sent_idx < 5:
                        socketio.emit('log_update', {"message": f"명령 {sent_line} -> 응답: ok", "color": "green"})
                    if (sent_idx + 1) % max(1, total // 10) == 0:
                        socketio.emit('log_update', {"message": f"가공 진행률: {progress}%", "color": "blue"})

                elif response.startswith("error"):
                    # GRBL은 오류가 난 줄을 버리고 다음 줄을 계속 실행하므로 즉시 피드 홀드 후 사용자 판단을 기다림
                    failed_line = ""
                    if pending:
                        _, failed_line, sent_len = pending.popleft()
                        buffer_used -= sent_len
                        acked += 1
                    idle_streak = 0
                    with ser_lock:
                        ser.write(b"!")
                    if state == "running":
                        pause = True
                        state = "paused"
                        if vibration_detection_active:
                            stop_vibration_detection()
                    socketio.emit('log_update', {"message": f"명령 {failed_line} -> 오류: {response}. 가공을 일시정지했습니다. 확인 후 '가공 시작'으로 재개하거나 긴급정지하세요.", "color": "red"})
                    socketio.emit('status_update', {"text": "일시정지", "color": "orange"})
                    notify_state()
                    
                elif response.startswith("<"):
                    status = parse_status_report(response)
                    if not status:
                        continue
                    # Check($C 검사 모드)는 실제 이동 없이 Idle처럼 대기하는 상태
                    idle_streak = idle_streak + 1 if status['state'] in ("Idle", "Check") else 0
                    if status['state'] == "Alarm":
                        state = "alarm"
                        socketio.emit('alarm_update', {'text': "알람 상태: 해제가 필요합니다", 'color': 'red'})
                    _apply_job_status(status)

                elif response.startswith("ALARM"):
                    state = "alarm"
                    last_alarm_message = response
                    socketio.emit('log_update', {"message": f"GRBL 알람 발생: {response}", "color": "red"})
                    socketio.emit('alarm_update', {'text': f"알람 상태: {response[:50]}", 'color': 'red'})

                else:
                    socketio.emit('log_update', {"message": f"GRBL: {response}", "color": "gray"})

            # 4) 완료: 모든 줄의 'ok' 수신 후 GRBL이 연속으로 Idle을 보고하면 실제 이동이 끝난 것
            if idx >= total and not pending and idle_streak >= 2:
                completed = True
                break

            # 응답이 오래 없으면 한 번만 경고 (재전송하지 않음, 필요하면 사용자가 긴급정지)
            if state == "running" and pending and not stall_warned and time.time() - last_ack_time > 30:
                stall_warned = True
                socketio.emit('log_update', {"message": "GRBL 응답을 30초 이상 기다리는 중입니다. 컨트롤러 상태를 확인하세요.", "color": "orange"})

    except Exception as e:
        logger.error(f"G코드 전송 중 오류: {e}", exc_info=True)
        socketio.emit('log_update', {"message": f"G코드 전송 중 오류: {str(e)}", "color": "red"})
        if generation == job_generation and state in ("running", "paused"):
            # GRBL 버퍼에 이미 보낸 이동이 남아 있으므로 소프트 리셋으로 정지 (긴급정지 해제 후 다시 시작)
            try:
                with ser_lock:
                    if ser and ser.is_open:
                        ser.write(b"\x18")
            except Exception:
                pass
            state = "emergency"
            socketio.emit('status_update', {"text": "긴급정지: 가공 오류로 중단됨", "color": "red"})
    finally:
        stop_csv_logging()
        if vibration_detection_active:
            stop_vibration_detection()
        if process_start_time:
            last_job_elapsed = time.time() - process_start_time
        process_start_time = None
        gcode_current_idx = 0 if completed else idx
        with job_lock:
            job_active = False

        if completed:
            state = "idle"
            progress = 100
            socketio.emit('status_update', {"text": "가공 완료", "color": "blue"})
            socketio.emit('log_update', {"message": "가공이 성공적으로 완료되었습니다.", "color": "green"})
            socketio.emit('enable_nc_upload', {})
        else:
            if state in ("running", "paused"):
                # 스레드가 끝났으므로 재개할 수 없음 (포트 끊김 등으로 중단된 경우 다시 시작)
                state = "idle"
            socketio.emit('log_update', {"message": "가공이 중단되었습니다.", "color": "red"})
        notify_state()
        update_ui()

# UI 업데이트 함수
def update_ui(full=False):
    """현재 가공 상태를 UI에 업데이트합니다. 경로는 지난번 전송 이후 추가된 점만 보냅니다 (full=True면 전체)."""
    global ui_sent_len
    
    # 현재 경과 시간 계산 (가공 중이 아니면 마지막 가공의 경과 시간 유지)
    elapsed_time = time.time() - process_start_time if process_start_time else last_job_elapsed
    
    start = 0 if full or ui_sent_len > len(x_vals) else ui_sent_len
    end = len(x_vals)
    socketio.emit('position_update', {
        'x': x_vals[-1],
        'y': y_vals[-1],
        'z': z_vals[-1],
        'path_start': start,
        'path_x': x_vals[start:end],
        'path_y': y_vals[start:end],
        'path_z': z_vals[start:end],
        'spindle': current_spindle,
        'feedrate': current_feedrate,
        'progress': progress,
        'elapsed_time': elapsed_time,  # 경과 시간 추가
        'state': state
    })
    ui_sent_len = end


# 주기적 연결 상태 확인
def check_connection():
    """주기적으로 컨트롤러 연결 상태를 확인합니다."""
    global ser, state
    
    # 연결 확인 관련 변수
    check_interval = 5.0  # 5초마다 연결 상태 확인
    consecutive_failures = 0
    max_consecutive_failures = 3
    last_check_time = 0
    
    while True:
        try:
            # 현재 시간 확인
            current_time = time.time()
            
            # 일정 간격으로만 확인 (너무 자주 확인하면 성능 저하)
            if current_time - last_check_time < check_interval:
                time.sleep(0.5)
                continue
                
            # 확인 시간 업데이트
            last_check_time = current_time
            
            # 시리얼 연결이 있고, 열려있고, 가공 작업(일시정지 포함)이 없을 때만 체크
            if ser and hasattr(ser, 'is_open') and ser.is_open and not job_active:
                try:
                    with ser_lock:
                        if job_active or not ser or not ser.is_open:
                            continue
                        # 버퍼에 남아 있는 데이터가 있으면 처리 (로그 남기지 않음)
                        if ser.in_waiting > 10:
                            ser.reset_input_buffer()
                    
                        # 간단한 상태 확인 명령 (실시간 명령이므로 개행 없이 전송)
                        ser.write(b"?")
                        ser.flush()
                    
                        # 응답 대기
                        start_time = time.time()
                        response_timeout = 0.8  # 응답 타임아웃 시간
                        response_received = False
                    
                        while time.time() - start_time < response_timeout:
                            if ser.in_waiting > 0:
                                try:
                                    response = ser.readline().decode('utf-8', errors='ignore').strip()
                                    if response:
                                        response_received = True
                                    
                                        # 알람 상태 확인 (긴급정지 상태는 사용자가 해제할 때까지 유지)
                                        if "ERROR" in response.upper() or "ALARM" in response.upper():
                                            if state not in ("alarm", "emergency"):
                                                state = "alarm"
                                    
                                        # 연결 경고 상태 해제
                                        if state == "connection_warning":
                                            state = "idle"
                                            socketio.emit('connection_status', {'connected': True, 'warning': False, 'port': ser.port})
                                            socketio.emit('log_update', {"message": "컨트롤러 응답이 정상으로 돌아왔습니다", "color": "green"})
                                    
                                        # 응답 성공으로 실패 카운터 초기화
                                        consecutive_failures = 0
                                        break
                                except Exception:
                                    # 응답 처리 중 오류는 무시하고 계속 진행
                                    pass
                                
                            time.sleep(0.05)
                    
                    # 응답을 받지 못한 경우
                    if not response_received:
                        consecutive_failures += 1
                        
                        # 연속 실패 횟수에 따른 처리 (로그 과다 방지)
                        if consecutive_failures >= max_consecutive_failures and state != "connection_warning":
                            state = "connection_warning"
                
                except Exception as e:
                    # 심각한 통신 문제 발생 시
                    consecutive_failures += 1
                    
                    if consecutive_failures >= max_consecutive_failures:
                        error_msg = str(e)
                        
                        # 연결이 실제로 끊어진 경우
                        if "not open" in error_msg.lower() or "timeout" in error_msg.lower():
                            socketio.emit('log_update', {"message": "컨트롤러 연결이 끊어졌습니다. 다시 연결해주세요.", "color": "red"})
                            
                            # 연결 상태 업데이트
                            with ser_lock:
                                ser = None
                            state = "idle"
                            socketio.emit('connection_status', {'connected': False, 'port': None})
                            socketio.emit('alarm_update', {'text': "알람 상태 확인중", 'color': 'red'})
                            
                            # 연속 실패 카운터 초기화
                            consecutive_failures = 0
            else:
                # 연결이 없거나 가공 중일 때는 카운터 초기화
                consecutive_failures = 0
        
        except Exception as e:
            # 스레드 내부 예외 처리 - 로그만 남기고 계속 실행
            print(f"연결 확인 스레드 내부 오류: {str(e)}")
            time.sleep(check_interval)  # 오류 후 대기

# rework.nc 감시 시작 (서버 프로세스 내부에서 콜백으로 처리, 별도 watcher.py 실행 불필요)
def start_nc_file_watcher():
    observer = start_rework_watcher(os.path.join(BASE_DIR, 'nc'), on_rework_file_ready)
    logging.info("[watcher] rework.nc 감시 시작됨 (백그라운드)")
    return observer

# 라우트 설정 초기화
init_routes(app, socketio, sys.modules[__name__])

# 애플리케이션 실행
if __name__ == '__main__':
    logging.info("CNC 밀링 머신 모니터링 시스템 시작")
    
    # 로그 디렉토리 생성
    if not os.path.exists(os.path.join(BASE_DIR, 'logs')):
        os.makedirs(os.path.join(BASE_DIR, 'logs'))

    # 진동 분석 모델 학습 (첫 가공 시작이 지연되지 않도록 서버 시작 시 백그라운드에서 수행)
    threading.Thread(target=get_vibration_monitor, daemon=True).start()
    
    # 알람 확인 스레드 시작 (백그라운드)
    alarm_thread = threading.Thread(target=check_alarm, daemon=True)
    alarm_thread.start()
    
    # 연결 상태 확인 스레드 시작 (백그라운드)
    connection_thread = threading.Thread(target=check_connection, daemon=True)
    connection_thread.start()
    
    # 아두이노 데이터 읽기 스레드 시작 (백그라운드)
    arduino_thread = threading.Thread(target=read_arduino_serial, daemon=True)
    arduino_thread.start()
    
    # rework.nc 감시 시작 (watchdog Observer가 자체 백그라운드 스레드로 동작)
    start_nc_file_watcher()

    # 시스템 시작 시 사용 가능한 포트 목록 로깅 (연결 시도 없이 목록만 표시)
    available_ports = list_serial_ports()
    logging.info(f"시스템 시작: 사용 가능한 포트 {len(available_ports)}개 발견")
    for idx, port in enumerate(available_ports, 1):
        logging.info(f"  {idx}. {port}")
    
    # Socket.IO 서버 시작
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        logging.info("CNC 밀링 머신 모니터링 시스템 종료")
    except Exception as e:
        logging.error(f"서버 실행 중 오류 발생: {str(e)}")
