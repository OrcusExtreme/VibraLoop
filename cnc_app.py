import serial
import serial.tools.list_ports
import threading
import time
import os
import json
import sys
import csv
import datetime
import math
import re
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if os.path.join(BASE_DIR, 'src') not in sys.path:
    sys.path.insert(0, os.path.join(BASE_DIR, 'src'))

from routes import init_routes
import subprocess
from rework import create_rework_nc
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest
from vibration_monitor import VibrationMonitor
import logging
import requests
from watcher import FileHandler  # watcher.py의 FileHandler 임포트

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

# 위치 및 경로 데이터
x_vals, y_vals, z_vals = [0], [0], [0]
offset_x, offset_y, offset_z = 0, 0, 0

# 가공 파라미터
current_spindle = None
current_feedrate = None
gcode_lines = []

# 시리얼 연결
ser = None

# CSV 로깅 관련 변수
csv_file = None
csv_writer = None
process_start_time = None
log_interval = 0.0002  # 로깅 간격 (0.0002초 = 200 마이크로초)
last_log_time = 0

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

# 진동 분석기 관련 변수
vibration_monitor = None  # VibrationMonitor 인스턴스를 저장할 변수
vibration_detection_active = False  # 진동 탐지 활성화 상태 변수

# 전역 변수 추가
last_spindle_adjustment_time = 0
MIN_SPINDLE_ADJUSTMENT_INTERVAL = 2.0  # 스핀들 속도 조정 간 최소 대기 시간 (초)

def start_vibration_detection():
    """진동 탐지를 시작하는 함수"""
    global vibration_detection_active, vibration_monitor, state
    
    logger.info("[vibration] 진동 탐지 시작")
    vibration_detection_active = True
    
    # 진동 모니터 초기화
    if vibration_monitor is None:
        # controllers를 전달하여 VibrationMonitor 인스턴스 생성
        vibration_monitor = VibrationMonitor(socketio, controllers, gcode_lines, tolerance_value)
    
    if vibration_monitor is not None and state == "running":
        vibration_monitor.is_running = True # 진동 모니터링 시작

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

# 포트 연결 (컨트롤러 연결)
def connect_serial(port):
    """GRBL 컨트롤러에 시리얼 포트 연결"""
    global ser, state
    
    if not port:
        return {"status": "error", "message": "포트를 선택해주세요"}
    
    try:
        # 기존 연결 해제
        disconnect_serial()
        
        # 데이터 처리를 위한 변수 초기화
        attempts = 0
        max_attempts = 10  # 최대 10번 시도
        connected = False
        alarm_status = "unknown"  # 기본적으로 상태 알 수 없음으로 설정
        
        # GRBL 컨트롤러에 연결 시도 (115200 baud로 시도)
        try:
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
            
            # 상태 확인 명령 전송
            ser.write(b"?\n")
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

# 아두이노 센서 데이터 읽기 함수
def read_arduino_serial():
    """아두이노 시리얼 연결을 통해 센서 데이터를 주기적으로 읽는 함수"""
    global latest_arduino_data, arduino_serial, arduino_connected, ARDUINO_SERIAL_PORT
    global vibration_monitor  # VibrationMonitor 인스턴스

    while True:
        # 연결이 활성화된 경우에만 데이터 읽기 시도
        if arduino_serial and arduino_serial.is_open:
            try:
                # 데이터 읽기
                if arduino_serial.in_waiting > 0:
                    line = arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # 데이터가 JSON 형식인지 확인
                        try:
                            # JSON 형식인 경우 파싱
                            data = json.loads(line)

                            # 단순 {"x":..,"y":..,"z":..} 형식 지원
                            if 'accel' not in data and all(axis in data for axis in ('x', 'y', 'z')):
                                accel_data = {
                                    'x': float(data.get('x', 0)),
                                    'y': float(data.get('y', 0)),
                                    'z': float(data.get('z', 0))
                                }
                                gyro_data = data.get('gyro')
                                if not isinstance(gyro_data, dict):
                                    gyro_data = {'x': 0, 'y': 0, 'z': 0}
                                data = {
                                    'accel': accel_data,
                                    'gyro': {
                                        'x': float(gyro_data.get('x', 0)),
                                        'y': float(gyro_data.get('y', 0)),
                                        'z': float(gyro_data.get('z', 0))
                                    }
                                }
                            latest_arduino_data = line
                            
                            # 센서 데이터를 웹 클라이언트에 전송
                            socketio.emit('arduino_data', {'data': latest_arduino_data})
                            
                            # 로그창에도 표시
                            try:
                                # 가속도계와 자이로스코프 데이터 추출
                                acc_data = data.get('accel', {})
                                gyro_data = data.get('gyro', {})
                                
                                # 가속도계와 자이로스코프 값 추출
                                acc_x = acc_data.get('x', 0)
                                acc_y = acc_data.get('y', 0)
                                acc_z = acc_data.get('z', 0)
                                
                                gyro_x = gyro_data.get('x', 0)
                                gyro_y = gyro_data.get('y', 0)
                                gyro_z = gyro_data.get('z', 0)
                                
                                # 로그 메시지 구성
                                log_message = f"[센서] ACC(X:{acc_x:.3f}, Y:{acc_y:.3f}, Z:{acc_z:.3f}) GYRO(X:{gyro_x:.3f}, Y:{gyro_y:.3f}, Z:{gyro_z:.3f})"
                                socketio.emit('log_update', {"message": log_message, "color": "sensor"})

                                # 진동 분석을 위한 데이터 처리
                                if vibration_monitor is not None:
                                    sensor_data = [[acc_x, acc_y, acc_z]]  # 2D 배열로 변환
                                    vibration_monitor.process_sensor_data(sensor_data)
                                    logging.debug(f"[vibration] 센서 데이터 처리: X={acc_x:.3f}, Y={acc_y:.3f}, Z={acc_z:.3f}")

                            except Exception as e:
                                # JSON 형식이 아니거나 다른 구조인 경우 기본 형식으로 출력
                                log_message = f"[센서] {latest_arduino_data[:100]}"
                                socketio.emit('log_update', {"message": log_message, "color": "sensor"})
                        except json.JSONDecodeError:
                            # JSON이 아닌 경우 텍스트 형식 파싱 시도 (예: "x: 5319 y: 5824 z: 2801")
                            text_match = re.match(r'x:\s*(\d+)\s+y:\s*(\d+)\s+z:\s*(\d+)', line, re.IGNORECASE)
                            if text_match:
                                # 텍스트 형식을 JSON으로 변환
                                x = float(text_match.group(1))
                                y = float(text_match.group(2))
                                z = float(text_match.group(3))
                                # JSON 형식으로 변환
                                data = {
                                    "accel": {"x": x, "y": y, "z": z},
                                    "gyro": {"x": 0, "y": 0, "z": 0}
                                }
                                latest_arduino_data = json.dumps(data)
                                socketio.emit('arduino_data', {'data': latest_arduino_data})
                                socketio.emit('log_update', {"message": f"[센서] x: {int(x)} y: {int(y)} z: {int(z)}", "color": "sensor"})
                                
                                # 진동 분석을 위한 데이터 처리
                                if vibration_monitor is not None:
                                    sensor_data = [[x, y, z]]  # 2D 배열로 변환
                                    vibration_monitor.process_sensor_data(sensor_data)
                            else:
                                # 파싱할 수 없는 형식인 경우 그대로 표시
                                latest_arduino_data = line
                                socketio.emit('arduino_data', {'data': latest_arduino_data})
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
            time.sleep(5)

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
    
# NC 코드 파일을 불러오는 함수
def load_nc_code(file_content):
    """NC 코드 파일 내용을 파싱하고 가공 경로를 초기화합니다."""
    global gcode_lines, offset_x, offset_y, offset_z
    # GRBL에서 지원하지 않는 명령어 목록 (필요시 추가)
    unsupported_cmds = ['M6', 'M0', 'M1', 'M60', 'M6T', 'M7', 'M9', 'M48', 'M49', 'M30']
    try:
        raw_lines = file_content.split('\n')
        gcode_lines = []
        for line_num, line in enumerate(raw_lines, 1):
            cleaned_line = line.strip()
            # 빈 줄, 주석, 한글 포함, 비표준 문자 포함 라인 무시
            if (not cleaned_line or
                cleaned_line.startswith(';') or
                cleaned_line.startswith('(') or
                re.search(r'[ㄱ-ㅎ가-힣]', cleaned_line)):
                continue
            # G28 또는 G30 복귀 명령을 GRBL 호환 G0 X0 Y0 Z0으로 변환
            if cleaned_line.upper().startswith('G28') or cleaned_line.upper().startswith('G30'):
                gcode_lines.append('G0 X0 Y0 Z0')
                continue
            # 지원하지 않는 명령어로 시작하는 라인 무시
            if any(cleaned_line.upper().startswith(cmd) for cmd in unsupported_cmds):
                continue
            gcode_lines.append(cleaned_line)
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
    """NC 코드의 이동 거리와 피드 속도를 분석하여 예상 가공 시간을 계산합니다."""
    total_time_min = 0  # 총 예상 시간 (분)
    current_x, current_y, current_z = 0, 0, 0
    current_feedrate = None  # 초기 이송 속도는 설정하지 않음
    rapid_feedrate = 5000  # 급속 이송 속도 (mm/min)
    
    for line in gcode_lines:
        x, y, z = None, None, None
        feedrate_in_line = None
        is_rapid = False
        
        # G0/G00은 급속 이송, G1/G01은 일반 이송
        if line.startswith(('G0', 'G00')):
            is_rapid = True
        
        # 좌표와 이송 속도 파싱
        for part in line.split():
            if part.startswith('X'): x = float(part[1:])
            elif part.startswith('Y'): y = float(part[1:])
            elif part.startswith('Z'): z = float(part[1:])
            elif part.startswith('F'): feedrate_in_line = float(part[1:])
        
        # 현재 라인에 F 코드가 있으면 이송 속도 업데이트
        if feedrate_in_line is not None:
            current_feedrate = feedrate_in_line
        
        # 이송 속도가 아직 설정되지 않았고, 이동 명령이면 기본값 설정
        if current_feedrate is None and (x is not None or y is not None or z is not None):
            # 처음 이송 속도가 설정되지 않은 경우, NC 코드를 검색하여 첫 F 값 찾기
            for check_line in gcode_lines:
                f_match = re.search(r'F(\d+(\.\d+)?)', check_line)
                if f_match:
                    current_feedrate = float(f_match.group(1))
                    break
            
            # 그래도 없으면 안전한 기본값 설정
            if current_feedrate is None:
                current_feedrate = 500  # 안전한 낮은 값으로 설정
        
        # 거리 계산 및 시간 추가
        if x is not None or y is not None or z is not None:
            next_x = x if x is not None else current_x
            next_y = y if y is not None else current_y
            next_z = z if z is not None else current_z
            
            # 3차원 거리 계산
            distance = ((next_x - current_x)**2 + 
                        (next_y - current_y)**2 + 
                        (next_z - current_z)**2)**0.5
            
            # 급속 이송이거나 이송 속도가 설정되지 않은 경우
            if is_rapid:
                move_time_min = distance / rapid_feedrate
            elif current_feedrate:
                move_time_min = distance / current_feedrate
            else:
                # 이송 속도가 없는 경우 (비정상)
                move_time_min = distance / 500  # 안전한 기본값 사용
            
            # 총 시간에 현재 이동 시간 추가
            total_time_min += move_time_min
            
            # 좌표 업데이트
            current_x, current_y, current_z = next_x, next_y, next_z
    
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
    global csv_file, csv_writer, process_start_time
    
    # 로그 디렉토리 생성
    log_dir = os.path.join(BASE_DIR, 'logs')
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # 파일명 생성 (현재 시간 기반)
    current_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"{log_dir}/cnc_log_{current_time}.csv"
    
    # CSV 파일 열기
    csv_file = open(filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # 헤더 작성 - 클라이언트 저장 방식과 동일하게 설정
    csv_writer.writerow(['X', 'Y', 'Z', 'ProcessTime'])
    
    # 프로세스 시작 시간 기록
    if process_start_time is None:
        process_start_time = time.time()
    
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
def log_process_data(custom_time=None):
    global last_log_time, csv_writer, process_start_time, state, auto_monitoring, auto_recovery

    if not csv_writer:
        return

    # custom_time이 있으면 그 값을, 없으면 실제 시간 사용
    if custom_time is not None:
        elapsed_sec_rounded = int(round(custom_time))
    else:
        current_time = time.time()
        elapsed_sec = current_time - process_start_time if process_start_time else 0
        elapsed_sec_rounded = int(round(elapsed_sec))

    current_x = x_vals[-1] if x_vals else 0
    current_y = y_vals[-1] if y_vals else 0
    current_z = z_vals[-1] if z_vals else 0

    csv_writer.writerow([
        f"{current_x:.4f}",
        f"{current_y:.4f}",
        f"{current_z:.4f}",
        elapsed_sec_rounded
    ])
    
    # 마지막 로깅 시간 업데이트
    if custom_time is None:
        last_log_time = current_time
    else:
        # 보간 경과시간 기반일 때는 실제 시간 기록하지 않음 또는 아래처럼 처리
        last_log_time = process_start_time + custom_time if process_start_time else None

# 알람 상태 체크 (주기적)
def check_alarm():
    """주기적으로 CNC 컨트롤러의 알람 상태를 확인합니다."""
    global ser, state, last_alarm_check, last_alarm_message, alarm_retry_count
    
    while True:
        # 현재 시간 기록
        current_time = time.time()
        
        # 시리얼 연결 상태와 현재 가공 상태 확인
        has_connection = ser and hasattr(ser, 'is_open') and ser.is_open
        is_processing = (state == "running")
        
        # 일정 간격으로만 알람 확인 (너무 자주 확인하면 통신 부하 발생)
        if has_connection and not is_processing and (current_time - last_alarm_check) >= alarm_check_interval:
            last_alarm_check = current_time
            
            try:
                # 이전에 입력 버퍼에 남아있는 데이터가 있으면 읽어서 처리
                if ser.in_waiting > 0:
                    # 버퍼 데이터 모두 읽기
                    while ser.in_waiting > 0:
                        buffer_data = ser.readline().decode('utf-8', errors='ignore').strip()
                        if buffer_data and ("ERROR" in buffer_data.upper() or "ALARM" in buffer_data.upper()):
                            # 버퍼에서 알람 상태 발견
                            if last_alarm_message != buffer_data:
                                last_alarm_message = buffer_data
                                state = "alarm"
                
                # 버퍼 내용 처리 후 입력 버퍼 비우기 (중요: 상태 쿼리 전에 버퍼 정리)
                ser.reset_input_buffer()
                
                # 상태 확인 명령 전송 (효율적인 하나의 명령만 사용)
                ser.write(b"?\n")
                
                # 응답 대기
                start_time = time.time()
                max_wait = 0.8  # 응답 대기 시간 (800ms로 충분)
                
                # 응답 대기 (제한된 시간 내에)
                response_received = False
                response_text = ""
                while time.time() - start_time < max_wait:
                    if ser.in_waiting > 0:
                        # 한 줄씩 읽어서 처리
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:  # 빈 응답 무시
                            response_received = True
                            response_text = response
                            
                            # 알람 상태인 경우
                            if "ALARM" in response.upper() or "ERROR" in response.upper():
                                if state != "alarm":
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
                
                # 응답이 없는 경우
                if not response_received:
                    alarm_retry_count += 1
                    
                    # 연속적으로 응답이 없을 때만 경고 표시 (로그만 표시하고 UI는 업데이트하지 않음)
                    if alarm_retry_count >= max_alarm_retries:
                        # 사용자 혼란 방지를 위해 로그는 한 번만 표시
                        if state != "connection_warning":
                            state = "connection_warning"
                
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
            
        # 알람 해제 명령 전송
        socketio.emit('log_update', {"message": "알람 해제 명령 전송 중...", "color": "blue"})
        
        # 버퍼 초기화 - 중요: 명확하게 이전 버퍼 데이터를 제거
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # 상태 확인 명령 먼저 전송하여 현재 상태 확인
        ser.write(b"?\n")
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
                ser.write(b"?\n")
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

# === 가공 진행 인덱스 전역 변수 추가 ===
gcode_current_idx = 0

# G코드 명령 전송 및 실행
def send_gcode():
    """G코드를 시리얼로 전송하는 함수"""
    global state, pause, progress, gcode_lines, current_spindle, current_feedrate, base_spindle, base_feedrate, vibration_detection_active, vibration_monitor, gcode_current_idx
    
    print("\n[DEBUG] send_gcode 함수 시작", file=sys.stderr)
    print(f"[DEBUG] 현재 상태: {state}", file=sys.stderr)
    
    if not ser or not ser.is_open:
        socketio.emit('log_update', {"message": "시리얼 포트가 연결되지 않았습니다.", "color": "red"})
        return

    # G-code에서 최초 스핀들 속도 및 이송 속도 파싱 또는 기본값 설정
    initial_spindle_found = False
    initial_feedrate_found = False
    for line in gcode_lines:
        s_match = re.search(r'S(\d+(\.\d+)?)', line)
        if s_match and not initial_spindle_found:
            try:
                current_spindle = float(s_match.group(1))
                initial_spindle_found = True
            except ValueError: pass

        f_match = re.search(r'F(\d+(\.\d+)?)', line)
        if f_match and not initial_feedrate_found:
            try:
                current_feedrate = float(f_match.group(1))
                initial_feedrate_found = True
            except ValueError: pass

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
    gcode_current_idx = 0  # G코드 인덱스 초기화
    
    if not ser or not ser.is_open:
        socketio.emit('log_update', {"message": "시리얼 포트가 연결되지 않았습니다.", "color": "red"})
        return

    # === 긴급정지 후 가공 재시작 시 항상 처음부터 시작 ===
    if state in ["idle", "emergency", "alarm_unlocked"]:
        print("[DEBUG] 가공 시작 조건 충족 (idle, emergency 또는 alarm_unlocked 상태)", file=sys.stderr)
        # 상태 초기화
        state = "running"
        pause = False
        progress = 0
        gcode_current_idx = 0
        # 진동 탐지 활성화
        if not vibration_detection_active:
            print("[DEBUG] 진동 탐지 활성화 시도", file=sys.stderr)
            start_vibration_detection()
        
        # CSV 로깅 시작
        start_csv_logging()
        process_start_time = time.time()
        
        # 웹 인터페이스 업데이트
        socketio.emit('log_update', {"message": "가공이 시작되었습니다.", "color": "green"})
        socketio.emit('state_update', {"state": state})
        socketio.emit('progress_update', {"progress": progress})
        # CNC 위치 읽기(유지)
        ser.reset_input_buffer()
        ser.write(b"?\n")
        ser.flush()
        time.sleep(0.2)
        actual_x, actual_y, actual_z = 0, 0, 0
        if ser.in_waiting > 0:
            status_line = ser.readline().decode('utf-8', errors='ignore').strip()
            mpos_match = re.search(r'MPos:([-\d\.]+),([-\d\.]+),([-\d\.]+)', status_line)
            if mpos_match:
                actual_x = float(mpos_match.group(1))
                actual_y = float(mpos_match.group(2))
                actual_z = float(mpos_match.group(3))
                with open(os.path.join(BASE_DIR, 'data', 'current_position.txt'), 'w') as f:
                    f.write(str(actual_z))
        # CNC 위치와 무관하게 UI 좌표는 0으로 초기화
        x_vals[:] = [0]
        y_vals[:] = [0]
        z_vals[:] = [0]
        cur_x = 0
        cur_y = 0
        cur_z = 0
    else:
        # 일시정지 후 재개만 이어서
        state = "running"
        pause = False
        start_csv_logging()
        # process_start_time = time.time()
        cur_x = x_vals[-1] if x_vals else 0
        cur_y = y_vals[-1] if y_vals else 0
        cur_z = z_vals[-1] if z_vals else 0
        socketio.emit('log_update', {"message": f"가공 재개: 현재 위치 X{cur_x} Y{cur_y} Z{cur_z}", "color": "blue"})
        try:
            ser.reset_input_buffer()
            ser.write(b"~\n")
            ser.flush()
            time.sleep(0.1)
            ser.write(b"?\n")
            ser.flush()
            time.sleep(0.1)
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        if "error" in response.lower() or "alarm" in response.lower():
                            socketio.emit('log_update', {"message": f"GRBL 오류 상태: {response}", "color": "red"})
                            return
                        elif "idle" in response.lower() or "run" in response.lower():
                            socketio.emit('log_update', {"message": "GRBL 정상 상태 확인", "color": "green"})
                            break
                time.sleep(0.01)
        except Exception as e:
            socketio.emit('log_update', {"message": f"GRBL 상태 확인 실패: {str(e)}", "color": "red"})
            return

    default_feedrate = 1000 # mm/min
    if not current_feedrate:
        current_feedrate = default_feedrate

    socketio.emit('log_update', {"message": f"가공 시작: 총 {len(gcode_lines)}개 명령 실행 예정", "color": "blue"})

    GRBL_BUFFER_SIZE = 128
    idx = gcode_current_idx
    pending_lines = []
    buffer_used = 0
    
    # gcode_lines가 비어있는지 확인
    if not gcode_lines:
        socketio.emit('log_update', {"message": "가공할 G코드가 없습니다.", "color": "red"})
        return
        
    # gcode_line_lengths 초기화
    gcode_line_lengths = [len(line) + 1 for line in gcode_lines]  # +1 for '\n'

    elapsed_time_accum = 0

    while idx < len(gcode_lines) or pending_lines:
        # 긴급정지 또는 중지 상태 확인
        if state in ["emergency", "stopped"]:
            socketio.emit('log_update', {"message": "가공이 중단되었습니다.", "color": "red"})
            stop_csv_logging()
            # 진동 탐지 중지
            if vibration_detection_active:
                stop_vibration_detection()
            gcode_current_idx = idx
            break

        # 일시정지 상태 확인 및 처리
        if pause and state == "paused":
            # 현재 실행 중인 명령어 완료 대기
            if pending_lines:
                sent_idx, sent_line, sent_len = pending_lines[0]
                start_resp_time = time.time()
                while time.time() - start_resp_time < 5.0:  # 5초 타임아웃
                    if ser.in_waiting > 0:
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if response and "ok" in response.lower():
                            pending_lines.pop(0)
                            buffer_used -= sent_len
                            break
                    time.sleep(0.01)
            # 입력 버퍼만 초기화
            ser.reset_input_buffer()
            pending_lines.clear()
            buffer_used = 0
            gcode_current_idx = idx
            stop_csv_logging()
            time.sleep(0.1)
            return  # 일시정지 시 함수 종료

        # --- 긴급정지 즉시 중단 로직 추가 ---
        if state == "emergency":
            socketio.emit('log_update', {"message": "긴급정지: 가공이 즉시 중단되었습니다.", "color": "red"})
            stop_csv_logging()
            gcode_current_idx = idx
            break
        # -----------------------------------

        # 버퍼가 다 차기 전에 한 줄씩 순차적으로 명령 전송
        while idx < len(gcode_lines) and idx < len(gcode_line_lengths):
            line = gcode_lines[idx]
            line_len = gcode_line_lengths[idx]
            # 버퍼가 충분히 비어있을 때만 전송 (여유 공간 16바이트 이상)
            if buffer_used + line_len <= GRBL_BUFFER_SIZE - 16:
                # 명령 전송 직전에만 버퍼 초기화
                if idx == 0 and ser.in_waiting > 0:
                    ser.reset_input_buffer()
                ser.write(f"{line}\n".encode())
                ser.flush()
                pending_lines.append((idx, line, line_len))
                buffer_used += line_len
                idx += 1
            else:
                break

        # ok 응답이 올 때까지 기다림
        if pending_lines:
            sent_idx, sent_line, sent_len = pending_lines.pop(0)
            start_resp_time = time.time()
            resp_timeout = 5.0  # 타임아웃 시간 조정
            response = ""
            for attempt in range(3):  # 최대 3회 재시도
                while time.time() - start_resp_time < resp_timeout:
                    if ser.in_waiting > 0:
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            if "ok" in response.lower():
                                socketio.emit('log_update', {"message": f"명령 {sent_line} -> 응답: {response}", "color": "green"})
                                break
                            elif "error" in response.lower():
                                socketio.emit('log_update', {"message": f"명령 {sent_line} -> 오류: {response}", "color": "red"})
                                # 오류 발생 시 재시도 직전에만 버퍼 초기화
                                ser.reset_input_buffer()
                                ser.write(f"{sent_line}\n".encode())
                                ser.flush()
                                pending_lines.insert(0, (sent_idx, sent_line, sent_len))
                                buffer_used += sent_len
                                time.sleep(0.1)
                                break
                            else:
                                socketio.emit('log_update', {"message": f"명령 {sent_line} -> 응답: {response}", "color": "orange"})
                                break
                    time.sleep(0.01)
                if response and ("ok" in response.lower() or "error" in response.lower()):
                    break
                # 응답 없음 시 재시도 (여기서는 버퍼 초기화 생략)
                socketio.emit('log_update', {"message": f"명령 {sent_line} -> 응답 없음, 재시도 {attempt+1}/3", "color": "orange"})
                ser.write(f"{sent_line}\n".encode())
                ser.flush()
                time.sleep(0.1)
            buffer_used -= sent_len

            # === 이동 경로 세분화 및 좌표 처리 ===
            # G코드 명령 파싱
            x, y, z, f, s = None, None, None, None, None
            for part in sent_line.split():
                if part.startswith('X'):
                    try: x = float(part[1:]) - offset_x
                    except: pass
                elif part.startswith('Y'):
                    try: y = float(part[1:]) - offset_y
                    except: pass
                elif part.startswith('Z'):
                    try: z = float(part[1:]) - offset_z
                    except: pass
                elif part.startswith('F'):
                    try:
                        f = float(part[1:])
                        current_feedrate = f
                    except:
                        current_feedrate = default_feedrate
                elif part.startswith('S'):
                    try:
                        s = float(part[1:])
                        current_spindle = s
                    except:
                        pass

            # 목적지 좌표 계산 (명령에 없는 축은 이전 값 유지)
            next_x = x if x is not None else cur_x
            next_y = y if y is not None else cur_y
            next_z = z if z is not None else cur_z

            # 이동 명령(G0, G1)만 경로 보간
            is_move = sent_line.strip().startswith(('G0', 'G1', 'G00', 'G01'))
            if is_move and (cur_x != next_x or cur_y != next_y or cur_z != next_z):
                distance = math.sqrt((next_x - cur_x) ** 2 + (next_y - cur_y) ** 2 + (next_z - cur_z) ** 2)
                steps = int(max(distance, 1))
                feedrate = current_feedrate if current_feedrate else default_feedrate  # mm/min
                feedrate_mm_per_sec = feedrate / 60.0  # mm/sec
                step_distance = distance / steps if steps > 0 else 0
                step_time = step_distance / feedrate_mm_per_sec if feedrate_mm_per_sec > 0 else 0

                for step in range(1, steps + 1):
                    # --- 긴급정지 즉시 중단 로직 추가 ---
                    if state == "emergency":
                        socketio.emit('log_update', {"message": "긴급정지: 가공이 즉시 중단되었습니다.", "color": "red"})
                        stop_csv_logging()
                        gcode_current_idx = sent_idx
                        return
                    # -----------------------------------
                    if pause and state == "paused":
                        gcode_current_idx = sent_idx
                        stop_csv_logging()
                        return
                    
                    interp_x = cur_x + (next_x - cur_x) * step / steps
                    interp_y = cur_y + (next_y - cur_y) * step / steps
                    interp_z = cur_z + (next_z - cur_z) * step / steps
                    x_vals.append(interp_x)
                    y_vals.append(interp_y)
                    z_vals.append(interp_z)
                    elapsed_time_accum += step_time
                    log_process_data(custom_time=elapsed_time_accum)
                    update_ui()  # 각 보간 좌표마다 차트 갱신
                    if step_time > 0:
                        time.sleep(step_time)  # 실제 이동 속도에 맞춰 대기

                if not (pause and state == "paused"):
                    cur_x, cur_y, cur_z = next_x, next_y, next_z
            else:
                x_vals.append(cur_x)
                y_vals.append(cur_y)
                z_vals.append(cur_z)
                log_process_data(custom_time=elapsed_time_accum)


            progress = int((sent_idx + 1) / len(gcode_lines) * 100)
            update_ui()

            if sent_idx % 10 == 0 or sent_idx < 5:
                socketio.emit('log_update', {"message": f"{sent_line} -> 위치 갱신", "color": "gray"})

            if (sent_idx + 1) % max(1, len(gcode_lines) // 10) == 0:
                percentage = int((sent_idx + 1) / len(gcode_lines) * 100)
                socketio.emit('log_update', {"message": f"가공 진행률: {percentage}%", "color": "blue"})

        gcode_current_idx = idx  # 현재 인덱스 저장

    if state == "running":
        state = "idle"
        gcode_current_idx = 0
        stop_csv_logging()
        socketio.emit('status_update', {"text": "가공 완료", "color": "blue"})
        socketio.emit('log_update', {"message": "가공이 성공적으로 완료되었습니다.", "color": "green"})
        socketio.emit('enable_nc_upload', {})
        
        # 진동 탐지 중지
        stop_vibration_detection()
        
        # 가공 완료 후 깊이 조정 및 재가공
        # adjust_depth_and_restart()

  

def set_workpiece_zero(axis='xyz'):
    """현재 위치를 G10 P1 L20 명령으로 진짜 원점으로 설정"""
    global ser, x_vals, y_vals, z_vals
    if not ser or not ser.is_open:
        return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."}
    try:
        if axis == 'xy':
            cmd = "G10 P1 L20 X0 Y0\n"
        elif axis == 'z':
            cmd = "G10 P1 L20 Z0\n"
        else:
            cmd = "G10 P1 L20 X0 Y0 Z0\n"
        ser.write(cmd.encode())
        ser.flush()
        time.sleep(0.2)
        return {"status": "success", "message": f"현재 위치를 {axis.upper()} 원점(G10 P1 L20 {axis.upper()}0)으로 설정했습니다."}
    except Exception as e:
        return {"status": "error", "message": f"G10 명령 전송 실패: {str(e)}"}

# UI 업데이트 함수
def update_ui():
    """현재 가공 상태를 UI에 업데이트합니다."""
    global process_start_time
    
    # 현재 경과 시간 계산 (프로세스가 시작된 경우에만)
    elapsed_time = 0
    if process_start_time:
        elapsed_time = time.time() - process_start_time
    
    # 긴급정지 상태에서는 경로 데이터를 전송하지 않음
    socketio.emit('position_update', {
        'x': x_vals[-1],
        'y': y_vals[-1],
        'z': z_vals[-1],
        'path_x': x_vals,
        'path_y': y_vals,
        'path_z': z_vals,
        'spindle': current_spindle,
        'feedrate': current_feedrate,
        'progress': progress,
        'elapsed_time': elapsed_time  # 경과 시간 추가
    })


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
            
            # 시리얼 연결이 있고, 열려있고, 가공 중이 아닐 때만 체크
            if ser and hasattr(ser, 'is_open') and ser.is_open and state != "running":
                try:
                    # 버퍼에 남아 있는 데이터가 있으면 처리 (로그 남기지 않음)
                    if ser.in_waiting > 10:
                        ser.reset_input_buffer()
                    
                    # 간단한 상태 확인 명령
                    ser.write(b"?\n")
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
                                    
                                    # 알람 상태 확인
                                    if "ERROR" in response.upper() or "ALARM" in response.upper():
                                        if state != "alarm":
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

# 이상 감지 NC 코드 파트
class NCFileWatcher(FileSystemEventHandler):
    def __init__(self):
        # 필요한 경우 초기화 로직 추가
        pass

    def on_created(self, event):
        # 파일 생성 이벤트 처리
        # is_directory 체크를 추가하여 폴더 생성 이벤트는 무시합니다.
        if not event.is_directory and os.path.basename(event.src_path) == "rework.nc":
            print(f"[watcher] rework.nc 파일이 생성됨: {event.src_path}", file=sys.stderr)
            # 파일이 완전히 작성될 때까지 잠시 대기합니다.
            # 파일 크기가 클 경우 이 시간을 늘려야 할 수 있습니다.
            time.sleep(0.5)
            self.handle_nc_file(event.src_path)

    def handle_nc_file(self, file_path):
        global gcode_lines, state, pause, progress, x_vals, y_vals, z_vals

        try:
            # 1. 가공 중이면 즉시 멈춤
            
            # 2. 시뮬레이션 경로 및 진행률 초기화
            x_vals[:] = [0]
            y_vals[:] = [0]
            z_vals[:] = [0]
            progress = 0
            update_ui()  # UI에 즉시 반영

            # 3. rework.nc 파일 읽고 가공 시작 (기존 코드 유지)
            with open(file_path, "r", encoding='cp949') as file:
                nc_code_content = file.read()

            print(f"[watcher] rework.nc 파일 내용 읽기 완료", file=sys.stderr)
            load_result = load_nc_code(nc_code_content)

            if load_result and load_result.get("status") == "success":
                print(f"[watcher] NC 코드 로드 완료. 총 {len(gcode_lines)} 라인.", file=sys.stderr)
                socketio.emit('log_update', {"message": "자동 복구 NC 코드 로드 완료. 가공을 자동으로 시작합니다.", "color": "green"})

                machining_thread = threading.Thread(target=send_gcode, daemon=True)
                machining_thread.start()
                print("[watcher] 자동 가공 시작 스레드 실행됨", file=sys.stderr)
            else:
                socketio.emit('log_update', {"message": "자동 복구 NC 코드 로드 실패.", "color": "red"})
                print("[watcher] NC 코드 로드 실패.", file=sys.stderr)

        except FileNotFoundError:
            print(f"[watcher] 오류: rework.nc 파일을 찾을 수 없습니다: {file_path}", file=sys.stderr)
            socketio.emit('log_update', {"message": f"오류: rework.nc 파일을 찾을 수 없습니다: {file_path}", "color": "red"})
        except Exception as e:
            print(f"[watcher] rework.nc 처리 중 오류: {e}", file=sys.stderr)
            socketio.emit('log_update', {"message": f"rework.nc 처리 중 오류: {str(e)}", "color": "red"})

# 가공 완료 후 Z축 깊이 조정 및 재가공을 위한 함수 추가
def adjust_depth_and_restart():
    global state, ser, tolerance_value, gcode_lines, vibration_detection_active
    
    try:
        # 현재 Z축 위치 확인
        ser.reset_input_buffer()
        ser.write(b"?\n")
        ser.flush()
        time.sleep(0.2)
        
        current_z = 0
        if ser.in_waiting > 0:
            status_line = ser.readline().decode('utf-8', errors='ignore').strip()
            mpos_match = re.search(r'MPos:([-\d\.]+),([-\d\.]+),([-\d\.]+)', status_line)
            if mpos_match:
                current_z = float(mpos_match.group(3))
        
        # Z축으로 허용 공차만큼 더 들어가기
        new_z = current_z - tolerance_value
        z_move_cmd = f"G0 Z{new_z}\n"
        
        print(f"[depth_adjust] Z축 깊이 조정: {current_z} -> {new_z}", file=sys.stderr)
        socketio.emit('log_update', {"message": f"Z축 깊이를 {new_z}mm로 조정합니다.", "color": "blue"})
        
        # Z축 이동 명령 전송
        ser.write(z_move_cmd.encode())
        ser.flush()
        time.sleep(0.5)  # 이동 완료 대기
        
        # 가공 재시작
        state = "running"
        socketio.emit('log_update', {"message": "새로운 깊이로 가공을 재시작합니다.", "color": "green"})
        
        # 가공 스레드 시작
        machining_thread = threading.Thread(target=send_gcode, daemon=True)
        machining_thread.start()
        
    except Exception as e:
        print(f"[depth_adjust] 깊이 조정 중 오류: {e}", file=sys.stderr)
        socketio.emit('log_update', {"message": f"깊이 조정 중 오류: {str(e)}", "color": "red"})



def check_vibration(vibration_data, tolerance_value, new_z=None):
    """진동 데이터를 확인하고 재가공 NC 파일 생성"""
    global FIXED_SPINDLE
    
    try:
        # 가공 중이 아닐 때는 진동 탐지하지 않음
        if state != "running":
            return False
            
        # 진동 데이터 분석
        if vibration_monitor and vibration_monitor.analyze_realtime_data(vibration_data):
            logger.info("[vibration] 이상 진동 감지됨")
            
            # 진동 알림 표시 (가공 중일 때만)
            if state == "running":
                socketio.emit('vibration_alert', {
                    'show': True,
                    'override': spindle_override
                })
            return True
        else:
            print("[vibration] 정상 진동")
            # 정상 진동 시 FIXED_SPINDLE에 현재 스핀들 값 저장
            FIXED_SPINDLE = current_spindle
            print(f"[vibration] 정상 진동 감지, FIXED_SPINDLE 저장: {FIXED_SPINDLE}")
            # VibrationMonitor에 정상 스핀들 속도 설정
            if vibration_monitor:
                vibration_monitor.set_fixed_spindle(FIXED_SPINDLE)
            # 정상 진동 시 알림 숨기기
            socketio.emit('vibration_alert', {
                'show': False
            })
            return False
    except Exception as e:
        print(f"[vibration] 진동 확인 중 오류 발생: {str(e)}")
        return False

# NC 파일 감시 스레드를 시작하는 함수
def start_nc_file_watcher():
    event_handler = FileHandler()  # FileHandler 사용
    observer = Observer()
    observer.schedule(event_handler, path=BASE_DIR, recursive=True)
    observer.start()
    logging.info("[watcher] NC 파일 감시 시작됨 (백그라운드)")

# 이전의 중복된 rework.nc 생성 요청 제거
# try:
#     requests.post('http://localhost:5000/make_rework')
# except Exception as e:
#     print("rework 생성 요청 실패:", e)

tolerance_value = 0

# 라우트 설정 초기화
init_routes(app, socketio, sys.modules[__name__])

# 애플리케이션 실행
if __name__ == '__main__':
    logging.info("CNC 밀링 머신 모니터링 시스템 시작")
    
    # 로그 디렉토리 생성
    if not os.path.exists(os.path.join(BASE_DIR, 'logs')):
        os.makedirs(os.path.join(BASE_DIR, 'logs'))
    
    # 알람 확인 스레드 시작 (백그라운드)
    alarm_thread = threading.Thread(target=check_alarm, daemon=True)
    alarm_thread.start()
    
    # 연결 상태 확인 스레드 시작 (백그라운드)
    connection_thread = threading.Thread(target=check_connection, daemon=True)
    connection_thread.start()
    
    # 아두이노 데이터 읽기 스레드 시작 (백그라운드)
    arduino_thread = threading.Thread(target=read_arduino_serial, daemon=True)
    arduino_thread.start()
    
    # NC 파일 감시 스레드 시작 (백그라운드)
    nc_file_watcher_thread = threading.Thread(target=start_nc_file_watcher, daemon=True)
    nc_file_watcher_thread.start()
    logging.info("[watcher] NC 파일 감시 스레드 시작됨")

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




    