from flask import render_template, request, jsonify, send_from_directory
import os
import time
import datetime
import sys
import json
import csv

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rework import create_rework_nc
from gcode_utils import parse_words
import logging

# 전역 변수 선언
gcode_lines = []
tolerance_value = 0
FIXED_SPINDLE = None

# 허용 공차 상한 (mm): 재가공 2차 패스는 원본 Z보다 공차만큼 더 깊게 가공하므로 과절삭·공구 파손을 막기 위해 제한
MAX_TOLERANCE_MM = 1.0
# 수동 재가공(/make_rework, make_rework 이벤트) 시 고정 스핀들 속도 (기존 동작 유지)
MANUAL_REWORK_SPINDLE = 23000
# 클라이언트 명령 → GRBL 1.1 실시간 Spindle Override 바이트
SPINDLE_OVERRIDE_COMMANDS = {
    '0x99': (b'\x99', "Spindle Override Reset"),
    '0x9A': (b'\x9A', "Spindle Override +10%"),
    '0x9B': (b'\x9B', "Spindle Override -10%"),
    '0x9C': (b'\x9C', "Spindle Override +1%"),
    '0x9D': (b'\x9D', "Spindle Override -1%"),
}

# 라우트 및 소켓 이벤트 핸들러를 설정하는 함수
def init_routes(app, socketio, controllers):
    global gcode_module, gcode_lines, tolerance_value, FIXED_SPINDLE
    gcode_module = controllers

    # 기본 웹 라우트
    @app.route('/')
    def index():
        return render_template('index.html')

    @app.route('/favicon.ico')
    def favicon():
        return app.send_static_file('favicon.ico')

    @app.route('/static/<path:path>')
    def send_static(path):
        return send_from_directory(os.path.join(BASE_DIR, 'static'), path)

    # 유틸리티 함수
    def clear_buffer_responses(ser):
        """시리얼 버퍼의 모든 응답을 읽어서 로그로 출력합니다."""
        if ser and ser.in_waiting > 0:
            responses = []
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    responses.append(response)
                    
            if responses:
                socketio.emit('log_update', {"message": f"응답: {', '.join(responses)}", "color": "green"})
            
    # 로그 다운로드 API
    @app.route('/download_logs')
    def download_logs():
        log_dir = os.path.join(BASE_DIR, 'logs')
        if not os.path.exists(log_dir):
            return jsonify({"status": "error", "message": "로그 디렉토리가 없습니다"})
        
        files = os.listdir(log_dir)
        csv_files = [f for f in files if f.endswith('.csv')]
        
        return jsonify({
            "status": "success", 
            "files": csv_files
        })

    @app.route('/download_log/<filename>')
    def download_log(filename):
        log_dir = os.path.join(BASE_DIR, 'logs')
        return send_from_directory(log_dir, filename, as_attachment=True)

    # 파일 업로드 엔드포인트 추가
    @app.route('/upload_csv', methods=['POST'])
    def upload_csv():
        """CSV 파일 업로드 요청을 처리합니다."""
        try:
            if 'csv_file' not in request.files:
                return jsonify({"status": "error", "message": "파일이 선택되지 않았습니다"})
                
            file = request.files['csv_file']
            
            if file.filename == '':
                return jsonify({"status": "error", "message": "파일이 선택되지 않았습니다"})
                
            if file and file.filename.endswith('.csv'):
                # 업로드 디렉토리 확인
                upload_dir = os.path.join(BASE_DIR, 'logs')
                if not os.path.exists(upload_dir):
                    os.makedirs(upload_dir)
                
                # 파일 저장
                filename = f"upload_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                file_path = os.path.join(upload_dir, filename)
                file.save(file_path)
                
                return jsonify({
                    "status": "success", 
                    "message": "파일이 업로드되었습니다",
                    "file_path": file_path
                })
            else:
                return jsonify({"status": "error", "message": "CSV 파일만 업로드할 수 있습니다"})
                
        except Exception as e:
            print(f"파일 업로드 중 오류: {str(e)}")
            return jsonify({"status": "error", "message": f"파일 업로드 중 오류가 발생했습니다: {str(e)}"})

    # 공구 정보 설정
    @socketio.on('set_tool_info')
    def handle_set_tool_info(data):
        """공구 정보를 서버에 저장합니다."""
        try:
            diameter = float(data.get('diameter', 0))
            flutes = int(data.get('flutes', 0))
            
            if diameter <= 0:
                return {"status": "error", "message": "공구 직경은 0보다 커야 합니다."}
                
            if flutes <= 0:
                return {"status": "error", "message": "공구 날 수는 1개 이상이어야 합니다."}
            
            # 컨트롤러에 공구 정보 저장 - 전역 변수나 설정 파일 등에 저장할 수 있음
            # 여기서는 예시만 제공하며, 실제 구현은 프로젝트 요구사항에 맞게 조정 필요
            controllers.tool_diameter = diameter
            controllers.tool_flutes = flutes
            
            # 로그에 기록
            socketio.emit('log_update', {"message": f"공구 정보 설정: 직경 {diameter}mm, 날 수 {flutes}개", "color": "green"})
            
            return {"status": "success", "message": "공구 정보가 저장되었습니다."}
        except Exception as e:
            socketio.emit('log_update', {"message": f"공구 정보 설정 오류: {str(e)}", "color": "red"})
            return {"status": "error", "message": f"공구 정보 설정 중 오류가 발생했습니다: {str(e)}"}

    # SocketIO 이벤트 핸들러
    @socketio.on('get_ports')
    def handle_get_ports(data=None):
        """사용 가능한 포트 목록을 반환합니다."""
        ports = controllers.list_serial_ports()
        return {'ports': ports}

    @socketio.on('connect_port')
    def handle_connect_port(data):
        if not data or 'port' not in data:
            socketio.emit('log_update', {"message": "포트를 선택해주세요", "color": "orange"})
            socketio.emit('connection_status', {'connected': False, 'port': None})
            return {"status": "error", "message": "포트를 선택해주세요"}
        port = data['port']
        connect_result = controllers.connect_serial(port)
        if connect_result['status'] == 'success':
            socketio.emit('log_update', {"message": f"포트 {port} 연결 성공", "color": "green"})
            alarm_status = connect_result.get('alarm_status', 'unknown')
            if alarm_status == 'alarm':
                controllers.state = 'alarm'
                socketio.emit('alarm_update', {'text': "알람 상태: 해제가 필요합니다", 'color': 'red'})
                socketio.emit('status_update', {"text": "알람 상태: 해제 필요", "color": "red"})
            elif alarm_status == 'none':
                controllers.state = 'alarm_unlocked'
                socketio.emit('alarm_update', {'text': "알람 없음", 'color': 'green'})
                socketio.emit('status_update', {"text": "정상 상태", "color": "green"})
            else:
                socketio.emit('alarm_update', {'text': "상태 확인 필요", 'color': 'orange'})
                socketio.emit('status_update', {"text": "상태 확인 필요", "color": "orange"})
            socketio.emit('connection_status', {'connected': True, 'port': port})
        else:
            socketio.emit('log_update', {"message": connect_result['message'], "color": "red"})
            socketio.emit('connection_status', {'connected': False, 'port': None})
        return connect_result

    @socketio.on('connect_sensor')
    def handle_connect_sensor(data):
        """센서 포트 연결 요청을 처리합니다."""
        if not data or 'port' not in data:
            return {"status": "error", "message": "센서 포트를 선택해주세요"}
        
        port = data['port']
        
        # connect_arduino 함수 호출하여 연결 시도
        if controllers.connect_arduino(port):
            return {"status": "success", "message": f"센서가 {port}에 성공적으로 연결되었습니다."}
        else:
            return {"status": "error", "message": f"센서 포트 {port} 연결 실패"}
    
    @socketio.on('disconnect_sensor')
    def handle_disconnect_sensor(data=None):
        """센서 포트 연결 해제 요청을 처리합니다."""
        try:
            if controllers.arduino_serial and controllers.arduino_serial.is_open:
                try:
                    controllers.arduino_serial.close()
                    socketio.emit('log_update', {"message": "센서 연결이 해제되었습니다.", "color": "blue"})
                except Exception as e:
                    error_message = f"센서 연결 해제 중 오류: {str(e)}"
                    socketio.emit('log_update', {"message": error_message, "color": "red"})
                    return {"status": "error", "message": error_message}
                
                # 연결 해제 후 상태 명확하게 초기화
                controllers.arduino_serial = None
                controllers.arduino_connected = False
                controllers.ARDUINO_SERIAL_PORT = None
                
                # 클라이언트에 상태 알림
                socketio.emit('arduino_status', {'connected': False, 'port': None})
                
                return {"status": "success", "message": "센서 연결이 해제되었습니다."}
            else:
                return {"status": "success", "message": "이미 센서 연결이 해제되어 있습니다."}
                
        except Exception as e:
            error_message = f"센서 연결 해제 중 예상치 못한 오류: {str(e)}"
            socketio.emit('log_update', {"message": error_message, "color": "red"})
            return {"status": "error", "message": error_message}

    @socketio.on('disconnect_port')
    def handle_disconnect_port(data=None):
        result = controllers.disconnect_serial()
        if result.get('status') in ['success', 'warning']:
            socketio.emit('log_update', {"message": "포트 연결이 해제되었습니다.", "color": "blue"})
            socketio.emit('connection_status', {'connected': False, 'port': None})
        return result

    @socketio.on('load_nc_file')
    def handle_load_nc_file(data):
        # 가공 중이거나 일시정지, 긴급정지 상태에서는 NC 코드 로드를 허용하지 않음
        if controllers.state in ["running", "paused"] or controllers.job_active:
            error_msg = "가공 중에는 NC 코드를 로드할 수 없습니다. 가공을 중지한 후 시도하세요."
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}
        elif controllers.state == "emergency":
            error_msg = "긴급정지 상태에서는 NC 코드를 로드할 수 없습니다. 긴급정지를 해제한 후 시도하세요."
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}
            
        result = controllers.load_nc_code(data['content'])
        if result['status'] == 'success':
            # 더 자세한 로깅 정보 추가
            file_size = len(data['content'])
            line_count = result['lines']
            current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # 파일명 정보 추가 (전달된 경우)
            file_name = data.get('fileName', 'unknown.nc')
            
            # 로그 메시지 구성
            log_message = f"[{current_time}] NC 코드 로드 '{file_name}': {line_count}줄, {file_size} 바이트"
            socketio.emit('log_update', {"message": log_message, "color": "green"})
            
            # 예상 가공 시간 계산 (controllers에 계산 함수가 있는 경우)
            if hasattr(controllers, 'calculate_estimated_time') and callable(controllers.calculate_estimated_time):
                try:
                    estimated_time = controllers.calculate_estimated_time(controllers.gcode_lines)
                    if estimated_time:
                        time_message = f"예상 가공 시간: {estimated_time['text']}"
                        socketio.emit('log_update', {"message": time_message, "color": "blue"})
                        result['estimated_time'] = estimated_time
                except Exception as e:
                    socketio.emit('log_update', {"message": f"예상 시간 계산 오류: {str(e)}", "color": "orange"})
            
            # G코드 라인을 응답에 포함
            result['gcode_lines'] = controllers.gcode_lines
        else:
            socketio.emit('log_update', {"message": result['message'], "color": "red"})
        return result

    @socketio.on('start_gcode')
    def handle_start_gcode(data=None):
        """가공 시작/재개 처리"""
        try:
            print("\n[DEBUG] handle_start_gcode 함수 시작", file=sys.stderr)
            print(f"[DEBUG] 현재 상태: {controllers.state}", file=sys.stderr)
            
            # 기본 상태 확인
            if not controllers.ser or not controllers.ser.is_open:
                return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."}
            
            if controllers.state == "alarm":
                return {"status": "error", "message": "알람을 먼저 해제해주세요"}
            
            if controllers.state == "emergency":
                return {"status": "error", "message": "긴급정지를 먼저 해제해주세요"}
            
            # 일시정지 상태에서 재개: 중단된 줄부터 이어서 진행 (처음부터 다시 보내지 않음)
            if controllers.state == "paused":
                ok, message = controllers.resume_job()
                if not ok:
                    return {"status": "error", "message": message}
                socketio.emit('log_update', {"message": "가공이 재개되었습니다.", "color": "blue"})
                socketio.emit('status_update', {"text": "가공 재개", "color": "blue"})
                return {"status": "success", "message": message, "resumed": True}

            if controllers.job_active:
                return {"status": "error", "message": "이미 가공이 진행 중입니다."}

            if not controllers.gcode_lines:
                return {"status": "error", "message": "NC 코드가 로드되지 않았습니다."}

            if controllers.state not in ["idle", "alarm_unlocked"]:
                return {"status": "error", "message": "현재 상태에서는 가공을 시작할 수 없습니다."}

            # GRBL 상태 확인
            try:
                status = controllers.query_status(timeout=0.5)
            except Exception as e:
                return {"status": "error", "message": f"GRBL 상태 확인 중 오류: {str(e)}"}
            if status and status['state'] == "Alarm":
                return {"status": "error", "message": "GRBL이 알람 상태입니다. 알람을 해제해주세요."}
            if status and status['state'] in ("Run", "Hold", "Jog", "Door"):
                return {"status": "error", "message": f"GRBL이 아직 동작 중이거나 일시정지 상태입니다 ({status['state']})."}

            # 새로운 가공 시작 (스트리밍 스레드 시작)
            print("[DEBUG] 가공 시작 조건 충족 (idle 또는 alarm_unlocked 상태)", file=sys.stderr)
            ok, message = controllers.start_job()
            if not ok:
                return {"status": "error", "message": message}
            socketio.emit('log_update', {"message": "가공을 시작합니다.", "color": "blue"})
            socketio.emit('status_update', {"text": "가공 시작", "color": "blue"})
            socketio.emit('disable_nc_upload', {})
            return {"status": "success", "message": message}

        except Exception as e:
            error_msg = f"가공 시작 중 오류 발생: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}

    @socketio.on('toggle_pause')
    def handle_toggle_pause(data=None):
        """일시정지 처리 (피드 홀드 후 스트리밍 스레드는 응답을 읽으며 재개를 기다림)"""
        try:
            if not controllers.ser or not controllers.ser.is_open:
                return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."}

            # 피드 홀드 명령 전송, 일시정지 상태로 변경, 진동 탐지 중지
            ok, message = controllers.pause_job()
            if not ok:
                return {"status": "error", "message": message}

            # 일시정지 메시지 표시
            reason = (data or {}).get('reason', '가공이 일시정지되었습니다.')
            socketio.emit('log_update', {"message": reason, "color": "orange"})
            socketio.emit('status_update', {"text": "일시정지", "color": "orange"})
            return {"status": "success", "message": reason}

        except Exception as e:
            error_msg = f"일시정지 처리 중 오류 발생: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}

    @socketio.on('emergency_stop')
    def handle_emergency_stop(data=None):
        """긴급정지 처리"""
        try:
            if not controllers.ser or not controllers.ser.is_open:
                return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."}

            if controllers.state == "emergency":
                # 긴급정지 해제: 스트리밍 스레드 종료를 기다린 뒤 소프트 리셋(CTRL+X) 후 상태 확인
                deadline = time.time() + 3.0
                while controllers.job_active and time.time() < deadline:
                    time.sleep(0.05)
                with controllers.ser_lock:
                    controllers.ser.reset_input_buffer()
                    controllers.ser.reset_output_buffer()
                    controllers.ser.write(b"\x18")  # Ctrl+X (소프트 리셋)
                    controllers.ser.flush()
                time.sleep(1.0)  # 리셋 후 GRBL 초기화 대기

                # 알람 상태 확인 (리셋 직후 출력되는 시작 메시지가 아니라 상태 보고로 판단)
                status = controllers.query_status()
                alarm_detected = bool(status and status['state'] == "Alarm")

                controllers.state = "alarm" if alarm_detected else "idle"
                controllers.pause = False
                controllers.notify_state()

                if alarm_detected:
                    socketio.emit('log_update', {"message": "긴급정지 해제 완료! 알람 상태가 감지되었습니다.", "color": "orange"})
                    socketio.emit('status_update', {"text": "알람 상태: 해제 필요", "color": "orange"})
                    socketio.emit('alarm_update', {'text': "알람 상태: 해제 필요", 'color': 'red'})
                    socketio.emit('enable_nc_upload', {})
                    return {"status": "success", "message": "긴급정지가 해제되었습니다. 알람 해제가 필요합니다.", "button_text": "긴급정지"}
                else:
                    socketio.emit('log_update', {"message": "긴급정지 해제 완료! 정상 상태로 돌아왔습니다.", "color": "green"})
                    socketio.emit('status_update', {"text": "대기 상태", "color": "green"})
                    socketio.emit('alarm_update', {'text': "알람 없음", 'color': 'green'})
                    socketio.emit('enable_nc_upload', {})
                    return {"status": "success", "message": "긴급정지가 해제되었습니다", "button_text": "긴급정지"}

            else:
                # 긴급정지 활성화: G코드 전송 중단 + 소프트 리셋(CTRL+X)으로 스핀들/이송 즉시 정지
                # (M5 같은 일반 명령은 GRBL 버퍼에 쌓인 이동 뒤에 실행되므로 즉시 정지에는 소프트 리셋을 사용)
                controllers.emergency_halt()

                socketio.emit('log_update', {"message": "긴급정지 실행! 소프트 리셋(CTRL+X)으로 스핀들/이송을 즉시 정지했습니다.", "color": "red"})
                socketio.emit('status_update', {"text": "긴급정지", "color": "red"})
                socketio.emit('alarm_update', {'text': "긴급정지 상태", 'color': 'red'})
                socketio.emit('disable_nc_upload', {})

                # 경로 및 상태 즉시 반영
                controllers.update_ui()

                return {"status": "success", "message": "긴급정지가 활성화되었습니다", "button_text": "긴급정지 해제"}

        except Exception as e:
            error_msg = f"긴급정지 처리 중 오류 발생: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}

    @socketio.on('unlock_alarm')
    def handle_unlock_alarm(data=None):
        """알람 해제 처리"""
        try:
            if not controllers.ser or not controllers.ser.is_open:
                return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."}
            if controllers.job_active:
                return {"status": "error", "message": "가공 중에는 알람을 해제할 수 없습니다."}

            with controllers.ser_lock:
                # 알람 상태 확인 (실시간 명령 '?'는 개행 없이 전송해야 추가 'ok' 응답이 생기지 않음)
                controllers.ser.reset_input_buffer()
                controllers.ser.write(b"?")
                controllers.ser.flush()
                time.sleep(0.5)

                alarm_state = False
                if controllers.ser.in_waiting > 0:
                    response = controllers.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "ALARM" in response.upper():
                        alarm_state = True

                if not alarm_state and controllers.state != "alarm":
                    socketio.emit('log_update', {"message": "알람 상태가 아닙니다.", "color": "green"})
                    socketio.emit('alarm_update', {'text': "알람 없음", 'color': 'green'})
                    socketio.emit('status_update', {"text": "정상 상태", "color": "green"})
                    socketio.emit('enable_nc_upload', {})
                    return {"status": "success", "message": "알람 상태가 아닙니다."}

                # 알람 해제 명령 전송
                controllers.ser.reset_input_buffer()
                controllers.ser.write(b"$X\n")
                controllers.ser.flush()
                time.sleep(0.5)

                # 응답 확인
                unlock_success = False
                start_time = time.time()
                while time.time() - start_time < 1.0:
                    if controllers.ser.in_waiting > 0:
                        response = controllers.ser.readline().decode('utf-8', errors='ignore').strip()
                        if "ok" in response.lower():
                            unlock_success = True
                            break
                    time.sleep(0.05)

            if unlock_success:
                controllers.state = "alarm_unlocked"
                controllers.notify_state()
                socketio.emit('log_update', {"message": "알람 해제 성공: 가공 시작 가능", "color": "green"})
                socketio.emit('alarm_update', {'text': "알람 해제됨", 'color': 'green'})
                socketio.emit('status_update', {"text": "가공 준비 완료", "color": "green"})
                socketio.emit('enable_nc_upload', {})
                return {"status": "success", "message": "알람이 해제되었습니다. 가공을 시작할 수 있습니다."}
            else:
                socketio.emit('log_update', {"message": "알람 해제 실패", "color": "red"})
                socketio.emit('alarm_update', {'text': "알람 해제 실패", 'color': 'red'})
                socketio.emit('status_update', {"text": "알람 상태 계속됨", "color": "red"})
                return {"status": "error", "message": "알람 해제에 실패했습니다."}

        except Exception as e:
            error_msg = f"알람 해제 중 오류 발생: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}

    @socketio.on('jog')
    def handle_jog(data):
        try:
            dx = float(data.get('dx', 0))
            dy = float(data.get('dy', 0))
            dz = float(data.get('dz', 0))
            feed = data.get('feed', None)
            
            # 가공 중에 G91/G90을 끼워 넣으면 진행 중인 프로그램의 좌표 모드가 바뀌므로 금지
            if controllers.job_active:
                return {"status": "error", "message": "가공 중에는 Jog 이동을 할 수 없습니다."}

            # 그랩 모드 전환 및 이동
            if controllers.ser and controllers.ser.is_open:
                with controllers.ser_lock:
                    # 재시도 관련 변수
                    max_retries = 3
                    retry_count = 0
                    command_sent = False
                
                    # 버퍼 초기화
                    controllers.ser.reset_input_buffer()
                    controllers.ser.reset_output_buffer()
                
                    # 상대 좌표계 모드로 전환
                    while not command_sent and retry_count < max_retries:
                        try:
                            controllers.ser.write(b"G91\n")
                            time.sleep(0.3)  # 좀 더 길게 대기
                            command_sent = True
                        except Exception:
                            retry_count += 1
                            socketio.emit('log_update', {"message": f"Jog 모드 전환 중 시간 초과. 재시도 중... ({retry_count}/{max_retries})"})
                            time.sleep(1)
                            if retry_count >= max_retries:
                                raise Exception("G91 모드 전환 시간 초과")
                
                    # 명령어 재설정 및 재시도 초기화
                    retry_count = 0
                    command_sent = False
                
                    # 이동 명령 전송
                    while not command_sent and retry_count < max_retries:
                        try:
                            # 빌드 G-code 명령
                            cmd = "G1"
                            if dx:
                                cmd += f" X{dx}"
                            if dy:
                                cmd += f" Y{dy}"
                            if dz:
                                cmd += f" Z{dz}"
                            if feed is not None:
                                cmd += f" F{feed}"
                            cmd += "\n"
                        
                            # 명령 전송 및 대기
                            controllers.ser.write(cmd.encode())
                            time.sleep(0.5)  # 충분한 처리 시간 부여
                            command_sent = True
                        except Exception:
                            retry_count += 1
                            socketio.emit('log_update', {"message": f"Jog 이동 명령 전송 중 시간 초과. 재시도 중... ({retry_count}/{max_retries})"})
                            time.sleep(1)
                            if retry_count >= max_retries:
                                raise Exception("이동 명령 전송 시간 초과")
                
                    # 절대 좌표계 모드로 돌아가기
                    retry_count = 0
                    command_sent = False
                    while not command_sent and retry_count < max_retries:
                        try:
                            # 응답을 기다리지 않고 전송만
                            controllers.ser.write(b"G90\n")
                            time.sleep(0.3)
                            command_sent = True
                        except Exception:
                            retry_count += 1
                            socketio.emit('log_update', {"message": f"Jog 모드 복귀 중 시간 초과. 재시도 중... ({retry_count}/{max_retries})"})
                            time.sleep(1)
                            if retry_count >= max_retries:
                                # 이 오류는 무시 가능 (G90으로 복귀 실패해도 다음 명령에서 설정 가능)
                                socketio.emit('log_update', {"message": "좌표계 모드 복귀 실패. 다음 명령에서 재설정됩니다."})
                
                    # 위치 업데이트
                    new_x = controllers.x_vals[-1] + dx
                    new_y = controllers.y_vals[-1] + dy
                    new_z = controllers.z_vals[-1] + dz
                    controllers.x_vals.append(new_x)
                    controllers.y_vals.append(new_y)
                    controllers.z_vals.append(new_z)
                
                    # UI 업데이트
                    controllers.update_ui()
                    return {"status": "success", "message": "Jog 이동 완료"}
            else:
                return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다"}
        except Exception as e:
            socketio.emit('alarm_update', {'text': f"Jog 이동 중 오류가 발생했습니다: {str(e)}", 'color': 'red'})
            return {"status": "error", "message": str(e)} 

    @socketio.on('check_alarm_status')
    def handle_check_alarm_status(data=None):
        """GRBL 컨트롤러의 알람 상태를 확인합니다."""
        if not controllers.ser or not controllers.ser.is_open:
            socketio.emit('alarm_update', {'text': "연결되지 않음", 'color': 'orange'})
            socketio.emit('status_update', {"text": "컨트롤러 연결 필요", "color": "orange"})
            return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다", "alarm": False}
        
        # 가공 중에는 스트리밍 스레드만 시리얼을 읽으므로 (버퍼를 비우면 'ok' 응답이 사라짐) 현재 상태만 반환
        if controllers.job_active:
            return {"status": "success", "alarm": controllers.state == "alarm", "message": "", "state": controllers.state}

        try:
            responses = []
            with controllers.ser_lock:
                # 기존 버퍼 비우기
                controllers.ser.reset_input_buffer()
                # 상태 확인 명령 전송 (실시간 명령이므로 개행 없이 전송)
                controllers.ser.write(b"?")
                controllers.ser.flush()  # 버퍼 즉시 전송
                # 응답 여러 줄 모두 읽기 (최대 0.8초 대기)
                start_time = time.time()
                while time.time() - start_time < 0.8:
                    while controllers.ser.in_waiting > 0:
                        response = controllers.ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            responses.append(response)
                    time.sleep(0.05)
            
            # 응답 확인
            alarm_state = False
            status_message = ""
            
            # 여러 줄 중 알람 상태 확인
            for response in responses:
                if "ALARM" in response.upper() or "ERROR" in response.upper():
                    alarm_state = True
                    status_message = response
                    break
                elif "<" in response and ">" in response:  # 상태 응답
                    status_message = response
            
            # 알람 상태 업데이트
            if alarm_state:
                # 알람 상태
                controllers.state = "alarm"
                # 알람 메시지는 최대 50자로 제한
                short_msg = status_message[:50] + ("..." if len(status_message) > 50 else "")
                socketio.emit('alarm_update', {'text': f"알람 상태: {short_msg}", 'color': 'red'})
                socketio.emit('status_update', {"text": "알람 상태: 해제 필요", "color": "red"})
                return {"status": "success", "alarm": True, "message": status_message, "state": "alarm"}
            else:
                # 알람이 없는 경우 - 각 상태에 따라 다른 정보 반환
                if controllers.state == "running":
                    # 가공 중 상태
                    socketio.emit('alarm_update', {'text': "알람 없음: 가공 중", 'color': 'green'})
                    socketio.emit('status_update', {"text": "가공 중", "color": "green"})
                    return {"status": "success", "alarm": False, "message": status_message, "state": "running"}
                elif controllers.state == "paused":
                    # 일시정지 상태
                    socketio.emit('alarm_update', {'text': "알람 없음: 일시정지 상태", 'color': 'orange'})
                    socketio.emit('status_update', {"text": "일시정지", "color": "orange"})
                    return {"status": "success", "alarm": False, "message": status_message, "state": "paused"}
                elif controllers.state == "emergency":
                    # 긴급정지 상태
                    socketio.emit('alarm_update', {'text': "긴급정지 상태", 'color': 'red'})
                    socketio.emit('status_update', {"text": "긴급정지: 알람 해제 필요", "color": "red"})
                    return {"status": "success", "alarm": False, "message": status_message, "state": "emergency"}
                elif controllers.state == "alarm_unlocked":
                    # 알람 해제 상태 (바로 가공 가능)
                    socketio.emit('alarm_update', {'text': "알람 해제됨: 가공 준비 완료", 'color': 'green'})
                    socketio.emit('status_update', {"text": "가공 준비 완료", "color": "green"})
                    return {"status": "success", "alarm": False, "message": status_message, "state": "alarm_unlocked"}
                else:
                    # 일반 준비 상태
                    if controllers.state == "idle":
                        # 알람이 없는 대기 상태면 가공 가능 상태로 표시
                        controllers.state = "alarm_unlocked"
                    
                    socketio.emit('alarm_update', {'text': "알람 없음", 'color': 'green'})
                    socketio.emit('status_update', {"text": "준비 완료", "color": "green"})
                    return {"status": "success", "alarm": False, "message": status_message, "state": "alarm_unlocked"}
        
        except Exception as e:
            socketio.emit('log_update', {"message": f"알람 상태 확인 중 오류: {str(e)}", "color": "red"})
            return {"status": "error", "message": f"오류: {str(e)}", "alarm": False}

    @app.route('/api/set-zero', methods=['POST'])
    def api_set_zero():
        axis = request.form.get('axis', 'xyz')  # 'xy', 'z', 'xyz'
        try:
            if not hasattr(gcode_module, 'ser') or not gcode_module.ser or not gcode_module.ser.is_open:
                return jsonify({"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."})
            if gcode_module.job_active:
                return jsonify({"status": "error", "message": "가공 중에는 원점을 설정할 수 없습니다."})

            # 현재 좌표
            x = gcode_module.x_vals[-1] if gcode_module.x_vals else 0
            y = gcode_module.y_vals[-1] if gcode_module.y_vals else 0
            z = gcode_module.z_vals[-1] if gcode_module.z_vals else 0

            # 오프셋 명령
            if axis == 'xy':
                cmd = 'G10 P1 L20 X0 Y0\nG54\n'
                gcode_module.x_vals.append(0)
                gcode_module.y_vals.append(0)
                gcode_module.z_vals.append(z)
            elif axis == 'z':
                cmd = 'G10 P1 L20 Z0\nG54\n'
                gcode_module.x_vals.append(x)
                gcode_module.y_vals.append(y)
                gcode_module.z_vals.append(0)
            else:
                cmd = 'G10 P1 L20 X0 Y0 Z0\nG54\n'
                gcode_module.x_vals.append(0)
                gcode_module.y_vals.append(0)
                gcode_module.z_vals.append(0)
            # GRBL로 전송
            with gcode_module.ser_lock:
                gcode_module.ser.write(cmd.encode())
                gcode_module.ser.flush()
            print(f"[G10] 명령 전송됨: {cmd.strip()}")
            gcode_module.update_ui()

            return jsonify({
                "status": "success",
                "message": f"현재 위치를 {axis.upper()} 원점(G10 P1 L20 {axis.upper()}0)으로 설정했습니다."
            })
        except Exception as e:
            return jsonify({
                "status": "error",
                "message": f"G10 명령 전송 실패: {str(e)}"
            })

    @socketio.on('request_status')
    def handle_request_status(data=None):
        """
        웹소켓 재연결(예: F5 새로고침) 시 서버의 최신 CNC 상태, 경로, 포트, 알람, 로그, 센서 정보를 모두 클라이언트로 전송합니다.
        """
        try:
            # 포트 상태
            port_status = {
                'connected': hasattr(gcode_module, 'ser') and gcode_module.ser and hasattr(gcode_module.ser, 'is_open') and gcode_module.ser.is_open,
                'port': getattr(gcode_module, 'ser', None).port if hasattr(gcode_module, 'ser') and gcode_module.ser else None
            }

            # 센서 포트 상태
            sensor_status = {
                'connected': getattr(gcode_module, 'arduino_connected', False),
                'port': getattr(gcode_module, 'ARDUINO_SERIAL_PORT', None),
                'latest_data': getattr(gcode_module, 'latest_arduino_data', None)
            }

            # 알람 상태
            alarm_state = gcode_module.state if hasattr(gcode_module, 'state') else 'unknown'
            alarm_message = getattr(gcode_module, 'last_alarm_message', '')

            # Override 상태
            override_status = {
                'spindle': getattr(gcode_module, 'spindle_override', 100),
                'feed': getattr(gcode_module, 'feed_override', 100)
            }

            # 시스템 로그 (최근 50개만)
            logs = []
            if hasattr(gcode_module, 'system_logs'):
                logs = gcode_module.system_logs[-50:]
            # 만약 별도 로그 리스트가 없다면, 로그를 따로 관리해야 함

            # CNC 경로 및 상태 (재접속한 클라이언트를 위해 전체 경로 전송)
            gcode_module.update_ui(full=True)
            gcode_module.notify_state(force=True)
            # 허용 공차 (클라이언트 입력창을 서버 값과 일치시킴)
            socketio.emit('tolerance_update', {'tolerance': gcode_module.tolerance_value})

            # 포트 상태
            socketio.emit('connection_status', port_status)
            # 센서 포트 상태
            socketio.emit('arduino_status', sensor_status)
            # 알람 상태
            socketio.emit('alarm_update', {'text': f"상태: {alarm_state} {alarm_message}", 'color': 'green' if alarm_state in ['idle', 'alarm_unlocked'] else 'red'})
            # Override 상태
            socketio.emit('override_status', override_status)
            # 시스템 로그
            if logs:
                for log in logs:
                    socketio.emit('log_update', log if isinstance(log, dict) else {"message": str(log), "color": "gray"})
            # 상태 표시
            socketio.emit('status_update', {"text": gcode_module.state, "color": "blue" if gcode_module.state == "running" else "green"})

            # request_status 핸들러 내
            nc_uploaded = bool(getattr(gcode_module, 'gcode_lines', []))
            is_processing = gcode_module.state in ['running', 'paused']

            socketio.emit('nc_upload_status', {
                'uploaded': nc_uploaded,
                'disable': is_processing
            })

            return {"status": "success"}
        except Exception as e:
            socketio.emit('log_update', {"message": f"상태 동기화 오류: {str(e)}", "color": "red"})
            return {"status": "error", "message": str(e)}

    @socketio.on('grbl_command')
    def handle_grbl_command(data):
        """클라이언트에서 입력한 GRBL 명령을 시리얼로 전송하고 결과를 로그로 반환"""
        try:
            cmd = data.get('command', '').strip()
            if not cmd:
                socketio.emit('log_update', {"message": "명령어를 입력하세요.", "color": "orange"})
                return
            if not hasattr(gcode_module, 'ser') or not gcode_module.ser or not gcode_module.ser.is_open:
                socketio.emit('log_update', {"message": "시리얼 포트가 연결되어 있지 않습니다.", "color": "red"})
                return
            # 가공 중 임의 명령을 끼워 넣으면 스트리밍 중인 프로그램과 응답 순서가 섞이므로 금지
            if gcode_module.job_active:
                socketio.emit('log_update', {"message": "가공 중에는 GRBL 명령을 직접 보낼 수 없습니다. 일시정지/긴급정지 버튼을 사용하세요.", "color": "red"})
                return
            responses = []
            with gcode_module.ser_lock:
                # 명령 전송
                gcode_module.ser.write((cmd + '\n').encode())
                gcode_module.ser.flush()
                socketio.emit('log_update', {"message": f"> {cmd}", "color": "blue"})
                # 응답 읽기 (최대 2초 대기)
                start_time = time.time()
                while time.time() - start_time < 2.0:
                    if gcode_module.ser.in_waiting > 0:
                        resp = gcode_module.ser.readline().decode('utf-8', errors='ignore').strip()
                        if resp:
                            responses.append(resp)
                    else:
                        time.sleep(0.05)
            if responses:
                for resp in responses:
                    socketio.emit('log_update', {"message": resp, "color": "green" if "ok" in resp.lower() else "red"})
            else:
                socketio.emit('log_update', {"message": "응답 없음", "color": "orange"})
        except Exception as e:
            socketio.emit('log_update', {"message": f"명령 처리 오류: {str(e)}", "color": "red"})

    @socketio.on('spindle_override_command')
    def handle_spindle_override_command(data):
            ser = getattr(controllers, 'ser', None)
            cmd = (data or {}).get('cmd')

            if not ser or not ser.is_open:
                socketio.emit('log_update', {"message": "CNC 컨트롤러가 연결되어 있지 않습니다.", "color": "red"})
                return

            try:
                if cmd not in SPINDLE_OVERRIDE_COMMANDS:
                    socketio.emit('log_update', {"message": f"알 수 없는 Spindle Override 명령: {cmd}", "color": "orange"})
                    return
                command, label = SPINDLE_OVERRIDE_COMMANDS[cmd]
                with controllers.ser_lock:
                    ser.write(command)
                    ser.flush()
                socketio.emit('log_update', {"message": f"{label} 명령({cmd}) 전송", "color": "green"})

                # 가공 중에는 스트리밍 스레드가 상태 보고(Ov)로 오버라이드 표시를 갱신하므로 여기서는 읽지 않음
                if controllers.job_active:
                    return

                # GRBL 상태 확인 후 실제 스핀들 RPM(FS)과 오버라이드 비율(Ov) 반영
                time.sleep(0.3)
                status = controllers.query_status()
                if status:
                    if 'ov' in status:
                        controllers.feed_override, controllers.spindle_override = status['ov'][0], status['ov'][2]
                        socketio.emit('override_status', {'spindle': controllers.spindle_override, 'feed': controllers.feed_override})
                    if 'spindle' in status:
                        socketio.emit("spindle_rpm_update", {"rpm": int(status['spindle']), "override": controllers.spindle_override})
            except Exception as e:
                socketio.emit('log_update', {"message": f"Spindle Override 명령 전송 오류: {str(e)}", "color": "red"})

    @socketio.on('set_feed_override')
    def handle_set_feed_override(data):
        """Feed Override 값을 설정합니다."""
        try:
            value = int(data.get('value', 100))
            result = controllers.apply_override("feed", value)
            if result['status'] == 'success':
                socketio.emit('log_update', {"message": result['message'], "color": "green"})
            else:
                socketio.emit('log_update', {"message": result['message'], "color": "red"})
            return result
        except Exception as e:
            error_msg = f"Feed Override 설정 중 오류: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}

    # 허용 공차 값을 받는 라우트
    @app.route('/set_tolerance', methods=['POST'])
    def set_tolerance():
        global tolerance_value
        try:
            new_tolerance = float(request.form['tolerance'])
            # 2차 재가공 깊이 = 원본 Z - 공차 이므로 큰 값은 과절삭/공구 파손으로 이어짐 (NaN도 여기서 거부됨)
            if not (0.0 <= new_tolerance <= MAX_TOLERANCE_MM):
                return jsonify({
                    'status': 'error',
                    'message': f'허용 공차는 0 ~ {MAX_TOLERANCE_MM}mm 범위로 입력하세요. (입력값: {new_tolerance}mm)'
                })
            tolerance_value = new_tolerance  # 전역 변수 업데이트
            controllers.tolerance_value = new_tolerance  # controllers 객체 업데이트
            socketio.emit('tolerance_update', {'tolerance': new_tolerance})
            print("서버에 tolerance_value 갱신:", tolerance_value)
            print("controllers.tolerance_value 갱신:", controllers.tolerance_value)
            print("현재 gcode_lines:", controllers.gcode_lines if controllers.gcode_lines else "비어있음")
            return jsonify({
                'status': 'success',
                'tolerance': tolerance_value,
                'message': f'허용 공차가 {tolerance_value}mm로 설정되었습니다.'
            })
        except Exception as e:
            print("tolerance_value 설정 중 오류:", str(e))
            return jsonify({
                'status': 'error',
                'message': str(e)
            })
    
    def create_rework_file():
        """rework.nc 파일을 생성하는 내부 함수 (수동 요청: 최적화 없이 스핀들 속도만 고정값으로 변경)

        생성된 파일은 자동 가공되지 않습니다 (자동 재가공은 이상 진동 복구 과정에서 만든 파일만 허용).
        """
        try:
            source_lines = controllers.rework_source_lines or controllers.gcode_lines
            if not source_lines:
                return {'status': 'error', 'message': 'G-code 데이터가 없습니다.'}

            # Vibration_detection.csv 파일 존재 여부 확인
            if not os.path.exists(os.path.join(BASE_DIR, 'data', 'Vibration_detection.csv')):
                error_msg = "Vibration_detection.csv 파일이 존재하지 않아 rework.nc를 생성할 수 없습니다."
                logging.error(error_msg)
                return {'status': 'error', 'message': error_msg}

            # 현재 절삭 조건 (가공 이력이 없으면 NC 코드의 첫 F 값 사용)
            current_n, current_F = controllers.effective_cutting_conditions()
            if current_F is None:
                current_F = next((value for line in source_lines for letter, value in parse_words(line) if letter == 'F' and value > 0), None)
            if current_F is None:
                return {'status': 'error', 'message': '이송 속도(F) 정보를 찾을 수 없어 rework.nc를 생성할 수 없습니다.'}

            # rework.nc 파일 생성 - 스핀들 속도를 고정값으로, 이송 속도는 현재 값 유지
            success = create_rework_nc(
                current_n=current_n or MANUAL_REWORK_SPINDLE,
                current_F=current_F,
                optimal_n=MANUAL_REWORK_SPINDLE,
                optimal_F=current_F,
                x_pos=controllers.x_vals[-1] if controllers.x_vals else 0.0,
                y_pos=controllers.y_vals[-1] if controllers.y_vals else 0.0,
                z_pos=controllers.z_vals[-1] if controllers.z_vals else 0.0,
                gcode_lines=source_lines,
                tolerance_value=controllers.tolerance_value
            )

            if success:
                logging.info("rework.nc 파일이 생성되었습니다.")
                return {'status': 'success', 'message': 'rework.nc 파일이 생성되었습니다.'}
            else:
                error_msg = "rework.nc 파일 생성 실패"
                logging.error(error_msg)
                return {'status': 'error', 'message': error_msg}
        except Exception as e:
            error_msg = f"rework.nc 생성 중 오류: {str(e)}"
            logging.error(error_msg)
            return {'status': 'error', 'message': error_msg}

    @app.route('/make_rework', methods=['POST'])
    def make_rework():
        """HTTP 엔드포인트: rework.nc 파일 생성"""
        print("[DEBUG] make_rework 호출됨")
        result = create_rework_file()
        return jsonify(result)

    @socketio.on('make_rework')
    def handle_make_rework(data=None):
        """Socket.IO 이벤트: rework.nc 파일 생성"""
        return create_rework_file()

    @socketio.on('reset_path')
    def handle_reset_path(data=None):
        """경로 초기화 이벤트 처리"""
        try:
            if controllers.job_active:
                return {'status': 'error', 'message': '가공 중에는 경로를 초기화할 수 없습니다.'}

            # 현재 위치 초기화
            controllers.reset_path()
            controllers.progress = 0
            
            # UI 업데이트
            controllers.update_ui(full=True)
            socketio.emit('progress_update', {"progress": controllers.progress})
            
            logging.info("경로가 초기화되었습니다.")
            return {'status': 'success', 'message': '경로가 초기화되었습니다.'}
        except Exception as e:
            logging.error(f"경로 초기화 중 오류: {str(e)}")
            return {'status': 'error', 'message': str(e)}

    @socketio.on('load_rework_file')
    def handle_load_rework_file(data=None):
        """rework.nc 파일을 로드하고 가공을 시작합니다.

        state를 미리 "running"으로 바꾸지 않고 start_job()에 맡겨, 재가공에서도
        진동 탐지·CSV 로깅이 정상 시작 경로로 켜지도록 합니다.
        """
        try:
            ok, message = controllers.start_rework_job()
            return {"status": "success" if ok else "error", "message": message}
            
        except Exception as e:
            error_msg = f"rework.nc 처리 중 오류: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}
