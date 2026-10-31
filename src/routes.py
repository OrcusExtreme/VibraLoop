from flask import render_template, request, jsonify, send_from_directory
import os
import threading
import time
import datetime
import sys
import json
import csv

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rework import create_rework_nc
import requests
import logging

# 전역 변수 선언
gcode_lines = []
tolerance_value = 0
FIXED_SPINDLE = None

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
        if controllers.state in ["running", "paused"]:
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
            
            controllers.gcode_current_idx = 0  # 반드시 0으로!
            # 기본 상태 확인
            if not controllers.ser or not controllers.ser.is_open:
                return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."}
            
            if controllers.state == "alarm":
                return {"status": "error", "message": "알람을 먼저 해제해주세요"}
            
            if controllers.state == "emergency":
                return {"status": "error", "message": "긴급정지를 먼저 해제해주세요"}
            
            if not controllers.gcode_lines:
                return {"status": "error", "message": "NC 코드가 로드되지 않았습니다."}

            # GRBL 상태 확인
            try:
                controllers.ser.reset_input_buffer()
                controllers.ser.write(b"?\n")
                controllers.ser.flush()
                time.sleep(0.3)

                response = controllers.ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    if "ALARM" in response.upper():
                        return {"status": "error", "message": "GRBL이 알람 상태입니다. 알람을 해제해주세요."}
                    elif "ERROR" in response.upper():
                        return {"status": "error", "message": f"GRBL 오류: {response}"}

            except Exception as e:
                return {"status": "error", "message": f"GRBL 상태 확인 중 오류: {str(e)}"}

            # 가공 시작/재개 처리
            if controllers.state == "paused":
                # 일시정지 상태에서 재개
                controllers.pause = False
                controllers.state = "running"
                socketio.emit('log_update', {"message": "가공이 재개되었습니다.", "color": "blue"})
                socketio.emit('status_update', {"text": "가공 재개", "color": "blue"})
            elif controllers.state in ["idle", "alarm_unlocked"]:
                print("[DEBUG] 가공 시작 조건 충족 (idle 또는 alarm_unlocked 상태)", file=sys.stderr)
                # 새로운 가공 시작
                controllers.pause = False
                controllers.x_vals = [0]
                controllers.y_vals = [0]
                controllers.z_vals = [0]
                controllers.progress = 0
                socketio.emit('log_update', {"message": "가공을 시작합니다.", "color": "blue"})
                socketio.emit('status_update', {"text": "가공 시작", "color": "blue"})
            else:
                return {"status": "error", "message": "현재 상태에서는 가공을 시작할 수 없습니다."}

            # 가공 스레드 시작
            print("[DEBUG] 가공 스레드 시작", file=sys.stderr)
            socketio.emit('disable_nc_upload', {})
            threading.Thread(target=controllers.send_gcode, daemon=True).start()
            return {"status": "success", "message": "가공이 시작되었습니다."}

        except Exception as e:
            error_msg = f"가공 시작 중 오류 발생: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}

    @socketio.on('toggle_pause')
    def handle_toggle_pause(data=None):
        """일시정지 처리"""
        try:
            if not controllers.ser or not controllers.ser.is_open:
                return {"status": "error", "message": "시리얼 포트가 연결되어 있지 않습니다."}

            if controllers.state != "running":
                return {"status": "error", "message": "가공 중이 아닐 때는 일시정지를 할 수 없습니다."}

            # 피드 홀드 명령 전송
            controllers.ser.write(b"!")
            controllers.ser.flush()
            time.sleep(0.1)

            # 일시정지 상태로 변경
            controllers.pause = True
            controllers.state = "paused"
            
            # 진동 탐지 중지
            if controllers.vibration_detection_active:
                controllers.stop_vibration_detection()
                socketio.emit('log_update', {"message": "진동 탐지가 중지되었습니다.", "color": "blue"})

            # 일시정지 메시지 표시
            reason = data.get('reason', '가공이 일시정지되었습니다.')
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
                # 긴급정지 해제: 소프트 리셋(CTRL+X) 후 상태 확인
                controllers.ser.reset_input_buffer()
                controllers.ser.reset_output_buffer()
                controllers.ser.write(b"\x18")  # Ctrl+X (소프트 리셋)
                controllers.ser.flush()
                time.sleep(0.3)

                # 알람 상태 확인
                controllers.ser.write(b"?\n")
                controllers.ser.flush()
                time.sleep(0.2)

                alarm_detected = False
                if controllers.ser.in_waiting > 0:
                    response = controllers.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "ALARM" in response.upper():
                        alarm_detected = True

                controllers.state = "alarm" if alarm_detected else "idle"
                controllers.pause = False

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
                # 긴급정지 활성화: 스핀들/피드 즉각 정지, G코드 전송 중단, 소프트 리셋
                controllers.pause = True
                controllers.state = "emergency"
                controllers.stop_csv_logging()

                # 스핀들/피드 즉각 정지
                try:
                    controllers.ser.write(b"M5\n")  # 스핀들 정지
                    controllers.ser.flush()
                    time.sleep(0.05)
                    controllers.ser.write(b"F0\n")  # 피드 0
                    controllers.ser.flush()
                    time.sleep(0.05)
                except Exception as e:
                    socketio.emit('log_update', {"message": f"스핀들/피드 정지 명령 오류: {str(e)}", "color": "red"})

                # 소프트 리셋(CTRL+X)로 모든 동작 즉시 정지
                controllers.ser.write(b"\x18")
                controllers.ser.flush()
                time.sleep(0.3)

                socketio.emit('log_update', {"message": "긴급정지 실행! 스핀들/피드 즉시 정지 및 소프트 리셋(CTRL+X)로 즉시 정지.", "color": "red"})
                socketio.emit('status_update', {"text": "긴급정지", "color": "red"})
                socketio.emit('alarm_update', {'text': "긴급정지 상태", 'color': 'red'})
                socketio.emit('disable_nc_upload', {})

                # 경로 및 상태 즉시 반영
                socketio.emit('position_update', {
                    'x': controllers.x_vals[-1] if controllers.x_vals else 0,
                    'y': controllers.y_vals[-1] if controllers.y_vals else 0,
                    'z': controllers.z_vals[-1] if controllers.z_vals else 0,
                    'path_x': controllers.x_vals,
                    'path_y': controllers.y_vals,
                    'path_z': controllers.z_vals,
                    'spindle': 0,
                    'feedrate': 0,
                    'progress': controllers.progress,
                    'state': controllers.state
                })

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

            # 알람 상태 확인
            controllers.ser.reset_input_buffer()
            controllers.ser.write(b"?\n")
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
            
            # 그랩 모드 전환 및 이동
            if controllers.ser and controllers.ser.is_open:
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
        
        try:
            # 기존 버퍼 비우기
            controllers.ser.reset_input_buffer()
            # 상태 확인 명령 전송
            controllers.ser.write(b"?\n")
            controllers.ser.flush()  # 버퍼 즉시 전송
            # 응답 여러 줄 모두 읽기 (최대 0.8초 대기)
            responses = []
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
            gcode_module.ser.write(cmd.encode())
            gcode_module.ser.flush()
            print(f"[G10] 명령 전송됨: {cmd.strip()}")

            return jsonify({
                "status": "success",
                "message": f"현재 위치를 {axis.upper()} 원점(G10 P0 L20 {axis.upper()}0)으로 설정했습니다."
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
            elapsed_time = 0
            if hasattr(gcode_module, 'process_start_time') and gcode_module.process_start_time:
                elapsed_time = time.time() - gcode_module.process_start_time

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

            # NC 코드 정보
            nc_code_info = {
                'lines': len(getattr(gcode_module, 'gcode_lines', [])),
                'sample': getattr(gcode_module, 'gcode_lines', [])[:10]
            }

            # CNC 경로 및 상태
            socketio.emit('position_update', {
                'x': gcode_module.x_vals[-1] if gcode_module.x_vals else 0,
                'y': gcode_module.y_vals[-1] if gcode_module.y_vals else 0,
                'z': gcode_module.z_vals[-1] if gcode_module.z_vals else 0,
                'path_x': gcode_module.x_vals,
                'path_y': gcode_module.y_vals,
                'path_z': gcode_module.z_vals,
                'spindle': gcode_module.current_spindle,
                'feedrate': gcode_module.current_feedrate,
                'progress': gcode_module.progress,
                'elapsed_time': elapsed_time,
                'state': gcode_module.state,
                'nc_code_info': nc_code_info
            })

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
            # 명령 전송
            gcode_module.ser.write((cmd + '\n').encode())
            gcode_module.ser.flush()
            socketio.emit('log_update', {"message": f"> {cmd}", "color": "blue"})
            # 응답 읽기 (최대 2초 대기)
            start_time = time.time()
            responses = []
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
            import time
            ser = getattr(controllers, 'ser', None)
            cmd = data.get('cmd')

            if not ser or not ser.is_open:
                socketio.emit('log_update', {"message": "CNC 컨트롤러가 연결되어 있지 않습니다.", "color": "red"})
                return

            try:
                if cmd == '0x9A':
                    ser.write(b'\x9A')
                    socketio.emit('log_update', {"message": "Spindle Override +10% 명령(0x9A) 전송", "color": "green"})
                elif cmd == '0x9C':
                    ser.write(b'\x9B')
                    socketio.emit('log_update', {"message": "Spindle Override -10% 명령(0x9C) 전송", "color": "green"})
                elif cmd == '0x99':
                    ser.write(b'\x99')
                    socketio.emit('log_update', {"message": "Spindle Override Reset 명령(0x99) 전송", "color": "green"})
                else:
                    socketio.emit('log_update', {"message": f"알 수 없는 Spindle Override 명령: {cmd}", "color": "orange"})
                ser.flush()

                # GRBL 상태 확인 및 FS 값 파싱
                time.sleep(0.3)
                ser.write(b"?\n")
                ser.flush()
                time.sleep(0.2)

                if ser.in_waiting > 0:
                    status = ser.readline().decode().strip()
                    if "|FS:" in status:
                        fs_part = status.split("|")
                        for part in fs_part:
                            if part.startswith("FS:"):
                                try:
                                    _, spindle = part.replace("FS:", "").split(",")
                                    # spindle 값을 controllers.current_spindle에 반영
                                    controllers.current_spindle = int(spindle)
                                    socketio.emit("spindle_rpm_update", {"rpm": spindle})
                                    # position_update도 emit
                                    socketio.emit('position_update', {
                                        'x': controllers.x_vals[-1] if controllers.x_vals else 0,
                                        'y': controllers.y_vals[-1] if controllers.y_vals else 0,
                                        'z': controllers.z_vals[-1] if controllers.z_vals else 0,
                                        'path_x': controllers.x_vals,
                                        'path_y': controllers.y_vals,
                                        'path_z': controllers.z_vals,
                                        'spindle': controllers.current_spindle,
                                        'feedrate': controllers.current_feedrate,
                                        'progress': controllers.progress
                                    })
                                except:
                                    pass
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
            tolerance_value = new_tolerance  # 전역 변수 업데이트
            controllers.tolerance_value = new_tolerance  # controllers 객체 업데이트
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
        """rework.nc 파일을 생성하는 내부 함수"""
        try:
            import os
            if not controllers.gcode_lines:
                return {'status': 'error', 'message': 'G-code 데이터가 없습니다.'}

            # Vibration_detection.csv 파일 존재 여부 확인
            if not os.path.exists(os.path.join(BASE_DIR, 'data', 'Vibration_detection.csv')):
                error_msg = "Vibration_detection.csv 파일이 존재하지 않아 rework.nc를 생성할 수 없습니다."
                logging.error(error_msg)
                return {'status': 'error', 'message': error_msg}

            # rework.nc 파일 생성 - 스핀들 속도를 23000으로 고정
            success = create_rework_nc(
                controllers.gcode_lines,
                controllers.tolerance_value,
                controllers.current_feedrate,
                23000  # 스핀들 속도를 23000으로 고정
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
            # 현재 위치 초기화
            controllers.x_vals[:] = [0]
            controllers.y_vals[:] = [0]
            controllers.z_vals[:] = [0]
            controllers.progress = 0
            
            # UI 업데이트
            socketio.emit('path_update', {
                'x': controllers.x_vals,
                'y': controllers.y_vals,
                'z': controllers.z_vals
            })
            socketio.emit('progress_update', {"progress": controllers.progress})
            
            logging.info("경로가 초기화되었습니다.")
            return {'status': 'success', 'message': '경로가 초기화되었습니다.'}
        except Exception as e:
            logging.error(f"경로 초기화 중 오류: {str(e)}")
            return {'status': 'error', 'message': str(e)}

    @socketio.on('load_rework_file')
    def handle_load_rework_file(data=None):
        """rework.nc 파일을 로드하고 가공을 시작합니다."""
        try:
            # rework.nc 파일 읽기
            try:
                with open(os.path.join(BASE_DIR, 'nc', 'rework.nc'), 'r') as f:
                    rework_content = f.read()
            except Exception as e:
                error_msg = f"rework.nc 파일을 읽을 수 없습니다: {str(e)}"
                socketio.emit('log_update', {"message": error_msg, "color": "red"})
                return {"status": "error", "message": error_msg}

            # NC 코드 로드
            result = controllers.load_nc_code(rework_content)
            if result['status'] != 'success':
                socketio.emit('log_update', {"message": f"rework.nc 로드 실패: {result['message']}", "color": "red"})
                return result

            # 가공 시작
            socketio.emit('log_update', {"message": "rework.nc 로드 완료, 가공을 시작합니다.", "color": "green"})
            
            # 가공 시작 처리
            controllers.gcode_current_idx = 0
            controllers.pause = False
            controllers.x_vals = [0]
            controllers.y_vals = [0]
            controllers.z_vals = [0]
            controllers.progress = 0
            controllers.state = "running"
            
            # UI 업데이트
            socketio.emit('status_update', {"text": "가공 시작", "color": "blue"})
            socketio.emit('disable_nc_upload', {})
            
            # 가공 스레드 시작
            threading.Thread(target=controllers.send_gcode, daemon=True).start()
            
            return {"status": "success", "message": "rework.nc 가공이 시작되었습니다."}
            
        except Exception as e:
            error_msg = f"rework.nc 처리 중 오류: {str(e)}"
            socketio.emit('log_update', {"message": error_msg, "color": "red"})
            return {"status": "error", "message": error_msg}
