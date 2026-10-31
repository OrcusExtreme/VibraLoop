#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
CNC 밀링 머신 모니터링 시스템을 위한 Python 센서 인터페이스
사용 가능한 COM 포트에서 센서 데이터를 읽어 JSON 형식으로 전송합니다.
"""

import serial
import serial.tools.list_ports
import time
import json
import random
import math
import argparse
import os
import sys
from datetime import datetime

# 디버그 로그 추가
print(f"[DEBUG] arduino_sensor.py가 실행됨 - PID: {os.getpid()}", file=sys.stderr)
print(f"[DEBUG] 실행 경로: {os.path.abspath(__file__)}", file=sys.stderr)
print(f"[DEBUG] 실행 인자: {sys.argv}", file=sys.stderr)

# 기본 설정값 - 고정 포트 대신 자동 감지로 변경
DEFAULT_BAUDRATE = 115200  # 아두이노 펌웨어(arduino/) 및 서버(cnc_app.py)와 동일
DEFAULT_SAMPLE_INTERVAL = 0.1  # 100ms 간격 (10Hz, 시뮬레이션 출력용)

class SensorSimulator:
    """실제 센서가 없을 경우 시뮬레이션된 센서 데이터를 생성하는 클래스"""
    
    def __init__(self):
        # 기준 값 설정
        self.base_accel_x = 0.0
        self.base_accel_y = 0.0
        self.base_accel_z = 1.0  # 중력 가속도
        self.base_temp = 25.0
        
        # 노이즈 레벨
        self.noise_level = 0.05
        
        # 시뮬레이션된 진동 패턴
        self.vibration_pattern = 0.0
        self.pattern_speed = 0.1
    
    def read_data(self):
        """시뮬레이션된 센서 데이터 생성"""
        # 시간에 따른 진동 패턴 업데이트
        self.vibration_pattern += self.pattern_speed
        
        # 노이즈와 패턴을 결합하여 가속도 값 생성
        noise_x = random.uniform(-self.noise_level, self.noise_level)
        noise_y = random.uniform(-self.noise_level, self.noise_level)
        noise_z = random.uniform(-self.noise_level, self.noise_level)
        
        # 가공 중 진동 시뮬레이션 (사인파)
        pattern_factor = math.sin(self.vibration_pattern) * 0.2
        
        # 가속도 값 계산
        accel_x = self.base_accel_x + noise_x + pattern_factor
        accel_y = self.base_accel_y + noise_y + pattern_factor
        accel_z = self.base_accel_z + noise_z
        
        # 진동 강도 계산 (RMS)
        vibration = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        # 온도 값 (약간의 변동)
        temp = self.base_temp + random.uniform(-0.5, 0.5)
        
        # 데이터 객체 생성
        data = {
            "accelX": round(accel_x, 3),
            "accelY": round(accel_y, 3),
            "accelZ": round(accel_z, 3),
            "vibration": round(vibration, 3),
            "temp": round(temp, 1),
            "timestamp": datetime.now().strftime("%H:%M:%S.%f")[:-3]
        }
        
        return data

# 사용 가능한 포트 찾기 함수 추가
def find_available_ports():
    """사용 가능한 시리얼 포트 목록을 반환합니다."""
    ports = []
    for port_info in serial.tools.list_ports.comports():
        ports.append(port_info.device)
    return ports

# 포트 연결 시도 함수 추가
def try_connect_port(port, baudrate, timeout=1):
    """지정된 포트에 연결을 시도합니다. 성공하면 시리얼 객체를, 실패하면 None을 반환합니다."""
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        # 정상적으로 연결되었는지 확인
        if ser.is_open:
            return ser
    except PermissionError as pe:
        print(f"포트 {port} 액세스 거부됨: 다른 프로그램이 이미 사용 중일 수 있습니다.")
        print("- 해결 방법: 아두이노 IDE, 시리얼 모니터 등 포트를 사용 중인 프로그램을 종료하세요.")
    except serial.SerialException as se:
        error_msg = str(se)
        if "PermissionError" in error_msg or "Access is denied" in error_msg or "액세스가 거부되었습니다" in error_msg:
            print(f"포트 {port} 액세스 거부됨: 다른 프로그램이 이미 사용 중일 수 있습니다.")
            print("- 해결 방법: 아두이노 IDE, 시리얼 모니터 등 포트를 사용 중인 프로그램을 종료하세요.")
        elif "not found" in error_msg or "찾을 수 없습니다" in error_msg:
            print(f"포트 {port}를 찾을 수 없습니다: 장치가 연결되지 않았거나 잘못된 포트명입니다.")
        else:
            print(f"포트 {port} 연결 실패: {error_msg}")
    except OSError as oe:
        print(f"포트 {port} 연결 중 OS 오류: {str(oe)}")
    except Exception as e:
        print(f"포트 {port} 연결 중 예기치 않은 오류: {str(e)}")
    return None

def main():
    parser = argparse.ArgumentParser(description='Arduino 센서 시뮬레이터 또는 실제 센서 연동')
    parser.add_argument('--port', type=str, help='시리얼 포트 (지정하지 않으면 자동 감지)')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE, help=f'Baud rate (기본값: {DEFAULT_BAUDRATE})')
    parser.add_argument('--interval', type=float, default=DEFAULT_SAMPLE_INTERVAL, help=f'샘플링 간격(초) (기본값: {DEFAULT_SAMPLE_INTERVAL})')
    parser.add_argument('--simulate', action='store_true', help='시뮬레이션 모드 사용 (실제 하드웨어 대신)')
    
    args = parser.parse_args()
    
    # 시리얼 포트가 지정되지 않은 경우 자동 감지
    selected_port = args.port
    if not selected_port and not args.simulate:
        available_ports = find_available_ports()
        if available_ports:
            # 사용 가능한 포트 목록 표시
            print("사용 가능한 포트 목록:")
            for i, port in enumerate(available_ports, 1):
                print(f"{i}. {port}")
                
            # 명시적으로 포트를 선택하도록 안내
            print("포트가 명시적으로 지정되지 않았습니다.")
            print("명령어에 --port 옵션을 추가하여 사용할 포트를 지정하세요.")
            print("예시: python arduino_sensor.py --port COM4")
            print("시뮬레이션 모드로 전환합니다.")
            args.simulate = True
        else:
            print("사용 가능한 시리얼 포트가 없습니다. 시뮬레이션 모드로 전환합니다.")
            args.simulate = True
    
    print(f"CNC 밀링 머신 모니터링 시스템 - 센서 인터페이스")
    print(f"{'시뮬레이션 모드' if args.simulate else '실제 센서 모드'}")
    if not args.simulate:
        print(f"포트: {selected_port}, 속도: {args.baudrate}, 샘플링 간격: {args.interval}초")
    else:
        print(f"시뮬레이션 샘플링 간격: {args.interval}초")
    print("-" * 50)
    
    # 시뮬레이터 초기화
    simulator = SensorSimulator()
    
    try:
        if args.simulate:
            # 시뮬레이션 모드: 가상 데이터 생성
            print("시뮬레이션 모드로 실행 중...")
            while True:
                data = simulator.read_data()
                json_data = json.dumps(data)
                print(json_data)
                time.sleep(args.interval)
        else:
            # 포트 연결 시도
            ser = None
            
            # 지정된 포트 연결 시도
            if selected_port:
                print(f"포트 {selected_port}에 연결 시도 중...")
                ser = try_connect_port(selected_port, args.baudrate)
                if ser:
                    print(f"{selected_port}에 연결됨")
                else:
                    print(f"지정된 포트 {selected_port}에 연결할 수 없습니다.")
                    print("\n연결 문제 해결 방법:")
                    print("1. 아두이노 또는 센서 장치가 컴퓨터에 물리적으로 연결되어 있는지 확인하세요.")
                    print("2. 아두이노 IDE, 시리얼 모니터 등 포트를 사용할 수 있는 다른 프로그램을 종료하세요.")
                    print("3. 장치 관리자에서 COM 포트 드라이버 상태를 확인하세요.")
                    print("4. 다른 USB 포트에 연결을 시도해보세요.")
                    print("5. 필요하다면 관리자 권한으로 이 프로그램을 실행하세요.")
                    print("\n시뮬레이션 모드로 전환합니다.")
                    
                    # 시뮬레이션 모드로 전환
                    print("시뮬레이션 모드로 실행 중...")
                    while True:
                        data = simulator.read_data()
                        json_data = json.dumps(data)
                        print(json_data)
                        time.sleep(args.interval)
            else:
                # 포트가 지정되지 않은 경우
                print("포트가 지정되지 않았습니다. 시뮬레이션 모드로 전환합니다.")
                print("시뮬레이션 모드로 실행 중...")
                while True:
                    data = simulator.read_data()
                    json_data = json.dumps(data)
                    print(json_data)
                    time.sleep(args.interval)
            
            # 실제 센서 모드: 시리얼 포트에서 데이터 읽기
            print(f"{selected_port}에 연결됨")
            time.sleep(2)  # 아두이노 리셋 시간 대기
            
            # 초기화 메시지 읽기
            ser.reset_input_buffer()
            
            print("센서 데이터 읽기 시작...")
            while True:
                try:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            # 받은 데이터가 JSON 형식인지 확인
                            try:
                                data = json.loads(line)
                                # 타임스탬프 추가
                                data["timestamp"] = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                json_data = json.dumps(data)
                                print(json_data)
                            except json.JSONDecodeError:
                                # JSON이 아닌 경우 그대로 출력
                                print(f"Raw data: {line}")
                    time.sleep(0.01)  # CPU 사용량 감소
                except serial.SerialException as se:
                    print(f"시리얼 통신 오류: {str(se)}")
                    # 연결 끊김 처리
                    print("센서 연결이 끊어졌습니다.")
                    if ser:
                        try:
                            ser.close()
                        except:
                            pass
                    
                    # 프로그램 종료
                    print("센서 연결이 끊어져 프로그램을 종료합니다.")
                    break
                except Exception as e:
                    print(f"데이터 읽기 오류: {str(e)}")
                    time.sleep(1)
                        
    except KeyboardInterrupt:
        print("\n프로그램 종료")
        if 'ser' in locals() and ser:
            ser.close()
    except Exception as e:
        print(f"오류 발생: {str(e)}")
        if 'ser' in locals() and ser:
            ser.close()

if __name__ == "__main__":
    main() 