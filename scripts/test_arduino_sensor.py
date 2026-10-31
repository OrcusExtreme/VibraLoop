#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
arduino_sensor.py의 수정된 동작을 테스트하기 위한 스크립트
"""

import subprocess
import time
import os

def run_test():
    """
    arduino_sensor.py의 동작을 테스트합니다.
    1. 포트를 지정하지 않고 실행 (시뮬레이션 모드로 전환해야 함)
    2. 존재하지 않는 포트 지정 (연결 실패 후 시뮬레이션 모드로 전환해야 함)
    3. 기타 테스트 케이스
    """
    print("=" * 70)
    print("arduino_sensor.py 수정 테스트")
    print("=" * 70)
    
    # 1. 포트를 지정하지 않고 실행 테스트
    print("\n[테스트 1] 포트를 지정하지 않고 실행 (시뮬레이션 모드로 전환해야 함)")
    print("-" * 70)
    
    process = subprocess.Popen(
        ["python", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "arduino_sensor.py")], 
        stdout=subprocess.PIPE, 
        stderr=subprocess.PIPE,
        text=True,
        encoding='utf-8'
    )
    
    # 5초 동안 실행 후 종료
    time.sleep(5)
    process.terminate()
    
    stdout, stderr = process.communicate()
    print("출력:")
    print(stderr)  # 표준 에러 출력 (디버그 메시지)
    print(stdout[:500] + "..." if len(stdout) > 500 else stdout)  # 표준 출력 제한
    
    # 2. 존재하지 않는 포트 지정 테스트
    print("\n\n[테스트 2] 존재하지 않는 포트 지정 (연결 실패 후 시뮬레이션 모드로 전환해야 함)")
    print("-" * 70)
    
    process = subprocess.Popen(
        ["python", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "arduino_sensor.py"), "--port", "COM99"], 
        stdout=subprocess.PIPE, 
        stderr=subprocess.PIPE,
        text=True,
        encoding='utf-8'
    )
    
    # 5초 동안 실행 후 종료
    time.sleep(5)
    process.terminate()
    
    stdout, stderr = process.communicate()
    print("출력:")
    print(stderr)  # 표준 에러 출력 (디버그 메시지)
    print(stdout[:500] + "..." if len(stdout) > 500 else stdout)  # 표준 출력 제한
    
    print("\n\n[테스트 완료]")
    print("수정된 arduino_sensor.py는 다음과 같이 동작해야 합니다:")
    print("1. 포트를 명시적으로 지정하지 않으면 시뮬레이션 모드로 실행")
    print("2. 지정된 포트에만 연결 시도 (다른 포트 자동 시도 없음)")
    print("3. 연결 실패 시 시뮬레이션 모드로 전환")
    print("4. 연결 끊김 시 자동 재연결 시도 없이 프로그램 종료")
    
if __name__ == "__main__":
    run_test() 