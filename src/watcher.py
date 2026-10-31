import os
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import subprocess
import logging
import socketio
import json
import threading
import requests

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class FileHandler(FileSystemEventHandler):
    def __init__(self):
        self.last_modified = time.time()
        self.processing = False
        self.sio = None
        self.connect_socket()

    def connect_socket(self):
        """Socket.IO 연결을 시도하고 실패시 재시도"""
        if self.sio is None:
            self.sio = socketio.Client(reconnection=True, reconnection_attempts=5, reconnection_delay=5)
        
        max_retries = 5
        retry_count = 0
        while retry_count < max_retries:
            try:
                if not self.sio.connected:
                    self.sio.connect('http://localhost:5000', wait_timeout=10)
                    logging.info("Socket.IO 연결 성공")
                    
                    # 연결 이벤트 핸들러 등록
                    @self.sio.on('connect')
                    def on_connect():
                        logging.info("Socket.IO 서버에 연결됨")
                    
                    @self.sio.on('disconnect')
                    def on_disconnect():
                        logging.info("Socket.IO 서버와 연결이 끊어짐")
                    
                    return True
            except Exception as e:
                retry_count += 1
                logging.error(f"Socket.IO 연결 실패 (시도 {retry_count}/{max_retries}): {str(e)}")
                time.sleep(5)  # 5초 대기 후 재시도
        return False

    def ensure_connection(self):
        """연결이 끊어졌다면 재연결 시도"""
        if not self.sio or not self.sio.connected:
            return self.connect_socket()
        return True

    def on_created(self, event):
        if event.is_directory:
            return

        if os.path.basename(event.src_path) == "Vibration_detection.csv":
            logging.info("Vibration_detection.csv 생성 감지")
            try:
                if not self.ensure_connection():
                    logging.error("Socket.IO 연결 실패로 인해 명령을 전송할 수 없습니다.")
                    return
                
                # reset_path 이벤트 처리를 위한 Event 객체 생성
                self.reset_path_completed = threading.Event()
                
                def on_reset_path_response(data):
                    if data.get('status') == 'success':
                        logging.info("경로 초기화 완료")
                        self.reset_path_completed.set()  # reset_path 완료 신호
                    else:
                        logging.error(f"경로 초기화 실패: {data.get('message', '알 수 없는 오류')}")
                
                def on_make_rework_response(data):
                    if data.get('status') == 'success':
                        logging.info("rework.nc 생성 성공")
                    else:
                        logging.error(f"rework.nc 생성 실패: {data.get('message', '알 수 없는 오류')}")
                
                # 경로 초기화 명령 전송
                self.sio.emit('reset_path', {}, callback=on_reset_path_response)
                
                # reset_path 완료 대기 (최대 10초)
                if self.reset_path_completed.wait(timeout=10.0):
                    logging.info("경로 초기화 완료 확인, rework.nc 생성 요청 시작")
                    # Socket.IO 이벤트로 rework.nc 생성 요청
                    self.sio.emit('make_rework', {}, callback=on_make_rework_response)
                else:
                    logging.error("경로 초기화 타임아웃")
                
            except Exception as e:
                logging.error(f"이벤트 처리 중 오류 발생: {str(e)}")

    def on_modified(self, event):
        if event.is_directory:
            return
        
        if not event.src_path.endswith('.nc'):
            return

        current_time = time.time()
        if current_time - self.last_modified < 1:  # 1초 이내 중복 방지
            return

        self.last_modified = current_time
        
        if self.processing:
            logging.info("이전 파일이 아직 처리 중입니다. 대기합니다.")
            return

        try:
            self.processing = True
            logging.info(f"변경된 파일 감지: {event.src_path}")
            
            if not self.ensure_connection():
                logging.error("Socket.IO 연결 실패로 인해 명령을 전송할 수 없습니다.")
                return
            
            # rework.nc 파일이 생성되면 즉시 가공 중지
            if os.path.basename(event.src_path) == "rework.nc":
                logging.info("rework.nc 파일 감지 - 새 경로 로드")
                try:
                    # 새 경로 로드 명령 전송
                    self.sio.emit('load_rework_file', {})
                    logging.info("새 경로 로드 명령 전송 완료")
                except Exception as e:
                    logging.error(f"경로 로드 중 오류: {str(e)}")
                    self.connect_socket()
            
        except Exception as e:
            logging.error(f"파일 처리 중 오류 발생: {str(e)}")
        finally:
            self.processing = False

    def __del__(self):
        try:
            if self.sio and self.sio.connected:
                self.sio.disconnect()
        except:
            pass

def main():
    path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # 프로젝트 루트 디렉토리 감시
    event_handler = FileHandler()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=True)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
        logging.info("감시가 중지되었습니다.")
    
    observer.join()

if __name__ == "__main__":
    main()