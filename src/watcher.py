"""rework.nc 파일 감시

서버(cnc_app.py) 프로세스 안에서 watchdog으로 nc/rework.nc의 생성·수정을 감지해 콜백을 호출합니다.
예전처럼 별도 프로세스에서 Socket.IO 클라이언트로 서버에 다시 접속하지 않으므로
`python watcher.py`를 따로 실행할 필요가 없습니다 (따로 실행하면 재가공이 중복으로 시작될 수 있었음).
"""
import os
import time
import threading
import logging
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

logger = logging.getLogger('watcher')


class FileHandler(FileSystemEventHandler):
    """지정한 파일(기본: rework.nc)이 생성·수정·교체(이동)되면 콜백을 호출합니다."""

    def __init__(self, on_file_ready, filename="rework.nc", debounce_sec=1.0):
        super().__init__()
        self.on_file_ready = on_file_ready
        self.filename = filename
        self.debounce_sec = debounce_sec
        self.last_handled = 0.0
        self.lock = threading.Lock()

    def on_created(self, event):
        self._handle(event, event.src_path)

    def on_modified(self, event):
        self._handle(event, event.src_path)

    def on_moved(self, event):
        # rework.py는 임시 파일에 쓴 뒤 os.replace로 교체하므로 이동 이벤트의 대상 경로를 확인
        self._handle(event, event.dest_path)

    def _handle(self, event, path):
        if event.is_directory or os.path.basename(path) != self.filename:
            return
        with self.lock:
            now = time.time()
            if now - self.last_handled < self.debounce_sec:  # 한 번 저장할 때 여러 이벤트가 연달아 발생하므로 1회만 처리
                return
            self.last_handled = now
        self._wait_until_stable(path)
        logger.info(f"{self.filename} 변경 감지: {path}")
        try:
            self.on_file_ready(path)
        except Exception as e:
            logger.error(f"{self.filename} 처리 중 오류: {e}", exc_info=True)

    @staticmethod
    def _wait_until_stable(path, timeout=3.0, interval=0.2):
        """파일 크기가 더 이상 바뀌지 않을 때까지(쓰기가 끝날 때까지) 잠시 기다립니다."""
        deadline = time.time() + timeout
        last_size = -1
        while time.time() < deadline:
            try:
                size = os.path.getsize(path)
            except OSError:
                size = -1
            if size == last_size and size > 0:
                return
            last_size = size
            time.sleep(interval)


def start_rework_watcher(watch_dir, on_rework_ready):
    """watch_dir(nc 폴더)의 rework.nc를 감시하는 Observer를 시작해 반환합니다."""
    os.makedirs(watch_dir, exist_ok=True)
    observer = Observer()
    observer.daemon = True
    observer.schedule(FileHandler(on_rework_ready), watch_dir, recursive=False)
    observer.start()
    return observer
