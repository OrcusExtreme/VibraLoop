"""데이터 수집 기록 (설계 근거: edit_analysis.md 3.3)

실행(run) 하나마다 data/raw/에 두 파일을 만들고, 기록을 마치면 manifest.csv에 한 줄을 추가합니다.
- <run_id>_sensor.csv : HostTime(서버 수신 시각, epoch 초), DeviceTime(장치 측정 시각, 초), Seq(장치 일련번호),
                        AccX, AccY, AccZ (g), GyroX, GyroY, GyroZ (deg/s, 없으면 빈칸)
- <run_id>_machine.csv: HostTime, State(서버 상태), GrblState, X, Y, Z(작업 좌표), SpindleCmd, SpindleOverride,
                        SpindleRPM(오버라이드 반영), FeedCmd, FeedOverride, FeedRate(오버라이드 반영),
                        FeedRT, SpindleRT(GRBL 실시간 보고), Line(GRBL로 보낸 줄 수)
- manifest.csv: 실행당 한 줄 (조건, 공구, 센서 설정, 시간 구간 라벨 JSON, 파일 이름)
두 파일의 HostTime은 같은 시계(서버)이므로 라벨 구간과 가공 로그로 센서 데이터를 나눌 수 있습니다.
"""
import csv
import json
import os
import re
import threading
import time

SENSOR_FIELDS = ['HostTime', 'DeviceTime', 'Seq', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ']
MACHINE_FIELDS = ['HostTime', 'State', 'GrblState', 'X', 'Y', 'Z', 'SpindleCmd', 'SpindleOverride', 'SpindleRPM',
                  'FeedCmd', 'FeedOverride', 'FeedRate', 'FeedRT', 'SpindleRT', 'Line']
META_FIELDS = ['program', 'run_type', 'S_rpm', 'F_mm_min', 'ap_mm', 'ae_mm', 'tool_id', 'tool_diameter_mm', 'flutes',
               'stickout_mm', 'material', 'photo', 'notes']
MANIFEST_FIELDS = (['run_id', 'started_at', 'ended_at', 'duration_s'] + META_FIELDS
                   + ['sensor_device', 'sensor_fs_hz', 'sensor_range_g', 'sensor_samples', 'sensor_dropped',
                      'machine_rows', 'labels', 'sensor_file', 'machine_file'])
LABELS = {'chatter': '채터', 'transition': '전이', 'air': '비가공', 'stable': '안정'}
FLUSH_INTERVAL = 0.5


def _fmt(value, digits=4):
    if value is None:
        return ''
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return value


class RunRecorder:
    def __init__(self, raw_dir):
        self.raw_dir = raw_dir
        self.manifest_path = os.path.join(raw_dir, 'manifest.csv')
        self._lock = threading.Lock()
        self.active = False
        self.run = None
        self.last_run = None

    def _new_run_id(self, requested):
        run_id = re.sub(r'[^0-9A-Za-z가-힣_.-]', '_', str(requested or '').strip())[:60]
        if not run_id:
            run_id = time.strftime('R%Y%m%d_%H%M%S')
        candidate, counter = run_id, 1
        while os.path.exists(os.path.join(self.raw_dir, f"{candidate}_sensor.csv")):
            candidate = f"{run_id}_{counter}"
            counter += 1
        return candidate

    def start(self, meta=None, sensor_info=None):
        """기록을 시작합니다. 반환: (성공 여부, 메시지, run_id)"""
        meta = meta or {}
        with self._lock:
            if self.active:
                return False, f"이미 {self.run['run_id']}을(를) 기록하고 있습니다.", self.run['run_id']
            os.makedirs(self.raw_dir, exist_ok=True)
            run_id = self._new_run_id(meta.get('run_id'))
            sensor_file = f"{run_id}_sensor.csv"
            machine_file = f"{run_id}_machine.csv"
            sensor_fh = open(os.path.join(self.raw_dir, sensor_file), 'w', newline='', encoding='utf-8')
            machine_fh = open(os.path.join(self.raw_dir, machine_file), 'w', newline='', encoding='utf-8')
            sensor_writer = csv.writer(sensor_fh)
            machine_writer = csv.writer(machine_fh)
            sensor_writer.writerow(SENSOR_FIELDS)
            machine_writer.writerow(MACHINE_FIELDS)
            self.run = {
                'run_id': run_id, 'started': time.time(),
                'meta': {key: str(meta[key]).strip() for key in META_FIELDS if meta.get(key) not in (None, '')},
                'sensor_info': dict(sensor_info or {}),
                'sensor_file': sensor_file, 'machine_file': machine_file,
                'sensor_fh': sensor_fh, 'machine_fh': machine_fh,
                'sensor_writer': sensor_writer, 'machine_writer': machine_writer,
                'sensor_samples': 0, 'machine_rows': 0, 'last_seq': None, 'dropped': 0,
                'labels': [], 'open_labels': {}, 'last_flush': time.time(),
            }
            self.active = True
            return True, f"{run_id} 기록을 시작했습니다.", run_id

    def _maybe_flush(self, run):
        now = time.time()
        if now - run['last_flush'] >= FLUSH_INTERVAL:
            run['sensor_fh'].flush()
            run['machine_fh'].flush()
            run['last_flush'] = now

    def add_sensor(self, samples):
        """sensor_protocol.Sample 목록을 기록합니다."""
        if not self.active:
            return
        with self._lock:
            run = self.run
            if run is None:
                return
            writer = run['sensor_writer']
            for s in samples:
                if s.seq is not None:
                    if run['last_seq'] is not None and s.seq - run['last_seq'] > 1:
                        run['dropped'] += s.seq - run['last_seq'] - 1
                    run['last_seq'] = s.seq
                writer.writerow([f"{s.host_time:.6f}", _fmt(s.device_time, 6), '' if s.seq is None else s.seq,
                                 f"{s.ax:.6f}", f"{s.ay:.6f}", f"{s.az:.6f}",
                                 _fmt(s.gx, 4), _fmt(s.gy, 4), _fmt(s.gz, 4)])
            run['sensor_samples'] += len(samples)
            self._maybe_flush(run)

    def add_machine(self, snap):
        """cnc_app.machine_snapshot() 결과를 기록합니다."""
        if not self.active or not snap:
            return
        with self._lock:
            run = self.run
            if run is None:
                return
            run['machine_writer'].writerow([
                f"{snap.get('time', time.time()):.6f}", snap.get('state') or '', snap.get('grbl_state') or '',
                _fmt(snap.get('x')), _fmt(snap.get('y')), _fmt(snap.get('z')),
                _fmt(snap.get('n_cmd'), 1), _fmt(snap.get('spindle_override'), 0), _fmt(snap.get('n_eff'), 1),
                _fmt(snap.get('F_cmd'), 1), _fmt(snap.get('feed_override'), 0), _fmt(snap.get('F_eff'), 1),
                _fmt(snap.get('feed_rt'), 1), _fmt(snap.get('spindle_rt'), 1), snap.get('line', ''),
            ])
            run['machine_rows'] += 1
            self._maybe_flush(run)

    def mark(self, label, edge, note='', position=None):
        """라벨 구간 표시. edge: 'start' 또는 'end'. 반환: (성공 여부, 메시지)"""
        if label not in LABELS:
            return False, f"알 수 없는 라벨입니다: {label}"
        with self._lock:
            run = self.run
            if not self.active or run is None:
                return False, "기록 중이 아닙니다. 먼저 기록을 시작하세요."
            now = time.time()
            x = position[0] if position else None
            name = LABELS[label]
            if edge == 'start':
                if label in run['open_labels']:
                    return False, f"'{name}' 구간이 이미 열려 있습니다."
                run['open_labels'][label] = {'label': label, 'start': now, 'start_x': x, 'note': note or ''}
                return True, f"'{name}' 구간 시작 ({now - run['started']:.1f}초)"
            if edge == 'end':
                segment = run['open_labels'].pop(label, None)
                if segment is None:
                    return False, f"열린 '{name}' 구간이 없습니다."
                segment.update(end=now, end_x=x)
                if note:
                    segment['note'] = (segment['note'] + ' ' + note).strip()
                run['labels'].append(segment)
                return True, f"'{name}' 구간 종료 ({segment['end'] - segment['start']:.1f}초 동안)"
            return False, f"알 수 없는 동작입니다: {edge}"

    def stop(self, sensor_info=None, position=None):
        """기록을 마치고 manifest.csv에 한 줄을 추가합니다. 반환: (성공 여부, 메시지, 요약)"""
        with self._lock:
            run = self.run
            if not self.active or run is None:
                return False, "기록 중이 아닙니다.", None
            now = time.time()
            for segment in run['open_labels'].values():  # 닫지 않은 구간은 기록 종료 시각에 닫음
                segment.update(end=now, end_x=position[0] if position else None)
                run['labels'].append(segment)
            run['open_labels'] = {}
            run['sensor_fh'].close()
            run['machine_fh'].close()
            info = dict(run['sensor_info'], **(sensor_info or {}))
            row = {
                'run_id': run['run_id'],
                'started_at': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(run['started'])),
                'ended_at': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)),
                'duration_s': f"{now - run['started']:.1f}",
                **run['meta'],
                'sensor_device': info.get('device') or '',
                'sensor_fs_hz': info.get('fs') or '',
                'sensor_range_g': info.get('range_g') or '',
                'sensor_samples': run['sensor_samples'],
                'sensor_dropped': run['dropped'],
                'machine_rows': run['machine_rows'],
                'labels': json.dumps(run['labels'], ensure_ascii=False),
                'sensor_file': run['sensor_file'],
                'machine_file': run['machine_file'],
            }
            self._append_manifest(row)
            self.last_run = {key: row[key] for key in ('run_id', 'duration_s', 'sensor_samples', 'sensor_dropped',
                                                       'machine_rows', 'sensor_file', 'machine_file')}
            self.last_run['labels'] = len(run['labels'])
            self.run = None
            self.active = False
        message = (f"{row['run_id']} 기록 완료: {row['duration_s']}초, 센서 {row['sensor_samples']}샘플"
                   + (f" (누락 {row['sensor_dropped']})" if row['sensor_dropped'] else "")
                   + f", 라벨 구간 {self.last_run['labels']}개")
        return True, message, self.last_run

    def _append_manifest(self, row):
        exists = os.path.exists(self.manifest_path) and os.path.getsize(self.manifest_path) > 0
        if exists:
            with open(self.manifest_path, encoding='utf-8-sig') as f:
                header = next(csv.reader(f), [])
            if header != MANIFEST_FIELDS:  # 형식이 바뀐 예전 파일은 보관하고 새로 시작
                os.replace(self.manifest_path, self.manifest_path + time.strftime('.%Y%m%d_%H%M%S.old'))
                exists = False
        # 엑셀에서 한글이 깨지지 않도록 새 파일은 BOM이 있는 UTF-8로 작성
        with open(self.manifest_path, 'a', newline='', encoding='utf-8' if exists else 'utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=MANIFEST_FIELDS, extrasaction='ignore')
            if not exists:
                writer.writeheader()
            writer.writerow(row)

    def status(self):
        with self._lock:
            run = self.run
            if not self.active or run is None:
                return {'active': False, 'last_run': self.last_run}
            return {
                'active': True, 'run_id': run['run_id'], 'elapsed': time.time() - run['started'],
                'sensor_samples': run['sensor_samples'], 'sensor_dropped': run['dropped'],
                'machine_rows': run['machine_rows'], 'labels': len(run['labels']),
                'open_labels': sorted(run['open_labels']), 'meta': dict(run['meta']), 'last_run': self.last_run,
            }


def read_manifest(raw_dir):
    """manifest.csv → 실행 목록(dict, labels는 파이썬 객체로)."""
    path = os.path.join(raw_dir, 'manifest.csv')
    if not os.path.exists(path):
        return []
    runs = []
    with open(path, encoding='utf-8-sig', newline='') as f:
        for row in csv.DictReader(f):
            try:
                row['labels'] = json.loads(row.get('labels') or '[]')
            except ValueError:
                row['labels'] = []
            runs.append(row)
    return runs
