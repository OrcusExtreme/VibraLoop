"""센서 시리얼 데이터 해석: 기존 JSON 펌웨어와 고속 텍스트 펌웨어를 함께 지원합니다.

고속 펌웨어(arduino/MPU6050_fast, 500000 bps) 형식, 한 줄에 하나:
  #H,device=MPU6050,fs=1000,range_g=8,dlpf_hz=184   헤더 (시작할 때와 5초마다)
  A,<seq>,<ax>,<ay>,<az>                             샘플 (센서 원시 정수값, seq는 샘플마다 1씩 증가)
  #O,<seq>                                           센서 FIFO가 넘쳐 샘플이 빠짐 (seq를 경과 시간 기준으로 다시 맞춤)
샘플의 장치 시각은 seq / fs 이므로 수신 지연과 관계없이 실제 측정 간격이 유지됩니다.

기존 형식: {"accel": {"x","y","z"}, "gyro": {"x","y","z"}} (선택: "seq", "t"=장치 ms),
{"accelX", ...}, {"x","y","z"}, 텍스트 "x: .. y: .. z: .."
"""
import json
import re
from collections import namedtuple

# device_time: 장치 기준 측정 시각(초, 모르면 None), seq: 장치 일련번호(없으면 None)
Sample = namedtuple('Sample', 'host_time device_time seq ax ay az gx gy gz')

TEXT_XYZ_RE = re.compile(r'x:\s*(-?\d+(?:\.\d+)?)\s+y:\s*(-?\d+(?:\.\d+)?)\s+z:\s*(-?\d+(?:\.\d+)?)', re.IGNORECASE)


def _num(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number == number else None  # NaN 제외


def parse_legacy_line(line):
    """기존 형식 한 줄 → (ax, ay, az, gx, gy, gz, seq, device_ms) 또는 None."""
    try:
        data = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        data = None
    if isinstance(data, dict):
        gyro = data.get('gyro') if isinstance(data.get('gyro'), dict) else {}
        if isinstance(data.get('accel'), dict):
            accel = data['accel']
            values = (accel.get('x'), accel.get('y'), accel.get('z'), gyro.get('x'), gyro.get('y'), gyro.get('z'))
        elif all(key in data for key in ('accelX', 'accelY', 'accelZ')):
            values = (data['accelX'], data['accelY'], data['accelZ'], data.get('gyroX'), data.get('gyroY'), data.get('gyroZ'))
        elif all(key in data for key in ('x', 'y', 'z')):
            values = (data['x'], data['y'], data['z'], gyro.get('x'), gyro.get('y'), gyro.get('z'))
        else:
            return None
        numbers = [_num(v) for v in values]
        if None in numbers[:3]:
            return None
        seq = data.get('seq')
        return (*numbers, int(seq) if isinstance(seq, (int, float)) else None, _num(data.get('t')))
    match = TEXT_XYZ_RE.match(line or '')
    if match:
        return (*(float(v) for v in match.groups()), None, None, None, None, None)
    return None


class SensorStream:
    """연결 하나의 센서 줄을 표준 샘플로 바꿉니다 (헤더 설정, 일련번호 누락 집계 포함)."""

    def __init__(self, default_range_g=2.0):
        self.device = None
        self.fs = None
        self.range_g = default_range_g
        self.dlpf_hz = None
        self.last_seq = None
        self.device_time = None  # 고속 펌웨어 샘플의 장치 시각 (seq 증가분 / fs를 누적, fs가 갱신돼도 시각이 튀지 않음)
        self.dropped = 0
        self.overflows = 0
        self.samples = 0
        self._waiting_header_reported = False

    def describe(self):
        return {'device': self.device, 'fs': self.fs, 'range_g': self.range_g, 'dlpf_hz': self.dlpf_hz,
                'samples': self.samples, 'dropped': self.dropped, 'overflows': self.overflows}

    def _check_seq(self, seq):
        """seq 증가분을 반환하고 빠진 샘플 수를 집계합니다 (장치가 다시 시작해 seq가 줄면 1로 봄)."""
        step = 1
        if self.last_seq is not None:
            step = seq - self.last_seq
            if step > 1:
                self.dropped += step - 1
            elif step <= 0:
                step = 1
        self.last_seq = seq
        return step

    def parse_line(self, line, host_time):
        """한 줄 → (샘플 목록, 화면에 보낼 메시지 또는 None)."""
        line = (line or '').strip()
        if not line:
            return [], None

        if line.startswith('A,'):
            if self.fs is None:
                if self._waiting_header_reported:
                    return [], None
                self._waiting_header_reported = True
                return [], "[센서] 고속 펌웨어 헤더(#H)를 기다리는 중입니다 (최대 5초)."
            parts = line.split(',')
            try:
                seq = int(parts[1])
                raw = [int(p) for p in parts[2:5]]
            except (IndexError, ValueError):
                return [], None
            if len(raw) != 3:
                return [], None
            step = self._check_seq(seq)
            self.device_time = (self.device_time or 0.0) + step / self.fs
            scale = self.range_g / 32768.0
            self.samples += 1
            return [Sample(host_time, self.device_time, seq, raw[0] * scale, raw[1] * scale, raw[2] * scale,
                           None, None, None)], None

        if line.startswith('#H'):
            fields = dict(item.split('=', 1) for item in line.split(',')[1:] if '=' in item)
            previous = (self.device, self.fs, self.range_g)
            self.device = fields.get('device', 'fast')
            self.fs = _num(fields.get('fs')) or self.fs
            self.range_g = _num(fields.get('range_g')) or self.range_g
            self.dlpf_hz = _num(fields.get('dlpf_hz'))
            if (self.device, self.fs, self.range_g) != previous:
                return [], f"[센서] {self.device}: {self.fs:g} Hz, ±{self.range_g:g} g, 저역통과 {self.dlpf_hz or '-'} Hz"
            return [], None

        if line.startswith('#O'):
            # 펌웨어가 빠진 샘플 수를 경과 시간으로 추정해 다음 샘플의 seq를 알려줌 → 그만큼 시각을 띄움
            self.overflows += 1
            try:
                next_seq = int(line.split(',')[1])
            except (IndexError, ValueError):
                next_seq = None
            if next_seq is not None and self.last_seq is not None and self.fs:
                skipped = next_seq - self.last_seq - 1
                if skipped > 0:
                    self.dropped += skipped
                    self.device_time = (self.device_time or 0.0) + skipped / self.fs
                self.last_seq = next_seq - 1
            return [], "[센서] 센서 FIFO가 넘쳐 샘플 일부가 빠졌습니다 (전송 속도·케이블 확인)."

        parsed = parse_legacy_line(line)
        if parsed is None:
            return [], f"[센서] {line}"
        ax, ay, az, gx, gy, gz, seq, device_ms = parsed
        self.device = self.device or 'json'
        if seq is not None:
            self._check_seq(seq)
        self.samples += 1
        device_time = device_ms / 1000.0 if device_ms is not None else None
        return [Sample(host_time, device_time, seq, ax, ay, az, gx, gy, gz)], None
