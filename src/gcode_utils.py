"""G-code 파싱 유틸리티

- 주석 제거 및 단어(문자+숫자) 분리
- 모달(Modal) 상태 추적: G0/G1/G2/G3 이동 모드, G90/G91, 평면(G17/G18/G19), F, S
  → 'X30 F70'처럼 G 코드가 생략된 줄도 직전 이동 모드로 해석합니다.
- 직선/원호(G2/G3) 이동 거리 계산
"""
import math
import re
from collections import namedtuple

WORD_RE = re.compile(r'([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))')
COMMENT_RE = re.compile(r'\([^)]*\)')

MOTION_CODES = {0: 'G0', 1: 'G1', 2: 'G2', 3: 'G3', 80: 'G80'}
# 축 단어(X/Y/Z)를 쓰지만 이동 명령이 아닌 G 코드
# (G4 드웰, G10 좌표계 설정, G28/G30 원점 복귀, G53 기계 좌표, G92 좌표 오프셋)
NON_MOTION_AXIS_CODES = {4, 10, 28, 30, 53, 92}
# 평면별 (주축1, 주축2, 수직축) 인덱스와 원호 중심 오프셋 문자
PLANE_AXES = {17: (0, 1, 2, 'I', 'J'), 18: (2, 0, 1, 'K', 'I'), 19: (1, 2, 0, 'J', 'K')}

Move = namedtuple('Move', ['mode', 'start', 'end', 'length'])


def strip_comments(line):
    """괄호 주석과 세미콜론 주석을 제거합니다."""
    return COMMENT_RE.sub('', line).split(';', 1)[0]


def parse_words(line):
    """G-code 한 줄을 [(문자, 값), ...] 목록으로 분리합니다. 예: 'G1X10 F50' → [('G',1),('X',10),('F',50)]"""
    return [(letter, float(value)) for letter, value in WORD_RE.findall(strip_comments(line).upper())]


def code_numbers(words, letter):
    """특정 문자(G 또는 M)의 정수 코드 목록 (G1.1 같은 소수 코드는 제외)"""
    return [int(value) for l, value in words if l == letter and value == int(value)]


def arc_length(start, end, offsets, radius, clockwise, plane=17):
    """원호(G2/G3) 길이를 계산합니다. 중심 오프셋(I/J/K) 또는 반지름(R) 형식을 지원합니다."""
    a1, a2, a3, o1, o2 = PLANE_AXES.get(plane, PLANE_AXES[17])
    s1, s2 = start[a1], start[a2]
    e1, e2 = end[a1], end[a2]
    height = abs(end[a3] - start[a3])  # 나선(helical) 이동 성분
    chord = math.hypot(e1 - s1, e2 - s2)

    if radius is not None:
        r = abs(radius)
        if r == 0:
            return math.hypot(chord, height)
        half = min(chord / (2 * r), 1.0)
        sweep = 2 * math.asin(half)
        if radius < 0:  # 음수 R은 180도보다 큰 원호
            sweep = 2 * math.pi - sweep
    else:
        c1 = s1 + offsets.get(o1, 0.0)
        c2 = s2 + offsets.get(o2, 0.0)
        r = math.hypot(s1 - c1, s2 - c2)
        if r == 0:
            return math.hypot(chord, height)
        a_start = math.atan2(s2 - c2, s1 - c1)
        a_end = math.atan2(e2 - c2, e1 - c1)
        sweep = (a_start - a_end) if clockwise else (a_end - a_start)
        if sweep <= 1e-9:  # 시작점과 끝점이 같으면 완전한 원
            sweep += 2 * math.pi
    return math.hypot(r * sweep, height)


class ModalState:
    """G-code 모달 상태를 따라가며 각 줄의 이동을 계산합니다."""

    def __init__(self, position=(0.0, 0.0, 0.0)):
        self.motion = 'G0'
        self.absolute = True
        self.plane = 17
        self.feed = None
        self.spindle = None
        self.position = [float(v) for v in position]

    def apply(self, line):
        """한 줄을 적용하고, 이동이 있으면 Move를 반환합니다 (이동이 없으면 None)."""
        words = parse_words(line)
        if not words:
            return None

        axes, offsets, radius = {}, {}, None
        non_motion = False
        for letter, value in words:
            if letter == 'G':
                code = int(value) if value == int(value) else None
                if code in MOTION_CODES:
                    self.motion = MOTION_CODES[code]
                elif code == 90:
                    self.absolute = True
                elif code == 91:
                    self.absolute = False
                elif code in PLANE_AXES:
                    self.plane = code
                elif code in NON_MOTION_AXIS_CODES:
                    non_motion = True
            elif letter in 'XYZ':
                axes[letter] = value
            elif letter in 'IJK':
                offsets[letter] = value
            elif letter == 'R':
                radius = value
            elif letter == 'F':
                self.feed = value
            elif letter == 'S':
                self.spindle = value

        if non_motion or not axes or self.motion not in ('G0', 'G1', 'G2', 'G3'):
            return None

        start = tuple(self.position)
        end = list(self.position)
        for i, axis in enumerate('XYZ'):
            if axis in axes:
                end[i] = axes[axis] if self.absolute else end[i] + axes[axis]
        self.position = end

        if self.motion in ('G2', 'G3'):
            length = arc_length(start, end, offsets, radius, self.motion == 'G2', self.plane)
        else:
            length = math.dist(start, end)
        return Move(self.motion, start, tuple(end), length)
