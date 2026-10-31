import re
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gcode_utils import ModalState, NON_MOTION_AXIS_CODES, code_numbers, parse_words, strip_comments

DEFAULT_REWORK_PATH = os.path.join(BASE_DIR, "nc", "rework.nc")
SAFE_Z_HEIGHT = 5.0  # 진입/복귀 시 공구를 올려둘 안전 높이 (mm, 원본 NC 코드의 "G0 Z5" 관례와 동일)

NUMBER = r'[-+]?(?:\d+\.?\d*|\.\d+)'
S_WORD_RE = re.compile(r'S\s*' + NUMBER, re.IGNORECASE)
F_WORD_RE = re.compile(r'F\s*' + NUMBER, re.IGNORECASE)
Z_WORD_RE = re.compile(r'Z\s*(' + NUMBER + ')', re.IGNORECASE)


def _rework_line(line, n_val, f_val, z_offset, modal):
    """한 줄에 최적 S/F를 적용하고, 절대좌표(G90) 이동이면 Z를 z_offset만큼 이동합니다."""
    code = strip_comments(line).strip()
    g_codes = code_numbers(parse_words(code), 'G')
    if 90 in g_codes:
        modal['absolute'] = True
    if 91 in g_codes:
        modal['absolute'] = False
    code = S_WORD_RE.sub(f'S{int(n_val)}', code)
    code = F_WORD_RE.sub(f'F{int(f_val)}', code)
    if z_offset and modal['absolute'] and not set(g_codes) & NON_MOTION_AXIS_CODES:
        code = Z_WORD_RE.sub(lambda m: f"Z{float(m.group(1)) + z_offset:.3f}", code)
    return code


def create_rework_nc(current_n, current_F, optimal_n, optimal_F, x_pos, y_pos, z_pos, gcode_lines=None,
                     tolerance_value=0.0, output_path=DEFAULT_REWORK_PATH, safe_z=SAFE_Z_HEIGHT):
    """
    최적 절삭 조건으로 rework G-code 파일을 생성합니다.

    1차 패스는 원본 깊이, 2차 패스는 모든 절대좌표 Z를 허용 공차만큼 더 깊게(Z - tolerance) 가공합니다.
    각 패스는 안전 높이 상승 → 시작 XY 이동 → 수직 하강 순서로 진입하여 소재 측면 충돌을 막습니다.

    Args:
        current_n (float): 현재 스핀들 속도 (RPM).
        current_F (float): 현재 이송 속도 (mm/min).
        optimal_n (float): 최적 스핀들 속도 (RPM).
        optimal_F (float): 최적 이송 속도 (mm/min).
        x_pos (float): 이상 진동 발생 시의 X 좌표. (현재 사용 안 됨, 필요 시 확장)
        y_pos (float): 이상 진동 발생 시의 Y 좌표. (현재 사용 안 됨, 필요 시 확장)
        z_pos (float): 이상 진동 발생 시의 Z 좌표. (현재 사용 안 됨, 필요 시 확장)
        gcode_lines (list, optional): 원본 G-code 라인 리스트. Defaults to None.
        tolerance_value (float, optional): Z축 공차 값 (mm, 0 이상). Defaults to 0.0.
        output_path (str, optional): 생성될 rework.nc 파일 경로. Defaults to nc/rework.nc.
        safe_z (float, optional): 진입/복귀 안전 높이 (mm). Defaults to 5.0.
    """
    rework_gcode = []
    tolerance_value = max(0.0, float(tolerance_value or 0.0))
    
    # gcode_lines가 None일 경우, 사용자 예시 G-code 사용
    if gcode_lines is None:
        gcode_lines = [
            "G21",      
            "G90",      
            "M3 S8000",
            "G1 Z-0.2 F200",
            "G1 Y70 F200",
            "M5",
            "G0 Z5",    
            "G0 X0 Y0 Z0",
            "M30"     
        ]
    
    original_z_line = None
    original_z_value = None
    
    # 순수 가공 경로만 추출하기 위한 리스트
    core_cutting_path_lines = []
    
    # G-code 라인에서 주석, 빈 줄, 그리고 우리가 직접 삽입할 설정/종료 명령을 제외한 핵심 가공 라인만 추출
    # G21, G90, M3 S, F, M5, G0 Z, G0 X Y Z, M30 등은 필터링
    setup_teardown_pattern = re.compile(r'^(G21|G90|M3\s*S\d+(\.\d+)?|F\s*\d+(\.\d+)?|M5|G0\s*Z\d+(\.\d+)?|G0\s*X\d+(\.\d+)?\s*Y\d+(\.\d+)?\s*Z\d+(\.\d+)?|M30)$')

    for line_raw in gcode_lines:
        line = strip_comments(line_raw).strip()
        if not line:
            continue
        
        # 설정/종료 명령이 아니면 핵심 가공 경로에 추가
        if not setup_teardown_pattern.match(line):
            core_cutting_path_lines.append(line)

    # 핵심 가공 경로에서 첫 번째 G1 Z 이동(절삭 깊이 하강) 라인 분리
    before_z_lines_cutting = []
    after_z_lines_cutting = []
    found_z = False

    for line in core_cutting_path_lines:
        words = parse_words(line)
        if not found_z and 1 in code_numbers(words, 'G') and any(letter == 'Z' for letter, _ in words):
            original_z_line = line
            original_z_value = next(value for letter, value in words if letter == 'Z')
            found_z = True
            continue
            
        if not found_z:
            before_z_lines_cutting.append(line)
        else:
            after_z_lines_cutting.append(line)

    if original_z_line is None:
        print("Error: Original G-code does not contain a G1 Z move in the main cutting path.", file=sys.stderr)
        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write("; Error: No G1 Z move found in original G-code\n")
            print(f"Created empty or error rework NC code at {output_path}", file=sys.stderr)
        except Exception as e:
            print(f"Error writing rework NC code: {str(e)}", file=sys.stderr)
        return False

    # 원본에서 Z 하강 직전에 공구가 있던 XY (없으면 원점에서 시작하는 프로그램으로 간주)
    approach = ModalState()
    for line in before_z_lines_cutting:
        approach.apply(line)
    plunge_x, plunge_y = approach.position[0], approach.position[1]

    def cutting_pass(z_offset):
        """안전 진입 → 원본 절삭 경로 (z_offset만큼 깊이 보정)"""
        modal = {'absolute': True}
        pass_lines = [
            "G21",
            "G90",
            f"G0 Z{safe_z:g}",                          # 1. 현재 위치에서 안전 높이로 먼저 상승
            f"G0 X{plunge_x:.3f} Y{plunge_y:.3f}",      # 2. 안전 높이에서 시작 XY로 수평 이동
            f"M3 S{int(optimal_n)}",                    # 3. 스핀들 가동
        ]
        for line in before_z_lines_cutting:
            pass_lines.append(_rework_line(line, optimal_n, optimal_F, z_offset, modal))
        # 4. 시작 XY에서 절삭 깊이로 수직 하강
        pass_lines.append(f"G1 Z{original_z_value + z_offset:.3f} F{int(optimal_F)}")
        for line in after_z_lines_cutting:
            pass_lines.append(_rework_line(line, optimal_n, optimal_F, z_offset, modal))
        return pass_lines
        
    # --- 첫 번째 가공 경로 생성 (원본 Z 사용) ---
    rework_gcode.extend(cutting_pass(0.0))

    # --- 중간 안전 복귀 블록 (다음 패스는 안전 높이에서 다시 진입) ---
    rework_gcode.extend([
        "M5",
        f"G0 Z{safe_z:g}",
        "" # 빈 줄 추가
    ])

    # --- 두 번째 가공 경로 생성 (Z에 허용 공차 적용: Z 값을 공차만큼 더 깊게) ---
    rework_gcode.extend(cutting_pass(-tolerance_value))

    # --- 최종 안전 복귀/원점 복귀 블록 추가 ---
    final_safe_retract_and_home = [
        "M5",
        f"G0 Z{safe_z:g}",
        "G0 X0 Y0 Z0",
        "M30"  
    ]
    rework_gcode.extend(final_safe_retract_and_home)

    # 파일 작성 (임시 파일에 쓴 뒤 교체하여 감시자가 반쯤 쓰인 파일을 읽지 않도록 함)
    try:
        tmp_path = output_path + ".tmp"
        with open(tmp_path, 'w', encoding='utf-8') as f:
            for line in rework_gcode:
                f.write(line + '\n')
        os.replace(tmp_path, output_path)
        print(f"Rework NC code created successfully at {output_path}", file=sys.stderr)
        return True
    except Exception as e:
        print(f"Error writing rework NC code to {output_path}: {str(e)}", file=sys.stderr)
        return False
