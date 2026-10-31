import re
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_REWORK_PATH = os.path.join(BASE_DIR, "nc", "rework.nc")

def create_rework_nc(current_n, current_F, optimal_n, optimal_F, x_pos, y_pos, z_pos, gcode_lines=None, tolerance_value=0.0, output_path=DEFAULT_REWORK_PATH):
    """
    최적 절삭 조건으로 rework G-code 파일을 생성합니다.

    Args:
        current_n (float): 현재 스핀들 속도 (RPM).
        current_F (float): 현재 이송 속도 (mm/min).
        optimal_n (float): 최적 스핀들 속도 (RPM).
        optimal_F (float): 최적 이송 속도 (mm/min).
        x_pos (float): 이상 진동 발생 시의 X 좌표. (현재 사용 안 됨, 필요 시 확장)
        y_pos (float): 이상 진동 발생 시의 Y 좌표. (현재 사용 안 됨, 필요 시 확장)
        z_pos (float): 이상 진동 발생 시의 Z 좌표. (현재 사용 안 됨, 필요 시 확장)
        gcode_lines (list, optional): 원본 G-code 라인 리스트. Defaults to None.
        tolerance_value (float, optional): Z축 공차 값. Defaults to 0.0.
        output_path (str, optional): 생성될 rework.nc 파일 경로. Defaults to "rework.nc".
    """
    rework_gcode = []
    
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
        line = line_raw.strip()
        if not line or line.startswith(';') or line.startswith('('):
            continue
        
        # 설정/종료 명령이 아니면 핵심 가공 경로에 추가
        if not setup_teardown_pattern.match(line):
            core_cutting_path_lines.append(line)

    # 핵심 가공 경로에서 Z 이동 라인 분리
    before_z_lines_cutting = []
    after_z_lines_cutting = []
    found_z = False

    for line in core_cutting_path_lines:
        if ('G1' in line or 'G01' in line) and 'Z' in line and not found_z:
            z_match = re.search(r'Z(-?\d*\.?\d+)', line)
            if z_match:
                original_z_line = line
                original_z_value = float(z_match.group(1))
                found_z = True
                continue
            
        if not found_z:
            before_z_lines_cutting.append(line)
        else:
            after_z_lines_cutting.append(line)

    if original_z_line is None:
        print("Error: Original G-code does not contain a G1 Z move in the main cutting path.", file=sys.stderr)
        try:
            with open(output_path, 'w') as f:
                 f.write("; Error: No G1 Z move found in original G-code\n")
            print(f"Created empty or error rework NC code at {output_path}", file=sys.stderr)
        except Exception as e:
             print(f"Error writing rework NC code: {str(e)}", file=sys.stderr)
        return False

    def _apply_optimal_params(line, n_val, f_val):
        temp_line = line
        temp_line = re.sub(r'M3\s*S\s*\d+(\.\d+)?', f'M3 S{int(n_val)}', temp_line)
        temp_line = re.sub(r'F\s*\d+(\.\d+)?', f'F{int(f_val)}', temp_line)
        return temp_line
        
    # --- 첫 번째 가공 경로 생성 (원본 Z 사용) ---
    rework_gcode.extend([
        "G21",      
        "G90",      
        f"M3 S{int(optimal_n)}",
        f"G1 Z{original_z_value:.3f} F{int(optimal_F)}" # Z-line with optimal F, original Z
    ])
    # Z-line은 위에서 명시적으로 추가했으므로, before/after Z 리스트에서 Z-line 자체는 건너뜁니다.

    for line in before_z_lines_cutting:
        rework_gcode.append(_apply_optimal_params(line, optimal_n, optimal_F))
    
    for line in after_z_lines_cutting:
        rework_gcode.append(_apply_optimal_params(line, optimal_n, optimal_F))
        
    # --- 중간 안전 복귀 블록 추가 ---
    rework_gcode.extend([
        "M5",
        "G0 Z5",    
        "G0 X0 Y0 Z0", 
        "" # 빈 줄 추가
    ])

    # --- 두 번째 가공 경로 생성 (Z에 허용 공차 적용) ---
    new_z_value = original_z_value - tolerance_value # Z 값을 공차만큼 더 깊게 (뺄셈)
    rework_gcode.extend([
        "G21",      
        "G90",      
        f"M3 S{int(optimal_n)}",
        f"G1 Z{new_z_value:.3f} F{int(optimal_F)}" # Z-line with optimal F, tolerance Z
    ])

    for line in before_z_lines_cutting:
        rework_gcode.append(_apply_optimal_params(line, optimal_n, optimal_F))
    
    for line in after_z_lines_cutting:
        rework_gcode.append(_apply_optimal_params(line, optimal_n, optimal_F))

    # --- 최종 안전 복귀/원점 복귀 블록 추가 ---
    final_safe_retract_and_home = [
        "M5",
        "G0 Z5",  
        "G0 X0 Y0 Z0",
        "M30"  
    ]
    rework_gcode.extend(final_safe_retract_and_home)

    # 파일 작성
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            for line in rework_gcode:
                f.write(line + '\n')
        print(f"Rework NC code created successfully at {output_path}", file=sys.stderr)
        return True
    except Exception as e:
        print(f"Error writing rework NC code to {output_path}: {str(e)}", file=sys.stderr)
        return False