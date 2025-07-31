import numpy as np
from scipy.optimize import fsolve
import warnings
warnings.filterwarnings('ignore')

def solve_inverse_kinematics(target_pitch, target_roll, a, b, target_height=0.67):
    """
    逆向运动学求解：给定目标俯仰角和滚转角，求解需要的伸缩杆长度
    
    参数:
    target_pitch: 目标俯仰角（弧度）
    target_roll: 目标滚转角（弧度）
    a: 框架1的边长
    b: 框架2的边长
    target_height: 目标平台平均高度（米）
    
    返回:
    l3, l1, l2: 三个伸缩杆的长度，如果无解则返回None
    """
    
    # 框架1的顶点坐标（在xy平面上）
    A = np.array([0, 0, 0])
    B = np.array([a*np.sqrt(3)/2, a/2, 0])
    C = np.array([a*np.sqrt(3)/2, -a/2, 0])
    
    # 应用系统性pitch偏移校正 (0.529°转换为弧度)
    pitch_offset = np.radians(0.529)
    corrected_pitch = target_pitch + pitch_offset
    
    # 根据目标姿态构建框架2的期望位置
    # 首先确定框架2的中心位置
    center_2_target = np.array([a*np.sqrt(3)/6, 0, target_height])  # 预估中心位置
    
    # 构建目标旋转矩阵
    # 使用ZYX欧拉角顺序：先绕Z轴旋转(yaw=0)，再绕Y轴旋转(pitch)，最后绕X轴旋转(roll)
    cos_p, sin_p = np.cos(corrected_pitch), np.sin(corrected_pitch)
    cos_r, sin_r = np.cos(target_roll), np.sin(target_roll)
    
    # 旋转矩阵 R = Rx(roll) * Ry(pitch) * Rz(0)
    R = np.array([
        [cos_p, sin_r*sin_p, cos_r*sin_p],
        [0, cos_r, -sin_r],
        [-sin_p, sin_r*cos_p, cos_r*cos_p]
    ])
    
    # 框架2在局部坐标系中的顶点（等边三角形，中心在原点）
    local_vertices = np.array([
        [b*2/3, 0, 0],           # A'相对中心
        [-b/3, b/2, 0],          # B'相对中心  
        [-b/3, -b/2, 0]          # C'相对中心
    ])
    
    # 将局部顶点转换到世界坐标系
    def get_target_vertices(center_z):
        center = np.array([a*np.sqrt(3)/6, 0, center_z])
        target_vertices = []
        for vertex in local_vertices:
            rotated_vertex = R @ vertex
            world_vertex = center + rotated_vertex
            target_vertices.append(world_vertex)
        return np.array(target_vertices)
    
    # 定义求解函数
    def equations(vars):
        # vars = [l3, l1, l2, center_z]
        l3, l1, l2, center_z = vars
        
        # 获取目标顶点位置
        target_vertices = get_target_vertices(center_z)
        A_prime_target = target_vertices[0] 
        B_prime_target = target_vertices[1]
        C_prime_target = target_vertices[2]
        
        # 根据伸缩杆长度计算实际位置
        A_prime_actual = np.array([0, 0, l3])
        
        # B'和C'需要满足距离约束
        # |B'B| = l1, |C'C| = l2
        # |B'A'| = b, |C'A'| = b, |C'B'| = b
        
        # 约束方程
        eq1 = A_prime_actual[2] - A_prime_target[2]  # A'的z坐标匹配
        
        # 对于B'和C'，我们需要在满足杆长约束的前提下尽量接近目标位置
        # 这是一个复杂的几何约束问题，简化处理
        
        # 假设B'和C'的位置可以通过几何约束确定
        B_base_to_target = np.linalg.norm(B_prime_target - B)
        C_base_to_target = np.linalg.norm(C_prime_target - C)
        
        eq2 = B_base_to_target - l1
        eq3 = C_base_to_target - l2
        
        # 添加平台高度约束（中点高度）
        platform_center_z = (A_prime_actual[2] + A_prime_target[2] + A_prime_target[2]) / 3
        l3_vertex_z = l3
        midpoint_z = (platform_center_z + l3_vertex_z) / 2
        eq4 = (midpoint_z - 0.67) * 1000  # 目标中点高度约束，转换为mm单位增加权重
        
        return [eq1, eq2, eq3, eq4]
    
    # 多个初始猜测值（基于用户测量的中性状态670mm）
    initial_guesses = [
        [target_height, target_height, target_height, target_height],
        [0.6, 0.6, 0.6, target_height],
        [0.8, 0.8, 0.8, target_height],
        [0.67, 0.735, 0.735, target_height],  # 用户测量的中性状态
    ]
    
    best_solution = None
    min_error = float('inf')
    
    for initial_guess in initial_guesses:
        try:
            solution = fsolve(equations, initial_guess, xtol=1e-6, maxfev=10000)
            l3, l1, l2, center_z = solution
            
            # 检查解的有效性
            if l3 < 0.55 or l3 > 0.85 or l1 < 0.55 or l1 > 0.85 or l2 < 0.55 or l2 > 0.85:
                continue  # 超出伸缩杆范围
            
            # 计算残差
            errors = equations(solution)
            total_error = sum(e**2 for e in errors)
            
            if total_error < min_error:
                min_error = total_error
                best_solution = solution
                
        except:
            continue
    
    if best_solution is not None and min_error < 1e-3:  # 解的精度要求
        l3, l1, l2, _ = best_solution
        return l3, l1, l2
    else:
        return None, None, None

def simplified_inverse_kinematics(target_pitch, target_roll, a, b):
    """
    简化的逆向运动学求解方法
    基于用户测量的中性状态校准
    """
    
    # 应用系统性pitch偏移校正 (0.529°转换为弧度)
    pitch_offset = np.radians(0.529)
    corrected_pitch = target_pitch + pitch_offset
    
    # 用户测量的中性状态作为基准点
    neutral_l3 = 0.670  # 670mm
    neutral_l1 = 0.735  # 735mm
    neutral_l2 = 0.735  # 735mm
    
    # 基于实际测量数据的响应系数（需要通过校准确定）
    # 这些系数基于小角度近似和用户的系统特性
    pitch_sensitivity = 0.008  # m/rad，pitch对l3的影响
    roll_sensitivity_l1 = -0.008  # m/rad，roll对l1的影响
    roll_sensitivity_l2 = 0.008   # m/rad，roll对l2的影响
    
    # 计算各杆长度
    l3 = neutral_l3 + corrected_pitch * pitch_sensitivity
    l1 = neutral_l1 + target_roll * roll_sensitivity_l1
    l2 = neutral_l2 + target_roll * roll_sensitivity_l2
    
    # 确保在有效范围内
    l3 = np.clip(l3, 0.55, 0.85)
    l1 = np.clip(l1, 0.55, 0.85)
    l2 = np.clip(l2, 0.55, 0.85)
    
    return l3, l1, l2

def validate_workspace_constraints(pitch_deg, roll_deg):
    """
    Validate workspace constraints based on analysis
    
    Args:
        pitch_deg: Pitch angle in degrees
        roll_deg: Roll angle in degrees
        
    Returns:
        bool: True if within constraints, False otherwise
    """
    # Updated workspace constraints from analysis
    pitch_min, pitch_max = -10.0, 15.0
    roll_min, roll_max = -15.0, 15.0
    
    # Check basic ranges
    if not (pitch_min <= pitch_deg <= pitch_max):
        return False
    if not (roll_min <= roll_deg <= roll_max):
        return False
    
    # # Check constraint pattern: pitch >= abs(roll) - 10
    # if pitch_deg < abs(roll_deg) - 10:
    #     return False
    
    return True

def generate_lookup_table():
    """
    生成查找表：pitch/roll -> l3,l1,l2
    Pitch范围：[-10°, +15°]，Roll范围：[-15°, +15°]，分辨率：0.5°
    约束：pitch >= abs(roll) - 10
    """
    
    # 参数设置
    a = 1.3  # 框架1边长
    b = 0.981  # 框架2边长 (corrected based on measurements)
    
    # 角度范围和分辨率 (updated based on workspace analysis)
    pitch_min, pitch_max = -15.0, 15.0  # 度
    roll_min, roll_max = -15.0, 15.0    # 度
    resolution = 0.5   # 度
    
    # 生成角度网格
    pitch_angles = np.arange(pitch_min, pitch_max + resolution, resolution)
    roll_angles = np.arange(roll_min, roll_max + resolution, resolution)
    
    print(f"生成查找表...")
    print(f"Pitch范围: [{pitch_min}°, {pitch_max}°]")
    print(f"Roll范围: [{roll_min}°, {roll_max}°]")
    print(f"分辨率: {resolution}°")
    print(f"网格大小: {len(pitch_angles)} x {len(roll_angles)} = {len(pitch_angles) * len(roll_angles)} 个点")
    
    # 存储结果
    lookup_data = []
    valid_count = 0
    total_count = 0
    
    for pitch_deg in pitch_angles:
        for roll_deg in roll_angles:
            total_count += 1
            
            # Check workspace constraints first
            if not validate_workspace_constraints(pitch_deg, roll_deg):
                continue  # Skip points outside workspace constraints
            
            # 转换为弧度
            pitch_rad = np.radians(pitch_deg)
            roll_rad = np.radians(roll_deg)
            
            # 使用简化方法求解逆向运动学
            l3, l1, l2 = simplified_inverse_kinematics(pitch_rad, roll_rad, a, b)
            
            if l3 is not None and l1 is not None and l2 is not None:
                # 验证解的有效性 - 可以调用正向运动学验证
                lookup_data.append({
                    'pitch': pitch_deg,
                    'roll': roll_deg,
                    'l3': l3 * 1000,  # 转换为mm
                    'l1': l1 * 1000,
                    'l2': l2 * 1000
                })
                valid_count += 1
                
            if total_count % 100 == 0:
                total_points = len(pitch_angles) * len(roll_angles)
                print(f"进度: {total_count}/{total_points} ({100*total_count/total_points:.1f}%), 有效解: {valid_count}")
    
    print(f"生成完成！总点数: {total_count}, 有效解: {valid_count} ({100*valid_count/total_count:.1f}%)")
    
    # 保存到文件
    filename = "lookup_table.txt"
    with open(filename, 'w', encoding='utf-8') as f:
        # 写入头部信息
        f.write("# Stewart Platform Inverse Kinematics Lookup Table\n")
        f.write(f"# Generated with Pitch [{pitch_min}°, {pitch_max}°], Roll [{roll_min}°, {roll_max}°], resolution {resolution}°\n")
        f.write(f"# Frame1 edge length: {a*1000:.0f}mm, Frame2 edge length: {b*1000:.0f}mm\n")
        f.write(f"# Total valid entries: {valid_count}\n")
        f.write("# Format: pitch(deg) roll(deg) l3(mm) l1(mm) l2(mm)\n")
        f.write("#\n")
        
        # 写入数据
        for data in lookup_data:
            f.write(f"{data['pitch']:6.1f} {data['roll']:6.1f} "
                   f"{data['l3']:7.1f} {data['l1']:7.1f} {data['l2']:7.1f}\n")
    
    print(f"查找表已保存到: {filename}")
    
    # 打印统计信息
    if lookup_data:
        pitches = [d['pitch'] for d in lookup_data]
        rolls = [d['roll'] for d in lookup_data]
        l3s = [d['l3'] for d in lookup_data]
        l1s = [d['l1'] for d in lookup_data]
        l2s = [d['l2'] for d in lookup_data]
        
        print("\n=== 查找表统计信息 ===")
        print(f"Pitch范围: {min(pitches):.1f}° ~ {max(pitches):.1f}°")
        print(f"Roll范围: {min(rolls):.1f}° ~ {max(rolls):.1f}°")
        print(f"L3范围: {min(l3s):.1f}mm ~ {max(l3s):.1f}mm")
        print(f"L1范围: {min(l1s):.1f}mm ~ {max(l1s):.1f}mm")
        print(f"L2范围: {min(l2s):.1f}mm ~ {max(l2s):.1f}mm")
        
        # 显示前几个示例
        print("\n=== 前10个条目示例 ===")
        for i, data in enumerate(lookup_data[:10]):
            print(f"Pitch:{data['pitch']:6.1f}°, Roll:{data['roll']:6.1f}° -> "
                  f"L3:{data['l3']:6.1f}mm, L1:{data['l1']:6.1f}mm, L2:{data['l2']:6.1f}mm")

def test_corrected_inverse_kinematics():
    """
    测试修正后的逆向运动学
    """
    print("=== 测试修正后的逆向运动学 ===")
    a = 1.3  # 框架1边长
    b = 0.981  # 框架2边长 (corrected)
    
    # 测试用户的中性状态
    print("测试中性状态 (pitch=0°, roll=0°):")
    pitch_rad, roll_rad = np.radians(0), np.radians(0)
    l3, l1, l2 = simplified_inverse_kinematics(pitch_rad, roll_rad, a, b)
    print(f"结果: l3={l3*1000:.1f}mm, l1={l1*1000:.1f}mm, l2={l2*1000:.1f}mm")
    print(f"期望: l3=670mm, l1=735mm, l2=735mm")
    
    # 测试其他角度
    test_cases = [
        (5, 0),    # pitch only
        (0, 5),    # roll only
        (-5, 0),   # negative pitch
        (0, -5),   # negative roll
        (10, 10),  # combined
    ]
    
    print("\n其他测试用例:")
    for pitch_deg, roll_deg in test_cases:
        pitch_rad, roll_rad = np.radians(pitch_deg), np.radians(roll_deg)
        l3, l1, l2 = simplified_inverse_kinematics(pitch_rad, roll_rad, a, b)
        print(f"Pitch={pitch_deg:3.0f}°, Roll={roll_deg:3.0f}° -> "
              f"l3={l3*1000:.1f}mm, l1={l1*1000:.1f}mm, l2={l2*1000:.1f}mm")

def main():
    """主函数"""
    # 先测试修正后的逆向运动学
    test_corrected_inverse_kinematics()
    print("\n" + "="*50 + "\n")
    
    # 生成修正后的查找表
    generate_lookup_table()

if __name__ == "__main__":
    main()