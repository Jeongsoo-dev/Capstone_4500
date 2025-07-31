import numpy as np
from scipy.optimize import fsolve
import warnings
warnings.filterwarnings('ignore')

def solve_platform_orientation(l1, l2, l3, a, b):
    """
    求解双三角形框架系统的姿态
    
    参数:
    l1, l2, l3: 三个伸缩杆的长度
    a: 框架1的边长
    b: 框架2的边长
    
    返回:
    pitch, roll: 框架2的俯仰角和滚转角（弧度）
    """
    
    # 框架1的顶点坐标（在xy平面上）
    # A在原点，BC边平行于y轴
    A = np.array([0, 0, 0])
    B = np.array([a*np.sqrt(3)/2, a/2, 0])
    C = np.array([a*np.sqrt(3)/2, -a/2, 0])
    
    # 伸缩杆1垂直于框架1，所以A'的位置确定
    A_prime = np.array([0, 0, l1])
    
    # 定义求解函数
    def equations(vars):
        # vars = [x_B', y_B', z_B', x_C', y_C', z_C']
        B_prime = np.array([vars[0], vars[1], vars[2]])
        C_prime = np.array([vars[3], vars[4], vars[5]])
        
        # 约束方程
        eq1 = np.linalg.norm(B_prime - B) - l2  # |B'B| = l2
        eq2 = np.linalg.norm(C_prime - C) - l3  # |C'C| = l3
        eq3 = np.linalg.norm(B_prime - A_prime) - b  # |B'A'| = b
        eq4 = np.linalg.norm(C_prime - A_prime) - b  # |C'A'| = b
        eq5 = np.linalg.norm(C_prime - B_prime) - b  # |C'B'| = b
        
        # 铰链约束（近似处理，允许小角度偏转）
        # 这里简化处理，主要约束在垂直方向
        AB_dir = (B - A) / np.linalg.norm(B - A)
        AC_dir = (C - A) / np.linalg.norm(C - A)
        
        # 铰链轴方向
        hinge_B_axis = np.cross(AB_dir, np.array([0, 0, 1]))
        hinge_C_axis = np.cross(AC_dir, np.array([0, 0, 1]))
        
        # 伸缩杆方向
        rod_B_dir = (B_prime - B) / np.linalg.norm(B_prime - B)
        rod_C_dir = (C_prime - C) / np.linalg.norm(C_prime - C)
        
        # 铰链约束：伸缩杆方向应该垂直于铰链轴（允许小偏差）
        eq6 = np.dot(rod_B_dir, hinge_B_axis) * 10  # 放大系数使约束更松弛
        
        return [eq1, eq2, eq3, eq4, eq5, eq6]
    
    # 初始猜测
    initial_guess = [
        B[0], B[1], l1,  # B'的初始位置
        C[0], C[1], l1   # C'的初始位置
    ]
    
    # 求解
    try:
        solution = fsolve(equations, initial_guess, xtol=1e-3, maxfev=20000)
        B_prime = np.array([solution[0], solution[1], solution[2]])
        C_prime = np.array([solution[3], solution[4], solution[5]])
        
        # 验证解的有效性，允许误差
        tolerance = 1e-2  # 允许1cm的误差
        errors = equations(solution)
        max_error = max(abs(e) for e in errors[:5])  # 只检查前5个几何约束
        
        if max_error > tolerance:
            return 0.0, 0.0  # 解不满足误差要求
            
    except:
        # 如果求解失败，返回默认值
        return 0.0, 0.0
    
    # 计算框架2的姿态
    # 框架2的三个顶点
    vertices_2 = np.array([A_prime, B_prime, C_prime])
    
    # 计算框架2的法向量
    v1 = B_prime - A_prime
    v2 = C_prime - A_prime
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)
    
    # 计算框架2的中心
    center_2 = (A_prime + B_prime + C_prime) / 3
    
    # 构建框架2的坐标系
    # x轴：从中心指向A'
    x_axis_2 = A_prime - center_2
    x_axis_2 = x_axis_2 / np.linalg.norm(x_axis_2)
    
    # z轴：法向量
    z_axis_2 = normal
    
    # y轴：右手坐标系
    y_axis_2 = np.cross(z_axis_2, x_axis_2)
    
    # 计算欧拉角（相对于世界坐标系）
    # Roll (绕x轴): atan2(R32, R33)
    # Pitch (绕y轴): -asin(R31)
    # Yaw (绕z轴): atan2(R21, R11)
    
    # 旋转矩阵（从框架2到世界坐标系）
    R = np.column_stack([x_axis_2, y_axis_2, z_axis_2]).T
    
    # 提取pitch和roll
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    
    # 调整roll角度，可能需要修正坐标系
    # 如果roll接近±180°，可能是坐标系定义问题
    if abs(roll) > np.pi/2:
        roll = roll - np.sign(roll) * np.pi
    
    return pitch, roll

import matplotlib.pyplot as plt

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
    
    # Check constraint pattern: pitch >= abs(roll) - 10
    if pitch_deg < abs(roll_deg) - 10:
        return False
    
    return True

def solve_platform_center_height(l1, l2, l3, a, b):
    """
    计算平台质心的z坐标偏移量（相对于l1）
    """
    # 复用现有的求解逻辑，但只返回质心高度
    A = np.array([0, 0, 0])
    B = np.array([a*np.sqrt(3)/2, a/2, 0])
    C = np.array([a*np.sqrt(3)/2, -a/2, 0])
    A_prime = np.array([0, 0, l1])
    
    # 定义求解函数
    def equations(vars):
        B_prime = np.array([vars[0], vars[1], vars[2]])
        C_prime = np.array([vars[3], vars[4], vars[5]])
        
        eq1 = np.linalg.norm(B_prime - B) - l2
        eq2 = np.linalg.norm(C_prime - C) - l3
        eq3 = np.linalg.norm(B_prime - A_prime) - b
        eq4 = np.linalg.norm(C_prime - A_prime) - b
        eq5 = np.linalg.norm(C_prime - B_prime) - b
        
        AB_dir = (B - A) / np.linalg.norm(B - A)
        hinge_B_axis = np.cross(AB_dir, np.array([0, 0, 1]))
        rod_B_dir = (B_prime - B) / np.linalg.norm(B_prime - B)
        eq6 = np.dot(rod_B_dir, hinge_B_axis) * 10
        
        return [eq1, eq2, eq3, eq4, eq5, eq6]
    
    initial_guess = [B[0], B[1], l1, C[0], C[1], l1]
    
    try:
        solution = fsolve(equations, initial_guess, xtol=1e-3, maxfev=20000)
        B_prime = np.array([solution[0], solution[1], solution[2]])
        C_prime = np.array([solution[3], solution[4], solution[5]])
        
        # 验证解的有效性
        tolerance = 1e-2
        errors = equations(solution)
        max_error = max(abs(e) for e in errors[:5])
        
        if max_error > tolerance:
            return 0.0
        
        # 计算平台质心
        center_z = (A_prime[2] + B_prime[2] + C_prime[2]) / 3
        return center_z - l1  # 返回相对于l1的偏移量
        
    except:
        return 0.0

def test_basic_cases():
    """
    Test some basic cases to verify the function works
    """
    print("=== 测试基本情况 ===")
    a = 1.3  # 框架1边长 1300mm
    b = 1.0  # 框架2边长 1000mm
    
    test_cases = [
        (0.7, 0.7, 0.7),  # 对称情况
        (0.75, 0.75, 0.75),  # 稍长一些
        (0.8, 0.8, 0.8),   # 更长一些
        (0.7, 0.75, 0.8),  # 非对称情况
    ]
    
    for l1, l2, l3 in test_cases:
        pitch, roll = solve_platform_orientation(l1, l2, l3, a, b)
        print(f"l1={l1*1000:.0f}, l2={l2*1000:.0f}, l3={l3*1000:.0f}mm -> "
              f"pitch={np.degrees(pitch):6.2f}°, roll={np.degrees(roll):6.2f}°")

def visualize_workspace():
    """
    Visualize the pitch and roll range for given parameters
    """
    # 先运行基本测试
    test_basic_cases()
    
    # 参数设置 (convert mm to meters for consistency)
    a = 1.3  # 框架1边长 1300mm
    b = 1.0  # 框架2边长 1000mm
    l_min, l_max = 0.55, 0.85  # 伸缩杆范围 [550, 850]mm
    
    # 采样点数
    n_samples = 20  # 每个轴向的采样点数（降低以便快速测试）
    
    # 生成采样点
    l_range = np.linspace(l_min, l_max, n_samples)
    
    # 存储结果
    pitch_values = []
    roll_values = []
    l1_values = []
    l2_values = []
    l3_values = []
    mass_center_values = []
    
    print("正在计算工作空间...")
    total_combinations = n_samples ** 3
    count = 0
    valid_count = 0
    zero_count = 0
    
    # 遍历所有可能的l1, l2, l3组合
    for l1 in l_range:
        for l2 in l_range:
            for l3 in l_range:
                count += 1
                if count % 100 == 0:
                    print(f"进度: {count}/{total_combinations} ({100*count/total_combinations:.1f}%), 有效解: {valid_count}")
                
                try:
                    pitch, roll = solve_platform_orientation(l1, l2, l3, a, b)
                    
                    # 检查解是否为默认值（表示求解失败）
                    if pitch == 0.0 and roll == 0.0:
                        zero_count += 1
                        continue
                    
                    # 将角度转换为度数
                    pitch_deg = np.degrees(pitch)
                    roll_deg = np.degrees(roll)
                    
                    # 调整roll角度到合理范围 [-180, 180]
                    while roll_deg > 180:
                        roll_deg -= 360
                    while roll_deg < -180:
                        roll_deg += 360
                    
                    # Check workspace constraints: pitch [-10, 15], roll [-15, 15], pitch >= abs(roll) - 10
                    if not validate_workspace_constraints(pitch_deg, roll_deg):
                        continue  # Skip points outside workspace constraints
                    
                    # 计算平台质心高度（z坐标）
                    mass_center_z = (l1 + solve_platform_center_height(l1, l2, l3, a, b)) * 1000  # 转换为mm
                    
                    # 计算质心与l1顶点A_prime的中点高度
                    l1_vertex_z = l1 * 1000  # A_prime点的z坐标，转换为mm
                    midpoint_z = (mass_center_z + l1_vertex_z) / 2
                    
                    # 检查质心与l1顶点中点的高度约束 [680, 720]mm
                    if 680 <= midpoint_z <= 720:
                        pitch_values.append(pitch_deg)
                        roll_values.append(roll_deg)
                        l1_values.append(l1 * 1000)  # convert back to mm
                        l2_values.append(l2 * 1000)
                        l3_values.append(l3 * 1000)
                        mass_center_values.append(midpoint_z)  # 现在存储中点高度而不是质心高度
                        valid_count += 1
                        
                        if count <= 10:  # 打印前几个解用于调试
                            print(f"  解: l=[{l1*1000:.0f},{l2*1000:.0f},{l3*1000:.0f}]mm -> pitch={pitch_deg:.1f}°, roll={roll_deg:.1f}°, 质心高度={mass_center_z:.0f}mm, 中点高度={midpoint_z:.0f}mm")
                    elif count <= 20:  # 打印一些被过滤的解用于调试
                        print(f"  过滤: l=[{l1*1000:.0f},{l2*1000:.0f},{l3*1000:.0f}]mm -> 中点高度={midpoint_z:.0f}mm (超出范围)")
                except Exception as e:
                    # 跳过无法求解的组合
                    continue
    
    print(f"总计算次数: {count}, 零解次数: {zero_count}, 有效解: {valid_count}")
    
    if not pitch_values:
        print("未找到有效解！请检查参数设置。")
        return
    
    print(f"找到 {len(pitch_values)} 个有效解")
    print(f"Pitch 范围: {min(pitch_values):.2f}° 到 {max(pitch_values):.2f}°")
    print(f"Roll 范围: {min(roll_values):.2f}° 到 {max(roll_values):.2f}°")
    print(f"中点高度范围: {min(mass_center_values):.1f}mm 到 {max(mass_center_values):.1f}mm")
    
    # 创建可视化
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Stewart Platform Workspace Analysis\n(a=1300mm, b=1000mm, l=[550,850]mm, midpoint_z=[680,720]mm)\nConstraints: Pitch[-10°,15°], Roll[-15°,15°], pitch >= abs(roll) - 10', fontsize=12)
    
    # 1. Pitch-Roll工作空间
    axes[0, 0].scatter(roll_values, pitch_values, c='blue', alpha=0.6, s=1)
    axes[0, 0].set_xlabel('Roll (degrees)')
    axes[0, 0].set_ylabel('Pitch (degrees)')
    axes[0, 0].set_title('Pitch-Roll Workspace')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    
    # 2. Pitch分布直方图
    axes[0, 1].hist(pitch_values, bins=30, alpha=0.7, color='green', edgecolor='black')
    axes[0, 1].set_xlabel('Pitch (degrees)')
    axes[0, 1].set_ylabel('Frequency')
    axes[0, 1].set_title('Pitch Distribution')
    axes[0, 1].grid(True, alpha=0.3)
    
    # 3. Roll分布直方图
    axes[1, 0].hist(roll_values, bins=30, alpha=0.7, color='red', edgecolor='black')
    axes[1, 0].set_xlabel('Roll (degrees)')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].set_title('Roll Distribution')
    axes[1, 0].grid(True, alpha=0.3)
    
    # 4. 极坐标表示
    angles = np.arctan2(pitch_values, roll_values)
    radii = np.sqrt(np.array(pitch_values)**2 + np.array(roll_values)**2)
    
    ax_polar = plt.subplot(2, 2, 4, projection='polar')
    ax_polar.scatter(angles, radii, c='purple', alpha=0.6, s=1)
    ax_polar.set_title('Polar View of Workspace')
    ax_polar.set_ylim(0, max(radii) * 1.1)
    
    plt.tight_layout()
    plt.show()
    
    # 打印统计信息
    print("\n=== 工作空间统计 ===")
    print(f"Pitch: {min(pitch_values):.2f}° ~ {max(pitch_values):.2f}° (范围: {max(pitch_values)-min(pitch_values):.2f}°)")
    print(f"Roll:  {min(roll_values):.2f}° ~ {max(roll_values):.2f}° (范围: {max(roll_values)-min(roll_values):.2f}°)")
    print(f"中点高度: {min(mass_center_values):.1f}mm ~ {max(mass_center_values):.1f}mm (范围: {max(mass_center_values)-min(mass_center_values):.1f}mm)")
    print(f"最大总角度偏移: {max(radii):.2f}°")

def main():
    # 运行工作空间可视化
    visualize_workspace()

if __name__ == "__main__":
    main()
