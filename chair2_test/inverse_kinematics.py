import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

class DualTriangleKinematics:
    def __init__(self):
        # 框架参数
        self.L1 = 1300  # 框架1边长
        self.L2 = 1000  # 框架2边长
        
        # 伸缩杆长度范围
        self.l_min = 550
        self.l_max = 850
        
        # 高度范围
        self.h_min = 650
        self.h_max = 690
        
        # 框架1顶点坐标（水平放置）
        # A1(l1顶点), B1(l2顶点), C1(l3顶点)
        self.A1 = np.array([self.L1/(2*np.sqrt(3)), 0, 0])
        self.B1 = np.array([-self.L1/(2*np.sqrt(3)), self.L1/2, 0])
        self.C1 = np.array([-self.L1/(2*np.sqrt(3)), -self.L1/2, 0])
        
        # 框架2相对顶点坐标（未旋转时）
        self.A2_rel = np.array([self.L2/(2*np.sqrt(3)), 0, 0])
        self.B2_rel = np.array([-self.L2/(2*np.sqrt(3)), self.L2/2, 0])
        self.C2_rel = np.array([-self.L2/(2*np.sqrt(3)), -self.L2/2, 0])
        
    def rotation_matrix(self, roll, pitch, yaw=0):
        """计算旋转矩阵（ZYX顺序）"""
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        return R_z @ R_y @ R_x
    
    def get_frame2_vertices(self, l3, roll, pitch):
        """计算框架2的顶点位置"""
        # C2位置由l3确定（垂直固定）
        C2 = self.C1 + np.array([0, 0, l3])
        
        # 计算框架2的重心位置
        # 等边三角形重心到顶点距离为边长/√3
        R = self.rotation_matrix(roll, pitch)
        
        # 将相对坐标旋转并平移到C2
        A2 = C2 + R @ (self.A2_rel - self.C2_rel)
        B2 = C2 + R @ (self.B2_rel - self.C2_rel)
        
        return A2, B2, C2
    
    def get_center_height(self, l3, roll, pitch):
        """计算框架2重心高度"""
        A2, B2, C2 = self.get_frame2_vertices(l3, roll, pitch)
        center = (A2 + B2 + C2) / 3
        return center[2]
    
    def check_constraints(self, l1, l2, l3, roll, pitch):
        """检查约束条件"""
        # 长度约束
        if not (self.l_min <= l1 <= self.l_max and 
                self.l_min <= l2 <= self.l_max and 
                self.l_min <= l3 <= self.l_max):
            return False
        
        # 高度约束
        h = self.get_center_height(l3, roll, pitch)
        if not (self.h_min <= h <= self.h_max):
            return False
        
        # 几何约束：检查伸缩杆长度是否匹配
        A2, B2, C2 = self.get_frame2_vertices(l3, roll, pitch)
        
        dist_A = np.linalg.norm(A2 - self.A1)
        dist_B = np.linalg.norm(B2 - self.B1)
        
        tolerance = 5.0  # 允许的误差
        if abs(dist_A - l1) > tolerance or abs(dist_B - l2) > tolerance:
            return False
        
        return True
    
    def solve_inverse_kinematics(self, target_roll, target_pitch):
        """求解逆运动学"""
        def objective(x):
            l1, l2, l3 = x
            A2, B2, C2 = self.get_frame2_vertices(l3, target_roll, target_pitch)
            
            # 计算实际距离与目标长度的误差
            error1 = (np.linalg.norm(A2 - self.A1) - l1)**2
            error2 = (np.linalg.norm(B2 - self.B1) - l2)**2
            
            # 高度约束的软约束
            h = self.get_center_height(l3, target_roll, target_pitch)
            h_penalty = 0
            if h < self.h_min:
                h_penalty = 100 * (self.h_min - h)**2
            elif h > self.h_max:
                h_penalty = 100 * (h - self.h_max)**2
            
            return error1 + error2 + h_penalty
        
        # 初始猜测（基于已知的零姿态解）
        x0 = [735, 735, 670]
        
        # 边界约束
        bounds = [(self.l_min, self.l_max), 
                  (self.l_min, self.l_max), 
                  (self.l_min, self.l_max)]
        
        # 优化求解
        result = minimize(objective, x0, bounds=bounds, method='L-BFGS-B')
        
        if result.fun < 1.0:  # 误差阈值
            l1, l2, l3 = result.x
            # 验证约束
            if self.check_constraints(l1, l2, l3, target_roll, target_pitch):
                return l1, l2, l3, True
        
        return None, None, None, False
    
    def generate_lookup_table(self, roll_range, pitch_range, step=5):
        """生成查找表"""
        rolls = np.arange(roll_range[0], roll_range[1] + step, step)
        pitches = np.arange(pitch_range[0], pitch_range[1] + step, step)
        
        lookup_table = []
        
        for roll_deg in rolls:
            for pitch_deg in pitches:
                roll_rad = np.radians(roll_deg)
                pitch_rad = np.radians(pitch_deg)
                
                l1, l2, l3, success = self.solve_inverse_kinematics(roll_rad, pitch_rad)
                
                if success:
                    h = self.get_center_height(l3, roll_rad, pitch_rad)
                    lookup_table.append({
                        'roll_deg': roll_deg,
                        'pitch_deg': pitch_deg,
                        'l1': round(l1, 1),
                        'l2': round(l2, 1),
                        'l3': round(l3, 1),
                        'height': round(h, 1)
                    })
                    print(f"Roll: {roll_deg}°, Pitch: {pitch_deg}° -> "
                          f"l1: {l1:.1f}, l2: {l2:.1f}, l3: {l3:.1f}, h: {h:.1f}")
        
        return pd.DataFrame(lookup_table)
    
    def visualize_configuration(self, l1, l2, l3, roll, pitch):
        """可视化机构配置"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制框架1
        frame1_points = np.array([self.A1, self.B1, self.C1, self.A1])
        ax.plot(frame1_points[:, 0], frame1_points[:, 1], frame1_points[:, 2], 
                'b-', linewidth=2, label='Frame 1')
        
        # 计算并绘制框架2
        A2, B2, C2 = self.get_frame2_vertices(l3, roll, pitch)
        frame2_points = np.array([A2, B2, C2, A2])
        ax.plot(frame2_points[:, 0], frame2_points[:, 1], frame2_points[:, 2], 
                'r-', linewidth=2, label='Frame 2')
        
        # 绘制伸缩杆
        ax.plot([self.A1[0], A2[0]], [self.A1[1], A2[1]], [self.A1[2], A2[2]], 
                'g--', linewidth=1, label=f'l1={l1:.1f}')
        ax.plot([self.B1[0], B2[0]], [self.B1[1], B2[1]], [self.B1[2], B2[2]], 
                'g--', linewidth=1, label=f'l2={l2:.1f}')
        ax.plot([self.C1[0], C2[0]], [self.C1[1], C2[1]], [self.C1[2], C2[2]], 
                'g--', linewidth=1, label=f'l3={l3:.1f}')
        
        # 绘制重心
        center2 = (A2 + B2 + C2) / 3
        ax.scatter(*center2, color='red', s=100, label=f'Center h={center2[2]:.1f}')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        ax.set_title(f'Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°')
        
        # 设置相等的轴比例
        max_range = 1000
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_zlim(0, 2*max_range)
        
        plt.show()

# 使用示例
if __name__ == "__main__":
    # 创建运动学求解器
    kinematics = DualTriangleKinematics()
    
    # 验证零姿态配置
    print("验证零姿态配置:")
    roll, pitch = 0, 0
    l1, l2, l3 = 735, 735, 670
    h = kinematics.get_center_height(l3, roll, pitch)
    print(f"l1={l1}, l2={l2}, l3={l3} -> height={h:.1f}")
    
    # 生成查找表
    print("\n生成查找表:")
    lookup_table = kinematics.generate_lookup_table(
        roll_range=(-15, 15),
        pitch_range=(-15, 15),
        step=0.5
    )
    
    # 保存查找表
    lookup_table.to_csv('lookup_table.csv', index=False)
    print(f"\n查找表已保存，共 {len(lookup_table)} 个有效配置")
    
    # 显示查找表
    print("\n查找表内容:")
    print(lookup_table)
    
    # 可视化几个典型配置
    print("\n可视化典型配置:")
    # 零姿态
    kinematics.visualize_configuration(735, 735, 670, 0, 0)
    
    # 从查找表中选择一个配置
    if len(lookup_table) > 5:
        row = lookup_table.iloc[5]
        kinematics.visualize_configuration(
            row['l1'], row['l2'], row['l3'],
            np.radians(row['roll_deg']), np.radians(row['pitch_deg'])
        )
