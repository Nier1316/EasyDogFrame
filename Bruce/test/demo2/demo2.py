import os
import torch

# ==================== 路径处理 ====================
# 基于脚本所在目录定位项目根目录和模型文件
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
MODEL_PATH = os.path.join(PROJECT_ROOT, "config", "policy.pt")

# ==================== 加载模型 ====================
# torch.jit.load: 加载 TorchScript 序列化的模型，无需原始 Python 类定义
model = torch.jit.load(MODEL_PATH)
model.eval()
print(f"模型已加载: {MODEL_PATH}")
print(f"模型类型: {type(model).__name__}")

# ==================== 生成输入数据 ====================
# 输入维度: 6 帧观测历史 × 58 维/帧 = 348 维
#   每帧 58 维的构成及各段实际数值范围 (经缩放因子归一化后):
#     0-2   (3):  IMU 陀螺仪角速度 ×0.25      → 范围 [-2, 2]
#     3-5   (3):  重力在机体坐标系的投影        → 范围 [-1, 1]
#     6-9   (4):  控制指令 ×[2, 2, 0.25, 2]  → 范围 [-2, 2]
#    10-25 (16):  关节位置误差 ×1.0            → 范围 [-0.5, 0.5]
#    26-41 (16):  关节速度 ×0.05              → 范围 [-2, 2]
#    42-57 (16):  上一帧动作                   → 范围 [-0.25, 0.25]

def build_frame():
    """构建一帧 58 维观测，各段数值范围与真实数据对齐"""
    frame = torch.zeros(58)
    frame[0:3]   = torch.randn(3) * 1.0          # imu_gyro:      均值0, std~1  → 大多在 [-2, 2]
    frame[3:6]   = torch.randn(3) * 0.3          # gravity:       均值0, std~0.3 → 大多在 [-1, 1]
    frame[3] += 0.0; frame[4] += 0.0; frame[5] -= 0.95  # 偏置: 近似 [0, 0, -1]
    frame[6:10]  = torch.randn(4) * 0.8          # commands:      std~0.8 → 大多在 [-2, 2]
    frame[10:26] = torch.randn(16) * 0.2         # dof_pos_err:   std~0.2 → 大多在 [-0.5, 0.5]
    frame[26:42] = torch.randn(16) * 0.8         # dof_vel:       std~0.8 → 大多在 [-2, 2]
    frame[42:58] = torch.randn(16) * 0.1         # actions:       std~0.1 → 大多在 [-0.25, 0.25]
    return frame

# 构建 6 帧并展平为 (1, 348)，每帧加微小扰动模拟时序变化
obs_buffer = torch.stack([build_frame() + torch.randn(58) * 0.02 for _ in range(6)])
dummy_input = obs_buffer.flatten().unsqueeze(0)  # (348,) → (1, 348)

print(f"输入形状: {dummy_input.shape}")
print(f"各段数值范围检查:")
print(f"  gyro[0:3]:     [{dummy_input[0,0:3].min().item():+.3f}, {dummy_input[0,0:3].max().item():+.3f}]")
print(f"  gravity[3:6]:  [{dummy_input[0,3:6].min().item():+.3f}, {dummy_input[0,3:6].max().item():+.3f}]")
print(f"  cmds[6:10]:    [{dummy_input[0,6:10].min().item():+.3f}, {dummy_input[0,6:10].max().item():+.3f}]")
print(f"  dof_pos[10:26]:[{dummy_input[0,10:26].min().item():+.3f}, {dummy_input[0,10:26].max().item():+.3f}]")
print(f"  dof_vel[26:42]:[{dummy_input[0,26:42].min().item():+.3f}, {dummy_input[0,26:42].max().item():+.3f}]")
print(f"  actions[42:58]:[{dummy_input[0,42:58].min().item():+.3f}, {dummy_input[0,42:58].max().item():+.3f}]")

# ==================== 推理 ====================
# torch.no_grad(): 禁用梯度计算，减少内存占用，加速推理
with torch.no_grad():
    output = model(dummy_input)

print(f"输出形状: {output.shape}")  # 应为 (1, 16)，即 16 个关节的动作偏移
print(f"输出数值:\n{output}")
