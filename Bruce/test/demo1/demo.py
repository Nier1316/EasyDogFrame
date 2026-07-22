import os
import yaml
import torch
import mujoco
import mujoco.viewer
import time
import numpy as np
import pygame


# ==================== 手柄映射 ====================
# Xbox 手柄 (Linux/pygame):
#   轴: 0=左摇杆X,  1=左摇杆Y(前推为负),  2=右摇杆X,  3=右摇杆Y(前推为负)
#   按键: 0=A, 1=B, 3=X, 4=Y, 6=LB, 7=RB, ...
#   左摇杆 Y(轴1) → vx (前进速度), 翻转符号
#   左摇杆 X(轴0) → vy (横向速度)
#   右摇杆 X(轴2) → heading 目标 (偏航角)
#   A(0) → 站定 (zero cmd)   B(1) → 阻尼模式

DEAD_ZONE = 0.08


def scale_axis(val, dead_zone=DEAD_ZONE):
    """摇杆死区处理: 过滤小幅度抖动，并将有效范围线性映射到 [0, 1]"""
    if abs(val) < dead_zone:
        return 0.0
    return (val - np.sign(val) * dead_zone) / (1.0 - dead_zone)


# ==================== 路径处理 ====================
# 获取脚本所在目录的绝对路径，所有资源文件路径都基于此目录解析，
# 确保在任何电脑、任何工作目录下运行都不会因路径问题报错。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 项目根目录 (demo1 的父目录)
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)


# ==================== 加载配置 ====================
# 优先查找脚本同目录下的 config_bigdog.yaml，否则回退到项目根目录的 config/config.yaml
_config_local = os.path.join(SCRIPT_DIR, "config_bigdog.yaml")
_config_shared = os.path.join(PROJECT_ROOT, "config", "config.yaml")
if os.path.exists(_config_local):
    config_path = _config_local
else:
    config_path = _config_shared
print(f"加载配置文件: {config_path}")

with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

paths = cfg["paths"]
# 将配置中的路径统一转换为绝对路径:
#   - 已经是绝对路径的保持不变
#   - 相对路径基于 PROJECT_ROOT 解析
for key in paths:
    if not os.path.isabs(paths[key]):
        paths[key] = os.path.join(PROJECT_ROOT, paths[key])

joint_names = cfg["joint_names"]
wheel_ids = cfg["wheel_ids"]

# 将配置列表 *4 是因为 BigDog 有四条腿，每组参数对应一条腿的 (hip, thigh, calf, wheel) 关节
default_dof_pos = torch.tensor(cfg["default_dof_pos"] * 4, dtype=torch.float32, device=device)
stand_dof_pos   = torch.tensor(cfg["stand_dof_pos"] * 4, dtype=torch.float32, device=device)

p_gains = torch.tensor(cfg["p_gains"] * 4, dtype=torch.float32, device=device)
d_gains = torch.tensor(cfg["d_gains"] * 4, dtype=torch.float32, device=device)
actions_scale = cfg["actions_scale"]
vel_scale = cfg["vel_scale"]
yaw_kp = cfg["yaw_kp"]
scale_factors = cfg["scale_factors"]
base_height_target = cfg["base_height_target"]
height_range = cfg["height_range"]

# ==================== 加载场景 ====================
m = mujoco.MjModel.from_xml_path(paths["scene_xml"])
d = mujoco.MjData(m)


def get_sensor_data(name):
    """从 MuJoCo 传感器数组中按名称读取数据，返回 torch 张量"""
    id_ = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if id_ == -1:
        raise ValueError(f"Sensor {name} not found")
    adr, dim = m.sensor_adr[id_], m.sensor_dim[id_]
    return torch.tensor(d.sensordata[adr:adr + dim], device=device, dtype=torch.float32)


def world2self(quat, v):
    """
    将世界坐标系下的向量 v 通过四元数 quat 旋转到机器人自身坐标系。

    数学原理:
      使用旋转公式 v' = q^{-1} ⊗ v ⊗ q 的展开形式（v 视为纯四元数）:
        v' = (2*q_w² - 1)*v  -  2*q_w*(q_vec × v)  +  2*(q_vec·v)*q_vec

    参数:
      quat: 机器人姿态四元数 [w, x, y, z] (world → body)
      v:    世界坐标系下的向量
    返回:
      v 在机器人自身坐标系下的表示
    """
    q_w, q_vec = quat[0], quat[1:]
    v_vec = v.clone().detach().to(dtype=torch.float32) if isinstance(v, torch.Tensor) else torch.tensor(v, device=device, dtype=torch.float32)
    a = v_vec * (2.0 * q_w ** 2 - 1.0)
    b = torch.linalg.cross(q_vec, v_vec) * q_w * 2.0
    c = q_vec * torch.dot(q_vec, v_vec) * 2.0
    return a - b + c


def get_obs(actions, default_dof_pos, commands):
    """
    构建 RL 策略网络的观测向量 (Observation Vector)。

    ==================== 观测向量结构 (共 58 维) ====================

     索引范围    维度    内容                        数据来源
    ─────────────────────────────────────────────────────────────────
     0  ~  2      3     imu_gyro * scale_ang_vel     IMU 陀螺仪 (角速度)
     3  ~  5      3     projected_gravity            重力在机体坐标系的投影
     6  ~  9      4     commands * commands_scale    控制指令 [vx, vy, yaw_rate, h]
    10  ~ 25     16     (dof_pos - default) * scale  关节位置误差 (16个关节)
    26  ~ 41     16     dof_vel * scale_dof_vel      关节速度 (16个关节)
    42  ~ 57     16     actions                      上一帧输出的动作
    ─────────────────────────────────────────────────────────────────
    总计: 3 + 3 + 4 + 16 + 16 + 16 = 58 维

    ==================== 各分量详细说明 ====================

    ① imu_gyro (3维) — 机体角速度
       来自 IMU 陀螺仪，表示机器人基座绕 x/y/z 轴的旋转速率。
       乘以 scale_ang_vel (0.25) 将 rad/s 量级缩放到适合网络的范围。

    ② projected_gravity (3维) — 重力投影
       世界坐标系的重力方向 [0, 0, -1] 通过姿态四元数旋转到机体坐标系。
       这个分量隐式编码了机器人的倾斜角度（roll/pitch），
       是行走控制器感知姿态的关键信息。

    ③ commands (4维) — 控制指令
       [vx, vy, yaw_rate, height_target] 分别表示:
         vx:       前进方向线速度目标
         vy:       横向线速度目标
         yaw_rate: 偏航角速度目标 (= yaw_kp * yaw_err)
         height:   目标身高偏移量
       各分量乘以 commands_scale 中对应的缩放因子进行归一化。

    ④ dof_pos - default_dof_pos (16维) — 关节位置误差
       16 个关节的当前位置与默认站立位置之间的偏差。
       轮子关节 (wheel_ids) 的位置被置零（轮子可无限旋转，绝对位置无意义）。
       乘以 scale_dof_pos (1.0) 进行缩放。

    ⑤ dof_vel (16维) — 关节速度
       16 个关节的当前角速度。
       乘以 scale_dof_vel (0.05) 将 rad/s 量级缩放到适合网络的范围。

    ⑥ actions (16维) — 上一帧动作
       策略网络上一时间步输出的 16 维动作向量。
       提供时间连续性信息，帮助网络产生平滑的动作序列。

    ==================== 缩放因子说明 ====================
    各分量通过 scale_factors 中定义的缩放因子进行归一化，
    将不同量纲的物理量（弧度、弧度/秒、米/秒等）映射到相近的数值范围，
    使神经网络各输入维度具有可比性，有利于训练收敛。
    """
    sf = scale_factors
    commands_scale = torch.tensor(
        [sf["scale_lin_vel"], sf["scale_lin_vel"], sf["scale_ang_vel"], sf["scale_height"]],
        device=device,
    )
    base_quat = get_sensor_data("imu_quat")
    # 将世界坐标系下的重力方向 [0, 0, -1] 转换到机器人自身坐标系
    projected_gravity = world2self(base_quat, torch.tensor([0.0, 0.0, -1.0], device=device))
    imu_gyro = get_sensor_data("imu_gyro")

    dof_pos = torch.zeros(16, device=device)
    for i, n in enumerate(joint_names):
        dof_pos[i] = get_sensor_data(n + "_pos")[0]
    dof_pos[wheel_ids] = 0.0  # 轮子关节位置置零（轮子无限旋转，绝对位置无意义）

    dof_vel = torch.zeros(16, device=device)
    for i, n in enumerate(joint_names):
        dof_vel[i] = get_sensor_data(n + "_vel")[0]

    cmds = torch.tensor(commands, device=device)
    return torch.cat(
        [
            imu_gyro * sf["scale_ang_vel"],
            projected_gravity,
            cmds * commands_scale,
            (dof_pos - default_dof_pos) * sf["scale_dof_pos"],
            dof_vel * sf["scale_dof_vel"],
            actions,
        ],
        dim=-1,
    )


def main():
    # ================== 初始化手柄 ==================
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("⚠ 未检测到手柄! 将以零指令运行（机器人站立不动）。")
        js = None
    else:
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"✓ 手柄已连接: {js.get_name()}")

    # ================== 加载策略网络 ==================
    #
    # 使用 TorchScript (torch.jit.load) 加载训练好的 RL 策略模型。
    # TorchScript 是 PyTorch 的序列化中间表示，可以在无 Python 环境的
    # C++ 运行时中执行，适合模型部署。
    #
    # ---- 网络输入: 348 维 (6 帧 × 58 维观测) ----
    #
    #   观测缓冲区 (obs_buffer) 存储最近 6 帧的观测向量，每帧 58 维。
    #   在每一步，新观测被插入缓冲区首部，最旧的一帧被丢弃，然后将
    #   6 帧展平为 (6 × 58 =) 348 维向量送入策略网络。
    #
    #   为什么需要 6 帧历史?
    #   - 单帧观测只包含瞬时位置、速度等信息，缺少动态上下文。
    #   - 堆叠多帧历史让网络能够隐式推断加速度、趋势等时序特征，
    #     类似于从位置序列推断速度——网络卷积层/全连接层可以学习
    #     帧间差分所蕴含的运动信息。
    #   - 这是模仿学习中常用的"观测历史"技巧，避免显式设计状态估计器。
    #
    #   观测缓冲区更新示意:
    #     obs_buffer[t] = [obs(t), obs(t-1), obs(t-2), obs(t-3), obs(t-4), obs(t-5)]
    #     网络输入 = flatten(obs_buffer[t])  →  348 维向量
    #
    # ---- 网络输出: 16 维 (16 个关节的目标位置偏移) ----
    #
    #   输出为各关节相对于默认站立位置 (default_dof_pos) 的目标偏移量。
    #   偏移量经过 actions_scale (0.25) 缩放后与 default_dof_pos 相加，
    #   得到各关节的目标角度，再由 PD 控制器执行跟踪。
    #
    #   数据流:
    #     policy(obs_seq) → actions[16]
    #     → actions_scaled = actions * actions_scale
    #     → target_pos = actions_scaled + default_dof_pos
    #     → PD 控制器计算力矩 → 关节执行
    #
    # ---- 模型设备 ----
    #   自动检测 CUDA GPU，若可用则在 GPU 上推理以加速；
    #   否则回退到 CPU。
    #
    try:
        policy = torch.jit.load(paths["policy_path"])
        policy.eval().to(device)
        print("✓ 策略网络加载成功")
        print(f"  运行设备: {device}")
    except Exception as e:
        print(f"✗ 策略网络加载失败: {e}")
        print("  将使用 PD 控制器维持站立姿态")
        policy = None

    # ================== PD 初始化稳定姿态 ==================
    # 在启动 RL 控制之前，先用 PD 控制器将机器人驱动到站立姿态并保持 200 步，
    # 为策略网络提供一个稳定的初始状态，避免从随机/跌落姿态起步导致失控。
    d.qpos[0:3] = [0.0, 0.0, base_height_target]
    for i, name in enumerate(joint_names):
        jnt_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        d.qpos[m.jnt_qposadr[jnt_id]] = stand_dof_pos[i].item()

    stand_np = stand_dof_pos.cpu().numpy()
    p_np = p_gains.cpu().numpy()
    d_np = d_gains.cpu().numpy()
    for _ in range(200):
        qpos_arr = np.array([d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] for n in joint_names])
        qvel_arr = np.array([d.qvel[m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] for n in joint_names])
        act = np.zeros(16)
        for j in range(16):
            if j in wheel_ids:
                act[j] = -d_np[j] * qvel_arr[j]  # 轮子仅阻尼，不控制位置
            else:
                # 1.2 * 1.25 = 1.5 倍 P 增益，使站立更"硬"
                act[j] = 1.2 * 1.25 * p_np[j] * (stand_np[j] - qpos_arr[j]) - d_np[j] * qvel_arr[j]
        d.ctrl[:] = np.clip(act, -100, 100)
        mujoco.mj_step(m, d)

    print("=" * 50)
    print("  BigDog RL 仿真")
    if js is not None:
        print("  左摇杆: 前进/后退 + 左右平移")
        print("  右摇杆 X: 转向")
        print("  D-pad 上下: 调节身高")
        print("  A: 站定 (归零指令)   B: 阻尼模式")
    else:
        print("  无手柄 — 机器人原地站立 (零指令)")
    print("=" * 50)

    # ================== 主循环 ==================
    #
    # actions:     上一帧输出的 16 维动作向量 (作为下一帧观测的一部分)
    # obs_buffer:  观测历史缓冲区，形状 (6, 58)，存储最近 6 帧观测
    #              索引 0 为最新帧，索引 5 为最旧帧
    # heading_target: 累积的目标偏航角 (rad)，由右摇杆控制
    # height_target:  目标身高偏移量 (m)，由 D-pad 上下控制
    #
    actions = torch.zeros(16, device=device)
    obs_buffer = torch.zeros((6, 58), device=device)
    heading_target = 0.0
    height_target = 0.0

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            # --- 处理手柄事件 ---
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:  # A 键: 站定
                        heading_target = 0.0
                        height_target = 0.0
                    elif event.button == 1:  # B 键: 阻尼模式
                        d.ctrl[:] = 0.0
                elif event.type == pygame.QUIT:
                    break

            # --- 读取摇杆指令 (无手柄时为零指令，机器人原地站立) ---
            if js is not None:
                # 左摇杆 Y (轴1): 前进/后退速度，前推为负值故取反
                raw_vx = -js.get_axis(1)
                vx = scale_axis(raw_vx) * 1.0
                # 左摇杆 X (轴0): 横向平移速度
                raw_vy = -js.get_axis(0)
                vy = scale_axis(raw_vy) * 0.6
                # 右摇杆 X (轴2): 偏航角速率，累积得到目标航向
                raw_wz = js.get_axis(2)
                wz_input = scale_axis(raw_wz)
                heading_target -= wz_input * 0.03
                heading_target = np.clip(heading_target, -3.14, 3.14)

                # D-pad 上下控制身高偏移
                hat_y = js.get_hat(0)[1]
                if hat_y != 0:
                    height_target += hat_y * 0.01
                    height_target = np.clip(height_target, height_range[0], height_range[1])
            else:
                # 无手柄: 零指令，机器人保持原地站立
                vx, vy = 0.0, 0.0

            commands = [vx, vy, heading_target, height_target]

            # --- 读取传感器 ---
            dof_pos = torch.cat([get_sensor_data(n + "_pos") for n in joint_names]).to(device)
            dof_vel = torch.cat([get_sensor_data(n + "_vel") for n in joint_names]).to(device)
            dof_err = default_dof_pos - dof_pos  # 位置误差 = 目标 - 当前

            # --- RL 控制 ---
            if policy is not None:
                # ---- 步骤1: 从 IMU 四元数提取当前偏航角 ----
                # 四元数转偏航角的公式 (ZYX 欧拉角):
                #   yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
                base_quat = get_sensor_data("imu_quat")
                q_w, q_x, q_y, q_z = base_quat
                yaw_now = torch.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z))

                # ---- 步骤2: 计算偏航角误差 (处理 ±π 环绕) ----
                yaw_err = torch.atan2(
                    torch.sin(torch.tensor(commands[2]) - yaw_now),
                    torch.cos(torch.tensor(commands[2]) - yaw_now),
                )
                # yaw_kp * yaw_err → 比例控制器将角度误差转换为角速度指令
                commands_rl = [commands[0], commands[1], yaw_kp * yaw_err.item(), commands[3]]

                print(f"\r vx={commands[0]:+5.2f} vy={commands[1]:+5.2f} "
                      f"heading={commands[2]:+5.2f} h={commands[3]:+5.2f} yaw={yaw_now:+5.2f}", end="")

                # ---- 步骤3: 构建观测 → 更新历史缓冲区 ----
                obs_now = get_obs(actions, default_dof_pos, commands_rl)
                obs_now = torch.clip(obs_now, -100, 100)
                # 新观测插入首部 (索引0)，最旧帧 (索引5) 被丢弃
                obs_buffer = torch.cat([obs_now.unsqueeze(0), obs_buffer[:-1]], dim=0)
                # 展平 6×58=348 维作为策略网络输入
                obs_seq = obs_buffer.flatten().float()

                # ---- 步骤4: 策略推理 → PD 跟踪 ----
                actions = policy(obs_seq)                    # 16维动作偏移
                actions_scaled = actions * actions_scale      # 缩放动作
                vel_ref = torch.zeros_like(actions_scaled)
                vel_ref[wheel_ids] = actions[wheel_ids] * vel_scale  # 轮子使用速度控制
                # PD 控制: 力矩 = Kp*(目标位置-当前位置) + Kd*(目标速度-当前速度)
                act = p_gains * (actions_scaled + dof_err) + d_gains * (vel_ref - dof_vel)
                d.ctrl[:] = torch.clip(act, -100, 100).detach().cpu().numpy()
            else:
                # 无策略时: PD 控制器维持站立姿态
                act = torch.zeros(16, device=device)
                for i in range(16):
                    if i in wheel_ids:
                        act[i] = -d_gains[i] * dof_vel[i]
                    else:
                        act[i] = 1.2 * 1.25 * p_gains[i] * dof_err[i] - d_gains[i] * dof_vel[i]
                d.ctrl[:] = torch.clip(act, -100, 100).cpu().numpy()

            # --- 物理步进 (每帧执行 sim_steps_per_loop 次物理步) ---
            step_start = time.time()
            for _ in range(cfg["sim_steps_per_loop"]):
                mujoco.mj_step(m, d)

            # 摄像机跟随机器人 base 连杆
            viewer.cam.lookat[:] = d.xpos[
                mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
            ]
            viewer.sync()
            time_until_next_step = m.opt.timestep * 4 - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
