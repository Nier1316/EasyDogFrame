%% 四足机器狗 三关节腿 正逆运动学可视化仿真
%
% ██████  物理坐标系 (符合实际工况) ██████
%
% 关节配置 (从狗体到足端):
%   θ1  - 髋外摆关节 (绕 X 轴旋转)
%         正方向: 腿向外翻 (与 Y 正方向同向)
%   θ2  - 髋屈曲/大腿关节
%         正方向: 大腿向后摆
%   θ3  - 膝关节/小腿关节
%         正方向: 小腿向后弯
%
% 基坐标系 (固定在髋关节):
%   X 轴: 正方向为向后 (狗体后方)
%         大腿后摆 → X 正方向; 小腿后弯 → X 正方向
%   Y 轴: 正方向为向外翻 (髋外翻同向)
%   Z 轴: 正方向竖直向上
%
% 连杆参数:
%   L1: 髋侧向偏移量 (髋外摆关节 → 髋屈曲关节的 Y 向距离)
%   L2: 大腿长度 (髋 → 膝)
%   L3: 小腿长度 (膝 → 足)
%
% 零位姿态 (θ1=0, θ2=0, θ3=0):
%   大腿竖直向下, 小腿竖直向下
%   足端: [0, L1, -(L2+L3)] — 位于身体正下方偏外侧
%
% 使用:
%   直接运行脚本 → 显示正逆运动学验证 + 工作空间 + 交互式 GUI

clear; clc; close all;

%% ========== 连杆参数 (单位: 米) — 根据实际硬件修改 ==========
L1 = 0.05;      % 髋侧向偏移量
L2 = 0.20;      % 大腿长度
L3 = 0.20;      % 小腿长度

% ====================================================================
%  关节零位偏移 & 限位宏定义
%  可直接复制到 C/C++ 代码中使用（角度单位为度，内部转为弧度）
%
%  物理关节角 = ZERO_OFFSET + 指令角
%  指令角 ∈ [LOWER_LIMIT, UPPER_LIMIT]
% ====================================================================

% --- θ1 髋外摆 ---
% 零位: 腿在 YZ 平面内竖直向下 (绕 X 轴)
ZERO_OFFSET_THETA1_DEG = 30.0;   % deg
LOWER_LIMIT_THETA1_DEG = -60.0; % deg
UPPER_LIMIT_THETA1_DEG =  0.0; % deg

% --- θ2 大腿 ---
% 零位: 大腿在 XZ 平面内竖直向下
ZERO_OFFSET_THETA2_DEG = 0.0;   % deg
LOWER_LIMIT_THETA2_DEG = -45.0; % deg
UPPER_LIMIT_THETA2_DEG =  90.0; % deg

% --- θ3 小腿 ---
% 零位: 小腿在 XZ 平面内竖直向下
ZERO_OFFSET_THETA3_DEG = 0.0;   % deg
LOWER_LIMIT_THETA3_DEG = 60.0;  % deg
UPPER_LIMIT_THETA3_DEG = 180.0; % deg

% 转为弧度（仿真内部使用）
theta1_offset = deg2rad(ZERO_OFFSET_THETA1_DEG);
theta2_offset = deg2rad(ZERO_OFFSET_THETA2_DEG);
theta3_offset = deg2rad(ZERO_OFFSET_THETA3_DEG);

theta1_min = deg2rad(ZERO_OFFSET_THETA1_DEG + LOWER_LIMIT_THETA1_DEG);
theta1_max = deg2rad(ZERO_OFFSET_THETA1_DEG + UPPER_LIMIT_THETA1_DEG);
theta2_min = deg2rad(ZERO_OFFSET_THETA2_DEG + LOWER_LIMIT_THETA2_DEG);
theta2_max = deg2rad(ZERO_OFFSET_THETA2_DEG + UPPER_LIMIT_THETA2_DEG);
theta3_min = deg2rad(ZERO_OFFSET_THETA3_DEG + LOWER_LIMIT_THETA3_DEG);
theta3_max = deg2rad(ZERO_OFFSET_THETA3_DEG + UPPER_LIMIT_THETA3_DEG);

%% ========== 正运动学验证 ==========
fprintf('========== 正运动学验证 ==========\n');
fprintf('连杆: L1=%.3f (髋偏), L2=%.3f (大腿), L3=%.3f (小腿)\n\n', L1, L2, L3);

% 零位: 腿竖直向下
q0 = [0, 0, 0];
p0 = fk(q0, L1, L2, L3, theta1_offset, theta2_offset, theta3_offset);
fprintf('零位 [0, 0, 0]:             足端 [%+.3f, %+.3f, %+.3f]  ← 期望[0, +L1, -(L2+L3)]\n', p0);

% 大腿后摆 +45° (θ₂ 正=向后)
q1 = [0, deg2rad(45), 0];
p1 = fk(q1, L1, L2, L3, theta1_offset, theta2_offset, theta3_offset);
fprintf('大腿后摆 [0, +45°, 0]:      足端 [%+.3f, %+.3f, %+.3f]  ← X>0 (X+向后)\n', p1);

% 站立姿态: 大腿略前摆(θ₂负), 小腿后弯(θ₃正)
q2 = [0, deg2rad(-30), deg2rad(60)];
p2 = fk(q2, L1, L2, L3, theta1_offset, theta2_offset, theta3_offset);
fprintf('站立姿态 [0, -30°, +60°]:   足端 [%+.3f, %+.3f, %+.3f]  ← X≈0 足在髋下\n', p2);

% 髋外翻 +30°
q3 = [deg2rad(30), 0, 0];
p3 = fk(q3, L1, L2, L3, theta1_offset, theta2_offset, theta3_offset);
fprintf('髋外翻 [+30°, 0, 0]:       足端 [%+.3f, %+.3f, %+.3f]  ← Y>0 外翻, Z↑抬升\n', p3);

%% ========== 逆运动学验证 ==========
fprintf('\n========== 逆运动学验证 ==========\n');

% 对几个典型位置进行 IK → FK 验算
% 每行: [px, py, pz]
test_positions = [
    0.00, 0.05, -0.346;    % 近零位 (θ₂=-30°, θ₃=60°)
    0.20, 0.05, -0.235;    % 后伸 (θ₂=0°, θ₃=80°)
   -0.12, 0.05, -0.326;    % 前伸 (θ₂=-50°, θ₃=60°)
    0.00, 0.14, -0.321;    % 外展 (θ₁=15°, θ₂=-30°, θ₃=60°)
];

for i = 1:size(test_positions, 1)
    p_target = test_positions(i, :)';
    q_ik = ik(p_target, L1, L2, L3, ...
        theta1_offset, theta2_offset, theta3_offset);
    p_rebuild = fk(q_ik, L1, L2, L3, ...
        theta1_offset, theta2_offset, theta3_offset);
    err = norm(p_rebuild - p_target);
    fprintf(' 目标 [%.2f, %.2f, %.2f]  →  IK [θ₁=%+5.1f° θ₂=%+5.1f° θ₃=%+5.1f°]  →  误差 %.2e m\n', ...
        p_target(1), p_target(2), p_target(3), ...
        rad2deg(q_ik(1)), rad2deg(q_ik(2)), rad2deg(q_ik(3)), err);
end

%% ========== 工作空间蒙特卡洛分析 ==========
fprintf('\n========== 足端工作空间分析 ==========\n');

N = 8000;
rng(42);
% 关节限幅 (使用上面定义的限位宏)
th1 = theta1_min + (theta1_max - theta1_min) * rand(N, 1);
th2 = theta2_min + (theta2_max - theta2_min) * rand(N, 1);
th3 = theta3_min + (theta3_max - theta3_min) * rand(N, 1);

ws = zeros(N, 3);
for i = 1:N
    % th1/2/3 已是物理角 (ZERO_OFFSET + LIMIT), 故 offset 传 0
    ws(i,:) = fk([th1(i), th2(i), th3(i)], L1, L2, L3, 0, 0, 0);
end

fprintf('  X 范围: [%+.3f, %+.3f] m\n', min(ws(:,1)), max(ws(:,1)));
fprintf('  Y 范围: [%+.3f, %+.3f] m\n', min(ws(:,2)), max(ws(:,2)));
fprintf('  Z 范围: [%+.3f, %+.3f] m\n', min(ws(:,3)), max(ws(:,3)));

%% ========== 绘制工作空间 ==========
figure('Name', '足端工作空间', 'Position', [50, 100, 1200, 500]);

subplot(1,2,1);
plot3(ws(:,1), ws(:,2), ws(:,3), '.', 'MarkerSize', 2, 'Color', [0.3 0.5 0.8]);
hold on; grid on; axis equal;
xlabel('X (向后)'); ylabel('Y (外翻)'); zlabel('Z (向上)');
title('足端工作空间 (XYZ)');
plot3(0, 0, 0, 'r^', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
plot3(0, L1, 0, 'g^', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
legend({'足端可达位置', '髋关节 (原点)', '髋屈曲关节'}, 'Location', 'best');
view(135, 25);

subplot(1,2,2);
plot(ws(:,1), ws(:,3), '.', 'MarkerSize', 2, 'Color', [0.3 0.5 0.8]);
hold on; grid on; axis equal;
xlabel('X (向后)'); ylabel('Z (向上)');
title('工作空间 XZ 投影 (矢状面)');
plot(0, 0, 'r^', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
line([-0.4, 0.4], [0, 0], 'Color', [0.5 0.5 0.5], 'LineStyle', '--');
text(0.05, -0.02, '地面', 'Color', [0.5 0.5 0.5]);

%% ========== 交互式 GUI 仿真 ==========
build_gui(L1, L2, L3, ...
    LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG, ...
    LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG, ...
    LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG, ...
    theta1_offset, theta2_offset, theta3_offset);

%% ============================================================
%                     运动学函数
% ============================================================

function p = fk(q_cmd, L1, L2, L3, off1, off2, off3)
    % 正运动学: 指令角 → 足端位置
    % 先加零位偏移得物理角, 再计算运动学
    %
    % 用户约定: θ2,θ3 正值=向后
    % 内部公式约定: θ2,θ3 正值=向前, 入口取反转换

    % 指令角 → 物理角 (用户约定)
    t1_phys = q_cmd(1) + off1;
    t2_phys = q_cmd(2) + off2;
    t3_phys = q_cmd(3) + off3;

    % 用户约定 → 内部公式约定 (取反)
    t2_int = -t2_phys;
    t3_int = -t3_phys;

    % 腿平面内的位置分量
    A = sin(t2_int)*L2 + sin(t2_int+t3_int)*L3;   % X₁ (前向分量)
    B = L1;                                        % Y₁ (外翻偏移)
    C = -cos(t2_int)*L2 - cos(t2_int+t3_int)*L3;   % Z₁ (竖直, 向下负)

    % 基坐标系 X+ = 向后, 故取负号
    px = -A;
    py = B*cos(t1_phys) - C*sin(t1_phys);
    pz = B*sin(t1_phys) + C*cos(t1_phys);

    p = [px; py; pz];
end

function q_cmd = ik(p_target, L1, L2, L3, off1, off2, off3)
    % 逆运动学: 足端位置 → 指令角
    % 先解算物理角, 再减零位偏移得指令角
    %
    % 解析法求解步骤 (内部使用"正值=向前"的约定):
    %   Step 1: 由 (py, pz) 和 L1 求 θ1 (髋外摆)
    %   Step 2: 还原腿平面内分量, 转化为 2R 平面臂问题
    %   Step 3: 余弦定理求 θ3 (取负解 = 向后弯)
    %   Step 4: 几何法求 θ2
    %   最后 θ2, θ3 取反, 转换为"正值=向后"的用户约定得到物理角,
    %   再减去零位偏移得到指令角

    px = p_target(1); py = p_target(2); pz = p_target(3);

    % ---- Step 1: 求 θ1_phys ----
    r_yz = sqrt(py^2 + pz^2);
    if r_yz < abs(L1) + 1e-9
        r_yz = abs(L1) + 1e-6;
    end
    t1_phys = real(pi - asin(L1 / r_yz) - atan2(py, pz));

    % ---- Step 2: 还原腿平面内分量 ----
    D = real(sqrt(max(r_yz^2 - L1^2, 0)));
    A = -px;

    % ---- Step 3: 求 θ3_internal ----
    numerator = A^2 + D^2 - L2^2 - L3^2;
    denominator = 2 * L2 * L3;

    if abs(denominator) < 1e-12
        t3_int = 0;
    else
        cos_t3 = numerator / denominator;
        cos_t3 = max(-1, min(1, cos_t3));
        t3_int = -acos(cos_t3);
    end

    % ---- Step 4: 求 θ2_internal ----
    k1 = L2 + L3 * cos(t3_int);
    k2 = L3 * sin(t3_int);
    denom = k1^2 + k2^2;

    if denom < 1e-12
        t2_int = 0;
    else
        sin_t2 = (A * k1 - D * k2) / denom;
        cos_t2 = (A * k2 + D * k1) / denom;
        t2_int = atan2(sin_t2, cos_t2);
    end

    % 内部约定 → 用户约定 → 物理角 → 指令角
    t2_phys = -t2_int;
    t3_phys = -t3_int;
    q_cmd = [t1_phys - off1; t2_phys - off2; t3_phys - off3];
end

%% ============================================================
%                     交互式 GUI
% ============================================================

function build_gui(L1, L2, L3, ...
        LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG, ...
        LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG, ...
        LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG, ...
        off1, off2, off3)
    % 构造带滑块和 IK 求解的交互界面

    fig = figure('Name', '三关节腿运动学 — 交互仿真', ...
        'Position', [100, 80, 1260, 650], ...
        'NumberTitle', 'off');

    % 初始关节角 (自然站立: 大腿略前摆θ₂负, 小腿后弯θ₃正)
    theta0 = [deg2rad(5), deg2rad(-30), deg2rad(60)];

    % ===== 主绘图区 =====
    ax = axes('Parent', fig, 'Units', 'pixels', ...
        'Position', [80, 120, 780, 500]);
    draw_leg(ax, theta0, L1, L2, L3, off1, off2, off3);

    % ===== 底部信息栏 =====
    info_h = uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [80, 15, 800, 30], ...
        'FontSize', 12, 'HorizontalAlignment', 'left', ...
        'FontWeight', 'bold');

    % ===== 右侧控制面板 =====
    px0 = 920;
    sw  = 240;  % 滑块宽度

    % θ1 滑块
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0, 565, 80, 22], ...
        'String', 'θ₁ 髋外摆', 'FontSize', 11, 'FontWeight', 'bold');
    s1_val = uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0+sw+10, 565, 70, 22], ...
        'FontSize', 11, 'FontWeight', 'bold');
    s1 = uicontrol('Parent', fig, 'Style', 'slider', ...
        'Position', [px0, 545, sw, 22], ...
        'Min', LOWER_LIMIT_THETA1_DEG, 'Max', UPPER_LIMIT_THETA1_DEG, ...
        'Value', rad2deg(theta0(1)));

    % θ2 滑块
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0, 505, 80, 22], ...
        'String', 'θ₂ 大腿', 'FontSize', 11, 'FontWeight', 'bold');
    s2_val = uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0+sw+10, 505, 70, 22], ...
        'FontSize', 11, 'FontWeight', 'bold');
    s2 = uicontrol('Parent', fig, 'Style', 'slider', ...
        'Position', [px0, 485, sw, 22], ...
        'Min', LOWER_LIMIT_THETA2_DEG, 'Max', UPPER_LIMIT_THETA2_DEG, ...
        'Value', rad2deg(theta0(2)));

    % θ3 滑块
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0, 445, 80, 22], ...
        'String', 'θ₃ 小腿', 'FontSize', 11, 'FontWeight', 'bold');
    s3_val = uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0+sw+10, 445, 70, 22], ...
        'FontSize', 11, 'FontWeight', 'bold');
    s3 = uicontrol('Parent', fig, 'Style', 'slider', ...
        'Position', [px0, 425, sw, 22], ...
        'Min', LOWER_LIMIT_THETA3_DEG, 'Max', UPPER_LIMIT_THETA3_DEG, ...
        'Value', rad2deg(theta0(3)));

    % 角度标记更新
    set(s1_val, 'String', sprintf('%.1f°', rad2deg(theta0(1))));
    set(s2_val, 'String', sprintf('%.1f°', rad2deg(theta0(2))));
    set(s3_val, 'String', sprintf('%.1f°', rad2deg(theta0(3))));

    % ===== IK 输入区 =====
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0, 375, 290, 22], ...
        'String', '━━━ IK 目标足端位置 ━━━', ...
        'FontSize', 11, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');

    % 输入框
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0, 350, 25, 22], 'String', 'X:', 'FontSize', 11, 'FontWeight', 'bold');
    h_px = uicontrol('Parent', fig, 'Style', 'edit', ...
        'Position', [px0+28, 348, 66, 26], 'String', '0.10', ...
        'FontSize', 11, 'BackgroundColor', 'w');

    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0+108, 350, 25, 22], 'String', 'Y:', 'FontSize', 11, 'FontWeight', 'bold');
    h_py = uicontrol('Parent', fig, 'Style', 'edit', ...
        'Position', [px0+136, 348, 66, 26], 'String', '0.10', ...
        'FontSize', 11, 'BackgroundColor', 'w');

    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0+218, 350, 25, 22], 'String', 'Z:', 'FontSize', 11, 'FontWeight', 'bold');
    h_pz = uicontrol('Parent', fig, 'Style', 'edit', ...
        'Position', [px0+246, 348, 66, 26], 'String', '-0.35', ...
        'FontSize', 11, 'BackgroundColor', 'w');

    % IK 按钮
    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [px0, 305, 300, 35], ...
        'String', 'IK 求解 → 更新关节角', ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.7 0.85 1.0], ...
        'ForegroundColor', [0 0 0.4]);

    % ===== 预设姿态按钮 =====
    uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [px0, 270, 290, 20], ...
        'String', '预设姿态:', ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left');

    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [px0, 240, 90, 28], ...
        'String', '站立', 'FontSize', 10, ...
        'Callback', {@preset_cb, s1, s2, s3, [5, -30, 60]});

    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [px0+100, 240, 90, 28], ...
        'String', '前伸', 'FontSize', 10, ...
        'Callback', {@preset_cb, s1, s2, s3, [10, -50, 80]});

    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [px0+200, 240, 90, 28], ...
        'String', '回收', 'FontSize', 10, ...
        'Callback', {@preset_cb, s1, s2, s3, [0, -40, 60]});

    % ===== 滑块回调 =====
    slider_data = struct('ax', ax, 'L', [L1, L2, L3], ...
        'info', info_h, 's1v', s1_val, 's2v', s2_val, 's3v', s3_val, ...
        'off', [off1, off2, off3], ...
        'lim1', [LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG], ...
        'lim2', [LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG], ...
        'lim3', [LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG]);
    set(s1, 'UserData', slider_data);
    set(s2, 'UserData', slider_data);
    set(s3, 'UserData', slider_data);

    set(s1, 'Callback', {@slider_cb, s1, s2, s3});
    set(s2, 'Callback', {@slider_cb, s1, s2, s3});
    set(s3, 'Callback', {@slider_cb, s1, s2, s3});

    % IK 按钮回调
    ik_data = struct('ax', ax, 'L', [L1, L2, L3], ...
        's1', s1, 's2', s2, 's3', s3, ...
        's1v', s1_val, 's2v', s2_val, 's3v', s3_val, ...
        'info', info_h, ...
        'off', [off1, off2, off3], ...
        'lim1', [LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG], ...
        'lim2', [LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG], ...
        'lim3', [LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG]);
    set(h_px, 'UserData', ik_data);
    set(h_py, 'UserData', ik_data);
    set(h_pz, 'UserData', ik_data);

    set(findobj(fig, 'String', 'IK 求解 → 更新关节角'), ...
        'Callback', {@ik_cb, h_px, h_py, h_pz, s1, s2, s3});

    % 初始更新信息
    update_info(info_h, theta0, L1, L2, L3, off1, off2, off3);
end

%% ---------- 滑块回调 ----------
function slider_cb(~, ~, s1, s2, s3)
    t_deg = [get(s1, 'Value'), get(s2, 'Value'), get(s3, 'Value')];
    t_rad = deg2rad(t_deg);

    ud = get(s1, 'UserData');
    ax = ud.ax; L = ud.L; off = ud.off;

    draw_leg(ax, t_rad, L(1), L(2), L(3), off(1), off(2), off(3));
    update_info(ud.info, t_rad, L(1), L(2), L(3), off(1), off(2), off(3));

    set(ud.s1v, 'String', sprintf('%.1f°', t_deg(1)));
    set(ud.s2v, 'String', sprintf('%.1f°', t_deg(2)));
    set(ud.s3v, 'String', sprintf('%.1f°', t_deg(3)));
end

%% ---------- IK 按钮回调 ----------
function ik_cb(~, ~, h_px, h_py, h_pz, s1, s2, s3)
    try
        px = str2double(get(h_px, 'String'));
        py = str2double(get(h_py, 'String'));
        pz = str2double(get(h_pz, 'String'));
    catch
        return;
    end

    ud = get(h_px, 'UserData');
    ax = ud.ax; L = ud.L; off = ud.off;

    q_rad = ik([px; py; pz], L(1), L(2), L(3), off(1), off(2), off(3));
    q_deg = rad2deg(q_rad)';

    % 钳位到限位宏定义范围 (从 UserData 读取)
    ud = get(h_px, 'UserData');
    q_deg(1) = max(ud.lim1(1), min(ud.lim1(2), q_deg(1)));
    q_deg(2) = max(ud.lim2(1), min(ud.lim2(2), q_deg(2)));
    q_deg(3) = max(ud.lim3(1), min(ud.lim3(2), q_deg(3)));

    set(s1, 'Value', q_deg(1));
    set(s2, 'Value', q_deg(2));
    set(s3, 'Value', q_deg(3));
    q_deg_clamped = q_deg;

    q_rad = deg2rad(q_deg);
    draw_leg(ax, q_rad, L(1), L(2), L(3), off(1), off(2), off(3));

    set(ud.s1v, 'String', sprintf('%.1f°', q_deg(1)));
    set(ud.s2v, 'String', sprintf('%.1f°', q_deg(2)));
    set(ud.s3v, 'String', sprintf('%.1f°', q_deg(3)));

    % 显示 IK 误差
    p_actual = fk(q_rad, L(1), L(2), L(3), off(1), off(2), off(3));
    err = norm(p_actual - [px; py; pz]);
    clamped_str = '';
    if any(abs(q_deg_clamped - rad2deg(q_rad)') > 1)
        clamped_str = ' (角度限幅)';
    end
    set(ud.info, 'String', sprintf(...
        'IK求解: 目标 [%.3f, %.3f, %.3f]  →  实际 [%.3f, %.3f, %.3f]  →  误差 %.2e m%s', ...
        px, py, pz, p_actual(1), p_actual(2), p_actual(3), err, clamped_str));
end

%% ---------- 预设姿态回调 ----------
function preset_cb(~, ~, s1, s2, s3, angles_deg)
    set(s1, 'Value', angles_deg(1));
    set(s2, 'Value', angles_deg(2));
    set(s3, 'Value', angles_deg(3));
    slider_cb([], [], s1, s2, s3);
end

%% ---------- 信息更新 ----------
function update_info(h, theta_cmd, L1, L2, L3, off1, off2, off3)
    p = fk(theta_cmd, L1, L2, L3, off1, off2, off3);
    q_deg = rad2deg(theta_cmd);
    set(h, 'String', ...
        sprintf('足端 [X=%+.3f, Y=%+.3f, Z=%+.3f] m  |  关节 [θ₁=%+.1f°, θ₂=%+.1f°, θ₃=%+.1f°]  |  离地 %.0f mm', ...
        p(1), p(2), p(3), q_deg(1), q_deg(2), q_deg(3), -p(3)*1000));
end

%% ---------- 腿绘制 ----------
function draw_leg(ax, theta_cmd, L1, L2, L3, off1, off2, off3)
    t1 = theta_cmd(1); t2 = theta_cmd(2); t3 = theta_cmd(3);

    % 计算关节点 (传入命令角, FK 内部加 offset 得物理角)
    p_hip     = [0, 0, 0];
    p_abduct  = fk([t1, 0, 0], L1, 0, 0, off1, off2, off3);
    p_knee    = fk([t1, t2, 0], L1, L2, 0, off1, off2, off3);
    p_foot    = fk([t1, t2, t3], L1, L2, L3, off1, off2, off3);

    cla(ax); hold(ax, 'on');

    % ----- 连杆 (粗线) -----
    % 髋偏移 (黑色)
    plot3(ax, [p_hip(1), p_abduct(1)], ...
              [p_hip(2), p_abduct(2)], ...
              [p_hip(3), p_abduct(3)], ...
              'Color', [0.2 0.2 0.2], 'LineWidth', 5);
    % 大腿 (蓝色)
    plot3(ax, [p_abduct(1), p_knee(1)], ...
              [p_abduct(2), p_knee(2)], ...
              [p_abduct(3), p_knee(3)], ...
              'Color', [0, 0.45, 0.74], 'LineWidth', 6);
    % 小腿 (橙色)
    plot3(ax, [p_knee(1), p_foot(1)], ...
              [p_knee(2), p_foot(2)], ...
              [p_knee(3), p_foot(3)], ...
              'Color', [0.85, 0.33, 0.10], 'LineWidth', 6);

    % ----- 关节标记 -----
    plot3(ax, p_hip(1), p_hip(2), p_hip(3), 'ko', ...
        'MarkerSize', 14, 'LineWidth', 2, 'MarkerFaceColor', 'k');
    plot3(ax, p_abduct(1), p_abduct(2), p_abduct(3), 'ks', ...
        'MarkerSize', 11, 'LineWidth', 2, 'MarkerFaceColor', [0.6 0.6 0.6]);
    plot3(ax, p_knee(1), p_knee(2), p_knee(3), 'ko', ...
        'MarkerSize', 11, 'LineWidth', 2, 'MarkerFaceColor', 'w');
    plot3(ax, p_foot(1), p_foot(2), p_foot(3), 'r*', ...
        'MarkerSize', 16, 'LineWidth', 2);

    % ----- 标签 -----
    text(ax, p_hip(1)-0.03, p_hip(2), p_hip(3)+0.02, '髋', ...
        'FontSize', 12, 'FontWeight', 'bold');
    text(ax, p_abduct(1)+0.02, p_abduct(2), p_abduct(3)+0.02, '髋屈', ...
        'FontSize', 12, 'FontWeight', 'bold');
    text(ax, p_knee(1)+0.02, p_knee(2), p_knee(3)+0.02, '膝', ...
        'FontSize', 12, 'FontWeight', 'bold');
    text(ax, p_foot(1), p_foot(2), p_foot(3)+0.03, ...
        sprintf('足 [%.2f, %.2f, %.2f]', p_foot(1), p_foot(2), p_foot(3)), ...
        'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.85, 0.33, 0.10]);

    % ----- 坐标系指示 -----
    quiver3(0, 0, 0, 0.14, 0, 0, 'r-', 'LineWidth', 2, 'MaxHeadSize', 0.4);
    quiver3(0, 0, 0, 0, 0.14, 0, 'g-', 'LineWidth', 2, 'MaxHeadSize', 0.4);
    quiver3(0, 0, 0, 0, 0, 0.14, 'b-', 'LineWidth', 2, 'MaxHeadSize', 0.4);
    text(ax, 0.15, 0, 0, 'X (后)', 'Color', 'r', 'FontWeight', 'bold');
    text(ax, 0, 0.15, 0, 'Y (外)', 'Color', 'g', 'FontWeight', 'bold');
    text(ax, 0, 0, 0.15, 'Z (上)', 'Color', 'b', 'FontWeight', 'bold');

    % ----- 地面参考网格 -----
    [gx, gz] = meshgrid(-0.4:0.1:0.4, -0.5:0.1:0.5);
    gy = zeros(size(gx)) - 0.01;
    mesh(ax, gx, gy, gz, 'EdgeColor', [0.7 0.7 0.7], 'FaceAlpha', 0, 'LineStyle', ':');

    axis(ax, 'equal');
    xlim(ax, [-0.45, 0.45]); ylim(ax, [-0.05, 0.40]); zlim(ax, [-0.55, 0.15]);
    title(ax, sprintf('θ₁=%+.1f°  θ₂=%+.1f°  θ₃=%+.1f°', ...
        rad2deg(t1), rad2deg(t2), rad2deg(t3)));
    grid(ax, 'on');
    xlabel(ax, 'X (向后)'); ylabel(ax, 'Y (外翻)'); zlabel(ax, 'Z (向上)');
    view(ax, 135, 25);
end
