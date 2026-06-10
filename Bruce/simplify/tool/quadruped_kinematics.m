%% 四足机器狗 完整运动学仿真
%
% 基于单腿正逆运动学, 构建四足全身运动学
%
% ██████  坐标系 ██████
%
% 身体坐标系 (原点在身体几何中心):
%   X 轴: 正方向为前进方向 (狗体向前)
%   Y 轴: 正方向为向左 (狗体左侧)
%   Z 轴: 正方向竖直向上
%
% 各腿髋关节坐标系 (与身体坐标系的关系见函数体):
%   每条腿的 hip 坐标系 X+ = 向后, Y+ = 向外翻, Z+ = 向上
%   与单腿仿真 leg_kinematics.m 中定义的坐标系一致
%
% 使用:
%   直接运行 → 显示全身 3D 模型 + 交互控制

clear; clc; close all;

%% ========== 连杆参数 (复制自单腿仿真, 单位: 米) ==========
L1 = 0.05;      % 髋侧向偏移量
L2 = 0.20;      % 大腿长度
L3 = 0.20;      % 小腿长度

%% ========== 身体尺寸 (单位: 米) ==========
BODY_LENGTH = 0.30;   % 身体长度 (X方向)
BODY_WIDTH  = 0.12;   % 身体宽度 (Y方向)
BODY_HEIGHT = 0.06;   % 身体高度 (Z方向)

%% ========== 腿安装位置 (在身体坐标系中) ==========
% 每行: [x, y, z]  腿从身体伸出的起始点
% x正=向前, y正=向左, z正=向上
LEG_MOUNT = [
     BODY_LENGTH/2,  BODY_WIDTH/2, 0;   % FL (左前)
     BODY_LENGTH/2, -BODY_WIDTH/2, 0;   % FR (右前)
    -BODY_LENGTH/2,  BODY_WIDTH/2, 0;   % RL (左后)
    -BODY_LENGTH/2, -BODY_WIDTH/2, 0;   % RR (右后)
];

%% ========== 关节零位偏移 & 限位宏 (同单腿仿真) ==========
ZERO_OFFSET_THETA1_DEG = 30.0;   % θ1 零位
LOWER_LIMIT_THETA1_DEG = -60.0;  % θ1 下限 (指令角)
UPPER_LIMIT_THETA1_DEG =   0.0;  % θ1 上限 (指令角)

ZERO_OFFSET_THETA2_DEG = 0.0;
LOWER_LIMIT_THETA2_DEG = -45.0;
UPPER_LIMIT_THETA2_DEG =  90.0;

ZERO_OFFSET_THETA3_DEG = 0.0;
LOWER_LIMIT_THETA3_DEG = 60.0;
UPPER_LIMIT_THETA3_DEG = 180.0;

theta1_offset = deg2rad(ZERO_OFFSET_THETA1_DEG);
theta2_offset = deg2rad(ZERO_OFFSET_THETA2_DEG);
theta3_offset = deg2rad(ZERO_OFFSET_THETA3_DEG);

%% ========== 演示: 基本站立姿态 ==========
fprintf('========== 四足运动学演示 ==========\n');
fprintf('身体: %.2f x %.2f x %.2f m\n', BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT);
fprintf('连杆: L1=%.2f (髋偏), L2=%.2f (大腿), L3=%.2f (小腿)\n\n', L1, L2, L3);

% 每条腿的指令角: [θ1, θ2, θ3] (度)
% 自然站立: 大腿略前摆, 小腿后弯
q_stand_deg = [
     0, -30,  60;    % FL
     0, -30,  60;    % FR
     0, -30,  60;    % RL
     0, -30,  60;    % RR
];

q_stand = deg2rad(q_stand_deg);

% 计算各腿足端位置 (在身体坐标系中)
foot_pos_body = leg_FK_all(q_stand, L1, L2, L3, ...
    theta1_offset, theta2_offset, theta3_offset, LEG_MOUNT);

for i = 1:4
    leg_names = {'FL', 'FR', 'RL', 'RR'};
    fprintf('站立 %s: 关节 [%+.0f°, %+.0f°, %+.0f°]  →  足端 [%+.2f, %+.2f, %+.2f] m\n', ...
        leg_names{i}, ...
        rad2deg(q_stand(i,1)), rad2deg(q_stand(i,2)), rad2deg(q_stand(i,3)), ...
        foot_pos_body(i,1), foot_pos_body(i,2), foot_pos_body(i,3));
end

% 计算腿长信息和离地高度
avg_z = mean(foot_pos_body(:,3));
fprintf('\n离地高度: %.0f mm (地面在 Z=%.2f)\n', avg_z*1000, avg_z);

%% ========== 整体可视化 ==========
figure('Name', '四足机器狗 3D 模型', 'Position', [50, 100, 1200, 800]);

% 默认姿态 (站立)
ax = axes('Position', [0.08, 0.08, 0.88, 0.88]);
theta0 = q_stand;  % 4x3 矩阵
draw_quadruped(ax, theta0, L1, L2, L3, ...
    theta1_offset, theta2_offset, theta3_offset, LEG_MOUNT, BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT);

%% ========== 交互式 GUI ==========
build_quadruped_gui(L1, L2, L3, ...
    LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG, ...
    LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG, ...
    LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG, ...
    theta1_offset, theta2_offset, theta3_offset, ...
    LEG_MOUNT, BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT);

%% ============================================================
%                     运动学函数
% ============================================================

function p = fk_leg(q_cmd, L1, L2, L3, off1, off2, off3)
    % 正运动学: 单腿 指令角 → 足端位置 (在髋关节坐标系中)
    % 与 leg_kinematics.m 中的 fk() 完全一致

    t1_phys = q_cmd(1) + off1;
    t2_phys = q_cmd(2) + off2;
    t3_phys = q_cmd(3) + off3;

    t2_int = -t2_phys;
    t3_int = -t3_phys;

    A = sin(t2_int)*L2 + sin(t2_int+t3_int)*L3;
    B = L1;
    C = -cos(t2_int)*L2 - cos(t2_int+t3_int)*L3;

    px = -A;
    py = B*cos(t1_phys) - C*sin(t1_phys);
    pz = B*sin(t1_phys) + C*cos(t1_phys);

    p = [px; py; pz];
end

function q_cmd = ik_leg(p_target, L1, L2, L3, off1, off2, off3)
    % 逆运动学: 单腿 足端位置 → 指令角 (在髋关节坐标系中)
    % 与 leg_kinematics.m 中的 ik() 完全一致

    px = p_target(1); py = p_target(2); pz = p_target(3);

    r_yz = sqrt(py^2 + pz^2);
    if r_yz < abs(L1) + 1e-9
        r_yz = abs(L1) + 1e-6;
    end
    t1_phys = real(pi - asin(L1 / r_yz) - atan2(py, pz));

    D = real(sqrt(max(r_yz^2 - L1^2, 0)));
    A = -px;

    numerator = A^2 + D^2 - L2^2 - L3^2;
    denominator = 2 * L2 * L3;

    if abs(denominator) < 1e-12
        t3_int = 0;
    else
        cos_t3 = numerator / denominator;
        cos_t3 = max(-1, min(1, cos_t3));
        t3_int = -acos(cos_t3);
    end

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

    t2_phys = -t2_int;
    t3_phys = -t3_int;
    q_cmd = [t1_phys - off1; t2_phys - off2; t3_phys - off3];
end

function R = hip_rotation_matrix(leg_idx)
    % 返回 3×3 旋转矩阵: 将髋关节坐标系中的点变换到身体坐标系
    %
    % 髋坐标系: X+向后, Y+向外翻, Z+向上
    % 身体坐标系: X+向前, Y+向左, Z+向上
    %
    % leg_idx: 1=FL, 2=FR, 3=RL, 4=RR

    switch leg_idx
        case {1, 3}  % 左腿: 向外 = 向左 = +Ybody
            % hip_X = [-1, 0, 0]'   (X+hip = -Xbody)
            % hip_Y = [0, +1, 0]'   (Y+hip = +Ybody, 向左)
            % hip_Z = [0, 0, 1]'    (Z+hip = +Zbody, 向上)
            R = [-1,  0, 0;
                  0, +1, 0;
                  0,  0, 1];
        case {2, 4}  % 右腿: 向外 = 向右 = -Ybody
            % hip_Y = [0, -1, 0]'   (Y+hip = -Ybody, 向右)
            R = [-1,  0, 0;
                  0, -1, 0;
                  0,  0, 1];
        otherwise
            R = eye(3);
    end
end

function foot_body = leg_FK_all(q_all, L1, L2, L3, off1, off2, off3, LEG_MOUNT)
    % 四腿正运动学: 12关节角 → 4足端位置 (身体坐标系)
    % q_all: 4×3 矩阵, 每行 [θ1, θ2, θ3]
    % foot_body: 4×3 矩阵, 每行 [px, py, pz] (身体坐标系)
    %
    % 归一化向量定义:
    %   body_X+ = 向前
    %   body_Y+ = 向左
    %   body_Z+ = 向上

    foot_body = zeros(4, 3);

    for leg = 1:4
        % 1. 单腿 FK: 在髋关节坐标系中的足端位置
        p_hip = fk_leg(q_all(leg,:), L1, L2, L3, off1, off2, off3);

        % 2. 髋坐标系 → 身体坐标系
        R = hip_rotation_matrix(leg);
        p_body = R * p_hip;

        % 3. 加上髋关节安装位置偏移
        foot_body(leg, :) = LEG_MOUNT(leg, :) + p_body';
    end
end

%% ============================================================
%                     可视化函数
% ============================================================

function draw_quadruped(ax, q_all, L1, L2, L3, off1, off2, off3, LEG_MOUNT, BL, BW, BH)
    % 绘制四足机器狗 3D 模型
    % q_all: 4×3 关节角矩阵

    cla(ax); hold(ax, 'on');

    % ---- 计算各腿关键点 (身体坐标系) ----
    % 每腿: [髋, 髋屈, 膝, 足] 在身体坐标系中的位置
    leg_points = cell(4, 1);

    for leg = 1:4
        R = hip_rotation_matrix(leg);

        % 髋关节固定点 (安装位置)
        p_hip_mount = LEG_MOUNT(leg, :)';

        % 髋屈曲关节 (θ1 旋转后, θ2=θ3=0)
        p_hip_flex_hip = fk_leg([q_all(leg,1), 0, 0], L1, 0, 0, off1, off2, off3);
        p_hip_flex_body = p_hip_mount + R * p_hip_flex_hip;

        % 膝关节 (θ1,θ2 旋转后, θ3=0)
        p_knee_hip = fk_leg([q_all(leg,1), q_all(leg,2), 0], L1, L2, 0, off1, off2, off3);
        p_knee_body = p_hip_mount + R * p_knee_hip;

        % 足端 (全部关节)
        p_foot_hip = fk_leg(q_all(leg,:), L1, L2, L3, off1, off2, off3);
        p_foot_body = p_hip_mount + R * p_foot_hip;

        leg_points{leg} = [p_hip_mount, p_hip_flex_body, p_knee_body, p_foot_body];
    end

    % ---- 绘制身体 (长方体) ----
    % 身体八顶点
    hb = BH/2;
    body_verts = [
         BL/2,  BW/2, -hb;   % 1 前上左
         BL/2, -BW/2, -hb;   % 2 前上右
        -BL/2, -BW/2, -hb;   % 3 后上右
        -BL/2,  BW/2, -hb;   % 4 后上左
         BL/2,  BW/2,  hb;   % 5 前下左
         BL/2, -BW/2,  hb;   % 6 前下右
        -BL/2, -BW/2,  hb;   % 7 后下右
        -BL/2,  BW/2,  hb;   % 8 后下左
    ]';
    body_faces = [
        1 2 3 4;  % 上面
        5 6 7 8;  % 下面
        1 2 6 5;  % 前面
        2 3 7 6;  % 右面
        3 4 8 7;  % 后面
        4 1 5 8;  % 左面
    ]';
    patch(ax, 'Vertices', body_verts', 'Faces', body_faces', ...
        'FaceColor', [0.2 0.6 0.8], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.2);

    % ---- 绘制四条腿 ----
    leg_colors = [0.85, 0.33, 0.10;  % FL 橙色
                  0.85, 0.60, 0.10;  % FR 金色
                  0.10, 0.60, 0.85;  % RL 青色
                  0.60, 0.10, 0.85]; % RR 紫色

    leg_labels = {'FL', 'FR', 'RL', 'RR'};

    for leg = 1:4
        pts = leg_points{leg};
        % pts: [hip_mount, hip_flex, knee, foot]  每列一个点

        color = leg_colors(leg, :);

        % 髋偏移 (hip_mount → hip_flex)
        plot3(ax, [pts(1,1), pts(1,2)], [pts(2,1), pts(2,2)], [pts(3,1), pts(3,2)], ...
            'Color', [0.3 0.3 0.3], 'LineWidth', 4);

        % 大腿 (hip_flex → knee)
        plot3(ax, [pts(1,2), pts(1,3)], [pts(2,2), pts(2,3)], [pts(3,2), pts(3,3)], ...
            'Color', color, 'LineWidth', 5);

        % 小腿 (knee → foot)
        plot3(ax, [pts(1,3), pts(1,4)], [pts(2,3), pts(2,4)], [pts(3,3), pts(3,4)], ...
            'Color', color*0.7, 'LineWidth', 5);

        % 关节标记
        plot3(ax, pts(1,1), pts(2,1), pts(3,1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        plot3(ax, pts(1,2), pts(2,2), pts(3,2), 'ks', 'MarkerSize', 7, 'MarkerFaceColor', [0.5 0.5 0.5]);
        plot3(ax, pts(1,3), pts(2,3), pts(3,3), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'w');
        plot3(ax, pts(1,4), pts(2,4), pts(3,4), 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);

        % 腿标签
        text(ax, pts(1,1), pts(2,1), pts(3,1)+0.02, leg_labels{leg}, ...
            'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end

    % ---- 足端连线 (左右对称参考) ----
    foot_pos = zeros(3, 4);
    for leg = 1:4
        foot_pos(:, leg) = leg_points{leg}(:, 4);
    end
    % 前腿连线
    plot3(ax, [foot_pos(1,1), foot_pos(1,2)], [foot_pos(2,1), foot_pos(2,2)], ...
               [foot_pos(3,1), foot_pos(3,2)], 'k:', 'LineWidth', 0.8);
    % 后腿连线
    plot3(ax, [foot_pos(1,3), foot_pos(1,4)], [foot_pos(2,3), foot_pos(2,4)], ...
               [foot_pos(3,3), foot_pos(3,4)], 'k:', 'LineWidth', 0.8);
    % 左侧前后连线
    plot3(ax, [foot_pos(1,1), foot_pos(1,3)], [foot_pos(2,1), foot_pos(2,3)], ...
               [foot_pos(3,1), foot_pos(3,3)], 'k:', 'LineWidth', 0.5);
    % 右侧前后连线
    plot3(ax, [foot_pos(1,2), foot_pos(1,4)], [foot_pos(2,2), foot_pos(2,4)], ...
               [foot_pos(3,2), foot_pos(3,4)], 'k:', 'LineWidth', 0.5);

    % ---- 坐标系指示 ----
    quiver3(ax, 0, 0, -BH/2, 0.15, 0, 0, 'r-', 'LineWidth', 2, 'MaxHeadSize', 0.35);
    quiver3(ax, 0, 0, -BH/2, 0, 0.15, 0, 'g-', 'LineWidth', 2, 'MaxHeadSize', 0.35);
    quiver3(ax, 0, 0, -BH/2, 0, 0, 0.15, 'b-', 'LineWidth', 2, 'MaxHeadSize', 0.35);
    text(ax, 0.16, 0, -BH/2, 'X (前)', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 11);
    text(ax, 0, 0.16, -BH/2, 'Y (左)', 'Color', 'g', 'FontWeight', 'bold', 'FontSize', 11);
    text(ax, 0, 0, -BH/2+0.16, 'Z (上)', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 11);

    % ---- 地面参考 ----
    gx = linspace(-0.5, 0.5, 5);
    gy = linspace(-0.3, 0.3, 4);
    [GX, GY] = meshgrid(gx, gy);
    GZ = zeros(size(GX)) + min(foot_pos(3,:)) - 0.01;
    GZ_fill = GZ(1,1);
    mesh(ax, GX, GY, GZ, 'EdgeColor', [0.6 0.6 0.6], 'FaceAlpha', 0, 'LineStyle', ':');
    fill3(ax, [0.5, 0.5, -0.5, -0.5], [0.3, -0.3, -0.3, 0.3], ...
        [1,1,1,1]*GZ_fill, [0.85 0.85 0.85], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

    % ---- 视图设置 ----
    axis(ax, 'equal');
    xlim(ax, [-0.45, 0.45]);
    ylim(ax, [-0.35, 0.35]);
    zlim(ax, [-0.45, 0.25]);
    grid(ax, 'on');
    xlabel(ax, 'X (向前)'); ylabel(ax, 'Y (向左)'); zlabel(ax, 'Z (向上)');
    view(ax, 120, 20);
end

%% ============================================================
%                     交互式 GUI
% ============================================================

function build_quadruped_gui(L1, L2, L3, ...
        LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG, ...
        LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG, ...
        LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG, ...
        off1, off2, off3, LEG_MOUNT, BL, BW, BH)

    fig = figure('Name', '四足机器狗 — 交互仿真', ...
        'Position', [50, 30, 1520, 800], ...
        'NumberTitle', 'off');

    % 初始关节角 (统一站立)
    q0_deg = [0, -30, 60; 0, -30, 60; 0, -30, 60; 0, -30, 60];
    q_all = deg2rad(q0_deg);

    % ===== 主绘图区 =====
    ax = axes('Parent', fig, 'Units', 'pixels', ...
        'Position', [80, 180, 880, 580]);
    draw_quadruped(ax, q_all, L1, L2, L3, off1, off2, off3, LEG_MOUNT, BL, BW, BH);

    % ===== 底部信息栏 =====
    info_h = uicontrol('Parent', fig, 'Style', 'text', ...
        'Position', [80, 15, 1100, 30], ...
        'FontSize', 12, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');

    % ===== 右侧 12 关节界面 (四行, 每行一条腿) =====
    % 布局参数
    px_panel = 1000;          % 右侧面板起始 X
    sw = 95;                  % 滑块宽度
    ew = 42;                  % 编辑框宽度
    gap_joint = 10;           % 关节间距
    gap_se = 3;               % 滑块与编辑框间距

    leg_labels = {'FL', 'FR', 'RL', 'RR'};
    joint_tags = {'θ₁', 'θ₂', 'θ₃'};
    row_colors = {[0.85,0.33,0.10], [0.85,0.60,0.10], [0.10,0.60,0.85], [0.60,0.10,0.85]};

    lim_arr = [LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG;
               LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG;
               LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG];

    init_deg = [0, -30, 60; 0, -30, 60; 0, -30, 60; 0, -30, 60];

    % 每个 joint 占据的宽度
    cell_w = sw + gap_se + ew + gap_joint;  % ~150px
    row_w = 40 + 3 * cell_w;  % leg_label + 3 joints ≈ 490px

    % 行 Y 位置
    row_y = [565, 515, 465, 415];
    row_h = 38;

    % 存储句柄
    h_slider = zeros(4, 3);
    h_edit   = zeros(4, 3);

    for leg = 1:4
        x_label = px_panel;
        x0 = x_label + 40;  % leg label 宽 35px

        % 腿标签
        uicontrol('Parent', fig, 'Style', 'text', ...
            'Position', [x_label, row_y(leg), 35, row_h], ...
            'String', leg_labels{leg}, ...
            'FontSize', 12, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', ...
            'ForegroundColor', row_colors{leg});

        for joint = 1:3
            x = x0 + (joint-1) * cell_w;
            val0 = init_deg(leg, joint);
            lo = lim_arr(joint, 1);
            hi = lim_arr(joint, 2);

            % 编辑框 (可回车输入)
            h_edit(leg, joint) = uicontrol('Parent', fig, 'Style', 'edit', ...
                'Position', [x, row_y(leg)+2, ew, 20], ...
                'String', sprintf('%.0f', val0), ...
                'FontSize', 9, 'BackgroundColor', 'w', ...
                'HorizontalAlignment', 'center', ...
                'Tag', sprintf('e_%d_%d', leg, joint));

            % 滑块
            h_slider(leg, joint) = uicontrol('Parent', fig, 'Style', 'slider', ...
                'Position', [x+ew+gap_se, row_y(leg), sw, 20], ...
                'Min', lo, 'Max', hi, 'Value', val0, ...
                'Tag', sprintf('s_%d_%d', leg, joint));

            % joint 小标签 (放在编辑框上方或旁边)
            uicontrol('Parent', fig, 'Style', 'text', ...
                'Position', [x-1, row_y(leg)+row_h-10, 20, 10], ...
                'String', joint_tags{joint}, ...
                'FontSize', 7, 'ForegroundColor', [0.4 0.4 0.4]);
        end
    end

    % ===== 预设 + 同步按钮 =====
    btn_y = 370;
    btn_w = 65;
    gap_btn = 12;
    x_btn = px_panel + (row_w - 4*btn_w - 3*gap_btn)/2;

    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [x_btn, btn_y, btn_w, 28], ...
        'String', '站立', 'FontSize', 9, ...
        'Callback', @(s,e) quad_preset());

    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [x_btn+btn_w+gap_btn, btn_y, btn_w, 28], ...
        'String', '前倾', 'FontSize', 9, ...
        'Callback', @(s,e) quad_preset());

    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [x_btn+2*(btn_w+gap_btn), btn_y, btn_w, 28], ...
        'String', '屈腿', 'FontSize', 9, ...
        'Callback', @(s,e) quad_preset());

    uicontrol('Parent', fig, 'Style', 'pushbutton', ...
        'Position', [x_btn+3*(btn_w+gap_btn), btn_y, btn_w, 28], ...
        'String', '全同步', 'FontSize', 9, ...
        'Callback', @(s,e) quad_sync());

    % ===== 存储共享数据 =====
    cb_data = struct('ax', ax, ...
        'L', [L1, L2, L3], 'off', [off1, off2, off3], ...
        'mount', LEG_MOUNT, 'body', [BL, BW, BH], ...
        'info', info_h, 'fig', fig, ...
        'h_slider', h_slider, 'h_edit', h_edit, ...
        'lim', lim_arr);
    guidata(fig, cb_data);

    % ===== 设置回调 =====
    for leg = 1:4
        for joint = 1:3
            set(h_slider(leg, joint), 'Callback', @(s,e) quad_update_from_slider());
            set(h_edit(leg, joint), 'Callback', @(s,e) quad_update_from_edit());
        end
    end

    % 初始更新
    q_init = deg2rad(init_deg);
    update_quad_info(info_h, q_init, L1, L2, L3, off1, off2, off3, LEG_MOUNT);
end

%% ---------- 滑块更新 (从滑块读值) ----------
function quad_update_from_slider()
    fig = gcbf; h = gcbo;
    if isempty(fig) || isempty(h), return; end
    cb = guidata(fig);
    if isempty(cb), return; end

    % 找到被拖动的滑块在数组中的索引
    idx = find(cb.h_slider == h);
    if isempty(idx), return; end

    v = get(h, 'Value');
    set(cb.h_edit(idx), 'String', sprintf('%.0f', round(v)));

    quad_update_all(fig, cb);
end

%% ---------- 编辑框更新 (回车触发) ----------
function quad_update_from_edit()
    fig = gcbf; h = gcbo;
    if isempty(fig) || isempty(h), return; end
    cb = guidata(fig);
    if isempty(cb), return; end

    idx = find(cb.h_edit == h);
    if isempty(idx), return; end

    str = get(h, 'String');
    v = str2double(str);
    if isnan(v)
        v = get(cb.h_slider(idx), 'Value');
        set(h, 'String', sprintf('%.0f', round(v)));
        return;
    end

    % 确定是哪个关节 (1..3)
    joint = mod(idx-1, 3) + 1;
    lo = cb.lim(joint, 1);
    hi = cb.lim(joint, 2);
    v = max(lo, min(hi, v));

    set(cb.h_slider(idx), 'Value', v);
    set(cb.h_edit(idx), 'String', sprintf('%.0f', round(v)));

    quad_update_all(fig, cb);
end

%% ---------- 通用更新: 读取所有滑块, 刷新绘图和信息 ----------
function quad_update_all(fig, cb)
    ax = cb.ax; L = cb.L; off = cb.off; mount = cb.mount;
    BL = cb.body(1); BW = cb.body(2); BH = cb.body(3);

    q_deg = zeros(4, 3);
    for leg = 1:4
        for joint = 1:3
            v = get(cb.h_slider(leg, joint), 'Value');
            q_deg(leg, joint) = v;
        end
    end

    q_all = deg2rad(q_deg);
    draw_quadruped(ax, q_all, L(1), L(2), L(3), off(1), off(2), off(3), ...
        mount, BL, BW, BH);
    update_quad_info(cb.info, q_all, L(1), L(2), L(3), off(1), off(2), off(3), mount);
    drawnow;
end

%% ---------- 预设回调 ----------
function quad_preset()
    fig = gcbf;
    if isempty(fig), return; end
    cb = guidata(fig);
    if isempty(cb), return; end

    label = get(gcbo, 'String');

    switch label
        case '站立'
            q_deg = [0, -30, 60; 0, -30, 60; 0, -30, 60; 0, -30, 60];
        case '前倾'
            q_deg = [0, -15, 45; 0, -15, 45; 0, -40, 70; 0, -40, 70];
        case '屈腿'
            q_deg = [0, -10, 80; 0, -10, 80; 0, -10, 80; 0, -10, 80];
        otherwise
            return;
    end

    for leg = 1:4
        for joint = 1:3
            v = q_deg(leg, joint);
            set(cb.h_slider(leg, joint), 'Value', v);
            set(cb.h_edit(leg, joint), 'String', sprintf('%.0f', round(v)));
        end
    end

    quad_update_all(fig, cb);
end

%% ---------- 全同步: FL 同步到所有腿 ----------
function quad_sync()
    fig = gcbf;
    if isempty(fig), return; end
    cb = guidata(fig);
    if isempty(cb), return; end

    for joint = 1:3
        v = get(cb.h_slider(1, joint), 'Value');
        for leg = 2:4
            set(cb.h_slider(leg, joint), 'Value', v);
            set(cb.h_edit(leg, joint), 'String', sprintf('%.0f', round(v)));
        end
    end

    quad_update_all(fig, cb);
end

%% ---------- 信息更新 ----------
function update_quad_info(h, q_all, L1, L2, L3, off1, off2, off3, LEG_MOUNT)
    foot = leg_FK_all(q_all, L1, L2, L3, off1, off2, off3, LEG_MOUNT);
    avg_z = mean(foot(:,3));
    min_z = min(foot(:,3));
    leg_names = {'FL', 'FR', 'RL', 'RR'};

    % 关节角汇总
    deg_str = '';
    for i = 1:4
        deg_str = [deg_str, sprintf('%s[%.0f,%.0f,%.0f]  ', ...
            leg_names{i}, rad2deg(q_all(i,1)), rad2deg(q_all(i,2)), rad2deg(q_all(i,3)))];
    end

    set(h, 'String', sprintf(...
        '足端平均 Z=%.0f mm (最低 %.0f)  |  %s', ...
        avg_z*1000, min_z*1000, deg_str));
end