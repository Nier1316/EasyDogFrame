%% 四足机器狗 — TCP 实时控制仿真
%
% 用法:
%   quadruped_realtime           — 默认端口 12345
%   quadruped_realtime(12346)    — 自定义端口
%
% C++ 端发送 12 个关节角（度, float32）:
%   [FLθ1, FLθ2, FLθ3, FRθ1, FRθ2, FRθ3,
%    RLθ1, RLθ2, RLθ3, RRθ1, RRθ2, RRθ3]
%   每帧 12×float32 = 48 字节, 二进制 Little-Endian
%
% 协议: TCP 客户端连接后持续发 48 字节帧即可
%
% 按 Ctrl+C 或关闭图窗退出

function quadruped_realtime(port)
    if nargin < 1
        port = 12345;
    end

    %% ===== 机器人参数 =====
    L1 = 0.05;  L2 = 0.20;  L3 = 0.20;
    BODY_LENGTH = 0.30;  BODY_WIDTH = 0.12;  BODY_HEIGHT = 0.06;
    LEG_MOUNT = [  BODY_LENGTH/2,  BODY_WIDTH/2, 0;
                   BODY_LENGTH/2, -BODY_WIDTH/2, 0;
                  -BODY_LENGTH/2,  BODY_WIDTH/2, 0;
                  -BODY_LENGTH/2, -BODY_WIDTH/2, 0 ];
    ZERO_OFFSET_THETA1_DEG = 30.0;   LOWER_LIMIT_THETA1_DEG = -60.0;  UPPER_LIMIT_THETA1_DEG = 0.0;
    ZERO_OFFSET_THETA2_DEG = 0.0;    LOWER_LIMIT_THETA2_DEG = -45.0;  UPPER_LIMIT_THETA2_DEG = 90.0;
    ZERO_OFFSET_THETA3_DEG = 0.0;    LOWER_LIMIT_THETA3_DEG = 60.0;   UPPER_LIMIT_THETA3_DEG = 180.0;
    off = deg2rad([ZERO_OFFSET_THETA1_DEG, ZERO_OFFSET_THETA2_DEG, ZERO_OFFSET_THETA3_DEG]);

    %% ===== 创建图窗 =====
    fig = figure('Name', '四足机器狗 — 实时控制 (TCP)', ...
        'Position', [100, 100, 1000, 800], ...
        'NumberTitle', 'off', ...
        'CloseRequestFcn', @(~,~) setappdata(gcbf,'running',false));
    ax = axes('Position', [0.08, 0.08, 0.88, 0.88]);

    % 初始站立
    q_init = deg2rad([0, -30, 60; 0, -30, 60; 0, -30, 60; 0, -30, 60]);
    draw_quadruped(ax, q_init, L1, L2, L3, off(1), off(2), off(3), LEG_MOUNT, BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT);
    h_title = get(ax, 'Title');

    % 存储接收到的数据

    setappdata(fig, 'running',  true);
    setappdata(fig, 'q_deg',    [0, -30, 60; 0, -30, 60; 0, -30, 60; 0, -30, 60]);
    setappdata(fig, 'L',        [L1, L2, L3]);
    setappdata(fig, 'off',      off);
    setappdata(fig, 'ax',       ax);
    setappdata(fig, 'h_title',  h_title);
    setappdata(fig, 'LEG_MOUNT',  LEG_MOUNT);
    setappdata(fig, 'BODY',     [BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT]);
    setappdata(fig, 'LIM',      [LOWER_LIMIT_THETA1_DEG, UPPER_LIMIT_THETA1_DEG;
                                  LOWER_LIMIT_THETA2_DEG, UPPER_LIMIT_THETA2_DEG;
                                  LOWER_LIMIT_THETA3_DEG, UPPER_LIMIT_THETA3_DEG]);
    % 帧计数器
    setappdata(fig, 'frame_count', 0);

    %% ===== 启动 TCP 服务器 =====
    try
        server = tcpserver("0.0.0.0", port, "Timeout", 30);
        fprintf('▶ TCP 服务器已启动 (端口 %d)\n', port);
        fprintf('  等待 C++ 客户端连接 127.0.0.1:%d ...\n', port);
        set(h_title, 'String', sprintf('TCP 服务器已启动 (端口 %d)，等待连接...', port));
    catch ME
        errordlg(sprintf('无法启动 TCP 服务器 (端口 %d 可能被占用)\n\n%s', port, ME.message), 'TCP 错误');
        delete(fig);
        return;
    end

    % 缓冲区
    recv_buf = uint8([]);

    %% ===== 主循环: 轮询 TCP + 刷新绘图 =====
    running = true;
    while running
        running = getappdata(fig, 'running');
        if ~running, break; end

        % ---- 轮询 TCP 数据 ----
        if server.Connected && server.NumBytesAvailable > 0
            bytes = read(server, server.NumBytesAvailable, 'uint8');
            recv_buf = [recv_buf; bytes(:)];

            LIM = getappdata(fig, 'LIM');
            % 尽可能多地从缓冲区提取完整帧
            while length(recv_buf) >= 48
                frame = recv_buf(1:48);
                recv_buf(1:48) = [];

                data = double(typecast(uint8(frame), 'single'));
                if numel(data) >= 12 && ~any(isnan(data)) && ~any(isinf(data))
                    data = data(1:12);
                    for j = 1:3:10
                        data(j)   = max(LIM(1,1), min(LIM(1,2), data(j)));
                        data(j+1) = max(LIM(2,1), min(LIM(2,2), data(j+1)));
                        data(j+2) = max(LIM(3,1), min(LIM(3,2), data(j+2)));
                    end
                    q_deg = reshape(data, 3, 4)';
                    setappdata(fig, 'q_deg', q_deg);
                    setappdata(fig, 'frame_count', getappdata(fig, 'frame_count') + 1);
                end
            end
        end

        % 从 appdata 读最新的关节角并更新
        q_deg = getappdata(fig, 'q_deg');
        q_all = deg2rad(q_deg);
        draw_quadruped(ax, q_all, L1, L2, L3, off(1), off(2), off(3), ...
            LEG_MOUNT, BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT);

        % 更新标题（显示连接和帧数状态）
        if server.Connected
            fc = getappdata(fig, 'frame_count');
            set(h_title, 'String', ...
                sprintf('● 已连接  |  帧 %d  |  FL[%.0f, %.0f, %.0f]', ...
                fc, q_deg(1,1), q_deg(1,2), q_deg(1,3)));
        else
            set(h_title, 'String', sprintf('○ 等待 C++ 客户端连接 %s:%d ...', '127.0.0.1', port));
        end

        drawnow;
        pause(0.03);  % ~33fps
    end

    %% ===== 清理 =====
    fprintf('■ 仿真已停止\n');
    try
        configureCallback(server, "byte", 48, []);
        delete(server);
    catch
    end
    if ishandle(fig)
        delete(fig);
    end
end

%% ============================================================
%  内嵌函数（同 quadruped_kinematics.m）
%% ============================================================

function p = fk_leg(q_cmd, L1, L2, L3, off1, off2, off3)
    t1_phys = q_cmd(1)+off1; t2_phys = q_cmd(2)+off2; t3_phys = q_cmd(3)+off3;
    t2_int = -t2_phys; t3_int = -t3_phys;
    A = sin(t2_int)*L2 + sin(t2_int+t3_int)*L3; B = L1;
    C = -cos(t2_int)*L2 - cos(t2_int+t3_int)*L3;
    p = [-A; B*cos(t1_phys)-C*sin(t1_phys); B*sin(t1_phys)+C*cos(t1_phys)];
end

function R = hip_rotation_matrix(leg_idx)
    switch leg_idx
        case {1,3}, R = [-1,0,0; 0,1,0; 0,0,1];
        case {2,4}, R = [-1,0,0; 0,-1,0; 0,0,1];
        otherwise,  R = eye(3);
    end
end

function draw_quadruped(ax, q_all, L1, L2, L3, off1, off2, off3, LEG_MOUNT, BL, BW, BH)
    cla(ax); hold(ax, 'on');

    % ---- 绘制身体 (长方体) ----
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
    leg_colors = [0.85,0.33,0.10; 0.85,0.60,0.10; 0.10,0.60,0.85; 0.60,0.10,0.85];
    leg_labels = {'FL','FR','RL','RR'};
    foot_pos = zeros(3,4);
    for leg = 1:4
        R = hip_rotation_matrix(leg); mount = LEG_MOUNT(leg,:)';
        fp  = mount + R*fk_leg([q_all(leg,1),0,0],            L1,0,0,  off1,off2,off3);
        kp  = mount + R*fk_leg([q_all(leg,1),q_all(leg,2),0], L1,L2,0, off1,off2,off3);
        ftp = mount + R*fk_leg(q_all(leg,:),                  L1,L2,L3,off1,off2,off3);
        c = leg_colors(leg,:);
        plot3(ax,[mount(1),fp(1)],[mount(2),fp(2)],[mount(3),fp(3)],'Color',[0.3,0.3,0.3],'LineWidth',4);
        plot3(ax,[fp(1),kp(1)],[fp(2),kp(2)],[fp(3),kp(3)],'Color',c,'LineWidth',5);
        plot3(ax,[kp(1),ftp(1)],[kp(2),ftp(2)],[kp(3),ftp(3)],'Color',c*0.7,'LineWidth',5);
        plot3(ax,mount(1),mount(2),mount(3),'ko','MarkerSize',8,'MarkerFaceColor','k');
        plot3(ax,fp(1),fp(2),fp(3),'ks','MarkerSize',7,'MarkerFaceColor',[0.5,0.5,0.5]);
        plot3(ax,kp(1),kp(2),kp(3),'ko','MarkerSize',7,'MarkerFaceColor','w');
        plot3(ax,ftp(1),ftp(2),ftp(3),'r*','MarkerSize',10,'LineWidth',1.5);
        text(ax,mount(1),mount(2),mount(3)+0.02,leg_labels{leg},'FontSize',9,'FontWeight','bold','HorizontalAlignment','center');
        foot_pos(:,leg) = ftp;
    end
    plot3(ax,[foot_pos(1,1),foot_pos(1,2)],[foot_pos(2,1),foot_pos(2,2)],[foot_pos(3,1),foot_pos(3,2)],'k:','LineWidth',0.8);
    plot3(ax,[foot_pos(1,3),foot_pos(1,4)],[foot_pos(2,3),foot_pos(2,4)],[foot_pos(3,3),foot_pos(3,4)],'k:','LineWidth',0.8);
    gz = min(foot_pos(3,:))-0.01; [gx,gy]=meshgrid(-0.5:0.1:0.5,-0.3:0.1:0.3);
    mesh(ax,gx,gy,zeros(size(gx))+gz,'EdgeColor',[0.6,0.6,0.6],'FaceAlpha',0,'LineStyle',':');
    quiver3(ax,0,0,-BH/2,0.15,0,0,'r-','LineWidth',2,'MaxHeadSize',0.35);
    quiver3(ax,0,0,-BH/2,0,0.15,0,'g-','LineWidth',2,'MaxHeadSize',0.35);
    quiver3(ax,0,0,-BH/2,0,0,0.15,'b-','LineWidth',2,'MaxHeadSize',0.35);
    text(ax,0.16,0,-BH/2,'X','Color','r','FontWeight','bold');
    text(ax,0,0.16,-BH/2,'Y','Color','g','FontWeight','bold');
    text(ax,0,0,-BH/2+0.16,'Z','Color','b','FontWeight','bold');
    axis(ax,'equal'); xlim(ax,[-0.45,0.45]); ylim(ax,[-0.35,0.35]); zlim(ax,[-0.45,0.25]);
    grid(ax,'on'); xlabel(ax,'X (向前)'); ylabel(ax,'Y (向左)'); zlabel(ax,'Z (向上)');
    view(ax,120,20);
end