clear;
clc;

% Build the Robot Model
L(1) = Link([0     0        0.150   -pi/2    0], 'standard'); 
L(2) = Link([0     0        0.700    0       0], 'standard');
L(3) = Link([0     0        0.109   -pi/2    0], 'standard');
L(4) = Link([0     0.600    0        pi/2    0], 'standard');
L(5) = Link([0     0        0       -pi/2    0], 'standard');
L(6) = Link([0     0.065    0        0       0], 'standard');
robot = SerialLink(L, 'name', '6DOF_Robot');

% Verify Forward Kinematics
q_init = [0 0 0 0 0 0];
T_init = robot.fkine(q_init).T

% Define the Spatial Trajectory
% Define the Spatial Trajectory
T = 10;            % total time in seconds
n = 100;          % number of points
t = linspace(0, T, n);  % time vector from 0 to 5 seconds
T_A = T_init;     % initial transformation matrix
P_A = T_init(1:3, 4)
% Define target position and orientation
P_B = [2; 1; 1];  % target position (point B), absolute coordinates
R_B = eul2rotm([0, 0, 0], 'XYZ');
T_B = [R_B, P_B; 0, 0, 0, 1];  % target transformation matrix

% Generate trajectory of transformation matrices
T_traj = cell(1, n);
for i = 1:n
    s = t(i)/T;
    T_traj{i} = trinterp(T_A, T_B, s);
end

% Extract desired positions for plotting
pos_desired = zeros(n, 3);
for i = 1:n
    pos_desired(i,:) = T_traj{i}(1:3, 4)';
end
xd=pos_desired(:,1);
yd=pos_desired(:,2);
zd=pos_desired(:,3);

% Inverse Kinematics to Compute Joint Angles
q_now = q_init;
q_traj = zeros(n, 6);
for i = 1:n
    next_x =  xd(i)
    next_y =  yd(i)
    next_z =  zd(i)
    R_init = T_init(1:3, 1:3);           % 提取 3x3 旋转矩阵
    p_target = [next_x; next_y; next_z]; % 目标位置向量 (3x1)
    T_target = [R_init, p_target; 0, 0, 0, 1]; % 组合成新的 4x4 变换矩阵
    %T_target = transl(next_x, next_y, next_z) * T_init(1:3, 1:3);
    q_now = robot.ikcon(T_target, q_now);
    q_traj(i,:) = q_now;
end

% Forward Kinematics Verification
pos_computed = zeros(n, 3);
for i = 1:n
    T_computed = robot.fkine(q_traj(i,:)).T;
    pos_computed(i,:) = T_computed(1:3, 4)';
end

% Output Initial and Final Joint Angles
disp('Initial joint angles (degrees):');
disp(q_traj(1,:) * 180/pi);
disp('Final joint angles (degrees):');
disp(q_traj(end,:) * 180/pi);

% Visualization
% Animation
figure;
for i = 1:5:n
    robot.plot(q_traj(i,:));
    hold on;
    plot3(T_init(1,4) + xd, T_init(2,4) + yd, T_init(3,4) + zd, 'r:', 'LineWidth', 2);
    pause(0.2);
    if i < n
        hold off;
    end
end

% Joint Angle Plot
figure;
plot(t, q_traj * 180/pi, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Joint Angle (deg)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
title('Joint Trajectories');

% 正确版：提取 T_traj 的位移量
desired_pos = zeros(n, 3);
for i = 1:n
    desired_pos(i, :) = T_traj{i}(1:3, 4)';
end

% 绘制
figure;
plot3(desired_pos(:,1), desired_pos(:,2), desired_pos(:,3), 'b-', 'LineWidth', 2);  % 期望轨迹
hold on;
plot3(pos_computed(:,1), pos_computed(:,2), pos_computed(:,3), 'g--', 'LineWidth', 1.5); % 实际轨迹
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
legend('Desired', 'Computed');
title('End-Effector Spatial Trajectory');
axis equal;
view(3);
