% Load the Puma 560 model
mdl_puma560;

% Define the target position and orientation
p = [0.5 0.2 0.1]; % New target position (x, y, z)
T = transl(p) * troty(pi/2); % Target transformation matrix

% Define initial and target joint configurations
qr = [-pi/2 0 0 0 0 0]; % Initial joint configuration (6DOF)
try
    qqr = p560.ikine6s(T, 'ru'); % Target joint configuration using inverse kinematics
catch
    error('Inverse kinematics failed. The target position [%f, %f, %f] may be unreachable.', p(1), p(2), p(3));
end

% Define trajectory parameters
T_total = 10; % Total time in seconds
n = 100; % Number of points
t = linspace(0, T_total, n); % Time vector

% Generate joint trajectory
q_traj = jtraj(qr, qqr, n); % Joint trajectory with n points

% Compute end-effector positions using forward kinematics
pos_computed = zeros(n, 3);
for i = 1:n
    T_computed = p560.fkine(q_traj(i,:)); % Forward kinematics
    pos_computed(i,:) = T_computed.t'; % Extract translation (x, y, z)
end

% Approximate desired trajectory (straight line in Cartesian space)
T_init = p560.fkine(qr); % Initial transformation matrix
P_A = T_init.t'; % Initial position (1x3 vector)
P_B = p; % Target position (1x3 vector)
pos_desired = zeros(n, 3);
for i = 1:n
    s = t(i)/T_total; % Interpolation parameter
    pos_desired(i,:) = (1-s)*P_A + s*P_B; % Linear interpolation
end

% Joint Angle Plot
figure;
plot(t, q_traj * 180/pi, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Joint Angle (deg)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
title('Joint Trajectories');

% End-Effector Spatial Trajectory Plot
figure;
plot3(pos_desired(:,1), pos_desired(:,2), pos_desired(:,3), 'b-', 'LineWidth', 2); % Desired trajectory
hold on;
plot3(pos_computed(:,1), pos_computed(:,2), pos_computed(:,3), 'g--', 'LineWidth', 1.5); % Computed trajectory
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
legend('Desired', 'Computed');
title('End-Effector Spatial Trajectory');
axis equal;
view(3);

% Original visualization
figure;
clf;
%plot_sphere(p, 0.05, 'y'); % Use updated target position for sphere
ae = [138 8]; % View angle
p560.plot3d(q_traj, 'view', ae);