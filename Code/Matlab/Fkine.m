clc;
clear;
close all;

% =====================================================
% Single Leg Forward Kinematics using Peter Corke Toolbox
% =====================================================

%% Leg Parameters (mm)
thigh_length = 71.83;
shank_length = 64.96;

% Joint angles (radians)
theta1 = deg2rad(325);  % Hip pitch angle
theta2 = deg2rad(-55);  % Knee pitch angle

%% Create 2-DOF Leg Robot using DH parameters
% DH Convention: [theta, d, a, alpha]
% Link 1: Thigh (revolute joint at hip)
% Link 2: Shank (revolute joint at knee)

L(1) = Revolute('d', 0, 'a', thigh_length, 'alpha', 0, 'offset', 0);
L(2) = Revolute('d', 0, 'a', shank_length, 'alpha', 0, 'offset', 0);

% Create serial link robot
robot = SerialLink(L, 'name', 'Dog Leg');

%% Joint Configuration
q = [theta1, theta2];  % [hip_angle, knee_angle]

%% Forward Kinematics
T = robot.fkine(q);

% Extract end-effector position
try
    foot_pos = T.t';  % RTB 10.x
catch
    foot_pos = transl(T)';  % RTB 9.x
end

%% Display Results
fprintf('=== DH Parameter Table ===\n');
fprintf('Link |  θ (deg)  |  d (mm)  |  a (mm)  |  α (deg)\n');
fprintf('-----|-----------|----------|----------|----------\n');
fprintf('  1  |  %.2f   |   %.2f   |  %.2f  |   %.2f\n', rad2deg(q(1)), L(1).d, L(1).a, rad2deg(L(1).alpha));
fprintf('  2  |  %.2f   |   %.2f   |  %.2f  |   %.2f\n', rad2deg(q(2)), L(2).d, L(2).a, rad2deg(L(2).alpha));
fprintf('\nNote: θ values shown are the current joint angles\n');
fprintf('      In DH convention, θ is the variable for revolute joints\n\n');

fprintf('=== Forward Kinematics Results ===\n');
fprintf('Joint Angles: [%.2f°, %.2f°]\n', rad2deg(q(1)), rad2deg(q(2)));
fprintf('Thigh Length: %.2f mm\n', thigh_length);
fprintf('Shank Length: %.2f mm\n', shank_length);
fprintf('\nFoot Position (mm):\n');
fprintf('  X: %.2f\n', foot_pos(1));
fprintf('  Y: %.2f\n', foot_pos(2));
fprintf('  Z: %.2f\n', foot_pos(3));

fprintf('\n=== Homogeneous Transformation Matrix (T₀²) ===\n');
fprintf('End-effector transform from base to foot:\n\n');
if isobject(T)
    % RTB 10.x - SE3 object
    try
        T_matrix = T.T;
    catch
        T_matrix = double(T);
    end
else
    % RTB 9.x - already a matrix
    T_matrix = T;
end
fprintf('T = \n');
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T_matrix(1,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T_matrix(2,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T_matrix(3,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T_matrix(4,:));

fprintf('\nRotation Matrix (R):\n');
fprintf('  [%8.4f  %8.4f  %8.4f]\n', T_matrix(1:3,1)');
fprintf('  [%8.4f  %8.4f  %8.4f]\n', T_matrix(1:3,2)');
fprintf('  [%8.4f  %8.4f  %8.4f]\n', T_matrix(1:3,3)');

fprintf('\nPosition Vector (P):\n');
fprintf('  [%8.4f  %8.4f  %8.4f]ᵀ mm\n', T_matrix(1:3,4)');

fprintf('\n=== Individual Link Transformations ===\n');
% Calculate individual transformations
T0_1 = robot.links(1).A(q(1));
T1_2 = robot.links(2).A(q(2));

fprintf('\nT₀¹ (Base to Knee):\n');
if isobject(T0_1)
    try
        T0_1_matrix = T0_1.T;
    catch
        T0_1_matrix = double(T0_1);
    end
else
    T0_1_matrix = T0_1;
end
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T0_1_matrix(1,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T0_1_matrix(2,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T0_1_matrix(3,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T0_1_matrix(4,:));

fprintf('\nT₁² (Knee to Foot):\n');
if isobject(T1_2)
    try
        T1_2_matrix = T1_2.T;
    catch
        T1_2_matrix = double(T1_2);
    end
else
    T1_2_matrix = T1_2;
end
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T1_2_matrix(1,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T1_2_matrix(2,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T1_2_matrix(3,:));
fprintf('  [%8.4f  %8.4f  %8.4f  %8.4f]\n', T1_2_matrix(4,:));

fprintf('\nVerification: T₀² = T₀¹ × T₁²\n');

%% Visualization
figure('Name', 'Leg Forward Kinematics', 'Position', [100 100 1000 800]);
hold on; grid on; axis equal;
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('2-DOF Leg: Forward Kinematics');

% Hip position (origin)
hip_pos = [0, 0, 0];

% Calculate knee position using first link
knee_x = thigh_length * cos(theta1);
knee_y = thigh_length * sin(theta1);
knee_pos = [knee_x, knee_y, 0];

% Calculate foot position using both links
foot_x = knee_x + shank_length * cos(theta1 + theta2);
foot_y = knee_y + shank_length * sin(theta1 + theta2);
foot_pos_manual = [foot_x, foot_y, 0];

% Draw hip joint
plot3(hip_pos(1), hip_pos(2), hip_pos(3), 'ro', ...
      'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
text(hip_pos(1), hip_pos(2), hip_pos(3)+5, 'Hip', 'FontSize', 12, 'FontWeight', 'bold');

% Draw thigh
plot3([hip_pos(1), knee_pos(1)], [hip_pos(2), knee_pos(2)], [hip_pos(3), knee_pos(3)], ...
      'b-', 'LineWidth', 6);
text(knee_x/2, knee_y/2, 5, sprintf('Thigh (%.1fmm)', thigh_length), 'FontSize', 10);

% Draw knee joint
plot3(knee_pos(1), knee_pos(2), knee_pos(3), 'ko', ...
      'MarkerSize', 12, 'MarkerFaceColor', 'k', 'LineWidth', 2);
text(knee_pos(1), knee_pos(2), knee_pos(3)+5, 'Knee', 'FontSize', 12, 'FontWeight', 'bold');

% Draw shank
plot3([knee_pos(1), foot_pos_manual(1)], [knee_pos(2), foot_pos_manual(2)], ...
      [knee_pos(3), foot_pos_manual(3)], 'g-', 'LineWidth', 6);
text((knee_x+foot_x)/2, (knee_y+foot_y)/2, 5, sprintf('Shank (%.1fmm)', shank_length), 'FontSize', 10);

% Draw foot
plot3(foot_pos_manual(1), foot_pos_manual(2), foot_pos_manual(3), 'mo', ...
      'MarkerSize', 12, 'MarkerFaceColor', 'm', 'LineWidth', 2);
text(foot_pos_manual(1), foot_pos_manual(2), foot_pos_manual(3)+5, 'Foot', 'FontSize', 12, 'FontWeight', 'bold');

% Draw ground reference
plot3([-20, foot_x+20], [0, 0], [0, 0], 'k--', 'LineWidth', 1);

% Add angle annotations
% Hip angle arc
theta_arc = linspace(0, theta1, 20);
arc_radius = 15;
arc_x = arc_radius * cos(theta_arc);
arc_y = arc_radius * sin(theta_arc);
plot3(arc_x, arc_y, zeros(size(arc_x)), 'r-', 'LineWidth', 2);
text(arc_radius*1.5*cos(theta1/2), arc_radius*1.5*sin(theta1/2), 0, ...
     sprintf('θ₁=%.0f°', rad2deg(theta1)), 'FontSize', 10, 'Color', 'r');

% Knee angle arc
theta_arc2 = linspace(theta1, theta1+theta2, 20);
arc_x2 = knee_x + arc_radius * cos(theta_arc2);
arc_y2 = knee_y + arc_radius * sin(theta_arc2);
plot3(arc_x2, arc_y2, zeros(size(arc_x2)), 'g-', 'LineWidth', 2);
text(knee_x + arc_radius*1.5*cos(theta1+theta2/2), ...
     knee_y + arc_radius*1.5*sin(theta1+theta2/2), 0, ...
     sprintf('θ₂=%.0f°', rad2deg(theta2)), 'FontSize', 10, 'Color', 'g');

view(2);  % Side view (2D)
xlim([-50 200]);
ylim([-200 100]);

%% Show robot using built-in plot function
figure('Name', 'Robot Toolbox Visualization');
robot.plot(q, 'workspace', [-100 200 -200 100 -50 50]);
title('Peter Corke Toolbox Visualization');

fprintf('\n=== Forward Kinematics Complete ===\n');