  clc;
clear;
close all;

% -----------------------------
% Base dimensions (mm)
base_length = 85;
base_width  = 110;
base_height = 6; % thickness

% -----------------------------
% Updated Thigh + Shank Parameters
thigh_length = 71.83;        % mm
thigh_angle_deg = 30;        % NEW THIGH ANGLE
thigh_angle = deg2rad(thigh_angle_deg);

shank_length = 64.96;        % mm
shank_angle_deg = 63;        % NEW SHANK ANGLE
shank_angle = deg2rad(shank_angle_deg);

% -----------------------------
figure;
hold on;
grid on;
axis equal;
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Robotic Dog Model');

% Sphere for joints
[sx, sy, sz] = sphere(20);
r = 5; % joint sphere radius


% ===============================================================
% BASE 1
% ===============================================================
[X1, Y1] = meshgrid([0 base_length], [0 base_width]);
Z1 = zeros(2);

surf(X1, Y1, Z1, 'FaceColor', [0.8 0.8 0.8]);
surf(X1, Y1, Z1+base_height, 'FaceColor', [0.9 0.9 0.9]);

plot3([0 base_length base_length 0 0], [0 0 base_width base_width 0], ...
      [0 0 0 0 0], 'k', 'LineWidth', 2);

% Base 1 joints (FRONT)
joint1_base1 = [0, 0, base_height];
joint2_base1 = [85, 0, base_height];


% ===============================================================
% BASE 2
% ===============================================================
base2_offset = [0, base_width + 20, 0];

[X2, Y2] = meshgrid([0 base_length], [0 base_width]);
Z2 = zeros(2);

surf(X2 + base2_offset(1), Y2 + base2_offset(2), Z2, 'FaceColor', [0.7 0.7 0.9]);
surf(X2 + base2_offset(1), Y2 + base2_offset(2), Z2+base_height, 'FaceColor', [0.6 0.6 0.8]);

plot3([0 base_length base_length 0 0] + base2_offset(1), ...
      [0 0 base_width base_width 0] + base2_offset(2), ...
      [0 0 0 0 0], 'k', 'LineWidth', 2);

% Base 2 joints (BACK)
joint1_base2 = [0, base_width, base_height] + base2_offset;
joint2_base2 = [85, base_width, base_height] + base2_offset;


% ===============================================================
% Middle Revolute Joint (between Base1 & Base2)
% ===============================================================
joint_between = [base_length/2, base_width + 10, base_height];

all_base_joints = [joint1_base1; joint2_base1; ...
                   joint1_base2; joint2_base2; ...
                   joint_between];

for i = 1:size(all_base_joints,1)
    surf(sx*r + all_base_joints(i,1), ...
         sy*r + all_base_joints(i,2), ...
         sz*r + all_base_joints(i,3), 'FaceColor', 'r');
end


% ===============================================================
% THIGHS (updated angles)
% ===============================================================
thigh_vec_front = [0,  thigh_length*cos(thigh_angle), -thigh_length*sin(thigh_angle)];
thigh_vec_back  = [0, -thigh_length*cos(thigh_angle), -thigh_length*sin(thigh_angle)];

knee1_base1 = joint1_base1 + thigh_vec_front;
knee2_base1 = joint2_base1 + thigh_vec_front;
knee1_base2 = joint1_base2 + thigh_vec_back;
knee2_base2 = joint2_base2 + thigh_vec_back;

knee_joints = [knee1_base1; knee2_base1; knee1_base2; knee2_base2];

% Draw thighs
plot3([joint1_base1(1) knee1_base1(1)], [joint1_base1(2) knee1_base1(2)], [joint1_base1(3) knee1_base1(3)], 'b', 'LineWidth', 4);
plot3([joint2_base1(1) knee2_base1(1)], [joint2_base1(2) knee2_base1(2)], [joint2_base1(3) knee2_base1(3)], 'b', 'LineWidth', 4);
plot3([joint1_base2(1) knee1_base2(1)], [joint1_base2(2) knee1_base2(2)], [joint1_base2(3) knee1_base2(3)], 'b', 'LineWidth', 4);
plot3([joint2_base2(1) knee2_base2(1)], [joint2_base2(2) knee2_base2(2)], [joint2_base2(3) knee2_base2(3)], 'b', 'LineWidth', 4);

% Draw knee joints
for i = 1:size(knee_joints,1)
    surf(sx*r + knee_joints(i,1), ...
         sy*r + knee_joints(i,2), ...
         sz*r + knee_joints(i,3), 'FaceColor', 'r');
end


% ===============================================================
% SHANKS (updated 63°)
% ===============================================================
shank_vec_front = [0,  shank_length*cos(thigh_angle + shank_angle), ...
                      -shank_length*sin(thigh_angle + shank_angle)];

shank_vec_back = [0, -shank_length*cos(thigh_angle + shank_angle), ...
                     -shank_length*sin(thigh_angle + shank_angle)];

% Draw shanks (green)
plot3([knee1_base1(1) knee1_base1(1)+shank_vec_front(1)], ...
      [knee1_base1(2) knee1_base1(2)+shank_vec_front(2)], ...
      [knee1_base1(3) knee1_base1(3)+shank_vec_front(3)], 'g', 'LineWidth', 4);

plot3([knee2_base1(1) knee2_base1(1)+shank_vec_front(1)], ...
      [knee2_base1(2) knee2_base1(2)+shank_vec_front(2)], ...
      [knee2_base1(3) knee2_base1(3)+shank_vec_front(3)], 'g', 'LineWidth', 4);

plot3([knee1_base2(1) knee1_base2(1)+shank_vec_back(1)], ...
      [knee1_base2(2) knee1_base2(2)+shank_vec_back(2)], ...
      [knee1_base2(3) knee1_base2(3)+shank_vec_back(3)], 'g', 'LineWidth', 4);

plot3([knee2_base2(1) knee2_base2(1)+shank_vec_back(1)], ...
      [knee2_base2(2) knee2_base2(2)+shank_vec_back(2)], ...
      [knee2_base2(3) knee2_base2(3)+shank_vec_back(3)], 'g', 'LineWidth', 4);


% ===============================================================
% UPDATED NECK REVOLUTE JOINT (absolute position)
% ===============================================================
neck_joint = [29.3, 227.5, 32];  % NEW EXACT LOCATION

surf(sx*r + neck_joint(1), ...
     sy*r + neck_joint(2), ...
     sz*r + neck_joint(3), ...
     'FaceColor', 'r');

plot3([neck_joint(1) neck_joint(1)], ...
      [neck_joint(2) neck_joint(2)], ...
      [0 neck_joint(3)], 'k--', 'LineWidth', 2);

% ===============================================================
% ADD 38 mm LINK ALONG +X AXIS FROM NECK JOINT
% ===============================================================

link_length = 38;         % length of neck link
link_vec = [link_length, 0, 0];   % ALONG +X AXIS

link_end = neck_joint + link_vec; % end-point of link

% Draw link
plot3([neck_joint(1) link_end(1)], ...
      [neck_joint(2) link_end(2)], ...
      [neck_joint(3) link_end(3)], ...
      'm', 'LineWidth', 5);      % magenta link

% ===============================================================
% NEW REVOLUTE JOINT AT SPECIFIED POSITION
% ===============================================================
new_joint = [54.5, neck_joint(2), neck_joint(3)+19]; % [x, y, z]

% Draw the new revolute joint
surf(sx*r + new_joint(1), ...
     sy*r + new_joint(2), ...
     sz*r + new_joint(3), 'FaceColor', 'r');

% Draw the link directly connecting neck joint to new joint
plot3([neck_joint(1) new_joint(1)], ...
      [neck_joint(2) new_joint(2)], ...
      [neck_joint(3) new_joint(3)], '', 'LineWidth', 5); % magenta link



view(3);
