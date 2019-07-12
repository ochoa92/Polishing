% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : results to IEEE conference
% =========================================================================

clear all
clc
close all


%% Co-manipulation pattern
P = importdata('patterns/polishing_pattern');
% t p_x p_y p_z Qx Qy Qz Qw Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O

t_pattern = P.data(:,1);
px_pattern = P.data(:,2);
py_pattern = P.data(:,3);
pz_pattern = P.data(:,4);

Fx_pattern = P.data(:,9);
Fy_pattern = P.data(:,10);
Fz_pattern = P.data(:,11);

%% Robot Performance
R = importdata('cartesian_impedance_controller');
% t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O e_px e_py e_pz e_ox e_oy e_oz pEE_x pEE_xd pEE_y pEE_yd pEE_z pEE_zd i_px i_py i_pz i_ox i_oy i_oz

t_robot = R.data(:,1);
px_robot = R.data(:,2);
pxd_robot = R.data(:,3);
py_robot = R.data(:,4);
pyd_robot = R.data(:,5);
pz_robot = R.data(:,6);
pzd_robot = R.data(:,7);

ox_robot = R.data(:,8);
oxd_robot = R.data(:,9);
oy_robot = R.data(:,10);
oyd_robot = R.data(:,11);
oz_robot = R.data(:,12);
ozd_robot = R.data(:,13);

Fx_robot = R.data(:,14);
Fy_robot = R.data(:,15);
Fz_robot = R.data(:,16);

error_px = R.data(:,20);
error_py = R.data(:,21);
error_pz = R.data(:,22);

error_ox = R.data(:,23);
error_oy = R.data(:,24);
error_oz = R.data(:,25);

%% plots
% Co-Manipulation ---------------------------------------------------------
% position
figure(1)
subplot(3,1,1)
plot(t_pattern, px_pattern, 'b', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Position_x (m)', 'FontSize', 15)
subplot(3,1,2)
plot(t_pattern, py_pattern, 'g', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Position_y (m)', 'FontSize', 15)
subplot(3,1,3)
plot(t_pattern, pz_pattern, 'r', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Position_z (m)', 'FontSize', 15)
sgtitle('End-Effector Position in Base frame (Co-Manipulation)')

% Force
figure(2)
subplot(3,1,1)
plot(t_pattern, Fx_pattern, 'b', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Force_x (N)', 'FontSize', 15)
subplot(3,1,2)
plot(t_pattern, Fy_pattern, 'g', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Force_y (N)', 'FontSize', 15)
subplot(3,1,3)
plot(t_pattern, Fz_pattern, 'r', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Force_z (N)', 'FontSize', 15)
sgtitle('End-Effector Force in End-Effector frame (Co-Manipulation)')

% Robot -------------------------------------------------------------------
% position
figure(3)
plot(t_robot, px_robot, '--k', 'linewidth', 2)
hold on
plot(t_robot, pxd_robot, 'b', 'linewidth', 1)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Position_x (m)', 'FontSize', 15)
legend('robot', 'desired')
xlim([t_robot(1) t_robot(100000)])
title('End-Effector Position in Base frame (Robot)')


figure(4)
plot(t_robot, py_robot, '--k', 'linewidth', 2)
hold on
plot(t_robot, pyd_robot, 'g', 'linewidth', 1)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Position_y (m)', 'FontSize', 15)
legend('robot', 'desired')
xlim([t_robot(1) t_robot(100000)])
title('End-Effector Position in Base frame (Robot)')

figure(5)
plot(t_robot, pz_robot, '--k', 'linewidth', 2)
hold on
plot(t_robot, pzd_robot, 'r', 'linewidth', 1)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Position_z (m)', 'FontSize', 15)
legend('robot', 'desired')
xlim([t_robot(1) t_robot(100000)])
title('End-Effector Position in Base frame (Robot)')

% orientation
figure(6)
plot(t_robot, ox_robot, '--k', 'linewidth', 2)
hold on
plot(t_robot, oxd_robot, 'b', 'linewidth', 1)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Yaw (rad)', 'FontSize', 15)
legend('robot', 'desired')
xlim([t_robot(1) t_robot(100000)])
title('End-Effector Orientation in Base frame (Robot)')

figure(7)
plot(t_robot, oy_robot, '--k', 'linewidth', 2)
hold on
plot(t_robot, oyd_robot, 'g', 'linewidth', 1)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Pitch (rad)', 'FontSize', 15)
legend('robot', 'desired')
xlim([t_robot(1) t_robot(100000)])
title('End-Effector Orientation in Base frame (Robot)')

figure(8)
plot(t_robot, oz_robot, '--k', 'linewidth', 2)
hold on
plot(t_robot, ozd_robot, 'r', 'linewidth', 1)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Roll (rad)', 'FontSize', 15)
legend('robot', 'desired')
xlim([t_robot(1) t_robot(100000)])
title('End-Effector Orientation in Base frame (Robot)')

% Force
figure(9)
subplot(3,1,1)
plot(t_robot, Fx_robot, 'b', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Force_x (N)', 'FontSize', 15)
xlim([t_robot(1) t_robot(100000)])
subplot(3,1,2)
plot(t_robot, Fy_robot, 'g', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Force_y (N)', 'FontSize', 15)
xlim([t_robot(1) t_robot(100000)])
subplot(3,1,3)
plot(t_robot, Fz_robot, 'r', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 15)
ylabel('Force_z (N)', 'FontSize', 15)
xlim([t_robot(1) t_robot(100000)])
sgtitle('End-Effector Force in End-Effector frame (Robot)')

% error
% figure(10)
% subplot(3,1,1)
% plot(t_robot, error_px, 'b', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 15)
% ylabel('Error_x (m)', 'FontSize', 15)
% xlim([t_robot(1) t_robot(100000)])
% subplot(3,1,2)
% plot(t_robot, error_py, 'g', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 15)
% ylabel('Error_y (m)', 'FontSize', 15)
% xlim([t_robot(1) t_robot(100000)])
% subplot(3,1,3)
% plot(t_robot, error_pz, 'r', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 15)
% ylabel('Error_z (m)', 'FontSize', 15)
% xlim([t_robot(1) t_robot(100000)])
% sgtitle('End-Effector Position Error in Base frame (Robot)')
% 
% figure(11)
% subplot(3,1,1)
% plot(t_robot, error_ox, 'b', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 15)
% ylabel('Error_{Yaw} (rad)', 'FontSize', 15)
% xlim([t_robot(1) t_robot(100000)])
% subplot(3,1,2)
% plot(t_robot, error_oy, 'g', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 15)
% ylabel('Error_{Pitch} (rad)', 'FontSize', 15)
% xlim([t_robot(1) t_robot(100000)])
% subplot(3,1,3)
% plot(t_robot, error_oz, 'r', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 15)
% ylabel('Error_{Roll} (rad)', 'FontSize', 15)
% xlim([t_robot(1) t_robot(100000)])
% sgtitle('End-Effector Orientation Error in Base frame (Robot)')
