% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : results to IEEE Journal
% =========================================================================

clear all
clc
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Co-manipulation pattern
P = importdata('mold_data/pattern');
% t p_x p_y p_z Qx Qy Qz Qw Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O

t_pattern = P.data(:,1);
px_pattern = P.data(:,2);
py_pattern = P.data(:,3);
pz_pattern = P.data(:,4);

F = importdata('EE_Force_pattern/cartesian_impedance_controller_1');
% t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O e_px e_py e_pz e_ox e_oy e_oz pEE_x pEE_xd pEE_y pEE_yd pEE_z pEE_zd i_px i_py i_pz i_ox i_oy i_oz
tF_pattern = F.data(:,1);
Fx_pattern = F.data(:,14);
Fy_pattern = F.data(:,15);
Fz_pattern = F.data(:,16);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot Performance
% R = importdata('results_curved_surface');
% % t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O e_px e_py e_pz e_ox e_oy e_oz pEE_x pEE_xd pEE_y pEE_yd pEE_z pEE_zd i_px i_py i_pz i_ox i_oy i_oz
% 
% t_robot = R.data(:,1);
% px_robot = R.data(:,2);
% pxd_robot = R.data(:,3);
% py_robot = R.data(:,4);
% pyd_robot = R.data(:,5);
% pz_robot = R.data(:,6);
% pzd_robot = R.data(:,7);
% 
% ox_robot = wrapTo2Pi(R.data(:,8));
% oxd_robot = wrapTo2Pi(R.data(:,9));
% oy_robot = wrapToPi(R.data(:,10));
% oyd_robot = wrapToPi(R.data(:,11));
% oz_robot = wrapToPi(R.data(:,12));
% ozd_robot = wrapToPi(R.data(:,13));
% 
% Fx_robot = R.data(:,14);
% Fy_robot = R.data(:,15);
% Fz_robot = R.data(:,16);
% 
% error_px = rms(R.data(:,20));
% error_py = rms(R.data(:,21));
% error_pz = rms(R.data(:,22));
% 
% error_ox = rms(R.data(:,23));
% error_oy = rms(R.data(:,24));
% error_oz = rms(R.data(:,25));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plots

% Co-Manipulation ---------------------------------------------------------
% position
figure(1)
plot(t_pattern, px_pattern, 'b', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 20)
ylabel('p_x (m)', 'FontSize', 20)
xlim([t_pattern(1) t_pattern(17229)])
figure(2)
plot(t_pattern, py_pattern, 'g', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 20)
ylabel('p_y (m)', 'FontSize', 20)
xlim([t_pattern(1) t_pattern(17229)])
figure(3)
plot(t_pattern, pz_pattern, 'r', 'linewidth', 2)
grid on
xlabel('time (s)', 'FontSize', 20)
ylabel('p_z (m)', 'FontSize', 20)
xlim([t_pattern(1) t_pattern(17229)])
% sgtitle('End-Effector Position in Base frame (Co-Manipulation)')

% % Force
% figure(1)
% plot(tF_pattern, Fx_pattern, 'b', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('F_x (N)', 'FontSize', 20)
% xlim([tF_pattern(1) tF_pattern(40000)])
% figure(2)
% plot(tF_pattern, Fy_pattern, 'g', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('F_y (N)', 'FontSize', 20)
% xlim([tF_pattern(1) tF_pattern(40000)])
% figure(3)
% plot(tF_pattern, Fz_pattern, 'r', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('F_z (N)', 'FontSize', 20)
% xlim([tF_pattern(1) tF_pattern(40000)])
% % sgtitle('End-Effector Force in End-Effector frame (Co-Manipulation)')


% Robot -------------------------------------------------------------------
% % position
% figure(3)
% plot(t_robot, px_robot, '--k', 'linewidth', 2)
% hold on
% plot(t_robot, pxd_robot, 'b', 'linewidth', 1)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('p_x (m)', 'FontSize', 20)
% legend('robot', 'desired', 'FontSize', 20)
% xlim([t_robot(1) t_robot(80000)])
% % title('End-Effector Position in Base frame (Robot)')
% 
% figure(4)
% plot(t_robot, py_robot, '--k', 'linewidth', 2)
% hold on
% plot(t_robot, pyd_robot, 'g', 'linewidth', 1)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('p_y (m)', 'FontSize', 20)
% legend('robot', 'desired', 'FontSize', 20)
% xlim([t_robot(1) t_robot(80000)])
% % title('End-Effector Position in Base frame (Robot)')
% 
% figure(5)
% plot(t_robot, pz_robot, '--k', 'linewidth', 2)
% hold on
% plot(t_robot, pzd_robot, 'r', 'linewidth', 1)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('p_z (m)', 'FontSize', 20)
% legend('robot', 'desired', 'FontSize', 20)
% ylim([0.05 0.55])
% xlim([t_robot(1) t_robot(80000)])
% % title('End-Effector Position in Base frame (Robot)')

% % orientation
% figure(6)
% plot(t_robot, ox_robot, '--k', 'linewidth', 2)
% hold on
% plot(t_robot, oxd_robot, 'b', 'linewidth', 1)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('Yaw (rad)', 'FontSize', 20)
% legend('robot', 'desired', 'FontSize', 20)
% % ylim([0 2*pi])
% xlim([t_robot(1) t_robot(80000)])
% % title('End-Effector Orientation in Base frame (Robot)')
% 
% figure(7)
% plot(t_robot, oy_robot, '--k', 'linewidth', 2)
% hold on
% plot(t_robot, oyd_robot, 'g', 'linewidth', 1)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('Pitch (rad)', 'FontSize', 20)
% legend('robot', 'desired', 'FontSize', 20)
% % ylim([-pi pi])
% xlim([t_robot(1) t_robot(80000)])
% % title('End-Effector Orientation in Base frame (Robot)')
% 
% figure(8)
% plot(t_robot, oz_robot, '--k', 'linewidth', 2)
% hold on
% plot(t_robot, ozd_robot, 'r', 'linewidth', 1)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('Roll (rad)', 'FontSize', 20)
% legend('robot', 'desired', 'FontSize', 20)
% % ylim([-pi pi])
% xlim([t_robot(1) t_robot(80000)])
% % title('End-Effector Orientation in Base frame (Robot)')

% % Force
% figure(9)
% hold on
% plot(t_robot, Fx_robot, 'b', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('F_x (N)', 'FontSize', 20)
% xlim([t_robot(1) t_robot(80000)])
% % sgtitle('End-Effector Force in End-Effector frame (Robot)')
% 
% figure(10)
% hold on
% plot(t_robot, Fy_robot, 'g', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('F_y (N)', 'FontSize', 20)
% xlim([t_robot(1) t_robot(80000)])
% % sgtitle('End-Effector Force in End-Effector frame (Robot)')
% 
% figure(11)
% hold on
% plot(t_robot, Fz_robot, 'r', 'linewidth', 2)
% grid on
% xlabel('time (s)', 'FontSize', 20)
% ylabel('F_z (N)', 'FontSize', 20)
% xlim([t_robot(1) t_robot(80000)])
% % sgtitle('End-Effector Force in End-Effector frame (Robot)')

