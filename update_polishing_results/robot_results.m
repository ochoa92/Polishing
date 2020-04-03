% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : 
% =========================================================================
clear all
clc
close all

P = importdata('robot_results/polishing_controller');
% t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) e_px e_py e_pz e_ox e_oy e_oz i_px i_py i_pz i_ox i_oy i_oz Fx Fy Fz Fx_filtered Fy_filtered Fz_filtered

t = P.data(:,1);

px = P.data(:,2);
pxd = P.data(:,3);
py = P.data(:,4);
pyd = P.data(:,5);
pz = P.data(:,6);
pzd = P.data(:,7);

ox = P.data(:,8);
oxd = P.data(:,9);
oy = P.data(:,10);
oyd = P.data(:,11);
oz = P.data(:,12);
ozd = P.data(:,13);

e_px = rms(P.data(:,14));
e_py = rms(P.data(:,15));
e_pz = rms(P.data(:,16));
e_ox = rms(P.data(:,17));
e_oy = rms(P.data(:,18));
e_oz = rms(P.data(:,19));

Fx = P.data(:,29);
Fy = P.data(:,30);
Fz = P.data(:,31);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% position plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
hold on
plot(t, px, '--r', 'linewidth', 2)
plot(t, pxd, 'k', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('p_x (m)', 'FontSize', 40)
xlim([t(1) t(end)])
legend('robot', 'desired', 'FontSize', 40)
set(gca,'FontSize',40)
box on

figure(2)
hold on
plot(t, py, '--g', 'linewidth', 2)
plot(t, pyd, 'k', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('p_y (m)', 'FontSize', 40)
xlim([t(1) t(end)])
legend('robot', 'desired', 'FontSize', 40)
set(gca,'FontSize',40)
box on

figure(3)
hold on
plot(t, pz, '--b', 'linewidth', 2)
plot(t, pzd, 'k', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('p_z (m)', 'FontSize', 40)
xlim([t(1) t(end)])
legend('robot', 'desired', 'FontSize', 40)
set(gca,'FontSize',40)
box on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% orientation plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1)
% hold on
% plot(t, ox, '--r', 'linewidth', 2)
% plot(t, oxd, 'k', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('Yaw (rad)', 'FontSize', 40)
% xlim([t(1) t(end)])
% 
% legend('robot', 'desired', 'FontSize', 40)
% set(gca,'FontSize',40)
% box on
% 
% figure(2)
% hold on
% plot(t, oy, '--g', 'linewidth', 2)
% plot(t, oyd, 'k', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('Pitch (rad)', 'FontSize', 40)
% xlim([t(1) t(end)])
% 
% legend('robot', 'desired', 'FontSize', 40)
% set(gca,'FontSize',40)
% box on
% 
% figure(3)
% hold on
% plot(t, oz, '--b', 'linewidth', 2)
% plot(t, ozd, 'k', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('Roll (rad)', 'FontSize', 40)
% xlim([t(1) t(end)])
% 
% legend('robot', 'desired', 'FontSize', 40)
% set(gca,'FontSize',40)
% box on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% force plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1)
% hold on
% plot(t, Fx, 'r', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('F_x (N)', 'FontSize', 40)
% xlim([t(1) t(end)])
% % ylim([-7 4])
% set(gca,'FontSize',40)
% % xticks(0:5:t(end))
% box on
% 
% figure(2)
% hold on
% plot(t, Fy, 'g', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('F_y (N)', 'FontSize', 40)
% xlim([t(1) t(end)])
% % ylim([-7 4])
% set(gca,'FontSize',40)
% % xticks(0:5:t(end))
% box on
% 
% figure(3)
% hold on
% plot(t, Fz, 'b', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('F_z (N)', 'FontSize', 40)
% % ylim([0 18])
% xlim([t(1) t(end)])
% set(gca,'FontSize',40)
% % xticks(0:5:t(end))
% box on
