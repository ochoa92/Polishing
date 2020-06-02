% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : 
% =========================================================================
clear all
clc
close all

P = importdata('robot_results/polishing_controller');
% t p_x p_xd p_y p_yd p_z p_zd Yaw Yaw_d Pitch Pitch_d Roll Roll_d e_px e_py e_pz e_ox e_oy e_oz FxEE_franka FyEE_franka FzEE_franka Fx Fy Fz

t = P.data(:,1);

px = P.data(:,2);
pxd = P.data(:,3);
py = P.data(:,4);
pyd = P.data(:,5);
pz = P.data(:,6);
pzd = P.data(:,7);
p = [px py pz];
pd = [pxd pyd pzd];

ox = P.data(:,8);
oxd = P.data(:,9);
oy = P.data(:,10);
oyd = P.data(:,11);
oz = P.data(:,12);
ozd = P.data(:,13);
o = [ox oy oz];
od = [oxd oyd ozd];

e_px = rms(P.data(:,14));
e_py = rms(P.data(:,15));
e_pz = rms(P.data(:,16));
e_ox = rms(P.data(:,17));
e_oy = rms(P.data(:,18));
e_oz = rms(P.data(:,19));

Fx = P.data(:,20);
Fy = P.data(:,21);
Fz = P.data(:,22);

% convert position and orientation to EE frame
% R = eul2rotm(o, 'XYZ');
% for i=1:length(R)
%    pEE = R(:,:,i)' *  p';
%    oEE = R(:,:,i)' *  o';
% end
% 
% Rd = eul2rotm(od, 'XYZ');
% for i=1:length(Rd)
%    pdEE = Rd(:,:,i)' *  pd';
%    odEE = Rd(:,:,i)' *  od';
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% position plot base frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
hold on
plot(t, px, '--k', 'linewidth', 2)
plot(t, pxd, 'r', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('p_x (m)', 'FontSize', 40)
xlim([t(1) t(end)])
ylim([0.3 0.7])
legend('robot', 'desired', 'FontSize', 40)
set(gca,'FontSize',40)
box on

figure(2)
hold on
plot(t, py, '--k', 'linewidth', 2)
plot(t, pyd, 'g', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('p_y (m)', 'FontSize', 40)
xlim([t(1) t(end)])
ylim([-0.1 0.4])
legend('robot', 'desired', 'FontSize', 40)
set(gca,'FontSize',40)
box on

figure(3)
hold on
plot(t, pz, '--k', 'linewidth', 2)
plot(t, pzd, 'b', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('p_z (m)', 'FontSize', 40)
xlim([t(1) t(end)])
ylim([0 0.6])
legend('robot', 'desired', 'FontSize', 40)
set(gca,'FontSize',40)
box on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% position plot EE frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1)
% hold on
% plot(t, pEE(1,:), '--k', 'linewidth', 2)
% plot(t, pdEE(1,:), 'r', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_x (m)', 'FontSize', 40)
% xlim([t(1) t(end)])
% legend('robot', 'desired', 'FontSize', 40)
% set(gca,'FontSize',40)
% box on
% 
% figure(2)
% hold on
% plot(t, pEE(2,:), '--k', 'linewidth', 2)
% plot(t, pdEE(2,:), 'g', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_y (m)', 'FontSize', 40)
% xlim([t(1) t(end)])
% legend('robot', 'desired', 'FontSize', 40)
% set(gca,'FontSize',40)
% box on
% 
% figure(3)
% hold on
% plot(t, pEE(3,:), '--k', 'linewidth', 2)
% plot(t, pdEE(3,:), 'b', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_z (m)', 'FontSize', 40)
% xlim([t(1) t(end)])
% legend('robot', 'desired', 'FontSize', 40)
% set(gca,'FontSize',40)
% box on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% orientation plot base frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1)
% hold on
% plot(t, ox, '--r', 'linewidth', 2)
% plot(t, oxd, 'k', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('Yaw (rad)', 'FontSize', 40)
% xlim([t(1) t(end)])
% ylim([-0.1 1.7])
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
% ylim([3.05 3.14])
% legend('robot', 'desired', 'FontSize', 40)
% set(gca,'FontSize',40)
% box on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% orientation plot EE frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1)
% hold on
% plot(t, oEE(1,:), '--k', 'linewidth', 2)
% plot(t, odEE(1,:), 'r', 'linewidth', 2)
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
% plot(t, oEE(2,:), '--k', 'linewidth', 2)
% plot(t, odEE(2,:), 'g', 'linewidth', 2)
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
% plot(t, oEE(3,:), '--k', 'linewidth', 2)
% plot(t, odEE(3,:), 'b', 'linewidth', 2)
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
% ylim([-6 3])
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
% ylim([-5 20])
% xlim([t(1) t(end)])
% set(gca,'FontSize',40)
% % xticks(0:5:t(end))
% box on
