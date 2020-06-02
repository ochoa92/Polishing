% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : 
% =========================================================================
clear all
clc
close all

%% position
P = importdata('co_manipulation_results/position_pattern');
% t p_x p_y p_z Qx Qy Qz Qw

t_pattern = P.data(:,1);
px_pattern = P.data(:,2);
py_pattern = P.data(:,3);
pz_pattern = P.data(:,4);
p = [px_pattern py_pattern pz_pattern];

Q = [P.data(:,8) P.data(:,5) P.data(:,6) P.data(:,7)];
R = quat2rotm(Q);
for i=1:length(R)
   pEE = R(:,:,i)' *  p'; 
end

%% plot base-frame
% figure(1)
% hold on
% plot(t_pattern, px_pattern, 'r', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_x (m)', 'FontSize', 40)
% xlim([t_pattern(1) t_pattern(end)])
% set(gca,'FontSize',40)
% box on
% 
% figure(2)
% hold on
% plot(t_pattern, py_pattern, 'g', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_y (m)', 'FontSize', 40)
% xlim([t_pattern(1) t_pattern(end)])
% set(gca,'FontSize',40)
% box on
% 
% figure(3)
% hold on
% plot(t_pattern, pz_pattern, 'b', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_z (m)', 'FontSize', 40)
% xlim([t_pattern(1) t_pattern(end)])
% set(gca,'FontSize',40)
% box on

%%
% figure(1)
% hold on
% plot(t_pattern, pEE(1,:), 'r', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_x (m)', 'FontSize', 40)
% xlim([t_pattern(1) t_pattern(end)])
% set(gca,'FontSize',40)
% box on
% 
% figure(2)
% hold on
% plot(t_pattern, pEE(2,:), 'g', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_y (m)', 'FontSize', 40)
% xlim([t_pattern(1) t_pattern(end)])
% set(gca,'FontSize',40)
% box on
% 
% figure(3)
% hold on
% plot(t_pattern, pEE(3,:), 'b', 'linewidth', 2)
% grid on
% xlabel('t (s)', 'FontSize', 40)
% ylabel('p_z (m)', 'FontSize', 40)
% xlim([t_pattern(1) t_pattern(end)])
% set(gca,'FontSize',40)
% box on

%% Force
F = importdata('co_manipulation_results/force_pattern');
% t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O e_px e_py e_pz e_ox e_oy e_oz pEE_x pEE_xd pEE_y pEE_yd pEE_z pEE_zd i_px i_py i_pz i_ox i_oy i_oz
tF_pattern = F.data(:,1);
Fx_pattern = F.data(:,14);
Fy_pattern = F.data(:,15);
Fz_pattern = F.data(:,16);

% plot
figure(1)
hold on
plot(tF_pattern, Fx_pattern, 'r', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('F_x (N)', 'FontSize', 40)
xlim([tF_pattern(1) tF_pattern(end)])
ylim([-7 4])
set(gca,'FontSize',40)
xticks(0:5:tF_pattern(end))
box on

figure(2)
hold on
plot(tF_pattern, Fy_pattern, 'g', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('F_y (N)', 'FontSize', 40)
xlim([tF_pattern(1) tF_pattern(end)])
ylim([-7 4])
set(gca,'FontSize',40)
xticks(0:5:tF_pattern(end))
box on

figure(3)
hold on
plot(tF_pattern, Fz_pattern, 'b', 'linewidth', 2)
grid on
xlabel('t (s)', 'FontSize', 40)
ylabel('F_z (N)', 'FontSize', 40)
ylim([0 18])
xlim([tF_pattern(1) tF_pattern(end)])
set(gca,'FontSize',40)
xticks(0:5:tF_pattern(end))
box on
