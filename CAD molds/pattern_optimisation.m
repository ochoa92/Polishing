% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description :         
% =========================================================================

clear all;
clc;
close all;


%% Get pattern from a file
A = importdata('/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/pattern');
% t p_x p_y p_z Qx Qy Qz Qw Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O

% time
t = A.data(:,1)';

% pattern position in Base frame
pattern = [A.data(:,2) A.data(:,3) A.data(:,4)]';  % p = [px py pz]';


% Zero-phase digital filtering
d = designfilt('lowpassfir', ...
               'CutoffFrequency', 10, ...
               'FilterOrder', 100, ...
               'SampleRate', 1000);

pattern_filtered = [filtfilt(d,pattern(1,:)); filtfilt(d,pattern(2,:)); filtfilt(d,pattern(3,:))];


%% plot
figure(1)
hold on
plot(t,pattern(1,:),'b','linewidth', 2)
plot(t,pattern(2,:),'g','linewidth', 2)
plot(t,pattern(3,:),'r','linewidth', 2)

plot(t,pattern_filtered(1,:),'--b','linewidth', 2)
plot(t,pattern_filtered(2,:),'--g','linewidth', 2)
plot(t,pattern_filtered(3,:),'--r','linewidth', 2)

grid on
xlabel('t')
ylabel('p')