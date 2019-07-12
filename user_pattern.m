% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : 
% =========================================================================
clear all;
clc;
close all;

%% pattern
filename = '/home/helio/kst/polishing/patterns/polishing_pattern';
A = importdata(filename);

% position in Base frame
px = A.data(:,2);
py = A.data(:,3);
pz = A.data(:,4);
p = [px py pz]';

% orientation in Base frame
qx = A.data(:,5);
qy = A.data(:,6);
qz = A.data(:,7);
qw = A.data(:,8);
Q = [qw qx qy qz];
R = quat2rotm(Q);

% offsets
offset = ones(size(p)-1);
for i = 1:(size(p,2)-1)
   offset(1,i) = p(1,i+1) - p(1,i);
   offset(2,i) = p(2,i+1) - p(2,i);
   offset(3,i) = p(3,i+1) - p(3,i);
end

%% mould points
filename = '/home/helio/kst/polishing/planes/mould_points';
points = importdata(filename);
P1 = points(1,:);
P2 = points(2,:);
P3 = points(3,:);
P4 = points(4,:);

% Compute the vectors of the desired rotation matrix
nx = (P4 - P1)/norm(P4 - P1);
ny = (P2 - P1)/norm(P2 - P1);
nz = cross(nx, ny);
nz = nz/norm(nz);

% Complete the rotation matrix (orthogonality characteristic)
ny = cross(nz,nx);
ny = ny/norm(ny);

% Get the desired rotation
Rmould = [nx',ny',nz'];

%% Compute the desired rotation in all points of plane
Rd = Rmould * R(:,:,1);

%% plane
delta_synthetic = [0, 0, -0.001];
P1_synthetic = P1 + delta_synthetic;
P2_synthetic = P2 + delta_synthetic;
P4_synthetic = P4 + delta_synthetic;
vec_x = (P4_synthetic - P1_synthetic);
vec_y = (P2_synthetic - P1_synthetic);
p = [];
for i=linspace(0,1,8)
   for j=linspace(0,1,8)
      p = [p; P1_synthetic + (vec_x*i + vec_y*j)];
   end      
end

%% Execution of the pattern in one the work-space points   
pd = ones(3,size(offset,2));
pd(:,1) = p(size(p,1),:)';
for k = 1:size(offset,2)
   pd(:,k+1) = pd(:,k) + offset(:,k);
end


%% plot

% mould work-space
pp = [P1;P2;P3;P4;P1];

figure(1);
plot3(pp(:,1), pp(:,2), pp(:,3), '--k')
hold on;
plot3(px, py, pz, '.r');
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
title('End-Effector position in Base frame');
legend('mould work-space', 'pattern');

figure(2);
plot3(pp(:,1), pp(:,2), pp(:,3), '--k')
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
hold on;

% plane
scatter3(p(:,1), p(:,2), p(:,3));

% desired rotation axis
K_axis = 0.05;
plot3([P1(1),P1(1)+K_axis*Rmould(1,1)], [P1(2),P1(2)+K_axis*Rmould(2,1)], [P1(3),P1(3)+K_axis*Rmould(3,1)],'r', 'LineWidth', 2);
plot3([P1(1),P1(1)+K_axis*Rmould(1,2)], [P1(2),P1(2)+K_axis*Rmould(2,2)], [P1(3),P1(3)+K_axis*Rmould(3,2)],'g', 'LineWidth', 2);
plot3([P1(1),P1(1)+K_axis*Rmould(1,3)], [P1(2),P1(2)+K_axis*Rmould(2,3)], [P1(3),P1(3)+K_axis*Rmould(3,3)],'b', 'LineWidth', 2);

% points pattern
plot3(pd(1,:), pd(2,:), pd(3,:), '.r')
legend('mould work-space', 'plane', 'X', 'Y', 'Z', 'pattern');
title('End-Effector position in Base frame');
