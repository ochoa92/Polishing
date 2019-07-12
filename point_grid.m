% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : 
% =========================================================================
clear all;
clc;
close all;


%% mould points
filename = '/home/helio/kst/polishing/planes/mould_points';
points = importdata(filename);
P1 = points(1,:);
P2 = points(2,:);
P3 = points(3,:);
P4 = points(4,:);

%% point grid
Nx = 5;
Ny = 5;
vec_x = (P4 - P1);
vec_y = (P2 - P1);
p = [];
for i=linspace(0,1,Nx)
   for j=linspace(0,1,Ny)
      p = [p; P1 + (vec_x*i + vec_y*j)];
   end      
end

p_grid_initial = P1 + vec_x/(2*Nx) + vec_y/(2*Ny);
p_grid = [];
step_x = 1/(Nx-1);
step_y = 1/(Ny-1);
for i=0:step_x:(1-step_x)
   for j=0:step_y:(1-step_y)
      p_grid = [p_grid; p_grid_initial + (vec_x*i + vec_y*j)];
   end      
end


%% plot
figure(1);
hold on;

% mould work-space
pp = [P1;P2;P3;P4;P1];
plot3(P1(:,1), P1(:,2), P1(:,3), 'or')
plot3(P2(:,1), P2(:,2), P2(:,3), 'or')
plot3(P3(:,1), P3(:,2), P3(:,3), 'or')
plot3(P4(:,1), P4(:,2), P4(:,3), 'or')
plot3(pp(:,1), pp(:,2), pp(:,3), '--r')

% point grid
scatter3(p(:,1), p(:,2), p(:,3));
scatter3(p_grid(:,1), p_grid(:,2), p_grid(:,3), 'og');

grid on;
xlabel('x');
ylabel('y');
zlabel('z');