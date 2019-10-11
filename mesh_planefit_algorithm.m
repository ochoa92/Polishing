% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : Compute the surface normals 
% =========================================================================

clear all
clc
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Build the surface using robot EE positions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = importdata('molds/mold_data_3');

% t p_x p_y p_z Qx Qy Qz Qw Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O
% position in Base frame
px = A.data(:,2);
py = A.data(:,3);
pz = A.data(:,4);

p = [px py pz]; % all positions 
[n,R,C] = plane_fit(p); % plane fit for all positions 

% mesh grid
d = -C(1)*n(1) - C(2)*n(2) - C(3)*n(3);
% x = linspace(min(px), max(px), 3); 
x = linspace(min(px), max(px), 4);
y = linspace(min(py), max(py), 9);
[xx,yy] = meshgrid(x,y);
zz = ((-d - (n(1)*xx) - (n(2)*yy))/n(3));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mesh grid areas
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c = length(x);
l = length(y);
areas = [];
normals = [];
centers = [];
points_inside = [];
nx = [];
ny = [];
for i=1:l-1
   for j=1:c-1
       
       area = [xx(i,j) yy(i,j); xx(i+1,j) yy(i+1,j); xx(i+1,j+1) yy(i+1,j+1); xx(i,j+1) yy(i,j+1)];
       areas = [areas; area(1,:) zz(i,j); area(2,:) zz(i+1,j); area(3,:) zz(i+1,j+1); area(4,:) zz(i,j+1)];
       
       points = get_points_inside_area(area, p);      
       if ~isempty(points)
            [n,R,center] = plane_fit(points);        
            normals = [normals; n'];
            centers = [centers; center];
            points_inside = [points_inside; points];
            nx = [nx; R(:,1)'];
            ny = [ny; R(:,2)'];
               
       end       
   end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open file to write
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
filepath = "/home/helio/catkin_ws/src/TOOLING4G/franka_polishing/mold_data/mold_points";
fileID = fopen(filepath,'w');
fprintf(fileID,'px py pz qw qx qy qz\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Align the axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
new_nx = [];
new_ny = [];
new_nz = [];
for i=1:length(centers)
%     vx = nx(6,:);
    vx = nx(13,:);

    vx = vx/norm(vx);
    
    vz = normals(i,:);
    vz = vz/norm(vz);
    
    vy = cross(vz,vx);
    vy = vy/norm(vy);
    
    vx = cross(vy,vz);
    vx = vx/norm(vx);
    
    new_nx = [new_nx; vx];
    new_ny = [new_ny; vy];
    new_nz = [new_nz; vz];
    
    Rd = [vx' vy' vz'];
    
    % write data to file
%     delta_synthetic = [0.0, 0.0, -0.001];
    delta_synthetic = [0.0, 0.0, -0.0005];
    position = centers(i,:) + delta_synthetic;
    orientation = rotm2quat(Rd);
    fprintf(fileID,'%d %d %d', position);
    fprintf(fileID,' %d %d %d %d\n', orientation);
end
fclose(fileID); % close file


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
hold on
plot3(p(:,1), p(:,2), p(:,3), 'or');
mesh(xx,yy,zz, 'FaceColor', 'none');
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

K = 0.1;
figure(2)
hold on
plot3(points_inside(:,1), points_inside(:,2), points_inside(:,3), 'or');
mesh(xx,yy,zz, 'FaceColor', 'none');
plot3(areas(:,1), areas(:,2), areas(:,3), '*k', 'linewidth', 2);
plot3(centers(:,1), centers(:,2), centers(:,3), 'ob', 'linewidth', 2);
quiver3(centers(:,1), centers(:,2), centers(:,3), K*normals(:,1), K*normals(:,2), K*normals(:,3), 'b', 'linewidth',2)
axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('points inside area', 'mesh', 'points of each area', 'plane center', 'normal')

figure(3)
hold on
mesh(xx,yy,zz, 'FaceColor', 'none');
plot3(centers(:,1), centers(:,2), centers(:,3), 'ob', 'linewidth', 2);
quiver3(centers(:,1), centers(:,2), centers(:,3), K*nx(:,1), K*nx(:,2), K*nx(:,3), 'r', 'linewidth',2)
quiver3(centers(:,1), centers(:,2), centers(:,3), K*ny(:,1), K*ny(:,2), K*ny(:,3), 'g', 'linewidth',2)
quiver3(centers(:,1), centers(:,2), centers(:,3), K*normals(:,1), K*normals(:,2), K*normals(:,3), 'b', 'linewidth',2)
axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('mesh', 'plane center', 'nx', 'ny', 'nz')


figure(4)
hold on
mesh(xx,yy,zz, 'FaceColor', 'none');
plot3(centers(:,1), centers(:,2), centers(:,3), 'ob', 'linewidth', 2);
quiver3(centers(:,1), centers(:,2), centers(:,3), K*new_nx(:,1), K*new_nx(:,2), K*new_nx(:,3), 'r', 'linewidth',2)
quiver3(centers(:,1), centers(:,2), centers(:,3), K*new_ny(:,1), K*new_ny(:,2), K*new_ny(:,3), 'g', 'linewidth',2)
quiver3(centers(:,1), centers(:,2), centers(:,3), K*new_nz(:,1), K*new_nz(:,2), K*new_nz(:,3), 'b', 'linewidth',2)
axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('mesh', 'plane center', 'new_{nx}', 'new_{ny}', 'nz')
