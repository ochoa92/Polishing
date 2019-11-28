% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description :         
% =========================================================================

clear all;
clc;
close all;


% Import an STL mesh, returning a PATCH-compatible face-vertex structure
[F,V,N] = stlread('polishing_mold2.stl');

% convert mm to m
% V = V*1e-3; % Vertices
% N = N*1e-3; % Face normal vectors
V = V*1e-4; % Vertices
N = N*1e-4; % Face normal vectors


%% PLOT 1
% figure(1)
% hold on
% 
% patch('Faces',F, ...
%       'Vertices',V, ...
%       'FaceColor', [0.8 0.8 1.0], ...
%       'EdgeColor', 'none', ...
%       'FaceLighting',    'gouraud', ...
%       'AmbientStrength', 0.15);
% 
% % Add a camera light, and tone down the specular highlighting
% camlight('headlight');
% material('metal');
% 
% axis equal
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

%% PLOT 2
TR = triangulation(F,V);
IC = incenter(TR);
FN = faceNormal(TR);

fh2 = figure(2);
fh2.WindowState = 'maximized';
hold on
trisurf(TR)
plot3(IC(:,1),IC(:,2),IC(:,3), '*r')
quiver3(IC(:,1),IC(:,2),IC(:,3), FN(:,1),FN(:,2),FN(:,3),2,'color','r');

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

% SELECT POLISHING AREA ---------------------------------------------------
points_selected = 0;
max_points_selected = 4;
while points_selected < max_points_selected
    points_selected = points_selected + 1;
    [x(points_selected), y(points_selected), button] = ginput(1);  
    plot(x(points_selected), y(points_selected), 'b+', 'MarkerSize', 15, 'linewidth', 2)
    if button == 3
        % Exit loop if
        break;
    end
end

area = [x', y'];
points_connected = [area(:,1), area(:,2); area(1,:)];
plot(points_connected(:,1), points_connected(:,2), 'b', 'linewidth', 2)

p = IC;
[p_inside, p_inside_index] = get_points_inside_area(area, p);

FN_inside = [];
for i=1:length(p_inside_index)
    FN_inside = [FN_inside; FN(p_inside_index(i),:)];
end

% remove underside IN's and FN's ------------------------------------------
polishing_IC = [];
polishing_FN = [];
for i=1:length(p_inside)
    if (p_inside(i,3) > -0.07)
        polishing_IC = [polishing_IC; p_inside(i,:)];
        polishing_FN = [polishing_FN; FN_inside(i,:)];
    end
end

% -------------------------------------------------------------------------
plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '*k')
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), polishing_FN(:,1),polishing_FN(:,2),polishing_FN(:,3),2,'color','k');


%% PLOT 3
nx = [];
ny = [];
nz = [];
for i=1:length(polishing_FN)
    R = normal2Rotation(polishing_FN(i,:));
    nx = [nx; R(:,1)'];
    ny = [ny; R(:,2)'];
    nz = [nz; R(:,3)'];
end

fh3 = figure(3);
fh3.WindowState = 'maximized';
hold on

trisurf(TR)
plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '*r')
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), nx(:,1),nx(:,2),nx(:,3),2,'color','r', 'linewidth', 2);
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), ny(:,1),ny(:,2),ny(:,3),2,'color','g', 'linewidth', 2);
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), nz(:,1),nz(:,2),nz(:,3),2,'color','b', 'linewidth', 2);

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')


%% PLOT 4
% Get mold position and orientation from a file
M = importdata('/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/mold_workspace');
P1 = M(1,:);
P2 = M(2,:);
P3 = M(3,:);
P4 = M(4,:);
Mold = [P1; P2; P3; P4; P1];
Rm = points2Rotation(P1', P2', P4'); % mold rotation

% polishing area projection 
Om = P1; % origem
pm = [];
for i=1:length(polishing_IC)    
    pm = [pm; (Rm * polishing_IC(i,:)')' + Om];   
end

% mold projection
Vm = [];
for i=1:length(V)
    Vm = [Vm; (Rm * V(i,:)')' + Om];
end
TRm = triangulation(F,Vm);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open file to write
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
filepath = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/polishing_area";
fileID = fopen(filepath,'w');
fprintf(fileID,'px py pz qw qx qy qz\n');

delta_synthetic = [0.0, 0.0, -0.002];
for i=1:length(pm)
    position = pm(i,:) + delta_synthetic;
    Rotation = [nx(i,:)' ny(i,:)' nz(i,:)'];
    orientation = rotm2quat(Rotation);
    fprintf(fileID,'%d %d %d', position);
    fprintf(fileID,' %d %d %d %d\n', orientation);
end
fclose(fileID); % close file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get pattern from a file
A = importdata('/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/pattern');
% t p_x p_y p_z Qx Qy Qz Qw Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O

% pattern position in Base frame
Ppattern = [A.data(:,2) A.data(:,3) A.data(:,4)]';  % p = [px py pz]';

% pattern orientation in Base frame
Qpattern = [A.data(:,8) A.data(:,5) A.data(:,6) A.data(:,7)];   % Q = [qw qx qy qz];
Rpattern = quat2rotm(Qpattern);

% pattern offsets
offset = zeros(size(Ppattern,1));
for k = 1:(size(Ppattern,2)-1)
   offset(:,k) = Ppattern(:,k+1) - Ppattern(:,k);
end

n_points = length(pm);
for n = 1:1
    pd(:,1) = pm(n,:)';
    Rd = [nx(n,:)' ny(n,:)' nz(n,:)'];
    for k = 1:size(offset,2)
       pd(:,k+1) = pd(:,k) + Rd*offset(:,k);
    end

    fh4 = figure(4);
    fh4.WindowState = 'maximized';
    hold on

    % mold work-space
    plot3(Mold(:,1), Mold(:,2), Mold(:,3), '--b', 'linewidth', 2)
    
    % mold mesh
    trisurf(TRm)
%     patch('Faces',F, ...
%           'Vertices',Vm, ...
%           'FaceColor', [0.8 0.8 1.0], ...
%           'EdgeColor', 'none', ...
%           'FaceLighting',    'gouraud', ...
%           'AmbientStrength', 0.15);
% 
%     % Add a camera light, and tone down the specular highlighting
%     camlight('headlight');
%     material('metal');
    
    
    % points
    plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '*r')
    plot3(pm(:,1),pm(:,2),pm(:,3), '*g','linewidth', 2)

    % mold axis
    quiver3(P1(1),P1(2),P1(3), Rm(1,1),Rm(2,1),Rm(3,1),0.2,'color','r', 'linewidth', 2);
    quiver3(P1(1),P1(2),P1(3), Rm(1,2),Rm(2,2),Rm(3,2),0.2,'color','g', 'linewidth', 2);
    quiver3(P1(1),P1(2),P1(3), Rm(1,3),Rm(2,3),Rm(3,3),0.2,'color','b', 'linewidth', 2);

    % points on mold axis
    quiver3(pm(:,1),pm(:,2),pm(:,3), nx(:,1),nx(:,2),nx(:,3),2,'color','r', 'linewidth', 2);
    quiver3(pm(:,1),pm(:,2),pm(:,3), ny(:,1),ny(:,2),ny(:,3),2,'color','g', 'linewidth', 2);
    quiver3(pm(:,1),pm(:,2),pm(:,3), nz(:,1),nz(:,2),nz(:,3),2,'color','b', 'linewidth', 2);
    
    % pattern on mold axis 
    plot3(pd(1,:), pd(2,:), pd(3,:), '.k')
    
    axis equal
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

end


