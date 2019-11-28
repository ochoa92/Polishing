% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description :         
% =========================================================================

clear all;
clc;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Import an STL mesh, returning a PATCH-compatible face-vertex structure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[F,V,N] = stlread('polishing_mold2.stl');

% convert mm to m
% V = V*1e-3; % Vertices
% N = N*1e-3; % Face normal vectors
V = V*1e-4; % Vertices
N = N*1e-4; % Face normal vectors

%% PLOT 1
TR = triangulation(F,V);
IC = incenter(TR);
FN = faceNormal(TR);

fh1 = figure(1);
fh1.WindowState = 'maximized';
hold on
trisurf(TR)

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select polishing area with N points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_points_selected = 0;
max_n_points = 100;
while n_points_selected < max_n_points
    n_points_selected = n_points_selected + 1;
    [x(n_points_selected), y(n_points_selected), button] = ginput(1);  
    plot(x(n_points_selected), y(n_points_selected), 'b+', 'MarkerSize', 15, 'linewidth', 2)
    if button == 3
        % Exit loop if
        break;
    end
end
pgon = polyshape(x,y);
plot(pgon)

in = inpolygon(IC(:,1),IC(:,2),x,y);
IC_in = [IC(in,1) IC(in,2) IC(in,3)];
FN_in = [FN(in,1) FN(in,2) FN(in,3)];

% plot3(IC_in(:,1),IC_in(:,2),IC_in(:,3),'r+') % points inside
% quiver3(IC_in(:,1),IC_in(:,2),IC_in(:,3), FN_in(:,1),FN_in(:,2),FN_in(:,3), 2, 'color','r'); % normals inside

% remove underside IN's and FN's 
polishing_IC = [];
polishing_FN = [];
for i=1:length(IC_in)
    if (IC_in(i,3) > -0.07)
        polishing_IC = [polishing_IC; IC_in(i,:)];
        polishing_FN = [polishing_FN; FN_in(i,:)];
    end
end
plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '+r')
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), polishing_FN(:,1),polishing_FN(:,2),polishing_FN(:,3), 2, 'color','r');


%% PLOT 2
nx = [];
ny = [];
nz = [];
for i=1:length(polishing_FN)
    R = normal2Rotation(polishing_FN(i,:));
    nx = [nx; R(:,1)'];
    ny = [ny; R(:,2)'];
    nz = [nz; R(:,3)'];
end

fh2 = figure(2);
fh2.WindowState = 'maximized';
hold on

trisurf(TR)
plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '*r')
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), nx(:,1),nx(:,2),nx(:,3), 2,'color','r', 'linewidth', 2);
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), ny(:,1),ny(:,2),ny(:,3), 2,'color','g', 'linewidth', 2);
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), nz(:,1),nz(:,2),nz(:,3), 2,'color','b', 'linewidth', 2);

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

%% PLOT 3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get mold pose from a file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = importdata('/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/mold_workspace');
P1 = M(1,:);
P2 = M(2,:);
P3 = M(3,:);
P4 = M(4,:);
Mold = [P1; P2; P3; P4; P1];
Rm = points2Rotation(P1', P2', P4'); % mold rotation

% Polishing area projection in mold frame
Om = P1; % origem
pm = [];
for i=1:length(polishing_IC)    
    pm = [pm; (Rm * polishing_IC(i,:)')' + Om];   
end

% Projection of the CAD in mold frame
Vm = [];
for i=1:length(V)
    Vm = [Vm; (Rm * V(i,:)')' + Om];
end
TRm = triangulation(F,Vm);

% Projection of the polygon vertices in mold frame
polygon_vertices = [];
for i=1:length(pgon.Vertices)    
    polygon_vertices = [polygon_vertices; pgon.Vertices(i,:) 1];   
end

polygon_vertices_m = [];
for i=1:length(polygon_vertices)    
    polygon_vertices_m = [polygon_vertices_m; (Rm * polygon_vertices(i,:)')' + Om];   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save polygon vertices in a file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
filepath = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/polygon_vertices";
fileID = fopen(filepath,'w');
fprintf(fileID,'Vx Vy\n');
for i=1:length(polygon_vertices_m)
    vertices = polygon_vertices_m(i,1:2);
    fprintf(fileID,'%d %d\n', vertices);
end
fclose(fileID); % close file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save the polishing area in a file
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
% Get polishing pattern from a file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        
    fh3 = figure(3);
    fh3.WindowState = 'maximized';
    hold on

    % mold work-space
    plot3(Mold(:,1), Mold(:,2), Mold(:,3), '--b', 'linewidth', 2)
    
    % mold mesh
    trisurf(TRm)   
    
    % points in CAD frame
    plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '+r')
    
    % points in mold frame
    plot3(pm(:,1),pm(:,2),pm(:,3), '+g')

    % mold axis
    quiver3(P1(1),P1(2),P1(3), Rm(1,1),Rm(2,1),Rm(3,1),0.2,'color','r', 'linewidth', 2);
    quiver3(P1(1),P1(2),P1(3), Rm(1,2),Rm(2,2),Rm(3,2),0.2,'color','g', 'linewidth', 2);
    quiver3(P1(1),P1(2),P1(3), Rm(1,3),Rm(2,3),Rm(3,3),0.2,'color','b', 'linewidth', 2);

    % axis associated to each point in mold frame
    quiver3(pm(:,1),pm(:,2),pm(:,3), nx(:,1),nx(:,2),nx(:,3),2,'color','r', 'linewidth', 2);
    quiver3(pm(:,1),pm(:,2),pm(:,3), ny(:,1),ny(:,2),ny(:,3),2,'color','g', 'linewidth', 2);
    quiver3(pm(:,1),pm(:,2),pm(:,3), nz(:,1),nz(:,2),nz(:,3),2,'color','b', 'linewidth', 2);
    
    % pattern in mold frame 
    pd(:,1) = pm(n,:)';
    Rd = [nx(n,:)' ny(n,:)' nz(n,:)'];
    for k = 1:size(offset,2)
       pd(:,k+1) = pd(:,k) + Rd*offset(:,k);
    end
    plot3(pd(1,:), pd(2,:), pd(3,:), '.r')
        
    % pattern inside polishing area
    pattern_in = inpolygon(pd(1,:),pd(2,:), polygon_vertices_m(:,1),polygon_vertices_m(:,2));
    pd_in = [pd(1,pattern_in); pd(2,pattern_in); pd(3,pattern_in)];
    plot3(pd_in(1,:), pd_in(2,:), pd_in(3,:), '.g', 'linewidth', 2)
    
    axis equal
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
    legend('mold ws', 'mold mesh', 'points in CAD frame', 'points in mold frame', 'X_m', 'Y_m', 'Z_m', 'n_x', 'n_y', 'n_z', 'pattern', 'pattern in')

end

