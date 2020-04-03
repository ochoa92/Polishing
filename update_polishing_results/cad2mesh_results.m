% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : Import a stl mesh and build a triangulation with this mesh. 
%               Then a polishing area is selected, where the starting
%               points and normals are identified. Finally, the
%               selected data is projected to robot base frame.
% =========================================================================

clear all;
clc;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CADMESH
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[F,V,N] = stlread('/home/ochoa/Desktop/TOOLING4G/Polishing/CAD molds/polishing_mold.stl');

% PLOT
TR = triangulation(F,V);
IC = incenter(TR);
FN = faceNormal(TR);

fh1 = figure(1);
fh1.WindowState = 'maximized';
hold on
trisurf(TR)

axis equal
grid on
xlabel('X (m)', 'FontSize', 30)
ylabel('Y (m)', 'FontSize', 30)
zlabel('Z (m)', 'FontSize', 30)
set(gca,'FontSize',30)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% POLISHING AREA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% PLOT 
fh2 = figure(2);
fh2.WindowState = 'maximized';
hold on
trisurf(TR)
axis equal
grid on
xlabel('X (m)', 'FontSize', 30)
ylabel('Y (m)', 'FontSize', 30)
zlabel('Z (m)', 'FontSize', 30)
set(gca,'FontSize',30)

n_points_selected = 0;
max_n_points = 100;
while n_points_selected < max_n_points
    n_points_selected = n_points_selected + 1;
    [x(n_points_selected), y(n_points_selected), button] = ginput(1);  
    p1 = plot(x(n_points_selected), y(n_points_selected), 'r*', 'MarkerSize', 15, 'linewidth', 2);
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

% remove underside IN's and FN's 
polishing_IC = [];
polishing_FN = [];
for i=1:length(IC_in)
    if (IC_in(i,3) > -0.07)
        polishing_IC = [polishing_IC; IC_in(i,:)];
        polishing_FN = [polishing_FN; FN_in(i,:)];
    end
end

p2 = plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '.g','MarkerSize', 20, 'linewidth', 2);
p3 = quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), polishing_FN(:,1),polishing_FN(:,2),polishing_FN(:,3), 2, 'color','b', 'linewidth', 2 );
hold off
legend([p1 p2 p3], {'V_{pa}','C_{pa}','N_{pa}'})

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CADMESH IN ROBOT BASE FRAME
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nx = [];
ny = [];
nz = [];
for i=1:length(polishing_FN)
    R = normal2Rotation(polishing_FN(i,:));
    nx = [nx; R(:,1)'];
    ny = [ny; R(:,2)'];
    nz = [nz; R(:,3)'];
end

% Get mold pose from a file
M = importdata('/home/ochoa/franka_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/mold_workspace');
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

% PLOT
fh3 = figure(3);
fh3.WindowState = 'maximized';
hold on
% axis equal
grid on
xlabel('X (m)', 'FontSize', 30)
ylabel('Y (m)', 'FontSize', 30)
zlabel('Z (m)', 'FontSize', 30)
set(gca,'FontSize',30)

% robot base-frame
quiver3(0,0,0, 1,0,0,0.1,'color','r', 'linewidth', 5);
quiver3(0,0,0, 0,1,0,0.1,'color','g', 'linewidth', 5);
quiver3(0,0,0, 0,0,1,0.1,'color','b', 'linewidth', 5);

% % points in CAD frame
% plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '*r')
% quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), polishing_FN(:,1),polishing_FN(:,2),polishing_FN(:,3), 2, 'color','b');

% mold work-space
plot3(P1(:,1), P1(:,2), P1(:,3), '.r', 'MarkerSize', 40, 'linewidth', 2)
plot3(P2(:,1), P2(:,2), P2(:,3), '.r', 'MarkerSize', 40, 'linewidth', 2)
plot3(P3(:,1), P3(:,2), P3(:,3), '.r', 'MarkerSize', 40, 'linewidth', 2)
plot3(P4(:,1), P4(:,2), P4(:,3), '.r', 'MarkerSize', 40, 'linewidth', 2)
plot3(Mold(:,1), Mold(:,2), Mold(:,3), '--r', 'linewidth', 2)

% mold mesh
trisurf(TRm)   

% points in mold frame
plot3(pm(:,1),pm(:,2),pm(:,3), '.g', 'MarkerSize', 20, 'linewidth', 2)

% mold frame
quiver3(P1(1),P1(2),P1(3), Rm(1,1),Rm(2,1),Rm(3,1),0.1,'color','r', 'linewidth', 5);
quiver3(P1(1),P1(2),P1(3), Rm(1,2),Rm(2,2),Rm(3,2),0.1,'color','g', 'linewidth', 5);
quiver3(P1(1),P1(2),P1(3), Rm(1,3),Rm(2,3),Rm(3,3),0.1,'color','b', 'linewidth', 5);

% axis associated to each point in mold frame
% quiver3(pm(:,1),pm(:,2),pm(:,3), nx(:,1),nx(:,2),nx(:,3),2,'color','r', 'linewidth', 3);
% quiver3(pm(:,1),pm(:,2),pm(:,3), ny(:,1),ny(:,2),ny(:,3),2,'color','g', 'linewidth', 3);
quiver3(pm(:,1),pm(:,2),pm(:,3), nz(:,1),nz(:,2),nz(:,3),2,'color','b', 'linewidth', 2);
