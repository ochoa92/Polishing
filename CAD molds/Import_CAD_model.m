% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : import cad model to matlab         
% =========================================================================

clear all;
clc;
close all;

% Import an STL mesh, returning a PATCH-compatible face-vertex structure
% fv = stlread('mold.stl');
% patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
%          'EdgeColor',       'none',        ...
%          'FaceLighting',    'gouraud',     ...
%          'AmbientStrength', 0.15);



% [F,V,N] = stlread('mold.stl');
% 
% % convert mm to m
% V = V*1e-3; % Vertices
% N = N*1e-3; % Face normal vectors
% 
% [meshXYZ] = CONVERT_meshformat(F,V);
% X = meshXYZ(:,1,1);
% Y = meshXYZ(:,2,1);
% Z = meshXYZ(:,3,1);
% 
% %% PLOT 1
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

% %% SELECT AREA
% input = ginput(4);
% P = [input(:,1), input(:,2); input(1,:)];
% plot(P(:,1), P(:,2), 'r')
% area = [input(:,1), input(:,2)];
% points = [X, Y, Z];
% points_inside_area = get_points_inside_area(area, points);
% X = points_inside_area(:,1);
% Y = points_inside_area(:,2);
% Z = points_inside_area(:,3);


%% PLOT 2
% figure(2)
% hold on
% 
% patch('Faces',F, ...
%       'Vertices',V, ...
%       'FaceColor', [0.8 0.8 1.0], ...
%       'EdgeColor', 'none', ...
%       'FaceLighting',    'gouraud', ...
%       'AmbientStrength', 0.15);
% camlight('headlight');
% material('metal');
% 
% % plot3(X,Y,Z,'or');
% TRI=delaunay(X,Y);
% trisurf(TRI,X,Y,Z);
% % trimesh(TRI,X,Y,Z);
% 
% TR = triangulation(TRI,X,Y,Z);
% IC = incenter(TR);
% plot3(IC(:,1),IC(:,2),IC(:,3), '*b')
% 
% FN = faceNormal(TR);
% quiver3(IC(:,1),IC(:,2),IC(:,3), FN(:,1),FN(:,2),FN(:,3),10,'color','r');
%      
% 
% axis equal
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

%% PLOT 3
figure(2)
hold on

model = createpde(1);
importGeometry(model,'mold.stl');
generateMesh(model, 'Hmax', 50, 'Hmin', 1); 

pdeplot3D(model)
      b   
%% PLOT 4
figure(3)
hold on

P = model.Mesh.Nodes;
P = P*1e-3; % convert mm to m
X = P(1,:)';
Y = P(2,:)';
Z = P(3,:)';

plot3(X,Y,Z,'or');

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
