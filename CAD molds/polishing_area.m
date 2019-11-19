% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : import cad model to matlab and select the polishing target
%               area ( px,py,pz ; ox,oy,oz )  
% =========================================================================

clear all;
clc;
close all;

%% IMPORT CAD TO MATLAB
[F,V,N] = stlread('mold.stl');

% convert mm to m
V = V*1e-3; % Vertices
N = N*1e-3; % Face normal vectors

[meshXYZ] = CONVERT_meshformat(F,V);
X = meshXYZ(:,1,1);
Y = meshXYZ(:,2,1);
Z = meshXYZ(:,3,1);


%% PLOT 1
figure(1)
hold on

patch('Faces',F, ...
      'Vertices',V, ...
      'FaceColor', [0.8 0.8 1.0], ...
      'EdgeColor', 'none', ...
      'FaceLighting',    'gouraud', ...
      'AmbientStrength', 0.15);
  
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('metal');

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')


% %% SELECT POLISHING AREA
% input = ginput(4);
% P = [input(:,1), input(:,2); input(1,:)];
% plot(P(:,1), P(:,2), 'r')
% area = [input(:,1), input(:,2)];
% points = [X, Y, Z];
% points_inside_area = get_points_inside_area(area, points);
% X = points_inside_area(:,1);
% Y = points_inside_area(:,2);
% Z = points_inside_area(:,3);


%% GET TRIANGLES AND INCENTERS
TRI = delaunay(X,Y);
TR = triangulation(TRI,X,Y,Z);
IC = incenter(TR);
FN = faceNormal(TR);

%% MESHGRID
P = [X Y Z]; % all positions 

C = max(P); % max of the samples 
n = [0 0 1];
d = -C(1)*n(1) - C(2)*n(2) - C(3)*n(3);

x = linspace(min(P(:,1)), max(P(:,1)), 20);
y = linspace(min(P(:,2)), max(P(:,2)), 10);
[xx,yy] = meshgrid(x,y);
zz = ((-d - (n(1)*xx) - (n(2)*yy))/n(3));


%% POINTS INSIDE MESHGRID AREAS
c = length(x);
l = length(y);

points_inside = [];
normals_inside = [];
for i=1:l-1
   for j=1:c-1
       
       area = [xx(i,j) yy(i,j); xx(i+1,j) yy(i+1,j); xx(i+1,j+1) yy(i+1,j+1); xx(i,j+1) yy(i,j+1)];
       
       points = get_points_inside_area(area, IC);
       points_inside = [points_inside; mean(points,1)];
       
       normals = get_points_inside_area(area, FN);
       normals_inside = [normals_inside; mean(normals,1)];
       
   end
end


%% PLOT 2
figure(2)
hold on

patch('Faces',F, ...
      'Vertices',V, ...
      'FaceColor', [0.8 0.8 1.0], ...
      'EdgeColor', 'none', ...
      'FaceLighting',    'gouraud', ...
      'AmbientStrength', 0.15);
  
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('metal');

% trisurf(TRI,X,Y,Z);
% trimesh(TRI,X,Y,Z);

plot3(IC(:,1),IC(:,2),IC(:,3), '*b')

mesh(xx,yy,zz, 'FaceColor', 'none');

plot3(points_inside(:,1), points_inside(:,2), points_inside(:,3), '+r');
% quiver3(points_inside(:,1),points_inside(:,2),points_inside(:,3), normals_inside(:,1),normals_inside(:,2),normals_inside(:,3), 0.5,'color','r');

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

