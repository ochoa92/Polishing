% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description :         
% =========================================================================

clear all;
clc;
close all;


% Import an STL mesh, returning a PATCH-compatible face-vertex structure
% fv = stlread('polishing_mold.stl');
% patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
%          'EdgeColor',       'none',        ...
%          'FaceLighting',    'gouraud',     ...
%          'AmbientStrength', 0.15);
[F,V,N] = stlread('polishing_mold.stl');

% convert mm to m
V = V*1e-3; % Vertices
N = N*1e-3; % Face normal vectors

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

figure(2)
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
input = ginput(4);
P = [input(:,1), input(:,2); input(1,:)];
plot(P(:,1), P(:,2), 'b', 'linewidth', 2)

area = [input(:,1), input(:,2)];
p = IC;
[p_inside, p_inside_index] = get_points_inside_area(area, p);
Ix = p_inside(:,1);
Iy = p_inside(:,2);
Iz = p_inside(:,3);

FN_inside = [];
for i=1:length(p_inside_index)
    FN_inside = [FN_inside; FN(p_inside_index(i),:)];
end

% remove underside IN's and FN's ------------------------------------------
polishing_IC = [];
polishing_FN = [];
for i=1:length(Iz)
    if (Iz(i) > -0.07)
        polishing_IC = [polishing_IC; Ix(i) Iy(i) Iz(i)];
        polishing_FN = [polishing_FN; FN_inside(i,:)];
    end
end

% -------------------------------------------------------------------------

plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '*k')
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), polishing_FN(:,1),polishing_FN(:,2),polishing_FN(:,3),2,'color','k');



%% PLOT 3
figure(3)
hold on

trisurf(TR)
plot3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), '*r')
quiver3(polishing_IC(:,1),polishing_IC(:,2),polishing_IC(:,3), polishing_FN(:,1),polishing_FN(:,2),polishing_FN(:,3),2,'color','r');

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')