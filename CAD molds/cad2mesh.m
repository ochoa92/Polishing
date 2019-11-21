% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description :         
% =========================================================================

clear all;
clc;
close all;

%% Import an STL mesh, returning a PATCH-compatible face-vertex structure
[F,V,N] = stlread('polishing_mold.stl');

% convert mm to m
V = V*1e-3; % Vertices
N = N*1e-3; % Face normal vectors

TR = triangulation(F,V);
IC = incenter(TR);
FN = faceNormal(TR);

fh = figure(1);
fh.WindowState = 'maximized';
hold on

trisurf(TR)
plot3(IC(:,1),IC(:,2),IC(:,3), '*r')
quiver3(IC(:,1),IC(:,2),IC(:,3), FN(:,1),FN(:,2),FN(:,3),2,'color','r');

axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

% Select Polishing area with N points
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

selected_points = [x' y'];
area = [selected_points(:,1), selected_points(:,2); selected_points(1,:)];
plot(area(:,1), area(:,2), 'b', 'linewidth', 2)

%%
p = IC;
[p_inside, p_inside_index] = get_npoints_inside_area(area, p);

plot3(p_inside(:,1), p_inside(:,2), p_inside(:,3), 'k*')

