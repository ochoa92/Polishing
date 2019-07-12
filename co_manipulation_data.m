% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : Build the co-manipulation planes          
% =========================================================================

clear all
clc
close all

%% plane points
points = importdata('/home/helio/catkin_ws/src/franka_ros/franka_polishing/co_manipulation_data/plane_points');

n_points = length(points);
n_planes = n_points/4;

n = 1;
k = 1;
while k <= n_planes
    P1 = points(n,:);
    P2 = points(n+1,:);
    P3 = points(n+2,:);
    P4 = points(n+3,:);

    % Compute the vectors of the desired rotation matrix
    nx = (P4 - P1)/norm(P4 - P1);
    ny = (P2 - P1)/norm(P2 - P1);
    nz = cross(nx, ny);
    nz = nz/norm(nz);

    % Complete the rotation matrix (orthogonality characteristic)
    ny = cross(nz,nx);
    ny = ny/norm(ny);

    % Get the desired rotation
    Rmold = [nx',ny',nz'];

    %% mold grid
    vec_x = (P4 - P1);
    vec_y = (P2 - P1);
    Nx = 5;
    Ny = 5;
    mold_grid = [];
    for i=linspace(0,1,Nx)
       for j=linspace(0,1,Ny)
          mold_grid = [mold_grid; P1 + (vec_x*i + vec_y*j)];
       end      
    end


    %% plot

    % mold work-space
    p_mold = [P1;P2;P3;P4;P1];
    K_axis = 0.01;

    figure(1);
    % mold
    plot3(p_mold(:,1), p_mold(:,2), p_mold(:,3), '--k')
    hold on
    plot3([P1(1), P1(1)+K_axis*Rmold(1,1)], [P1(2), P1(2)+K_axis*Rmold(2,1)], [P1(3), P1(3)+K_axis*Rmold(3,1)],'r', 'LineWidth', 2);
    plot3([P1(1), P1(1)+K_axis*Rmold(1,2)], [P1(2), P1(2)+K_axis*Rmold(2,2)], [P1(3), P1(3)+K_axis*Rmold(3,2)],'g', 'LineWidth', 2);
    plot3([P1(1), P1(1)+K_axis*Rmold(1,3)], [P1(2), P1(2)+K_axis*Rmold(2,3)], [P1(3), P1(3)+K_axis*Rmold(3,3)],'b', 'LineWidth', 2);
    hold on
    scatter3(mold_grid(:,1), mold_grid(:,2), mold_grid(:,3), 'k');
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend('mold_{points}', 'X_{mold}', 'Y_{mold}', 'Z_{mold}', 'mold-grid');
   
    k = k + 1;
    n = n + 4;
end