% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : 
% =========================================================================
clear all;
clc;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get pattern from a file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = importdata('/home/helio/kst/polishing/patterns/polishing_pattern');
% t p_x p_y p_z Qx Qy Qz Qw Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O

% pattern position in Base frame
Ppattern = [A.data(:,2) A.data(:,3) A.data(:,4)]';  % p = [px py pz]';

% pattern orientation in Base frame
Qpattern = [A.data(:,8) A.data(:,5) A.data(:,6) A.data(:,7)];   % Q = [qw qx qy qz];
Rpattern = quat2rotm(Qpattern);

% pattern offsets
offset = ones(size(Ppattern)-1);
for i = 1:(size(Ppattern,2)-1)
   offset(1,i) = Ppattern(1,i+1) - Ppattern(1,i);
   offset(2,i) = Ppattern(2,i+1) - Ppattern(2,i);
   offset(3,i) = Ppattern(3,i+1) - Ppattern(3,i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get the limit points of the mold work-space from a file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = importdata('/home/helio/kst/polishing/planes/mould_points');
P1 = P(1,:);
P2 = P(2,:);
P3 = P(3,:);
P4 = P(4,:);

% Compute the mold rotation 
Rmold = points2Rotation(P1', P2', P4');

% Compute the desired rotation in all points of the work-space
Rd = Rmold * Rpattern(:,:,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Build the points of the mold work-space (point grid)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_synthetic = [0, 0, -0.001];
Nx = 7;
Ny = 5;
Pmold = moldWorkSpace(P1 ,P2, P4, delta_synthetic, Nx, Ny);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Execution of the pattern in one the mold points   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mold point 'n'
n_points = length(Pmold);
for n = 1:1
    pd(:,1) = Pmold(n,:);
    for k = 1:size(offset,2)
       pd(:,k+1) = pd(:,k) + Rmold*offset(:,k);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Define work-space limits
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    new_pd = get_points_inside_area(P, pd');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% plot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    figure(1);
    hold on;

    % mold work-space
    mold_ws = [P1;P2;P3;P4;P1];
    plot3(mold_ws(:,1), mold_ws(:,2), mold_ws(:,3), '--r')
    plot3(P1(:,1), P1(:,2), P1(:,3), 'or')
    plot3(P2(:,1), P2(:,2), P2(:,3), 'or')
    plot3(P3(:,1), P3(:,2), P3(:,3), 'or')
    plot3(P4(:,1), P4(:,2), P4(:,3), 'or')

    % pattern in robot base frame
    plot3(Ppattern(1,:), Ppattern(2,:), Ppattern(3,:), '.r');

    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('End-Effector position in Base frame and Mold WS');
    legend('mold_{WS}', 'P1', 'P2', 'P3', 'P4', 'pattern');


    figure(2);
    hold on;
    plot3(mold_ws(:,1), mold_ws(:,2), mold_ws(:,3), '--r')
    plot3(P1(:,1), P1(:,2), P1(:,3), 'or')
    plot3(P2(:,1), P2(:,2), P2(:,3), 'or')
    plot3(P3(:,1), P3(:,2), P3(:,3), 'or')
    plot3(P4(:,1), P4(:,2), P4(:,3), 'or')

    % point grid of the mold work-space
    scatter3(Pmold(:,1), Pmold(:,2), Pmold(:,3), 'og');
    
    % pattern in mold frame
    plot3(pd(1,:), pd(2,:), pd(3,:), '.r')
    plot3(new_pd(:,1), new_pd(:,2), new_pd(:,3), '.g')

    % mold rotation axis
    K = 0.05;
    quiver3(P1(1), P1(2), P1(3), K*Rmold(1,1), K*Rmold(2,1), K*Rmold(3,1), 'r', 'linewidth',2)
    quiver3(P1(1), P1(2), P1(3), K*Rmold(1,2), K*Rmold(2,2), K*Rmold(3,2), 'g', 'linewidth',2)
    quiver3(P1(1), P1(2), P1(3), K*Rmold(1,3), K*Rmold(2,3), K*Rmold(3,3), 'b', 'linewidth',2)
    axis equal  
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend('mold_{WS}', 'P1', 'P2', 'P3', 'P4', 'Pm', 'pattern', 'pattern inside ws', 'Rm_{x}', 'Rm_{y}', 'Rm_{z}');
    title('End-Effector position in mold frame');

end
