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
% A = importdata('/home/helio/catkin_ws/src/franka_ros/franka_polishing/mold_data/pattern');
A = importdata('/home/helio/kst/polishing/patterns/polishing_pattern');
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get mold points from a file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
B = importdata('/home/helio/catkin_ws/src/franka_ros/franka_polishing/mold_data/mold_points');
% px py pz qw qx qy qz
% mold position in Base frame
Pmold = [B.data(:,1) B.data(:,2) B.data(:,3)]';

% mold orientation in Base frame
Qmold = [B.data(:,4) B.data(:,5) B.data(:,6) B.data(:,7)]; 
Rmold = quat2rotm(Qmold);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get points of the mold work-space from a file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = importdata('/home/helio/catkin_ws/src/franka_ros/franka_polishing/mold_data/mold_workspace');
P1 = P(1,:);
P2 = P(2,:);
P3 = P(3,:);
P4 = P(4,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Execution of the pattern in one the mold points   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mold point 'n'
n_points = length(Pmold);
for n = 1:1
    pd(:,1) = Pmold(:,n);
    for k = 1:size(offset,2)
       pd(:,k+1) = pd(:,k) + Rmold(:,:,n)*offset(:,k);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Define work-space limits
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    new_pd = get_points_inside_area(P, pd');


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Plot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % mold work-space
    pp = [P1;P2;P3;P4;P1];
    K = 0.05;

    figure(1);
    hold on
    plot3(pp(:,1), pp(:,2), pp(:,3), '--r')
    plot3(P(:,1), P(:,2), P(:,3), 'or')
    scatter3(Pmold(1,:), Pmold(2,:), Pmold(3,:), 'b');
    plot3(pd(1,:), pd(2,:), pd(3,:), '.r')
    plot3(new_pd(:,1), new_pd(:,2), new_pd(:,3), '.g')
    quiver3(Pmold(1,n), Pmold(2,n), Pmold(3,n), K*Rmold(1,1,n), K*Rmold(2,1,n), K*Rmold(3,1,n), 'r', 'linewidth',2)
    quiver3(Pmold(1,n), Pmold(2,n), Pmold(3,n), K*Rmold(1,2,n), K*Rmold(2,2,n), K*Rmold(3,2,n), 'g', 'linewidth',2)
    quiver3(Pmold(1,n), Pmold(2,n), Pmold(3,n), K*Rmold(1,3,n), K*Rmold(2,3,n), K*Rmold(3,3,n), 'b', 'linewidth',2)
    legend('mold_{ws}', 'points_{ws}', 'mold-points', 'pd', 'pd_{inside ws}', 'Rm_{x}', 'Rm_{y}', 'Rm_{z}')
    axis equal
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('End-Effector position inside the work-space limits');
end


