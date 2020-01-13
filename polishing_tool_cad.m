% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description : 
% =========================================================================
clear all;
clc;
close all;

%% POLISHING TOOL CAD

m = 0.610;  % mass(kg)

center_of_mass = [0 0 0.0575];  % center of mass of load Vector

% moments of inertia of a solid cylinder
% Iz = (1/2)*m*r²
% Ix = Iz = (1/12)*m*(3r²+h²)

r = 0.0315;   % radius(m)
h = 0.12;   % height(m)

Ix = (1/12)*m*(3*(r^2) + (h^2)); 
Iy = Ix;
Iz = (1/2)*m*(r^2);

I = [Ix 0 0;
     0 Iy 0;
     0 0 Iz];   

% Transformation Matrix from Flange(panda_link8) to End-Effector
delta_z = 0.115; %m
delta_x = 0.065;  %m 

% h² = x² + y² -> (x = y): angle between panda_link8 and buttons is -45 degrees

px = sqrt( (delta_x^2)/2 );
py = -px;
pz = delta_z;
T = [1 0 0 px;
     0 1 0 py;
     0 0 1 pz;
     0 0 0  1];

 angle_x = 0;   %rad
 Tx = [1            0             0 0;
       0 cos(angle_x) -sin(angle_x) 0;
       0 sin(angle_x)  cos(angle_x) 0;
       0            0             0 1];
   
 angle_y = pi/3;  %rad
 Ty = [ cos(angle_y) 0 sin(angle_y) 0;
                   0 1            0 0;
       -sin(angle_y) 0 cos(angle_y) 0;
                   0 0            0 1];
   
 angle_z = -pi/4;  %rad
 Tz = [cos(angle_z) -sin(angle_z) 0 0;
       sin(angle_z)  cos(angle_z) 0 0;
                  0             0 1 0;
                  0             0 0 1];

 P_T_EE = T*Tz*Ty;
 
 % Rotation Matrix from Flange(panda_link8) to End-Effector
 p_8EE = P_T_EE(1:3,end);
 R_8EE = tform2rotm(P_T_EE);
 RPY = rotm2eul(R_8EE, 'ZYX');
 Q = rotm2quat(R_8EE);
 