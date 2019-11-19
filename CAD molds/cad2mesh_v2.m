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

figure(1)
patch('Faces',F, ...
      'Vertices',V, ...
      'FaceColor', [0.8 0.8 1.0], ...
      'EdgeColor', 'r', ...
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

%%
% I = imread('eight.tif');
% figure(1)
% [BW,xi2,yi2] = roipoly(I);
% imshow(I)
% 
% figure(2)
% imshow(BW)