% =========================================================================
% Project Name: TOOLING4G - Polishing
% Author      : Hélio Ochoa
% Description :         
% =========================================================================

clear all;
clc;
close all;


c = 0.5;
x = -15:0.001:15;
sigmoid = 1./(1+exp(-c*x));

figure(1)
hold on 
plot(x,sigmoid)
grid on