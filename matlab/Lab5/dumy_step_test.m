% clc
% clear
% 
% 
% % controlling variable: zeta
% 
% addpath('../Core')
% addpath('maps')
% qStart = [0 0 0 0 0 0];
% qGoal = [0 -pi/4 -pi/4 0 0 0]; 
% 
% map = loadmap('map2.txt');
% 
% % default 
% zeta_default = 0.1*ones(6,3);
% eta_default = 1e6*ones(6,3);
% rhoa_default = 50*ones(6,1);
% rhob_default = 20*ones(6,1);
% stepsize_default = 0.08*ones(1,6);
% 
% % zeta_value = [0.001, 0.1, 1, 10, 100];
% zeta_value = zeta_default(1,1)
% eta_value = [0.001, 0.1, 1, 10 100]; 
% rhoa_value = [5, 10, 50, 100, 200]; 
% rhob_value = [5, 10, 50, 100, 200]; 
% 
% stepsize_value = []; 
% 
% [path] = potentialFieldPath(map, qStart, qGoal);

clear all
close all
addpath('maps')
addpath('../Core')

%% Simulation Parameters

qStart = [0,0,0,0,0,0];
qGoal = [0 ,-pi/4,-pi/4,0,0,0];

map = loadmap('map2.txt');

% Get the path
[path] = potentialFieldPath(map, qStart, qGoal);
