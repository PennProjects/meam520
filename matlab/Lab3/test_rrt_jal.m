clear all
close all
addpath('maps')

start = [-pi/4,-pi/4,0,0,0,0];
goal = [1.2,1,0,0,0,0];

map = loadmap('map2.txt');

% Find collision-free path using RRT to get list of waypoints
[path] = jal_rrt_plot(map, start, goal);