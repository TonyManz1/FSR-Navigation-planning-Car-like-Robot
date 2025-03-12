% Main script for Dijkstra path planning with Reed-Shepp curves
clear all;
close all;
clc;

% Load map
load("image_map.mat");
map = double(image_map);

% Initialize parameters
params = struct();
params.length_step = 9;      % Length of the step forward
params.radius_curve = 9;     % Bending radius for left and right curves

% Define start and goal points
start = [30 120];  % Coordinates in format (x,y)
goal = [400 135];  % Coordinates in format (x,y)

% Execute Dijkstra algorithm 
% This will show only the exploration plot with the nodes, not a separate plot for results
[path_points, sample_points, tree_data, path_node_count] = dijkstra_planning(map, start, goal, params);

% Process results
if ~isempty(path_points)
    % Generate desired trajectory
    [xd, yd, thetad, phid, q0] = des_trajectory(path_points);
    
    % Save trajectory data
    save('trajectory_data_dijkstra.mat', 'xd', 'yd', 'thetad', 'phid', 'q0');
    
    % Display information about algorithm performance
    disp('Dijkstra path planning completed successfully.');
    disp(['Path nodes: ', num2str(path_node_count), ' nodes']);
    disp(['Total explored nodes: ', num2str(tree_data.counter), ' nodes']);
else
    disp('No path found.');
end