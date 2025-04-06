% Main script for RRT* with smooth curves
clear all;
close all;
clc;

% Load map
load('image_map.mat');
map = image_map;

% Initialize parameters
params = struct();
params.length_step = 7;        % Lunghezza del passo
params.radius_curve = 10;      % Raggio di curvatura
params.max_iterations = 700;   % Numero massimo di iterazioni
params.goal_radius = 3;        % Raggio di accettazione del goal
params.neighbor_radius = 10;   % Raggio di vicinato per RRT*
params.goal_bias = 0.15;       % Bias verso il goal

% Define start and goal points
start = [30 120];   % Coordinate in formato (x,y)
goal  = [400 135];  % Coordinate in formato (x,y)

% Initialize visualization
figure('Name', 'RRT* Path Planning with Smooth Curves', 'NumberTitle', 'off', 'Position', [100 100 800 600]);
imshow(map);
hold on;
% FIXED: Set Y-axis direction to be upward-increasing (Cartesian)
set(gca, 'YDir', 'normal');

% Initial title
title('\textbf{RRT* Path Planning with Smooth Curves}', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('x [m]', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('y [m]', 'Interpreter', 'latex', 'FontSize', 10);

% Plot start and goal points
h_start = plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
h_goal = plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Execute RRT*
[path, tree_x, tree_y, theta, num_iterations] = rrtStar_smooth(map, start, goal, params);

% Update title with iteration count
title({'\textbf{RRT* Path Planning with Smooth Curves}', ...
       ['\textbf{Iterations: ' num2str(num_iterations) '}']}, ...
       'Interpreter', 'latex', 'FontSize', 12);

% Process and plot results
if ~isempty(path)
    % Plot smooth path
    h_smooth = plot(path.points(:,1), path.points(:,2), 'm-', 'LineWidth', 2);
    
    % Add legend
    legend([h_start, h_goal, h_smooth], ...
           {'Start point', 'Goal point', 'Smoothed path'}, ...
           'Location', 'best', 'Interpreter', 'latex', 'FontSize', 9);
    
    % Print path information
    fprintf('Path found successfully!\n');
    fprintf('Path length: %.2f meters\n', sum(sqrt(sum(diff(path.points).^2, 2))));
    fprintf('Total iterations: %d\n', num_iterations);
    
    % Generate desired trajectory
    [xd, yd, thetad, phid, q0] = des_trajectory(path.points);
    
    % Save trajectory data
    save('trajectory_data.mat', 'xd', 'yd', 'thetad', 'phid', 'q0');
    fprintf('Trajectory data saved to trajectory_data.mat\n');
else
    % Add legend without path
    legend([h_start, h_goal], {'Start point', 'Goal point'}, ...
           'Location', 'best', 'Interpreter', 'latex', 'FontSize', 9);
    fprintf('No path found.\n');
    fprintf('Maximum iterations (%d) reached.\n', num_iterations);
end

grid on;