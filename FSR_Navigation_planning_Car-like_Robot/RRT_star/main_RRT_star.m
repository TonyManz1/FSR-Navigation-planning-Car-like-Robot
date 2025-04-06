% Main script for RRT* with smooth curves
clear all;
close all;
clc;

% Load map
load('image_map.mat');
map = image_map;

% Initialize parameters
params = struct();
params.length_step = 9;        % [m]
params.radius_curve = 9;       % [m]
params.max_iterations = 700;   
params.goal_radius = 3;        % [m]
params.neighbor_radius = 10;   % [m]
params.goal_bias = 0.15;       % Bias towards the goal

% Define start and goal points
start = [30 120];   
goal  = [400 135];  

% Initialize visualization
figure('Name', 'RRT* Path Planning', 'NumberTitle', 'off', 'Position', [100 100 800 600]);
imshow(map);
hold on;
% Set Y-axis direction to be upward-increasing (Cartesian)
set(gca, 'YDir', 'normal');

% Title and labels with standard font
title('RRT* Path Planning', 'FontSize', 12);
xlabel('x [m]', 'FontSize', 10);
ylabel('y [m]', 'FontSize', 10);

% Plot start and goal points
h_start = plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
h_goal = plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Execute RRT*
[path, tree_x, tree_y, theta, num_iterations] = rrtStar_smooth(map, start, goal, params);

% Update title with iteration count
title(['RRT* Path Planning - Iterations: ' num2str(num_iterations)], 'FontSize', 12);

% Process and plot results
if ~isempty(path)
    % Plot smooth path
    h_smooth = plot(path.points(:,1), path.points(:,2), 'm-', 'LineWidth', 3);
    set(gcf, 'Position', [100, 100, 1200, 600]);
    set(gca, 'Position', [0.1, 0.2, 0.8, 0.7]);
    
    % Add legend 
   leg = legend([h_start, h_goal, h_smooth], ...
       {'Start point', 'Goal point', 'Smoothed path'}, ...
       'Location', 'northeast', 'FontSize', 8);

    leg.Position = [0.8, 0.85, 0.15, 0.1];

    % Uncomment if you want to save the plots
    % save_path = 'D:\Automazione e Robotica\FSR\FSR_Navigation_planning_Car-like_Robot\Images\Control_simulations\RRT_star\Matlab';
    % filename = [save_path, '\path.pdf'];
    % exportgraphics(gcf, filename, 'Resolution', 300, 'ContentType', 'image');
    
    % Print path information
    fprintf('Path found successfully!\n');
    fprintf('Path length: %.2f meters\n', sum(sqrt(sum(diff(path.points).^2, 2))));
    fprintf('Total iterations: %d\n', num_iterations);
    
    % Generate desired trajectory
    [xd, yd, thetad, phid, q0] = des_trajectory(path.points, params.radius_curve);
    
    % Save trajectory data
    % save('trajectory_data_rrt_star.mat', 'xd', 'yd', 'thetad', 'phid', 'q0');
    % fprintf('Trajectory data saved to trajectory_data_rrt_star.mat\n');
else
    % Add legend without path
    legend([h_start, h_goal], {'Start point', 'Goal point'}, ...
           'Location', 'best', 'FontSize', 9);
    fprintf('No path found.\n');
    fprintf('Maximum iterations (%d) reached.\n', num_iterations);
end