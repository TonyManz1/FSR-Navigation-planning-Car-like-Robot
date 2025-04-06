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

% Initialize visualization
figure('Name', 'Dijkstra Path Planning', 'NumberTitle', 'off', 'Position', [100 100 800 600]);
imshow(map);
hold on;
% Set Y-axis direction to be upward-increasing (Cartesian)
set(gca, 'YDir', 'normal');

% Title and labels with standard font
title('Dijkstra Path Planning', 'FontSize', 12);
xlabel('x [m]', 'FontSize', 10);
ylabel('y [m]', 'FontSize', 10);

% Plot start and goal points
h_start = plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start point');
h_goal = plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal point');

% Execute Dijkstra algorithm 
[path_points, sample_points, tree_data, path_node_count] = dijkstra_planning(map, start, goal, params);

% Update title with path information
title(['Dijkstra Path Planning - Path nodes: ' num2str(path_node_count) ' | Total explored: ' num2str(tree_data.counter)], 'FontSize', 12);

% Process results
if ~isempty(path_points)
    % Plot path with thicker line
    h_smooth = plot(path_points(:,1), path_points(:,2), 'm-', 'LineWidth', 3, 'DisplayName', 'Smoothed path');
    
    % Adjust figure for better display
    set(gcf, 'Position', [100, 100, 1200, 600]);
    set(gca, 'Position', [0.1, 0.2, 0.8, 0.7]);
    
    % Add legend 
    leg = legend([h_start, h_goal, h_smooth], ...
        {'Start point', 'Goal point', 'Smoothed path'}, ...
        'Location', 'northeast', 'FontSize', 8);
    
    leg.Position = [0.8, 0.85, 0.15, 0.1];
    
    % Save the plot with proper legend
     % save_path = 'D:\Automazione e Robotica\FSR\FSR_Navigation_planning_Car-like_Robot\Images\Control_simulations\Dijkstra\Matlab'; 
     % filename = [save_path, '\dijkstra_path.pdf'];
     % exportgraphics(gcf, filename, 'Resolution', 300, 'ContentType', 'image');
    
    % Generate desired trajectory
    [xd, yd, thetad, phid, q0] = des_trajectory(path_points, params.radius_curve);
    
    % Save trajectory data
    %save('trajectory_data_dijkstra.mat', 'xd', 'yd', 'thetad', 'phid', 'q0');
    
    % Print path information
    fprintf('Path found successfully!\n');
    fprintf('Path length: %.2f meters\n', sum(sqrt(sum(diff(path_points).^2, 2))));
    fprintf('Total nodes: %d\n', path_node_count);
    fprintf('Total explored: %d\n', tree_data.counter);
    fprintf('Trajectory data saved to trajectory_data_dijkstra.mat\n');
else
    % Add legend without path
    legend([h_start, h_goal], {'Start point', 'Goal point'}, ...
           'Location', 'best', 'FontSize', 9);
    fprintf('No path found.\n');
end