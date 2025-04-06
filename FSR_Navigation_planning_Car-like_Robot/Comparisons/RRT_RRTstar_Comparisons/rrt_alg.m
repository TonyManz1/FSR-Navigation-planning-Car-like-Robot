clc; close all; clear all;

% Inizializzazione dell'algoritmo

% Load the map and initialize parameters
load('image_map.mat');
start_point = [120, 30];
goal_point = [135, 400];
goal_radius = 5;          % Acceptable radius to consider goal reached
step_size = 10;           % Maximum step size for extending tree (eta)

% Initialize tree with start point
tree_x = start_point(1);
tree_y = start_point(2);
parent_index = -1;        % Array to store parent indices (-1 for root)
goal_reached_index = -1;  % Index of node that reached goal
max_iterations = 3000;    % Maximum number of iterations

% Create figure and plot start/goal points
figure;
imshow(image_map);
hold on;
h1 = plot(start_point(2), start_point(1), '.', 'markersize', 24, 'color', [1 0 1]);
h2 = plot(goal_point(2), goal_point(1), '.', 'markersize', 24, 'color', [1 0.5 0]);

% Main RRT loop
for iter = 1:max_iterations
    % Generate random point (biased towards goal)
    if rand < 0.1  % 10% chance to sample goal directly
        random_x = goal_point(1);
        random_y = goal_point(2);
    else
        random_x = randi(size(image_map, 1));
        random_y = randi(size(image_map, 2));
    end
    
    % Find nearest node in tree
    distances_to_random = sqrt((random_x-tree_x).^2 + (random_y-tree_y).^2);
    [min_dist, nearest_index] = min(distances_to_random);
    
    % Check if path to random point is collision-free
    if check_collision(tree_x(nearest_index), random_x, tree_y(nearest_index), random_y, image_map)
        % Limit step size
        step_length = min(step_size, min_dist);
        k = step_length / min_dist;
        
        % Calculate new node position
        new_x = round((1-k)*tree_x(nearest_index) + k*random_x);
        new_y = round((1-k)*tree_y(nearest_index) + k*random_y);
        
        % Check if new node is valid and not already in tree
        if new_x >= 1 && new_x <= size(image_map, 1) && new_y >= 1 && new_y <= size(image_map, 2)
            if check_collision(tree_x(nearest_index), new_x, tree_y(nearest_index), new_y, image_map)
                % Plot new node and connection
                h3 = plot(new_y, new_x, '.', 'markersize', 3, 'color', [0 0 1]);
                h4 = plot([tree_y(nearest_index) new_y], [tree_x(nearest_index) new_x], 'g-', 'LineWidth', 0.5);
                
                % Add node to tree
                tree_x = [tree_x new_x];
                tree_y = [tree_y new_y];
                parent_index = [parent_index nearest_index];
                current_node_idx = length(tree_x);
                
                % Check if goal reached
                dist_to_goal = sqrt((new_x-goal_point(1))^2 + (new_y-goal_point(2))^2);
                if dist_to_goal < goal_radius && goal_reached_index == -1
                    goal_reached_index = current_node_idx;
                    % Non interrompiamo il ciclo, continuiamo per tutte le iterazioni
                end
            end
        end
    end
    
    title(['Iteration: ', num2str(iter)]);
    drawnow;
end

% Final plot with results
if goal_reached_index > 0
    h5 = plot_path(tree_x, tree_y, parent_index, goal_reached_index, image_map);
    title(sprintf('Path Found - Total Iterations: %d', iter));
    legend([h1, h2, h3, h4, h5], 'Start Point', 'Goal Point', 'RRT Nodes', 'Tree Expansion', 'Final Path', 'Location', 'southoutside');
else
    title(sprintf('Path not found - Total Iterations: %d', iter));
    legend([h1, h2, h3, h4], 'Start Point', 'Goal Point', 'RRT Nodes', 'Tree Expansion', 'Location', 'southoutside');
end

% Function to check if path between two points is collision-free
function valid = check_collision(x1, x2, y1, y2, map)
    % Generate points along line
    points = [linspace(x1, x2, 20)', linspace(y1, y2, 20)'];
    rounded_points = round(points);
    
    % Check if points are within map bounds
    valid = all(rounded_points(:,1) >= 1 & rounded_points(:,1) <= size(map,1) & ...
               rounded_points(:,2) >= 1 & rounded_points(:,2) <= size(map,2));
    
    % Check if path intersects obstacles
    if valid
        valid = all(map(sub2ind(size(map), rounded_points(:,1), rounded_points(:,2))));
    end
end

% Function to plot the path from start to goal
function h = plot_path(tree_x, tree_y, parent_index, goal_idx, map)
    path_x = [];
    path_y = [];
    current_idx = goal_idx;
    
    % Trace path from goal to start
    while current_idx > 0 && parent_index(current_idx) ~= -1
        path_x = [path_x, tree_x(current_idx)];
        path_y = [path_y, tree_y(current_idx)];
        current_idx = parent_index(current_idx);
    end
    
    % Add start point
    path_x = [path_x, tree_x(1)];
    path_y = [path_y, tree_y(1)];
    
    % Plot path
    h = plot(path_y, path_x, 'r-', 'linewidth', 2);
end