clc; close all; clear all;

% Start timing for performance measurement
tic;

% Load the map and initialize parameters
load('image_map.mat');
start_point = [120, 30];
goal_point = [135, 400];
goal_radius = 5;         % Acceptable radius to consider goal reached
step_size = 10;           % Maximum step size for extending tree (eta)
neighbor_radius = 10;     % Radius for neighbor search in rewiring

% Initialize tree with start point
tree_x = start_point(1);
tree_y = start_point(2);
parent_index = -1;        % Array to store parent indices (-1 for root)
path_costs = 0;           % Array to store cost-to-come for each node
best_path_cost = Inf;     % Cost of best path found
goal_reached_index = -1;  % Index of node that reached goal
max_iterations = 1000;     % Maximum number of iterations

% Create figure and plot start/goal points
figure;
imshow(image_map);
hold on;
h1 = plot(start_point(2), start_point(1), '.', 'markersize', 24, 'color', [1 0 1]);
h2 = plot(goal_point(2), goal_point(1), '.', 'markersize', 24, 'color', [1 0.5 0]);

% Main RRT* loop
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
        
        % Find neighbors for rewiring
        distances_to_new = sqrt((tree_x - new_x).^2 + (tree_y - new_y).^2);
        neighbor_indices = find(distances_to_new <= neighbor_radius);
        
        % Choose best parent from neighbors
        [sorted_costs, sort_indices] = sort(path_costs(neighbor_indices) + distances_to_new(neighbor_indices));
        
        valid_parent_found = false;
        parent_search_idx = 1;
        while ~valid_parent_found && parent_search_idx <= length(sort_indices)
            candidate_parent_idx = sort_indices(parent_search_idx);
            if check_collision(tree_x(neighbor_indices(candidate_parent_idx)), new_x, ...
                             tree_y(neighbor_indices(candidate_parent_idx)), new_y, image_map)
                new_cost = sorted_costs(parent_search_idx);
                valid_parent_found = true;
            else
                parent_search_idx = parent_search_idx + 1;
            end
        end
        
        % Add node to tree if valid parent found
        if valid_parent_found
            % Plot new node and connection
            h3 = plot(new_y, new_x, '.', 'markersize', 3, 'color', [0 0 1]);
            h4 = plot([tree_y(neighbor_indices(candidate_parent_idx)) new_y], ...
                     [tree_x(neighbor_indices(candidate_parent_idx)) new_x], 'g-', 'LineWidth', 0.5);
            
            % Add node to tree
            tree_x = [tree_x new_x];
            tree_y = [tree_y new_y];
            parent_index = [parent_index neighbor_indices(candidate_parent_idx)];
            path_costs = [path_costs new_cost];
            current_node_idx = length(tree_x);
            
            % Rewiring step - check if new node provides better path to neighbors
            neighbors_to_rewire = find(new_cost + distances_to_new(neighbor_indices) < path_costs(neighbor_indices));
            for rewire_idx = 1:length(neighbors_to_rewire)
                node_idx = neighbor_indices(neighbors_to_rewire(rewire_idx));
                if node_idx <= length(tree_x) && check_collision(tree_x(node_idx), new_x, tree_y(node_idx), new_y, image_map)
                    parent_index(node_idx) = current_node_idx;
                    path_costs(node_idx) = new_cost + distances_to_new(node_idx);
                    plot([tree_y(node_idx) new_y], [tree_x(node_idx) new_x], 'g-', 'LineWidth', 0.5);
                end
            end
            
            % Check if goal reached
            dist_to_goal = sqrt((new_x-goal_point(1))^2 + (new_y-goal_point(2))^2);
            if dist_to_goal < goal_radius
                current_path_cost = new_cost + dist_to_goal;
                if current_path_cost < best_path_cost
                    best_path_cost = current_path_cost;
                    goal_reached_index = current_node_idx;
                end
            end
        end
    end
    title(['Iteration: ', num2str(iter)]);
    drawnow;
end

% Calculate total execution time
execution_time = toc;

% Final plot with results
if goal_reached_index > 0
    h5 = plot_optimal_path(tree_x, tree_y, parent_index, goal_reached_index, image_map);
    title(sprintf('Best Path Cost: %.2f - Total Iterations: %d - Time: %.2f s', ...
          best_path_cost, iter, execution_time));
    legend([h1, h2, h3, h4, h5], 'Start Point', 'Goal Point', 'RRT* Nodes', 'Tree Expansion', 'Optimal Path', 'Location', 'southoutside');
else
    title(sprintf('Path not found - Total Iterations: %d - Time: %.2f s', ...
          iter, execution_time));
    legend([h1, h2, h3, h4], 'Start Point', 'Goal Point', 'RRT* Nodes', 'Tree Expansion', 'Location', 'southoutside');
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

% Function to plot the optimal path from start to goal
function h = plot_optimal_path(tree_x, tree_y, parent_index, goal_idx, map)
    if goal_idx <= length(tree_x)
        current_idx = goal_idx;
        while current_idx > 0 && parent_index(current_idx) ~= -1 && current_idx <= length(parent_index)
            if check_collision(tree_x(parent_index(current_idx)), tree_x(current_idx), ...
                             tree_y(parent_index(current_idx)), tree_y(current_idx), map)
                h = plot([tree_y(parent_index(current_idx)) tree_y(current_idx)], ...
                        [tree_x(parent_index(current_idx)) tree_x(current_idx)], 'r-', 'linewidth', 2);
            end
            current_idx = parent_index(current_idx);
        end
    end
end