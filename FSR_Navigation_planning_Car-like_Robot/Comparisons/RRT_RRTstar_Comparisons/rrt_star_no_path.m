clc; close all; clear all;

% Load the map and initialize parameters
load('image_map.mat');
start_point = [120, 30];
step_size = 10;           % Maximum step size for extending tree (eta)
neighbor_radius = 10;     % Radius for neighbor search in rewiring

% Initialize tree with start point
tree_x = start_point(1);
tree_y = start_point(2);
parent_index = -1;        % Array to store parent indices (-1 for root)
path_costs = 0;           % Array to store cost-to-come for each node
max_iterations = 3000;    % Maximum number of iterations

% Initialize visualization
figure('Name', 'RRT* Tree Expansion', 'NumberTitle', 'off', 'Position', [100 100 800 600]);
imshow(image_map);
hold on;
% Set Y-axis direction to be upward-increasing (Cartesian)
set(gca, 'YDir', 'normal');

% Title and labels with standard font
title('RRT* Tree Expansion', 'FontSize', 12);
xlabel('x [m]', 'FontSize', 10);
ylabel('y [m]', 'FontSize', 10);

% Plot start point
h1 = plot(start_point(2), start_point(1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

% Main RRT* loop - only tree expansion
for iter = 1:max_iterations
    % Generate random point
    random_x = randi(size(image_map, 1));
    random_y = randi(size(image_map, 2));
    
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
            plot(new_y, new_x, '.', 'markersize', 3, 'color', [0 0 1]);
            plot([tree_y(neighbor_indices(candidate_parent_idx)) new_y], ...
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
        end
    end
    
    if mod(iter, 500) == 0
        title(['RRT* - Iterazione: ', num2str(iter)]);
        drawnow;
    end
end

% Final title
title(['RRT* Tree Expansion - Iterations: ', num2str(iter)], 'FontSize', 12);

% Add legend
leg = legend(h1, 'Start point', 'Location', 'northeast', 'FontSize', 8);
leg.Position = [0.8, 0.85, 0.15, 0.1];

% Set final figure properties
set(gcf, 'Position', [100, 100, 1200, 600]);
set(gca, 'Position', [0.1, 0.2, 0.8, 0.7]);

% Save figure as PDF
% Uncomment the following lines to save the figure
 save_path = ''; % Set your path here if needed
 filename = 'rrt_star_tree_expansion.pdf';
 exportgraphics(gcf, filename, 'Resolution', 300, 'ContentType', 'image');

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