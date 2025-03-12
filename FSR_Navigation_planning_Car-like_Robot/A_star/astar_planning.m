function [path_points, sample_points, tree_data, path_node_count] = astar_planning(map, start, goal, params)
% ASTAR_PLANNING Performs path planning using A* algorithm with Reed-Shepp curves
%
% Inputs:
%   map - Binary map of the environment (1 = free space, 0 = obstacle)
%   start - Start coordinates [x, y]
%   goal - Goal coordinates [x, y]
%   params - Structure with algorithm parameters including:
%            - length_step: Length of the step forward
%            - radius_curve: Bending radius for curves
%            - heuristic_type: Type of heuristic (1=Euclidean, 2=Manhattan)
%
% Outputs:
%   path_points - Array of points along the smoothed path
%   sample_points - Array of all sample points generated
%   tree_data - Structure with tree information (points, connections, etc.)

% Initialize tree data structure
tree_data = struct();
tree_data.points = zeros(5000, 2);  % Preallocate for efficiency
tree_data.points(1, :) = start;
tree_data.counter = 1;
tree_data.curve_points = struct('start_node', [], 'end_node', [], 'points', {});
tree_data.sample_points = [];
tree_data.connections = [];
tree_data.goal_reached = false;
tree_data.legend = 1;

% Occupancy matrix - Cartesian coordinates [x, y] -> matrix indices [y, x]
tree_data.occupancy_matrix = zeros(size(map));
tree_data.occupancy_matrix(start(2), start(1)) = 1;  % Using (y, x) for matrix indexing
tree_data.occupancy_matrix(map == 0) = 1;

% Create figure for exploration
figure('Name', 'A* Reed-Shepp Path Planning');
imshow(map);  % Display the map with correct orientation
hold on;
set(gca, 'YDir', 'normal');  % Set Y-axis direction to be upward-increasing (Cartesian)

% Plot start and goal points with appropriate labels
plot(start(1), start(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Start point');
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Goal point');

% Queue for points to explore - [x, y, angle, g_cost, h_cost]
% Initialize with start point, 0 cost, and heuristic to goal
h_start = calculate_heuristic(start, goal, params.heuristic_type);
queue = [start, 0, 0, h_start];  % [x, y, angle, g_cost, h_cost]

% Exploration loop - explore paths from start point
while ~isempty(queue)
   % A* selects node with minimum f(n) = g(n) + h(n)
   f_values = queue(:, 4) + queue(:, 5);
   [~, min_idx] = min(f_values);
   
   current_point = queue(min_idx, 1:2);
   current_angle = queue(min_idx, 3);
   current_g_cost = queue(min_idx, 4);
   
   % Remove selected node from queue
   queue(min_idx, :) = [];
   
   % Check if we're close enough to goal
   if norm(current_point - goal) < params.length_step
       % We've reached close enough to the goal
       tree_data.sample_points = [tree_data.sample_points; goal];
       tree_data.connections = [tree_data.connections; current_point, goal];
       tree_data.goal_reached = true;
       break;
   end

   % Calculate new points and add them to the queue if not visited
   [new_points, tree_data] = calculate_points(current_point, current_angle, current_g_cost, goal, map, tree_data, params);
   queue = [queue; new_points];
end

% After exploration, check if any point is within range of the goal if goal not already reached
if ~tree_data.goal_reached
    close_points = find_close_points(tree_data.sample_points, goal, params.length_step);

    % Connect the goal to accessible points
    if ~isempty(close_points)
        for i = 1:size(close_points, 1)
            idx = close_points(i, 1);
            tree_data.sample_points = [tree_data.sample_points; goal];
            tree_data.connections = [tree_data.connections; tree_data.sample_points(idx,:), goal];
        end
        tree_data.goal_reached = true;
    else
        disp('Goal not reachable with current parameters.');
    end
end

% Set labels and title
title('A* with Reed-Shepp Primitive Curves');
xlabel('X [m]');
ylabel('Y [m]');
axis equal;
grid on;

% We'll update this with number of path nodes after path finding is complete

% Add start point to sample points
sample_points = [start; tree_data.sample_points];

% Create adjacency matrix and find shortest path
adjacency_matrix = create_adjacency_matrix(sample_points, tree_data.connections);
shortest_path = find_shortest_path(adjacency_matrix, sample_points, start, goal, params);

% Plot path and get smoothed path points
if ~isempty(shortest_path)
    path_points = plot_path(shortest_path, sample_points, 'm', tree_data);
    
    % Get statistics
    path_node_count = length(shortest_path);
    total_explored_nodes = tree_data.counter;
    
    % Get heuristic type name for display
    if params.heuristic_type == 1
        heuristic_name = 'Euclidean';
    elseif params.heuristic_type == 2
        heuristic_name = 'Manhattan';
    else
        heuristic_name = 'Unknown';
    end
    
    % Update the title to include both node counts and heuristic type
    title({'A* with Reed-Shepp Primitive Curves', ...
          ['Heuristic: ', heuristic_name], ...
          ['Path nodes: ', num2str(path_node_count), ...
           ' | Total explored nodes: ', num2str(total_explored_nodes)]});
else
    path_points = [];
    disp('No path found to goal');
end
end

function [new_points, tree_data] = calculate_points(current_point, current_angle, current_g_cost, goal, map, tree_data, params)
   % Calculate new potential points from current point with A* costs
   
   new_points = [];
   
   % Forward point
   point_forward = round(current_point + params.length_step * [cos(current_angle), sin(current_angle)]);
   if is_within_bounds(point_forward, map) && ~is_point_visited(point_forward, tree_data.occupancy_matrix)
       if ~is_collision_forward(current_point, point_forward, map)
           % Calculate g cost (cost from start) - straight movement cost
           g_cost_forward = current_g_cost + params.length_step;
           
           % Calculate h cost (estimated cost to goal) using heuristic
           h_cost_forward = calculate_heuristic(point_forward, goal, params.heuristic_type);
           
           [tree_data] = add_point(current_point, point_forward, current_angle, [], map, tree_data, params);
           new_points = [new_points; point_forward, current_angle, g_cost_forward, h_cost_forward];
       end
   end

   % Right turn
   center_right = current_point + params.radius_curve * [cos(current_angle - pi/2), sin(current_angle - pi/2)];
   theta_right = linspace(current_angle + pi/2, current_angle, 1001);
   x_right = center_right(1) + params.radius_curve * cos(theta_right);
   y_right = center_right(2) + params.radius_curve * sin(theta_right);
   end_point_right = round([x_right(end), y_right(end)]);
   
   if is_within_bounds(end_point_right, map) && ~is_point_visited(end_point_right, tree_data.occupancy_matrix)
       if ~is_collision_curve(x_right, y_right, map)
           % Calculate g cost for curved movement (approximated by arc length)
           g_cost_right = current_g_cost + (pi/2) * params.radius_curve;
           
           % Calculate h cost using heuristic
           h_cost_right = calculate_heuristic(end_point_right, goal, params.heuristic_type);
           
           [tree_data] = add_point(current_point, end_point_right, current_angle - pi/2, [x_right', y_right'], map, tree_data, params);
           new_points = [new_points; end_point_right, current_angle - pi/2, g_cost_right, h_cost_right];
       end
   end

   % Left turn
   center_left = current_point + params.radius_curve * [cos(current_angle + pi/2), sin(current_angle + pi/2)];
   theta_left = linspace(current_angle - pi/2, current_angle, 1001);
   x_left = center_left(1) + params.radius_curve * cos(theta_left);
   y_left = center_left(2) + params.radius_curve * sin(theta_left);
   end_point_left = round([x_left(end), y_left(end)]);
   
   if is_within_bounds(end_point_left, map) && ~is_point_visited(end_point_left, tree_data.occupancy_matrix)
       if ~is_collision_curve(x_left, y_left, map)
           % Calculate g cost for curved movement (approximated by arc length)
           g_cost_left = current_g_cost + (pi/2) * params.radius_curve;
           
           % Calculate h cost using heuristic
           h_cost_left = calculate_heuristic(end_point_left, goal, params.heuristic_type);
           
           [tree_data] = add_point(current_point, end_point_left, current_angle + pi/2, [x_left', y_left'], map, tree_data, params);
           new_points = [new_points; end_point_left, current_angle + pi/2, g_cost_left, h_cost_left];
       end
   end
end

function heuristic = calculate_heuristic(point, goal, heuristic_type)
   % A* heuristic function
   % heuristic_type: 1 = Euclidean distance, 2 = Manhattan distance
   
   if heuristic_type == 1
       % Euclidean distance heuristic
       heuristic = norm(point - goal);
   elseif heuristic_type == 2
       % Manhattan distance heuristic
       heuristic = abs(point(1) - goal(1)) + abs(point(2) - goal(2));
   else
       % Default to Euclidean if invalid type
       warning('Invalid heuristic type specified. Using Euclidean distance.');
       heuristic = norm(point - goal);
   end
end

function [tree_data] = add_point(current_point, end_point, angle, curve_data, map, tree_data, params)
   % Add a new point to the exploration tree
   
   tree_data.counter = tree_data.counter + 1;
   tree_data.points(tree_data.counter, :) = end_point;
   tree_data.sample_points = [tree_data.sample_points; end_point];
   
   if isempty(curve_data)
       % Straight line
       x_points = round(linspace(current_point(1), end_point(1), 300));
       y_points = round(linspace(current_point(2), end_point(2), 300));
       
       % Store in occupancy matrix with correct indexing (y, x)
       for i = 1:length(x_points)
           if is_within_bounds([x_points(i), y_points(i)], map)
               tree_data.occupancy_matrix(y_points(i), x_points(i)) = 1;
           end
       end
       plot([current_point(1), end_point(1)], [current_point(2), end_point(2)], 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
   else
       % Curve
       for i = 1:size(curve_data, 1)
           x = round(curve_data(i,1));
           y = round(curve_data(i,2));
           if is_within_bounds([x, y], map)
               tree_data.occupancy_matrix(y, x) = 1;
           end
       end
       plot(curve_data(:,1), curve_data(:,2), 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
       
       % Store curve points for later path reconstruction
       new_curve = struct('start_node', current_point, 'end_node', end_point, 'points', curve_data);
       tree_data.curve_points(end+1) = new_curve;
   end
   
   tree_data.connections = [tree_data.connections; current_point, end_point];
   plot(end_point(1), end_point(2), 'b.', 'MarkerSize', 10, 'HandleVisibility','off');
   
   if tree_data.legend == 1
       plot(end_point(1), end_point(2), 'b.', 'MarkerSize', 10, 'DisplayName', 'Nodes');
       tree_data.legend = 0;
   end
end

function visited = is_point_visited(point, occupancy_matrix)
   % Check if a point has already been visited
   visited = occupancy_matrix(point(2), point(1)) == 1;
end

function within_bounds = is_within_bounds(point, map)
   % Check if point is within map bounds, with correct x,y order
   within_bounds = point(1) > 0 && point(2) > 0 && ...
                  point(1) <= size(map, 2) && ... % width = columns = x
                  point(2) <= size(map, 1);       % height = rows = y
end

function collision_forward = is_collision_forward(current_point, end_point_forward, map)
   % Check for collisions along a straight line
   collision_forward = false;
   x_points = round(linspace(current_point(1), end_point_forward(1), 300));
   y_points = round(linspace(current_point(2), end_point_forward(2), 300));
   for i = 2:(length(x_points)-1)
       % Check map with correct indexing (y, x)
       if x_points(i) > 0 && y_points(i) > 0 && ...
          x_points(i) <= size(map, 2) && y_points(i) <= size(map, 1) && ...
          map(y_points(i), x_points(i)) == 0
           collision_forward = true;
           break
       end
   end
end

function collision_curve = is_collision_curve(x, y, map)
   % Check for collisions along a curve
   collision_curve = false;
   for i = 2:(length(x)-1)
       rx = round(x(i));
       ry = round(y(i));
       % Check map with correct indexing (y, x)
       if rx > 0 && ry > 0 && ...
          rx <= size(map, 2) && ry <= size(map, 1) && ...
          map(ry, rx) == 0
           collision_curve = true;
           break
       end
   end
end

function adjacency_matrix = create_adjacency_matrix(nodes, connections)
   % Create adjacency matrix for path finding algorithm
   num_nodes = size(nodes, 1);
   adjacency_matrix = inf(num_nodes, num_nodes); % Make sure it's square
   
   % Set diagonal to 0 (distance to self)
   for i = 1:num_nodes
       adjacency_matrix(i, i) = 0;
   end

   for i = 1:size(connections, 1)
       point_1 = connections(i, 1:2);
       point_2 = connections(i, 3:4);
       
       idx_1 = find(ismember(nodes, point_1, 'rows'), 1); % Use 1 to get just first match
       idx_2 = find(ismember(nodes, point_2, 'rows'), 1); % Use 1 to get just first match
       
       if ~isempty(idx_1) && ~isempty(idx_2)
           dist = norm(point_1 - point_2);
           adjacency_matrix(idx_1, idx_2) = dist;
           adjacency_matrix(idx_2, idx_1) = dist; % Ensure symmetry
       end
   end
end

function path = find_shortest_path(adjacency_matrix, sample_points, start, goal, params)
   % Find shortest path using A* algorithm
   
   % Find indices of start and goal points
   start_idx = find(ismember(sample_points, start, 'rows'));
   goal_idx = find(ismember(sample_points, goal, 'rows'));
   
   if isempty(goal_idx)
       error('Goal point not found in sample points.');
   end
   
   % Ensure indices are scalar
   if length(start_idx) > 1
       start_idx = start_idx(1);
   end
   
   if length(goal_idx) > 1
       goal_idx = goal_idx(1);
   end
   
   n = size(adjacency_matrix, 1); % Number of nodes
   
   % Initialize
   g_cost = inf(n, 1);         % Cost from start node
   g_cost(start_idx) = 0;
   f_cost = inf(n, 1);         % f(n) = g(n) + h(n)
   
   % Calculate initial f_cost using heuristic
   if isfield(params, 'heuristic_type')
       f_cost(start_idx) = calculate_heuristic(sample_points(start_idx,:), sample_points(goal_idx,:), params.heuristic_type);
   else
       % Default to Euclidean distance if heuristic_type is not provided
       f_cost(start_idx) = calculate_heuristic(sample_points(start_idx,:), sample_points(goal_idx,:), 1);
   end
   
   visited = false(n, 1);      % Track visited nodes
   previous = zeros(n, 1);     % Previous node in optimal path
   
   % Main loop
   while true
       % Find unvisited node with minimum f cost
       unvisited_costs = f_cost;
       unvisited_costs(visited) = inf;
       [min_cost, current] = min(unvisited_costs);
       
       % Check if we're done
       if isscalar(min_cost) && isscalar(current) && isscalar(goal_idx)
           if isinf(min_cost) || current == goal_idx
               break; % Done if we reach goal or no path exists
           end
       else
           % Handle non-scalar case
           if any(isinf(min_cost)) || any(current == goal_idx)
               break;
           end
       end
       
       % Mark as visited
       visited(current) = true;
       
       % Check all neighbors
       for neighbor = 1:n
           if ~visited(neighbor) && ~isinf(adjacency_matrix(current, neighbor))
               % Calculate new g cost
               tentative_g = g_cost(current) + adjacency_matrix(current, neighbor);
               
               if tentative_g < g_cost(neighbor)
                   % This path is better than any previous one
                   previous(neighbor) = current;
                   g_cost(neighbor) = tentative_g;
                   
                   % Update f_cost with heuristic
                   if isfield(params, 'heuristic_type')
                       h_cost = calculate_heuristic(sample_points(neighbor,:), sample_points(goal_idx,:), params.heuristic_type);
                   else
                       % Default to Euclidean distance if heuristic_type not provided
                       h_cost = calculate_heuristic(sample_points(neighbor,:), sample_points(goal_idx,:), 1);
                   end
                   f_cost(neighbor) = g_cost(neighbor) + h_cost;
               end
           end
       end
   end
   
   % Extract path (if exists)
   if isinf(g_cost(goal_idx))
       path = []; % No path found
       disp('No path found to goal');
   else
       % Reconstruct path
       path = goal_idx;
       while previous(path(1)) ~= 0
           path = [previous(path(1)), path];
       end
   end
end

function close_points = find_close_points(sample_points, goal, length_step)
   % Find points that are close to the goal
   close_points = [];
   for i = 1:size(sample_points, 1)
       if norm(sample_points(i,:) - goal) < length_step
           close_points = [close_points; i, norm(sample_points(i,:) - goal)];
       end
   end
end

function collected_points = plot_path(path, nodes, color, tree_data)
    % Plot the path and return collected points for trajectory generation
    path_nodes = nodes(path,:);
    
    % Collect all points in the path
    raw_points = [];
    for i = 1:length(path)-1
        start_node = path(i);
        end_node = path(i+1);
        
        % Check if there is a curve between these nodes
        curve_index = find_curve_index(nodes(start_node,:), nodes(end_node,:), tree_data.curve_points);
        if ~isempty(curve_index)
            curve = tree_data.curve_points(curve_index).points;
            if isempty(raw_points)
                raw_points = curve;
            else
                raw_points = [raw_points; curve(2:end, :)];
            end
        else
            % Simple straight line
            points = [nodes(start_node,:); nodes(end_node,:)];
            if isempty(raw_points)
                raw_points = points;
            else
                raw_points = [raw_points; points(2:end, :)];
            end
        end
    end
    
    % Apply smoothing with spline
    % First, get cumulative distance along the path for parameterization
    n_points = size(raw_points, 1);
    cum_dist = zeros(n_points, 1);
    for i = 2:n_points
        cum_dist(i) = cum_dist(i-1) + norm(raw_points(i,:) - raw_points(i-1,:));
    end
    
    % Normalize parameter to [0,1]
    t = cum_dist / cum_dist(end);
    
    % Create a finer parameter vector for smoother curve
    t_fine = linspace(0, 1, 3000);
    
    % Use cubic spline interpolation for smoothing
    x_spline = spline(t, raw_points(:,1), t_fine);
    y_spline = spline(t, raw_points(:,2), t_fine);
    
    % Combine into smooth path
    smooth_path = [x_spline', y_spline'];
    
    % Plot the original path points for reference
    plot(raw_points(:,1), raw_points(:,2), 'b.', 'MarkerSize', 5, 'HandleVisibility','off');
    
    % Plot the smoothed path with correct label
    plot(smooth_path(:,1), smooth_path(:,2), 'Color', color, 'LineWidth', 2, 'DisplayName', 'Smoothed path');
    
    % Add subtitle with path information
    path_node_count = length(path);
    subtitle(['Path nodes: ', num2str(path_node_count)]);
    
    % Create legend with correct order
    legend('Location', 'northeast');
    
    % Return the smoothed path
    collected_points = smooth_path;
end

function index = find_curve_index(start_node, end_node, curve_points)
    % Find the index of a curve connecting two nodes
    index = [];
    for i = 1:length(curve_points)
        curve = curve_points(i);
        if (isequal(curve.start_node, start_node) && isequal(curve.end_node, end_node)) || ...
           (isequal(curve.start_node, end_node) && isequal(curve.end_node, start_node))
            index = i;
            break;
        end
    end
end