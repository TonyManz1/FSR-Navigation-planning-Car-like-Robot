clear; clc; close all;

global length_step radius_curve points counter occupancy_matrix map sample_points connections curve_points legend goal_reached
% Load map from a .mat file
load ("image_map.mat")
map_b = image_map;
map = double (map_b);

length_step = 9;            % Length of the step forward
radius_curve = 9;           % Bending radius for left and right curves
legend = 1;
curve_points = struct('start_node', [], 'end_node', [], 'points', {});
sample_points = [];
connections = [];
goal_reached = false;

% Define Cartesian coordinates for start and goal
% Changed from [row, col] to [col, row] to match Cartesian [x, y]
Qs = [30 120];  % [x, y]
Qg = [400 135]; % [x, y]

% Vector for points
points = zeros(5000, 2);  % Preallocato a un valore ragionevole
points(1, :) = Qs;

% Occupancy matrix - Cartesian coordinates [x, y] -> matrix indices [y, x]
occupancy_matrix = zeros(size(map));
occupancy_matrix(Qs(2), Qs(1)) = 1;  % Changed to (y, x) for matrix indexing         
occupancy_matrix(map == 0) = 1;       

% Point counter
counter = 1;

% Queue for points to explore
queue = [Qs, 0];                         % [x, y, angle]

figure;
imshow(map);  % Display the map with correct orientation
hold on;
set(gca, 'YDir', 'normal');  % FIX: Set Y-axis direction to be upward-increasing (Cartesian)
plot(Qs(1), Qs(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);                    
plot(Qg(1), Qg(2), 'go', 'MarkerSize', 10, 'LineWidth', 1.5);                    

% Exploration loop - explore paths from start
while ~isempty(queue)
   current_point = queue(1, 1:2);
   current_angle = queue(1, 3);
   queue(1, :) = [];                   

   % Calculate new points and add them to the queue if not visited
   new_points = calculate_points(current_point, current_angle);
   queue = [queue; new_points];
end

% After full exploration, check if any point is within range of the goal
close_points = [];
for i = 1:size(sample_points, 1)
    if norm(sample_points(i,:) - Qg) < length_step
        close_points = [close_points; i, norm(sample_points(i,:) - Qg)];
    end
end

% Connect the goal to accessible points
if ~isempty(close_points)
    for i = 1:size(close_points, 1)
        idx = close_points(i, 1);
        sample_points = [sample_points; Qg];
        connections = [connections; sample_points(idx,:), Qg];
    end
    goal_reached = true;
else
    disp('Goal not reachable with current parameters.');
end

title('Primitive Reed-Sheep Curves Iterative');
xlabel('X');
ylabel('Y');
axis equal;
grid on;

sample_points = [Qs; sample_points];
adjacency_matrix = create_adjacency_matrix(sample_points, connections);

shortest_path = dijkstra(adjacency_matrix, sample_points, Qs, Qg);

collected_points = plot_path(shortest_path, sample_points, 'm');

[xd, yd, thetad, phid, q0] = des_trajectory(collected_points);

function new_points = calculate_points(current_point, current_angle)
   global length_step radius_curve points counter occupancy_matrix sample_points connections curve_points legend Qg

   new_points = [];
   
   % Forward point
   point_forward = round(current_point + length_step * [cos(current_angle), sin(current_angle)]);
   if is_within_bounds(point_forward) && ~is_point_visited(point_forward)
       if ~is_collision_forward(current_point, point_forward)
           add_point(current_point, point_forward, current_angle, []);
           new_points = [new_points; point_forward, current_angle];
       end
   end

   % Right turn
   center_right = current_point + radius_curve * [cos(current_angle - pi/2), sin(current_angle - pi/2)];
   theta_right = linspace(current_angle + pi/2, current_angle, 1001);
   x_right = center_right(1) + radius_curve * cos(theta_right);
   y_right = center_right(2) + radius_curve * sin(theta_right);
   end_point_right = round([x_right(end), y_right(end)]);
   
   if is_within_bounds(end_point_right) && ~is_point_visited(end_point_right)
       if ~is_collision_curve(x_right, y_right)
           add_point(current_point, end_point_right, current_angle - pi/2, [x_right', y_right']);
           new_points = [new_points; end_point_right, current_angle - pi/2];
       end
   end

   % Left turn
   center_left = current_point + radius_curve * [cos(current_angle + pi/2), sin(current_angle + pi/2)];
   theta_left = linspace(current_angle - pi/2, current_angle, 1001);
   x_left = center_left(1) + radius_curve * cos(theta_left);
   y_left = center_left(2) + radius_curve * sin(theta_left);
   end_point_left = round([x_left(end), y_left(end)]);
   
   if is_within_bounds(end_point_left) && ~is_point_visited(end_point_left)
       if ~is_collision_curve(x_left, y_left)
           add_point(current_point, end_point_left, current_angle + pi/2, [x_left', y_left']);
           new_points = [new_points; end_point_left, current_angle + pi/2];
       end
   end
end

function add_point(current_point, end_point, angle, curve_data)
   global points counter occupancy_matrix sample_points connections curve_points legend
   
   counter = counter + 1;
   points(counter, :) = end_point;
   sample_points = [sample_points; end_point];
   
   if isempty(curve_data)
       x_points = round(linspace(current_point(1), end_point(1), 300));
       y_points = round(linspace(current_point(2), end_point(2), 300));
       % Store in occupancy matrix with correct indexing (y, x)
       for i = 1:length(x_points)
           if is_within_bounds([x_points(i), y_points(i)])
               occupancy_matrix(y_points(i), x_points(i)) = 1;
           end
       end
       plot([current_point(1), end_point(1)], [current_point(2), end_point(2)], 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
   else
       % Store curve points in occupancy matrix with correct indexing
       for i = 1:size(curve_data, 1)
           x = round(curve_data(i,1));
           y = round(curve_data(i,2));
           if is_within_bounds([x, y])
               occupancy_matrix(y, x) = 1;
           end
       end
       plot(curve_data(:,1), curve_data(:,2), 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
       curve_points(end+1).start_node = current_point;
       curve_points(end).end_node = end_point;
       curve_points(end).points = curve_data;
   end
   
   connections = [connections; current_point, end_point];
   plot(end_point(1), end_point(2), 'b.', 'MarkerSize', 10, 'HandleVisibility','off');
   
   if legend == 1
       plot(end_point(1), end_point(2), 'b.', 'MarkerSize', 10, 'DisplayName', 'Nodes');
       legend = 0;
   end
end

function visited = is_point_visited(point)
   global occupancy_matrix
   % Check with correct indexing (y, x) for matrix
   visited = occupancy_matrix(point(2), point(1)) == 1;
end

function within_bounds = is_within_bounds(point)
   global occupancy_matrix
   % Check if point is within map bounds, with correct x,y order
   within_bounds = point(1) > 0 && point(2) > 0 && ...
                  point(1) <= size(occupancy_matrix, 2) && ... % width = columns = x
                  point(2) <= size(occupancy_matrix, 1);       % height = rows = y
end

function collision_forward = is_collision_forward(current_point, end_point_forward)
   global map
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

function collision_curve = is_collision_curve(x, y)
   global map
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

function path = dijkstra(adjacency_matrix, sample_points, Qs, Qg)
   % Find indices of start and goal points
   start_idx = find(ismember(sample_points, Qs, 'rows'));
   goal_idx = find(ismember(sample_points, Qg, 'rows'));
   
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
   dist = inf(n, 1);         % Distance from start node
   dist(start_idx) = 0;
   visited = false(n, 1);    % Track visited nodes
   previous = zeros(n, 1);   % Previous node in optimal path
   
   % Main loop
   while true
       % Find unvisited node with minimum distance
       unvisited_distances = dist;
       unvisited_distances(visited) = inf;
       [min_dist, current] = min(unvisited_distances);
       
       % Check if we're done - ensure scalar comparison
       if isscalar(min_dist) && isscalar(current) && isscalar(goal_idx)
           if isinf(min_dist) || current == goal_idx
               break; % Done if we reach goal or no path exists
           end
       else
           % Handle non-scalar case
           if any(isinf(min_dist)) || any(current == goal_idx)
               break;
           end
       end
       
       % Mark as visited
       visited(current) = true;
       
       % Check all neighbors
       for neighbor = 1:n
           if ~visited(neighbor) && ~isinf(adjacency_matrix(current, neighbor))
               alt_dist = dist(current) + adjacency_matrix(current, neighbor);
               if alt_dist < dist(neighbor)
                   dist(neighbor) = alt_dist;
                   previous(neighbor) = current;
               end
           end
       end
   end
   
   % Extract path (if exists)
   if isinf(dist(goal_idx))
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

function collected_points = plot_path(path, nodes, color)
    global curve_points;
    path_nodes = nodes(path,:);
    
    % Collect all points in the path
    raw_points = [];
    for i = 1:length(path)-1
        start_node = path(i);
        end_node = path(i+1);
        
        % Check if there is a curve between these nodes
        curve_index = find_curve_index(nodes(start_node,:), nodes(end_node,:));
        if ~isempty(curve_index)
            curve = curve_points(curve_index).points;
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
    
    % Plot the smoothed path
    plot(smooth_path(:,1), smooth_path(:,2), 'Color', color, 'LineWidth', 2, 'DisplayName', 'Smoothed path');
    
    % Highlight start and end points
    plot(raw_points(1,1), raw_points(1,2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Start point');
    plot(raw_points(end,1), raw_points(end,2), 'go', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Goal point');
    
    % Show legend senza data1 e data2
    leg = legend('Nodes', 'Smoothed path', 'Start point', 'Goal point');
    set(leg, 'Interpreter','latex', 'fontsize', 9,'Location', 'best');
    
    % Return the smoothed path
    collected_points = smooth_path;
end

function index = find_curve_index(start_node, end_node)
    global curve_points;
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

function [xd, yd, thetad, phid, q0] = des_trajectory(collected_points)
    global map

    num_rows_map = size(map,1);  
    l = 0.6;                     % Wheelbase length
    tf = 600;                    % Final time in seconds
    v_max = 3;                   % Maximum linear velocity
    omega_max = 0.43;            % Maximum angular velocity
    eps = 1e-6;                  % Small value to prevent division by zero

    % Extract x and y coordinates from collected_points directly
    % No coordinate transform needed since we're using proper Cartesian coordinates
    x = collected_points(:, 1);
    y = collected_points(:, 2);

    % Define time vector
    t = linspace(0, tf, length(x));         % Normalize time from 0 to tf

    % Fit a 7-th order polynomial to the arc length s as a function of t
    % Polyfit to fit a 7-th order polynomial to a set of sample s values
    s_sample = linspace(0, 1, length(t));   % Sample s values
    [p,~,mu] = polyfit(t, s_sample, 7);     % Fit polynomial with mu for centring and scaling

    % Generate a finer time grid for interpolation
    T = linspace(0, tf, 10000);             % Increased number of points for interpolation

    % Compute interpolated s values using the 7-th order polynomial
    s_t = polyval(p, T, [], mu);

    % Interpolate x and y values using spline interpolation based on s_t with smoothing
    x_t = smooth(interp1(s_sample, x, s_t, 'spline'), 100);
    y_t = smooth(interp1(s_sample, y, s_t, 'spline'), 100);

    % Compute derivatives using gradient with smoothing
    dt = mean(diff(T));
    x_dot = smooth(gradient(x_t, dt), 50);
    y_dot = smooth(gradient(y_t, dt), 50);
    x_ddot = smooth(gradient(x_dot, dt), 50);
    y_ddot = smooth(gradient(y_dot, dt), 50);
    x_dddot = smooth(gradient(x_ddot, dt), 50);
    y_dddot = smooth(gradient(y_ddot, dt), 50);

    % Compute theta and linear velocity
    theta = atan2(y_dot, x_dot);
    v = sqrt(x_dot.^2 + y_dot.^2);
    
    % Ensure minimum velocity to prevent divisions by very small numbers
    v = max(v, eps);
    
    % Compute curvature and steering angle
    curvature = (y_ddot .* x_dot - x_ddot .* y_dot) ./ (v.^3 + eps);
    phi = atan(l * curvature);
    
    % Limit steering angle
    phi = min(max(phi, -pi/4), pi/4);
    
    % Compute angular velocity with safeguards
    omega = zeros(size(T));
    for i = 1:length(T)
        if v(i) > eps
            num = (y_dddot(i) * x_dot(i) - x_dddot(i) * y_dot(i)) * v(i)^2 - ...
                  3 * (y_ddot(i) * x_dot(i) - x_ddot(i) * y_dot(i)) * ...
                  (x_dot(i) * x_ddot(i) + y_dot(i) * y_ddot(i));
            den = v(i)^6 + l^2 * (y_ddot(i) * x_dot(i) - x_ddot(i) * y_dot(i))^2;
            
            if abs(den) > eps
                omega(i) = l * v(i) * num / den;
                % Limit angular velocity
                omega(i) = min(max(omega(i), -omega_max), omega_max);
            end
        end
    end
    
    % Apply additional smoothing to omega
    omega = smooth(omega, 200);
    
    % Final limiting of velocities
    v = min(max(v, -v_max), v_max);
    omega = min(max(omega, -omega_max), omega_max);

    % Check velocity bounds
    fprintf('Velocity bounds check:\n');
    fprintf('Max linear velocity: %.2f m/s (limit: %.2f)\n', max(abs(v)), v_max);
    fprintf('Max angular velocity: %.2f rad/s (limit: %.2f)\n', max(abs(omega)), omega_max);

    % Create timeseries objects
    xd = timeseries(x_t, T);
    yd = timeseries(y_t, T);
    thetad = timeseries(theta, T);
    phid = timeseries(phi, T);

    % Initial conditions
    xd0 = x_t(1);
    yd0 = y_t(1);
    thetad0 = theta(1);
    phid0 = phi(1);
    q0 = [xd0, yd0, thetad0, phid0]';

    % Visualization of the generated trajectory - exactly 3 figures as requested
    % Figure 1: Trajectory plot
    figure('Name', 'Trajectory');
    plot(x, y, 'o', 'MarkerSize', 5, 'DisplayName', 'Path points');
    hold on;
    set(gca, 'YDir', 'normal');  % Set Y-axis direction to be upward-increasing (Cartesian)
    plot(x_t, y_t, 'm-', 'LineWidth', 3, 'DisplayName', 'Generated Trajectory');
    plot(x(1), y(1), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Start point');
    plot(x(end), y(end), 'go', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Goal point');
    title('Trajectory', 'FontSize', 12);
    xlabel('x [m]', 'FontSize', 10);
    ylabel('y [m]', 'FontSize', 10);
    legend('Path points', 'Generated Trajectory', 'Start point', 'Goal point', 'Location', 'best');
    grid on;
    
    % Figure 2: Heading and Angular Velocity
    figure('Name', 'Velocities');
    subplot(2,1,1);
    plot(T, v, 'LineWidth', 2, 'Color', 'b');
    hold on;
    yline(v_max, 'r--', 'LineWidth', 1.5);
    yline(-v_max, 'r--', 'LineWidth', 1.5);
    title('Linear Velocity', 'FontSize', 12);
    xlabel('t [s]', 'FontSize', 10);
    ylabel('v [m/s]', 'FontSize', 10);
    xlim([0 tf]);
    ylim([-3 3]);
    grid on;
    
    subplot(2,1,2);
    plot(T, omega, 'LineWidth', 2, 'Color', 'b');
    hold on;
    yline(omega_max, 'r--', 'LineWidth', 1.5);
    yline(-omega_max, 'r--', 'LineWidth', 1.5);
    title('Angular Velocity', 'FontSize', 12);
    xlabel('t [s]', 'FontSize', 10);
    ylabel('\omega [rad/s]', 'FontSize', 10);
    xlim([0 tf]);
    ylim([-0.5 0.5]);
    grid on;

    % Figure 3: Evolution of states
    figure('Name', 'State Evolution');
    subplot(4,1,1);
    plot(T, x_t, 'b', 'LineWidth', 2);
    title('Evolution of x_d(t)', 'FontSize', 12);
    ylabel('x_d [m]', 'FontSize', 10);
    xlim([0 tf]);
    grid on;

    subplot(4,1,2);
    plot(T, y_t, 'r', 'LineWidth', 2);
    title('Evolution of y_d(t)', 'FontSize', 12);
    ylabel('y_d [m]', 'FontSize', 10);
    xlim([0 tf]);
    grid on;

    subplot(4,1,3);
    plot(T, theta, 'g', 'LineWidth', 2);
    title('Evolution of \theta_d(t)', 'FontSize', 12);
    ylabel('\theta_d [rad]', 'FontSize', 10);
    xlim([0 tf]);
    grid on;

    subplot(4,1,4);
    plot(T, phi, 'k', 'LineWidth', 2);
    title('Evolution of \phi_d(t)', 'FontSize', 12);
    xlabel('t [s]', 'FontSize', 10);
    ylabel('\phi_d [rad]', 'FontSize', 10);
    xlim([0 tf]);
    grid on;
end