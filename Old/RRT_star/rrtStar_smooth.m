function [path, tree_x, tree_y, theta, num_iterations] = rrtStar_smooth(map, start, goal, params)
  
    tic 

    % Initialize tree with start point
    tree_x = start(1); 
    tree_y = start(2); 
    theta = 0;  % Initial orientation
    parent_index = 0;
    costs = 0;
    tree_segments = {};
    num_iterations = 0;
    
    % Initialize best path variables
    best_path = [];
    best_cost = inf;
    goal_reached = false;
    
    % Debug variables
    debug_goal_checks = 0;
    debug_path_updates = 0;
    
    for iter = 1:params.max_iterations
        num_iterations = iter;
        
        % Sample random point with goal bias
        if rand < params.goal_bias
            rand_point = goal;
        else
            rand_point = [randi(size(map,2)), randi(size(map,1))];
        end
        
        % Find nearest node
        distances = sqrt((tree_x - rand_point(1)).^2 + (tree_y - rand_point(2)).^2);
        [~, nearest_idx] = min(distances);
        
        % Steer towards random point with smooth curves
        [x_new, theta_new, path_points] = steer([tree_x(nearest_idx), tree_y(nearest_idx)], ...
                                 theta(nearest_idx), rand_point, ...
                                 params.length_step, params.radius_curve);
        % Check if new point is valid
        if ~checkCollision([tree_x(nearest_idx), tree_y(nearest_idx)], ...
                          x_new, map, 'curve', params.radius_curve, path_points)
            % Find neighbors
            neighbors = findNeighbors(tree_x, tree_y, x_new, params.neighbor_radius);
            
            % Find best parent
            min_cost = inf;
            best_parent = nearest_idx;
            best_path_points = path_points;
            
            for i = 1:length(neighbors)
                potential_parent = neighbors(i);
                [~, ~, potential_path] = steer([tree_x(potential_parent), tree_y(potential_parent)], ...
                                             theta(potential_parent), x_new, ...
                                             params.length_step, params.radius_curve);
                
                new_cost = costs(potential_parent) + computePathLength(potential_path);
                
                if new_cost < min_cost && ...
                   ~checkCollision([tree_x(potential_parent), tree_y(potential_parent)], ...
                                 x_new, map, 'curve', params.radius_curve, potential_path)
                    min_cost = new_cost;
                    best_parent = potential_parent;
                    best_path_points = potential_path;
                end
            end
            
            % Add node to tree
            new_idx = length(tree_x) + 1;
            tree_x = [tree_x; x_new(1)];
            tree_y = [tree_y; x_new(2)];
            theta = [theta; theta_new];
            parent_index = [parent_index; best_parent];
            costs = [costs; min_cost];
            
            % Store the tree segment
            tree_segments{end+1} = best_path_points;
            
            % Plot the tree segment with continuous curve
            plot(best_path_points(:,1), best_path_points(:,2), 'g-', 'LineWidth', 0.5);
            drawnow;
         
% Check if this node reaches the goal
if norm([x_new(1) - goal(1), x_new(2) - goal(2)]) < params.goal_radius
    goal_reached = true;
    debug_goal_checks = debug_goal_checks + 1;
    current_path_indices = extractPath(parent_index, new_idx);
    
    % Calculate total cost for this path
    current_cost = costs(new_idx);
    
    % Sempre aggiorna il path la prima volta che raggiungiamo il goal
    if current_cost < best_cost || isempty(best_path)
        debug_path_updates = debug_path_updates + 1;
        best_cost = current_cost;
        path_segments = [];
        
        % Costruisci il path direttamente dai segmenti dell'albero
        for i = 1:length(current_path_indices)-1
            start_idx = current_path_indices(i);
            end_idx = current_path_indices(i+1);
            
            % Genera un nuovo segmento di path se necessario
            [~, ~, new_segment] = steer([tree_x(start_idx), tree_y(start_idx)], ...
                                      theta(start_idx), ...
                                      [tree_x(end_idx), tree_y(end_idx)], ...
                                      params.length_step, ...
                                      params.radius_curve);
            
            % Aggiungi il segmento al path
            if isempty(path_segments)
                path_segments = new_segment;
            else
                % Evita duplicati ai punti di giunzione
                path_segments = [path_segments; new_segment(2:end,:)];
            end
        end
        
        % Aggiungi l'ultimo segmento fino al goal
        [~, ~, final_segment] = steer([tree_x(current_path_indices(end)), tree_y(current_path_indices(end))], ...
                                    theta(current_path_indices(end)), ...
                                    goal, ...
                                    params.length_step, ...
                                    params.radius_curve);
        
        path_segments = [path_segments; final_segment(2:end,:)];
        
        % Crea il path
        if ~isempty(path_segments)
            best_path = struct('points', path_segments);
            fprintf('One path has been found: (Segments: %d)\n', ...
                     size(path_segments, 1));
        end
    end
end
            % Rewire neighbors
            for i = 1:length(neighbors)
                neighbor = neighbors(i);
                if neighbor ~= best_parent  % Avoid rewiring to the parent
                    [~, ~, rewire_path] = steer(x_new, theta_new, ...
                                              [tree_x(neighbor), tree_y(neighbor)], ...
                                              params.length_step, params.radius_curve);
                    
                    potential_cost = min_cost + computePathLength(rewire_path);
                    
                    if potential_cost < costs(neighbor) && ...
                       ~checkCollision(x_new, [tree_x(neighbor), tree_y(neighbor)], ...
                                     map, 'curve', params.radius_curve, rewire_path)
                        % Update parent and cost
                        parent_index(neighbor) = new_idx;
                        costs(neighbor) = potential_cost;
                        
                        % Update tree segments
                        tree_segments{end+1} = rewire_path;
                        plot(rewire_path(:,1), rewire_path(:,2), 'g-', 'LineWidth', 0.5);
                        drawnow;
                    end
                end
            end
        end
    end
    
    % Calculate execution time
    rrt_time = toc;
   
    % Print debug information
    fprintf('RRT* Execution Time: %.2f seconds\n', rrt_time);
    fprintf('Goal checks: %d\n', debug_goal_checks);
    fprintf('Path updates: %d\n', debug_path_updates);
    
    % Return the best path found
    if goal_reached && ~isempty(best_path) && size(best_path.points, 1) >= 2
        fprintf('Returning valid path with %d points\n', size(best_path.points, 1));
        path = best_path;
    else
        fprintf('No valid path found. Goal reached: %d, Path empty: %d\n', ...
                goal_reached, isempty(best_path));
        path = [];
    end
end

function [x_new, theta_new, path_points] = steer(current_point, current_angle, rand_point, length_step, radius_curve)
    % Calculate target angle and total distance
    target_direction = atan2(rand_point(2) - current_point(2), rand_point(1) - current_point(1));
    total_distance = norm(rand_point - current_point);
    
    % Limit the maximum distance to prevent long straight lines
    max_distance = length_step * 2;  % Reduced maximum distance
    if total_distance > max_distance
        % Scale down the target point to be closer
        scale_factor = max_distance / total_distance;
        rand_point = current_point + (rand_point - current_point) * scale_factor;
        total_distance = max_distance;
    end

    % Parameters for curve generation
    num_points = 150;  % Reduced for shorter segments
    max_curve_angle = pi/2;  
    min_straight_length = radius_curve * 0.8;  % Reduced for shorter straight segments

    % First curve generation
    angle_diff = wrapToPi(target_direction - current_angle);
    turn_angle = min(max_curve_angle, abs(angle_diff));
    turn_direction = sign(angle_diff);

    % Generate clothoid-like transition with shorter segments
    s = linspace(0, turn_angle, num_points);
    curve1_points = zeros(num_points, 2);
    for i = 1:num_points
        % Modified radius calculation for shorter segments
        local_radius = radius_curve / (1 + abs(s(i))/(pi/3));
        
        % Reduce the effect of the radius for shorter curves
        if total_distance < length_step * 1.5
            local_radius = local_radius * 0.7;
        end
        
        center = current_point + local_radius * [cos(current_angle + turn_direction*pi/2), ...
                                               sin(current_angle + turn_direction*pi/2)];
        theta = current_angle + turn_direction * s(i);
        curve1_points(i,:) = center + local_radius * [cos(theta - turn_direction*pi/2), ...
                                                     sin(theta - turn_direction*pi/2)];
    end

    % Smooth transition with smaller window
    path_points = smoothTransition(curve1_points, 2);

    % Generate shorter approach curve
    final_points = generateSmoothApproach(path_points(end,:), theta, rand_point, radius_curve * 0.6, num_points);

    % Combine points
    path_points = [path_points; final_points];

    % Final smoothing with reduced window
    path_points = smoothPath(path_points, 2);

    % Return final position and orientation
    x_new = path_points(end,:);
    theta_new = atan2(diff(path_points(end-1:end,2)), diff(path_points(end-1:end,1)));
end


function smooth_points = smoothTransition(points, window_size)
    smooth_points = points;
    for i = 1:size(points,1)
        start_idx = max(1, i-window_size);
        end_idx = min(size(points,1), i+window_size);
        smooth_points(i,:) = mean(points(start_idx:end_idx,:));
    end
end


function approach_points = generateSmoothApproach(start_point, start_angle, target_point, radius, num_points)
    dx = target_point(1) - start_point(1);
    dy = target_point(2) - start_point(2);
    dist = norm([dx, dy]);
    
    % Adjust control points for shorter segments
    p0 = start_point;
    p1 = start_point + [cos(start_angle), sin(start_angle)] * dist/5;  % Reduced from dist/4
    p2 = target_point - [cos(start_angle), sin(start_angle)] * dist/5;  % Reduced from dist/4
    p3 = target_point;
    
    t = linspace(0, 1, num_points);
    approach_points = zeros(num_points, 2);
    for i = 1:num_points
        tt = t(i);
        approach_points(i,:) = (1-tt)^3 * p0 + ...
                             3*(1-tt)^2 * tt * p1 + ...
                             3*(1-tt) * tt^2 * p2 + ...
                             tt^3 * p3;
    end
end

function smooth_path = smoothPath(path, window_size)
    smooth_path = path;
    for pass = 1:3  % Multiple smoothing passes
        for i = 1:size(path,1)
            start_idx = max(1, i-window_size);
            end_idx = min(size(path,1), i+window_size);
            smooth_path(i,:) = mean(path(start_idx:end_idx,:));
        end
        path = smooth_path;
    end
end

function collision = checkCollision(current_point, new_point, map, type, radius_curve, path_points)
    % If path_points is provided, use those for collision checking
    if nargin > 5 && ~isempty(path_points)
        check_points = round(path_points);
    else
        if strcmp(type, 'curve')
            check_points = path_points;
        else
            x_points = round(linspace(current_point(1), new_point(1), 100));
            y_points = round(linspace(current_point(2), new_point(2), 100));
            check_points = [x_points', y_points'];
        end
    end
    
    % Check bounds and collisions
    collision = false;
    for i = 1:size(check_points, 1)
        x = round(check_points(i,1));
        y = round(check_points(i,2));
        
        % Check bounds
        if x < 1 || x > size(map,2) || y < 1 || y > size(map,1)
            collision = true;
            break;
        end
        
        % Check collision with obstacles
        if map(y,x) == 0
            collision = true;
            break;
        end
    end
end

function neighbors = findNeighbors(tree_x, tree_y, point, radius)
    distances = sqrt((tree_x - point(1)).^2 + (tree_y - point(2)).^2);
    neighbors = find(distances <= radius);
end

function path_indices = extractPath(parent_index, goal_idx)
    path_indices = goal_idx;
    current_idx = goal_idx;
    
    while parent_index(current_idx) ~= 0
        current_idx = parent_index(current_idx);
        path_indices = [current_idx, path_indices];
    end
end

function length = computePathLength(path)
    diff_points = diff(path);
    length = sum(sqrt(sum(diff_points.^2, 2)));
end