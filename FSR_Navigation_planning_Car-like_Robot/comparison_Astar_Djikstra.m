% Comparison script between Dijkstra and A* with different heuristics
clear all;
close all;
clc;

% Define folder paths
dijkstra_folder = 'D:\Automazione e Robotica\FSR\Project_FSR\Dijkstra';
astar_folder = 'D:\Automazione e Robotica\FSR\Project_FSR\A_star';

% Add paths to MATLAB path
addpath(dijkstra_folder);
addpath(astar_folder);

% Load map (assuming it's in the current directory or in one of the added paths)
load("image_map.mat");
map = double(image_map);

% Initialize parameters
params = struct();
params.length_step = 9;      % Length of the step forward
params.radius_curve = 9;     % Bending radius for left and right curves

% Define start and goal points
start = [30 120];   % Coordinates in format (x,y)
goal = [400 135];  % Coordinates in format (x,y)

% Variables to store results
path_points_dijkstra = [];
tree_data_dijkstra = struct('counter', 0);
path_node_count_dijkstra = 0;

path_points_astar_euclidean = [];
tree_data_astar_euclidean = struct('counter', 0);
path_node_count_astar_euclidean = 0;

path_points_astar_manhattan = [];
tree_data_astar_manhattan = struct('counter', 0);
path_node_count_astar_manhattan = 0;

% 1. Run Dijkstra
try
    disp('Running Dijkstra algorithm...');
    [path_points_dijkstra, ~, tree_data_dijkstra, path_node_count_dijkstra] = dijkstra_planning(map, start, goal, params);
    disp(['  Completed - Path nodes: ', num2str(path_node_count_dijkstra), ', Explored: ', num2str(tree_data_dijkstra.counter)]);
catch ME
    disp(['Error during Dijkstra execution: ', ME.message]);
end

% 2. Run A* with Euclidean heuristic
try
    disp('Running A* algorithm with Euclidean heuristic...');
    params.heuristic_type = 1;
    [path_points_astar_euclidean, ~, tree_data_astar_euclidean, path_node_count_astar_euclidean] = astar_planning(map, start, goal, params);
    disp(['  Completed - Path nodes: ', num2str(path_node_count_astar_euclidean), ', Explored: ', num2str(tree_data_astar_euclidean.counter)]);
catch ME
    disp(['Error during A* Euclidean execution: ', ME.message]);
end

% 3. Run A* with Manhattan heuristic
try
    disp('Running A* algorithm with Manhattan heuristic...');
    params.heuristic_type = 2;
    [path_points_astar_manhattan, ~, tree_data_astar_manhattan, path_node_count_astar_manhattan] = astar_planning(map, start, goal, params);
    disp(['  Completed - Path nodes: ', num2str(path_node_count_astar_manhattan), ', Explored: ', num2str(tree_data_astar_manhattan.counter)]);
catch ME
    disp(['Error during A* Manhattan execution: ', ME.message]);
end

% Verify that there are results to analyze
if path_node_count_dijkstra == 0 && path_node_count_astar_euclidean == 0 && path_node_count_astar_manhattan == 0
    disp('No algorithm found a path. Unable to generate comparisons.');
    rmpath(dijkstra_folder);
    rmpath(astar_folder);
    return;
end

% Ensure there are no divisions by zero
if path_node_count_dijkstra == 0
    path_node_count_dijkstra = 1;
    disp('Warning: Dijkstra did not find a valid path.');
end
if path_node_count_astar_euclidean == 0
    path_node_count_astar_euclidean = 1;
    disp('Warning: A* Euclidean did not find a valid path.');
end
if path_node_count_astar_manhattan == 0
    path_node_count_astar_manhattan = 1;
    disp('Warning: A* Manhattan did not find a valid path.');
end

% Calculate performance metrics
exploration_ratio_dijkstra = tree_data_dijkstra.counter / path_node_count_dijkstra;
exploration_ratio_astar_euclidean = tree_data_astar_euclidean.counter / path_node_count_astar_euclidean;
exploration_ratio_astar_manhattan = tree_data_astar_manhattan.counter / path_node_count_astar_manhattan;

% Save results in separate files in respective folders
if ~isempty(path_points_dijkstra)
    save(fullfile(dijkstra_folder, 'trajectory_data_dijkstra.mat'), 'path_points_dijkstra');
end
if ~isempty(path_points_astar_euclidean)
    save(fullfile(astar_folder, 'trajectory_data_astar_euclidean.mat'), 'path_points_astar_euclidean');
end
if ~isempty(path_points_astar_manhattan)
    save(fullfile(astar_folder, 'trajectory_data_astar_manhattan.mat'), 'path_points_astar_manhattan');
end

% Create comparison charts
figure('Name', 'Comparison of Path Planning Algorithms');

% Path nodes
subplot(2, 1, 1);
data_path = [path_node_count_dijkstra, path_node_count_astar_euclidean, path_node_count_astar_manhattan];
h1 = bar(data_path);
set(gca, 'XTickLabel', {'Dijkstra', 'A* Euclidean', 'A* Manhattan'});
title('Comparison of Path Nodes');
ylabel('Number of Nodes');
grid on;

% Add labels with values above the bars
text(1:length(data_path), data_path, num2str(data_path', '%d'), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

% Explored nodes
subplot(2, 1, 2);
data_explored = [tree_data_dijkstra.counter, tree_data_astar_euclidean.counter, tree_data_astar_manhattan.counter];
h2 = bar(data_explored);
set(gca, 'XTickLabel', {'Dijkstra', 'A* Euclidean', 'A* Manhattan'});
title('Comparison of Explored Nodes');
ylabel('Number of Nodes');
grid on;

% Add labels with values above the bars
text(1:length(data_explored), data_explored, num2str(data_explored', '%d'), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

% Create another chart for exploration ratio (shows efficiency)
figure('Name', 'Exploration Ratio (Explored Nodes / Path Nodes)');
data_ratio = [exploration_ratio_dijkstra, exploration_ratio_astar_euclidean, exploration_ratio_astar_manhattan];
h3 = bar(data_ratio);
set(gca, 'XTickLabel', {'Dijkstra', 'A* Euclidean', 'A* Manhattan'});
title('Exploration Ratio (lower is better)');
ylabel('Explored Nodes / Path Nodes');
grid on;

% Add labels with values above the bars
text(1:length(data_ratio), data_ratio, num2str(data_ratio', '%.2f'), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

% Display comparison statistics with better alignment
disp('=== ALGORITHM COMPARISON ===');
fprintf('%-18s %12s %17s %18s\n', 'Algorithm', 'Path Nodes', 'Explored Nodes', 'Exploration Ratio');
fprintf('%-18s %12d %17d %18.2f\n', 'Dijkstra:', path_node_count_dijkstra, tree_data_dijkstra.counter, exploration_ratio_dijkstra);
fprintf('%-18s %12d %17d %18.2f\n', 'A* Euclidean:', path_node_count_astar_euclidean, tree_data_astar_euclidean.counter, exploration_ratio_astar_euclidean);
fprintf('%-18s %12d %17d %18.2f\n', 'A* Manhattan:', path_node_count_astar_manhattan, tree_data_astar_manhattan.counter, exploration_ratio_astar_manhattan);

% Percentage improvement over Dijkstra
if tree_data_dijkstra.counter > 0
    improvement_euclidean = (1 - (tree_data_astar_euclidean.counter / tree_data_dijkstra.counter)) * 100;
    improvement_manhattan = (1 - (tree_data_astar_manhattan.counter / tree_data_dijkstra.counter)) * 100;

    fprintf('\n=== IMPROVEMENT COMPARED TO DIJKSTRA ===\n');
    fprintf('A* Euclidean:   Reduction of %6.2f%% in explored nodes\n', improvement_euclidean);
    fprintf('A* Manhattan:   Reduction of %6.2f%% in explored nodes\n', improvement_manhattan);
else
    disp('Unable to calculate improvement compared to Dijkstra (no data available).');
end

% Clean up the path at the end of execution
rmpath(dijkstra_folder);
rmpath(astar_folder);