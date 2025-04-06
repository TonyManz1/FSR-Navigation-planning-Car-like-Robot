% Comparison script between Dijkstra and A* with different heuristics
clear all;
close all;
clc;

% Define folder paths
dijkstra_folder = 'D:\Automazione e Robotica\FSR\FSR_Navigation_planning_Car-like_Robot\Dijkstra';
astar_folder = 'D:\Automazione e Robotica\FSR\FSR_Navigation_planning_Car-like_Robot\A_star';

% Add paths to MATLAB path
addpath(dijkstra_folder);
addpath(astar_folder);

% Load map (assuming it's in the current directory or in one of the added paths)
load("image_map.mat");
map = double(image_map);

% Initialize parameters
params = struct();
params.length_step = 9;      % Length of the step forward [m]
params.radius_curve = 9;     % Bending radius for left and right curves [m]

% Define start and goal points
start = [30 120];   
goal = [400 135];  

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

% 1. Chiamata all'algoritmo Dijkstra (senza creare il plot)
try
    disp('Running Dijkstra algorithm...');
    % Chiamata alla funzione di algoritmo, senza visualizzazione
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

% Create comparison charts - FIGURE 2 invece di FIGURE 1
figure_comparison = figure('Name', 'Comparison of Path Planning Algorithms', 'Visible', 'on', 'NumberTitle', 'off');

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

% Create another chart for exploration ratio (shows efficiency) - FIGURE 3
figure_ratio = figure('Name', 'Exploration Ratio (Explored Nodes / Path Nodes)', 'Visible', 'on', 'NumberTitle', 'off');
data_ratio = [exploration_ratio_dijkstra, exploration_ratio_astar_euclidean, exploration_ratio_astar_manhattan];
h3 = bar(data_ratio);
set(gca, 'XTickLabel', {'Dijkstra', 'A* Euclidean', 'A* Manhattan'});
title('Exploration Ratio (lower is better)');
ylabel('Explored Nodes / Path Nodes');
grid on;

% Add labels with values above the bars
text(1:length(data_ratio), data_ratio, num2str(data_ratio', '%.2f'), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

% Display comparison statistics with better alignment
disp('=== ALGORITMI DI PIANIFICAZIONE - CONFRONTO PRESTAZIONI ===');
fprintf('%-18s %12s %17s %18s\n', 'Algoritmo', 'Nodi percorso', 'Nodi esplorati', 'Rapporto esplorazione');
fprintf('%-18s %12d %17d %18.2f\n', 'Dijkstra:', path_node_count_dijkstra, tree_data_dijkstra.counter, exploration_ratio_dijkstra);
fprintf('%-18s %12d %17d %18.2f\n', 'A* Euclidean:', path_node_count_astar_euclidean, tree_data_astar_euclidean.counter, exploration_ratio_astar_euclidean);
fprintf('%-18s %12d %17d %18.2f\n', 'A* Manhattan:', path_node_count_astar_manhattan, tree_data_astar_manhattan.counter, exploration_ratio_astar_manhattan);

% Percentage improvement over Dijkstra
if tree_data_dijkstra.counter > 0
    improvement_euclidean = (1 - (tree_data_astar_euclidean.counter / tree_data_dijkstra.counter)) * 100;
    improvement_manhattan = (1 - (tree_data_astar_manhattan.counter / tree_data_dijkstra.counter)) * 100;

    fprintf('\n=== MIGLIORAMENTO RISPETTO A DIJKSTRA ===\n');
    fprintf('A* Euclidean:   Riduzione del %6.2f%% nei nodi esplorati\n', improvement_euclidean);
    fprintf('A* Manhattan:   Riduzione del %6.2f%% nei nodi esplorati\n', improvement_manhattan);
else
    disp('Impossibile calcolare il miglioramento rispetto a Dijkstra (nessun dato disponibile).');
end

% Ora chiamiamo i rispettivi main per generare i plot dei percorsi
% Eseguiamo i main in figure separate e senza sovrascrivere le variabili già ottenute

% Plot del percorso Dijkstra - FIGURE 4
figure_dijkstra = figure('Name', 'Dijkstra Path Planning', 'Visible', 'on', 'NumberTitle', 'off');
imshow(map);
hold on;
set(gca, 'YDir', 'normal');
title('Dijkstra Path Planning', 'FontSize', 12);
xlabel('x [m]', 'FontSize', 10);
ylabel('y [m]', 'FontSize', 10);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start point');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal point');

if ~isempty(path_points_dijkstra)
    plot(path_points_dijkstra(:,1), path_points_dijkstra(:,2), 'b-', 'LineWidth', 3, 'DisplayName', 'Smoothed path');
    legend('Start point', 'Goal point', 'Smoothed path', 'Location', 'northeast', 'FontSize', 8);
    title(['Dijkstra Path Planning - Path nodes: ' num2str(path_node_count_dijkstra) ' | Total explored: ' num2str(tree_data_dijkstra.counter)], 'FontSize', 12);
else
    legend('Start point', 'Goal point', 'Location', 'northeast', 'FontSize', 8);
    title('Dijkstra Path Planning - No path found', 'FontSize', 12);
end

% Plot del percorso A* con euristica euclidea - FIGURE 5
figure_astar_euclidean = figure('Name', 'A* Path Planning (Euclidean)', 'Visible', 'on', 'NumberTitle', 'off');
imshow(map);
hold on;
set(gca, 'YDir', 'normal');
title('A* Path Planning (Euclidean)', 'FontSize', 12);
xlabel('x [m]', 'FontSize', 10);
ylabel('y [m]', 'FontSize', 10);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start point');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal point');

if ~isempty(path_points_astar_euclidean)
    plot(path_points_astar_euclidean(:,1), path_points_astar_euclidean(:,2), 'm-', 'LineWidth', 3, 'DisplayName', 'Smoothed path');
    legend('Start point', 'Goal point', 'Smoothed path', 'Location', 'northeast', 'FontSize', 8);
    title(['A* Path Planning (Euclidean) - Path nodes: ' num2str(path_node_count_astar_euclidean) ' | Total explored: ' num2str(tree_data_astar_euclidean.counter)], 'FontSize', 12);
else
    legend('Start point', 'Goal point', 'Location', 'northeast', 'FontSize', 8);
    title('A* Path Planning (Euclidean) - No path found', 'FontSize', 12);
end

% Plot del percorso A* con euristica Manhattan - FIGURE 6
figure_astar_manhattan = figure('Name', 'A* Path Planning (Manhattan)', 'Visible', 'on', 'NumberTitle', 'off');
imshow(map);
hold on;
set(gca, 'YDir', 'normal');
title('A* Path Planning (Manhattan)', 'FontSize', 12);
xlabel('x [m]', 'FontSize', 10);
ylabel('y [m]', 'FontSize', 10);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start point');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal point');

if ~isempty(path_points_astar_manhattan)
    plot(path_points_astar_manhattan(:,1), path_points_astar_manhattan(:,2), 'c-', 'LineWidth', 3, 'DisplayName', 'Smoothed path');
    legend('Start point', 'Goal point', 'Smoothed path', 'Location', 'northeast', 'FontSize', 8);
    title(['A* Path Planning (Manhattan) - Path nodes: ' num2str(path_node_count_astar_manhattan) ' | Total explored: ' num2str(tree_data_astar_manhattan.counter)], 'FontSize', 12);
else
    legend('Start point', 'Goal point', 'Location', 'northeast', 'FontSize', 8);
    title('A* Path Planning (Manhattan) - No path found', 'FontSize', 12);
end

% Generate desired trajectories for each valid path
if ~isempty(path_points_dijkstra)
    disp('Generating Dijkstra trajectory...');
    [xd_dijkstra, yd_dijkstra, thetad_dijkstra, phid_dijkstra, q0_dijkstra] = des_trajectory(path_points_dijkstra, params.radius_curve);
    save(fullfile(dijkstra_folder, 'trajectory_data_dijkstra_complete.mat'), 'xd_dijkstra', 'yd_dijkstra', 'thetad_dijkstra', 'phid_dijkstra', 'q0_dijkstra');
end

if ~isempty(path_points_astar_euclidean)
    disp('Generating A* Euclidean trajectory...');
    [xd_euclidean, yd_euclidean, thetad_euclidean, phid_euclidean, q0_euclidean] = des_trajectory(path_points_astar_euclidean, params.radius_curve);
    save(fullfile(astar_folder, 'trajectory_data_astar_euclidean_complete.mat'), 'xd_euclidean', 'yd_euclidean', 'thetad_euclidean', 'phid_euclidean', 'q0_euclidean');
end

if ~isempty(path_points_astar_manhattan)
    disp('Generating A* Manhattan trajectory...');
    [xd_manhattan, yd_manhattan, thetad_manhattan, phid_manhattan, q0_manhattan] = des_trajectory(path_points_astar_manhattan, params.radius_curve);
    save(fullfile(astar_folder, 'trajectory_data_astar_manhattan_complete.mat'), 'xd_manhattan', 'yd_manhattan', 'thetad_manhattan', 'phid_manhattan', 'q0_manhattan');
end

% Modifica: chiudi la figura 1 se è stata creata durante l'esecuzione degli algoritmi
if ishandle(1)
    close(1);
end

% Clean up the path at the end of execution
rmpath(dijkstra_folder);
rmpath(astar_folder);