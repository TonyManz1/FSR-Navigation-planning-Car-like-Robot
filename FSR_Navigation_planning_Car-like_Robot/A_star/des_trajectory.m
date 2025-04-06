function [xd, yd, thetad, phid, q0] = des_trajectory(collected_points, radius_curve)

% DES_TRAJECTORY Generate desired trajectory from path points
    %
    % Input:
    %   collected_points - Array of points along the path [x, y]
    %   radius_curve - Minimum turning radius for the vehicle [m]
    %
    % Outputs:
    %   xd - Timeseries of x coordinates
    %   yd - Timeseries of y coordinates
    %   thetad - Timeseries of heading angles
    %   phid - Timeseries of steering angles
    %   q0 - Initial state [x0, y0, theta0, phi0]

    % Path for saving figures
    %save_path = 'D:\Automazione e Robotica\FSR\FSR_Navigation_planning_Car-like_Robot\Images\Control_simulations\A_star\Matlab\';  

    % Parameters
    l = 0.85;                       % Wheelbase length [m]
    tf = 600;                       % Final time in seconds [s]
    v_max = 5;                      % Maximum linear velocity [m/s]
    omega_max = v_max/radius_curve; % Maximum angular velocity [rad/s]
    eps = 1e-6;                     % Small value to prevent division by zero

    % Extract x and y coordinates from collected_points directly
    x_orig = collected_points(:, 1);
    y_orig = collected_points(:, 2);
    
    % Simply use the original points directly
    x = x_orig;
    y = y_orig;
    
    % Create a uniform parameterization along path length
    cum_length = zeros(length(x), 1);
    for i = 2:length(x)
        cum_length(i) = cum_length(i-1) + norm([x(i) - x(i-1), y(i) - y(i-1)]);
    end
    
    % Normalized parameter along trajectory
    if cum_length(end) > 0
        s_orig = cum_length / cum_length(end);
    else
        s_orig = linspace(0, 1, length(x))';
    end
    
    % Generate time vector with uniform spacing
    T = linspace(0, tf, 10000)';
    s_t = linspace(0, 1, length(T))';
    
    % Create a simple piecewise cubic interpolation that ensures C1 continuity
    % while exactly passing through all waypoints
    pp_x = pchip(s_orig, x);
    pp_y = pchip(s_orig, y);
    
    % Evaluate the interpolated trajectory
    x_t = ppval(pp_x, s_t);
    y_t = ppval(pp_y, s_t);
    
    % Calculate derivatives using the same method as the original code
    dt = mean(diff(T));
    
    % Apply original smoothing to derivatives
    x_dot = smooth(gradient(x_t, dt), 20);
    y_dot = smooth(gradient(y_t, dt), 20);
    x_ddot = smooth(gradient(x_dot, dt), 30);
    y_ddot = smooth(gradient(y_dot, dt), 30);
    x_dddot = smooth(gradient(x_ddot, dt), 40);
    y_dddot = smooth(gradient(y_ddot, dt), 40);

    % Compute theta and linear velocity as in the original code
    theta = atan2(y_dot, x_dot);
    v = sqrt(x_dot.^2 + y_dot.^2);
    
    % Ensure minimum velocity to prevent divisions by very small numbers
    v = max(v, eps);
    
    % Compute curvature and steering angle using original formula
    curvature = (y_ddot .* x_dot - x_ddot .* y_dot) ./ (v.^3 + eps);
    phi = atan(l * curvature);
    
    % Limit steering angle
    phi = min(max(phi, -pi/4), pi/4);
    
    % Compute angular velocity with original safeguards
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
    
    % Apply additional smoothing to omega as in original code
    omega = smooth(omega, 100);
    
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
    figure('Name', 'Trajectory', 'Position', [100, 100, 800, 600]);
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
    xlim([0 420]);
    grid on;

    yl = ylim;
    axis equal; 
    grid on;
    
    % filename = [save_path, 'trajectory_astar.pdf'];
    % exportgraphics(gcf, filename, 'Resolution', 300, 'ContentType', 'image');
    
    % Figure 2: Heading and Angular Velocity
    figure('Name', 'Velocities', 'Position', [100, 100, 800, 600]);
    subplot(2,1,1);
    plot(T, v, 'LineWidth', 2, 'Color', 'b');
    hold on;
    yline(v_max, 'r--', 'LineWidth', 1.5);
    yline(-v_max, 'r--', 'LineWidth', 1.5);
    title('Heading Velocity', 'FontSize', 12);
    xlabel('t [s]', 'FontSize', 10);
    ylabel('v [m/s]', 'FontSize', 10);
    xlim([0 tf]);
 
    v_min = min(v) * 0.9; % Un po' di margine
    v_max_plot = max(v) * 1.1;
    ylim([v_min v_max_plot]);
    grid on;
    
    subplot(2,1,2);
    plot(T, omega, 'LineWidth', 2, 'Color', 'b');
    hold on;
    yline(omega_max, 'r--', 'LineWidth', 1.5);
    yline(-omega_max, 'r--', 'LineWidth', 1.5);
    title('Angular Velocity', 'FontSize', 12);
    xlabel('t [s]', 'Interpreter', 'tex', 'FontSize', 10);
    ylabel('\omega [rad/s]', 'Interpreter', 'tex', 'FontSize', 10);
    xlim([0 tf]);
   
    omega_min = min(omega) * 1.1;
    omega_max_plot = max(omega) * 1.1;
    ylim([omega_min omega_max_plot]);
    grid on;

    % filename = [save_path, 'velocities_astar.pdf'];
    % exportgraphics(gcf, filename, 'Resolution', 300, 'ContentType', 'image');

    % Figure 3: Evolution of states
    figure('Name', 'State Evolution', 'Position', [100, 100, 800, 800]);
    subplot(4,1,1);
    plot(T, x_t, 'b', 'LineWidth', 2);
    title('Evolution of x_d(t)', 'Interpreter', 'tex', 'FontSize', 12);
    ylabel('x_d [m]', 'Interpreter', 'tex', 'FontSize', 10);
    xlim([0 tf]);
    grid on;

    subplot(4,1,2);
    plot(T, y_t, 'r', 'LineWidth', 2);
    title('Evolution of y_d(t)', 'Interpreter', 'tex', 'FontSize', 12);
    ylabel('y_d [m]', 'Interpreter', 'tex', 'FontSize', 10);
    xlim([0 tf]);
    grid on;

    subplot(4,1,3);
    plot(T, theta, 'g', 'LineWidth', 2);
    title('Evolution of \theta_d(t)', 'Interpreter', 'tex', 'FontSize', 12);
    ylabel('\theta_d [rad]', 'Interpreter', 'tex', 'FontSize', 10);
    xlim([0 tf]);
    grid on;

    subplot(4,1,4);
    plot(T, phi, 'k', 'LineWidth', 2);
    title('Evolution of \phi_d(t)', 'Interpreter', 'tex', 'FontSize', 12);
    xlabel('t [s]', 'Interpreter', 'tex', 'FontSize', 10);
    ylabel('\phi_d [rad]', 'Interpreter', 'tex', 'FontSize', 10);
    xlim([0 tf]);
    grid on;

    % filename = [save_path, 'state_evolution_astar.pdf'];
    % exportgraphics(gcf, filename, 'Resolution', 300, 'ContentType', 'image');
end