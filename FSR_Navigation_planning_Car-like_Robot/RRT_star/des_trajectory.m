function [xd, yd, thetad, phid, q0] = des_trajectory(path_points)
    % Parameters
    l = 0.6;                    % Wheelbase length
    tf = 600;                   % Final time
    v_max = 3;                  % Maximum linear velocity
    omega_max = 0.43;           % Maximum angular velocity
    eps = 1e-6;                 % Small value to prevent division by zero
    
    % Extract x and y coordinates directly
    % FIXED: Remove the y-coordinate flipping
    x = path_points(:,1);
    y = path_points(:,2);
    
    % Define time vector for polynomial fitting
    t = linspace(0, tf, length(x));
    
    % Fit a 7th order polynomial to the normalized arc length
    s_sample = linspace(0, 1, length(t));
    [p, ~, mu] = polyfit(t, s_sample, 7);
    
    % Generate finer time grid for interpolation
    T = linspace(0, tf, 10000);
    
    % Compute interpolated s values
    s_t = polyval(p, T, [], mu);
    
    % Interpolate x and y values using spline with smoothing
    x_t = smooth(interp1(s_sample, x, s_t, 'spline'), 100);
    y_t = smooth(interp1(s_sample, y, s_t, 'spline'), 100);
    
    % Compute derivatives using gradient with smoothing
    x_dot = smooth(gradient(x_t, mean(diff(T))), 50);
    y_dot = smooth(gradient(y_t, mean(diff(T))), 50);
    x_ddot = smooth(gradient(x_dot, mean(diff(T))), 50);
    y_ddot = smooth(gradient(y_dot, mean(diff(T))), 50);
    x_dddot = smooth(gradient(x_ddot, mean(diff(T))), 50);
    y_dddot = smooth(gradient(y_ddot, mean(diff(T))), 50);
    
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
    q0 = [x_t(1), y_t(1), theta(1), phi(1)]';
    
    % Visualization
    figure('Name', 'Trajectory Analysis');
    
    % Plot original path and generated trajectory
    subplot(2,2,1);
    plot(x, y, 'b.', 'DisplayName', 'Path points');
    hold on;
    % FIXED: Set Y-axis direction to be upward-increasing (Cartesian)
    set(gca, 'YDir', 'normal');
    plot(x_t, y_t, 'm-', 'LineWidth', 2, 'DisplayName', 'Generated Trajectory');
    plot(x(1), y(1), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Start point');
    plot(x(end), y(end), 'go', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Goal point');
    title('\textbf{Trajectory}', 'Interpreter', 'latex');
    xlabel('x [m]', 'Interpreter', 'latex');
    ylabel('y [m]', 'Interpreter', 'latex');
    legend('Location', 'best', 'Interpreter', 'latex');
    grid on;
    
    % Plot velocities
    subplot(2,2,2);
    plot(T, v, 'LineWidth', 2);
    hold on;
    yline(v_max, 'r--', 'LineWidth', 1.5);
    yline(-v_max, 'r--', 'LineWidth', 1.5);
    title('\textbf{Linear Velocity}', 'Interpreter', 'latex');
    xlabel('t [s]', 'Interpreter', 'latex');
    ylabel('v [m/s]', 'Interpreter', 'latex');
    grid on;
    
    subplot(2,2,4);
    plot(T, omega, 'LineWidth', 2);
    hold on;
    yline(omega_max, 'r--', 'LineWidth', 1.5);
    yline(-omega_max, 'r--', 'LineWidth', 1.5);
    title('\textbf{Angular Velocity}', 'Interpreter', 'latex');
    xlabel('t [s]', 'Interpreter', 'latex');
    ylabel('$\omega$ [rad/s]', 'Interpreter', 'latex');
    grid on;
    
    % Plot angles
    subplot(2,2,3);
    plot(T, theta, 'LineWidth', 2);
    title('\textbf{Heading Angle $\theta$}', 'Interpreter', 'latex');
    xlabel('t [s]', 'Interpreter', 'latex');
    ylabel('$\theta$ [rad]', 'Interpreter', 'latex');
    grid on;
    
    % Additional figure for state evolution
    figure;
    subplot(4,1,1);
    plot(T, x_t, 'b', 'LineWidth', 2);
    title('\textbf{Evolution of $x_d(t)$}', 'FontSize', 10, 'Interpreter', 'latex');
    xlabel('t [s]', 'Interpreter', 'latex', 'FontSize', 10);
    ylabel('$x_d$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
    grid on;

    subplot(4,1,2);
    plot(T, y_t, 'r', 'LineWidth', 2);
    title('\textbf{Evolution of $y_d(t)$}', 'FontSize', 10, 'Interpreter', 'latex');
    xlabel('t [s]', 'Interpreter', 'latex', 'FontSize', 10);
    ylabel('$y_d$ [m]', 'Interpreter', 'latex', 'FontSize', 10);
    grid on;

    subplot(4,1,3);
    plot(T, theta, 'g', 'LineWidth', 2);
    title('\textbf{Evolution of $\theta_d(t)$}', 'FontSize', 10, 'Interpreter', 'latex');
    xlabel('t [s]', 'Interpreter', 'latex', 'FontSize', 10);
    ylabel('$\theta_d$ [rad]', 'Interpreter', 'latex', 'FontSize', 10);
    grid on;

    subplot(4,1,4);
    plot(T, phi, 'k', 'LineWidth', 2);
    title('\textbf{Evolution of $\phi_d(t)$}', 'FontSize', 10, 'Interpreter', 'latex');
    xlabel('t [s]', 'Interpreter', 'latex', 'FontSize', 10);
    ylabel('$\phi_d$ [rad]', 'Interpreter', 'latex', 'FontSize', 10);
    grid on;
end