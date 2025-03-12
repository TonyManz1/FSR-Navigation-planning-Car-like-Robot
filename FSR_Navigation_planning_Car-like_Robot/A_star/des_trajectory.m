function [xd, yd, thetad, phid, q0] = des_trajectory(collected_points)
    % DES_TRAJECTORY Generate desired trajectory from path points
    %
    % Input:
    %   collected_points - Array of points along the path [x, y]
    %
    % Outputs:
    %   xd - Timeseries of x coordinates
    %   yd - Timeseries of y coordinates
    %   thetad - Timeseries of heading angles
    %   phid - Timeseries of steering angles
    %   q0 - Initial state [x0, y0, theta0, phi0]

    % Parameters
    l = 0.6;                     % Wheelbase length
    tf = 600;                    % Final time in seconds
    v_max = 3;                   % Maximum linear velocity
    omega_max = 0.43;            % Maximum angular velocity
    eps = 1e-6;                  % Small value to prevent division by zero

    % Extract x and y coordinates from collected_points directly
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