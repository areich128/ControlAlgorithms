%% MCMC PID tuning for inverted pendulum

%% System Dynamics Code

tspan = 1:0.01:30;
m = 0.2;        
M = 0.4;        
L = 0.75;       
g = 9.81;      
d = 0.5;
esum = 0;

% initial conditions:
x0 = 0;
xdot0 = 0;
theta0 = pi;
thetadot0 = 0;

y0 = [theta0; thetadot0; x0; xdot0];

% Solve ODE
[t, y] = ode45(@(t, y) cartPendulumODE(t, y, m, M, L, g, d, esum), tspan, y0);

% figure;
% theta = zeros(length(tspan));
% errordot = zeros(length(tspan));
% x = zeros(length(tspan));
% error = zeros(length(tspan));
% F = zeros(length(tspan));
% for i = 1:length(tspan)
%     theta(i) = y(i, 1);
%     x(i) = y(i, 3);
%     error(i) = 0 - theta(i);
%     errordot(i) = 0 - y(i, 2);
%     F(i) = -(error(i)) - (errordot(i));
% end
% plot(theta, Color='red');
% hold on;
% plot(x, Color='green');
% % plot(error, Color='blue');
% % plot(errordot, Color='black');
% plot(F, Color='cyan');
% legend("theta", "x", "theta error", "errordot", "F");

% Set up figure for animation

figure;
axis equal;
axis([-5 5 -0.8 0.8]);
grid on;
hold on;

% Cart and pendulum dimensions
cartWidth = 0.3;
cartHeight = 0.1;
pendulumWidth = 0.02;

% Loop over the time vector to create the animation
for k = 1:length(t)
    % Extract current positions
    theta = y(k, 1);
    % theta = 45;
    x = y(k, 3);
    % x = 0;

    % Calculate pendulum position
    pendulumX = x + L*sin(theta);
    pendulumY = -L*cos(theta);

    % Clear previous frame
    cla;

    % Draw the cart
    rectangle('Position', [x - cartWidth/2, -cartHeight/2, cartWidth, cartHeight], 'Curvature', 0.1, 'FaceColor', [0.8 0.1 0.1]);

    % Draw the pendulum
    line([x pendulumX], [0 pendulumY], 'Color', 'b', 'LineWidth', 2);

    % Draw the pendulum bob
    rectangle('Position', [(pendulumX - pendulumWidth/2) (pendulumY - pendulumWidth/2) (pendulumWidth) (pendulumWidth)], 'Curvature', [1, 1], 'FaceColor', [0.1 0.1 0.8]);

    % Pause to create animation effect
    pause(0.02);
end

% Define the ODE function
function dydt = cartPendulumODE(t, y, m, M, L, g, d, esum)
    theta = y(1);
    thetadot = y(2);
    x = y(3);
    xdot = y(4);

    % Kd_e = 2;
    Kp_e = 2;
    Kd_t = 0.5;
    Kp_t = 0.5;
    Ki = 0;
    ref = 0;
    error = ref - theta;
    errordot = -thetadot;
    esum = esum + theta;

    if abs(error) < 0.25
        F = -(Kp_t * error) - (Kd_t * errordot) + (Ki * esum);
    else
        energy_ref = m*g*2*L;
        energy = 0.5*m*(L*thetadot)^2 - m*g*L*cos(theta);
        energy_error = energy_ref - energy;
        if theta ~= 0 && thetadot ~= 0
            F = sign(-thetadot) * (Kp_e * energy_error);
        else
            F = Kp_e * energy_error;
        end
    end


    % Equations of motion
    dydt = zeros(4,1);
    dydt(1) = thetadot;
    dydt(2) = ((-m*g*L*sin(theta) - m*g*dydt(4)*cos(theta) - F*cos(theta))/(m*L*L));
    dydt(3) = xdot;
    dydt(4) = ((F - m*L*dydt(2)*cos(theta) - m*L*thetadot*thetadot*sin(theta) - d*xdot)/(M+m));
end


