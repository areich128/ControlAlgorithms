%% MCMC PID tuning for inverted pendulum
clear;

%% System Dynamics Code

tspan = 1:0.01:10;
h = 0.01;
m = 0.2;        
M = 0.4;        
L = 0.75;       
g = 9.81;      
d = 0.75;

% initial conditions:
x0 = 0;
xdot0 = 0;
theta0 = 3*pi/2;
thetadot0 = 0;

y0 = [theta0; thetadot0; x0; xdot0];

y = zeros(4, length(tspan));
y(:,1) = y0;

error = zeros(length(tspan));
errordot = zeros(length(tspan));
esum = zeros(length(tspan));
x_esum = zeros(length(tspan));
F = zeros(length(tspan));
energy = zeros(length(tspan));

%% SIMULATION LOOP (usking RK 4th order)
for i = 1 : length(tspan)-1
    theta = y(1, i);
    thetadot = y(2, i);
    x = y(3, i);
    xdot = y(4, i);

    % Control Algorithm
    Kp = 2;
    Kd = 6;
    Ki = 0.001;
    Kp_x = 3;
    Kd_x = 3;
    Ki_x = 0.01;
    ref = pi; % in radians
    error(i) = ref - theta;

    % ENERGY SHAPING ALGORITHM
    energy_ref = m*g*L*(1-cos(ref));
    KE_c = 0.5*M*xdot^2;
    PE_p = m*g*L*(1 - cos(theta));
    KE_p = 0.5*m*(L*thetadot)^2;

    Ku_e = 5.1; % swing up energy shaping gain
    Kd_e = 1; % swing down energy shaping gain

    energy_error = energy_ref - (KE_c + PE_p + KE_p);
    if abs(error(i)) < 0.2
        %PID
        if (ref == pi) % SWING UP
            F(i) = (Kp * error(i)) - (Kd * errordot(i)) - (Ki * esum(i));
            fprintf("PID SWING UP ACTIVATED AT x=" + i + "\n");
        elseif (ref == 0) % SWING DOWN
            F(i) = (Kp * error(i)) + (Kd * errordot(i)) - (Ki * esum(i));
            fprintf("PID SWING DOWN ACTIVATED AT x=" + i + "\n");
        end
    else
        if energy_error > 0
            % INJECT ENERGY
            fprintf("ENERGY INJECTION ACTIVATED AT x=" + i + "\n");
            if PE_p == 0 && thetadot == 0
                F(i) = Ku_e;
            elseif PE_p < m*g*L
                F(i) = Ku_e * sign(-thetadot) * (energy_error * 2 * M);
            else
                F(i) = Ku_e * sign(thetadot) * (energy_error * 2 * M);
            end
        elseif energy_error < 0
            % DIFFUSE ENERGY
            fprintf("ENERGY DIFFUSION ACTIVATED AT x=" + i + "\n");
            if PE_p == 2*m*g*L
                F(i) = Kd_e;
            % elseif PE_p > m*g*L
            %     F(i) = sign(-thetadot) * energy_error;
            elseif PE_p < m*g*L
                F(i) = Kd_e * sign(-thetadot) * energy_error;
            end
        elseif error(i) == 0
            F(i) = 0;
        else
            % TRANSFER ENERGY USING CONSERVATION OF MOMENTUM
        end
    end


    % Solving the ODE
    k1 = h * cartPendulumODE(tspan(i), y(:, i), m, M, L, g, d, F(i));
    k2 = h * cartPendulumODE(tspan(i) + h/2, y(:, i) + k1/2, m, M, L, g, d, F(i));
    k3 = h * cartPendulumODE(tspan(i)+h/2, y(:, i) + k2/2, m, M, L, g, d, F(i));
    k4 = h * cartPendulumODE(tspan(i)+h, y(:, i) + k3, m, M, L, g, d, F(i));
    y(:, i+1) = y(:, i) + (k1 + 2*(k2+k3) + k4)/6.0;
end


% Solve ODE
% [t, y] = ode45(@(t, y) cartPendulumODE(t, y, m, M, L, g, d, esum), tspan, y0);

%% ANIMATION CODE
graph = 1;

if graph == 0
    figure;
    theta = zeros(length(tspan));
    x = zeros(length(tspan));
    e_error = zeros(length(tspan));
    for i = 1:length(tspan)
        theta(i) = y(1, i);
        x(i) = y(3, i);
        e_error = energy_ref - energy(i);
        % error(i) = 0 - theta(i);
        % errordot(i) = 0 - y(2, i);
    end
    plot(tspan, theta, 'r');
    hold on;
    plot(tspan, x,'g');
    plot(tspan, F, 'b');
    plot(tspan, error,'c');
    hold off;
    % legend({'Theta','X','Control force','Energy'});
    % plot(x, Color='green');
    % plot(error, Color='blue');
    % plot(errordot, Color='black');
    % plot(F, Color='cyan');
else
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
    for k = 1:length(tspan)
        % Extract current positions
        theta = y(1, k);
        % theta = 45;
        x = y(3, k);
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
        pause(0.01);
    end
end

%% DEFINING THE ODE FUNCTION

function dydt = cartPendulumODE(t, y, m, M, L, g, d, F)
    theta = y(1);
    thetadot = y(2);
    x = y(3);
    xdot = y(4);

    % F = 0;

    % Equations of motion
    dydt = zeros(4,1);
    dydt(1) = thetadot;
    dydt(2) = ((-m*g*L*sin(theta) - m*g*dydt(4)*cos(theta) - F*cos(theta))/(m*L*L));
    dydt(3) = xdot;
    dydt(4) = ((F - m*L*dydt(2)*cos(theta) + m*L*thetadot*thetadot*sin(theta) - d*xdot)/(M+m));
end


