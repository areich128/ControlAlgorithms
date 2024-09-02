%% MCMC PID tuning for inverted pendulum
clear;
% clc;

%% System Dynamics Code

tspan = 1:0.01:10;

y = zeros(4, length(tspan));

error = zeros(length(tspan));
F = zeros(length(tspan));

Kp = 7;
Ki = 0.001;
Kd = 5;
Ku_e = 4;
Kd_e = 1.5;
m = 0.2;        
M = 0.4;        
L = 0.75;       
g = 9.81;      
d = 0.75;

ref = pi;
theta0 = 0;

ref1 = 0;
theta01 = pi;

% y = simulate(error, F, tspan, Kp, Kd, Ku_e, Kd_e, m, M, L, g, d, ref, theta0);

%% SIMULATION LOOP (MCMC and Metropolis-Hastings Algorithm)
trainspan = 0:10;
x = zeros(4, length(trainspan));
x(:, 1) = [Kp; Kd; Ku_e; Kd_e];
xprime = zeros(4, 1);

accepted = 0;
sigma = 0.005;
J = cost(tspan, x(1,1), x(2,1), x(3,1), x(4,1), m, M, L, g, d, ref, theta0) + cost(tspan, x(1,1), x(2,1), x(3,1), x(4,1), m, M, L, g, d, ref1, theta01);
%fprintf("Initial J = " + J + "\n");
for i = 1:length(trainspan) - 1
    xprime = min(x(:,i) + (sigma * randn(4, 1)), [10; 10; 10; 10]);
    Jprime = cost(tspan, xprime(1,1), xprime(2,1), xprime(3,1), xprime(4,1), m, M, L, g, d, ref, theta0) + cost(tspan, xprime(1,1), xprime(2,1), xprime(3,1), xprime(4,1), m, M, L, g, d, ref1, theta01);
    alpha = min(1, exp(J - Jprime));
    u = rand;
    if u <= alpha
        x(:, i+1) = xprime;
        J = Jprime;
        accepted = accepted + 1;
        %fprintf("GAINS UPDATED\n");
        %fprintf("Updated J = " + J + "\n");
    else
        x(:, i+1) = x(:, i);
        %fprintf("GAINS NOT UPDATED\n");
    end
    
    % decreasing the standard deviation over time to help results converge
    % if i < 503
    %     sigma = sigma - 0.0001;
    % end
end
percent = (accepted / length(trainspan)) * 100;
% fprintf("Accepted: " + percent + "%% \n");
% fprintf("Kp = " + x(1, length(trainspan)) + "\nKd = " + x(2, length(trainspan)) + "\nKu_e = " + x(3, length(trainspan)) + "\nKd_e = " + x(4, length(trainspan)) + "\n");

% RUN THE SIMULATION WITH THE CALCULATED GAINS
%y = simulate(error, F, tspan, x(1,1), x(2,1), x(3,1), x(4,1), m, M, L, g, d, ref, theta0);
y = simulate(error, F, tspan, 8.5656, 0.6379, 1.2872, 1.4782, m, M, L, g, d, ref, pi/4);

%% ANIMATION CODE
graph = 1;

if graph == 0
    figure;
    theta = zeros(length(tspan));
    x = zeros(length(tspan));
    % e_error = zeros(length(tspan));
    for i = 1:length(tspan)
        theta(i) = y(1, i);
        x(i) = y(3, i);
        % e_error = energy_ref - energy(i);
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
elseif graph == 1
    % Set up figure for animation
    
    figure;
    axis equal;
    axis([-3 3 -0.8 0.8]);
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
        line([x pendulumX], [0 pendulumY], 'Color', 'b', 'LineWidth', 3);
    
        % Draw the pendulum bob
        rectangle('Position', [(pendulumX - pendulumWidth/2) (pendulumY - pendulumWidth/2) (pendulumWidth) (pendulumWidth)], 'Curvature', [1, 1], 'FaceColor', [0.1 0.1 0.8]);
    
        % Pause to create animation effect
        if k == 1
            pause(1);
        end
        pause(0.05);
    end
elseif graph == 2
    figure;
    hold on;
    plot(x(1,:), 'b');
    plot(x(2,:), 'g');
    plot(x(3,:), 'r');
    plot(x(4,:), 'c');
    legend('Kp', 'Kd', 'Ku_e', 'Kd_e');
    ylabel("Gain");
    xlabel("Iterations");
end

%% DEFINING COST FUNCTION
function J = cost(tspan, Kp, Kd, Ku_e, Kd_e, m, M, L, g, d, ref, theta0)
    error = zeros(length(tspan));
    F = zeros(length(tspan));
    stability_penalty = 0;
    error_sum = 0;
    resting = 0;
    [y, F] = simulate(error, F, tspan, Kp, Kd, Ku_e, Kd_e, m, M, L, g, d, ref, theta0);
    for i = 1:length(tspan)
        %error_avg = ((error_avg * i-1) + abs(ref - y(1,i)))/i;
        error_sum = error_sum + (ref - y(1,i))^2;
        if i < 1
            stability_penalty = stability_penalty + (y(1,i) - y(1,i-1))^2;
        end
        if ref - y(1,i) < 0.1
            resting = resting + 1;
        end
    end
    ts = length(tspan) - resting;

    control_effort = sum(F.^2);
    max_error = pi * length(tspan);
    scaled_error = error_sum/max_error;
    scaled_CE = control_effort / (20*length(tspan));
    max_stability_cost = pi^2 * length(tspan);
    scaled_stability = stability_penalty / max_stability_cost;

    J = 75*scaled_error + scaled_CE + 75*scaled_stability + 500*ts;% + 10*(Kp^2 + Kd^2 + Ku_e^2 + Kd_e^2)/400;
end

%% DEFINING THE SIMULATION + CONTROL ARCHITECTURE

function [y, F] = simulate(error, F, tspan, Kp, Kd, Ku_e, Kd_e, m, M, L, g, d, ref, theta0)

    x0 = 0;
    xdot0 = 0;
    thetadot0 = 0;

    Ki = 0;
    
    y0 = [theta0; thetadot0; x0; xdot0];

    h = 0.01;
    y = zeros(4, length(tspan));
    errordot = zeros(length(tspan));
    esum = zeros(length(tspan));
    y(:,1) = y0;

    for i = 1 : length(tspan)-1
        theta = y(1, i);
        thetadot = y(2, i);
        x = y(3, i);
        xdot = y(4, i);
    
        error(i) = ref - theta;
        errordot(i) = -thetadot;
    
        % ENERGY SHAPING ALGORITHM
        energy_ref = m*g*L*(1-cos(ref));
        KE_c = 0.5*M*xdot^2;
        PE_p = m*g*L*(1 - cos(theta));
        KE_p = 0.5*m*(L*thetadot)^2;
    
        % Ku_e = 5.1; % swing up energy shaping gain
        % Kd_e = 1; % swing down energy shaping gain
    
        energy_error = energy_ref - (KE_c + PE_p + KE_p);
        if abs(error(i)) < 1
            %PID
            if (ref == pi) % SWING UP
                F(i) = min((Kp * error(i)) + (Kd * errordot(i)) - (Ki * esum(i)), 20);
                %fprintf("PID SWING UP ACTIVATED AT x=" + i + "\n");
            elseif (ref == 0) % SWING DOWN
                if theta > pi
                    ref = 2*pi;
                    error(i) = ref - theta;
                    errordot(i) = thetadot;
                end
                F(i) = min(-(Kp * error(i)) - (Kd * errordot(i)) - (Ki * esum(i)), 20);
                %fprintf("PID SWING DOWN ACTIVATED AT x=" + i + "\n");
            end
        else
            if energy_error > 0
                % INJECT ENERGY
                %fprintf("ENERGY INJECTION ACTIVATED AT x=" + i + "\n");
                if PE_p == 0 && thetadot == 0
                    F(i) = Ku_e;
                elseif PE_p < m*g*L
                    F(i) = min(Ku_e * sign(-thetadot) * (energy_error * 2 * M), 20);
                else
                    F(i) = min(Ku_e * sign(thetadot) * (energy_error * 2 * M), 20);
                end
            elseif energy_error < 0
                % DIFFUSE ENERGY
                %fprintf("ENERGY DIFFUSION ACTIVATED AT x=" + i + "\n");
                if PE_p == 2*m*g*L
                    F(i) = Kd_e;
                % elseif PE_p > m*g*L
                %     F(i) = min(Kd_e * sign(thetadot) * energy_error, 20);
                elseif PE_p < m*g*L
                    F(i) = min(Kd_e * sign(-thetadot) * energy_error, 20);
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


