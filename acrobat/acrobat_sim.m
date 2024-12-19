clc;
clear all;
close all;

c = getConst();
tspan = 0:0.1:50;
initialstate = [c.theta_init; c.thetadot_init];

[tspan,state] = ode45(@(tspan,state) dynamics(state,c),tspan,initialstate);
state(:,1) = mod(state(:,1), 360);
% Ft = debuglog(state, c);

figure();
plot(tspan, state(:,1));

figure;
axis equal;
axis([-1.5 1.5 -1.5 1.5]);
grid on;
hold on;
pendulumWidth = 0.05;
for i = 1:length(tspan)
    theta = state(i,1);
    pendulumX = c.L*sind(state(i,1));
    pendulumY = -c.L*cosd(state(i,1));

    % Clear previous frame
    cla;
    rectangle('Position', [-0.05 -0.05 0.1 0.1], 'Curvature', [0.3, 0.3], 'FaceColor', [0 0 0]);
    % Draw the pendulum
    line([0 pendulumX], [0 pendulumY], 'Color', 'b', 'LineWidth', 3);

    % Draw the pendulum bob
    rectangle('Position', [(pendulumX - pendulumWidth/2) (pendulumY - pendulumWidth/2) (pendulumWidth) (pendulumWidth)], 'Curvature', [1, 1], 'FaceColor', [0.1 0.1 0.8]);

    % Pause to create animation effect
    if i == 1
        pause(1);
    end
    pause(0.01);
end

% figure();
% plot(tspan, Ft);

function dydt = dynamics(state, c)
    theta = state(1);
    theta = mod(theta, 360);
    thetadot = state(2);
    ref = c.ref;
    m = c.m;
    g = c.g;
    b = c.damp_coeff;

    e = ref - theta;
    ed = 0 - thetadot;
    tau_g = m * g * c.COM * sind(theta);
    tau_d = b * thetadot;

    Ft = ((c.kp * e) + (c.kd * ed) + tau_g + tau_d)/c.L;

    if (Ft < 0)
        Ft = 0;
    end

    % Ft = 0;

    thetadotdot = (Ft * c.L / m) - (tau_g / m) - (tau_d);

    dydt = [thetadot; thetadotdot];
end

function debug = debuglog(state, c)
    theta = state(:, 1);
    thetadot = state(:, 2);
    ref = c.ref;

    e = ref - theta;
    ed = 0 - thetadot;
    Ft = (c.kp .* e) + (c.kd .* ed);

    Ft(Ft < 0) = 0;

    % debug = [e, ed, Ft];
    debug = Ft;
end

function c = getConst()
    c.g = 9.81;
    c.m = 0.1; %kg
    c.theta_init = 20;
    c.thetadot_init = 0;
    c.damp_coeff = 0.1;
    c.ref = 135;
    c.kp = 0.5;
    c.kd = 1.5;
    c.L = 1;
    c.COM = 0.75; %consider doing automatic adjustment of Kp and Kd for different COM locations
end