clc;
close all;
clear all;

c = getConst();
tspan = 0:0.1:50;
initialstate = [c.theta_init; c.thetadot_init];

[tspan,state] = ode45(@(tspan,state) dynamics(state,c),tspan,initialstate);
% state(:,1) = mod(state(:,1), 360);

[tspan,omega] = ode45(@(tspan,omega) torque_to_wheelspeed(state,c),tspan,0);

figure();
plot(tspan, state(:,1));

figure();
plot(tspan, state(:,2));

figure();
plot(tspan, omega);

function dydt = dynamics(state, c)
    theta = state(1);
    thetadot = state(2);
    mode = c.mode;

    e = c.thetaref - theta;
    ed = c.thetadotref - thetadot;

    if (mode == 1)
        tau_w = (c.Kd * thetadot);
    elseif (mode == 2)
        tau_w = -((c.Kd * e) + (c.Kp * ed));
    end
    
    % tau_disturbance = rand;
    tau_disturbance = 2;
    thetadotdot = (tau_disturbance - tau_w) / c.Is;
    dydt = [thetadot; thetadotdot];
end

function omegadot = torque_to_wheelspeed(state, c)
    theta = state(1);
    thetadot = state(2);
    mode = c.mode;

    e = c.thetaref - theta;
    ed = c.thetadotref - thetadot;

    if (mode == 1)
        tau_w = (c.Kd * thetadot);
    elseif (mode == 2)
        tau_w = -((c.Kd * e) + (c.Kp * ed));
    end

    omegadot = tau_w ./ c.Iw;
end

function c = getConst()
    c.Is = 4;
    c.Iw = 0.1;
    c.Kp = 1;
    c.Kd = 1;
    c.thetaref = 270;
    c.thetadotref = 0;
    c.theta_init = 90;
    c.thetadot_init = -20;
    c.mode = 2;
end