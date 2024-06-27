# Learning about control algorithms

Summer 2024

Planned algorithms to learn about:
- PID
- LQR
- Markov Chain Monte Carlo (MCMC)

## PD Algorithm

To explor this type, I used the system of a 1-DOF Thrust Vector Control, i.e. the "rocket" is free to rotate along its COM in one axis, and the thrust is able to correct this only in that same axis.
- Controlled variable: $\phi$, representing the rotation in degrees of the rocket from upright
- Actuator: $\theta$, representing the angle in degrees at which the thrust is applied. 

$\theta$ creates a torque which is able to correct $\phi$ to follow the reference signal. 

As of right now, $K_d$ and $K_p$ are set manually. In the coming days I will add formulas which will set these automatically based on the desired damping ratio $\zeta$ and natural frequency $\omega_n$. This will allow for more control over the tuning and improve settling time, overshoot, and stability.