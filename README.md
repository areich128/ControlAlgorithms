# Learning about control algorithms

Summer 2024

Planned algorithms to learn about:
- PID
- LQR
- Markov Chain Monte Carlo (MCMC)

## PD Algorithm

To explor this type, I used the system of a 1-DOF Thrust Vector Control, i.e. the "rocket" is free to rotate in one axis, and the thrust is able to correct this only in that same axis.
- Controlled variable: $\phi$, representing the rotation in degrees of the rocket from upright
- Actuator: $\theta$, representing the angle in degrees at which the thrust is applied. 

$\theta$ creates a torque which is able to correct $\phi$ to follow the reference signal. 

The `tune()` method allows for the user to tune $K_p$ and $K_d$ gains automatically based on the desired $\zeta$ (damping ratio) and $\omega_n$ (natural frequency) of the system, using the following system dynamics:

#### System Dynamics and Tuning for the 1-DOF TVC
Thrust is created at the end of the rocket, thus creating a torque about the axis of rotation of the rocket. The magnitude of this torque varies with the angle of the thrust relative to that of the rocket ($\theta$). If we do not allow $\theta > 15$, we can use the small angle approximation in the following steps:

$\tau = dFsin(\theta)$

$\tau = dF\theta$

We also know from angular kinematics that $\tau = I\alpha$, and that $\alpha = \ddot \phi$. From this we can solve to get the equation

$\ddot \phi = \frac{dF\theta}{I}$

From here, we use the control transfer function:

$\theta = -K_p\phi - K_d\dot\phi$

and our block chart (coming soon) to determine the closed loop transfer function $G(s)$. The denominator of this equation will give us our characteristic polynomial, which works out to 

$s^2 - x(K_p + sK_d)$

(where $x = \frac{dF}{I}$)

Comparing this to the base characteristic equation $s^2 + 2\zeta\omega_n s + \omega_n ^2 = 0$, we can solve for $K_d$ and $K_p$.

**NOTE:** The use of an integral gain $K_i$ was intentionally omitted. I wanted to be able to actually do the math to tune the controller, and using an integral gain creates a 3rd order system, which I do not know how to work with yet. Since integral gains are intended to remedy steady state error, I didn't feel this was necessary yet anyways.