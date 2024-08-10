# Notes on using MCMC for tuning gains in inverted pendulum control

## My architecture

Based off research, I decided upon a hybrid control structure. When $ref - \theta < 1$, a PD controller is enabled, which simply feeds back $\theta$, $\dot \theta$ in order to stabilize the system at the top or bottom. When the error is larger than 1, the strategy of energy shaping is employed.

Energy shaping is a method of control that allows you to take advantage of the fact that the energy in the system can be determined by the control input, in this case a force on the cart, F. Essentially, for large errors (such as in a swing-up or swing-down scenario), instead of trying to change theta directly, you strategically inject energy into the system. When the pendulum is at the top of it's swing, all of its energy will be converted to potential energy, and when it is at the bottom it will all be kinetic due to conservation of energy. Through applying the control force, we can do work on the system in order to add energy, or force the system to do work on us in order to remove energy.

Based off this architecture, there are 4 gains which needed to be tuned. $K_p$ (the proportional gain in the PD), $K_d$ (the derivative gain in the PD), $K_{ue}$ (the gain for the energy shaping swing up strategy), and $K_{de}$ (the gain for the energy shaping swing down strategy).

## Tuning using MCMC and Metropolis-Hastings
Once the control architecture was set up, the gains needed to be tuned. I decided to use MCMC for this because the nonlinearity of the system meant that finding an analytic solution by hand would be very difficult.

The steps were these:
1. An initial guess was made on the gains I was trying to tune
2. A cost function was created, which would run the simulation with the given gains and produce a cost, which was based on settling time, overall error sum, control effort, etc.
3. A random amount was added to each gain, based on a gaussian distribution with a standard deviation of 0.005 (determined through guessing).
4. A cost was calculated for the old set of gains (lets call them $\bf(x)$) and the proposed set of gains ($\bf(x')$) by running the control system simulation for both sets of gains and comparing the outputs.
    - To increase efficiency, only the cost for $\bf(x')$ is computed. If $\bf(x')$ is accepted, then the original cost becomes this new cost. Otherwise it stays the same.
    - Since I wanted to optimize for both swing-up and swing-down, my cost was the sum of the costs for both scenarios. I.e., to compute the cost I found the swing-up scenario's cost as well as the swing-down scenario's cost, and added the two for the total cost of that particular gain set.
5. An acceptance probability was computed based on the formula $\alpha = min(1, exp(J - J'))$ where J and J' are the current and proposed costs.
6. A random number between 0 and 1 was computed. If $\alpha$ is less than or equal to that number, $\bf(x)$ becomes $\bf(x')$. Otherwise, $\bf(x)$ remains the same. This means that the gain set with the lowest cost does not always get accepted, which is important because that prevents the solution from becoming stuck in local minima. This is the **Metropolis-Hastings Algorithm** and is important because it allows a larger amount of the state space to be explored.
7. Steps 2-6 were repeated between 1,000 and 10,000 times, until the gains converged. The percentage of accepted gains was recorded as well. The percentage of accepted gains to total iterations should be around 20-40%. If it is too high, you can either decrease your standard deviation or change your cost function. If it is too low, increase the standard deviation or also change the cost function.
    - Getting a good cost function is crucial, learned this the hard way.
    - Gains are going to be specific to the situation that you "train" them on.
