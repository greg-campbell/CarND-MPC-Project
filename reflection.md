# CarND-MPC-Project

## The Model

The model consists of 6 state variables:
* The car's x and y coordinates, x(t) and y(t)
* The car's heading, psi(t)
* The car's velocity, v(t)
* The cross-track error, cte(t)
* The directional error, epsi(t)

The model has two actuators:
* The steering angle, delta(t)
* The acceleration, a(t)

The update equations:
    x(t + 1) = x(t) + v(t) * cos(psi(t)) * dt
    y(t + 1) = y(t) + v(t) * sin(psi(t)) * dt
    psi(t + 1) = psi(t) + v(t)/Lf * delta(t) * dt
    v(t + 1) = v(t) + a(t) * dt
    cte(t + 1) = f(x(t)) - y(t) + v(t) * sin(epsi(t)) * dt
    epsi(t + 1) = psi(t) - atan(f'(x(t))) + v(t)/Lf * delta(t) * dt 

## Timestep Length and Elapsed Duration

I used N = 10, dt = 0.1 as suggested in the project Q&A.

Using larger values of N does not improve handling, as the model is unable to predict accurately too far in the future. Smaller values of N do not allow MPC to have enough information to create a smooth and consistent path because the model will oscillate as it is recomputing for a small number of points.

As mentioned in the project Q&A, these values of N and dt mean that the model is predicting for one second in the future. This seems like a reasonable timeframe for the model to operate in, as it would be unreasonable to expect the model to be able to accurately optimize further out than that.

## Latency

In order to account for the 100 ms latency, the initial values are calculated as follows:
    double initial_x = v * 0.1; // Future position of the car after 100 ms.
    double initial_y = 0;
    double initial_psi = -v * steer_value * 0.05; // Future heading of the car, which is some function of the velocity and steering angle.
    double initial_v = v + throttle_value * 0.25; // Future velocity of the car taking into account acceleration.
 
