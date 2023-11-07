# radar-ego-speed-detection-carla
In this project, I have used Carla simulator and radar sensor to calculate the velocity of an ego vehicle. I justify the correctness of the calculated ego velocity by comparing Carla provided ego velocity and observing that most of the results are the same as Carla provided ego velocity or very close to it.

Radar detects the static objects (Trees, road signs, buildings or anything that doesn’t have movement), and provides us azimuth, elevation and doppler velocity. Using this we can calculate the ego velocity using a mathematical formula.

Ego vehicle moves with velocity vx (longitudinal velocity) and vy (lateral speed). Every static object (trees, buildings, road signs) moves with the same velocity in the opposite direction (-vx, -vy). Radar cannot detect vx vy directly. It detects doppler speed vr. Doppler speed is maximum at 0 degree angle (azimuth A and elevation E), it reduces when angle is increased. There is a math relationship between vr, A, E, vx, vy. But two unknowns are in this relationship: vx, vy. We need 2 targets so we get two sets of vr, A, E. We can create two equations, where vx and vy are unknowns. Using Cramer's rule, we solve the equation to get vx and vy. I find ego velocity v = root of vx^2 + vy^2