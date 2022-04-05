# Exploring-Kalman-Filter-in-a-V2I-Implementation
Simulate a V2I environment by generating noisy measurements from 3 separate radar towers that will return the distance of the vehicle from each radar tower at each time step. These noisy measurements will then be propagated through a KF to estimate the current state of the vehicle in the 2D environment and then compare the results to the ground truth. I will assume no knowledge of the vehicle motion (random walk motion model) and start with an initial guess for the state.

1. Ensure "generate_measure.m" and "Ali_V2I_run.m" are in the same path 
2. Run "Ali_V2I_run.m"

