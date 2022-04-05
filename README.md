# Exploring-Kalman-Filter-in-a-V2I-Implementation
Simulate a V2I environment by generating noisy measurements from 3 separate radar towers that will return the distance of the vehicle from each radar tower at each time step. These noisy measurements will then be propagated through a KF to estimate the current state of the vehicle in the 2D environment and then compare the results to the ground truth. I will assume no knowledge of the vehicle motion (random walk motion model) and start with an initial guess for the state.

1. Open Matlab and load ground truth tragectory 'transformations&GT07.mat' whcih was taken from KITTI data set and parsed to generate transformations as well as ground truth data for map sequence 07 (http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
2. Ensure "generate_measure.m" and "Ali_V2I_run.m" are in the same path 
3. Run "Ali_V2I_run.m"

#RESULTS

![image](https://user-images.githubusercontent.com/49213550/161667956-ac279c37-42a0-4acb-a55e-43181921276f.png)

*Ground Truth Trajectory & Radar Tower Positions*

![image](https://user-images.githubusercontent.com/49213550/161668156-de02fffc-6a24-4a24-9f18-28233cb74e0f.png)

*Kalman Filter Approximation va Ground Truth*

![image](https://user-images.githubusercontent.com/49213550/161668218-a9bacbd9-468e-4212-8c21-856846ac0f45.png)
![image](https://user-images.githubusercontent.com/49213550/161668258-69f59f73-2ed3-4958-9ecd-91d08e55deef.png)

*Deviation from ground truth for x, and y coordinates*




