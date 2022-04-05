%_________________________________________________________________________
%   Course: ECE 531
%   Author: Ali Badreddine
%   Date: 08/09/2020
%_________________________________________________________________________

clear all 
close all

%base ground trouth tragectory on KITTI data set where I parse and generate
%transformations as well as ground truth data for map sequence 07
%http://www.cvlibs.net/datasets/kitti/eval_odometry.php
load('transformations&GT07.mat')

%load only the 2D ground truth data, x and y coordinates (ground truth data originaly saved as an SE3 matrix concatinated as a row)
gt = [];
for i = 1:size(T_Pose_GT,1)
gt.x(i) = T_Pose_GT(i,5);
gt.y(i) = T_Pose_GT(i,9);
end

figure(1);
%plot ground truth map
plot(gt.x,gt.y,'linewidth', 2);
hold on
axis([-200 50 -20 15])

%% Generate 3 sets of noisy measurements rempresenting distance of robot/vehicle from traffic radar towers 
%by applying noise to ground truth data above

% x,y coordinates for radar tower 1,2, and 3
%[x1, x2, x3;
% y1, y2, y3]
radartowers = [-60, 20, -150; -5, 10, -15];

%define noise covarience to be applied to ground truth to generate noisy
%measurements
NoiseCov =  diag([0.04^2; 0.04^2; 0.04^2]);

%call function to generate noisy measurement data based on ground truth,
%noise covarience, and radar coerdinates...
%data will be returned in the format below:
%column 1 = distance from vehicle to radar tower 1
%column 2 = distance from vehicle to radar tower 2
%column 3 = distance from vehicle to radar tower 3
%sample from a zero mean gaussian with covariance "NoiseCov" 
[z,z_no_noise] = generate_measur (gt, radartowers, NoiseCov);

%plot radar towers
plot(radartowers(1,:), radartowers(2,:), 'pg', 'MarkerFaceColor', 'magenta', 'markersize', 10);

%% GOAL: Estimate State of the robot using a KALMAN FILTER
%Assumptions:
%[1] Assume a random walk motion model; no knowledge of target motion
%[2] No knowlege of robot's initial starting position other than the noisy
%distance measurements as generated above (distance from vehicle to each radar tower)

%% Find initial guess to be used in the KALMAN FILTER; first pose estamite of robot based on first noisey measurement of distnace

%We can compute the approximate location of the robot intialy by triangulation of that robot in relation to the three reference points (radar towers) 
%in-order to find the approximate initial position.

%Set up a system of two equations where circle1 = circle2 and circle 2 =
%circle 3, these circles constitude a circle with radius = distance measured from robot to radar tower.

syms x y
C1C2 = (x-radartowers(1,1))^2 + (y-radartowers(2,1))^2 - z(1,1)^2 == (x-radartowers(1,2))^2 + (y-radartowers(2,2))^2 - z(1,2)^2;
C2C3 = (x-radartowers(1,2))^2 + (y-radartowers(2,2))^2 - z(1,2)^2 == (x-radartowers(1,3))^2 + (y-radartowers(2,3))^2 - z(1,3)^2;
[A,B] = equationsToMatrix([C1C2, C2C3], [x, y]);
state_init = double(linsolve(A,B));

%% Initialization of KALMAN FILTER system paramteres
A = eye(2);
B = [];
f = @(x) [x(1); x(2)];    %Since we assume a random walk model we propogate the state of the robot as is but with added motion noise
h = @(x,rt) [norm(x-rt(:,1)); norm(x-rt(:,2)); norm(x-rt(:,3))]; %We define the measurement model as a function converting pose to
%distance to landmark which is the tower here; essentialy the relationship
%between the current state and the measurements (which are the distance to each tower here)
H = @(x,rt) [(x(1) - rt(1,1))/((x(1) - rt(1,1))^2 + (x(2) - rt(2,1))^2)^(1/2), (x(2) - rt(2,1))/((x(1) - rt(1,1))^2 + (x(2) - rt(2,1))^2)^(1/2);
              (x(1) - rt(1,2))/((x(1) - rt(1,2))^2 + (x(2) - rt(2,2))^2)^(1/2), (x(2) - rt(2,2))/((x(1) - rt(1,2))^2 + (x(2) - rt(2,2))^2)^(1/2);
              (x(1) - rt(1,3))/((x(1) - rt(1,3))^2 + (x(2) - rt(2,3))^2)^(1/2), (x(2) - rt(2,3))/((x(1) - rt(1,3))^2 + (x(2) - rt(2,3))^2)^(1/2)];
Q =  1e-1 * eye(2);
R = diag([0.04^2; 0.04^2; 0.04^2]);%radio tower coordinates
rt = [-60, 20, -150; -5, 10, -15];

% initialization! 
state = zeros(2,1);
state(1,1) = state_init(1);
state(2,1) = state_init(2);
Cov = 1 * eye(2);

sigma_cont.one = [];
sigma_cont.two = [];

result = state_init;     % state
% sigma_cont.one(1) = 3*sqrt(init.Sigma(1,1));
% sigma_cont.two(1) = 3*sqrt(init.Sigma(2,2));
    
% main loop; iterate over the measurements
for i = 2:size(z,1)
    
    %% Prediction step
    %propogate past state through motion model
    pred_state = f(state);
    %compute covarience
    pred_Cov = A * Cov * A' + Q;
    
    %propogate state through measurement model
    z_hat = h(pred_state,rt);
    
    %% Correction step
    %compute measurement Jacobian based on predicted state
    H_ = H(pred_state, rt);
    
    %find the innovation
    v = z(i,:)' - z_hat;
    S = H_ * pred_Cov * H_' + R;
    %kalman gain
    K = pred_Cov * H_' * (S \ eye(size(S)));
    
    % correct the state prediction based on mesurement innovation and KG
    state = pred_state + K * v;
    
    
    I = eye(size(state,1));
    %update covarience using Joseph form
    Cov = ...
        (I - K * H_) * pred_Cov * (I - K * H_)' ...
        + K * R * K';
    
    %% Plot results and save 3rd sigma (varience) contours for each time stamp for the x and y coordinate respectivley 
    result(:,i) = state;
    plot(result(1,:), result(2,:),'--m');
    
    sigma_cont.one(i) = 3*sqrt(Cov(1,1));
    sigma_cont.two(i) = 3*sqrt(Cov(2,2));
        pause(0.01);
end

title('Odometry Result vs Ground Truth')
legend('Ground Truth', 'Radar Towers', 'Kalman Filter State approximation')

diff = [];
diff.x = result(1,2:end) - gt.x(2:end);
diff.y = result(2,2:end) - gt.y(2:end);

figure(2)
hold on
grid on
plot(diff.x);
plot(sigma_cont.one,'r');
plot(-1.*sigma_cont.one,'r');

title('Deviation of X coordinate from ground truth (Sigma Contour)')
legend('Deviation from GT', 'Sigma Contours')

figure(3)
hold on
grid on
plot(diff.y);
plot(sigma_cont.two,'r');
plot(-1.*sigma_cont.two,'r');

title('Deviation of Y coordinate from ground truth (Sigma Contour)')
legend('Deviation from GT', 'Sigma Contours')

MSE = norm([result(1,2:end);result(2,2:end)] - [gt.x(2:end);gt.y(2:end)])

