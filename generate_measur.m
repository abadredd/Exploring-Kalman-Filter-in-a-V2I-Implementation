%_________________________________________________________________________
%   Course: ECE 531
%   Author: Ali Badreddine
%   Date: 08/09/2020
%_________________________________________________________________________

function [z, z_no_noise] = generate_measur (gt, rt, NoiseCov)
%call function to generate noisy measurement data based on ground truth,
%noise covarience, and radar coerdinates...
%data will be returned in the format below:
%column 1 = distance from vehicle to radar tower 1
%column 2 = distance from vehicle to radar tower 2
%column 3 = distance from vehicle to radar tower 3
%sample from a zero mean gaussian with covariance "NoiseCov" 

z=[];
z_no_noise=[];

L = chol(NoiseCov, 'lower');

for j = 1:size(rt,2)
for i = 1:size(gt.x,2)
  
z(i,j) = norm([gt.x(i); gt.y(i)] - rt(:,j)) ;

z_no_noise(i,j) = norm([gt.x(i); gt.y(i)] - rt(:,j));

end
end

%Apply noise to measurements
for k = 1:size(z,1)

    noise = L * randn(3,1); 
    
    z(k,:) = z(k,:) + noise';
    
end

end






