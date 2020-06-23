function [H] = HJac(robotPose,mu)
%This program takes in the the data for a single marker with the assumption
%the point has been seen before and is seen now, parent function takes care
%of this
%
% INPUT:
%       robotPose   [x,y,t] global coords of robot
%       mu          [x,y] of marker in Global coords

% OUTPUT:
%       H           Jacobian of function for EKF 
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May


%inialize jacobian matrix
H = zeros(2,2);
% 
% %set small number as delta for function
% delta = 10^(-3);
% 
% %find actual baseline measurement
% meas_actual = hFunc(robotPose,mu);
% 
% %calculate the [x,y] for adding a delta to x and y respectively
% dx = hFunc(robotPose, [mu(1) + delta,mu(2)]);
% dy = hFunc(robotPose, [mu(1),mu(2) + delta]);
% 
% %set H to be difference in measurements
% H(1,:) = dx - meas_actual;
% H(2,:) = dy - meas_actual;
% 
% %divide by the small delta
% H = H ./ delta;
% 
% H = transpose(H);

theta = robotPose(3);

H = [[cos(theta), sin(theta)];[-sin(theta),cos(theta)]];




end %end function