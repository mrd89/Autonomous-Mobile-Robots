function [mu,sig,weight] = markerPF(robotPose,mu,sig,meas,Q)
%This program takes in the the data for a single marker with the assumption
%the point has been seen before and is seen now, parent function takes care
%of this
%
% INPUT:
%       robotPose   [x,y,t] global coords of robot
%       mu          [x,y] of marker in Global coords
%       sig         2-by-2 covariance matrix of prev 
%       meas        [x,y] in LOCAL coords
%       Q           normal distribution covariance
%
% OUTPUT:
%       muNew       updated marker location  
%       sigNew      updates covariance
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

%save previous state
mu_prev = mu'; %2x1

sig_prev = sig; %2x2

%predicted measurement
pred_Meas = hFunc(robotPose,mu_prev')';

%jacobian of function
H = HJac(robotPose,mu_prev');

% intermediate useful calculation
Kt = sig_prev * transpose(H) * inv(H * sig_prev * transpose(H) + eye(2)*Q);

%find new wall location in global coords
mu = mu_prev + Kt * (meas' - pred_Meas);

%solve for new covariance matrix
sig = (eye(2) - Kt * H) * sig_prev;

%difference between expected and actual measurement
diff = norm(pred_Meas - meas');

%find the weight of the marker
weight = normpdf( diff, 0, sqrt(Q));
%weight = 1/diff;
% weights usually -> 0, thus useles
%weight = weight * 10^(4);


end %end function