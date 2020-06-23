function [H] = HJacEKF(mu)
%This program takes in the the data for a single marker with the assumption
%the point has been seen before and is seen now, parent function takes care
%of this
%
% INPUT:
%       mu          1-by-N of state
%
% OUTPUT:
%       H           Jacobian of function for EKF 
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

%size of matrix
N = length(mu);
M = N - 3; %assuming we have x,y,theta for robot in mu...

%initialize matrix
H = zeros(M,N);

%decompose robot pose
robX = mu(1);
robY = mu(2);
robT = mu(3);


%create the columns 1 and 2 for robot x,y
H(1:2:end,1) = -cos(robT);
H(1:2:end,2) = -sin(robT);

H(2:2:end,1) =  sin(robT);
H(2:2:end,2) = -cos(robT);


for i=4:2:N
    
    %decompose marker pose
    markerX = mu(i);
    markerY = mu(i+1);
    
    %create theta jacobian 
    H(i-3,3) = -sin(robT) .* (markerX - robX) + cos(robT) .* (markerY - robY);
    H(i-2,3) = -cos(robT) .* (markerX - robX) - sin(robT) .* (markerY - robY);
    
    
    %jacobian corresponding to marker measurements
    H(i-3,i) = cos(robT);
    H(i-3,i+1) = sin(robT);
    H(i-2,i) = -sin(robT);
    H(i-2,i+1) = cos(robT);
    
end %end for i=4:2:N

end %end of function











