function [G] = GJacEKF(mu,d,phi)
%This program takes in the the data for a single marker with the assumption
%the point has been seen before and is seen now, parent function takes care
%of this
%
% INPUT:
%       mu          1-by-N of state
%       d           distance traveled
%       phi         angle traveled
%
% OUTPUT:
%       G           Jacobian of function for EKF dynamics
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

N = length(mu);

%initiate
G = eye(N);

%check foor phi==0
if (0==phi)
    
    G(1,3) = -d .* sin(mu(3));
    G(2,3) =  d .* cos(mu(3));
    
    
else %find derivative at correct points
    
    G(1,3) = d/phi .* (cos( mu(3) + phi) - cos(mu(3)));
    G(2,3) = d/phi .* (sin( mu(3) + phi) - sin(mu(3)));
    
end%end if phi==0









end %end function