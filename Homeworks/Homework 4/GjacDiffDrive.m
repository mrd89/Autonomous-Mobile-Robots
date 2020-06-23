function J = GjacDiffDrive(xt,d,phi)
% Function calculates the Jacobian for the update step of an EKG

%INPUTS
%   xt      Current position [x;y;theta]
%   d       distance
%   phi     angle

%check for case where phi =0
if (phi ==0)
    %calculates jacobian for system
  J= [1,0,-d* sin(xt(3));
   0,1,d*cos(xt(3));
   0,0,1];
else
%calculates jacobian for system
J = [1,0, -2 *d / phi * sin(phi /2 + xt(3)) * sin( phi / 2);
    0,1, 2*d/phi * cos(phi/2 + xt(3)) * sin( phi / 2);
    0,0,1];
end
end
