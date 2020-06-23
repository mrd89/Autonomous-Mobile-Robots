function [potential,gradient] = goalWeights( pose,goal,C_att)
% This function taes in the distance and constant, and outputs the
% potential at that point
%
%
% INPUT:
%       pose        [x,y] Location of point on map
%       goal        [x,y] location for robot goal point
%       C_att       Constant used for attractive force
%
% OUTPUT:
%       potential    Output of potential field at that point
%       gradient     [x,y] vector of gradient field at that point

% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu


%calculate weight of that position
potential =C_att * norm(pose - goal)^2  ./ 2;

%calculate weight of gradient
gradient = C_att * (pose - goal);



end %end function