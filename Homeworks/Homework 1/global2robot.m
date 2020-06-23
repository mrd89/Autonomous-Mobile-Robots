function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
%
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
%
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
%
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Daniel,  Matthew

% uses same transformation matrix as  rob -> global
T = [cos(pose(3)),-sin(pose(3)) pose(1);sin(pose(3)),cos(pose(3)),pose(2);0,0,1]; % error likely here

%  use inverse of transformation matrix to get robot coordinates
temp = inv(T) * [xyG';2];

% decompose into a single 2x1 matrix as ouput of function
xyR = [temp(1);temp(2)];
end
