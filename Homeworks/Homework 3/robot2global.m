function[xyG] = robot2global(pose,xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
%
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
%
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
%
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Daniel, Matthew



%// create the transform matrix T B-> I using
%T = | R , p |
%    | 0 , 1 |
T = [cos(pose(3)),-sin(pose(3)) pose(1);sin(pose(3)),cos(pose(3)),pose(2);0,0,1];

% create a temp matrix with [x,y,1] as outputs
temp = T* [xyR';1];

% decompose into a single 2x1 matrix as ouput of function
xyG = [temp(1);temp(2)];


end

