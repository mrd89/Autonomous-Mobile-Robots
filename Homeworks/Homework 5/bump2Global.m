function[bumppose] = bump2Global(pose,bumps)
% this function takes in the current location of the robot in global
% coordinates and outputs the x and y of the robot bump location :)
%
%       INPUTS:
%           pose   [x,y,theta] in global coordiantes
%           bumps  Array of bump locations
%
%       OUTPUTS:
%           bumppose    [x,y] of bump location

%initiate local pose
localPose = [0,0];
%radius of robot
Radius = .2;
%theta angle of bump sensors
t = 32.8477;

% only concerned cells are going to be 1,2, and 6 (right,left, front)

if (bumps(1))
    % calculate x and y based on local coordinates
    x = Radius * cos(-t);
    y = Radius *sin(-t);
    
    %update local position
    localPose = [x,y];
elseif (bumps(2))
    % calculate x and y based on local coordinates
    x = Radius * cos(t);
    y = Radius *sin(t);
    
    %update local position
    localPose = [x,y];
    
elseif (bumps(6))
    % calculate x and y based on local coordinates
    x = Radius;
    y = 0;
    
    %update local position
    localPose = [x,y];
end

%conver from robot to global coords
bumppose = robot2global(pose,localPose);





