function [meas] = hFunc(robotPose,markerPose)
%Takes in the global coord of market and robot and estimates the
%measurement function that the robot would see
%
% INPUT:
%       robotPose    [x,y,t] global coords of robot
%       markerPose   [x,y] global coords of marker
%
% OUTPUT:
%       meas        [x,y] estiamted measurement
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

%find local coords of marker
meas = global2robot(robotPose,markerPose);

end %end function