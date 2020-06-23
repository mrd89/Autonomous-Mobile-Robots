function[lidarXY] = lidar_range2xy(lidarR, robotRad, angRange, N)
% LIDAR_RANGE2XY: convert lidar measurements (range & bearing) into x/y
% coordinates (in robot local frame)
%
%   LIDARXY = LIDAR_RANGE2XY(LIDARR,ROBOTRAD,ANGRANGE,N) returns the
%   x/y coordinates (in robot local frame) of lidar measurements.
%
%   INPUTS
%       lidarR      1-by-N vector of scan ranges (meters)
%       robotRad    robot radius (meters)
%       angRange    total angular field of view of lidar (radians)
%       N           number of lidar points in scan
%
%   OUTPUTS
%       lidarXY     2-by-N matrix of x/y scan locations
%
%   NOTE: Assume lidar is located at front of robot and pointing forward
%         (along robot's x-axis).
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   DANIEL,MATTHEW

% define theta to start at - half the range, to half the range with
% a total of N steps
theta = linspace(-angRange/2 , angRange/2,N);

lidarXY = zeros(2,N); % allocate matrix

%simple coordinate change
lidarXY(1,:) = lidarR .* cos(theta) + robotRad; % calculate the x
lidarXY(2,:) = lidarR .* sin(theta); % calculate y



end

