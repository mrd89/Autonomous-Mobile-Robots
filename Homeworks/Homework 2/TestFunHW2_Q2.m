function [range,depth] = TestFunHW2_Q2(robotPose,map,sensorOrigin,angles)
% Test function for MAE 4180/5180, Homework 2 - Q2.  
% This function checks the student's rangePredict and depthPredict implementation.
% 
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       range       	K-by-1 vector of ranges (meters)
%       depth       	K-by-1 vector of depths (meters)

% STUDENTS: Insert a call to rangePredict, such that the function TestFunHW2_Q2 
% outputs the following variable: range 

range = rangePredict(robotPose,map,sensorOrigin,angles);




% STUDENTS: Insert a call to depthPredict, such that the function TestFunHW2_Q2 
% outputs the following variable: depth 
depth = depthPredict(range,angles);



% Open a figure
figure
hold on

subplot(2,1,1)
hold on
plot(range,'bo')
xlabel('N');
ylabel('range measurement(m)');
ylim([0,2*max(range)])

subplot(2,1,2)
hold on
plot(depth,'ro')
xlabel('N');
ylabel('depth measurement(m)');
ylim([0,2*max(range)])



end
% END