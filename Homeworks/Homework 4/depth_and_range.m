function [depth] = depth_and_range(pose,map,sensorOrigin,angles)
% depthPredict takes in the range and angles of the sensor inputs after
% being processed by rangePredict and then outputs an estimated depth based
% on the two inputs.
%INPUTS
%range      vector of length N with range readings
%angles     vector of length N with angle readings

%Calculate Range Prediction
range= rangePredict(pose,map,sensorOrigin,angles);

%initiate the matrix
[depth] = zeros(length(range),1);

%loop through all angles
for i=1:length(angles)
    %calculate the depth based on trig
    depth(i) = range(i) * cos(angles(i));
end

end