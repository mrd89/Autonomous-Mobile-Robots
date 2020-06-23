function [depth] = depthPredict(range,angles)
% depthPredict takes in the range and angles of the sensor inputs after
% being processed by rangePredict and then outputs an estimated depth based
% on the two inputs.

%range      vector of length N with range readings
%angles     vector of length N with angle readings


%initiate the matrix
[depth] = zeros(length(range),1);


%loop through all angles
for i=1:length(angles)
    
    %calculate the depth based on trig
    depth(i) = range(i) * cos(angles(i));
    
end




end