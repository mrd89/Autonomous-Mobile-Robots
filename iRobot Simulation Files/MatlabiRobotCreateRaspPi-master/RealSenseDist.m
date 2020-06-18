function depth_array = RealSenseDist(serPort)
% RealSenseDist(serPort) returns an array of floats. The first represents
% the time the measurement was taken. The remaining elements represent the
% depth of the point in meters from the camera. depth of the point in meters,
% from the camera. 
%   Inputs: 
%       serPort: port to connect to RaspPi
%   Outputs:
%       depth_array: delay from when image was taken and depth measurement 
%       [10 x 1] matrix of floats. depth_array(1) = delay from image taken, 
%       depth_array(2:10) = depth to point. depth_array(2) is left most 
%       point, depth_array(10) is right most point
%
% Note the minimum effective distance for the depth sensor is 0.175m, and
% the maximum effective distance is ~10 meters. 
 
% Port should be closed. If it is open close it first 
if (strcmp(serPort.status,'open'))
		fclose(serPort);
end 

% Open the port	
fopen(serPort);

warning off
global td
num_points = 10; % delay + 9 distance values

while serPort.BytesAvailable==0
    %pause(0.1); 
end

%Read packet
resp = fread(serPort, serPort.BytesAvailable);

fclose(serPort);

if resp == 99
    disp('No camera connected, cannot call this function')
else
    %convert the response to string
    to_str = char(resp.');
    
    %split by empty space and convert to array of data
    cell_array = strsplit(to_str, ' ');
    
    depth_array = zeros(num_points, 1);
    for i=1:num_points
        depth_array(i) = str2double(cell_array(i));
    end
end
pause(td)

return
end