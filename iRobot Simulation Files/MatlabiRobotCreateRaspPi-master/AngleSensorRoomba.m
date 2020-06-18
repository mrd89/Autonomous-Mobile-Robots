function [AngleR] = AngleSensorRoomba(serPort);
%[AngleR] = AngleSensorRoomba(serPort)
% Displays the angle in radians and degrees that Create has turned since the angle was last requested.
% Counter-clockwise angles are positive and Clockwise angles are negative.


% By; Joel Esposito, US Naval Academy, 2011
% Modified by: Chuck Yang, ty244, 2012

%Initialize preliminary return values
AngleR = nan;

try
   
%Flush Buffer    
flushinput(serPort);

warning off
global td

fwrite(serPort, [142 20]);
while serPort.BytesAvailable==0
    %pause(0.1); 
end
AngleR = fread(serPort, 1, 'int16')*pi/180;
pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end