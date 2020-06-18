function [state] = CliffRightSensorRoomba(serPort);
%[state] = CliffRightSensorRoomba(serPort)
% Specifies the state of the cliff right sensor
% Either triggered or not triggered

% By; Joel Esposito, US Naval Academy, 2011
% Modified by: Chuck Yang, ty244, 2012
%Initialize Preliminary return values
state = nan;

try

%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end
warning off
global td

fwrite(serPort, [142 12]);

CliffRight = dec2bin(fread(serPort, 1));
state = bin2dec(CliffRight(end));
pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end