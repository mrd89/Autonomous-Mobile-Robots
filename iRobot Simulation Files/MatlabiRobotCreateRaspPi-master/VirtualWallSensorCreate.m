function [state] = VirtualWallSensorCreate(serPort);
% Specifies the state of the virtual wall detector sensor
% Note that the force field on top of the Home Base also trips this sensor
% Either triggered or not triggered

% Salzberger 7/6/10

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

fwrite(serPort, [142],'async');
pause(td)
fwrite(serPort,13,'async'); 
pause(td)

virtWall = dec2bin(fread(serPort, 1));
state = bin2dec(virtWall(end));
pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end