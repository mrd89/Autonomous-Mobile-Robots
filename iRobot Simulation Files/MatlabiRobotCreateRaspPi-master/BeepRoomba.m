function [] = BeepRoomba(serPort);
%[] = BeepRoomba(serPort)
% Plays a song made by RoombaInit command.

% By; Joel Esposito, US Naval Academy, 2011
% sing it
try
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

global td
fwrite(serPort, [141 1])
pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end