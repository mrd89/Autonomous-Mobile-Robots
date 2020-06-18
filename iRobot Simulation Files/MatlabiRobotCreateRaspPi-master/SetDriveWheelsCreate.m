function [] = SetDriveWheelsCreate(serPort, rightWheelVel, leftWheelVel )
%[] = SetDriveWheelsCreate(serPort, rightWheelVel, leftWheelVel )
%  Specify linear velocity of left wheel and right wheel in meters/ sec
%  [-0.5, 0.5].   Negative velocity is backward.  Caps overflow.
%  Note that if you prefer to specify ANGULAR velocity of wheels (omega in rad/sec) 
%  you must determine radius of wheel (r in meters).   Ex:   rightWheelVel =omegaRight*r
% only works with Creater I think...not Roomba?
% By; Joel Esposito, US Naval Academy, 2011
% Modified by: Chuck Yang, ty244, 2012
% try
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

global td

[computerType, maxSize, endian] = computer;
isLittleEndian = (endian == 'L');
if (isLittleEndian)
rightWheelVel = typecast(swapbytes(int16(min( max(1000* rightWheelVel, -500) , 500))),'uint8');
leftWheelVel = typecast(swapbytes(int16(min( max(1000* leftWheelVel, -500) , 500))),'uint8');
else
rightWheelVel = typecast(int16(min( max(1000* rightWheelVel, -500) , 500)),'uint8');
leftWheelVel = typecast(int16(min( max(1000* leftWheelVel, -500) , 500)),'uint8');
end

fwrite(serPort, [145 rightWheelVel leftWheelVel]);
pause(td)
% catch
%     disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
% end