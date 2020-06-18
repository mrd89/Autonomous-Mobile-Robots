function port = CreatePiInit(remoteHost)
%port = CreatePiInit(remoteHost)
% 'port' is the tcp/ip port for commanding create
%
% This file initializes tcp/ip port for use with iRobot Create
% remoteHost is the ip address of the beagleboard
% ex. CreateBeagleInit('192.168.1.141') sets ip = '192.168.1.141'
% copy this file along with PacketType.m into MatlabToolBoxiRobotCreate
%
% The tcp/ip server must be running on the Raspberry Pi before running this
% code.
%
% If you receive the error "Unsuccessful open: Connection refused: connect"
% ensure that the server code is running properly on the Raspberry Pi
%
% An optional time delay can be added after all commands
% if your code crashes frequently.  15 ms is recommended by iRobot
%
% By: Chuck Yang, ty244, 2012
% Modified By: Alec Newport, acn55, 2018

global td
td = 0.015;

CreatePort = 8865; % TCP
DistPort = 8833; % UDP
TagPort = 8844; % UDP

% Open SSH connection to the host
InitSSH_Connection(remoteHost);
% Patience
pause (3);

% use TCP for control commands and data from the Create
port.create = tcpip(remoteHost, CreatePort, 'inputbuffersize', 64);

% use UDP for distance and tag reading
port.dist = udp(remoteHost, DistPort, 'LocalPort', DistPort);
port.tag = udp(remoteHost, TagPort, 'LocalPort', TagPort);

port.dist.ReadAsyncMode = 'continuous';
set(port.dist,'Timeout',1);
port.dist.inputbuffersize = 512;

port.tag.ReadAsyncMode = 'continuous';
set(port.tag,'Timeout',1);
port.tag.inputbuffersize = 512;

warning off

disp('Opening connection to iRobot Create...');
	fopen(port.create);
	pause(0.5)
% udp ports are opened and closed in the tag and dist functions

%% Confirm two way connumication
disp('Setting iRobot Create to Control Mode...');
% Start! and see if its alive
fwrite(port.create,128);
pause(0.1)

% Set the Create in Full Control mode
% This code puts the robot in CONTROL(132) mode, which means does NOT stop 
% when cliff sensors or wheel drops are true; can also run while plugged 
% into charger
fwrite(port.create,132);
pause(0.1)

% light LEDS
fwrite(port.create,[139 25 0 128]);

% set song
fwrite(port.create, [140 1 1 48 20]);
pause(0.1)

% sing it
fwrite(port.create, [141 1])

pause(0.1)

end