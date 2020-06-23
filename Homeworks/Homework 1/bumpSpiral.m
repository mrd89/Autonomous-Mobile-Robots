function[dataStore] = bumpSpiral(CreatePort,DistPort,TagPort,tagNum,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
%
%   dataStore = backupBump(CreatePort,DistPort,TagPort,tagNum,maxTime) runs
%
%   INPUTStype
%       CreatePort  Create port object (get from running RoombaInit)
%       DistPort    Depth ports object (get from running CreatePiInit)
%       TagPort      Tag ports object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
%
%   OUTPUTS
%       dataStore   struct containing logged data

%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%         rate between sensor readings.
%
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DistPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 5
    maxTime = 500;
end


% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%SetFwdVelAngVelCreate(CreatePort, .25,0);
tic;
% this is to make sure it hits a wall straight on the first time
bumped=0;
SetFwdVelAngVelCreate(CreatePort,.3,0);
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    
    [m,n]=size(dataStore.bump);
    if (dataStore.bump(m,7)|| dataStore.bump(m,2)|| dataStore.bump(m,3))
        bumped=1;
        %move .25m backward
        travelDist(CreatePort,.25,-.25);
        
        %turn 70 degrees
        turnAngle(CreatePort,.2,70);
        
        %move .3 m/s forward at pi/6 rad/s
        [vx,vy] = limitCmds(.3,pi/6,.5,.13); %ensure safely below speed
        SetFwdVelAngVelCreate(CreatePort,vx,vy) %calls safe vels
        
        %after the robot has bumped into something
    elseif(bumped==1)
        [vx,vy] = limitCmds(.3,pi/6,.5,.13); %ensure safely below speed
        SetFwdVelAngVelCreate(CreatePort,vx,vy) %call safe velocity
    end
    
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
