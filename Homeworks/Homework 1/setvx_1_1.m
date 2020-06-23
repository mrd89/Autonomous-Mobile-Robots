function[dataStore] = setvx_1_1(CreatePort,DistPort,TagPort,tagNum,maxTime)
% SETVX_1_1 sets the desired velocity to Vx,Vy = [1,1] which uses the
% feedback linearization function to do so
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

tic
while toc < maxTime
    % checks the dataStore variable to read current position
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    
    %calls feeback linearization on the desired V= [1,1]
    [x,y]= feedbackLin(1,1,[dataStore.truthPose(end,2),dataStore.truthPose(end,3)...
        dataStore.truthPose(end,4)],2);
    
    %limits the speed based on robot max =.5m/s
    [xx,yy] = limitCmds(x,y,.5,.13);
    
    %sets velocity and omega based on limited speed 
    SetFwdVelAngVelCreate(CreatePort, xx,yy );
    
    
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
