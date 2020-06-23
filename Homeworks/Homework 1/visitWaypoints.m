function[dataStore] = visitWaypoints(CreatePort,DistPort,TagPort,tagNum,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
%
%   dataStore = TURNINPLACE(CreatePort,DistPort,TagPort,tagNum,maxTime) runs
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
% set variables here
epsilon = .2; closeEnough=.1;

%coment out which waypoint as needed
%waypoint =[-3 0; 0 -3; 3 0; 0 3];
waypoint = [-1 0; 1 0 ];


while toc < maxTime
    % loop through number of waypoints
    for gotopt=1:1:size(waypoint(:,1))
        
        %read sensor data
        [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
        
        % decompose data to x,y,theta positions
        x = dataStore.truthPose(end,2);
        y = dataStore.truthPose(end,3);
        theta = dataStore.truthPose(end,4);
        
        %while the distance between the robot / waypoint are greater than
        %closeenough point, keep going toward the waypoint
        while (sqrt((x-waypoint(gotopt,1))^2+(y-waypoint(gotopt,2))^2)>closeEnough)
            
            % calculate dx
            dx = waypoint(gotopt,1) -x;
            dy = waypoint(gotopt,2) - y;
            
            % run linearize on the dx and dy using x,y,theta, and e
            [vx,vy]= feedbackLin(dx,dy,[x,y,theta],epsilon);
            
            %limit the speed of the wheels
            [vxx,vyy] = limitCmds(vx,vy,.5,.13);
            
            %send limited wheel speeds
            SetFwdVelAngVelCreate(CreatePort, vxx,vyy );
            
            % check data points again and set x,y,theta
            [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
            x = dataStore.truthPose(end,2);
            y = dataStore.truthPose(end,3);
            theta = dataStore.truthPose(end,4);
            
        end
        
    end
    
end
% set to zero after it finishes the waypoints and repeats until timeout
SetFwdVelAngVelCreate(CreatePort, 0,0 );

end


