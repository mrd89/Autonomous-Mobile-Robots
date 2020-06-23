function[dataStore] = rrtPlanner(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
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
    maxTime = 500;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CREATE POTENTIAL / GRADIENT FIELD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    pause(.1);
temp = load('cornerMap.mat');
map = temp.map;

goal = [2,2.5];

delta = 5;
    currX = dataStore.truthPose(end, 2);
    currY = dataStore.truthPose(end, 3);
    
    pose = [currX,currY];

vart = buildRRT(map,mapsize,pose,goal,delta,robRad);


mapH = plotMapMatt(map);

pathHandle = plot(vart(:,1),vart(:,2),'r');

legend([pathHandle],'RRT Path Found')

xlabel('x (m)')
ylabel('y (m)')

closeEnough = .3;
epsilon = .1;




finished = 0;


SetFwdVelAngVelCreate(CreatePort, 0,0);
tic
while toc < maxTime && ~finished
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    for gotopt=1:1:size(waypoint(:,1))
        
        %read sensor data
        [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
        
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
            [vxx,vyy] = limitCmds(vx,vy,.4,.13);
            
            %send limited wheel speeds
            SetFwdVelAngVelCreate(CreatePort, vxx,vyy );
            
            % check data points again and set x,y,theta
            [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
            x = dataStore.truthPose(end,2);
            y = dataStore.truthPose(end,3);
            theta = dataStore.truthPose(end,4);
            
        end
        
    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
