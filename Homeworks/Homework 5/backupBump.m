function [dataStore] = backupBump(Robot,maxTime)
% BACKUPBUMP: Reads data from sensors and moves the robot forward until a
%   bump sensor is triggered, meaning the robot hit a wall. When this
%   happens, the robot backs up and turns.
%   
%   dataStore = BACKUPBUMP(Robot, maxTime)  
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
%   MAE 4180: Autonomous Mobile Robots

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 1500;
end

try 
    % When running with     the real robot, we need to define the appropriate 
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

SetFwdVelAngVelCreate(CreatePort, 0.3,0);
tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    
    % did Robot bump? Check the front, right, and left bump sensors
    front = dataStore.bump(end, 7);
    right = dataStore.bump(end, 2);
    left = dataStore.bump(end, 3);
    
    % Set forward velocity.
    cmdV = 0.3;
    cmdW = 0;
    
    % Limit the velocity in case the velocity exceeds the max velocity. 
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, 0.5, 0.13);
    
    % If one of the bump sensors is triggered, go back 0.25 m and then turn
    % 30 degrees. clockwise.
    if front == 1 || right == 1 || left == 1
        travelDist(CreatePort, cmdV, -0.25);
        
        if (toc>700)
            turnAngle(CreatePort, 0.2, 30);
        else
            turnAngle(CreatePort, 0.2, -30);
        end
        
    end
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

end

