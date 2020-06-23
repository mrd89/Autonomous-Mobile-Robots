function[dataStore] = potentialPlanner(Robot,maxTime)
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
map =load('hw7aSphereMap.txt');


goal = [0,0];

C_att = 100;

C_rep = 5;

Q = 1;

maxWeight = 500;
N=60;

%potentialPlot( map, goal, C_att, C_rep, Q,maxWeight);
%[potField,gradField,Xpoints,Ypoints] = potentialMap( map, N,goal, C_att, C_rep, Q,maxWeight);


closeEnough = .3;
epsilon = .1;




finished = 0;


SetFwdVelAngVelCreate(CreatePort, 0,0);
tic
while toc < maxTime && ~finished
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    
    currX = dataStore.truthPose(end, 2);
    currY = dataStore.truthPose(end, 3);
    
    pose = [currX,currY];
%                         %find nearest xpoint,ypoint index
%                         mini = intmax;
%                         indx = NaN;
%                         for i=1:N
%                             for j=1:N
%                                 dist = sqrt( (currX - Xpoints(i)).^2 + (currY - Ypoints(i)).^2);
% 
%                                 if dist<mini
%                                     mini = dist;
%                                     indx = [i,j];
%                                 end %end if
%                             end %end for j
%                         end %end for i
% 
% 
%                         %get gradient at that index
%                         gradient = -gradField(indx(1),indx(2),:);
    

    [~,gradient] = potentialPoint( map, goal, C_att, C_rep, Q, pose, maxWeight);

    %set point [x,y] at gradient + currpose
%    goalptx = currX + gradField(1);
    %goalptx = currY + gradField(2);
    
    %create vector to point
    
    
    %calculate distance
    goaldist = sqrt( (currX - goal(1)).^2 + (currY - goal(2)).^2);
    
    %check if chose enough to goal
    if goaldist < closeEnough
        finished =1;
        cmdV = 0;
        cmdW = 0;
    else
        
        %else need to move via feedback lin
        [cmdV, cmdW] = feedbackLin(-gradient(1)/10, -gradient(2)/10, dataStore.truthPose(end,4), epsilon);
        
        %limit velocity
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, 0.3, 0.13);
        
        if abs(cmdW)<.001
            cmdW=0;
        end
        
    end
    

    
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
        disp('sent');
    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
