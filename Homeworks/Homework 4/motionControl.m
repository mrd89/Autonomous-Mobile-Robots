function[dataStore] = motionControl(Robot,maxTime)
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
    maxTime = 60;
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




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SETUP DATA STORE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', [],...
    'deadReck',[],...
    'EKFmU',[],...
    'EKFsIGMA',[],...
    'GPS',[]);


map = load('cornerMap.mat');
map = map.cornerMap;
angles = linspace(deg2rad(27),deg2rad(-27),9);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

SetFwdVelAngVelCreate(CreatePort, 0,0);
tic

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   INIT VARS / READ DATA TRUPOSE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
x = dataStore.truthPose(end,2);
y = dataStore.truthPose(end,3);
t = dataStore.truthPose(end,3);
dataStore.EKFmU = (dataStore.truthPose(end,2:4))';
dataStore.EKFsIGMA = [2,0,0;0,2,0;0,0,0.1];
%dataStore.EKFsIGMA = [4, 0, 0; 0, 4, 0; 0, 0,.02];
dataStore.deadReck = (dataStore.truthPose(end,2:4))';
dataStore.GPS= [x,y,t];

sensorOrigin = [0,0.13];



%    H_gps = @(pose) HjacDepth(map,pose,angles,sensorOgigin);
%    H_depth = @(pose) HjacGPS(map,pose,angles,sensorOgigin);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   CHANGE THE METHOD HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%uncomment wanted method, comment others out
%method = 1; % EKF with GPS
method = 2; % EKF with Depth
%method = 3; %PF with Depth

if (method ==1) %EKF with GPS
    
    R = .01 * eye(3);
    Q = .001 * eye(3);
    xt=dataStore.truthPose(end,2:4)';
    Sig =  dataStore.EKFsIGMA;
    Hfunc = @(pose) HjacGPS(map,pose,angles,sensorOrigin);
    hfunc = @(pose) hGPS(pose);    
    
elseif (method ==2) %EKF with Depth
    R = .001 * eye(3);
    Q = .01 * eye(9);
    xt=dataStore.truthPose(end,2:4)';
    Sig = dataStore.EKFsIGMA;
    %  Main_Func = @(Ht,xt,Sig,R,Q,d,phi) EKF(map,angles,measurements,SensorOrigin,Ht,xt,Sig,R,Q,d,phi);
    Hfunc = @(pose) HjacDepth(map,pose,angles,sensorOrigin);
    hfunc = @(pose) depth_and_range(pose,map,sensorOrigin,angles);
    
else %PF with Depth
    
    Q = .001;
    Hfunc = @(pose) HjacDepth(map,pose,angles,sensorOrigin);
    NN = 10;
    %use mesh grid please
    
    %pset_x = linspace(-5,0,
   % y is -5,5
    %pset=[]
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN FUNCTION RUNNING TILL MAX TIME
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MOTION OF ROBOT DETERMINED HERE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [m,n]=size(dataStore.bump);
    if (dataStore.bump(m,7)|| dataStore.bump(m,2)|| dataStore.bump(m,3))
        %move .25m backward
        travelDist(CreatePort,.25,-.25);
        
        %turn 30 degrees
        turnAngle(CreatePort,.15,30);
        
        %continue moving, speed is safe because .35 <.5 and no rotation
        SetFwdVelAngVelCreate(CreatePort,.35,0)
    else
        %continue moving, speed is safe because .25 <.5 and no rotation
        SetFwdVelAngVelCreate(CreatePort,.25,0)
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   THIS IS THE IF STATMENT WHICH DETERMINES WHICH METHOD,1/2 = EKF, 3=PF
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(method ==1 || method ==2)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CALCULATE THE POSE,D,PHI, OF ROBOT
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %update new position from dataStore
    x = dataStore.truthPose(end,2);
    y = dataStore.truthPose(end,3);
    t = dataStore.truthPose(end,3);
    disp('hey');
    
    
    % set pose, D, and phi
    pose = [x,y,t];
    d = dataStore.odometry(end,2);
    phi =dataStore.odometry(end,3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ADD NOISE TO GPS DATA
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gps(1) = x + normrnd(0,0.01);
    gps(2) = y + normrnd(0,0.01);
    gps(3) = t + normrnd(0,0.01);
    dataStore.GPS = [dataStore.GPS;gps];
    
    
    
    % SET DEAD RECK BASED ON INTEGRADE ODOM
    dataStore.deadReck = [dataStore.deadReck,integrateOdom(pose,d,phi)];
    
    if (method==1)
        meas = gps';
    elseif (method ==2)
        meas = dataStore.rsdepth(end,3:11)';
    end
    

    [xt,Sig]=EKF(xt,Sig,meas,Q,R,d,phi,hfunc,Hfunc);

    dataStore.EKFsIGMA=[dataStore.EKFsIGMA,Sig];
    dataStore.EKFmU=[dataStore.EKFmU,xt];
    pause(.25);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHOD 3 IS PARTICLE FILTER TO RUN
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (method ==3)
        
        
        %oof
        
    end
    
        
end


% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0);

end



