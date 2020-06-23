function [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore)
% This function tries to read all the sensor information from the Create
% and store it in a data structure
%
%   [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore) runs 
% 
%   INPUTS
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       noRobotCount Number of consecutive times the robot was "lost" by the overhead localization
%       dataStore    struct containing logged data
% 
%   OUTPUTS
%       noRobotCount   Updated number of consecutive times the robot was "lost" by the overhead localization
%       dataStore   Updated struct containing logged data
%
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
    DistPort=Robot.DistPort;
    TagPort=Robot.TagPort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort=Robot;
    DistPort=Robot;
    TagPort=Robot;
end

    % read truth pose (from overhead localization system)
    try
        try
            [px, py, pt] = OverheadLocalizationCreate(Robot);
            poseX = px; poseY = py; poseTheta = pt;
            dataStore.truthPose = [dataStore.truthPose ; ...
                               toc poseX poseY poseTheta];
            noRobotCount = 0;
        catch
            disp('Overhead localization lost the robot!')
            noRobotCount = noRobotCount + 1;
        end
    catch
        disp('Error retrieving or saving overhead localization data.');
    end
    
    %read odometry distance & angle
    try
        deltaD = DistanceSensorRoomba(CreatePort);
        deltaA = AngleSensorRoomba(CreatePort);
        dataStore.odometry = [dataStore.odometry ; ...
                              toc deltaD deltaA];
    catch
        disp('Error retrieving or saving odometry data.');
    end
    
 
    
    % read bump data
    try
        [BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront] = ...
            BumpsWheelDropsSensorsRoomba(CreatePort);
        dataStore.bump = [dataStore.bump ; toc ...
            BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront];
    catch
        disp('Error retrieving or saving bump sensor data.');
    end
    


    %read Real Sense depth data
    try
           
        depth_array = RealSenseDist(DistPort);
        dataStore.rsdepth = [dataStore.rsdepth ; toc depth_array'];
    catch
        disp('Error retrieving or saving RealSense depth data.');
    end

    %read camera data (beacons)
    try
        tags = RealSenseTag(TagPort);
        if ~isempty(tags)
            dataStore.beacon = [dataStore.beacon ; repmat(toc,size(tags,1),1) tags];
        end
    catch
        disp('Error retrieving or saving beacon (AprilTag) data.');
    end
