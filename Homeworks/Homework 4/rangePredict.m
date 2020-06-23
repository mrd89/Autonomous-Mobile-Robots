function [range] = rangePredict(robotPose,map,sensorOrigin,angles)


%max sense is the distance in the Y direction the sensor will read
maxsense = 100;

%Initialize the range vector
range = zeros(1,length(angles));

%calculate position of sensor, Global Frame
sensor_pos = robot2global(robotPose,sensorOrigin);


%loop through all map locations
for i=1:length(angles)
    
    %Choose the [x2,y2] based on the angle
    if (angles(i) < 0 || angles(i)>pi)
        
        % move the y2 to maxSense distance behind the robot
        y2 = sensorOrigin(2) - maxsense;
        
        % given y distance, calculate X such that the angle becomes the
        % target angle.
        x2 = (y2-sensorOrigin(2))/tan(angles(i)) + sensorOrigin(1);
        
    elseif (angles(i) <= pi && angles(i) > 0)
        
        % Assume Y is maxsense forward
        y2 = sensorOrigin(2) + maxsense;
        
        % given y distance, calculate X such that the angle becomes the
        % target angle.
        x2 = (y2-sensorOrigin(2))/tan(angles(i)) + sensorOrigin(1);
        
        %else the angle is 0
    else
        
        %angle is 0, so y stays at sensor Origin
        y2 = sensorOrigin(2);
        
        %all in y direction, maxsense forward
        x2 = sensorOrigin(1) + maxsense;
    end
    
    %Convert Robot to Global Coordinates
    pt2 = robot2global(robotPose,[x2,y2]);
    
    %initiate minimum counter
    min =intmax;
    
    %loop through all map locations, calculate the intersection of each
    for j=1:length(map(:,1))
        
        %used for debugging, makes code cleaner by defining the variables
        %here instead of in function call
        a = map(j,1);
        b = map(j,2);
        c = map(j,3);
        d = map(j,4);
        
        %call intersection helper function to find distance
        [isect,x,y,ua] = intersectPoint(sensor_pos(1),sensor_pos(2),pt2(1),pt2(2),a,b,c,d);
        
        %eficiency code, only calculate if line intersected
        if (isect)
            %calculate distance from the sensor
            dist = sqrt(  (abs(x-sensor_pos(1)))^2 + (abs(y - sensor_pos(2)))^2    );
        end
        
        
        % find minimum wall distance
        if (isect && dist<min)
            
            %update the minimum if 1,2 satisfied
            min = dist;
        end
        
    end
     %if the minimum was updated (ergo, line intersected)
    if (min~=intmax)
        %set the range to be the minimum
        range(i) = min;
    else
        
        %if there was no wall detected, set the range to be intmax
        range(i) = NaN;
    end
    
    
    
end
end

