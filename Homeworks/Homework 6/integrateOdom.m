function[future_pose] = integrateOdom( pose , d, phi)
% integrateOdom takes in position pose, d distance , and phi and outputs
% the odometry to output configuarion of the robot.

%pose   	3-by-1 pose vector in global coordinates (x,y,theta)
% d         is the distance in meters (type double)
% phi       is the angle phi (type double)

%future_pose is a 3x1 vector with robot odometry as output 
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May



if (phi ~= 0) % ensure no divide by 0
    
    %calculate radius of the arc
    r = abs(d/phi);
    
    %solve for length of the triangle
    l = abs(2*r * sin(phi/2));
    
    % convert to dx and dy 
    dx = l * cos(phi/2+pose(3));
    dy = l* sin(phi/2+pose(3));
else
    %if phi =0, there is no arc, and so the distance traveled will be in
    %the x direction with body fixed frame
    dx = d*cos(pose(3));
    dy = 0;
end

%add the position to the previous position
x = dx + pose(1);
y = dy + pose(2);

%output future position 
future_pose = [x;y;phi+pose(3)];

end
