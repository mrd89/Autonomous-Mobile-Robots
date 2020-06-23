function TestSphereWorldPot(map, goal, c_att, c_rep, Q, points)
% TESTSPHEREWORLD
% Test function for MAE 4180/5180 CS 3758, Homework 7. 
% Plots the potential field.
%
%       TestSphereWorld(map, goal, c_att, c_rep, Q, k, lambda, points)
%
%       INPUTS:
%           map         map of the environment defined by circles.
%                       k x 3 matrix [x_center y_center radius]
%           goal        goal point
%                       1 x 2 array [x_goal y_goal]
%           c_att       scaling factor for the attractive force (potential fun).
%           c_rep       scaling factor for the repulsive force (potential fun).
%           Q           influence range of the obstacles (real number)
%           points      list of points to be evaluate
%                       n x 2 matrix [x, y]
%
%                       
%       OUTPUTS:
%           none
%       Figures Created:
%           Figure 1    Potential field 
%
% Autonomous Mobile Robots
% 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%       POTENTIAL FUNCTION
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% STUDENTS: Call the function potentialPlot to generate the potential field
% plot in the specified points

%I use this in my function
maxWeight = 200;


%decompose
x = points(:,1);
y = points(:,2);

%length
n = length(points(:,1));

%initiate
pot = zeros(n,n);

%loop through all sets of points 
for i=1:n
    for j=1:n
        
        %get pot here
        [pot(i,j),~] = potentialPoint( map, goal, c_att, c_rep, Q, [points(i,1) points(j,2)], maxWeight);

    end
end

%mesh
[x,y] = meshgrid(x,y);

%plot figure
figure
mesh(y,x,pot);

hold on

xlabel('x (m)')
ylabel('y (m)')
title('potential field')

hold off




%END
end

