function [potField,gradField,X,Y] = potentialMap( map, N,goal, C_att, C_rep, Q,maxWeight)
% This function takes in a map, goal, useful constants, and a location and
% ouptuts the potential field and gradiant at the given location for use in
% a parent function
%
%
%
% INPUT:
%       map         M-by-3 matrix of the sphere map
%       N           number (int) of grids
%       goal        [x,y] location for robot goal point
%       C_att       Constant used for attractive force
%       C_rep       Constant used for repulsive force
%       Q           Influence range of obsticals (double or int)
%       maxWeight   Maximum number weight can be

% OUTPUT:
%       potField    Output of potential field
%       gradField   Output of gradient field
%       X           x points
%       Y           y points

% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu



%map length is M
M = length(map(:,1));


%finds min and max x,y coords
xmin = map(1,1) - map(1,3);
xmax = map(1,1) + map(1,3);
ymin = map(1,2) - map(1,3);
ymax = map(1,2) + map(1,3);


%mesh grid formed here
xpoints = linspace(xmin,xmax,N);
ypoints = linspace(ymin,ymax,N);


%initialize fields
potField = zeros(N,N);
gradField = zeros(N,N,2);



% loop through all i,j to make grid
for i=1:N
    for j=1:N
        
        %find curr pose 
        currPose = [xpoints(i),ypoints(j)];
        
        %call point
        [potField(i,j),gradField(i,j,:)] = potentialPoint( map, goal, C_att, C_rep, Q, currPose, maxWeight);
        
        
    end %end for j
end %end for i

%output list of points
X = xpoints;
Y = ypoints;

end %end function


