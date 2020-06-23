function [] = potentialPlot( map, goal, C_att, C_rep, Q,maxWeight)
% This function takes in a map, goal, useful constants, and a location and
% ouptuts the potential field and gradiant at the given location for use in
% a parent function
%
%
%
% INPUT:
%       map         M-by-3 matrix of the sphere map
%       goal        [x,y] location for robot goal point
%       C_att       Constant used for attractive force
%       C_rep       Constant used for repulsive force
%       Q           Influence range of obsticals (double or int)
%       maxWeight   Maximum number weight can be

% OUTPUT:
%       potField    Output of potential field 
%       gradField   Output of gradient field

% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu




% N is number of the grids in this program.
N = 40;

%call potential for whole map :)
[potField,gradField,xpoints,ypoints] = potentialMap( map, N,goal, C_att, C_rep, Q,maxWeight);

%gradField(:,:,1) = rot90(fliplr(gradField(:,:,1)),-1);
%gradField(:,:,2) = rot90(fliplr(gradField(:,:,2)),-1);


%xg = (gradField(:,:,1));
%yg = (gradField(:,:,2));

%gradField(:,:,1) = yg;
%gradField(:,:,2) = xg;
[x,y] = meshgrid(xpoints,ypoints);


figure
mesh(y,x,potField);

hold on
[~] = plotSphereMap(map,100);
xlabel('x (m)')
ylabel('y (m)')
title('potential field')

hold off


figure 
quiver(y,x,gradField(:,:,1),gradField(:,:,2));

hold on

[~] = plotSphereMap(map,100);
xlabel('x (m)')
ylabel('y (m)')
title('gradient field')


end %end fnction call









