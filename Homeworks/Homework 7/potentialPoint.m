function [potField,gradField] = potentialPoint( map, goal, C_att, C_rep, Q, Loc, maxWeight)
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
%       Loc         [x,y] Location of potential field and gradient
%       maxWeight   Maximum number weight can be

% OUTPUT:
%       potField    Output of potential field 
%       gradField   [x,y] Output of gradient field

% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu


%IF YOU ARE READING THIS: sorry I abstracted this program out so much, I
%misread the prompt and thought this was the program that called the
%complete map and output the potential field and gradient and not just at a
%single point...I have trouble reading a lot

%Please see my subfunctions goalWeights and obstacleWeights...I took the
%effort to write them so I may as well keep them right? 


%find potential and gradient of goal attractive force
[goalPot,goalGrad] = goalWeights(Loc,goal,C_att);


%find potential and gradient of obstacles
[obstaclePot,obstacleGrad] = obstacleWeights(Loc,map(2:end,:),C_rep,Q,maxWeight);

%find potential of boundary
[boundPot,boundGrad] = boundWeights(Loc,map(1,:),C_rep,Q,maxWeight);


%sum dis her
potField =  goalPot +  obstaclePot  +boundPot;
gradField = goalGrad + obstacleGrad +boundGrad;



end %end function call
