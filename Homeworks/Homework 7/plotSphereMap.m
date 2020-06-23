function [figHandle] = plotSphereMap(mapFile,nPoints)
% Plots the map from a matrix file of the map and outputs the handle of the
% plot for use in the parent function
%
% Must use HOLD ON, and FIGURE before using this function
% 
% INPUT:
%       newNode     k-by-3 matrix of the sphere world map
%       nPoints     (int) points on each circle
%       specs       specs of plot, e.g. '-' etc / delete / 
%
% OUTPUT:
%       figHandle   1-by-k vector of figure handles 


k = length(mapFile(:,1));

    figHandle = circle([mapFile(1,1),mapFile(1,2)],mapFile(1,3),nPoints,'r-');

for i=2:k
    
    h = circle([mapFile(i,1),mapFile(i,2)],mapFile(i,3),nPoints);
    figHandle = [figHandle,h];
   
end % end for loop



end
