function [potential,gradient] = obstacleWeights(pose,obstacles,C_rep,Q,maxWeight)
% This function taes in the current position and the sphere obstacle and
% calculates the weight of the potential function
%
%
% INPUT:
%       pose        [x,y] Location of point on map
%       obstacle    N-by-3 [x,y,radius] of map obstacles
%       C_rep       Constant used for repulsive force
%       Q           Influence range of obsticals (double or int)
%       maxWeight   Maximum number weight can be

% OUTPUT:
%       potential    Output of potential field at that point
%       gradient     [x,y] gradient field at that point

% Written by Matthew Daniel, 2020-April, mrd89@cornell.edu

%number of obstalces in the map
N = length (obstacles(:,1));

%initalize to zero
potential = 0;
gradient = [0,0];

%for all obstacles
for i=1:N
    
    
    %distance to center of obstacle
    dist =  norm(pose - obstacles(i,1:2));
    
    %direction to obstacle
    dir = -(pose - obstacles(i,1:2));
    
    
    %If distance is less than radius, it is inside the obstacle
    if (dist < obstacles(i,3))
        potential = maxWeight;
        gradient = maxWeight .* dir ./ dist;
        
        %distance is now distance from obstacle boundary
        d = dist - obstacles(i,3);
        
        %if within sphere of influence Q
        if (d <=Q)
            
            %set potential
            potential = potential +  1/2 .* C_rep .* (1/d - 1/Q ) .^2;
            
            % set gradient
            gradient = gradient + C_rep .* (1./Q - 1./d) / d.^2  .* dir ./ dist;
            
        else
            potential = 0;
            
            gradient = [0,0];
            
            
        end
        
        
        %make sure no points are greater than max
        
        if potential > maxWeight
            potential = maxWeight;
        end
        
        if norm(gradient) > maxWeight
            gradient = gradient ./ norm(gradient) * maxWeight;
        end
       
    end

    
end %end for i loop
end  %end function