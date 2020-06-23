function [minx,miny]= findCurrentGrid(pose,X,Y,N,M)
%this function finds the nearest grid to the current pose of any object
%sent to this function. Outputs the index of the nearest grid location
%
%       INPUTS:
%           pose   [x,y] in global coordiantes
%           X   Center point of X grid
%           Y   Center point of Y grid
%           N   number of cells in the x direction (int)
%           M   number of cells in the y direction (int)
%
%       OUTPUTS:
%           minx    The X index closest to the point(int)
%           miny    The Y index closest to the point(int)

%find the minimum distance
min = intmax; 
minx=NaN;
miny=NaN;

for i=1:M
    for j=1:N
        %Calculate the distance between the two points
        curDist = sqrt ( (X(i) - pose(1)).^2 + (Y(j) - pose(2)).^2);
        
        %if the current distance is less than the minimum
        if (curDist < min)
            
            %min gets current distance
            min = curDist;
            
            %the indexes are saved here
            minx = j;
            miny = i;
        end % end of if statement
    end %end for j loop
end % end for i loop (x)

        
end %end function
