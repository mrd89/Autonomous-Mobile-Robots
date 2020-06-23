function[Map,cpointX,cpointY] =  logOddsBump(datastore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY )
% logOddBump function takes in the bump and location data from the robot
% and creates an estimate occupancy grid of the map based on that data
% output as a matrix of size specified in the inputs :)
%
%       INPUTS:
%           datastore   struct from running SimulatorGUI
%           l_0         initial log odds
%           NumCellsX   number of cells in the x direction (integer)
%           NumCellsY   number of cells in the y direction (integer)
%           boundaryX   boundary of the environment in the x direction.
%                       1 x 2 array [min_x max_x]
%           boundaryY   boundary of the environment in the Y direction.
%                       1 x 2 array [min_y max_y]
%       OUTPUTS:
%           Map         [x,y] matrix of the map with boundaries



%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup Constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wallHitNum = intmax;
noHitNum = intmax;

%initialize the occupancy grid to be grey...
Map = zeros(NumCellsX,NumCellsY);
Map(:,:) = intmax/20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creates the grid for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%find width of each grid
xWidth = ((boundaryX(2)-boundaryX(1)) / NumCellsY);
yWidth = ((boundaryY(2)-boundaryY(1)) / NumCellsX);

%calculate center points for the grid
cpointX = (boundaryX(1) + xWidth/2) : xWidth:boundaryX(2);
cpointY = (boundaryY(1) + yWidth/2) : yWidth:boundaryY(2);




%length of the for loop
N = length(datastore.truthPose(:,1));

% for all truthpose measurements
for i=1:N
    curr_bump = datastore.bump(i,2:end);
    curr_pose = datastore.truthPose(i,2:4);
    
    % if there was a bump
    if ( sum(curr_bump)>0)
        
        % Converts current location to the location of the wall
        bumpPose = bump2Global(curr_pose,curr_bump);
        
        % helper function gets the x,y index of closest cell location
        [cellx,celly] = findCurrentGrid(bumpPose,cpointX,cpointY,NumCellsX,NumCellsY);
        
        %flipped x and y here to fix my issue earlier (easier than changing
        %later)
        Map(cellx,celly) = Map(cellx,celly) + wallHitNum - l_0;
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ONLY FOR STEP BY STEP TESTING
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %plotGridBelief(cpointX,cpointY,Map);
        
    else %if there was no bump
        
        % helper function gets the x,y index of closest cell location
        
        [cellx,celly] = findCurrentGrid(curr_pose,cpointX,cpointY,NumCellsX,NumCellsY);
        %flipped x and y here to fix my issue earlier (easier than changing
        %later)
        Map(cellx,celly) = Map(cellx,celly)-noHitNum - l_0;
        
    end % end if statement
end % end reading loop, k






