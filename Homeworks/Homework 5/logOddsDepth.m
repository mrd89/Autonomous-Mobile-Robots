function[Map,cpointX,cpointY] =  logOddsDepth(datastore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY )
% logOddBump function takes in the depth and location data from the robot
% and creates an estimate occupancy grid of the map based on that data
% output as a matrix of size specified in the inputs :)
%
%       INPUTS:
%           dataStore   struct from running SimulatorGUI
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
noHitNum = intmax/4;

%initialize the occupancy grid to be grey...
Map = zeros(NumCellsX,NumCellsY);
Map(:,:) = intmax/2;

%angles of range sensor (left <positive> to right <negative> readings)
angles = deg2rad(linspace(27,-27,9));
sensorOrigin = [.13,0];




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
M = 9; %number of range readings currently. Not robust :(

% for all truthpose measurements
for i=1:N
    % retreive the current position and range readings
    curr_pose = datastore.truthPose(i,2:4);
    curr_depth = datastore.rsdepth(i,3:11);
    
    %determine wall local coordinates
    localRangeX = curr_depth .* cos(angles) + sensorOrigin(1);
    localRangeY = curr_depth .* sin(angles) + sensorOrigin(2);
    
    %delete / initiate the vector of free cells
    freeCells = []; 
    %for all range measurements
    for j=1:M
        
        
        %find global pose of sensor reading from range
        wallPose = robot2global(curr_pose,[localRangeX(j),localRangeY(j)]);
        
        %find index of wall
        [tailx,taily] = findCurrentGrid(wallPose,cpointX,cpointY,NumCellsX,NumCellsY);
        [headx,heady] = findCurrentGrid(curr_pose,cpointX,cpointY,NumCellsX,NumCellsY);
        
        %add probability to wall cell
        Map(tailx,taily) = Map(tailx,taily) + wallHitNum - l_0;
        
        % use Bresenham's line algo (helper function) to find all of the grids
        % that are free and thus subtract (ensure none coincide with the wall)
        curr_free = findFreeCells([headx,heady],[tailx,taily]);
        

        %if length of the findFreeCells is greater than 2 (used the algo)
       % wut = length (curr_free(1,:));
        if (length (curr_free(1,:))>2)
            
            %remove head and tail from vector
            curr_free = curr_free(:,2:end-1);
        else
            %else, need to make it an empty matrix
            curr_free=[];
        end
        
        %add the head back to the array
        curr_free = [curr_free,[headx;heady]];
        
        %curr_free is 2 by n matrix
        freeCells = [freeCells,curr_free];
                
    end % end of reading range sensor
    
    %delete 
    freeCells = (unique(freeCells','rows'))';
    
    %subtract free cells probability by a set amount
     for k =1:length(freeCells(1,:))
        
        %subtract unique values by noHitNum
        Map(freeCells(1,k),freeCells(2,k)) = Map(freeCells(1,k),freeCells(2,k)) -noHitNum - l_0;
    end
    
end % end reading loop, i






