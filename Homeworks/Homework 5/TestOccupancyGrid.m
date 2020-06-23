function TestOccupancyGrid(datastore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
% TESTOCCUPANCYGRID 
% Test function for MAE 4180/5180 CS 3758, Homework 5. 
% Plots the final occupancy grids using the bump sensor and using sonar.
% Will create two (2) figures containing the occupany grids.
%
%       TestOccupancyGrid(dataStore,ell_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
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
%           none
%       Figures Created:
%           Figure 1    Occupancy grid using bump sensor
%           Figure 2    Occupancy grid from depth information
%
% Autonomous Mobile Robots

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bump Log Odds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%call the logOddsBump function
[bumpMap,cpointx,cpointy ]= logOddsBump(datastore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY);

%rescale the data from 0 to 1
bumpMap = rescale(bumpMap);


%Commented out as it is to round the data :)
%depthMap = round(depthMap);


%plotting specs here
figure
hold on
plotGridBelief(cpointx,cpointy,bumpMap);
xlabel('X direction (m)')
ylabel('Y direction (m)')
title('Map Occupancy Grid Based on Bump Data');
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Depth Log Odds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%call the logOddsdepth function
[depthMap,cpointx,cpointy]=logOddsDepth(datastore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY );

%rescale the data from 0 to 1
depthMap = rescale(depthMap);

%Commented out as it is to round the data :)
%depthMap = round(depthMap);

%plotting specs below
figure
hold on
plotGridBelief(cpointx,cpointy,depthMap);
xlabel('X direction (m)')
ylabel('Y direction (m)')
title('Map Occupancy Grid Based on Depth Data');
hold off


end %end program

