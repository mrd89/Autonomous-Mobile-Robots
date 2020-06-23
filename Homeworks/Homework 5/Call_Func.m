function Call_Func()
%This program is my function that calls the Test function that is used for
%grading.

global dataStore;
N = length (dataStore.truthPose(:,1));

%Create Step 1 as N/3
Step1.bump      = dataStore.bump(1:floor(N/3),:);
Step1.odometry  = dataStore.odometry(1:floor(N/3),:);
Step1.rsdepth   = dataStore.rsdepth(1:floor(N/3),:);
Step1.truthPose = dataStore.truthPose(1:floor(N/3),:);

%Create Step 1 as N*2/3
Step2.bump      = dataStore.bump(1:floor(2*N/3),:);
Step2.odometry  = dataStore.odometry(1:floor(2*N/3),:);
Step2.rsdepth   = dataStore.rsdepth(1:floor(2*N/3),:);
Step2.truthPose = dataStore.truthPose(1:floor(N/3),:);

%Create Step 1 as N/15
Step0.bump      = dataStore.bump(1:floor(N/15),:);
Step0.odometry  = dataStore.odometry(1:floor(N/15),:);
Step0.rsdepth   = dataStore.rsdepth(1:floor(N/15),:);
Step0.truthPose = dataStore.truthPose(1:floor(N/15),:);

%some parameters to use
l_0 = 0;
NumCellsX = 69;
NumCellsY = 420;
boundaryX = [-2.5,2.5];
boundaryY = [-2.5,2.5];

%here I choose which step I want to send to the test function

%TestOccupancyGrid(Step0,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY);
%TestOccupancyGrid(Step1,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY);
%TestOccupancyGrid(Step2,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY);
TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY);


end
