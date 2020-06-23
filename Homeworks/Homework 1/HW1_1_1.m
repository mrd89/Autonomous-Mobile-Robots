
%close all for testing
close all

%this is for plotting, I use the same exact method for each plot where I
%convert the cell to matrix and then plot for Problem 2/3

%% P1.1 plot
% make a linear range of the angles
load('lidarScan.mat');

theta = linspace(-2/3*pi,2/3*pi , 681); 


%ploting here
figure
hold on
plot(theta, lidarScan,'.');
xlabel('Radians');
ylabel('Distance (m)');
title('Lidar Reading Plot');
hold off

%% P1.2 Plot of Converted points
N = 681;
lidarXY = lidar_range2xy(lidarScan,0.2, 4/3*pi,N);

figure
hold on
plot(lidarXY(1,:), lidarXY(2,:),'.');
xlabel('Distance X axis (m)');
ylabel('Distance Y axis (m)');
title('Lidar Reading Plot 2');

hold off
%% P1.3 Plot
curPose = [3,-4,2/3*pi];



globalXY = zeros(2,N); % initiate the matrix
 
% loop through all points to convert from robot to global coordinates
for i=1:1:N
tempXY = robot2global(curPose,[lidarXY(1,i),lidarXY(2,i)]);
globalXY(1,i)=tempXY(1);
globalXY(2,i)=tempXY(2);

end
% plot the figure
figure
hold on
plot(globalXY(1,:), globalXY(2,:),'.');
xlabel('Distance X axis (m)');
ylabel('Distance Y axis (m)');
title('Global XY Lidar Plot');
hold off


%% P1.4 Lidar to robot Plot

curPose = [0,3,pi/2];



robXY = zeros(2,N);

% loop through to convert all global to robot coordinates
for i=1:1:N
tempXY = global2robot(curPose,[globalXY(1,i),globalXY(2,i)]);
robXY(1,i)=tempXY(1);
robXY(2,i)=tempXY(2);

end

figure
plot(robXY(1,:), robXY(2,:),'.');
xlabel('Distance X axis (m)');
ylabel('Distance Y axis (m)');
title('Robot XY Lidar Plot');


%% Problem 2 Bump Data

load('bumpData_2.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);


figure
plot(XX,YY);
xlabel('X (m)');
ylabel('Y (m)');
title('Plot of backupBump script on box map');


%% Problem 2.4 Own Control Function

load('bumpspiral.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);


figure
plot(XX,YY);
xlabel('X (m)');
ylabel('Y (m)');
title('Plot of bumpSpiral script on box map');


%% problem 3.2 Feedback Lin 
figure
hold on
load('lin_.1.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);
plot(XX,YY);

load('lin_.25.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);
plot(XX,YY);

load('lin_.5.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);
plot(XX,YY);

load('lin_1.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);
plot(XX,YY);



load('lin_10.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);
plot(XX,YY);

xlabel('X (m)');
ylabel('Y (m)');
title('Plot of Linear Feedback Function');
legend('e=.1','e=.25','e=.5','e=.1','e=10');
hold off

%% P4.3 Waypoints

load('waypoint1.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);


figure
plot(XX,YY);
xlabel('X (m)');
ylabel('Y (m)');
title('Plot of Robot Via Waypoint 1');

load('waypoint2.mat');
x = datahistory(:,2);
y = datahistory(:,3);
XX = cell2mat(x);
YY = cell2mat(y);


figure
plot(XX,YY);
xlabel('X (m)');
ylabel('Y (m)');
title('Plot of Robot Via Waypoint 2');



