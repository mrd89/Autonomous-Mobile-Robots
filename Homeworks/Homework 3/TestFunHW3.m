function TestFunHW3(map, measurements, gridSize, mu0)
% Test function for Homework 3.  
% This function checks the student's grid localization and stationary KF implementation.
% 
%   INPUTS
%       map       	The name of the map .mat file (e.g. 'HW3map.mat')
%       measurements	The name of the measurement .txt file (e.g. 'stationary.txt')
%       gridSize 	Number of cells in the X dimension x number of cells in the Y dimension 	1x2 [n m]
%       mu0          The initial position for Stationary KF 2x1
%
%   OUTPUTS
%       The function generates plots as detailed below

%~~~~~~~~~~~~~~~~~~~~~
% Grid localization
%~~~~~~~~~~~~~~~~~~~~~

close all;
% STUDENTS: Create the grid based on the input gridSize
n=gridSize(1); m=gridSize(2);

% initialize the grid location / pdf
pdf_init = zeros(n,m);
bel_init = 1/ (n*m);


% set inital belief
pdf_init(:,:) = bel_init;


% STUDENTS: Call the function gridLocalizationStationary to get the updated pdf
[pdf,cpointx,cpointy] = gridLocalizationStationary(gridSize(1),gridSize(2),map,measurements);

% STUDENTS: Plot the following graphs. Each graph should display the map walls and have a title.
%			- Initial pdf
%			- Final pdf after incorporating all of the measurements



figure
hold on
% use helper function to plot the init probability
plotGridBelief(cpointx,cpointy,pdf_init);
% call helper map drawing function
plotMap(map);
title('Initial Belief of Position');
xlabel('Global Coordinate X (m)');
ylabel('Global Coordinate Y (m)');
hold off

figure
hold on
% use helper function to plot the probability
plotGridBelief(cpointx,cpointy,pdf);

% call helper map drawing function
plotMap(map);

title('Final Belief Plot With Map Overlay');
xlabel('Global Coordinate X (m)');
ylabel('Global Coordinate Y (m)');
hold off

%~~~~~~~~~~~~~~~~~~~~~
% Stationary KF
%~~~~~~~~~~~~~~~~~~~~~

% STUDENTS: Call the function KFStationary to get the updated pdf with the given initial position

[mu,sig]=KFStationary(map,measurements,[1,1]');


% STUDENTS: Plot the following graphs. Each graph should display the map walls and have a title.
%			- Initial pdf (mean and 1-sigma covarience)
%			- Final pdf after incorporating all of the measurements (mean and 1-sigma covarience)

plotOpts = [{'color'},{'b'},{'linestyle'},{'-'},{'linewidth'},{1}];
fig4=figure(4);

% plot for intial
figure(4); hold on;
title('Initial Mean and Vairnace Plot with Map Overlay');
xlabel('Global Coordinate X (m)');
ylabel('Global Coordinate Y (m)');
plotMap(map);
plotCovEllipse(mu0,eye(2),[1,2,3],plotOpts,fig4);
hold off

fig5 = figure(5);
figure(5); hold on;
title('Final Kalman Filter Plot with Map Overlay');
xlabel('Global Coordinate X (m)');
ylabel('Global Coordinate Y (m)');
plotMap(map);
plotCovEllipse(mu,sig,[1,2,3],plotOpts,fig5);
hold off


end
% END
