function plotGridBelief(coodinatesXOfGrid, coodinatesYOfGrid, pdf)
% plotGridBelief: plot a pdf 
%                          
%   INPUTS
%       coodinatesXOfGrid   x coorindates of grid centers 
%       coodinatesYOfGrid   y coorindates of grid centers
%       pdf                 pdf representing the location of the robot 

% plot the belief
image(coodinatesXOfGrid,coodinatesYOfGrid,pdf,'CDataMapping','scaled')
colormap (flipud(gray))
colorbar
caxis([0 max(max(pdf))])

% flip axis (to make y point up)
ax = gca;
ax.YDir = 'normal';


