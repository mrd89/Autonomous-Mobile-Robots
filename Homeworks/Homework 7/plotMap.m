function obstacleVerts = plotMap(mapFile, mapLimits, figHdl)
% Draws a map from vertices contained in a text file.
%
% INPUTS:
%       mapFile         String containing the .txt file containing the map vertices (e.g. 'hw7.txt')
%       mapLimits       1-by-4 vector defining the map boundary, where mapLimits = [x_min y_min x_max y_max].
%
% OPTIONAL INPUT:
%       figHdl          Figure handle where the map will be displayed (default: current figure)
%             
% OUTPUTS:
%       obstacleVerts   n-dimensional cell array of obstacle vertices, with the ith entry representing an (m_i+1)-by-2 array of coordinate values 
% 
% Author: Jon DeCastro, Sp'15

if ~exist('figHdl','var') || isempty(figHdl), figHdl = gcf; end

% Read the input file
mapData = dlmread(mapFile);

if rem(size(mapData,2),2)
    error('Map data must contain an even number of columns!')
end

figure(figHdl)
hold on

% Parse the input file
for i = 1:size(mapData,1)
    for j = 1:2:size(mapData,2)
        vx(i,(j+1)/2) = mapData(i,j);
        vy(i,(j+1)/2) = mapData(i,j+1);
    end
    
    % Draw all the required lines
    for j = 1:size(mapData,2)/2-1
        if (vx(i,j+1) == 0 && vy(i,j+1) == 0)  % Last vertex in the row-- close the loop.
            line([vx(i,j) vx(i,1)],[vy(i,j) vy(i,1)],'LineWidth',3,'Color',[0 0 0])
            obstacleVerts{i}(j,:) = [vx(i,j) vy(i,j)];
            j = j-1;
            break
        else
            line([vx(i,j) vx(i,j+1)],[vy(i,j) vy(i,j+1)],'LineWidth',3,'Color',[0 0 0])
            obstacleVerts{i}(j,:) = [vx(i,j) vy(i,j)];
        end
    end
    
    % If the loop hasn't been closed, do so here
    if j == size(mapData,2)/2-1
        line([vx(i,j+1) vx(i,1)],[vy(i,j+1) vy(i,1)],'LineWidth',3,'Color',[0 0 0])
        obstacleVerts{i}(j+1,:) = [vx(i,j+1) vy(i,j+1)];
    end
    obstacleVerts{i}(end+1,:) = [vx(i,1) vy(i,1)];
end

% Define map limits
vxMin = mapLimits(1); 
vyMin = mapLimits(2);
vxMax = mapLimits(3);
vyMax = mapLimits(4);

% Draw the boundary
line([vxMin vxMax],[vyMin vyMin],'LineWidth',3,'Color',[0 0 0])
line([vxMax vxMax],[vyMin vyMax],'LineWidth',3,'Color',[0 0 0])
line([vxMax vxMin],[vyMax vyMax],'LineWidth',3,'Color',[0 0 0])
line([vxMin vxMin],[vyMax vyMin],'LineWidth',3,'Color',[0 0 0])

% Configure the axes
axis([vxMin-5 vxMax+5 vyMin-5 vyMax+5])
axis equal
