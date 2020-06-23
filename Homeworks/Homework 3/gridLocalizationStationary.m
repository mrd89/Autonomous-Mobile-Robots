function [bel,cpointx,cpointy] = gridLocalizationStationary(n,m,map,sReadings)
% This function takes in the map, sensor readings, and the number of grid
% items and runs through all of the sensor readings. At each step, we run a
%grid localizatoin filter to get a probability density function of the
%location of the robot

% ** INPUT**
%n,m            Int of size of nap, n by m grid
% map           Map environment [x1,y1,x2,y2]
% sReadings     sensor readings as an array size k x 4 in NESW

% ** OUTPUT**
%bel            the belief matrix
% cpointx       center points for x grid
% cpointy       center points for y grid

% sensorORigin is assumed to be 0,0
sensorOrigin = [0,0];


% find the min/max x and y coords to map grid
xmin = min(min(map(:,1)),min(map(:,3)));
ymin = min(min(map(:,2)),min(map(:,4)));
xmax = max(min(map(:,1)),max(map(:,3)));
ymax = max(min(map(:,2)),max(map(:,4)));

%set grid width in x and y direction (x is column, y is row)
grid_width_x = (xmax-xmin)/n;
grid_width_y = (ymax-ymin)/m;

% initialize the grid location / pdf
bel = zeros(n,m);
bel_init = 1/ (n*m);


% set inital belief
bel(:,:) = bel_init;
 

% calculate the center points [x,y]
cpointx = (xmin +grid_width_x/2):grid_width_x:xmax;
cpointy = (ymin +grid_width_y/2):grid_width_y:ymax;

%used in middle for loop as varance
loopVarience = [.1,.3,.1,.3];

%used as direction for NESW
alpha=[0,3*pi/2,pi,pi/2];

%number of measuremts of the readings input
N_measurements = length(sReadings(:,1));

%update the grid the N times
for k=1:N_measurements
    
    %for all x
    for i = 1:n
        % for all y
        for j =1:m
            
            % set current position in grid
            x = cpointx(i);
            y = cpointy(j);
            
            %set probability to be 1
            P=1;
            
            %loop through NESW directions
            for l =1:4
                
                %robot position
                rob = [x,y,pi/2];
                
                % calculate range
                range = rangePredict(rob,map,sensorOrigin,alpha(l)); 
                
                %diff in range
                dr = range - sReadings(k,l);
                
                %if a reading is NaN, ignore and set P = 1
                if (isnan(dr))
                    curprob=1;
                else
                    %calculate the probability that the difference is
                    %correct given the mean and variance
                    curprob =pdf('Normal',dr,0,sqrt(loopVarience(l)));
                end
                 %update current belief
                P = P * curprob;
            end
            
            %update belief for the grid
            bel(i,j) = P*bel(i,j);
        end
    end
    
    %normalize distribution such that sum of P =1
    bel = bel ./ sum(sum(bel));
end

%tranpose the matrix for plotting :)
bel=bel';
end % end of function



