function [pset,trajectory] = fastSLAM(inData)
%This program takes in the data of the robot controls and positions of the
%walls to be used in a SLAM problem
%
% INPUT:
%       iNData        K-by-L [t,V,W, x1,y1,x2,y2...] input data
%
% OUTPUT:
%       pset            Struct of particles at final timestep

% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

%PLOTTING VARIABLE SET HERE
plotting =0; %1 is plot, 0 is DO NOT plot

if plotting
    figure
    hold on
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% STRUCT PSET DEFINITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x         global x coord
% y         global y coord
% t         global theta coord
% length    number of markers
% weight    weight of a particle = PI (all markers)
% marker(i).mu      contains mu estimate for marker
% marker(i).sig     contains covariance matrix
% marker(i).seen    seen=1 means seen before, seen=0 init
% marker(i).weight  marker weight



% length of vector
K = length(inData(:,1));
L = length(inData(1,:));

%number of markers is number of columns - 3 (time step,V,W)
nMarkers = (L - 3) ./2;

%init trajectory of highest weighted particle
trajectory = zeros(K,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INITIALIZE PARTICLE SET
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%number of particles
N = 500;

%amount of noise aded
Noise = .002;

%Noise = 0 ;

for i=1:N
    
    %global pose is 0 at t=0
    pset(i).x = 0;
    pset(i).y = 0;
    pset(i).t = 0;
    
    %number of markers in length
    pset(i).length = nMarkers;
    
    %weight distributed evenly
    pset(i).weight= 1/N;
    
    %init each marker
    for j=1:nMarkers
        pset(i).marker(j).seen=0;
        pset(i).marker(j).mu =[];
        pset(i).marker(j).sig=[];
        pset(i).marker(j).weight = [];
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%for all measurements
for i=1:K
    
    if plotting
        if i==1
            prev_pose = [0,0];
        else
            prev_pose = [plot_particle.x,plot_particle.y];
        end
    end
    
    curr_meas = inData(i,:);
    
    %for all particles
    for j=1:N
        
        curr_particle = pset(j);
        
        [upd_part] = slamPF(curr_particle,curr_meas,Noise);
        
        pset(j) = upd_part;
        
    end %end for all particles
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % NORMALIZE WEIGHTS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    normFactor = sum([pset(:).weight]);
    
    %make a vector of weights, easy to run randsample on
    weights = zeros(1,N);
    %if norm factor too small, reset particle weights
    if normFactor < 10^(-10)
        for l=1:N
            pset(l).weight = 1/N;
            weights(l) = 1/N;
        end %end for
    else
        %normalize weights here
        for l=1:N
            weights(l) = pset(l).weight ./ normFactor;
            pset(l).weight = weights(l);
        end %end for
    end %enf if norm factor <
    
    
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RESAMPLE PARTICLES
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %oof the inefficiency...
    tempPset = pset;
    
    for p=1:N
        
        %    use randsample to get index of new pset
        index = randsample(N,1,true,weights);
        
        % use index to create new pset
        pset(p) = tempPset(index);
        
    end %end final loop through pset
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PLOTTING HERE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    color = ['r','b','g','c'];
            [~,maxIndex] = max(weights);
        plot_particle = pset(maxIndex);
        
        
        
    if plotting
        


        robHandle = plot([prev_pose(1),plot_particle.x],[prev_pose(2),plot_particle.y],'-k');
        
        for oi =1:plot_particle.length/2
            
            p1 = plot_particle.marker(oi*2-1).mu;
            p2 = plot_particle.marker(oi*2).mu;
            
            if ~( isempty(p1) || isempty(p2) )
            wallHandle = plot([p1(1), p2(1)],[p1(2),p2(2)],color(oi));
            end 
        end      
        
        xlabel('x')
        ylabel('y')
     % legend([robHandle, wallHandle],'Robot Position','wall Location')
        legend('hey boi')

        %close
        
    end %end plotting
    
    
    trajectory(i,:) = [plot_particle.x,plot_particle.y];
    
    
end %end main loop, for i=1:k






end

