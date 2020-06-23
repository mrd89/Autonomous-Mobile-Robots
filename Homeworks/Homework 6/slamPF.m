function [particle] = slamPF(particle,measurements,Noise)
%This program takes in the data of the robot controls and positions of the
%walls to be used in a SLAM problem
%
% INPUT:
%       particle        struct with fields x,y,t,length,marker(1:length)  
%       measurements    [tstep,d,phi,x1,y1...xn,yn] of marker locations
%       Noise           double noise added to particles location
%
% OUTPUT:
%       particle     new particle estimate (same memory, new particle)
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

% set covariance here
Q = .1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% took out noise here, added to each particle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%decompose robot position
x = particle.x+random('normal',0,sqrt(Noise),[1,1]);
y = particle.y+random('normal',0,sqrt(Noise),[1,1]);
theta = particle.t+random('normal',0,sqrt(Noise/2),[1,1]);


robotPose = [x,y,theta];




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INTEGRATE ODOM / SET robotPose 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%decompose d,phi, because timestep = 1s, no need to multiply
d = measurements(2);
phi = measurements(3);

%set robot pose with integrate odom
robotPose = integrateOdom(robotPose,d,phi)';

%update particle location
particle.x = robotPose(1);
particle.y = robotPose(2);
particle.t = robotPose(3);


%number of markers
nMarkers = particle.length;

%inialize weight to 1, multiplied by seen particles previously only (2
%cases)
particleWeight = particle.weight;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOOP THROUGH ALL MARKERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:nMarkers
    
    %decompose measurement of marker
    curr_meas = measurements(2*i+2:2*i+3);
    
    %see if marker has been seen before
    prev_seen = particle.marker(i).seen;
    
    %decompose current position
    marker_pose = particle.marker(i).mu';
    
    
    % seen if NOT BOTH of curr meas==0;
    curr_seen = sum(curr_meas==[0,0]) <2;
    
    
    %decompose covariance matrix
    marker_sig = particle.marker(i).sig;
    
    
    %if never seen before AND seen now
    if ~ prev_seen && curr_seen 
    
        %initlize the mu
        particle.marker(i).mu = hFuncInverse(robotPose,curr_meas);
        
        %find H jacobian
        H = HJac(robotPose,curr_meas);
        
        %find covariance of marker
        particle.marker(i).sig = inv(H) *( eye(2)*Q) * transpose(inv(H));
        
        %set marker to have been seen
        particle.marker(i).seen = 1;
              
     
        %if not seen before and not seen now, ignore
    elseif ~(prev_seen && curr_seen)
        %no update to particle marker
        

         
        
        %if previously seen and currently seen
    elseif prev_seen && curr_seen
        
        %run PF on this marker
        [mu,sig,weight] = markerPF(robotPose,marker_pose,marker_sig,curr_meas,Q);
        
        %save variables to particle
        particle.marker(i).mu = mu;
        particle.marker(i).sig = sig;
        particle.marker(i).weight = weight;

        %multiply all SEEN marker weights to find total particle weight
        particleWeight=particleWeight*weight;
        
        %if previously seen, and not currenltly seen
    elseif prev_seen && ~curr_seen
        %do nothing to particles
        
        %multiply all marker weights to find total particle weight
        particleWeight=particleWeight*particle.marker(i).weight;
        
    else %should never happen, show error
        disp('this should not happen, check IF statment in slamPF')
    end %end if statement for
    

end %end for all markers

%save particle weight
particle.weight = particleWeight;


end %end function