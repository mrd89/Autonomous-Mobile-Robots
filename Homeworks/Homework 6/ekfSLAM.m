function [mu,sig,trajectory] = ekfSLAM(inData)
%This program takes in the data of the robot controls and positions of the
%walls to be used in a SLAM problem
%
% INPUT:
%       iNData        K-by-L [t,V,W, x1,y1...x8,y8] input data
%
% OUTPUT:
%       mu              final state of the system
%       sig             final cov of the system
%       trajectory      trajectory of system

% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May


% length of vector
K = length(inData(:,1));
L = length(inData(1,:));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SETUP VARIABLES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R = zeros(L,L); %19 by 19 with test data

R(1,1:3) = [0.01  , 0.001 , 0.002];
R(2,1:3) = [0.001 , 0.01  , 0.002];
R(3,1:3) = [0.002 , 0.002 , 0.015];


%number of markers is number of columns - 3 (time step,V,W)
nMarkers = (L - 3) ./2;


Q = .1 .* eye(nMarkers*2); %should be 16 here


% Create covariance matrix
sig = 10 * eye(L); %19x19 for test data (8 markers

%very sure of init pose as 0,0,0
sig(1,1) = .001;
sig(2,2) = .001;
sig(3,3) = .001;

mu = zeros(L,1);


%init trajectory of highest weighted particle
trajectory = zeros(K,2);

%for all measurements
for i=1:K
    
    %calculate dt for each time step
    if (1==i)
        dt = 0;
    else
        dt = inData(i,1) - inData(i-1,1);
    end
    
    %decompose d, phi
    d = inData(i,2) * dt;
    phi = inData(i,3) * dt; 
    
    %use integrade odom to find new position
    %currPose = integrateOdom(mu(1:3),d,phi);
    
    meas = inData(i,:);
    
    [mu,sig] = singleEKF(mu,sig,meas,d,phi,R,Q);
    
    trajectory(i,1:2) = mu(1:2);
    
    
end %end for all measurements, i:k

end %end function
