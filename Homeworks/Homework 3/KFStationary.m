function [ut,Sig] = KFStationary(map,sReadings,mu0);
% This function takes in the map, sensor readings, and initial positition
% and runs through all of the sensor readings. At each step, we run a
% Kalman filter and get a new updated position and variance calcualted at
% each step. 

% *INPUTS*
% map           Map environment [x1,y1,x2,y2]
% sReadings     sensor readings as an array size k x 4 in  NESW
% mu0           initial position [2 by 1]

% *OUTPUT*
% ut            is the final estimated  position [2 by 1]
% Sig           final variance of the robot location


% ***CODE BELOW HERE ****
% B matrix is 0
B = 0;

% set covar to 0 
epsilon = 0;
% initial position given
ut = mu0;

%Set R and Sigma matrix
Sig = eye(2).*50;
R = 0;
U=0;

% for all readings
for i=1:length(sReadings(:,1))
    
    
    % need to reinit the matrices as they are modified later on
    %Initilize The Matrix Q
    Q = [.1,0,0,0;
         0,.3,0,0;
         0,0,.1,0;
         0,0,0,.3];
    
     % A Matrix is Identity
     A = eye(2);
     
     %Four Directions In Order NESW
     angles = [0,3*pi/2, pi,pi/2];
     
     %Init C Matrix
     C = [0,-1;-1,0;0,1;1,0];
    

    %prediction step
    u_bar = A*ut + B * U + epsilon;
    sig_bar = A*Sig*A' + R;
    
    % find the curent readings
    cur_reading= sReadings(i,:);
    
    %indx becomes the locations that are NaN of cur_readings
    indx = find(isnan(cur_reading));
    
    %remove correct rows and columns (as needed due to NaN) for matrix 
    %multiplication stage
    Q(indx,:) = [];
    Q(:,indx) = [];
    C(indx,:) = [];
    angles(indx) = []; % remove angles, no need to call extra
    cur_reading(indx)=[]; % remove current readings
    
    %calculate range with rangePredict (corrected)
    range = rangePredict([ut(1),ut(2),pi/2],map,[0,0],angles);
    
    %calculate the difference
    difference = cur_reading-range;

    
    
    %Kalman Gain calc
    kGain = sig_bar * C' *inv( C* sig_bar*C' + Q);
    
    
    %Update Step
    ut = u_bar + kGain * (difference');
    Sig = (eye(2)-kGain*C) * sig_bar;
   
    
end




