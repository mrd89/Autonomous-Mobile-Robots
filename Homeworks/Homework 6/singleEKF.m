function [mu,sig] = singleEKF(mu,sig,data,d,phi,R,Q)
%This program takes in the the data for a single marker with the assumption
%the point has been seen before and is seen now, parent function takes care
%of this
%
% INPUT:
%       mu          K-by-1 length of [x,y,theta,x1,y1,x2,y2...]'
%       sig         2-by-2 covariance matrix of prev 
%       meas        1-by-M measurement input
%       Q           normal distribution covariance
%
% OUTPUT:
%       muNew       updated marker location  
%       sigNew      updates covariance
%
% Written by Matthew Daniel, mrd89@cornell.edu, 2019-May

%find useful sizes of arrays
L = length(mu); %should be 19 for test case
N = L - 3; %should be 16 :)

meas = data(4:end);

%find new position of robot
pose = integrateOdom(mu(1:3),d,phi); %3-by-1


%prediction variables
G = GJacEKF(mu,d,phi);
H = HJacEKF(mu);
h = zeros(N,1);
%h(:) = NaN;

for i=1:2:N
    
    %if the markers has been seen before
    if ~( (0==meas(i)) && (0 ==meas(i+1)))
        
        %estiamted position
        h(i:i+1) = global2robot(pose',([meas(i),meas(i+1)]))';
        
    end %end if
end %end for 



%remove when a marker measurement is [0,0]
indx = [];
for i=1:2:N
    
    %if both measurements are zero
    if ((0==meas(i)) && (0==meas(i+1))) 
        indx = [indx,i,i+1];
        
    end %end if
end %end for all markers

meas(indx) = [] ;
h(indx) = [];
H(indx,:) = [];

Q(indx,:) = [];
Q(:,indx) = [];

mu_bar = [pose;mu(4:end)];

% Predict Step
sig_bar = G * sig * transpose(G) + R;
Kt = sig_bar * transpose(H) * inv(H * sig_bar* transpose(H) + Q);



diff = meas' - h;




mu = mu_bar + Kt * (diff);
sig = (eye(L) - Kt * H)*sig_bar;








end %end function