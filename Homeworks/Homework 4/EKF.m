function [mu,Sig] = EKF(mu,Sig,meas,Q,R,d,phi,hfunc,Hfunc)
% Extended Kalman Filter function can take in either depth or gps
% coordingates as shown below.
%   INPUTS
%       mu            Mean: 3-by-1 Previous pose vector in global coordinates
%       Sig           Previous covariance
%       meas            Measurement Data
%       Q               Kalman Noise
%       R               Prediction Noise
%       d               Distance travelled
%       phi             Angular displacement
%       ht              Expected measurement function
%       Ht              measurement function

%   OUTPUTS
%       mut             Mean at time t ( After one time step)
%       sigt            Covariance at time t ( After one time step)


% set mu bar
mu_bar= integrateOdom(mu,d,phi);

%call functions to set the Ht, and ht matrix
Ht = Hfunc(mu_bar);
ht=hfunc(mu_bar);

%filter the NaN, there should only be NaN from the expected measurements

%indx becomes the locations that are NaN of cur_readings
indx = find(isnan(meas));

%remove correct rows and columns (as needed due to NaN) for matrix
%multiplication stage
Q(indx,:) = [];
Q(:,indx) = [];

Ht(indx,:) = [];
%Ht(:,indx) = [];

ht(indx)=[];
meas(indx)=[];  % remove NaN readings


%remove measurement ==0 readings
indx = find(meas==0);

%remove correct rows and columns (as needed due to NaN) for matrix
%multiplication stage
Q(indx,:) = [];
Q(:,indx) = [];

Ht(indx,:) = [];
%Ht(:,indx) = [];

ht(indx)=[];
meas(indx)=[];  % remove NaN readings

%Solve for Jacobians Gt
Gt = GjacDiffDrive(mu,d,phi);

% Predict Step
Sig_bar = Gt * Sig * transpose(Gt) + R;
Kt = Sig_bar * transpose(Ht) * inv(Ht * Sig_bar* transpose(Ht) + Q);

%calculate the difference

diff = -(ht-meas);


if (isempty(Kt)||isempty(diff)|| sum(diff)<(10^(-2)) || sum(isnan(diff))>0 || sum(sum(isnan(Kt))>0 ))% || any(abs(diff)>1))
    mu=mu_bar;
    Sig = Sig_bar;
else
    mu = mu_bar + Kt * (diff);
    Sig = (eye(3) - Kt * Ht)*Sig_bar;
end


%final step






